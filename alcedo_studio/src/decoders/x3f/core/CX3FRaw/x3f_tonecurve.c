/* x3f_tonecurve.c —— 「色调曲线」四段调节（阴影/暗部/亮度/高光）。
 *
 * 模型（2026-10-26 与用户定案）：显示域 0..255 按三个标尺 r2/r3/r4
 * （默认 64/128/192 = 四等份）分成四段，每段滑杆 val ∈ ±2 让自己那段
 * 隆起/下凹：val>0 提亮该段、val<0 压暗该段。
 *
 * ★曲线构造（2026-10-26 用户两轮反馈定案："分界处不能突然陡峭" +
 * "允许多段调整时曲线连成一条缓坡，斜率共同计算"）：9 个控制点用
 * **Fritsch–Carlson 单调三次 Hermite 样条**穿过，节点位移**共同计算**：
 *   · 段中点位移 A = val·0.22·段宽，并按邻段跨度联合钳制
 *     （|A| ≤ 0.42·min(本段/左右段跨度)，保证控制点严格递增）；
 *   · 段界节点位移 = 相邻两段位移的**均值** —— 分界不再锚死恒等线：
 *     同向调整（阴影+2 & 暗部+2）位移跨过分界保持不变，两段连成一条
 *     大缓坡；反向调整则平滑过渡（位移过零），不产生折角；
 *   · 0/65535 黑白点固定为恒等（位移归零）；
 *   · 割线全部为正（邻段联合钳制保证控制点严格递增）+ FC 限幅器
 *     （α²+β²≤9）→ 数学上保证整条 LUT 单调、斜率处处有界；
 *     切线由相邻割线共同决定 = "斜率共同计算"，无逐段独立突变。
 *   · 标尺退化（相邻相等）时节点去重，保证 px 严格递增。
 *
 * 注：之前复刻的 SPP GenerateToneCurveEditorLut 真身数学已按用户
 * 要求撤换 —— 用户要的是"四段各管各的、分界可拖、过渡平滑"的直观模型。
 *
 * 渲染语义：LUT 是**显示域**映射（in/out 都是 sRGB 显示 u16），在
 * build_tone_lut 产物（基础曲线/高光恢复/对比度/色彩模式曲线之后）的显示值
 * 上施加。四滑杆全 0 → 恒等（返回 0，调用方整段跳过 = 零回归）。
 */
#include "x3f_tonecurve.h"
#include <math.h>

#define TC_NNODE 9
#define TC_AMP   0.22   /* 段中点最大位移 = |val|·TC_AMP·段宽（val=±2 → 0.44 段宽） */

int x3f_tone_curve_build(double sh, double dk, double li, double hi,
                         double r2, double r3, double r4,
                         uint16_t out[X3F_TONECURVE_LUT_SIZE])
{
    double val[4] = { sh, dk, li, hi };
    double b[5];
    double px[TC_NNODE], py[TC_NNODE], tg[TC_NNODE], dl[TC_NNODE - 1];
    int i, k, n;

    if (sh == 0.0 && dk == 0.0 && li == 0.0 && hi == 0.0) {
        for (i = 0; i < X3F_TONECURVE_LUT_SIZE; i++) out[i] = (uint16_t)i;
        return 0;                               /* 恒等 */
    }

    /* 标尺换算到 0..65535 并保序（UI 已约束范围，这里兜底）。
     * 0 与 65535 是固定端点，永远不动。 */
    b[0] = 0.0;
    b[1] = r2 * 65535.0 / 255.0;
    b[2] = r3 * 65535.0 / 255.0;
    b[3] = r4 * 65535.0 / 255.0;
    b[4] = 65535.0;
    for (k = 1; k <= 3; k++) {
        if (!(b[k] > 0.0)) b[k] = 0.0;
        if (b[k] > 65535.0) b[k] = 65535.0;
    }
    for (k = 1; k <= 3; k++)
        if (b[k] < b[k - 1]) b[k] = b[k - 1];

    /* 段位移 A_k = val·AMP·段宽，再按邻段跨度联合钳制：
     * |A_k| ≤ 0.42·min(本段, 左右段跨度)。这是控制点严格递增的保证 ——
     * 否则宽段的隆起会越过窄邻段的中点（数据下降，FC 对非单调数据
     * 不保证单调；r2=40/r3=110 时实测复现过）。 */
    {
        double A[4];
        for (k = 0; k < 4; k++) A[k] = val[k] * TC_AMP * (b[k + 1] - b[k]);
        for (k = 0; k < 4; k++) {
            double m = b[k + 1] - b[k];
            double cap;
            if (k > 0 && b[k] - b[k - 1] < m) m = b[k] - b[k - 1];
            if (k < 3 && b[k + 2] - b[k + 1] < m) m = b[k + 2] - b[k + 1];
            cap = 0.42 * m;
            if (A[k] > cap) A[k] = cap;
            if (A[k] < -cap) A[k] = -cap;
        }

        /* 控制点：0 | m1 b1 | m2 b2 | m3 b3 | m4 | 65535。
         * 段中点位移 = A_k；段界位移 = 相邻两段位移的均值
         * （同向调整 → 连成一条缓坡；反向 → 平滑过零）。黑白点固定恒等。
         * 标尺退化（相邻相等）时节点去重，保证 px 严格递增。 */
        n = 0;
        px[n] = 0.0; py[n] = 0.0; n++;
        for (k = 1; k <= 4; k++) {
            double lo = b[k - 1], hiy = b[k];
            double mid = (lo + hiy) / 2.0;
            if (mid > px[n - 1]) { px[n] = mid; py[n] = mid + A[k - 1]; n++; }
            if (k <= 3 && hiy > px[n - 1]) {
                px[n] = hiy; py[n] = hiy + (A[k - 1] + A[k]) / 2.0; n++;
            }
        }
    }
    if (65535.0 > px[n - 1]) { px[n] = 65535.0; py[n] = 65535.0; n++; }

    /* Fritsch–Carlson 单调三次 Hermite：
     * 割线 → 内点切线（异号取 0，同号取均值）→ 逐段限幅（α²+β²≤9）。 */
    for (k = 0; k < n - 1; k++)
        dl[k] = (py[k + 1] - py[k]) / (px[k + 1] - px[k]);
    tg[0] = dl[0];
    tg[n - 1] = dl[n - 2];
    for (k = 1; k < n - 1; k++)
        tg[k] = (dl[k - 1] * dl[k] > 0.0) ? (dl[k - 1] + dl[k]) / 2.0 : 0.0;
    for (k = 0; k < n - 1; k++) {
        if (dl[k] == 0.0) { tg[k] = 0.0; tg[k + 1] = 0.0; continue; }
        {
            double a = tg[k] / dl[k], bb = tg[k + 1] / dl[k];
            double s2 = a * a + bb * bb;
            if (s2 > 9.0) {
                double tau = 3.0 / sqrt(s2);
                tg[k] = tau * a * dl[k];
                tg[k + 1] = tau * bb * dl[k];
            }
        }
    }

    /* 逐点求值成 65536 项 LUT */
    k = 0;
    for (i = 0; i < X3F_TONECURVE_LUT_SIZE; i++) {
        double x = (double)i, t, y;
        while (k < n - 2 && px[k + 1] < x) k++;
        if (x <= px[k]) {
            y = py[k];
        } else if (x >= px[k + 1]) {
            y = py[k + 1];
        } else {
            double h = px[k + 1] - px[k];
            double t2, t3;
            t = (x - px[k]) / h;
            t2 = t * t; t3 = t2 * t;
            y = (2.0 * t3 - 3.0 * t2 + 1.0) * py[k]
              + (t3 - 2.0 * t2 + t) * h * tg[k]
              + (-2.0 * t3 + 3.0 * t2) * py[k + 1]
              + (t3 - t2) * h * tg[k + 1];
        }
        if (y < 0.0) y = 0.0;
        if (y > 65535.0) y = 65535.0;
        out[i] = (uint16_t)(y + 0.5);
    }
    return 1;
}
