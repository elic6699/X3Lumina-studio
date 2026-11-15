/* v6 裁剪孔洞修正：lin 域按裁剪深度给 G/B 增益（EV 无关）。
 * 完整说明见 x3f_hole_fix.h。独立可编译（供单元测试，无需主工程 vendor）。 */
#include "x3f_hole_fix.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

/* 白点（own/Quattro，= WLIN_V3 = hl9_wlin_v3）。
 * mx = max_c(lin_c / wlin_c)，lin 域量 -> EV 无关。 */
static const double HLFIX_WLIN[3] = { 0.99516, 1.11761, 1.80162 };

/* NODES_V6：mx : gG : gB（升序，段间线性插值；首尾外推钳制）。
 * 与 outputs/tools/lab/fit_hole_gain.py NODES_V6 完全一致。 */
#define HLFIX_N 7
static const double HLFIX_MX[HLFIX_N] = { 0.85,    0.96,    1.075,   1.275,   1.60,    2.10,    3.00 };
static const double HLFIX_GG[HLFIX_N] = { 0.0,    -0.1959, -0.1447,  0.2078,  0.5982,  1.3746,  1.3746 };
static const double HLFIX_GB[HLFIX_N] = { 0.0,     0.0829,  0.0246,  0.0791,  0.2057,  0.4329,  0.4329 };

/* 帧级门控：整帧 mx>=HLFIX_GATE_MX 的占比 >= 该值才启用。 */
#define HLFIX_GATE_MX     1.15
#define HLFIX_GATE_AREA   0.05

/* 色相门控：粉斑只发生在冷色/中性高光；暖色（黄墙 R+G 双裁剪）不吃这套
 * G/B 增益（否则 B 增益把黄掺蓝脱饱和、G 增益改色相 → 等值线分层）。
 * 判据 wlin 归一化后 bn/gn：天空粉斑区 ~0.6-1.2，黄墙暖区 ~0.1-0.3。
 * 线性坡道 HLFIX_HUE_T0 -> HLFIX_HUE_T1 映射到权重 0 -> 1。 */
#define HLFIX_HUE_T0      0.30
#define HLFIX_HUE_T1      0.55

/* 负 G 段（mx 0.96-1.275 的粉推）单独用更严的冷色核坡道。2026-10-30 用户
 * 实测 DP2Q2443（麻园站黄墙）：暖色墙越接近裁剪 bn/gn 越高（趋向中性白），
 * 会爬进 0.30-0.55 坡道吃到负 G（-15~-18%）→ R/G 相对升高 → 沿 mx 等值线
 * 出现橙粉弧带；关掉 fix 反而干净。分离度实测（diag_2443_arc.py，两帧
 * gate_area 都 ~10.5-10.7% 同口径）：2443 负G像素 92.4% hue<0.55（保护），
 * 寺庙负G像素 90.0% hue>=0.70（保留）。 */
#define HLFIX_HUEN_T0     0.55
#define HLFIX_HUEN_T1     0.70

/* 与 numpy.interp 等价的线性插值（含首尾外推钳制）。 */
static double hlfix_lerp(const double *xs, const double *ys, int n, double x)
{
    if (x <= xs[0])      return ys[0];
    if (x >= xs[n - 1])  return ys[n - 1];
    for (int i = 1; i < n; i++) {
        if (x <= xs[i]) {
            double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
            return ys[i - 1] + t * (ys[i] - ys[i - 1]);
        }
    }
    return ys[n - 1];
}

/* 单像素 mx = max_c(lin_c / wlin_c)。 */
static inline double hlfix_mx(const float *px, const double *invWL)
{
    double m0 = px[0] * invWL[0];
    double m1 = px[1] * invWL[1];
    double m2 = px[2] * invWL[2];
    double mx = m0;
    if (m1 > mx) mx = m1;
    if (m2 > mx) mx = m2;
    return mx;
}

/* 显式开关：App UI / CLI 通过 x3f_hole_fix_set_enabled() 交给 C（铁律：不用
 * setenv）。-1 = 尚未显式设置，此时按 env 语义决定（X3F_HOLE_FIX=1 -> 开，
 * 未设 -> 默认关）。一旦 set，env 不再参与。
 * ★ 2026-10-30 用户拍板基线回退：默认关闭 = hole-fix 诞生前的解码行为；
 * 修复成为显式 opt-in，以后过曝矫正改动一律以「无 fix」为对照基线。 */
static int g_enabled = -1;

void x3f_hole_fix_set_enabled(int en)
{
    g_enabled = en ? 1 : 0;
}

int x3f_hole_fix_enabled(void)
{
    if (g_enabled >= 0)
        return g_enabled;
    const char *e = getenv("X3F_HOLE_FIX");
    return (e != NULL && *e == '1') ? 1 : 0;
}

double x3f_hole_fix_apply(float *rgb, int w, int h)
{
    if (!x3f_hole_fix_enabled())
        return 0.0;

    size_t npx = (size_t)w * (size_t)h;
    if (npx == 0)
        return 0.0;

    double invWL[3] = { 1.0 / HLFIX_WLIN[0], 1.0 / HLFIX_WLIN[1], 1.0 / HLFIX_WLIN[2] };

    /* 帧级门控：统计整帧 mx>=HLFIX_GATE_MX 的占比 */
    size_t gate_cnt = 0;
    for (size_t p = 0; p < npx; p++) {
        const float *px = rgb + p * 3;
        if (hlfix_mx(px, invWL) >= HLFIX_GATE_MX)
            gate_cnt++;
    }
    double gate_area = (double)gate_cnt / (double)npx;
    if (getenv("X3F_HOLE_FIX_DEBUG"))
        fprintf(stderr, "[hole_fix] called %dx%d gate_area=%.4f\n", w, h, gate_area);
    if (gate_area < HLFIX_GATE_AREA)
        return 0.0;   /* 非大面冷色高光场景，跳过（保护 2501 焰核等） */

    /* 施加：G *= 1+w*gG(mx), B *= 1+w*gB(mx), R 不动。
     * w  = 色相门控权重（bn/gn 坡道 0.30-0.55），管正 G 与 B；
     * wn = 负 G 段专用的冷色核坡道（0.55-0.70），暖墙近白裁剪尾不吃粉推。 */
    const double invWLG = 1.0 / HLFIX_WLIN[1];
    const double invWLB = 1.0 / HLFIX_WLIN[2];
    for (size_t p = 0; p < npx; p++) {
        float *px = rgb + p * 3;
        double mx = hlfix_mx(px, invWL);
        double gG = hlfix_lerp(HLFIX_MX, HLFIX_GG, HLFIX_N, mx);
        double gB = hlfix_lerp(HLFIX_MX, HLFIX_GB, HLFIX_N, mx);
        double gGpos = gG > 0.0 ? gG : 0.0;
        double gGneg = gG < 0.0 ? gG : 0.0;
        if (fabs(gG) < 1e-9 && fabs(gB) < 1e-9)
            continue;
        double gn = px[1] * invWLG;
        if (gn <= 1e-6)
            continue;
        double hue = (px[2] * invWLB) / gn;
        double w = 0.0, wn = 0.0;
        if (hue >= HLFIX_HUE_T1) w = 1.0;
        else if (hue > HLFIX_HUE_T0) w = (hue - HLFIX_HUE_T0) / (HLFIX_HUE_T1 - HLFIX_HUE_T0);
        if (gGneg < 0.0) {
            if (hue >= HLFIX_HUEN_T1) wn = 1.0;
            else if (hue > HLFIX_HUEN_T0) wn = (hue - HLFIX_HUEN_T0) / (HLFIX_HUEN_T1 - HLFIX_HUEN_T0);
        }
        if (w > 0.0 || wn > 0.0) {
            px[1] = (float)(px[1] * (1.0 + w * gGpos + wn * gGneg));
            px[2] = (float)(px[2] * (1.0 + w * gB));
        }
    }
    return gate_area;
}
