// x3f_dodrx.c — SPP Stage3（DoDRX）纯 C 忠实移植，语义见 include/x3f_dodrx.h
//
//移植自 起的 ProcessToOutputRowTemplate
//<FPixmapTemplate<FPixelClass48RGB>>::DoDRX（2026-10-22 转写）。
// 数值行为对齐要点：
//   - 饱和度权与 weight 用 float32（真身用 fcmp/fmadd s 寄存器）
//   - 窗口/增益/系数多项式用 double（d 寄存器）
//   - fcvtzs = 向零截断；定点舍入 = trunc(x*4096 + 0.5)；>>12 为算术移位
//   - 窗口上限 31744.0、窗口有效域 63488.0、增益饱和 ±2130706432 均为真身立即数

#include "x3f_dodrx.h"
#include <math.h>
#include <stdlib.h>

// 逐级饱和（窗口级：<0→0，>63488→65535，否则截断；前面已 clamp 到 31744
// 的值永远不会触发 65535 分支——真身共享模板的死分支，这里同样保留）
static inline int sat_window(double v)
{
    if (v < 0.0) return 0;
    if (v > 63488.0) return 0xffff;
    return (int)v;                                   // fcvtzs：向零截断
}

// 增益级：±2130706432 饱和后 clamp [0,65535]
static inline uint16_t sat_gain(double v)
{
    if (v < -2130706432.0) v = -2130706432.0;
    else if (v > 2130706432.0) v = 2130706432.0;
    int i = (int)v;                                  // fcvtzs
    if (i < 0) return 0;
    if (i > 0xffff) return 0xffff;
    return (uint16_t)i;
}

static inline uint16_t lut_lookup(const x3f_dodrx_fclut *lut, int ch, uint16_t idx)
{
    uint32_t i = (uint32_t)idx;
    if (i > lut->numEntries) i = lut->numEntries;    // min(index, numEntries)
    return lut->ch[ch][i];
}

static const double k_ident[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

// smoothstep 的真身原式：t(1-t)² + (1-t)(1-t²) = 1 - (3t²-2t³)
static inline float weight_for_sat(float sat)
{
    if (sat <= 0.85f) return 1.0f;
    float t = (sat - 0.85f) / 0.15f;
    float u = 1.0f - t;
    float w = t * (u * u) + u * (1.0f - t * t);      // = 1 - smoothstep(t)
    return w * 0.15f + 0.85f;
}

void x3f_dodrx_row(const uint16_t *src, int n,
                   const x3f_dodrx_fclut *lut1, const x3f_dodrx_fclut *lut2,
                   const double *matrix1, const double *matrix2,
                   const double *gains, double win_lo, double win_hi,
                   const x3f_dodrx_ushortlut *out_lut,
                   int src_x0, uint16_t *dst)
{
    (void)src_x0;   // 调用方已按行首偏移；真身用它做 src/mono 的列偏移（bool2）

    const double *m1 = matrix1 ? matrix1 : k_ident;
    const double *m2 = matrix2 ? matrix2 : k_ident;
    double g0 = gains ? gains[0] : 1.0;
    double g1 = gains ? gains[1] : 1.0;
    double g2 = gains ? gains[2] : 1.0;

    // 窗口增益：0<=lo<=63488 && 0<=hi<=63488 && hi>lo → 16384/(hi-lo)，否则 1
    double win_gain = 1.0;
    if (win_lo >= 0.0 && win_lo <= 63488.0 &&
        win_hi >= 0.0 && win_hi <= 63488.0 && (win_hi - win_lo) > 0.0)
        win_gain = 16384.0 / (win_hi - win_lo);

    for (int i = 0; i < n; i++) {
        // 1) 窗口 + clamp [0,31744]（真身两步合并为 saturate）
        double vr = ((double)src[i * 3 + 0] - win_lo) * win_gain;
        double vg = ((double)src[i * 3 + 1] - win_lo) * win_gain;
        double vb = ((double)src[i * 3 + 2] - win_lo) * win_gain;
        vr = vr < 0.0 ? 0.0 : (vr > 31744.0 ? 31744.0 : vr);
        vg = vg < 0.0 ? 0.0 : (vg > 31744.0 ? 31744.0 : vg);
        vb = vb < 0.0 ? 0.0 : (vb > 31744.0 ? 31744.0 : vb);

        // 2) FColorFloat 通道增益
        uint16_t rv = sat_gain(g0 * (double)sat_window(vr));
        uint16_t gv = sat_gain(g1 * (double)sat_window(vg));
        uint16_t bv = sat_gain(g2 * (double)sat_window(vb));

        // 3) 两级 LUT（ch[0]→R ch[1]→G ch[2]→B；真身 [sp+0xc0]=ch[1] 给 G、
        //    [sp+0xc8]=ch[0] 给 R、[sp+0xb8]=ch[2] 给 B）
        uint16_t r1 = lut_lookup(lut1, 0, rv);
        uint16_t g1v = lut_lookup(lut1, 1, gv);
        uint16_t b1 = lut_lookup(lut1, 2, bv);
        uint16_t r2 = lut_lookup(lut2, 0, r1);
        uint16_t g2v = lut_lookup(lut2, 1, g1v);
        uint16_t b2 = lut_lookup(lut2, 2, b1);

        // 4) 饱和度权（float32）
        const float k_norm = 32767.0f;
        float rn = (float)r2 / k_norm;
        float gn = (float)g2v / k_norm;
        float bn = (float)b2 / k_norm;
        float cb = -0.169f * rn - 0.331f * gn + 0.5f * bn;
        float cr = 0.5f * rn - 0.419f * gn - 0.081f * bn;
        cr *= 2.125f;   // Cr 加权
        float sat = sqrtf(cb * cb + cr * cr) / 0.70710678f;
        if (sat > 1.0f) sat = 1.0f;
        float w = weight_for_sat(sat);

        // 5) 系数矩阵（double）：P = M2·k，C = P·M1ᵀ
        //    Rec.601 自洽：g_c = w·(1-wc) + wc，h_c = (1-w)·wc
        //    （wc = 0.299/0.587/0.114；w=1 时 k=单位阵）
        double wd = (double)w;
        double gr = wd * 0.701 + 0.299, hr = (1.0 - wd) * 0.299;
        double gg = wd * 0.413 + 0.587, hg = (1.0 - wd) * 0.587;
        double gb = wd * 0.886 + 0.114, hb = (1.0 - wd) * 0.114;

        double p[9];
        for (int r = 0; r < 3; r++) {
            p[r * 3 + 0] = m2[r * 3 + 0] * gr + m2[r * 3 + 1] * hr + m2[r * 3 + 2] * hr;
            p[r * 3 + 1] = m2[r * 3 + 0] * hg + m2[r * 3 + 1] * gg + m2[r * 3 + 2] * hg;
            p[r * 3 + 2] = m2[r * 3 + 0] * hb + m2[r * 3 + 1] * hb + m2[r * 3 + 2] * gb;
        }
        double c[9];
        for (int r = 0; r < 3; r++)
            for (int cc = 0; cc < 3; cc++)
                c[r * 3 + cc] = p[r * 3 + 0] * m1[cc * 3 + 0] +
                                p[r * 3 + 1] * m1[cc * 3 + 1] +
                                p[r * 3 + 2] * m1[cc * 3 + 2];

        // 6) 12.4 定点应用 + 饱和 [0,65535]
        int32_t fx[9];
        for (int k = 0; k < 9; k++)
            fx[k] = (int32_t)(c[k] * 4096.0 + 0.5);      // trunc(x+0.5) = fcvtzs
        int32_t or_ = (fx[0] * r2 + fx[1] * g2v + fx[2] * b2 + 0x800) >> 12;
        int32_t og_ = (fx[3] * r2 + fx[4] * g2v + fx[5] * b2 + 0x800) >> 12;
        int32_t ob_ = (fx[6] * r2 + fx[7] * g2v + fx[8] * b2 + 0x800) >> 12;
        if (or_ < 0) or_ = 0; else if (or_ > 0xffff) or_ = 0xffff;
        if (og_ < 0) og_ = 0; else if (og_ > 0xffff) og_ = 0xffff;
        if (ob_ < 0) ob_ = 0; else if (ob_ > 0xffff) ob_ = 0xffff;

        // 7) 输出级查表：out = data[out & mask]（真身 DoDRX+2712..2740）
        if (out_lut && out_lut->data) {
            or_ = out_lut->data[or_ & out_lut->mask];
            og_ = out_lut->data[og_ & out_lut->mask];
            ob_ = out_lut->data[ob_ & out_lut->mask];
        }

        dst[i * 3 + 0] = (uint16_t)or_;
        dst[i * 3 + 1] = (uint16_t)og_;
        dst[i * 3 + 2] = (uint16_t)ob_;
    }
}
