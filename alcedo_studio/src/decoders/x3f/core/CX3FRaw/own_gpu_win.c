/* own_gpu_win.c — own_gpu（Mac 是 Metal 加速层）的 Windows 实现
 *
 * nlm3（own_decoder.c fuse 两次 NLM 的公共出口 nlm3_den，占解码大头）
 * 走 x3fuse 里新增的 own_nlm3 —— wgpu（Vulkan/DX12）整幅派发，int64 定点
 * 语义与 own 的 CPU 路径逐位一致（X3FDEC_NLMDBG=1 可复核）。
 * 2026-10-24：guided_up_2x / bilateral5 / chroma_nr 也 GPU 化了 —— 全部
 * f32 运算、循环序与 CPU 相同、查表原样上传（expf/除法 ulp 差进不来），
 * 与 CPU 路径逐位一致（X3FDEC_GPU=0 做 rgb dump A/B 复核）。
 *
 * Rust 侧符号（x3f_own_gpu_probe / x3f_own_gpu_nlm3）只有 X3F_GPU=ON
 * 编出的静态库里才有，CMake 用 X3F_HAVE_RUST_GPU 把关；没有时退回
 * 全占位实现（纯 CPU，与 2026-10 之前的行为一致）。
 */

#include "own_gpu.h"
#include <stdlib.h>

/* 线程局部的 GPU 关闭（SFD 七帧解码线程用，见 own_gpu.h 的说明）。
 * MSVC / clang 都支持 C11 _Thread_local（own_decoder.c 的 g_own_frame_nr
 * 已经这么用）。放在两个分支之外：没编 Rust GPU 后端时 own_gpu_available
 * 恒 0，但符号仍要存在，x3f_sfd.c 的调用点不用 #ifdef。 */
static _Thread_local int g_gpu_thread_off = 0;

void own_gpu_thread_disable(int on)
{
    g_gpu_thread_off = on ? 1 : 0;
}

#ifdef X3F_HAVE_RUST_GPU

extern int x3f_own_gpu_probe(void);
/* 1 = Rust 侧已因 device lost / wgpu panic 熔断本进程的 GPU。 */
extern int x3f_own_gpu_poisoned(void);
extern int x3f_own_gpu_nlm3(const unsigned short *src3, unsigned short *dst3,
                            int w, int h, const int *lut, int lut_n,
                            int twin, int swin, int bin_shift);
extern int x3f_own_gpu_chroma_nr(float *crA, float *cbA, float *crB, float *cbB,
                                 int w, int h, int iters);
extern int x3f_own_gpu_guided_up_2x(const float *src, const float *t2,
                                    const float *guide, float *dst,
                                    int hw, int hh, const float *dtab,
                                    const float *wv, int wv_n);
extern int x3f_own_gpu_bilateral5(float *img, int w, int h,
                                  const float *rlut, int rlut_n,
                                  const float *sw);
extern int x3f_own_gpu_chrmed_plane(const float *pl, float *omed,
                                    int w, int h, int r);
/* 渲染层 apply 链 + stage1 双边（2026-10-27，对齐 Mac own_gpu.m） */
extern int x3f_own_gpu_apply_lut16(const float *lin, const float *clip,
                                   const unsigned short *lut,
                                   const unsigned short *lutg,
                                   const unsigned short *lutb,
                                   const float *gain, const float *cm3,
                                   const float *cmcb, const float *cmcr,
                                   float sat, int use_sat, float hl_desat,
                                   float desat_lo, float desat_hi,
                                   unsigned short *out, int w, int h);
extern int x3f_own_gpu_chrmed_u16(unsigned short *buf, int w, int h,
                                  int stride, int r);
extern int x3f_own_gpu_down8(const unsigned short *tmp, unsigned char *out,
                             int w, int h, int stride, int dither);
extern int x3f_own_gpu_stage1_bilat(const float *src, float *dst, int w, int h,
                                    int r, const double coef[9],
                                    double lin_sat, double level_scale,
                                    double nscale);

int own_gpu_available(void)
{
    const char *e = getenv("X3FDEC_GPU");
    if (g_gpu_thread_off) return 0;   /* 本线程被 SFD 流水线点名绕开 */
    if (e && e[0] == '0') return 0;   /* 与 Mac gpu_enabled() 同一开关 */
    static int cached = -1;           /* 探针要建 wgpu Instance，进程内缓存 */
    if (cached < 0) cached = x3f_own_gpu_probe();
    /* Rust 侧熔断（device lost / wgpu 内部 panic）后立刻改判不可用：否则
     * 缓存里的 1 会让后续每一张照片再进一次 GPU、再 panic 一次。 */
    if (cached && x3f_own_gpu_poisoned()) cached = 0;
    return cached;
}

int own_gpu_nlm3(const uint16_t *src3, uint16_t *dst3, int w, int h,
                 const int32_t *lut, int lut_n, int twin, int swin,
                 int bin_shift)
{
    return x3f_own_gpu_nlm3(src3, dst3, w, h, lut, lut_n, twin, swin,
                            bin_shift);
}

int own_gpu_guided_up_2x(const float *src, const float *t2, const float *guide,
                         float *dst, int hw, int hh, float k, float fd,
                         const float *dtab, const float *wv, int wv_n)
{
    (void)k; (void)fd;   /* 表已由 C 侧 preinit 建好、原样上传 */
    return x3f_own_gpu_guided_up_2x(src, t2, guide, dst, hw, hh, dtab, wv, wv_n);
}

int own_gpu_bilateral5(float *img, int w, int h, float sigma_r,
                       const float *rlut, int rlut_n, const float *sw)
{
    (void)sigma_r;       /* rlut 已按 sigma_r 预计算 */
    return x3f_own_gpu_bilateral5(img, w, h, rlut, rlut_n, sw);
}

int own_gpu_chroma_nr(float *crA, float *cbA, float *crB, float *cbB,
                      int w, int h, int iters)
{
    return x3f_own_gpu_chroma_nr(crA, cbA, crB, cbB, w, h, iters);
}

int own_gpu_chrmed_plane(const float *pl, float *omed, int w, int h, int r)
{
    /* 2026-10-24：Rust 侧 fkey 键域二分中值（own_chrmed_sel.wgsl），与 CPU
     * chrmed_select 值精确一致（第 k 小语义，算法无关；±0 分歧见 wgsl 注释）。
     * apply_chroma_median 的 prep/emit（u16 解包、UV 平面、重建回写）仍在
     * CPU 并行行 worker —— 只把 5.4s 的窗口中值平面搬上 GPU。
     * X3FDEC_GPUCHR=0 单独关（对拍用，不影响解码层 GPU）。 */
    {
        const char *e = getenv("X3FDEC_GPUCHR");
        if (e && e[0] == '0') return -1;
    }
    return x3f_own_gpu_chrmed_plane(pl, omed, w, h, r);
}

/* ── 渲染层 apply 链 + stage1 双边（2026-10-27）──────────────────────────
 * 原 2026-10-23 恒 -1 的四个桩换成真实现（Rust wgpu，对齐 Mac own_gpu.m 的
 * apply_lut / chrmed / down8 / s1bilat）。仍走 own_gpu_available()（X3FDEC_GPU）
 * 与调用点各自的 gate：apply 链默认预览与导出/CLI 都进（X3F_GPU_APPLY=0 强制回 CPU）。
 * 失败返 -1，x3f_render.c / x3f_stage1.c 各自回退 CPU 对应段。 */

int own_gpu_stage1_bilat(const float *src, float *dst, int w, int h, int r,
                         const double coef[9], double lin_sat,
                         double level_scale, double nscale)
{
    return x3f_own_gpu_stage1_bilat(src, dst, w, h, r, coef, lin_sat,
                                    level_scale, nscale);
}

int own_gpu_apply_lut16(const float *lin, const float *clip,
                        const uint16_t *lut, const uint16_t *lutg,
                        const uint16_t *lutb,
                        const float *gain, const float *cm3,
                        const float *cmcb, const float *cmcr,
                        float sat, int use_sat, float hl_desat,
                        float desat_lo, float desat_hi,
                        uint16_t *out, int w, int h)
{
    return x3f_own_gpu_apply_lut16(lin, clip, lut, lutg, lutb, gain, cm3,
                                   cmcb, cmcr, sat, use_sat, hl_desat,
                                   desat_lo, desat_hi, out, w, h);
}

int own_gpu_chrmed_u16(uint16_t *buf, int w, int h, int stride, int r)
{
    /* X3FDEC_GPUCHR=0 与 chrmed_plane 同开关（对拍用）。 */
    {
        const char *e = getenv("X3FDEC_GPUCHR");
        if (e && e[0] == '0') return -1;
    }
    return x3f_own_gpu_chrmed_u16(buf, w, h, stride, r);
}

int own_gpu_down8(const uint16_t *tmp, uint8_t *out, int w, int h,
                  int stride, int dither)
{
    return x3f_own_gpu_down8(tmp, out, w, h, stride, dither);
}

#else  /* X3F_HAVE_RUST_GPU */

int own_gpu_available(void)
{
    return 0;
}

int own_gpu_guided_up_2x(const float *src, const float *t2, const float *guide,
                         float *dst, int hw, int hh, float k, float fd,
                         const float *dtab, const float *wv, int wv_n)
{
    (void)src; (void)t2; (void)guide; (void)dst;
    (void)hw; (void)hh; (void)k; (void)fd; (void)dtab; (void)wv; (void)wv_n;
    return -1;
}

int own_gpu_bilateral5(float *img, int w, int h, float sigma_r,
                       const float *rlut, int rlut_n, const float *sw)
{
    (void)img; (void)w; (void)h; (void)sigma_r; (void)rlut; (void)rlut_n; (void)sw;
    return -1;
}

int own_gpu_chroma_nr(float *crA, float *cbA, float *crB, float *cbB,
                      int w, int h, int iters)
{
    (void)crA; (void)cbA; (void)crB; (void)cbB;
    (void)w; (void)h; (void)iters;
    return -1;
}

int own_gpu_nlm3(const uint16_t *src3, uint16_t *dst3, int w, int h,
                 const int32_t *lut, int lut_n, int twin, int swin,
                 int bin_shift)
{
    (void)src3; (void)dst3; (void)w; (void)h; (void)lut; (void)lut_n;
    (void)twin; (void)swin; (void)bin_shift;
    return -1;
}

int own_gpu_chrmed_plane(const float *pl, float *omed, int w, int h, int r)
{
    (void)pl; (void)omed; (void)w; (void)h; (void)r;
    return -1;
}

#endif /* X3F_HAVE_RUST_GPU */

/* 以下四个仅在没有 Rust GPU 后端时编（有后端时上面是真实现）。 */
#ifndef X3F_HAVE_RUST_GPU

int own_gpu_stage1_bilat(const float *src, float *dst, int w, int h, int r,
                         const double coef[9], double lin_sat,
                         double level_scale, double nscale)
{
    (void)src; (void)dst; (void)w; (void)h; (void)r; (void)coef;
    (void)lin_sat; (void)level_scale; (void)nscale;
    return -1;
}

/* 预览 apply 链（own_gpu.h 2026-10-23 新增）—— Windows 回 CPU。 */
int own_gpu_apply_lut16(const float *lin, const float *clip,
                        const uint16_t *lut, const uint16_t *lutg,
                        const uint16_t *lutb,
                        const float *gain, const float *cm3,
                        const float *cmcb, const float *cmcr,
                        float sat, int use_sat, float hl_desat,
                        float desat_lo, float desat_hi,
                        uint16_t *out, int w, int h)
{
    (void)lin; (void)clip; (void)lut; (void)lutg; (void)lutb;
    (void)gain; (void)cm3; (void)cmcb; (void)cmcr;
    (void)sat; (void)use_sat; (void)hl_desat; (void)desat_lo; (void)desat_hi;
    (void)out; (void)w; (void)h;
    return -1;
}

int own_gpu_chrmed_u16(uint16_t *buf, int w, int h, int stride, int r)
{
    (void)buf; (void)w; (void)h; (void)stride; (void)r;
    return -1;
}

int own_gpu_down8(const uint16_t *tmp, uint8_t *out, int w, int h,
                  int stride, int dither)
{
    (void)tmp; (void)out; (void)w; (void)h; (void)stride; (void)dither;
    return -1;
}

#endif /* !X3F_HAVE_RUST_GPU */

/* ── 行带色度中值（x3f_render.c 2026-11-08 的行带实现用）────────────────
 * Mac 侧在 own_gpu.m 里用 Metal 一次派发一条带；Windows 侧 Rust wgpu 后端
 * 没有对应 entry（x3f_own_gpu_chrmed_plane_rows 不存在），所以这里恒返回
 * -1 —— 调用方 apply_chroma_median_banded 按「GPU 未接」回 CPU 选择，
 * 中值语义与算法无关，结果与整帧版 / Mac 逐像素相同，只是这一段的窗口
 * 选择跑在 CPU 上。整帧版 own_gpu_chrmed_plane 仍照旧走 Rust GPU。
 * 不放进 #ifdef：有没有 Rust GPU 后端都得有这个符号（否则链接期 LNK2019）。 */
int own_gpu_chrmed_plane_rows(const float *pl, float *omed,
                              int w, int bh, int ry0, int ry1, int r)
{
    (void)pl; (void)omed; (void)w; (void)bh; (void)ry0; (void)ry1; (void)r;
    return -1;
}
