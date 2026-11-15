/* Merrill SPP colour export — host integration entry point (local addition).
 *
 * Runs the SPP float front end plus the SPP noise-reduction chain and
 * encodes the result as 16-bit sRGB, i.e. the same output the
 * `-spp-pipeline` CLI switch produces (SPP with its noise sliders at the
 * neutral position).
 *
 * The caller opens the file and passes the FILE *; this entry point always
 * uses the x3fuse reader, so the host's legacy/x3fuse decoder split does not
 * leak in here. The caller keeps ownership of the FILE *.
 *
 * x3f_spp_color_export_alloc hands back a buffer the caller owns and must
 * release with x3f_spp_color_export_free.
 *
 * Return codes:
 *    0  success
 *   -1  bad arguments
 *   -2  the file is not a readable X3F
 *   -3  outside the SPP export envelope (not Merrill, not the HI
 *       resolution, Cineon, …)
 *   -4  the SPP pipeline failed
 *   -5  out of memory
 */
#ifndef X3F_SPP_H
#define X3F_SPP_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Gate check only — loads the header/CAMF and applies the SPP envelope
 * test without rendering. Returns 0 when the file can take the SPP path;
 * on success the export geometry is written to out_w / out_h (either may
 * be NULL). */
extern int x3f_spp_export_check(FILE *file, uint32_t *out_w, uint32_t *out_h);

/* Creative overrides for the SPP tone and denoise stage.
 *
 * Every double uses NaN as "keep the native value": the CAMF contrast
 * calibration, zero for shadows and highlights, and a neutral saturation
 * gain of 1.0. `nr_level` selects one of the five body-calibrated denoise
 * rows (0..4); -1 keeps the native middle row.
 *
 * The layout matches the Rust `SppTone` struct (repr(C)) byte for byte.
 *
 * --- ABI 护栏（2026-10-28）-------------------------------------------
 * 这个结构体是**按指针**跨 FFI 传的，所以它和 Rust 的 SppTone 必须逐字节
 * 一致。布局一旦对不上，后果不是"报错"而是**静默出错**：
 *   - Rust 按新布局去读 C 侧结构体尾部的内存 → 越界读；
 *   - 读到的是栈上的垃圾值而**不是 NaN** → "保持原生值"的哨兵失效 →
 *     真的套用一个随机 creative override（而且是每次调用都不同）。
 * 所以这里加三道护栏：
 *   1) 下面的静态断言抓**本文件自身**的漂移（字段被改顺序/改类型）；
 *   2) Rust 侧的 const assert 抓 Rust 侧的漂移；
 *   3) x3f_spp_tone_abi_size() 抓**跨文件**漂移（Rust 改了、C 头没改）——
 *      运行时比对 sizeof；而且 Rust 库没重编时这个符号根本不存在 →
 *      **链接期就炸**，不会带着错位的结构体被打包进 app。
 * ----------------------------------------------------------------------
 */
typedef struct x3f_spp_tone {
    double exposure;   /* tone exposure in EV */
    double contrast;   /* added to the CAMF contrast calibration */
    double shadows;    /* positive lifts the shadows */
    double highlights; /* positive lifts the highlights */
    double saturation; /* SPP slider units (-2..+2), 0.0 is neutral */
    int32_t nr_level;  /* denoise row 0..4, -1 = native middle row */
    int32_t reserved;  /* explicit padding */
    /* ↓ 与 Rust SppTone 的同名字段逐字节对应（2026-10-28 同步） */
    double sharpness;         /* SPP Sharpness 滑杆（±2）；NaN = 用原生值 */
    double fill_light;        /* SPP Fill Light 滑杆（−2..+2）；NaN = 用原生值(0) */
    double highlight_control; /* fHighB = 1 − 滑杆；NaN = 用原生值(1.0=恒等) */
    double coladj[3];         /* ColAdj R/G/B 增益（1.0=恒等）；NaN = 用原生值 */
} x3f_spp_tone;

#if defined(__cplusplus)
#define X3F_SPP_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define X3F_SPP_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

X3F_SPP_STATIC_ASSERT(sizeof(x3f_spp_tone) == 96,
                       "x3f_spp_tone must match Rust SppTone (repr(C)) byte for byte");
X3F_SPP_STATIC_ASSERT(offsetof(x3f_spp_tone, sharpness) == 48,
                       "x3f_spp_tone field offsets must match Rust SppTone");
X3F_SPP_STATIC_ASSERT(offsetof(x3f_spp_tone, coladj) == 72,
                       "x3f_spp_tone field offsets must match Rust SppTone");

/* Rust 侧导出的 sizeof(SppTone)。C 侧用它做跨文件 ABI 比对；
 * 若 libx3f.a 是旧的（没有这个符号），链接期就失败，不会静默错位。 */
extern uint32_t x3f_spp_tone_abi_size(void);

/* `tone` may be NULL, which means "no overrides at all" and reproduces the
 * native export byte for byte. */
extern int x3f_spp_color_export_alloc(FILE *file,
                                       uint16_t **out,
                                       uint32_t *out_w,
                                       uint32_t *out_h,
                                       const x3f_spp_tone *tone);

extern void x3f_spp_color_export_free(uint16_t *ptr);

/* ------------------------------------------------------------------ *
 * Two-stage export: prepare once, apply the tone curve many times.
 * ------------------------------------------------------------------ *
 *
 * The one-shot export above pays the whole SPP chain every time. The
 * two-stage split does the expensive half once (decode + float front end +
 * noise reduction, ~3.3 s on a 24 MP body) and keeps the float buffer
 * alive, so that the tone stage (tone curve + output transform, ~0.03 s)
 * can be re-run for every slider change.
 *
 * The denoise row is baked into the prepared buffer, because the noise
 * reduction itself depends on it and not just the tone curve: x3f_spp_prepare
 * reads tone->nr_level, and x3f_spp_apply_tone refuses a different row
 * with -5.
 */

/* Opaque prepared-buffer handle. */
typedef struct x3f_spp_prepared x3f_spp_prepared;

/* Stage 1: decode + noise reduction.
 *
 * Only tone->nr_level is read; every other tone field takes effect in
 * stage 2. The caller owns the returned handle and must release it with
 * x3f_spp_prepared_free. out_w / out_h receive the output geometry and
 * may be NULL.
 *
 * Return codes:
 *    0  success
 *   -1  bad arguments
 *   -2  the file is not a readable X3F
 *   -3  outside the SPP export envelope
 *   -4  the SPP pipeline failed
 */
extern int x3f_spp_prepare(FILE *file,
                            const x3f_spp_tone *tone,
                            x3f_spp_prepared **out,
                            uint32_t *out_w,
                            uint32_t *out_h);

/* Stage 2: apply the tone curve to a prepared buffer.
 *
 * On success the caller owns *out (a w * h * 3 sample block, rows
 * contiguous) and must release it with x3f_spp_color_export_free.
 *
 * Return codes:
 *    0  success
 *   -1  bad arguments
 *   -4  the tone stage failed
 *   -5  the denoise row does not match the prepared buffer; prepare again
 *   -6  out of memory
 */
extern int x3f_spp_apply_tone(x3f_spp_prepared *handle,
                               const x3f_spp_tone *tone,
                               uint16_t **out,
                               uint32_t *out_w,
                               uint32_t *out_h);

extern void x3f_spp_prepared_free(x3f_spp_prepared *handle);

#ifdef __cplusplus
}
#endif

#endif /* X3F_SPP_H */
