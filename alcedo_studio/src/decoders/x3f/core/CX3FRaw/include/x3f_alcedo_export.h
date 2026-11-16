/* x3f_alcedo_export.h — Alcedo 侧场景线性出口（广色域合规）
 *
 * x3f_scene_linear / x3f_own_dispatch_scene_linear 输出的是「扩展线性 sRGB」：
 * f32、通道交错、保留负通道与 >1 高光余量，但相机原生色彩已经被
 * camera_to_rgb 投到 sRGB 原色。直接把它当作相机空间喂给 Alcedo 会造成
 * 二次换域，色域与 ICC 都不自洽。
 *
 * 本出口在**解码核之外**做一次严格可逆的线性变换：
 *     bmt      = inverse(camera_to_rgb) * scene_rgb      (不 clamp)
 *     cam_xyz  = sRGB_to_XYZ * camera_to_rgb             (bmt -> XYZ, D65)
 * 于是恒有 cam_xyz * bmt = sRGB_to_XYZ * scene_rgb = XYZ。
 * 输出保持 f32、负值/越界值原样保留，Alcedo 可从相机原生空间直接换到
 * AP1 参考空间，再导出 P3 / Rec2020 / ProPhoto 等自洽 ICC。
 *
 * 不修改 x3fuse / own_decoder 内部，两条解码链都返回 camera_to_rgb，
 * 因此本变换对二者统一适用。
 */

#ifndef X3F_ALCEDO_EXPORT_H
#define X3F_ALCEDO_EXPORT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    X3F_ALCEDO_OK            = 0,
    X3F_ALCEDO_ERR_ARG       = -1,
    X3F_ALCEDO_ERR_CONTAINER = -2,
    X3F_ALCEDO_ERR_DECODE    = -3,
    X3F_ALCEDO_ERR_SINGULAR  = -4,
    X3F_ALCEDO_ERR_NOMEM     = -5
};

/* 机型类别 */
enum {
    X3F_ALCEDO_CLASSIC  = 0,  /* SD9/SD10/SD14/SD15/DP1 系等 */
    X3F_ALCEDO_MERRILL  = 1,
    X3F_ALCEDO_QUATTRO  = 2
};

/* 解码后端 */
enum {
    X3F_ALCEDO_DECODER_AUTO = 0,  /* Quattro -> own，其余 -> fuse */
    X3F_ALCEDO_DECODER_FUSE = 1,
    X3F_ALCEDO_DECODER_OWN  = 2,
    X3F_ALCEDO_DECODER_SPP  = 3   /* Merrill 原链，旁路成品，不走本出口 */
};

/* 可用后端位掩码 */
enum {
    X3F_ALCEDO_HAS_FUSE = 1 << 0,
    X3F_ALCEDO_HAS_OWN  = 1 << 1,
    X3F_ALCEDO_HAS_SPP  = 1 << 2
};

typedef struct {
    float  *rgb;              /* width*height*3，RGB 交错，相机原生线性，未 clamp */
    int     width;
    int     height;
    int     orientation;      /* EXIF 1..8，由渲染方应用一次 */
    double  cam_xyz[9];       /* 相机原生 RGB -> XYZ(D65)，与 rgb 严格自洽 */
    double  camera_to_rgb[9]; /* 诊断：原生 -> 线性 sRGB */
    int     decoder_used;     /* 0 = fuse, 1 = own */
    int     model_class;
} x3f_alcedo_linear_t;

/* 只读文件头判定机型（不解码整帧）。返回 X3F_ALCEDO_CLASSIC/MERRILL/QUATTRO。
 *
 * 注意：这是**像素阵列口径**（X3F_IMAGE_RAW_QUATTRO = 0x23）。SD Quattro /
 * SD Quattro H 在 X3F 里是独立的 type_format（SDQ=0x25 / SDQH=0x27），不算
 * QUATTRO。解码后端选择与绿增强用它（历史口径，保持不动）。 */
int x3f_alcedo_model_class(const char *path);

/* **门控口径**的世代：SDQ / SDQH / SFD 容器与 DP-Q 一起返回 QUATTRO。
 *
 * 过曝矫正（oe mode8）、hole-fix、SDQ 高光这三项在 X3Lumina 里按「Quattro
 * 世代」而不是「Quattro 阵列」门控；用 model_class 判断会把 SDQ/SDQH 误置灰
 * （2026-11-16 用户实测报障）。面板置灰与 oe 解析一律用本函数。 */
int x3f_alcedo_model_gen(const char *path);

/* 本文件是否 SD Quattro / SD Quattro H 机身（RAW type_format SDQ / SDQH）——
 * 「SDQ 高光」滑杆的**门控**口径：X3Lumina 在两种 SDQ 机身上都摆滑杆。 */
int x3f_alcedo_sdq_hl_capable(const char *path);

/* 该文件的 DigitalISOGain 是否偏离 (1,1,1)——归一在该机身上是否**真的**
 * 改变像素（SDQH 4,4,2 = 有效；SDQ 1,1,1 = 滑杆可拖但乘 1.0 无变化）。
 * 诊断用，不要拿它做面板门控。 */
int x3f_alcedo_sdq_hl_effective(const char *path);

/* 该机型可用的解码后端位掩码，供 UI 置灰与参数校验。 */
int x3f_alcedo_available_decoders(const char *path);

/* 导出相机原生 scene-linear f32。
 *   frame    < 0 用默认帧（0 EV）；普通 X3F 忽略。
 *   decoder  X3F_ALCEDO_DECODER_*；SPP 在此返回 ERR_ARG（应走旁路）。
 * 成功时 out->rgb 由 x3f_alcedo_export_free() 释放。
 */
int  x3f_alcedo_export_linear(const char *path, int frame, int decoder,
                              x3f_alcedo_linear_t *out);

/* ---------------------------------------------------------------------------
 * Decode-level options (camera-native BMT domain).
 *
 * These all act BEFORE the scene-linear export and never enter the display
 * encoding chain: no tone curve, no colour-mode matrix, no gamma and no clamp.
 * They are therefore wide-gamut safe.
 *
 * This is path A (the default wide-gamut editing path). The X3Lumina look
 * sliders, mode8 and the SPP native chain are reproduced by path B in
 * x3f_alcedo_foveon.h: that is an explicitly selected, display-referred CPU
 * sidecar (16-bit sRGB finished image) and never feeds path A or the Develop
 * DAG. Keeping the two paths separate is what preserves A's code-value
 * gamut (negative channels and >1 headroom) and its self-consistent ICC.
 *
 * "denoise" here is the decode-stage NLM (x3f_denoise_t); highlighting
 * recovery is the decode-stage BMT highlight reconstruction. Both run once at
 * decode time (cached), not per preview frame.
 * ------------------------------------------------------------------------- */
typedef struct {
    int    decoder;              /* X3F_ALCEDO_DECODER_*, AUTO/FUSE/OWN only   */
    int    denoise_intensity;    /* 0..10 NLM strength; 0 = off (default)      */
    double denoise_strength;     /* 0.1..3.0; <=0 keeps the core default 1.0   */
    int    denoise_luminance;    /* 0=off, 1=on, 2=auto by ISO (default 2)     */
    int    denoise_chroma;       /* 0/1, default 1                             */
    int    highlight_recovery;   /* 0/1, decode-stage BMT highlight rebuild    */
} x3f_alcedo_decode_opts_t;

void x3f_alcedo_decode_opts_default(x3f_alcedo_decode_opts_t *opts);

/* Extended entry points. opts == NULL is equivalent to the default options
 * with AUTO decoder, i.e. bit-identical to x3f_alcedo_export_linear(...,0,..).
 * SPP is rejected here; use x3f_alcedo_export_spp16() for that sidecar. */
int x3f_alcedo_export_linear_ex(const char *path, int frame,
                                const x3f_alcedo_decode_opts_t *opts,
                                x3f_alcedo_linear_t *out);
int x3f_alcedo_export_linear_memory_ex(const void *bytes, size_t len,
                                       const x3f_alcedo_decode_opts_t *opts,
                                       x3f_alcedo_linear_t *out);

/* Same contract for an in-memory standalone X3F frame (FOVb). Alcedo's decode
 * cache hands the decoder an opaque byte buffer rather than a path, and the
 * underlying scene-linear readers are path-based. The buffer is staged to a
 * private temporary file for the duration of the call and removed on return.
 * X3I containers must already be materialized to a single frame by the caller.
 * out->rgb is released with x3f_alcedo_export_free(). */
int  x3f_alcedo_export_linear_memory(const void *bytes, size_t len, int decoder,
                                     x3f_alcedo_linear_t *out);
void x3f_alcedo_export_free(x3f_alcedo_linear_t *out);

/* ---------------------------------------------------------------------------
 * Merrill SPP native-chain sidecar.
 *
 * The legacy SPP chain is a f64, tone-mapped, display-referred pipeline. It
 * produces a 16-bit sRGB *finished image*, not scene-linear f32, and must not
 * be fed into the Develop DAG (double gamma/tone). It is offered as a separate
 * "SPP native output" option: CPU-only, computed once on export/straight-out,
 * never part of the wide-gamut DAG. Only Merrill HI-resolution files qualify.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint16_t *rgb;               /* width*height*3, RGB interleaved, sRGB u16  */
    int       width;
    int       height;
    int       orientation;       /* EXIF 1..8, sensor-order image              */
} x3f_alcedo_spp16_t;

/* ---------------------------------------------------------------------------
 * SPP tone-stage controls (Merrill native chain only).
 *
 * These values are applied by SPP itself, in its own camera-code-value tone
 * stage, *before* the sRGB OETF. They are NOT Alcedo Develop-DAG parameters:
 * Alcedo's exposure/contrast/saturation live in ACEScc/AP1 and shadows /
 * highlights are an LLF operator, so reusing those would change the product
 * and is deliberately not done here.
 *
 * Field semantics follow x3f_render_spp_export_tone():
 *   exposure     EV, positive brightens
 *   contrast     added to the CAMF contrast calibration
 *   shadows      positive lifts the shadows
 *   highlights   SPP semantics: positive lifts highlights. The X3Lumina UI
 *                uses the opposite sign for its "高光" slider, so the host
 *                bridge negates it before filling this struct.
 *   saturation   SPP slider units -2..+2, 0 = native
 *   nr_level     0..4 body-calibrated denoise row; -1 = native middle row.
 *                This one is baked into the prepared buffer, so changing it
 *                forces a new prepare().
 * ------------------------------------------------------------------------- */
typedef struct {
    double exposure;
    double contrast;
    double shadows;
    double highlights;
    double saturation;
    int    nr_level;
} x3f_alcedo_spp_tone_t;

/* One-shot SPP tone export. Equivalent to open + apply + close; out->rgb is
 * released with x3f_alcedo_spp16_free(). */
int  x3f_alcedo_spp_tone_export(const char *path, int frame,
                                const x3f_alcedo_spp_tone_t *tone,
                                x3f_alcedo_spp16_t *out);

/* Two-stage SPP tone session: prepare (decode + float front end + NR) is the
 * expensive half and stays cached; apply() re-runs only the tone stage. The
 * prepared buffer is owned by the session; the destination is caller-owned. */
typedef struct x3f_alcedo_spp_session x3f_alcedo_spp_session;

int  x3f_alcedo_spp_session_open(const char *path, int frame, int nr_level,
                                 x3f_alcedo_spp_session **out, int *w, int *h);
/* dst is width*height*3 uint16 samples; stride is in bytes. */
int  x3f_alcedo_spp_session_apply(x3f_alcedo_spp_session *session,
                                  const x3f_alcedo_spp_tone_t *tone,
                                  uint16_t *dst, int stride, int *w, int *h);
int  x3f_alcedo_spp_session_orientation(const x3f_alcedo_spp_session *session);
void x3f_alcedo_spp_session_close(x3f_alcedo_spp_session *session);

/* Cheap capability probe (header + CAMF only, no render). Returns 1 when the
 * SPP sidecar can produce this file and fills w/h; 0 otherwise. Used to grey
 * out the option in the UI. */
int  x3f_alcedo_spp_capable(const char *path, int frame, int *w, int *h);

/* Renders the SPP finished image. Returns X3F_ALCEDO_OK on success.
 * out->rgb is released with x3f_alcedo_spp16_free(). */
int  x3f_alcedo_export_spp16(const char *path, int frame,
                             x3f_alcedo_spp16_t *out);
void x3f_alcedo_spp16_free(x3f_alcedo_spp16_t *out);

/* Decode-free colour matrix. Reads only the header, the trailing directory
 * and the CAMF metadata block (~3 MB); it never touches or decompresses the
 * sensor planes. This is what bulk import uses, so importing a library of
 * X3F/X3I files costs milliseconds per file instead of a full decode.
 *
 * The returned cam_to_xyz is the exact matrix the scene-linear export uses
 * (x3f_alcedo_export_linear), so import and render always agree. frame < 0
 * selects the default (0 EV) frame of an X3I container. */
int x3f_alcedo_probe_color(const char *path, int frame, double cam_to_xyz[9]);

/* Legacy Foveon orientation fallback. Older bodies (SD9/SD10/SD14/SD15,
 * X3F v2.x) write rotation only in the X3F header at offset 36 while the
 * embedded JPEG's EXIF Orientation stays 1. Merrill and later dual-write
 * (Rotation=270 <-> EXIF 8), so this is applied only when EXIF says 1.
 * Returns an EXIF orientation 1..8; FOVi containers and v4+ files return 1. */
int x3f_alcedo_header_orientation(const char *path);

/* 最近一次错误的静态字符串，无需释放。 */
const char *x3f_alcedo_export_error(void);

#ifdef __cplusplus
}
#endif

#endif /* X3F_ALCEDO_EXPORT_H */
