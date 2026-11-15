/*
 * own_decoder.h — 自研 Quattro 解码核心（x3fcore-hybrid）
 *
 * 定位：x3fuse 的**可替换解码后端**。只覆盖 dp Quattro 世代（半层 G/R +
 * 全分辨率 B 三平面 TRUE 编码）；其余机型由调用方回落到 x3fuse。
 *
 * 对外契约与 x3fuse 的 x3f_scene_linear 完全同构（见
 * fuj/outputs/X3FCore/Sources/CX3FRaw/x3f_render.c 的 load_common_inner）：
 *   data   = f32 RGB 交错，场景线性 extended sRGB/D65，
 *            ActiveImageArea 已裁剪、**不做** EXIF 方向旋转，
 *            负值与 >1 保留；
 *   camera_to_rgb[9] = iso_scale · XYZ_to_sRGB · bmt_to_xyz · diag(wb_gain)，
 *            列再乘 DigitalISOGain —— 与 x3fuse get_conv_controlled 同一公式链
 *            （x3f_process.c: x3f_get_gain / x3f_get_bmt_to_xyz /
 *            x3f_get_raw_to_xyz）。
 *
 * 阶段一（本文件）有意不做：
 *   - BMT 域高光重建（x3fuse highlight_recovery；我们走渲染层 v2 引擎）；
 *   - NLM 传感器降噪（渲染层自己做色度降噪）。
 * spatial gain（SPP 口径）已移植但默认关闭：x3fuse 的 scene-linear 路径
 * 不施加 sgain（只在 u16/DNG 路径用），接管 DNG 导出时 X3FDEC_SGAIN=1 打开。
 */

#ifndef OWN_DECODER_H
#define OWN_DECODER_H

#include <stddef.h>
#include <stdint.h>

/* 探测：这个文件是不是"我们能解的 Quattro"？——FOVb 容器 + SECi TRUE RAW
 * + 三层布局（半层 G/R + 全分辨率 B），或 CAMMODEL 含 "Quattro"。
 * 返回 1 = 可以交给 own_scene_linear；0 = 交给 x3fuse。 */
int own_probe(const char *path);

/* 场景线性解码。成功返回 0 并填充所有出参；
 * 返回 1 = 不支持（非 Quattro / 缺关键元数据），调用方应回落 x3fuse；
 * 返回 -1 = 探测说能解但解坏了（格式细节超出覆盖），同样建议回落。
 * wb_preset: NULL / "" = as-shot；否则用指定白平衡预设名重定白平衡。 */
int own_scene_linear(const char *path,
                     float **data, unsigned *w, unsigned *h,
                     unsigned short *orientation,
                     double *camera_to_rgb /* [9] 出参 */,
                     const char *wb_preset,
                     int denoise_intensity /* 0..10，驱动 XFCHAIN 的 σ */,
                     const void *denoise /* x3f_denoise_t*，可为 NULL */);

void own_scene_linear_free(float *data);

/* SFD 逐帧解码的降噪开关（线程局部，缺省 1 = 全开）。
 * 0 = 跳过解码内降噪（XFCHAIN 两趟 NLM / 回退双边 / 色度中值），上采样
 * 不动 —— SFD 合成在 x3f_sfd.c 里对非基准帧设 0，降噪由合成后的 Stage1
 * 一次收掉（与 fuse 后端 nlm=0 的行为对齐）。 */
void own_set_frame_nr(int on);

/* -------- 逐通道裁剪标记（过曝矫正 mode7 的输入） --------
 * 在下一次解码前调用 own_request_clip_mask()，解码就会在 fuse_quattro 里
 * 按**修复前的 raw**（半层 R/G + 全层 B 各自对比本通道饱和值）记下每像素
 * 哪些通道真裁剪了 —— 这是"全像素亮度阈值中性化"做不到的事：它区分
 * 「三通道全裁的真白」和「只裁 R/G 的黄色高光」。取走（own_take_clip_mask）
 * 后所有权归调用方，用完 free；不取则在下一次解码时被释放。
 * 非 Quattro / 走 x3fuse 回落时取到 NULL。 */
void      own_request_clip_mask(void);
uint8_t  *own_take_clip_mask(int *w, int *h);

/* 解码产物（调试 / 实验室用；scene_linear 内部自己也走这一套） */
typedef struct {
    float  *bmt;             /* w*h*3，黑电平已扣、ActiveImageArea 已裁剪，序 R,G,B */
    int     w, h;
    int     orientation;     /* EXIF 1..8 */
    double  black[3];
    double  maxraw[3];
    double  wbgain[3];       /* as-shot 增益（含 GainFact 修正） */
    double  cc[9];           /* as-shot CC 矩阵（WhiteBalanceColorCorrections） */
    double  iso_scale;       /* CaptureISO / SensorISO */
    double  dig_gain[3];     /* DigitalISOGain，缺省 [1,1,1] */
    char    camera[64];
    char    wb_name[32];     /* as-shot 预设名，如 "Auto" */
    uint8_t *clipmask;       /* w*h，每像素 3 位：bit0=R bit1=G bit2=B 顶到
                              * 传感器饱和（fuse_quattro 内按修复前 raw 判）。
                              * 仅 own_request_clip_mask() 请求过才非 NULL；
                              * own_decoded_free / own_scene_linear 负责 free。 */
} OwnDecoded;

int  own_decode_file(const char *path, OwnDecoded *dec);
void own_decoded_free(OwnDecoded *dec);

/* 8-bit sRGB PNG 预览（线性输入 → gamma 编码），实验室对拍用 */
int own_write_preview_png(const char *path, int w, int h, const float *linear);

/* 调试：解码 CAMF 并列出全部条目（实验室对拍用） */
void own_camf_dump(const char *path);

/* -------- 分发钩子（接入 fuj 用） --------
 * 与 x3f_scene_linear 完全同签名（denoise 用 void* 携带 fuj 的
 * x3f_denoise_t*，指针 ABI 等价）。fuj 侧把 x3f_render.c 的
 * x3f_scene_linear 调用与全部 x3f_scene_linear_free 换成下面这对即可，
 * 渲染层 / SFD / Swift 层零改动。后端选择见 own_decoder.c 注释。 */
int  x3f_own_dispatch_scene_linear(const char *path, float **data,
                                   unsigned *w, unsigned *h,
                                   unsigned short *orientation,
                                   double *camera_to_rgb /* [9] */,
                                   int highlight_recovery,
                                   unsigned char denoise_intensity,
                                   const void *denoise,
                                   const char *wb_preset);
void x3f_own_dispatch_scene_linear_free(float *data);

#endif /* OWN_DECODER_H */
