/* own_gpu.h — Metal GPU 加速层（own_decoder.c 热点核）
 *
 * 覆盖三个最大可并行热点（语义与 own_decoder.c CPU 版一致，float 精度）:
 *   guided_up_2x  引导双边 2x 上采样（融合管线最大块）
 *   bilateral5    5x5 双边滤波（两 pass，水平+垂直）
 *   chroma_nr     Cr/Cb 3x3 中值降噪（迭代 ping-pong）
 *
 * 约定:
 *   - 所有函数成功返回 0，失败返回 -1（调用方自行回退 CPU 路径）
 *   - own_gpu_available() 内部检查 X3FDEC_GPU=0（强制关闭）与设备可用性
 *   - Metal 库由内嵌 MSL 源码运行时编译（免打包 .metal 资源）
 *   - 线程安全：可在多线程同时调用（Metal device/queue 本身线程安全）
 */
#ifndef OWN_GPU_H
#define OWN_GPU_H

/* 自包含：本头用 uint16_t/int32_t（x3f_stage1.c 的 __has_include 在
 * <stdint.h> 之前就把我们带进来了，不能指望包含者先备好）。 */
#include <stdint.h>

int own_gpu_available(void);

/* **线程局部**地关掉本线程的 GPU（SFD 七帧流水线用），0 = 恢复。
 *
 * 为什么需要它：GPU 后端（win: wgpu / mac: Metal）在**多个解码线程同时调用**
 * 时是负优化 ——
 *   · Rust 侧 with_ctx 有一把全局 CTX 互斥锁，并发调用排队；
 *   · 每次调用都新建并销毁 ~300MB 缓冲（guided_up 的 src/t2/guide/dst/readback），
 *     上传 ~130MB、回读 86.5MB；
 * 集显（共享系统内存带宽）上单次 guided_up 从独占 0.6~1.3s 涨到 13~33s
 * （实测 DP2Q1514.X3I，7 帧全分辨率）。SFD 的预取流水线正是"N 个解码线程
 * 同时跑"，所以让这些线程自己绕开 GPU，比整进程 X3FDEC_GPU=0 精确：
 * 单帧解码、预览、导出渲染这些单线程场合照旧吃 GPU。
 * 要改回 GPU：X3F_SFD_GPU=1（见 x3f_sfd.c）。 */
void own_gpu_thread_disable(int on);

/* dtab = [2][2][5][5] 距离核表（own_decoder.c guided_up_preinit 产物）。
 * wv = 值域核表 wv_lut（WVLUT_N 项）—— GPU 端 expf/除法与 CPU ulp 不同，
 * 表必须 C 侧算好原样上传。k/fd 只留签名兼容（GPU 端不用）。 */
int own_gpu_guided_up_2x(const float *src, const float *t2, const float *guide,
                         float *dst, int hw, int hh, float k, float fd,
                         const float *dtab, const float *wv, int wv_n);

/* rlut = 值域核 LUT（RLUT_N 项）、sw = 空间核 5 项（CPU expf 产物）。
 * 两 pass 可分离滤波，结果写回 img。 */
int own_gpu_bilateral5(float *img, int w, int h, float sigma_r,
                       const float *rlut, int rlut_n, const float *sw);

/* iters 次迭代后结果写回 crA/cbA */
int own_gpu_chroma_nr(float *crA, float *cbA, float *crB, float *cbB,
                      int w, int h, int iters);

/* 快速非局部均值（x3fuse FastNlMeans 语义, u16 3ch, L1, 定点权重 LUT）。
 * src3/dst3: 交错 3ch (w*h*3)；lut: int32[lut_n*3]（调用方按 CPU 版同式
 * 构造）；twin/swin 奇数窗宽（如 3/11）。与 CPU 版逐位一致。
 * X3FDEC_GPUNLM=0 单独关闭（X3FDEC_GPU=0 仍关整层）。 */
int own_gpu_nlm3(const uint16_t *src3, uint16_t *dst3, int w, int h,
                 const int32_t *lut, int lut_n, int twin, int swin,
                 int bin_shift);

/* 渲染层显示域色度中值的单通道窗口中值平面（r ≤ 7）。
 * pl: CPU 预计算色度平面（U/V，逐位一致）；omed: 输出每像素窗口中值。
 * 与 CPU chrmed_select 值精确一致（选中值 = 第 n/2 小值，算法无关）。 */
int own_gpu_chrmed_plane(const float *pl, float *omed, int w, int h, int r);

/* 同上，但只算带内 [ry0, ry1) 行 —— 分带给大图省内存用。
 *
 * pl 是一**带**色度平面（bh 行 × w，上下各已带 r 行 halo），omed 接收
 * (ry1-ry0) 行结果。kernel 的边界 clamp 按传入的 bh 做：中间带的 halo
 * 行夹在带内（结果不读，无影响），只有真正贴图像顶/底的带才会 clamp 到
 * 图像边缘 —— 那也正是整幅版会 clamp 的地方，所以两者逐位相同。 */
int own_gpu_chrmed_plane_rows(const float *pl, float *omed,
                              int w, int bh, int ry0, int ry1, int r);

/* Stage1 多尺度降噪的主双边（x3f_stage1.c s1_bilateral，三级金字塔共用）。
 * float 域 + GPU 硬件 exp → 输出与 CPU double 版差 ±1 LSB 级（非逐位）。
 * r ≤ 6；nscale 传 g_s1_nscale。失败返回 -1，调用方回 CPU 行并行版。 */
int own_gpu_stage1_bilat(const float *src, float *dst, int w, int h, int r,
                         const double coef[9], double lin_sat,
                         double level_scale, double nscale);

/* ── 预览 apply 链 GPU 化（2026-10-23，x3f_render.c apply_ex_common 提速）
 *
 * 三段：LUT 管线（apply_row 常规支）→ 显示域色度中值（U/V 计算+中值+重建
 * 全在 GPU，替代 CPU 的 load/uv/rebuild 三趟搬运）→ 16→8 降位（Bayer 抖动）。
 * 约定同上：成功 0 / 失败 -1 回 CPU。float 精度 → 与 CPU double 链 ±1 LSB 级
 * （非逐位，±1 LSB，已 X3F_GPU_APPLY_CMP 校验），**默认预览与导出/CLI 都走**；
 * 需要位级一致时 X3F_GPU_APPLY=0 强制回 CPU（x3f_render_set_gpu_apply 在
 * x3f_render.c 侧把关）。
 *
 * 覆盖范围（不满足时 x3f_render.c 侧不调用，不在此重复校验）：
 *   fillgain=NULL、mono=NULL、hl_y_only=0、HL_WARM/COLOR_FIT 未开。
 * apply_lut16 的可选入参传 NULL 时 kernel 侧绑哑缓冲，按标志跳过。 */
int own_gpu_apply_lut16(const float *lin, const float *clip,
                        const uint16_t *lut, const uint16_t *lutg,
                        const uint16_t *lutb,
                        const float *gain,                    /* [3]，可空 */
                        const float *cm3,                     /* [9]，可空 */
                        const float *cmcb, const float *cmcr, /* [65*65]，可空 */
                        float sat, int use_sat, float hl_desat,
                        float desat_lo, float desat_hi,       /* clip 驱动门槛 */
                        uint16_t *out, int w, int h);

/* buf（u16 交错 3ch，stride 字节/行）原地位色度中值：prep→sel(U/V)→emit。
 * r ≤ 4（r≥5 无寄存器变体，返回 -1 由调用方回 CPU，高 ISO 场景）。
 * CPU 的中值选择值精确一致，亮度重算 float 化。 */
int own_gpu_chrmed_u16(uint16_t *buf, int w, int h, int stride, int r);

/* 16→8 降位：v=(s+128+dth)>>8，dth=Bayer4 抖动（dither=0 时恒 0）。 */
int own_gpu_down8(const uint16_t *tmp, uint8_t *out, int w, int h,
                  int stride, int dither);

#endif
