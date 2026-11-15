/* X3F_QUATTRO_C.C
 *
 * Quattro 合并的纯 C 实现，用来替换上游依赖 OpenCV 的 x3f_denoise.cpp。
 *
 * 上游实现里真正和 OpenCV 有关的只有两处：
 *   1) cv::resize(..., INTER_CUBIC)  —— 把 2944x1836 的 YUV 放大到 5888x3672
 *   2) fastNlMeansDenoising / medianBlur —— 降噪（本移植不使用）
 * 其余（BMT<->YUV 的整数矩阵）本来就是纯算术。
 *
 * 这里按 OpenCV 的定义复刻双三次插值：内核参数 A = -0.75，
 * 采样坐标 fx = (dx + 0.5) * scale - 0.5，边界按 replicate 处理。
 *
 * 注意 scale 的方向：OpenCV 里 scale = 源/目标（不是目标/源）。
 * 早期版本写反了，2 倍放大时算成 2 而不是 0.5，采样坐标超出源图 4 倍、
 * 后半张被 replicate 钳位。因为顶层 Y 会覆盖放大出来的 Y，
 * 亮度看着正常，只是 U/V 被冲淡、整张图发灰 —— 已修正，
 * 修正后 B/M/T 三通道与上游 x3f_extract 逐位相同。
 *
 * 上游 x3f_tools 为 BSD-3-Clause（见 COPYRIGHT_x3f_tools.txt）。
 */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "x3f_io.h"
#include "x3f_denoise.h"
#include "x3f_parallel.h"

#define X3F_O_UV 32768

static inline uint16_t sat16(int32_t v)
{
  return (uint16_t)(v < 0 ? 0 : (v > 65535 ? 65535 : v));
}

/* ---------------------------------------------------------------- */
/* BMT <-> YUV（Y is 4T，用于 F23/Quattro），与上游 Yis4T 完全一致 */

static void bmt_to_yuv_row(void *vctx, size_t row)
{
  x3f_area16_t *image = (x3f_area16_t *)vctx;
  uint32_t col;

  for (col = 0; col < image->columns; col++) {
      uint16_t *p = &image->data[row*image->row_stride + col*image->channels];

      int32_t B = (int32_t)p[0];
      int32_t M = (int32_t)p[1];
      int32_t T = (int32_t)p[2];

      int32_t Y = +4*T;
      int32_t U = +2*B - 2*T;
      int32_t V = +B - 2*M + T;

      p[0] = sat16(Y);
      p[1] = sat16(U + X3F_O_UV);
      p[2] = sat16(V + X3F_O_UV);
  }
}

static void bmt_to_yuv_yis4t(x3f_area16_t *image)
{
  x3f_parallel_for(image->rows, bmt_to_yuv_row, image);
}

static void yuv_to_bmt_row(void *vctx, size_t row)
{
  x3f_area16_t *image = (x3f_area16_t *)vctx;
  uint32_t col;

  for (col = 0; col < image->columns; col++) {
      uint16_t *p = &image->data[row*image->row_stride + col*image->channels];

      int32_t Y = (int32_t)p[0];
      int32_t U = (int32_t)p[1] - X3F_O_UV;
      int32_t V = (int32_t)p[2] - X3F_O_UV;

      int32_t B = ( +Y + 2*U      + 2) / 4;
      int32_t M = ( +Y +   U - 2*V + 2) / 4;
      int32_t T = ( +Y            + 2) / 4;

      p[0] = sat16(B);
      p[1] = sat16(M);
      p[2] = sat16(T);
  }
}

static void yuv_to_bmt_yis4t(x3f_area16_t *image)
{
  x3f_parallel_for(image->rows, yuv_to_bmt_row, image);
}

/* ---------------------------------------------------------------- */
/* 双三次插值（OpenCV INTER_CUBIC 等价） */

/* OpenCV interpolateCubic()，A = -0.75 */
static void cubic_coeffs(double x, double *c)
{
  const double A = -0.75;

  c[0] = ((A*(x + 1) - 5*A)*(x + 1) + 8*A)*(x + 1) - 4*A;
  c[1] = ((A + 2)*x - (A + 3))*x*x + 1;
  c[2] = ((A + 2)*(1 - x) - (A + 3))*(1 - x)*(1 - x) + 1;
  c[3] = 1.0 - c[0] - c[1] - c[2];
}

typedef struct {
  const x3f_area16_t *src;
  x3f_area16_t *dst;
  const int *xs, *ys;
  const double *wx, *wy;
  int dw, sc, dc;
} resize_ctx_t;

/* 输出一行。逐像素的运算顺序与串行版完全一致，保证结果逐位相同 */
static void resize_row(void *vctx, size_t dy)
{
  const resize_ctx_t *c = (const resize_ctx_t *)vctx;
  const x3f_area16_t *src = c->src;
  uint16_t *out = &c->dst->data[dy * c->dst->row_stride];
  double ry[4];
  int dx, ky;

  for (ky = 0; ky < 4; ky++) ry[ky] = c->wy[dy*4 + ky];

  for (dx = 0; dx < c->dw; dx++) {
    double acc[3] = {0, 0, 0};
    int kx, ch;

    for (ky = 0; ky < 4; ky++) {
      const uint16_t *row = &src->data[(size_t)c->ys[dy*4 + ky] * src->row_stride];
      double wyv = ry[ky];

      for (kx = 0; kx < 4; kx++) {
        const uint16_t *p = &row[(size_t)c->xs[dx*4 + kx] * c->sc];
        double w = wyv * c->wx[dx*4 + kx];

        for (ch = 0; ch < 3; ch++) acc[ch] += w * (double)p[ch];
      }
    }
    for (ch = 0; ch < 3; ch++) {
      int32_t v = (int32_t)floor(acc[ch] + 0.5);
      out[(size_t)dx * c->dc + ch] = sat16(v);
    }
  }
}

/* 把 src（3 通道 uint16）放大到 dst 尺寸；索引越界按 replicate 处理 */
static void cubic_resize_rgb16(const x3f_area16_t *src, x3f_area16_t *dst)
{
  const int sw = (int)src->columns, sh = (int)src->rows;
  const int dw = (int)dst->columns, dh = (int)dst->rows;
  const int sc = (int)src->channels, dc = (int)dst->channels;
  /* 注意方向：OpenCV resize 里 fx = (dx + 0.5) * scale - 0.5，
   * scale 是「每个目标像素跨过多少源像素」，即 src/dst。
   * 之前写成了 dst/src（2x 放大时是 2 而不是 0.5），采样坐标一下子
   * 超出源图 4 倍，dx 超过 sw/2 之后全被 replicate 钳位 ——
   * 结果就是 Y 通道被顶层覆盖看不出问题，但 U/V 只剩左上角一小块是对的，
   * 整张图色彩被冲淡。 */
  const double scale_x = (double)sw / dw;
  const double scale_y = (double)sh / dh;
  int dx, dy;

  /* 预先算好每一列/行的源坐标与权重，避免在内层重复计算 */
  int    *xs = (int *)malloc(sizeof(int) * (size_t)dw * 4);
  double *wx = (double *)malloc(sizeof(double) * (size_t)dw * 4);
  int    *ys = (int *)malloc(sizeof(int) * (size_t)dh * 4);
  double *wy = (double *)malloc(sizeof(double) * (size_t)dh * 4);
  int i;

  if (xs == NULL || wx == NULL || ys == NULL || wy == NULL) {
    free(xs); free(wx); free(ys); free(wy);
    return;
  }

  for (dx = 0; dx < dw; dx++) {
    double fx = (dx + 0.5) * scale_x - 0.5;
    int sx = (int)floor(fx);
    double c[4];

    cubic_coeffs(fx - sx, c);
    for (i = 0; i < 4; i++) {
      int ix = sx - 1 + i;
      if (ix < 0) ix = 0;
      if (ix > sw - 1) ix = sw - 1;
      xs[dx*4 + i] = ix;
      wx[dx*4 + i] = c[i];
    }
  }
  for (dy = 0; dy < dh; dy++) {
    double fy = (dy + 0.5) * scale_y - 0.5;
    int sy = (int)floor(fy);
    double c[4];

    cubic_coeffs(fy - sy, c);
    for (i = 0; i < 4; i++) {
      int iy = sy - 1 + i;
      if (iy < 0) iy = 0;
      if (iy > sh - 1) iy = sh - 1;
      ys[dy*4 + i] = iy;
      wy[dy*4 + i] = c[i];
    }
  }

  {
    resize_ctx_t rc;
    rc.src = src; rc.dst = dst;
    rc.xs = xs; rc.ys = ys; rc.wx = wx; rc.wy = wy;
    rc.dw = dw; rc.sc = sc; rc.dc = dc;
    x3f_parallel_for((uint32_t)dh, resize_row, &rc);
  }

  free(xs); free(wx); free(ys); free(wy);
}

/* ---------------------------------------------------------------- */

/* 本移植不做降噪：NLM 需要 OpenCV，且对高光恢复没有帮助 */
void x3f_denoise(x3f_area16_t *image, x3f_denoise_type_t type)
{
  (void)image;
  (void)type;
}

void x3f_set_use_opencl(int flag)
{
  (void)flag;
}

typedef struct {
  x3f_area16_t *expanded;
  const x3f_area16_t *qtop;
} top_copy_ctx_t;

/* 把顶层全分辨率的 Y 写进放大结果，替换掉插值出来的低频 Y */
static void copy_top_row(void *vctx, size_t row)
{
  const top_copy_ctx_t *c = (const top_copy_ctx_t *)vctx;
  uint16_t *out = &c->expanded->data[row * c->expanded->row_stride];
  const uint16_t *top = &c->qtop->data[row * c->qtop->row_stride];
  uint32_t col;

  for (col = 0; col < c->expanded->columns; col++)
    out[col * c->expanded->channels] = sat16((int32_t)top[col] * 4);
}

/* 与上游语义一致：image/active/qtop 会被就地修改。
 * active / active_exp 只用于降噪，本实现忽略。 */
void x3f_expand_quattro(x3f_area16_t *image, x3f_area16_t *active,
			x3f_area16_t *qtop,
			x3f_area16_t *expanded, x3f_area16_t *active_exp)
{
  top_copy_ctx_t top_ctx;

  (void)active;
  (void)active_exp;

  top_ctx.expanded = expanded;
  top_ctx.qtop = qtop;

  bmt_to_yuv_yis4t(image);
  cubic_resize_rgb16(image, expanded);

  /* 顶层（全分辨率）x4 写入 Y 通道，替换掉放大后的低频 Y */
  x3f_parallel_for(expanded->rows, copy_top_row, (void *)&top_ctx);

  yuv_to_bmt_yis4t(expanded);
}
