/* X3FDEC.H — 解码后端分发
 *
 * 现在二进制里同时有**两个解码实现**，运行时可切：
 *   0 = x3fuse（Rust，vendor/lib/libx3f.a）—— 对照/后备
 *   1 = 旧 x3f_tools C 版（Sources/CX3FLegacy）—— **默认**（更快）
 *
 * 两者是同一套 C API（x3fuse 把上游的 C API 原样导出了），所以只要在
 * 入口处分发就行，渲染层 x3f_render.c 的逻辑完全不用分叉。
 *
 * 旧版那一份在编译时被 gen_prefix.sh 生成的宏整体改名成 legacy_*，
 * 否则两边会在链接期撞符号（实测 71 个重名）。
 *
 * 只分发**有状态**的入口：解析、加载、取图、取标定、空间增益。
 * 纯数学的（x3f_3x3_3x3_mul / x3f_scalar_3x3_mul / x3f_XYZ_to_sRGB）
 * 两边一模一样，不分发，直接走 Rust 那份。
 */

#ifndef X3FDEC_H
#define X3FDEC_H

#include <stdio.h>

#include "x3f_io.h"
#include "x3f_process.h"
#include "x3f_meta.h"
#include "x3f_spatial_gain.h"

enum {
    X3F_DECODER_X3FUSE = 0,
    X3F_DECODER_LEGACY = 1
};

void        x3f_render_set_decoder(int which);
int         x3f_render_decoder(void);
const char *x3f_render_decoder_name(void);
/* 运行时解码后端覆盖（GUI setDecoder 写入）。host 壳层是 /MT 静态 CRT，DLL 是 /MD，
 * 壳层 _putenv 写的环境副本 DLL 的 getenv 读不到，故用 DLL 内部全局绕过该错配。 */
void        x3f_decoder_set_override(const char *d);
const char *x3f_decoder_override(void);

/* 与 x3f_* 同名同签名，只是多一层分发。 */
x3f_t *x3fdec_new_from_file(FILE *infile);
x3f_return_t x3fdec_delete(x3f_t *x3f);
x3f_return_t x3fdec_load_data(x3f_t *x3f, x3f_directory_entry_t *DE);
x3f_directory_entry_t *x3fdec_get_raw(x3f_t *x3f);
x3f_directory_entry_t *x3fdec_get_camf(x3f_t *x3f);
char *x3fdec_get_wb(x3f_t *x3f);
int x3fdec_get_camf_float(x3f_t *x3f, char *name, double *val);

/* 取 CAMF 里的一个矩阵（**支持 3 维**，值以 f64 返回）。
 *
 * 为什么要单开一个：`x3f_get_camf_matrix_var` 的维数是"精确匹配"的 ——
 * 2 维的条目必须把 dim2 传 NULL、3 维的必须传非 NULL，传错就是参数错误。
 * 色彩模式那几张表正好两种都有（CMCM 是 3x3、MultiAxisTable 是 2x5x21），
 * 所以这里按 3 维 → 2 维 → 1 维依次试，命中即返回，调用方不用管维数。
 *
 * 返回**元素个数**（0 = 没有这个条目 / 类型不是 float / 不是矩阵条目），
 * f64 最多拷 cap 个进 out。dim0/dim1/dim2 可传 NULL。 */
int x3fdec_get_camf_matrix_var(x3f_t *x3f, const char *name,
                               int *dim0, int *dim1, int *dim2,
                               matrix_type_t type, double *out, int cap);

int x3fdec_get_image(x3f_t *x3f,
                     x3f_area16_t *image,
                     x3f_image_levels_t *ilevels,
                     x3f_color_encoding_t encoding,
                     int crop,
                     int fix_bad,
                     int denoise,
                     int apply_sgain,
                     char *wb);
int x3fdec_get_raw_to_xyz(x3f_t *x3f, char *wb, double *raw_to_xyz);
int x3fdec_get_spatial_gain(x3f_t *x3f, char *wb,
                            x3f_spatial_gain_corr_t *corr);
void x3fdec_cleanup_spatial_gain(x3f_spatial_gain_corr_t *corr, int corr_num);
double x3fdec_calc_spatial_gain(x3f_spatial_gain_corr_t *corr,
                                int corr_num, int row, int col, int chan,
                                int rows, int cols);

#endif /* X3FDEC_H */
