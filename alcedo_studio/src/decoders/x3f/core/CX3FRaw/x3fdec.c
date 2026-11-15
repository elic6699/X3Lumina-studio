/* X3FDEC.C — 解码后端分发实现（见 x3fdec.h）
 *
 * 默认走旧 x3f_tools C 版（更快；Merrill 色彩上两版在默认渲染设置下不可分辨——
 * 见 x3f_render.h 里那张实测表）。旧 C 版的符号在编译期被改名成 legacy_*，
 * 这里手写它们的声明——注意用的是**同一套结构体**（x3f_t / x3f_area16_t /
 * x3f_directory_entry_t …）。这是成立的：两边的结构体同源，唯一的差异
 * （x3f_hufftree_t 多出来的 fast 字段）藏在指针后面，消费者读到的字段
 * 偏移两边完全一致。所以这里不需要包含 CX3FLegacy 的头文件，也就不会
 * 和本目录的同名头打架。
 */

#include <stdlib.h>
#include <string.h>

#include "x3fdec.h"

#include "x3f_matrix.h"

/* ---- 旧 C 解码器（符号已加 legacy_ 前缀）---- */
extern x3f_t *legacy_x3f_new_from_file(FILE *infile);
extern x3f_return_t legacy_x3f_delete(x3f_t *x3f);
extern x3f_return_t legacy_x3f_load_data(x3f_t *x3f, x3f_directory_entry_t *DE);
extern x3f_directory_entry_t *legacy_x3f_get_raw(x3f_t *x3f);
extern x3f_directory_entry_t *legacy_x3f_get_camf(x3f_t *x3f);
extern char *legacy_x3f_get_wb(x3f_t *x3f);
extern int legacy_x3f_get_camf_float(x3f_t *x3f, char *name, double *val);
extern int legacy_x3f_get_camf_matrix_var(x3f_t *x3f, char *name,
                                          int *dim0, int *dim1, int *dim2,
                                          matrix_type_t type, void **matrix);
extern int legacy_x3f_get_image(x3f_t *x3f,
                                x3f_area16_t *image,
                                x3f_image_levels_t *ilevels,
                                x3f_color_encoding_t encoding,
                                int crop, int fix_bad, int denoise,
                                int apply_sgain, char *wb);
extern int legacy_x3f_get_raw_to_xyz(x3f_t *x3f, char *wb, double *raw_to_xyz);
extern int legacy_x3f_get_spatial_gain(x3f_t *x3f, char *wb,
                                       x3f_spatial_gain_corr_t *corr);
extern void legacy_x3f_cleanup_spatial_gain(x3f_spatial_gain_corr_t *corr,
                                            int corr_num);
extern double legacy_x3f_calc_spatial_gain(x3f_spatial_gain_corr_t *corr,
                                           int corr_num, int row, int col,
                                           int chan, int rows, int cols);

/* ---- 后端选择 ---- */

static int g_decoder = X3F_DECODER_LEGACY;
static int g_decoder_init = 0;
static char g_decoder_env[64];
static char g_decoder_override[64];   /* 运行时覆盖，由 host 壳层经 x3f_decoder_set_override 写入 */

/* 运行时解码后端覆盖：host 壳层（/MT 静态 CRT）调用 x3f_decoder_set_override 写入
 * 本 DLL 内部的全局；本 DLL 是 /MD，壳层里 _putenv 写的是壳层自己那份静态 CRT 的
 * 环境副本，getenv() 在这里读不到，所以必须走 DLL 内部全局。（CLI 进程启动时从 OS
 * 环境块继承，两边都能读到，所以命令行 + 环境变量方式不受影响。） */
void x3f_decoder_set_override(const char *d)
{
    snprintf(g_decoder_override, sizeof(g_decoder_override), "%s", d ? d : "");
}
const char *x3f_decoder_override(void)
{
    return g_decoder_override[0] ? g_decoder_override : NULL;
}

/* 进程用 X3F_DECODER 预设后端，方便批量脚本/界面在不改代码的前提下跑对照。
 * 取值与 x3f_own_dispatch_scene_linear 同一套词（避免两个解析器抢同一环境变量时
 * 语义打架）：
 *   legacy / c / old            → 旧 x3f_tools C 版（底层元数据解析用）
 *   own / auto / fuse / independent / x3fuse / rust
 *                               → 现代路径。底层没有 own 这一档，统一走 x3fuse 的
 *                                 元数据解析，保证与像素解码侧（x3f_scene_linear /
 *                                 own_scene_linear）一致；own 只在 Quattro 世代真正
 *                                 替换像素解码，其余回落 x3fuse。
 * 环境每次变化都重读（不是只在第一解码读一次）——GUI 切解码核心要能即时生效。
 * 运行时覆盖（x3f_decoder_override）优先于进程环境变量。 */
static const char *decoder_env_value(void)
{
    const char *o = x3f_decoder_override();
    return o ? o : getenv("X3F_DECODER");
}

static void ensure_decoder_init(void)
{
    const char *v = decoder_env_value();
    const char *cv = g_decoder_init ? g_decoder_env : "";
    int changed = !g_decoder_init
               || ((v == NULL) != (cv[0] == 0))
               || (v != NULL && cv[0] != 0 && strcmp(v, cv) != 0);
    if (!changed) return;          /* 环境没变：保持当前选择 */
    g_decoder_init = 1;
    snprintf(g_decoder_env, sizeof(g_decoder_env), "%s", v ? v : "");

    if (v == NULL) return;         /* 未设：保持缺省 legacy */
    if (strcmp(v, "legacy") == 0 || strcmp(v, "c") == 0 || strcmp(v, "old") == 0)
        g_decoder = X3F_DECODER_LEGACY;
    else                        /* x3fuse/rust/own/auto/fuse/independent → 现代 x3fuse 路径 */
        g_decoder = X3F_DECODER_X3FUSE;
}

void x3f_render_set_decoder(int which)
{
    g_decoder_init = 1;
    g_decoder = (which == X3F_DECODER_LEGACY) ? X3F_DECODER_LEGACY
                                              : X3F_DECODER_X3FUSE;
}

int x3f_render_decoder(void)
{
    ensure_decoder_init();
    return g_decoder;
}

const char *x3f_render_decoder_name(void)
{
    ensure_decoder_init();
    return g_decoder == X3F_DECODER_LEGACY ? "legacy-c" : "x3fuse";
}

static int is_legacy(void)
{
    ensure_decoder_init();
    return g_decoder == X3F_DECODER_LEGACY;
}

/* ---- 分发 ---- */

x3f_t *x3fdec_new_from_file(FILE *infile)
{
    return is_legacy() ? legacy_x3f_new_from_file(infile)
                       : x3f_new_from_file(infile);
}

x3f_return_t x3fdec_delete(x3f_t *x3f)
{
    return is_legacy() ? legacy_x3f_delete(x3f) : x3f_delete(x3f);
}

x3f_return_t x3fdec_load_data(x3f_t *x3f, x3f_directory_entry_t *DE)
{
    return is_legacy() ? legacy_x3f_load_data(x3f, DE) : x3f_load_data(x3f, DE);
}

x3f_directory_entry_t *x3fdec_get_raw(x3f_t *x3f)
{
    return is_legacy() ? legacy_x3f_get_raw(x3f) : x3f_get_raw(x3f);
}

x3f_directory_entry_t *x3fdec_get_camf(x3f_t *x3f)
{
    return is_legacy() ? legacy_x3f_get_camf(x3f) : x3f_get_camf(x3f);
}

char *x3fdec_get_wb(x3f_t *x3f)
{
    return is_legacy() ? legacy_x3f_get_wb(x3f) : x3f_get_wb(x3f);
}

int x3fdec_get_camf_float(x3f_t *x3f, char *name, double *val)
{
    return is_legacy() ? legacy_x3f_get_camf_float(x3f, name, val)
                       : x3f_get_camf_float(x3f, name, val);
}

int x3fdec_get_camf_matrix_var(x3f_t *x3f, const char *name,
                               int *dim0, int *dim1, int *dim2,
                               matrix_type_t type, double *out, int cap)
{
    char nm[96];
    void *p = NULL;
    int d0 = 0, d1 = 0, d2 = 0;
    long n;

    if (x3f == NULL || name == NULL || out == NULL || cap <= 0) return 0;
    /* x3f_get_camf_matrix_var 收的是 char*（内部只读），这里给一份可写的副本，
     * 免得调用方为每个键名准备一个数组。 */
    snprintf(nm, sizeof(nm), "%s", name);

    /* 维数是精确匹配的：3 维 → dim2 非空；2 维 → dim2 必须 NULL；
     * 1 维 → dim1 也得是 NULL。三种都试一遍，命中即止。 */
    if (is_legacy()) {
        if (!legacy_x3f_get_camf_matrix_var(x3f, nm, &d0, &d1, &d2, type, &p)) {
            p = NULL; d1 = 1; d2 = 1;
            if (!legacy_x3f_get_camf_matrix_var(x3f, nm, &d0, &d1, NULL, type, &p)) {
                p = NULL; d1 = 1;
                if (!legacy_x3f_get_camf_matrix_var(x3f, nm, &d0, NULL, NULL, type, &p))
                    return 0;
            }
        }
    } else {
        if (!x3f_get_camf_matrix_var(x3f, nm, &d0, &d1, &d2, type, &p)) {
            p = NULL; d1 = 1; d2 = 1;
            if (!x3f_get_camf_matrix_var(x3f, nm, &d0, &d1, NULL, type, &p)) {
                p = NULL; d1 = 1;
                if (!x3f_get_camf_matrix_var(x3f, nm, &d0, NULL, NULL, type, &p))
                    return 0;
            }
        }
    }
    if (p == NULL) return 0;
    n = (long)(d0 > 0 ? d0 : 1) * (d1 > 0 ? d1 : 1) * (d2 > 0 ? d2 : 1);
    if (dim0) *dim0 = d0;
    if (dim1) *dim1 = d1;
    if (dim2) *dim2 = d2;
    if (n > cap) n = cap;
    memcpy(out, p, (size_t)n * sizeof(double));
    return (int)n;
}

int x3fdec_get_image(x3f_t *x3f,
                     x3f_area16_t *image,
                     x3f_image_levels_t *ilevels,
                     x3f_color_encoding_t encoding,
                     int crop, int fix_bad, int denoise,
                     int apply_sgain, char *wb)
{
    if (is_legacy())
        return legacy_x3f_get_image(x3f, image, ilevels, encoding,
                                    crop, fix_bad, denoise, apply_sgain, wb);
    return x3f_get_image(x3f, image, ilevels, encoding,
                         crop, fix_bad, denoise, apply_sgain, wb);
}

int x3fdec_get_raw_to_xyz(x3f_t *x3f, char *wb, double *raw_to_xyz)
{
    return is_legacy() ? legacy_x3f_get_raw_to_xyz(x3f, wb, raw_to_xyz)
                       : x3f_get_raw_to_xyz(x3f, wb, raw_to_xyz);
}

int x3fdec_get_spatial_gain(x3f_t *x3f, char *wb, x3f_spatial_gain_corr_t *corr)
{
    return is_legacy() ? legacy_x3f_get_spatial_gain(x3f, wb, corr)
                       : x3f_get_spatial_gain(x3f, wb, corr);
}

void x3fdec_cleanup_spatial_gain(x3f_spatial_gain_corr_t *corr, int corr_num)
{
    if (is_legacy()) legacy_x3f_cleanup_spatial_gain(corr, corr_num);
    else             x3f_cleanup_spatial_gain(corr, corr_num);
}

double x3fdec_calc_spatial_gain(x3f_spatial_gain_corr_t *corr,
                                int corr_num, int row, int col, int chan,
                                int rows, int cols)
{
    if (is_legacy())
        return legacy_x3f_calc_spatial_gain(corr, corr_num, row, col, chan,
                                            rows, cols);
    return x3f_calc_spatial_gain(corr, corr_num, row, col, chan, rows, cols);
}
