/* X3F_RENDER.C — X3F 调整管线实现 */

#include "x3f_render.h"
#include "x3f_tonecurve.h"    /* SPP「色调曲线」四滑杆（显示域 LUT） */
#include "x3f_mac.h"           /* SPP Stage2 MAC（MultiAxisCC），见 x3f_mac.c */
#include "x3f_rcps.h"          /* SPP RowColumnPatternSuppression 复刻，见 x3f_rcps.c */
#include "x3f_spp.h"           /* Merrill SPP 原生链成品旁路，见文件末尾 */
#include "x3f_hole_fix.h"      /* v6 裁剪孔洞修正（X3F_HOLE_FIX 门控，默认关） */
#include "x3f_mono_grain_tables.h" /* 黑白「胶片颗粒」四张静态表（lib95 逐值提取） */
#include "x3f_classic_tone_v2.h"  /* Classic-Foveon 多帧影调结点（加载期还原） */

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* X3F_APPLY_TIME 打点用的底层函数（定义在 apply 计时那一段）。 */
static int apply_t_on(void);
static double apply_t_footprint_mb(void);
static double apply_t_peak_mb(void);
#if !defined(_WIN32)
#include <sys/resource.h>   /* getrusage：X3F_APPLY_TIME 的峰值内存打点 */
#endif
#ifdef __APPLE__
#include <mach/mach.h>      /* task_info：X3F_APPLY_TIME 的 phys_footprint 打点 */
#include <mach/task_info.h>
#include <mach-o/dyld.h>   /* _NSGetExecutablePath：定位主程序，读同目录配置 */
#endif

#include "x3fdec_alias.h"

#ifdef _WIN32
/* MSVC 无 pthread：pthread_self 目前只有调试打印（X3F_OE_SEL_DEBUG）在用，
 * 垫成线程 id —— 与 POSIX 的 pthread_t 同样只作 %p 显示，不参与比较。 */
#include <windows.h>
static DWORD pthread_self(void) { return GetCurrentThreadId(); }
#endif

/* 给 x3f_camf_matrix_f64 传 dims 的尺寸宏（文件末尾 6322 处也有同值定义，
 * 此处前置以满足 3522 经典 Foveon auto-tone 块的提前引用）。 */
#define X3F_CAMF_DIMS 3

/* Metal GPU 层（显示域色度中值提速，与 own_decoder.c 的解码 GPU 层同一套） */
#if defined(__has_include)
#  if __has_include("own_gpu.h")
#    include "own_gpu.h"
#    define HAVE_OWN_GPU 1
#  endif
#endif

/* ── 黑白锐度半径：主程序同目录配置文件 ──────────────────────────────────
 * 文件名 bw_sharp_r.txt（置于可执行文件同目录），内容仅接受 2 或 3
 * （"2" / "r=3" 均可），缺失或越界一律按 2。
 * 取值优先级：环境变量 X3F_SHARP_BW_R > 同目录文件 > 默认 2。
 * 文件只读一次（启动时），结果缓存于 g_bw_r_file。
 */
static int g_bw_r_file = -2;   /* -2=尚未读取；-1=文件无效/缺失 */

static int parse_bw_r(const char *s)
{
    while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r') s++;
    if ((*s == 'r' || *s == 'R') && s[1] == '=') s += 2;
    while (*s == ' ' || *s == '\t') s++;
    if (!*s) return -1;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s) return -1;
    return (v == 2 || v == 3) ? (int)v : -1;
}

static int read_bw_r_from_file(void)
{
#if defined(__APPLE__)
    char path[4096];
    uint32_t size = (uint32_t)sizeof(path);
    if (_NSGetExecutablePath(path, &size) != 0) return -1;
    char *slash = strrchr(path, '/');
    if (!slash) return -1;
    *(slash + 1) = '\0';                 /* 截断到主程序目录（保留尾斜杠） */
    char cfg[4200];
    snprintf(cfg, sizeof(cfg), "%sbw_sharp_r.txt", path);
    FILE *f = fopen(cfg, "r");
    if (!f) return -1;
    char line[64];
    int r = -1;
    if (fgets(line, sizeof(line), f)) r = parse_bw_r(line);
    fclose(f);
    return r;
#else
    return -1;   /* 非 macOS：仅环境变量/默认 */
#endif
}

/* ── SPP 派生表：**生成文件，本仓库不带** ──────────────────────────────
 *
 * 下面两份表里的数字，都是让 SIGMA Photo Pro 自己的 dylib 把结果算出来得到
 * 的输出（探针与生成脚本在 outputs/tools/sppref/，方法见该目录的 README）：
 *
 *   x3f_colormode_tone.h —— 每档色彩模式的色调曲线
 * （ `GenerateF20ToneCurve`，modesweep 扫出来）
 *   x3f_oe_tone_gain.h   —— 「过曝矫正」勾选 / 未勾选两条 LUT 的比值
 *
 * 它们**不进仓库**：公开的是方法（怎么把它问出来 + 生成脚本），产物留在本机。
 * 本机装了 SPP 就按那本 README 生成；没有这两份头文件也编得过 —— 回落成
 * "不带 SPP 曲线"的口径（每档曲线增益恒 1、过曝差恒 1），只走我们自己的
 * 基础曲线。想核对带表与不带表的差别：`X3F_OE_TONE=0/1` 那套环境变量。 */
#if defined(__has_include)
#  if __has_include("x3f_colormode_tone.h")
#    include "x3f_colormode_tone.h"
#    define X3F_HAVE_SPP_CMTONE 1
#  else
#    define X3F_HAVE_SPP_CMTONE 0
#  endif
#  if __has_include("x3f_oe_tone_gain.h")
#    include "x3f_oe_tone_gain.h"
#    define X3F_HAVE_SPP_OE_TONE 1
#  else
#    define X3F_HAVE_SPP_OE_TONE 0
#  endif
#else
#  define X3F_HAVE_SPP_CMTONE  0
#  define X3F_HAVE_SPP_OE_TONE 0
#endif

#include "x3fcore.h"
#include "x3f_io.h"
#include "x3f_process.h"
#include "x3f_matrix.h"
#include "x3f_meta.h"
#include "x3f_parallel.h"
#include "x3f_spatial_gain.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 解码层的 CAMF 矩阵取数口（定义在同文件后段，这里提前声明给 §19 实验用）。 */
extern int x3f_camf_matrix_f64(const char *path, const char *name,
                               double *out, int cap, unsigned *dims);

/* 「SPP 色彩模式」那一段在文件中间（见下面的长注释），这里先声明一下：
 * x3f_edit_defaults 要用它把 color 字段铺成"什么都不做"。 */
static void color_mode_unit(x3f_color_mode_t *o);

/* ---------------------------------------------------------------- */
/* 默认参数：全部归零，即"只做相机标定的还原 + 基础曲线" */

void x3f_edit_defaults(x3f_edit_params_t *p)
{
    if (p == NULL) return;
    memset(p, 0, sizeof(*p));
    p->hl_keep = 0.5;      /* 高光端按"相机观感"和"有层次"的中间值起手 */
    p->nr_banding = 0.0;   /* 降噪默认关：SPP 默认也是 0，且全分辨率很慢 */
    p->nr_luma = 0.0;
    p->nr_chroma = 0.0;
    /* Quattro 基线亮度降噪：2026-11-04 起**默认关**（见 nr_luma_baseline
     * 那段说明）。档位 0.8 留着供显式开启时用；只对 g_nr.quattro_model
     * 生效，Merrill 走它自己的 CAMF 噪声模型。 */
    p->nr_q_baseline = 0;
    p->nr_q_base = 0.8;
    /* 结构门控（同一段）：基线是"我们自己加的"，所以按局部信噪比打折扣 ——
     * 平坦区照跑满、纹理区把残差还回去。keep_max 0.65 是初始值，待实测。 */
    p->nr_q_gate = 1;
    p->nr_q_gate_k = 0.75;
    p->sharp_res = 1.0;    /* 锐化量分辨率缩放：全尺寸 = 1（预览由 app 传入 <1） */
    /* 输出色彩空间：默认 sRGB —— 与加这个开关之前逐位相同（memset 已经是 0，
     * 这里显式写一遍是为了让"默认值"这件事在代码里可搜索、不被将来改 memset
     * 的人无意破坏）。广色域只在导出时按用户选的档位覆盖（六档都是完整观感链，
     * 见 x3f_render.h 的 out_space 段）。 */
    p->out_space = X3F_OUT_SRGB;
    color_mode_unit(&p->color);   /* 色彩模式：默认单位阵（= 与加这个功能之前逐字节同） */
    /* 辉光默认关（glow=0）；radius/threshold 给非零默认，仅 glow>0 时参与，
     * 因此零滑杆输出与加 glow 前逐位相同。 */
    p->glow = 0.0;
    p->glow_radius = 0.5;
    p->glow_threshold = 0.7;
    /* WB RGB 三通道强度：**1.0 = 中性**（乘子），不是 0。 */
    p->wb_red = 1.0;
    p->wb_green = 1.0;
    p->wb_blue = 1.0;
    /* 亮部保色（x3f_render.h 末尾那段）：k = 1.0 = 关（逐位不变）；
     * 窗与色相闸给"裸乘子"的默认（窗 0.78..0.93、色相不限）。 */
    p->hl_chroma = 1.0;
    p->hl_chroma_win_lo = 0.78;
    p->hl_chroma_win_hi = 0.93;
    p->hl_chroma_hue_lo = 0.0;
    p->hl_chroma_hue_hi = 360.0;
    /* 第二段不重放机内对齐表：默认 0（= 单段显示链照旧，逐位不变）。
     * 只有两段式的第二段会置 1，见 x3f_render.h 的 align_look_off 段。 */
    p->align_look_off = 0;
}

/* ---------------------------------------------------------------- */
/* 基础曲线（"相机观感"）
 *
 * 裸的 sRGB 伽马只是把线性数据搬进显示空间，不是一条"照片"曲线：中间调
 * 落不到位、两端没有滚降。相机直出（SIGMA Photo Pro 的默认档也一样）在
 * 伽马之前还有一条完整的色调曲线。这里补的就是它。
 *
 * 曲线是**在线性域**上定义的：cin = T(lin)。为什么不在显示域做？因为
 * 相机的曲线本来就是"线性光 -> 线性显示值"的函数，放到显示域反而要把
 * 伽马来回搬两次。
 *
 * T 是拿相机内嵌 JPEG 直出当参照拟合出来的：
 *   同一张 raw，我们的 liner 数据（base=0 就是 srgb_encode(lin)）
 *   对同一像素的 JPEG 亮度做 srgb_decode，得到大批 (lin, T) 样本，
 *   按 log2(lin) 分箱取中位数 -> 打点 -> 单调化。
 * 6 张样片、约 1200 万个像素的拟合结果：平均绝对误差 1.73/255
 * （不做任何曲线时是 26.57/255）。过程见 work/lab/fit2.py。
 *
 * 曲线特征（对照相机）：
 *   · 暗部  T ≈ 0.59·lin   —— 比纯伽马更压，暗部沉得下去
 *   · 中间  T ≈ 2.1·lin    —— 中调抬起，这是"不偏暗"的主要来源
 *   · 高光  T = 1 - C/lin  —— 柔肩收进白，不是硬切
 *
 * 表是 log2(lin) 上 0.5 EV 一格、41 个点，线性插值（建 LUT 时查一次，
 * 开销可以忽略）。所有用户参数为 0 时看到的就是这条曲线。
 */



/* ---------- 主流程 ---------- */

/* 降噪标定的加载（定义在文件后半段：先声明，load_common 里要用） */
static void nr_load_model(const char *path);
static double read_shooting_iso(const char *path);
/* 这张照片的 ISO（同文件后半段，从 CAMF 的 SensorISO 读）。
 * 「细节」的增益按 公式依赖它，所以提前声明。*/
static double x3f_file_iso(void);
static double g_shooting_iso;      /* 拍摄 ISO，nr_load_model 里填 */
/* 解码那一步的 NLM 有没有真的连亮度一起降。**定义在下面降噪那一段**
 * （`static int g_nlm_luma_on;`，和 g_nr 放一起），但 load_common 要用它，
 * 所以要在这儿先声明一下 —— 否则那个使用点在定义之前，编译不过。 */
static int g_nlm_luma_on;
/* NR 那几个 worker 要用（定义在文件后半段，和 nr_apply 一起） */
static int nr_flag(const char *name, int dflt);
static double nr_env(const char *name, double dflt);

/* ---------- 基础曲线的开关与表 ---------- */

/* 基础曲线开关（2026-11-05 改为**线程局部**）。
 *
 * 原来是进程级 `static int`，两段式（胶片引擎前置）的用法是
 *     set_base_curve(0) → 渲染第二段 → set_base_curve(1)
 * 中间没有任何锁。App 里预览 / 并行导出是**多线程**的：只要这个窗口里有
 * 另一条渲染线程出图，它就会在"曲线关掉"的状态下渲染 —— 整幅变平、黄墙
 * 发粉，而且**跨帧持续**，只有重启进程才恢复（2026-11-05 实际踩到：
 * 导出 TIFF 比 CLI 同参数暗部亮 6 码、亮部暗 6 码，重启 App 后消失）。
 *
 * 改成 thread_local 后，set 与 apply 在同一个线程（Swift 侧是同步调用），
 * 互不干扰；没调用过 set 的线程拿到默认 1，行为与改前逐位一致。 */
static _Thread_local int g_base_curve = 1;

void x3f_render_set_base_curve(int enable) { g_base_curve = enable ? 1 : 0; }
int  x3f_render_base_curve_enabled(void)    { return g_base_curve; }

/* sd Quattro H 机内 as-shot 链（own_decoder.c）已经把机内影调曲线烤进
 * scene-linear 输出，渲染层不能再叠一次产品基础曲线。解码侧注册查询回调；
 * 未注册（Mac / 其它后端 / 其它机型）= 0，整条路径逐位不变。 */
static int (*g_sdqh_native_query)(void) = NULL;
void x3f_render_register_sdqh_query(int (*fn)(void)) { g_sdqh_native_query = fn; }
static int sdqh_native_display(void)
{
    return g_sdqh_native_query != NULL && g_sdqh_native_query() != 0;
}

/* ---------- Classic-Foveon 相机 auto-tone S 曲线（SD14 等 DBG_AutoContrast 机型）----------
 *
 * 移植自 x3fuse process.rs 的 classic_tone_curve。相机内 auto-tone solver 把一张
 * 基础 S 曲线按该帧的 DBG_AutoHilite/Shadow/Contrast 弯成观感。产品这条渲染链走
 * x3f_get_image(NONE)+build_tone_lut，不经过 process.rs 的 convert_data_controlled
 * （那条只对非 NONE 编码生效），所以经典 tone 必须在这里独立施加。
 *
 * 作用位置：base curve 之后的"显示线性"（tone_curve(p,lin) 的输出，0..1），量纲
 * 与 process.rs 里 srgb_decode(显示值) 完全一致，无需任何域换算 —— 这正是
 * convert_data_controlled 里 classic tone 所作用的那个量。 */
/* ---------- Classic 原生链（2026-11-05）----------
 *
 * 需求：非 Merrill / 非 Quattro 世代的机型（SD14 / SD15 / SD9 / SD10 / DP1 /
 * DP2 / DP1x / DP2x …）保留 x3fuse 的全部原生解码算法，只叠加早期那项
 * SD14 色彩修复（AutoRGBNeutral 白平衡，位于 x3fuse 的 x3f_get_gain，见
 * process.rs；本层不动它）。Quattro / Merrill 的管线一个字节都不改。
 *
 * 像素解码本来就满足：own 解码器只认 Quattro，其余一律回落 x3f_scene_linear
 * （见 own_decoder.c 的 is_quattro 判据），即 fuse 域原生。差距来自**本渲染层**
 * 后来叠加的画质算子：相机 auto-tone 曲线 / 基础曲线、align_look 输出码重映射、
 * 过曝矫正差值曲线、色彩模式小 S、线性域 R×0.96 冷调、高频软压缩。这些在
 * 零滑杆时仍默认生效，把 x3fuse 原本的皮肤质感改掉了。
 *
 * 门控与 g_hf_classic 同口径：!is_merrill && !is_quattro_gen。X3I(SFD) 容器
 * 属于 Quattro 世代，不会命中。X3F_CLASSIC_NATIVE 四档：
 *   0 = 旧链（3.7.31 现状，全部叠加算子保留）
 *   1 = 严格原生：连相机 / 基础曲线一起退出，等价 x3fuse 原版。
 *       ⚠ 实测 0173 会重新偏橙红（中性 R/G 0.978 vs 相机 0.957）且暗 5~7 级，
 *       因为显示域的色彩对齐链也被拆了，**不作默认**。
 *   2 = 原生解码 + 早期 SD14 相机影调：保留 10-25 那张 65 点 auto-tone
 *       曲线（v2/v3 与全部后期算子仍关），影调不回落成「暗平」
 *   3 = 默认。只关软压缩与冷调，相机曲线 / align_look / 色彩模式小 S
 *       / oe_tone 全部按旧链保留（影调不变，只还皮肤细节）。
 *       AutoRGBNeutral 白平衡修复在 fuse 域，四档都原样保留。
 *       0173 实测：中性 R/G 0.9569 B/G 0.9210（相机 0.9574/0.9251），
 *       皮肤高频 3.527→4.020（相机 3.841）。
 * 注意这是 per-load 的进程级状态，与 g_classic_active 同一套既有约定。 */
static int g_classic_native = 0;

/* 只有严格原生档连影调曲线一起旁路；2 档仍吃早期那条相机曲线。 */
static int classic_native_strict(void) { return g_classic_native == 1; }
/* 1/2 = 完整原生链（连影调重映射一起旁路）；3 = 只关皮肤算子。 */
static int classic_native_full(void)
{
    return g_classic_native == 1 || g_classic_native == 2;
}

static int g_classic_tone = 1;   /* 总开关（默认开；X3F_CLASSIC_TONE=0 关） */
static int g_classic_active = 0; /* 当前文件是否 SD14 classic 机型（load 阶段探测） */
static int g_classic_sd1 = 0;    /* 当前文件是否**原版** SIGMA SD1（走 SD1 专属实测曲线） */
static double g_classic_hilite = 0.0, g_classic_shadow = 0.0, g_classic_contrast = 0.0;
static double g_classic_exposure = 0.0;  /* 逐图 DBG_AutoExposureNA（相机自动曝光记录） */

void x3f_render_set_classic_tone(int enable) { g_classic_tone = enable ? 1 : 0; }

#define CLASSIC_TONE_N 65
static const double kClassicToneBase[CLASSIC_TONE_N] = {
    0.000954, 0.009936, 0.028653, 0.045767, 0.065975, 0.089017, 0.112060, 0.138157,
    0.165243, 0.192223, 0.217870, 0.243518, 0.269166, 0.294456, 0.319298, 0.344140,
    0.368982, 0.391516, 0.410053, 0.428590, 0.447127, 0.465664, 0.476216, 0.481879,
    0.487543, 0.493206, 0.498869, 0.505867, 0.515537, 0.525206, 0.534876, 0.544545,
    0.554215, 0.564830, 0.578086, 0.591342, 0.604599, 0.617855, 0.631112, 0.644368,
    0.658296, 0.673344, 0.688393, 0.703442, 0.718490, 0.733539, 0.748588, 0.763825,
    0.782026, 0.800228, 0.818429, 0.836631, 0.854832, 0.873033, 0.890647, 0.904703,
    0.918760, 0.932816, 0.946873, 0.960929, 0.969826, 0.977922, 0.986018, 0.994115,
    1.000000,
};
static const double kClassicToneHilite[CLASSIC_TONE_N] = {
    0.003498, -0.000284, 0.010055, 0.018152, 0.026837, 0.036753, 0.047351, 0.061344,
    0.077080, 0.093190, 0.110711, 0.127115, 0.143309, 0.157748, 0.171693, 0.185588,
    0.199753, 0.214883, 0.229882, 0.244769, 0.258773, 0.262643, 0.262036, 0.262404,
    0.262772, 0.262670, 0.250433, 0.229361, 0.208364, 0.187476, 0.166588, 0.145700,
    0.124263, 0.102062, 0.081871, 0.061971, 0.042070, 0.022169, 0.001344, -0.022656,
    -0.044414, -0.067753, -0.091349, -0.114944, -0.138540, -0.162136, -0.185956, -0.208287,
    -0.230473, -0.252659, -0.274845, -0.297031, -0.318856, -0.337796, -0.354263, -0.369225,
    -0.384187, -0.399149, -0.414111, -0.429073, -0.444035, -0.426500, -0.284334, -0.142167,
    0.000000,
};
static const double kClassicToneShadow[CLASSIC_TONE_N] = {
    -0.004735, 0.022435, 0.029190, 0.014837, -0.014417, -0.049743, -0.085884, -0.123718,
    -0.162504, -0.201227, -0.236410, -0.267906, -0.299317, -0.327534, -0.352342, -0.377032,
    -0.401832, -0.422449, -0.437894, -0.453121, -0.468012, -0.478968, -0.480538, -0.478835,
    -0.477131, -0.474318, -0.454488, -0.434387, -0.412921, -0.391357, -0.369792, -0.348227,
    -0.326037, -0.300715, -0.272103, -0.243143, -0.214183, -0.185223, -0.155207, -0.121569,
    -0.083843, -0.049735, -0.016155, 0.017425, 0.051004, 0.084584, 0.118004, 0.154801,
    0.191903, 0.229005, 0.266107, 0.303209, 0.340643, 0.380735, 0.424884, 0.472139,
    0.519395, 0.566650, 0.613905, 0.661160, 0.708416, 0.701652, 0.467768, 0.233884,
    -0.000000,
};
static const double kClassicToneContrast[CLASSIC_TONE_N] = {
    -0.001227, -0.002128, -0.001485, -0.002364, -0.003240, -0.003023, -0.002026, -0.000163,
    0.002039, 0.004548, 0.011008, 0.022285, 0.033479, 0.043282, 0.052835, 0.062557,
    0.072386, 0.077627, 0.081563, 0.085625, 0.089307, 0.088914, 0.084205, 0.079003,
    0.073801, 0.070175, 0.086493, 0.093848, 0.100460, 0.107133, 0.113806, 0.120479,
    0.126832, 0.130641, 0.131352, 0.131969, 0.132586, 0.133203, 0.133282, 0.131514,
    0.128943, 0.132763, 0.137353, 0.141943, 0.146534, 0.151124, 0.155440, 0.154324,
    0.152754, 0.151185, 0.149615, 0.148045, 0.146679, 0.146938, 0.143021, 0.134575,
    0.126129, 0.117683, 0.109237, 0.100791, 0.092345, 0.077220, 0.051480, 0.025740,
    -0.000000,
};

static double classic_tone_lookup(const double *tab, double x)
{
    if (x < 0.0) x = 0.0;
    else if (x > 1.0) x = 1.0;
    double pos = x * (CLASSIC_TONE_N - 1);
    int i = (int)pos;
    if (i > CLASSIC_TONE_N - 2) i = CLASSIC_TONE_N - 2;
    double t = pos - i;
    return tab[i] + t * (tab[i + 1] - tab[i]);
}

/* 与 process.rs classic_tone_curve 逐位对应：base 基础 S 曲线，三滑块把曲线
 * 弯离 base（hilite 钳到 ≥0），75% 保留 base 形状，避免极端滑块下曲线翻转。 */
static double classic_tone_curve(double x, double hilite, double shadow, double contrast)
{
    double base = classic_tone_lookup(kClassicToneBase, x);
    double h = hilite > 0.0 ? hilite : 0.0;
    double lift = classic_tone_lookup(kClassicToneHilite, x) * h
               + classic_tone_lookup(kClassicToneShadow, x) * shadow
               + classic_tone_lookup(kClassicToneContrast, x) * contrast;
    double slider = x + lift;
    if (slider < 0.0) slider = 0.0;
    else if (slider > 1.0) slider = 1.0;
    double out = base + (slider - base) * 0.75;
    if (out < 0.0) out = 0.0;
    else if (out > 1.0) out = 1.0;
    return out;
}

/* ---------- Classic-Foveon 多帧影调曲线 v2（带高光余量） ----------
 *
 * 旧 65 点表的定义域只到 lin=1.0，classic_tone_lookup 把更大的值一律钳到
 * 表末 1.0，于是真过曝帧里 lin∈[1,1.6] 的层次全变成 255。相机的影调表
 * 索引域按传感器饱和点归一，天然留有余量，高光沿肩部连续滚降。
 *
 * v2 用 9 张经典 Foveon 样张（含首张真过曝帧）对各自机内 JPEG 逐箱拟合，
 * 加权 pooled 出一条统一曲线，索引域扩到 1.6：
 *   · 结点 u_i = 1.6·(i/128)^(1/0.6)，暗部加密；查表 q = (u/1.6)^0.6。
 *   · 表末渐近 0.947（sRGB≈249），不再硬裁到纯白。
 *   · DBG_AutoExposureNA 是逐图自动曝光记录（9 张实测 −0.4…+0.9）。
 *     多帧标定显示它**不是** EV 级的索引域缩放：扫 2^(a·ex) 的最优 a≈0.05，
 *     明显缩放会让各帧曲线更分散。故只保留这个极弱系数，勿改成 2^ex。
 *
 * 门控：仅 classic 且非原版 SD1；X3F_CLASSIC_TONE_V2=0（或既有的
 * X3F_CLASSIC_TONE=0）整体回退旧表 + 旧输出码重映射，便于逐位回归。
 * 表值由 x3f_packed_assets.c 在加载期还原，出货二进制不含明文。 */
#define CLASSIC_V2_HEADROOM 1.6
#define CLASSIC_V2_GAMMA    0.6
#define CLASSIC_V2_EX_SCALE 0.05

static int classic_tone_v2_enabled(void)
{
    static int cached = -1;
    if (cached < 0) {
        const char *e  = getenv("X3F_CLASSIC_TONE_V2");
        const char *ce = getenv("X3F_CLASSIC_TONE");
        cached = ((e != NULL && *e != '\0' && atoi(e) == 0) ||
                  (ce != NULL && *ce != '\0' && atoi(ce) == 0)) ? 0 : 1;
    }
    return cached && g_classic_tone && g_classic_active && !g_classic_sd1
           && (g_classic_native == 0 || g_classic_native == 3);
}

/* 相机影调曲线本就自带输出码映射，再叠按 Quattro 参考图标定的 align_look
 * 等于二次塑形（实测 9 张经典 Foveon 平均多偏亮约 0.8 级）。SD1 早已跳过，
 * v2 生效的 SD14 同样跳过；classic 原生链连输出码重映射一起旁路（那也不是
 * x3fuse 原生的东西）。非 classic 机型路径不变。 */
static int classic_skip_align_look(void)
{
    return classic_native_full() || g_classic_sd1 || classic_tone_v2_enabled();
}

/* ---------- Classic v2 影调曲线（9 张 SD14 pooled） ---------- */
/* ---------- Classic v2 高光肩逐图微调（2026-11-04 v3） ----------
 *
 * v2 表是 9 张 pooled 出来的**单一**曲线，把逐图差异抹平了：同一个线性
 * 输入，9 张相机的显示值散开约 19 级（SDIM0092 最低、SDIM7908 最高）——
 * 这就是「这张图高光过曝、那张图高光发闷」的来源。
 *
 * 相机给每张图记了一组 auto-tone 滑杆（DBG_AutoHilite / AutoContrast）。
 * 9 张实测回归（R² 0.83…0.97）：两项系数同号且随显示值单调增，合成量
 * (hilite + contrast) 与「单独压/提高光后 hotMAE 的增减」单调对应。
 *
 * **窗口只在真正的高光段**（显示线性 0.55…0.88 ≈ 8bit 200…245）。
 * 2026-11-04 第一版用 smoothstep(0.25, 0.75) 覆盖到 8bit 137，等于给每张
 * 图套了一条不同的曲线，中间调和肤色全被改，被判定为「颜色色阶都变了」，
 * 已整体回退。这次把起效点抬到 8bit ~200，以下**逐字节不变**。
 *
 * A/B：X3F_CL_TRIM=0 关闭。 */
#define CL_TRIM_LO 0.55
#define CL_TRIM_HI 0.88

static double g_cl_trim_k = -1.0, g_cl_trim_lo = CL_TRIM_LO, g_cl_trim_hi = CL_TRIM_HI;

static void cl_trim_init(void)
{
    const char *e;
    if (g_cl_trim_k > -0.5) return;
    g_cl_trim_k = 0.18;
    e = getenv("X3F_CL_TRIM");
    if (e != NULL && *e != '\0') g_cl_trim_k = atof(e);
    e = getenv("X3F_CL_TRIM_LO");
    if (e != NULL && *e != '\0') g_cl_trim_lo = atof(e);
    e = getenv("X3F_CL_TRIM_HI");
    if (e != NULL && *e != '\0') g_cl_trim_hi = atof(e);
}

static double classic_tone_v2_curve(double lin)
{
    double u = lin * pow(2.0, CLASSIC_V2_EX_SCALE * g_classic_exposure);
    double q, pos;
    int i;
    double f, out;
    double k, lo, hi;
    if (u <= 0.0) return 0.0;
    cl_trim_init();
    k = g_cl_trim_k;
    lo = g_cl_trim_lo;
    hi = g_cl_trim_hi;
    q = u / CLASSIC_V2_HEADROOM;
    if (q >= 1.0) out = kClassicToneV2[CLASSIC_TONE_V2_N - 1];
    else {
        q = pow(q, CLASSIC_V2_GAMMA);
        pos = q * (CLASSIC_TONE_V2_N - 1);
        i = (int)pos;
        if (i > CLASSIC_TONE_V2_N - 2) i = CLASSIC_TONE_V2_N - 2;
        f = pos - i;
        out = kClassicToneV2[i] + f * (kClassicToneV2[i + 1] - kClassicToneV2[i]);
    }
    if (k != 0.0) {
        double s = g_classic_hilite + g_classic_contrast;
        double t;
        if (hi <= lo) hi = lo + 0.01;
        t = (out - lo) / (hi - lo);
        if (t > 0.0) {
            if (t > 1.0) t = 1.0;
            out += k * s * (t * t * (3.0 - 2.0 * t));
            if (out < 0.0) out = 0.0;
            else if (out > 1.0) out = 1.0;
        }
    }
    return out;
}

/* ---------- SIGMA SD1（原版 46MP 机身，非 SD1 Merrill）相机影调曲线 ----------
 *
 * SD1 与 SD1 Merrill 同代传感器，但内嵌 JPEG 走的是**老一代**引擎：和 SD14
 * 一样带 S 曲线，而通用渲染路径（matrix→ev→纯 sRGB gamma）出来发灰、高光闷
 * （对相机 JPEG 的线性拟合斜率只有 ~0.6）。与 SD14 不同的是它**没有**
 * DBG_Auto* 滑杆块（查过的文件里 TCColorModeSettings 全是同一组默认值），
 * 所以整张观感就是一条固定曲线。
 *
 * 测法与 SD14 base 相同：camera_linear = T(our_linear)，10 张 SD1 对各自内嵌
 * JPEG 取逐箱 median 再跨文件 median（黑电平修复之后，见 process.rs 里
 * get_black_level 的 "SIGMA SD1" 条目）。跨文件一致性很紧（0.03..0.5 线性域
 * cv 中位 3.6%、p90 5.1%），单条固定曲线成立。形态：暗部压深
 * （T(0.016)≈0.015）、中间调 +1.2EV（T(0.125)=0.289）、肩部在 our_lin≈0.72
 * 触顶 —— 相机整体跑亮约 0.5EV、裁剪像素是我们的 2~3 倍，our_lin>0.72 的
 * 数据全被裁剪删掉，平尾就是相机自身的硬裁剪。
 *
 * 表与 process.rs 的 CLASSIC_TONE_SD1_BASE 同源（拟合脚本同一份），改一处要
 * 同步另一处。门控 = CAMMODEL "SIGMA SD1"；X3F_CLASSIC_TONE=0 同样能关。 */
static const double kClassicToneSd1[CLASSIC_TONE_N] = {
    0.000000, 0.014876, 0.049081, 0.095053, 0.139507, 0.180082, 0.218561, 0.254535,
    0.289078, 0.319809, 0.348545, 0.377784, 0.407257, 0.437447, 0.463528, 0.491859,
    0.520210, 0.543910, 0.566746, 0.587931, 0.609403, 0.631030, 0.656286, 0.685289,
    0.704781, 0.723408, 0.736885, 0.754659, 0.776826, 0.798804, 0.822157, 0.840844,
    0.858777, 0.872094, 0.883193, 0.895471, 0.908092, 0.917778, 0.924786, 0.931670,
    0.941600, 0.949513, 0.958343, 0.966215, 0.971343, 0.976672, 0.979566, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000,
};

static double classic_tone_sd1_curve(double x)
{
    double out = classic_tone_lookup(kClassicToneSd1, x);
    if (out < 0.0) out = 0.0;
    else if (out > 1.0) out = 1.0;
    return out;
}

/* ---------------------------------------------------------------- */
/* 高光去饱和（"相机同款"）
 *
 * 相机在接近白点处会把颜色往中性收。蓝天+薄云这类场景，RAW 里各通道到顶
 * 的时刻不同，直接渲染会在接近纯白处留下品红/青色斑块；相机 JPEG 看不到
 * 这些，正是因为它在高光处做了去饱和。我们补上这一步，让天空干净、自然。
 *
 * 做法：显示空间里，用**最低通道** m 决定权重 w，把像素朝自身的亮度收缩
 * （亮度不变，只掉饱和度）。用最低通道而不是亮度，是为了只处理"三通道都
 * 接近白"的真高光：亮黄色（254,245,0）这类纯色 m 很低，不会被误伤。
 * w 随 m 从 knee 到 1.0 用 smoothstep 升起，所以中低亮度完全不受影响。
 * strength=0 关闭。
 */
/* 0..1，**默认关闭**：高光收色已经挪到传感器域做了（见 load_common_scene 里
 * "高光收色：在传感器域做"那段）。显示域这一步是在信息已经丢掉的层次上
 * 做启发式补救——补不干净，还会在边界上框出硬边。保留开关只为 A/B。 */
/* 0..1，**默认关闭**。
 *
 * 高光这件事最终是用 x3fuse 自带的高光重建解决的（BMT/传感器域，
 * 见 x3f_scene_linear 的 highlight_recovery 参数）——实测亮部偏黄从
 * −0.0104 降到 −0.0062，相机是 −0.0050，基本追平；而打开重建之后，
 * 显示域这一步开/关**结果完全相同**（−0.0062），说明它是多余的。
 *
 * 保留开关和下面那些阈值只为 A/B：如果哪天关掉重建，这一层还能顶一部分
 * （实测 −0.0293 → −0.0104）。但它是显示域的补救，会漏、会在边界上留边。 */
static double g_hl_desat = 0.0;

/* 线性域那把细节已经做过了（apply_ex_common 置位）→ 显示域那把跳过，
 * 免得同一个滑块被贴两次。 */
static int g_detail_done_linear = 0;
#define X3F_HL_DESAT_KNEE 0.80           /* 显示亮度超过此值才开始收色 */
/* RAW 驱动时的触发区间（饱和分 0..3）：一个通道顶基本不动，两个以上全收。
 *
 * 原来是 0.90/1.90。DP3M2109（镀铬高光）实测那条黄带上，三通道饱和比是
 * (R 0.68, G 0.958, B 0.983) —— G/B 都快顶了，但因为 clip_score 的门槛
 * 是 0.97，G 差 0.012 没被计入，饱和分只有 0.41，去饱和几乎没启动；
 * 而 R 的通道增益 2.7 把线性值放大到 2.55、B 变负 → 一条饱和的黄带
 * （相机直出没有）。门槛改成 0.90、区间收到 0.40/1.20 之后该处正常收色，
 * 黄带消失，与相机一致。
 *
 * 为什么要这么低：Merrill 各通道增益差得大（这里 R 是 2.7），"快顶但没顶"
 * 的组合在镀铬/镜面高光上很常见，按"必须 ≥0.97 才算"会整片漏掉。
 * 单通道饱和的纯色物体仍然只算 1 个通道，不受影响。 */
#define X3F_HL_DESAT_CLIP_LO 0.40
#define X3F_HL_DESAT_CLIP_HI 1.20

void x3f_render_set_hl_desat(double strength)
{
    if (strength < 0.0) strength = 0.0;
    if (strength > 1.0) strength = 1.0;
    g_hl_desat = strength;
}

double x3f_render_hl_desat(void) { return g_hl_desat; }

/* ---- SDQ / SDQH 高光归一收色（2026-10-31；默认 0 = 关）-------------------
 *
 * 起因：SDQH0184 水面日光反光点在 RAW 里**根本没截断**（顶层峰值 9351/16383
 * = 0.57，全图连一块截断平台都没有），我们出图却是纯黄 (251,251,0)，而相机
 * 内嵌 JPEG 与 SPP 同点纯白。
 *
 * 机理：SDQH 的 CAMF `DigitalISOGain = (4,4,2)`（DP-Q 系是 1,1,1）。解码器按
 * 上游 x3fuse 的契约把它当作**列缩放**折进相机矩阵，于是这里反解出来的通道
 * 饱和比 sat 是「相对本机通道白点」，不是「相对统一传感器白电平」：该点
 * sat = (0.448, 0.418, 0.544)，最大的才 0.544 —— 连 clip_score 的 0.90 门槛
 * 都够不到，于是整幅 25.6M 像素 clip 最大 0.544，**高光收色一次都没触发**。
 * 而显示域线性算出 (1.762, 1.942, -0.743)，蓝被夹到 0 → 黄。同一塌陷的
 * 镜像（绿先塌）就是这张图里那些粉点。
 *
 * strength > 0 时做两件事：
 *   ① clip 评分前把 sat 乘回 DigitalISOGain，换算成统一白电平下的饱和比
 *      （SDQH 该点 → (1.80, 1.67, 1.09)，这才叫"真的顶了"）；
 *   ② 用这个强度驱动高光收色，与 g_hl_desat 相乘叠加（互不覆盖）。
 *
 * dig == (1,1,1) 的文件乘 1.0 精确无变化 ⇒ DP-Q / Merrill 零回归；
 * strength == 0 时整条支路不启用 ⇒ 未开箱行为逐位不变。 */
/* sstep01 定义在下面（约 508 行），这里的补收 helper 要用到 → 前置声明。 */
static inline double sstep01(double a, double b, double x);

static int g_sdq_neut = -1;   /* X3F_SDQ_NEUT，-1 = 未读 */
static double g_sdq_hl = 0.0;
/* 场景判据开关（2026-11-14）：只给「引擎前置」两段式的第二段用。
 *
 * 不挂胶片时 SDQ 作用在 C 自己的显示链上：基础曲线 + 高光肩部已经把过曝
 * 推成近白，所以「显示域够亮 + 色差小」这两条判据天然成立。挂上胶片后，
 * 显影由引擎承担，引擎自带高光滚降把同一块天空压到 Y 0.5~0.75 —— 显示域
 * 的亮度判据在原处失效（实测 yy=0.69 时权重只有 0.4，滑杆到 2 也只到
 * 0.79），而引擎还会给中性白拉出 0.2~0.5 的色差，正落进「浓彩色，别动」
 * 的排除区。于是同一个滑杆，不挂胶片收得白、挂上胶片收不动。
 *
 * 打开后额外读解码段的 RAW 逐通道饱和分（clip）：cs ≥ 2.4（三通道全饱和）
 * 直接满权重 —— RAW 已经说「这是白」，显示域说什么都不重要。cs 1.6~2.4
 * 渐入；单通道饱和的纯色物体（夕阳 / 橙反光，cs ≈ 1）不受影响，与第一段
 * hl_desat 的 clip 判据同一口径。默认关 ⇒ 无胶片路径逐位零回归。 */
static int g_sdq_scene = 0;

void x3f_render_set_sdq_scene_judge(int on) { g_sdq_scene = on ? 1 : 0; }
static double g_sdq_dig[3] = { 1.0, 1.0, 1.0 };

void x3f_render_set_sdq_hl(double strength)
{
    if (strength < 0.0) strength = 0.0;
    /* 上限 2.0（2026-11-13）：胶片引擎（Fotufilm/SpektraLab 两段式）在
     * 引擎显影里自带高光滚降，滚降后的高光亮度落在 Y 0.55~0.80 那段，
     * 收色权重只有 0.2~0.8 —— 滑杆到 1 也收不白。允许 >1 就是让这段
     * 权重补到 1.0；实际权重在应用处再夹到 1（不许过冲反色）。 */
    if (strength > 2.0) strength = 2.0;
    g_sdq_hl = strength;
}

double x3f_render_sdq_hl(void) { return g_sdq_hl; }

/* 这个文件的 DigitalISOGain 是否偏离 1（= 是不是 SDQ/SDQH 那一代）。
 * **与滑杆无关**：clip 归一在加载时无条件做（乘 1.0 对其它机型精确无变化），
 * 这样滑杆只在 apply 那一步改强度，拖动就能实时看效果、不必重新解码。 */
static int sdq_dig_active(void)
{
    return g_sdq_dig[0] != 1.0 || g_sdq_dig[1] != 1.0 || g_sdq_dig[2] != 1.0;
}

/* 「RAW 中性、显示偏色的暗白」补收 —— 显示域版本，给不经过 apply_common 的
 * 通路（mode8 的 oe_spec_row 直写、或者 clip 拿不到的时候）用。
 * 返回 0..1 的收色权重；dig == (1,1,1) 或滑杆 0 时恒为 0（零回归）。
 *
 * 判据（详细推导见 apply_row 里同名注释）：亮度够高（Y 0.58 起、0.84 全量）
 * 且不是浓彩色（色差 < 0.40 起、0.55 以上排除）。要收的正是「B 先掉下来」
 * 的淡黄，它的 min 天生就低，所以不能用 min 作门槛；而夕阳 / 灯光 / 橙色
 * 反光的色差都在 0.5 以上，正好被色差项挡掉。 */
/* SDQ 收色权重（唯一的形状定义，apply_row 与 oe_spec_row 共用）。
 *
 * 亮度门槛 Y 0.58 起、0.84 全量；色差 dmx-dmn 0.40 起收、0.55 以上排除。
 * 强度 s ∈ [0,2]（滑杆 2026-11-13 起可到 2）：
 *   · s ≤ 1 —— 原行为，权重 = 形状 × s；s=1 与 2026-10-31 版**逐位相同**。
 *   · s > 1 —— 胶片引擎（Fotufilm / SpektraLab 两段式）在显影里自带高光
 *     滚降，过曝天空被压到 Y 0.55~0.75，形状项只剩 0.2~0.6，滑杆到 1
 *     也收不白。这一档除了把权重推满，还把两个门槛一起下移
 *     （s=2 时 0.48 / 0.70），让滚降后的高光也进收色区。
 *     权重最后夹到 1：w>1 会把 B 推过目标 R，黄反成蓝。 */
static double sdq_weight_at(double yy, double chroma, double s)
{
    double y0 = 0.58, y1 = 0.84, k = s, w;
    if (s > 1.0) {
        double d = s - 1.0;
        y0 = 0.58 - 0.10 * d;
        y1 = 0.84 - 0.14 * d;
        k = 1.0;
    }
    w = sstep01(y0, y1, yy) * (1.0 - sstep01(0.40, 0.55, chroma)) * k;
    return w > 1.0 ? 1.0 : w;
}

/* 显示域权重 + 场景（RAW 饱和分）判据的并集。
 * cs < 0 = 没有 clip 信息，行为与 sdq_weight_at 完全一致。 */
static double sdq_weight_scene(double yy, double chroma, double s, double cs)
{
    double w = sdq_weight_at(yy, chroma, s);
    if (g_sdq_scene && cs >= 0.0) {
        double ws = sstep01(1.6, 2.4, cs);
        if (ws > w) w = ws;
    }
    return w;
}

static double sdq_neut_w(double d0, double d1, double d2)
{
    double yy, dmn, dmx;
    if (g_sdq_neut < 0) {
        const char *e = getenv("X3F_SDQ_NEUT");
        g_sdq_neut = (e == NULL || atoi(e) != 0) ? 1 : 0;
    }
    if (!g_sdq_neut || g_sdq_hl <= 0.0 || !sdq_dig_active()) return 0.0;
    yy = 0.2126 * d0 + 0.7152 * d1 + 0.0722 * d2;
    dmn = dmx = d0;
    if (d1 < dmn) dmn = d1;
    if (d1 > dmx) dmx = d1;
    if (d2 < dmn) dmn = d2;
    if (d2 > dmx) dmx = d2;
    (void)dmn;
    return sdq_weight_at(yy, dmx - dmn, g_sdq_hl);
}

/* 读本文件的 DigitalISOGain（3 个 double；缺条目 / 非法值 → 保持 1,1,1）。
 * 渲染层自己读 CAMF：这个量只在 clip 评分那一步用得上，没必要穿过 dispatch
 * 的 ABI（那边的 own_decoder.c 正被另一个会话改着）。 */
static void sdq_load_dig(const char *path)
{
    double g[3] = { 1.0, 1.0, 1.0 };
    unsigned d[X3F_CAMF_DIMS] = { 1, 1, 1 };
    int i;
    g_sdq_dig[0] = g_sdq_dig[1] = g_sdq_dig[2] = 1.0;
    /* A/B 用环境变量（与 X3F_HL_DESAT 同一套），App 走 setter。 */
    {
        const char *e = getenv("X3F_SDQ_HL");
        if (e != NULL && *e != '\0') x3f_render_set_sdq_hl(atof(e));
    }
    if (path == NULL) return;
    if (x3f_camf_matrix_f64(path, "DigitalISOGain", g, 3, d) != 3) return;
    for (i = 0; i < 3; i++) {
        if (g[i] != g[i]) return;                 /* NaN */
        if (g[i] <= 0.0 || g[i] > 64.0) return;   /* 不合理的增益 */
    }
    memcpy(g_sdq_dig, g, sizeof g_sdq_dig);
    if (getenv("X3F_SDQ_DEBUG"))
        fprintf(stderr, "[sdq] hl=%.3f dig=%.3f/%.3f/%.3f\n",
                g_sdq_hl, g_sdq_dig[0], g_sdq_dig[1], g_sdq_dig[2]);
}

/* 纯元数据探测：这个文件的 DigitalISOGain 是否 ≠ (1,1,1)，即 SDQ / SDQH 那一代
 * —— 也就是「SDQ 高光归一」唯一会生效的机型。不解码、不动全局状态；
 * 给 App 面板决定要不要把滑杆摆出来（非该代摆出来也是白摆）。 */
int x3f_render_probe_sdq_hl(const char *path)
{
    double g[3] = { 1.0, 1.0, 1.0 };
    unsigned d[X3F_CAMF_DIMS] = { 1, 1, 1 };
    int i;
    if (path == NULL) return 0;
    if (x3f_camf_matrix_f64(path, "DigitalISOGain", g, 3, d) != 3) return 0;
    for (i = 0; i < 3; i++) {
        if (g[i] != g[i] || g[i] <= 0.0 || g[i] > 64.0) return 0;
        if (g[i] != 1.0) return 1;
    }
    return 0;
}

static inline double sstep01(double a, double b, double x)
{
    double t;
    if (b <= a) return x >= b ? 1.0 : 0.0;
    t = (x - a) / (b - a);
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;
    return t * t * (3.0 - 2.0 * t);
}

/* ---------------------------------------------------------------- */
/* Merrill 机型的线性色彩修正
 *
 * 为什么需要它（详见 outputs/Merrill_色彩问题分析.md）：
 *   Merrill（DP1M/DP2M/DP3M）这条解码链里，电平归一化用的 white[] 是按白平衡
 *   增益推出来的（white ∝ gain），而归一化之后色彩矩阵里又乘了一次 diag(gain)
 *   ——两次相消，等于**没有应用白平衡**；彩色矩阵 AutoCCMatrix 又是保中性的，
 *   于是残余的通道失衡被放大成明显偏色（整幅偏绿、高光泛黄）。
 *   Quattro 的增益接近 1，同样的 bug 只造成 ≤16% 失衡，所以看不出来。
 *
 * 做法：不动解码，只在线性 sRGB 域左乘一个 3×3——等价于给 conv 再乘一层。
 *
 * 拟合与验证（相机内嵌 JPEG 当标尺，显示域平均绝对差，单位 /255）：
 *   拟合：DP3M2111  24.99 → 10.08（-60%）
 *   留一：DP3M2108  24.74 →  9.70（-61%）
 *         DP3M2119  24.45 → 10.50（-57%）
 *         DP3M1884  47.57 →  9.79（-79%，另一场拍摄/另一种光源）
 *   拟合方法：把「归一化 RAW → 本文件的线性数据」当模型，
 *   用相机直出当目标，坐标下降解 9 个系数（见 tools/lab/merrill_color_check.py 的
 *   同族脚本；那批脚本在 /tmp 里跑的，方法写进了上面的分析文档）。
 *
 * 只对 Merrill 生效：Quattro 一行不动（输出逐字节不变），机型判定用 RAW 的
 * type_format == X3F_IMAGE_RAW_MERRILL。关掉：x3f_render_set_merrill_fix(0)。 */

/* Merrill 显示域逐通道曲线（33 锚点，0..255 均匀，线性插值）。
 *
 * 线性 3×3（kMerrillFix）修掉的是"整幅偏绿"那部分；剩下的残差是**随影调变化**的：
 * 中间调我们偏红、高光（尤其蓝通道）偏暗——线性矩阵管不了这种误差。
 * 这三条曲线就是冲着它去的：显示值直接映射到相机直出的显示值。
 *
 * 拟合：两张 Merrill（DP3M2111 / DP3M2108）的"我们出图 vs 相机内嵌 JPEG"
 * 逐像素配对，按 33 个亮度箱取中位数、单调化、轻度平滑（脚本见
 * outputs/Merrill_色彩问题分析.md 记录的方法）。
 * 效果：相机中性处 R/G 1.052 → 1.008（相机 0.955）、B/G 0.975 → 0.984（相机 0.984）；
 *       显示域平均绝对差 9.96 → 7.x/255。只对 Merrill 生效（按文件传参）。 */



/* ---------------------------------------------------------------- */
/* 相机色彩匹配
 *
 * 我们的 RAW→RGB + 色调曲线出来的颜色总体比相机内嵌 JPEG **偏淡**，
 * 肤色最明显（R 低约 8%、B 高约 20%），其他有色区域 4~9%。
 * 影调是准的（全图逐通道差 ~1），差的是色彩浓度。
 *
 * 做法：在显示空间把色度 (Cb, Cr) 按**色相**做一个小 2x2 线性变换
 * （等于"按色相决定饱和度缩放 + 少量色相旋转"），亮度 Y 不动。
 * 12 个色相箱的系数是对 6 张不同题材样片（人物/花草/蓝天/海港/红衣）
 * 用「我们的渲染 vs 相机内嵌 JPEG」逐像素最小二乘拟合出来的，
 * 留一图验证：色度平均误差 4.62 → 3.70（-20%，6 张全部改善）。
 *
 * 低色度处按比例淡入（|C| < 8/255 基本不动），避免给中性色上色。
 * 系数只跟色相有关，所以运行时用一张 (Cb,Cr) 二维查表 + 双线性，不需要 atan2。
 */
static int g_color_match = 1;            /* 默认开启 */

void x3f_render_set_color_match(int enable) { g_color_match = enable ? 1 : 0; }
int  x3f_render_color_match_enabled(void)   { return g_color_match; }

/* Quattro 绿增强（2026-10-30）：
 *   · g_cm_quattro      —— 当前文件是否 Quattro 世代（load 阶段用
 *     x3f_render_probe_is_quattro 探测，与 g_classic_active 同套路）。
 *     仅 Quattro 才对绿家族（Cb/Cr 空间的 box 0/1/2）施饱和增强；
 *     Merrill / Classic 不动，不影响它们已交付的严格对齐。
 *   · g_cm_q_lut        —— build_color_match_lut 建表时对应的机型（-1=未建）。
 *     色彩匹配表是全局缓存（拖强度滑块每帧复用），切机型必须重建，
 *     否则会把 Quattro 的绿增强串到 Merrill 上。 */
static int g_cm_quattro  = 0;
static int g_cm_q_lut    = -1;

/* 「过曝矫正」mode 4（产品「关」档）的机型门（2026-11-05）：
 * mode 4 的实际动作是 SPP Stage2 高光中性化 —— kernel 挂在 Quattro 引擎的
 * Stage2、判据来自 own 解码产出的裁剪 mask。**非 Quattro 没有这一段**：
 * SD14/SD1 走 fuse，Merrill 也没有 Stage2。套上去只会把正常曝光的高光
 * （WB 域 alpha 0.49 起、0.67 就全量抹平）当裁剪区拉成中性白 → 人脸高光
 * 失色。故用 Quattro 世代探针（含 SDQ/SDQH）把 mode 4 限定在 Quattro 上，
 * 其余机型按真「关」处理；mode 1/2/3/5/6/7/8/9 是实验/产品档，不受此门
 * 影响。探测与 g_cm_quattro 同套路（load 前赋值，per-load）。 */
static int g_oe_quattro_gen = 0;

/* 「皮肤卡粉」高频软压缩的机型门（2026-11-05）：
 * 同场景对相机内嵌 JPEG 逐区域量 σ1.0 亮度高频（我们 / 相机）：
 *     SD14   皮肤 1.89、额头 1.37、树叶 1.30~1.40、头发 1.49
 *     Merrill（DP1M/DP2M/SD1/SD1M）全图 0.75~0.78
 *     Quattro（DP2Q2501）全图 0.65
 * 即**只有经典 Foveon 相对相机 JPEG 偏锐**；Merrill/Quattro 本来就比相机软，
 * 对它们施压等于雪上加霜。故软压缩只对「非 Merrill 且非 Quattro 世代」生效。
 * 探测与 g_oe_quattro_gen 同套路（load 前赋值，per-load）。 */
static int g_hf_classic = 0;

#define X3F_CM_GREEN_BOOST 1.06          /* Quattro 绿箱整体系数（×1.06 ≈ 让绿落到内嵌 JPEG 水平，中性不变） */

/* 彩色阴影 / 暗角校正（spatial gain）——默认开启。
 *
 * 上游 x3f_tools 对**非 Quattro** 机型默认施加（命令行开关是 `-sgain`，
 * 帮助里写明 "default except for Quattro"），标定表就在 CAMF 里。
 * 我们这条渲染路线走的是 `x3f_get_image(..., NONE, ...)`，而 sgain 是在
 * 上游的 convert_data() 里乘上去的——NONE 会跳过 convert_data，于是这一步
 * 一直缺失。表现：偏色随画面位置变化（DP3M2108 左上 R/G 0.83 → 右下 0.90）。
 * Quattro 默认不施加（上游也是），所以开着这个开关 Quattro 的输出不受影响。 */
static int g_sgain = 1;

void x3f_render_set_spatial_gain(int enable) { g_sgain = enable ? 1 : 0; }
int  x3f_render_spatial_gain_enabled(void)   { return g_sgain; }

/* 每个色相箱： Cb' = m0*Cb + m1*Cr,  Cr' = m2*Cb + m3*Cr */
static const double kColorMatch[12][4] = {
    { +0.93200, -0.29847, -0.01021, +0.96533 },   /* -165° 青 */
    { +0.89074, -0.14921, -0.05769, +0.90501 },   /* -135° */
    { +0.91028, -0.06980, -0.06567, +0.92204 },   /* -105° 蓝 */
    { +1.01915, -0.06010, -0.05552, +0.92649 },   /*  -75° */
    { +1.02786, -0.17477, -0.04492, +0.94721 },   /*  -45° 青蓝 */
    { +0.87701, -0.26981, -0.04516, +0.88230 },   /*  -15° */
    { +0.82620, -0.10436, -0.03723, +0.82939 },   /*  +15° 红 */
    { +0.90258, +0.01651, -0.00823, +0.91064 },   /*  +45° */
    { +0.89772, -0.00749, -0.04293, +0.96135 },   /*  +75° 黄 */
    { +0.93302, +0.00530, -0.13955, +0.95481 },   /* +105° */
    { +1.02965, -0.05574, -0.12843, +1.05872 },   /* +135° 绿 */
    { +1.00120, -0.25333, -0.02325, +1.10479 },   /* +165° */
};

#define X3F_CM_N 65
#define X3F_CM_R 0.6
static float g_cm_cb[X3F_CM_N * X3F_CM_N];
static float g_cm_cr[X3F_CM_N * X3F_CM_N];
static int   g_cm_built = 0;

/* 建 LUT 时用的是哪一份「色彩模式」表 —— 换了就得重建（拖强度滑块时每帧都变）。
 * 只比多轴表那 42 个数：矩阵那一段不走这张 LUT（它在 apply_row 里直接乘）。 */
static x3f_color_mode_t g_cm_mode;
static int g_cm_mode_valid = 0;

static int cm_mode_same(const x3f_color_mode_t *a, const x3f_color_mode_t *b)
{
    int i;
    if (a == NULL || b == NULL) return a == b;
    if (a->axis != b->axis) return 0;
    if (!a->axis) return 1;
    for (i = 0; i < X3F_COLOR_MODE_AXIS; i++)
        if (a->hue_shift[i] != b->hue_shift[i] ||
            a->sat_gain[i] != b->sat_gain[i]) return 0;
    return 1;
}

/* 色彩模式（SPP 那根强度滑块 + 下拉）的多轴表：在 (Cb,Cr) 平面上按**色相**
 * 做偏移 + 饱和度缩放。表是 21 个等分格（每格 360/21 ≈ 17.1°），环形线性插值。
 *
 * 这里与 对应关系： MultiAxisCC 是拿 里的正弦/余弦 +
 * 6 个常量把每个格子的（角度, 倍率）展开成一个逐格 2x2，再按强度线性缩放。
 * 我们这一步只需要"偏移多少度 + 缩放多少倍"这两个量，效果等价。
 *
 * **符号约定**：正的角度 = 在 (Cb,Cr) 平面上逆时针转。SPP 那边的正负没对拍过
 * （角度只有 ±6°，肉眼看不出方向），将来有参考图再校。 */
static void cm_axis_apply(const x3f_color_mode_t *cm, double cb, double cr,
                          double *ob, double *or_)
{
    double ang, t, f, sh, sg, rad, ca, sa, x, y;
    int a0, a1;

    *ob = cb; *or_ = cr;
    if (cm == NULL || !cm->axis) return;

    ang = atan2(cr, cb);                      /* -pi .. pi */
    t = (ang + 3.14159265358979323846)
      * ((double)X3F_COLOR_MODE_AXIS / 6.28318530717958647692);
    a0 = (int)t;
    f = t - a0;
    if (a0 < 0) { a0 = 0; f = 0.0; }
    if (a0 >= X3F_COLOR_MODE_AXIS) { a0 = X3F_COLOR_MODE_AXIS - 1; f = 0.0; }
    a1 = (a0 + 1) % X3F_COLOR_MODE_AXIS;
    sh = cm->hue_shift[a0] + (cm->hue_shift[a1] - cm->hue_shift[a0]) * f;
    sg = cm->sat_gain[a0] + (cm->sat_gain[a1] - cm->sat_gain[a0]) * f;
    if (sh == 0.0 && sg == 1.0) return;

    rad = sh * 3.14159265358979323846 / 180.0;
    ca = cos(rad); sa = sin(rad);
    x = cb * ca - cr * sa;
    y = cb * sa + cr * ca;
    *ob = x * sg;
    *or_ = y * sg;
}

static void build_color_match_lut(const x3f_color_mode_t *cm)
{
    int i, j, b;
    double CM[12][4];                            /* 当前机型生效的 12 格色相表 */
    /* 先把基准表拷进可写副本；Quattro 再对绿家族 box 0/1/2 整体 ×X3F_CM_GREEN_BOOST。
     * 这些箱是过原点的线性映射，整体系数缩放 = 绿家族饱和度提升、中性点不动、
     * 其他色相箱（红/蓝/黄…）完全不受影响。 */
    for (b = 0; b < 12; b++) {
        CM[b][0] = kColorMatch[b][0];
        CM[b][1] = kColorMatch[b][1];
        CM[b][2] = kColorMatch[b][2];
        CM[b][3] = kColorMatch[b][3];
    }
    if (g_cm_quattro) {
        /* 绿家族落在 Cb/Cr 空间的 box 0/1/2（-165°/-135°/-105°，黄绿→绿→青绿）。
         * 注意：kColorMatch 表里 "+135° 绿" 是误标——纯绿实测映射到 box 1，
         * 真实绿箱在 box 0..2 一带；box 10(+135°) 实际是橙/红橙，不能动。
         * 对这三个箱整体 ×X3F_CM_GREEN_BOOST = 绿家族饱和度提升、中性点不动、
         * 红/蓝/黄不受影响。 */
        int gi;
        for (gi = 0; gi <= 2; gi++) {
            CM[gi][0] *= X3F_CM_GREEN_BOOST;
            CM[gi][1] *= X3F_CM_GREEN_BOOST;
            CM[gi][2] *= X3F_CM_GREEN_BOOST;
            CM[gi][3] *= X3F_CM_GREEN_BOOST;
        }
    }
    for (j = 0; j < X3F_CM_N; j++) {
        double cr = -X3F_CM_R + 2.0 * X3F_CM_R * j / (X3F_CM_N - 1);
        for (i = 0; i < X3F_CM_N; i++) {
            double cb = -X3F_CM_R + 2.0 * X3F_CM_R * i / (X3F_CM_N - 1);
            double mb, mr;                         /* 过完色彩模式多轴表之后的值 */
            double ang, t, f, cb0, cr0, cb1, cr1, ncb, ncr, s, w;
            int i0, i1;

            /*① 色彩模式（多轴表）：先把色度按色相偏移 + 缩放改成 mb/mr。
             * 近中性的像素本身色度就小，这一步在数学上近似无操作。 */
            cm_axis_apply(cm, cb, cr, &mb, &mr);

            /* ② 相机色彩匹配（我们手工拟合的那张 12 格表），作用在 ① 的结果上 */
            ang = atan2(mr, mb);                   /* -pi .. pi */
            t = (ang + 3.14159265358979323846) * (12.0 / 6.28318530717958647692);
            i0 = (int)t;
            f = t - i0;
            if (i0 < 0) { i0 = 0; f = 0.0; }
            if (i0 > 11) { i0 = 11; f = 0.0; }
            i1 = (i0 + 1) % 12;
            cb0 = CM[i0][0] * mb + CM[i0][1] * mr;
            cr0 = CM[i0][2] * mb + CM[i0][3] * mr;
            cb1 = CM[i1][0] * mb + CM[i1][1] * mr;
            cr1 = CM[i1][2] * mb + CM[i1][3] * mr;
            ncb = cb0 + (cb1 - cb0) * f;
            ncr = cr0 + (cr1 - cr0) * f;
            s = sqrt(mb * mb + mr * mr);
            w = s / (8.0 / 255.0);
            if (w > 1.0) w = 1.0;
            g_cm_cb[j * X3F_CM_N + i] = (float)(mb + (ncb - mb) * w);
            g_cm_cr[j * X3F_CM_N + i] = (float)(mr + (ncr - mr) * w);
        }
    }
    if (cm != NULL) { g_cm_mode = *cm; g_cm_mode_valid = 1; }
    else            { memset(&g_cm_mode, 0, sizeof(g_cm_mode)); g_cm_mode_valid = 0; }
    g_cm_q_lut = g_cm_quattro;               /* 记下来这张表是按哪种机型建的 */
    g_cm_built = 1;
}

/* ===================================================================
 * SPP「色彩模式」（Color Mode）
 *
 * 面板上那个下拉 + 它下面的强度滑块。数据与算法都是得来的，推导见
 * ；这里只记落地口径。
 *
 * 下拉（11 档，名字 = 文件 CAMF 里的短名，也是 CMCM_ 的后缀）：
 *     Standard  → 相机的"标准"。**它就是我们的基线**（基础曲线与手工拟合的
 *                 kColorMatch 都是对着相机 Standard 标定的），CMCM_Standard
 *                 实测是单位阵、多轴表差分也是 0，所以这一档整段是 no-op。
 *     其余 10 档 → 各带一套 CMCM（3x3，行和 = 1）、CMCC（对比度补偿标量）、
 *                 MultiAxisTable（2x5x21：组 0 = 21 个色相格的偏移（度）、
 *                 组 1 = 21 个色相格的饱和度倍率；D1 那 5 行在实机上完全相同）。
 *
 * 强度滑块（**全局**，Standard 也吃；说明串 12016：
 * "通过滑块操作，可以调节色彩效果的强弱"）：
 *     内部值 v ∈ [-5,+5]，默认 0；权重 w = clamp(1 + 0.2·v, 0, 2)
 * 取值表见 `AdjustColorModeEffectParam` 0xa0960（表在，
 *     逐模式 11 个数 × 网格 [-5,…,+5] 线性插值）。界面上按 0..100 显示，
 *     50 = 默认档（w = 1），0 → w = 0，100 → w = 2。
 *     （实测自真身：mode 5/8/11/12 那四档的正半段是 1+0.1·v、1.5 封顶，
 *       mode 13 是 1+0.15·v；实现里按模式名查形状，见下面那张小表。）
 *
 * 权重落在哪（同样出自）：
 *     · 色彩矩阵   M = w·CMCM_<模式> + (1−w)·N
 * N 是 rodata 里的常量矩阵（见 kColorModeNeutral）——
 *                  它把 w = 0 那端定死，所以 Standard + 强度 0 也不是"不动"，
 *                  而是变成 N：跨通道混合更强、色度更小。用户实测"强度拉到 0
 *                  时更柔和、饱和度低一点"，与 N 的行为一致。
 *     · 多轴表     色相偏移 ×w，饱和度倍率 1 + w·(倍率−1)（w = 0 → 恒等）
 * · 色调曲线 在 两组 ToneParams 之间插值（ `SetUpToneParamsF20`
 *                  的色彩模式表）。**我们没做**：我们的基础曲线不是 SPP 那条，
 *                  硬插会把标定弄坏 —— 所以我们的模式差异少了"对比度"那一层。
 *
 * 三个"作用在哪"的决定：
 *   1) 矩阵在**线性域**（apply_row 里、tone LUT 之前）—— SPP 也是把它乘在
 *      白平衡之后、色调曲线之前（`GetColorModeContrastCompensation` 0x97478
 *      把 CMCM 左乘到白平衡矩阵上：M_total = CMCM·M_wb）。
 *   2) 矩阵**不做色彩空间共轭**，直接乘在我们的线性 sRGB 上。判据是那几张表的
 * **行和恒为 1**：说明它作用在"中性 = 等通道"的域。乘序（白平衡
 *      之后）正是这样的域；我们的线性数据（cam_to_rgb 之后）也是。反过来拿
 *      cam_to_rgb 去共轭（= 假装它作用在传感器域）会让 Cinéma 这种表的行和
 *      变成 1.44，中性色直接被染色 —— 试过，错的。
 *   3) 多轴表在**显示域**，和 kColorMatch 合用同一张 (Cb,Cr) 查表 ——
 *      SPP 那一步也是显示域（MultiAxisCC 有一份符号就是 48SRGB 域）。
 * =================================================================== */

/* 取 CAMF 矩阵的分发口（定义在 x3fdec.c；签名与 x3f_get_camf_matrix_var 不同，
 * 所以不走 x3fdec_alias.h 那套宏，这里手写声明）。 */
extern int x3fdec_get_camf_matrix_var(x3f_t *x3f, const char *name,
                                      int *dim0, int *dim1, int *dim2,
                                      matrix_type_t type, double *out, int cap);

/*11 档模式名（CAMF 后缀），顺序照菜单。*/
static const char *kColorModeNames[] = {
    "Standard", "Vivid", "Neutral", "Portrait", "Landscape",
    "FCBlue", "FCYellow", "Cinema", "ForestGreen", "SunsetRed", "Monochrome"
};
#define X3F_COLOR_MODE_N ((int)(sizeof(kColorModeNames) / sizeof(kColorModeNames[0])))

/*w = 0 那一端的矩阵 N。数来自 ：0x19F0F0 / 0x19F100 / 0x19F110 三对
 * double，加上三个立即数（36501e26 / 0x3fabf0995aaf78ff /
 * 0x3feab7564302b40f），装配在 `GetColorModeContrastCompensation` 0x97478 的
 * 0x9769c–0x97750。行的和都是 1，是个正经的色彩矩阵。 */
static const double kColorModeNeutral[9] = {
     0.81661,  0.03459,  0.1537492528152466,
    -0.13675,  1.08214,  0.05457,
    -0.20383,  0.36891,  0.83488
};

static char g_color_mode_err[256];

const char *x3f_color_mode_last_error(void) { return g_color_mode_err; }

static void color_mode_err(const char *msg, const char *what)
{
    if (what != NULL)
        snprintf(g_color_mode_err, sizeof(g_color_mode_err), "%s%s", msg, what);
    else
        snprintf(g_color_mode_err, sizeof(g_color_mode_err), "%s", msg);
}

int x3f_color_mode_count(void) { return X3F_COLOR_MODE_N; }

const char *x3f_color_mode_name(int idx)
{
    if (idx < 0 || idx >= X3F_COLOR_MODE_N) return NULL;
    return kColorModeNames[idx];
}

/*强度 0..100 → 权重 w（对应 的表，见上面那段说明）。*/
static const struct { const char *name; int shape; } kColorModeWeightShape[] = {
    { "Standard",    0 },
    { "Vivid",       0 },
    { "Neutral",     1 },
    { "Portrait",    0 },
    { "Landscape",   0 },
    { "Cinema",      1 },
    { "SunsetRed",   0 },
    { "ForestGreen", 0 },
    { "FCBlue",      1 },
    { "FCYellow",    1 },
};

static int color_mode_weight_shape(const char *mode)
{
    int i;
    if (mode == NULL) return 0;
    for (i = 0; i < (int)(sizeof(kColorModeWeightShape) / sizeof(kColorModeWeightShape[0])); i++)
        if (strcmp(kColorModeWeightShape[i].name, mode) == 0)
            return kColorModeWeightShape[i].shape;
    return 0;
}

static double color_mode_weight(const char *mode, double strength)
{
    double v, w;
    if (!(strength >= 0.0)) strength = 0.0;      /* NaN 也走这条 */
    if (strength > 100.0) strength = 100.0;
    v = (strength - 50.0) / 10.0;
    if (v < -5.0) v = -5.0;
    if (v > 5.0) v = 5.0;
    /*mode 5/8/11/12 那四档的正半段斜率减半（1.5 封顶）—— 实测自*/
    w = (v > 0.0 && color_mode_weight_shape(mode) == 1) ? 1.0 + 0.1 * v
                                                        : 1.0 + 0.2 * v;
    if (w < 0.0) w = 0.0;
    if (w > 2.0) w = 2.0;
    return w;
}

/* 3x3 行主序乘法：c = a·b（不借 x3f_3x3_3x3_mul，那个的行列约定是给编码那套的） */
static void cm3_mul(const double *a, const double *b, double *c)
{
    double t[9];
    int i, j, k;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            double s = 0.0;
            for (k = 0; k < 3; k++) s += a[i * 3 + k] * b[k * 3 + j];
            t[i * 3 + j] = s;
        }
    memcpy(c, t, sizeof(t));
}

/* 色彩矩阵施加后的**色相保持收口**（gamut compression）。
 *
 * 背景：FCBlue 这类离群矩阵（离对角系数 ±2~3）乘在线性 sRGB 上会把大量自然
 * 色推到 [0,1] 之外，原代码只夹负端（<0→0），正端直接灌进下游的 mode8 拟合
 * 矩阵 oeM / 逐通道硬夹 —— 结果是绿色被推到 >1 后夹成纯白、蓝色夹成近黑，
 * 画面"塌成纯色"。SPP 在 FOV 蓝/黄 这类档是平滑的，因为它对越界值做的是沿
 * 明度方向的收口（只降饱和、保色相），而不是硬夹。
 *
 * 做法：以明度 Y 为锚，把 (RGB−Y) 的偏离整体缩放 s 倍塞回 [0,1]；s 取"让任一
 * 通道刚好不越界"的最大值且 ≤1（只降不增）。因此域内像素 / 中性灰 s=1 是精确
 * no-op，不影响 Standard/Vivid 等温和档、也不动中性色。
 * 锚 Y 自身越界（整像素已近白/近黑）时收口无解，回落硬夹（此时本就是近白/黑，
 * 不会塌成异色）。
 *
 * 环境变量 X3F_CM_GAMUT=0 退回旧的"只夹负端"行为（供 A/B 对照）。
 *
 * 2026-11-10 核查过「广色域档跳过本函数」是否可行：**不可行**。FCBlue 强度
 * 100% + ProPhoto 档下，强矩阵把线性值推到极远，0.33% 像素反解出的 sRGB
 * 线性通道 < −0.5（min −0.86），天空与火焰高光塌成大片纯蓝/纯白块。
 * 本函数是强色彩模式矩阵的防崩坏保护，不是单纯的色域裁剪；要放宽必须改成
 * 「保留负余量的宽立方体收口」，那会动到负值幅度 —— 按广色域铁律先报用户
 * 决策，不擅自改。详见当日日志。 */
static void cm_gamut_compress(double *pv0, double *pv1, double *pv2)
{
    static int init = 0;
    static int enabled = 1;
    double v0 = *pv0, v1 = *pv1, v2 = *pv2;
    double Y, s;
    if (!init) {
        const char *e = getenv("X3F_CM_GAMUT");
        if (e != NULL && (e[0] == '0' || strcmp(e, "off") == 0
                          || strcmp(e, "false") == 0 || strcmp(e, "no") == 0))
            enabled = 0;
        init = 1;
    }
    if (!enabled) {                       /* 旧行为：只夹负端 */
        if (v0 < 0.0) v0 = 0.0;
        if (v1 < 0.0) v1 = 0.0;
        if (v2 < 0.0) v2 = 0.0;
        *pv0 = v0; *pv1 = v1; *pv2 = v2;
        return;
    }
    /* 域内（全在 [0,1]）精确 no-op —— 温和档与中性色完全不受影响 */
    if (v0 >= 0.0 && v0 <= 1.0 && v1 >= 0.0 && v1 <= 1.0
        && v2 >= 0.0 && v2 <= 1.0)
        return;

    Y = 0.2126 * v0 + 0.7152 * v1 + 0.0722 * v2;
    if (Y < 0.0 || Y > 1.0) {             /* 锚越界：收口无解，硬夹 */
        if (v0 < 0.0) v0 = 0.0; if (v0 > 1.0) v0 = 1.0;
        if (v1 < 0.0) v1 = 0.0; if (v1 > 1.0) v1 = 1.0;
        if (v2 < 0.0) v2 = 0.0; if (v2 > 1.0) v2 = 1.0;
        *pv0 = v0; *pv1 = v1; *pv2 = v2;
        return;
    }
    s = 1.0;
    /* 上界（>1）通道：s ≤ (1−Y)/(v−Y)，仅当 v−Y>0 有意义 */
    if (v0 > 1.0 && (v0 - Y) > 1e-9) { double k = (1.0 - Y) / (v0 - Y); if (k < s) s = k; }
    if (v1 > 1.0 && (v1 - Y) > 1e-9) { double k = (1.0 - Y) / (v1 - Y); if (k < s) s = k; }
    if (v2 > 1.0 && (v2 - Y) > 1e-9) { double k = (1.0 - Y) / (v2 - Y); if (k < s) s = k; }
    /* 下界（<0）通道：s ≤ (0−Y)/(v−Y)，仅当 v−Y<0 有意义 */
    if (v0 < 0.0 && (v0 - Y) < -1e-9) { double k = (0.0 - Y) / (v0 - Y); if (k < s) s = k; }
    if (v1 < 0.0 && (v1 - Y) < -1e-9) { double k = (0.0 - Y) / (v1 - Y); if (k < s) s = k; }
    if (v2 < 0.0 && (v2 - Y) < -1e-9) { double k = (0.0 - Y) / (v2 - Y); if (k < s) s = k; }
    if (s > 1.0) s = 1.0;
    v0 = Y + s * (v0 - Y);
    v1 = Y + s * (v1 - Y);
    v2 = Y + s * (v2 - Y);
    if (v0 < 0.0) v0 = 0.0; if (v0 > 1.0) v0 = 1.0;
    if (v1 < 0.0) v1 = 0.0; if (v1 > 1.0) v1 = 1.0;
    if (v2 < 0.0) v2 = 0.0; if (v2 > 1.0) v2 = 1.0;
    *pv0 = v0; *pv1 = v1; *pv2 = v2;
}

static int cm3_is_unit(const double *m)
{
    static const double I[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    int i;
    for (i = 0; i < 9; i++)
        if (fabs(m[i] - I[i]) > 1e-9) return 0;
    return 1;
}

/* 表全成"什么都不做"。x3f_edit_defaults() 也走这里（保证默认 = 原行为）。 */
static void color_mode_unit(x3f_color_mode_t *o)
{
    int i;
    memset(o, 0, sizeof(*o));
    o->m[0] = o->m[4] = o->m[8] = 1.0;
    o->unit = 1;
    o->tone_row = -1;
    o->tone_w = 1.0;
    for (i = 0; i < X3F_COLOR_MODE_AXIS; i++) o->sat_gain[i] = 1.0;
}

/*---- 每档的色调曲线（表在 x3f_colormode_tone.h，相机算法跑出来的）------
 *
 * 下标按名字找家族；找不到就是 default（= 与基线相同的那一族）。
 * 只有 5 档与基线不同：标准 / 鲜艳 / 中性 / 肖像 / FOV 黄色经典。
 *
 * 表是生成文件（不入库），没有它就走下面那三个桩：tone_row 恒为 -1 =
 * 每档都按"与基线相同"处理，别的什么也不变。 */
#if X3F_HAVE_SPP_CMTONE
static int cmtone_row_for(const char *mode)
{
    int i;
    if (mode == NULL) return -1;
    for (i = 0; i < X3F_CMTONE_ROWS; i++) {
        const char *n = kCMToneRows[i].mode;
        if (strcmp(n, "default") == 0) continue;
        if (strcmp(n, mode) == 0) return i;
    }
    /* 其余各档 = 默认那一行（风景/电影院/夕阳红/森林绿/FOV 蓝/黑白）——
     * 它们在 的模式表里本来就是同一组参数。*/
    for (i = 0; i < X3F_CMTONE_ROWS; i++)
        if (strcmp(kCMToneRows[i].mode, "default") == 0) return i;
    return -1;
}

static int cmtone_row_default(void)
{
    int i;
    for (i = 0; i < X3F_CMTONE_ROWS; i++)
        if (strcmp(kCMToneRows[i].mode, "default") == 0) return i;
    return -1;
}

/* in = lin / X3F_OE_LIN_WHITE（与 kX3FOEToneGainIn 同口径，1.0 = 传感器饱和）。
 * w 在两个相邻列之间按 log 域线性插值（= 几何插值）；表外直接夹住。 */
static double cmtone_gain(int row, double w, double in)
{
    const cmtone_col_t *c;
    double e, f, a, b, t;
    int i, k;

    if (row < 0 || row >= X3F_CMTONE_ROWS) return 1.0;
    c = kCMToneRows[row].cols;
    if (in <= 0.0) return 1.0;
    e = in * (X3F_CMTONE_N - 1);
    if (e >= X3F_CMTONE_N - 1) e = X3F_CMTONE_N - 1;
    i = (int)e;
    f = e - i;
    if (i >= X3F_CMTONE_N - 1) { i = X3F_CMTONE_N - 2; f = 1.0; }
    if (w <= c[0].w) return c[0].g[i] + (c[0].g[i + 1] - c[0].g[i]) * f;
    if (w >= c[X3F_CMTONE_WCOL - 1].w)
        return c[X3F_CMTONE_WCOL - 1].g[i]
             + (c[X3F_CMTONE_WCOL - 1].g[i + 1] - c[X3F_CMTONE_WCOL - 1].g[i]) * f;
    for (k = 1; k < X3F_CMTONE_WCOL; k++) {
        if (w <= c[k].w) break;
    }
    t = (w - c[k - 1].w) / (c[k].w - c[k - 1].w);
    a = c[k - 1].g[i] + (c[k - 1].g[i + 1] - c[k - 1].g[i]) * f;
    b = c[k].g[i] + (c[k].g[i + 1] - c[k].g[i]) * f;
    if (a <= 0.0 || b <= 0.0) return 1.0;
    return exp((1.0 - t) * log(a) + t * log(b));
}
#else
/* 没有生成表（本机没跑过 tools/sppref/gen_mode_tone.py）：不乘任何曲线增益。
 * 界面和数据流都不受影响 —— x3f_color_mode_t.tone_row 恒为 -1，
 * 也就是"这一档与基线完全相同"。 */
static int cmtone_row_for(const char *mode) { (void)mode; return -1; }
static int cmtone_row_default(void) { return -1; }
static double cmtone_gain(int row, double w, double in)
{
    (void)row; (void)w; (void)in;
    return 1.0;
}
#endif

/* 这两张 SPP 派生表有没有编进来（bit0 = 色彩模式曲线，bit1 = 过曝矫正差）。
 * 宿主拿它决定界面上那几句说明怎么写：生成文件不在时，相关增益恒 1。 */
int x3f_render_has_spp_tables(void)
{
    return (X3F_HAVE_SPP_CMTONE ? 1 : 0) | (X3F_HAVE_SPP_OE_TONE ? 2 : 0);
}

/* 给宿主用的同一个入口（x3f_color_mode_t 全零 = 零矩阵，不是"不做"）。 */
/* mode8 直通用：最近一次 x3f_color_mode_resolve 读到的原始 CMCM_Standard
 * （填充点见 resolve 内；没有 Standard 表的文件保持 has=0 = mode8 不做差） */
static double g_cm_standard_m[9];
static int g_cm_standard_has = 0;

void x3f_color_mode_default(x3f_color_mode_t *out)
{
    if (out == NULL) return;
    color_mode_unit(out);
}

/* ---- 取表 ---------------------------------------------------------- */

/* 一次打开把某一档的三张表都读出来（每次 x3fdec_* 调用都要重解一遍 CAMF，
 * 所以是"一次打开 + 六次查表"）。 */
typedef struct {
    double m[9];                              int has_m;
    double axis[2 * 5 * X3F_COLOR_MODE_AXIS]; int has_axis;
    double cmcc;                              int has_cmcc;
} cm_tables_t;

static void cm_tables_read(x3f_t *x3f, const char *mode, cm_tables_t *t)
{
    char key[96];
    memset(t, 0, sizeof(*t));
    snprintf(key, sizeof(key), "CMCM_%s", mode);
    t->has_m = (x3fdec_get_camf_matrix_var(x3f, key, NULL, NULL, NULL,
                                           M_FLOAT, t->m, 9) >= 9);
    snprintf(key, sizeof(key), "MultiAxisTable_%s", mode);
    t->has_axis = (x3fdec_get_camf_matrix_var(x3f, key, NULL, NULL, NULL, M_FLOAT,
                                              t->axis,
                                              2 * 5 * X3F_COLOR_MODE_AXIS)
                   >= 2 * 5 * X3F_COLOR_MODE_AXIS);
    snprintf(key, sizeof(key), "CMCC_%s", mode);
    t->has_cmcc = (x3fdec_get_camf_matrix_var(x3f, key, NULL, NULL, NULL,
                                              M_FLOAT, &t->cmcc, 1) >= 1);
}

int x3f_color_mode_resolve(const char *path, const char *mode, double strength,
                           x3f_color_mode_t *out)
{
    char tmp[1024];
    const char *use;
    FILE *fp;
    x3f_t *x3f;
    x3f_directory_entry_t *de;
    cm_tables_t cur, std_;
    double w, M[9];
    int i, any;
    int diag = getenv("X3F_COLOR_MODE_DEBUG") != NULL;

    if (out == NULL) return X3FRENDER_ERR_PARAM;
    color_mode_unit(out);
    g_color_mode_err[0] = '\0';
    if (path == NULL || mode == NULL || mode[0] == '\0') {
        color_mode_err("x3f_color_mode_resolve: 缺少路径或模式名", NULL);
        return X3FRENDER_ERR_PARAM;
    }

    w = color_mode_weight(mode, strength);

    /* X3I(SFD) 这类容器得先落成独立 X3F 才取得到 CAMF（与 load_common 同一条路）。 */
    use = x3f_standalone_path(path, -1, tmp, sizeof(tmp));
    fp = fopen(use, "rb");
    if (fp == NULL) {
        color_mode_err("x3f_color_mode_resolve: 打不开文件 ",
                       use != path ? tmp : path);
        return X3FRENDER_ERR_OPEN;
    }
    x3f = x3fdec_new_from_file(fp);
    if (x3f == NULL) {
        fclose(fp);
        if (use != path) x3f_temp_release(tmp);
        color_mode_err("x3f_color_mode_resolve: 不是 X3F（或解析失败）：",
                       use != path ? tmp : path);
        return X3FRENDER_ERR_PARSE;
    }
    de = x3fdec_get_camf(x3f);
    if (de == NULL || x3fdec_load_data(x3f, de) != X3F_OK) {
        x3fdec_delete(x3f);
        fclose(fp);
        if (use != path) x3f_temp_release(tmp);
        color_mode_err("x3f_color_mode_resolve: 这个文件没有可用的 CAMF 段", NULL);
        return X3FRENDER_ERR_PARSE;
    }
    cm_tables_read(x3f, mode, &cur);
    if (strcmp(mode, "Standard") != 0) cm_tables_read(x3f, "Standard", &std_);
    else std_ = cur;
    x3fdec_delete(x3f);
    fclose(fp);
    if (use != path) x3f_temp_release(tmp);

    if (!cur.has_m) {
        color_mode_err("这张文件的 CAMF 里没有这一档模式的色彩矩阵（CMCM_）——"
                       "这台机身的固件不带它，或模式名拼错：", mode);
        return X3FRENDER_ERR_PARAM;
    }

    /* ① 色彩矩阵。
     * 原公式 M = w·cur.m + (1−w)·N 在 w=0 退化为 N（与 cur.m 无关）：N 的 G/B 行
     * 带负 R 系数，会把饱和红算成负→线性域夹 0→纯红死白（SDQ_9541 全模式强度 0
     * 实测 2127px 溢出；用户报"调 0 溢出、调 100 正常"）。
     * 修复（双锚点，最低效果风险）：
     *   w≤1 这一端改用单位阵锚点 M = I + w·(cur.m − I) —— w=0→I（红不再溢出），
     *        w=1→cur.m（默认档与现状逐字节一致）；
     *   w>1 维持原 N 外推 M = w·cur.m + (1−w)·N —— 即强度 100 行为完全不变
     *        （用户已确认正常），避免动到已认可的高强度端。
     *   两段在 w=1 处连续（都给出 cur.m）。
     * ▶ 回退：把下面 if/else 整段恢复成一句
     *   `for(i=0;i<9;i++) M[i]=w*cur.m[i]+(1.0-w)*kColorModeNeutral[i];` 即可。 */
    {
        static const double I9[9] = { 1,0,0, 0,1,0, 0,0,1 };
        if (w <= 1.0)
            for (i = 0; i < 9; i++)
                M[i] = I9[i] + w * (cur.m[i] - I9[i]);
        else
            for (i = 0; i < 9; i++)
                M[i] = w * cur.m[i] + (1.0 - w) * kColorModeNeutral[i];
    }
    memcpy(out->m, M, sizeof(M));
    out->unit = cm3_is_unit(out->m);

    /* mode8 直通（apply_ex_common）用：把这次读到的**原始** CMCM_Standard
     * 存起来。直通的端到端拟合吸收了 Standard 档的矩阵（拟合目标 = SPP 出图，
     * SPP 在它的线性段乘的就是这一档），所以 mode8 里非 Standard 档要按
     * M_当前 · M_Standard⁻¹ 的**差**折进线性段，Standard 档 = 恒等不乘。 */
    if (std_.has_m) {
        memcpy(g_cm_standard_m, std_.m, sizeof(g_cm_standard_m));
        g_cm_standard_has = 1;
    }

    /* ② 多轴表：取**相对 Standard 的差**（我们的基线就是 Standard）。
     *
     * **默认不启用**（axis = 0），理由是实测：拿"相机 Vivid 直出"当参照
     * （YC-28_VIVID.X3F，2712×1808，平均色度按 max(RGB)−min(RGB)）：
     *     相机 Vivid JPEG 33.37 ｜ 我们 Standard 30.67（参照差 +8.8%）
     *     只加矩阵        35.09（+14.4%）← 与参照同量级，留作默认
     *     只加多轴表      38.98（+27.1%）
     *     矩阵 + 多轴表   44.49（+45.1%）← 明显过头
     * 也就是说：多轴表那一列（组 1）当"饱和度倍率"用在我们的链上大约强了
     * 三倍。 的 MultiAxisCC 里它是经 sincos + 6 个常量展开成逐格 2x2 的
     * （ 一带），那 6 个常量的来路还没读通 —— 在没对拍清楚之前，
     * 宁可只做矩阵那一条（那一列也照旧解析出来，供诊断与后续标定用）。
     * 想要它上来试：环境变量 X3F_COLOR_MODE_AXIS=1。 */
    if (cur.has_axis && std_.has_axis) {
        any = 0;
        for (i = 0; i < X3F_COLOR_MODE_AXIS; i++) {
            double hs = cur.axis[i] - std_.axis[i];
            double s0 = std_.axis[5 * X3F_COLOR_MODE_AXIS + i];
            double s1 = cur.axis[5 * X3F_COLOR_MODE_AXIS + i];
            double sg = (s0 != 0.0) ? (s1 / s0) : 1.0;
            out->hue_shift[i] = w * hs;
            out->sat_gain[i] = 1.0 + w * (sg - 1.0);
            if (fabs(out->hue_shift[i]) > 1e-9 ||
                fabs(out->sat_gain[i] - 1.0) > 1e-9) any = 1;
        }
        out->axis = any && (getenv("X3F_COLOR_MODE_AXIS") != NULL);
        if (any && !out->axis && diag)
            fprintf(stderr, "[cm]   多轴表：解析到了差别，但按默认不启用"
                            "（X3F_COLOR_MODE_AXIS=1 可强制打开）\n");
    }

    /* ③ CMCC：读出来备用/诊断（我们的曲线不是 SPP 那条，暂时不接）。 */
    out->cmcc = cur.has_cmcc ? cur.cmcc : 0.0;
    out->cmcc_std = std_.has_cmcc ? std_.cmcc : 0.0;

    /* ④ 每档的色调曲线：家族按名字查（表里只有 5 档 + default），权重就是 w。
     * 参考是 mode 7（风景），所以 default 族（风景/电影院/夕阳红/森林绿/FOV 蓝）
     * 在 w=1 时增益恰好是 1.0 —— 那几档出图与基线逐字节相同。 */
    out->tone_row = cmtone_row_for(mode);
    out->tone_w = w;
    if (out->tone_row < 0) out->tone_row = cmtone_row_default();

    if (diag) {
        fprintf(stderr,
                "[cm] %s 强度 %.0f → w %.3f\n"
                "[cm]   矩阵   = %s\n"
                "[cm]   多轴表 = %s（CMCC %.3f，Standard %.3f）\n",
                mode, strength, w,
                out->unit ? "单位阵（不动）" : "已按强度混合（线性 sRGB 域）",
                out->axis ? "生效（相对 Standard）" : "与 Standard 相同",
                out->cmcc, out->cmcc_std);
    }
    return X3FRENDER_OK;
}

#define X3F_BASE_LOG2_MIN  (-16.0)
#define X3F_BASE_LOG2_STEP (0.5)
#define X3F_BASE_NA        (41)

/* T(2^-16) .. T(2^+4)，每 0.5 EV 一格；单调递增。
 *
 * 2026-10 重新定标（对齐 SIGMA Photo Pro 的导出）：
 *   参照：DP2Q0337 在 SPP 里 Fill Light=0 导出的 JPEG，与我们的线性数据按
 *   场景亮度分箱、比中位显示值。原表在 **中间调偏暗 14~19 个灰阶**
 *   （log2(L) = −5…−3 那一段），暗部只差 1~2 级、高光基本一致。
 *   修正是保守的：比值限制在 0.85~1.80、并按分箱样本数加权收敛，
 *   改动集中在锚点 18~27（约 0.94~1.31 倍）。
 *   顺带核对了同一张的**相机内嵌 JPEG**：SPP 与相机直出各档相差 ≤7 个灰阶，
 *   而原来那张表比两者都暗 —— 所以这次对齐不会跟"相机观感"打架。
 *
 * 高光段的原始验证记录（用 4 张高死白样片，只统计相机未切平的像素）保留：
 *   高光段 0.35~1.4 与相机实测曲线的平均差 1.25/255，最大 3.1
 *     线性 0.50 → 相机 230.0 / 本表 228.8
 *     线性 0.80 → 相机 248.2 / 本表 246.3
 *     线性 1.00 → 相机 252.3 / 本表 253.5
 *     线性 1.40 → 相机 253.8 / 本表 253.9（渐近到 ~254，不硬切）
 *   相机在 **线性 ≈1.2** 处到达白点，过渡带 1.17~1.40；本表同为渐近形状。 */
static const double kX3FBaseCurve[X3F_BASE_NA] = {
    0.000009, 0.000013, 0.000018, 0.000025,
    0.000035, 0.000050, 0.000071, 0.000100,
    0.000142, 0.000201, 0.000284, 0.000402,
    0.000568, 0.000803, 0.001136, 0.001624,
    0.001788, 0.002502, 0.003282, 0.004635,
    0.007449, 0.013139, 0.026769, 0.057818,
    /*24…31（lin = 2^-4 … 2^-0.5，也就是"亮段"）：按 FL=0 导出反解出来的
     * 映射重定。原值在这段系统性偏高 10~20%（实测显示值 +5~13 灰阶），
     * 之前用显示域标定表压不动（几何关系不同），这里直接把锚点改对。
     * 前段（≤23）与 SPP 几乎重合（如 index 18：0.003315 vs 0.003282），未动。 */
    0.092199, 0.163640, 0.256734, 0.347419,
    0.437330, 0.538847, 0.650903, 0.769143,
    0.986740, 0.990624, 0.993370, 0.995312,
    0.996685, 0.997656, 0.998343, 0.998828,
    0.999171
};

/* 比表起点更暗的地方按表起点那条直线的斜率延伸（T ≈ 0.59·lin）。
 * 0.000009 / 2^-16 ≈ 0.59 */
#define X3F_BASE_TINY_SLOPE 0.590

/*X3I(SFD) 专用显示曲线 —— X3I 窗口**默认档**（细节最平滑、对比度最低、
 * 阴影最高，其余中间值）跑出来的显示映射。
 *
 * **那一档已经烤在这张表里了**，所以 App 侧的面板虽然就停在 那几个位置
 * （对比度最左 / 阴影最右 / 锐度最左 / 细节最左，见 `RawEditParams.x3iDefault`），
 * 渲染前会先把这段基线减掉（`removingX3IBaseline`，对比度/阴影各减一半行程），
 * 免得同一件事扣两遍 —— 表 + 零点参数 = SPP 默认档那张图。
 *
 * 怎么来的：用户在 SPP 里把 DP0Q0187.X3I 按默认导了一张 16bit TIFF（5424×3616，
 * sRGB），把它的像素值与**我们自己的合成线性数据**逐通道配对（×4 对齐，
 * 每 0.5 档 log2 取中位），得到下面这张表。也就是说它不是"拟合我们的参数"，
 * 而是"SPP 那条链在这个宽范围输入上的实测结果"。
 *
 * 与 kX3FBaseCurve 同一套网格（log2 从 −16 起、步 0.5、41 个锚点），
 * 差别在：白点被推到 lin≈1.4 以后（lin=1.0 只到 236/255），
 * 也就是 SPP 那种"偏灰、高光留着"的观感 —— 超出 SDR 的那 2~3 档才有地方落脚。
 * 表外（比 −9.5 档更暗 / 比 +0.5 档更亮）按相邻斜率外推。
 *
 * **注意这里是"线性光"值**：量出来的 SPP 显示值是 sRGB 编码后的，
 * 落表前要先 srgb_decode 回线性光（apply_base_tone 的输出还要过一次 srgb_encode）。
 * 第一版忘了这一步，等于编码了两遍，整幅偏亮 50/255。 */
static const double kX3FSFDTone[X3F_BASE_NA] = {
    0.000418, 0.000513, 0.000629, 0.000773,
    0.000948, 0.001164, 0.001429, 0.001755,
    0.002154, 0.002645, 0.003247, 0.003986,
    0.004893, 0.006007, 0.007202, 0.008256,
    0.009979, 0.012146, 0.015276, 0.019271,
    0.024980, 0.032423, 0.043001, 0.057282,
    0.077258, 0.103345, 0.143471, 0.205805,
    0.275015, 0.360737, 0.488228, 0.694570,
    0.842396, 0.927589, 0.945692, 0.959269,
    0.969452, 0.977089, 0.982816, 0.987112,
    0.990334
};

/* ---------- SPP「过曝矫正」的色调曲线差异（勾选 vs 未勾选）----------
 *
 * 勾选框在 SPP 里做两件事：① DP23HLPsRemoval 修高光专用像素（见
 * x3f_render_set_hlp_fix）；② 把 ToneParams.m_fTCFSteepness2 减 0.7
 * （勾选 1.4 → 未勾选 0.7），整条 sigmoid 色调曲线跟着变（§18.11）。
 *
 * 下面这张表是 相机给的差： 加载
 * ，调用真正的 FPixelProc2::GenerateF20ToneCurve
 * （fPixelSaturation=16384、uiLUTShift=16、色彩模式 7、四个滑块全 0、权重 1），
 * 把 steepness2=1.4 与 0.7 两次输出的 16bit LUT 逐点相除，得到
 * 【未勾选 / 勾选】的增益；自变量就是 LUT 输入码（0..1 = 传感器读数/饱和）。
 * 生成脚本 outputs/tools/sppref/gen_tone_gain.py，它把比值写成
 * include/x3f_oe_tone_gain.h（**生成文件，不入库**）；原始两条 LUT 是探针
 * harness.c 跑出来的 lut_ref.txt，同样不入库，见 。
 *
 * 形状：~0.94（暗部）→ 1.146（中高光）→ 1.0（白点），即**未勾选**时暗部略暗、
 * 高光更亮。我们的基础曲线是对着 SPP *勾选态* 标定的（实测 |我们−SPP勾选|
 * = 4.6 灰阶且是平坦偏置，|我们−SPP未勾选| = 7.0），所以只有"未勾选"要乘这张表。
 * 160 格线性插值，in>0.01 段相对误差 < 0.23%。
 *
 * 口径提醒：差作用在"色调 LUT 输出"上，而我们的基础曲线把显示 gamma
 * 一起折进去了，所以这里只能在线性域做近似换算（见 §18.15 的对拍表：
 * 低/中段吻合，高光偏保守约 6 成）。要分毫不差就得把色调段换成 SPP 那条真 LUT。 */
/* 线性域里"传感器饱和"对应的值：基础曲线标定时白点在 lin≈1.4（见 kX3FBaseCurve
 * 上方的说明）。 LUT 输入码 = 传感器读数 / 饱和，所以 in = lin / 1.4。*/
#define X3F_OE_LIN_WHITE 1.4

#if X3F_HAVE_SPP_OE_TONE
/* 表在 x3f_oe_tone_gain.h 里（生成文件，见上面那段）。 */
#else
/* 没有生成表 —— 这一项按"没有差"处理（未勾选 = 勾选，增益恒 1.0）。
 * 只影响「过曝矫正」未勾选时暗部/高光那点差别，别的参数一概不受影响。
 * 想恢复：装上 SPP，按 跑 gen_tone_gain.py。*/
#define X3F_OE_TONE_IN_NA 161   /* 输入域表（160 格） */
static const double kX3FOEToneGainIn[X3F_OE_TONE_IN_NA] = {
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0
};
#endif

static int g_oe_tone_off = -1;
static int g_oe_tone_env = -2;      /* -2 = 还没读环境变量 */

void x3f_render_set_oe_tone_off(int off) { g_oe_tone_off = off ? 1 : 0; }

int x3f_render_oe_tone_off(void)
{
    if (g_oe_tone_env == -2) {
        const char *e = getenv("X3F_OE_TONE");
        g_oe_tone_env = (e != NULL && *e != '\0') ? ((*e != '0') ? 1 : 0) : -1;
    }
    if (g_oe_tone_env >= 0) return g_oe_tone_env;   /* 环境变量优先，方便 A/B */
    return g_oe_tone_off > 0 ? 1 : 0;
}

/* 同一条 SPP 差，但按**输入**（归一化线性值，1.0 = 传感器饱和）取表。
 * 这是 SPP 本来的口径：两条 LUT 都是输入的曲线，比值当然也是输入的函数。 */

/* 【2026-10-30 边缘限幅（opt-in，X3F_OE_EDGE_CLIP=1）】
 * oe_tone「未勾选」差值曲线是全图逐像素施加的（编在 tone LUT 里），高对比
 * 1px 边缘（建筑鳍片、天线、反光丝）会整条被差值曲线再推亮 ~10 级 —— 用户
 * 实测 Quattro 1:1 截图过冲 46 vs SPP 41 的放大主因。本开关在 LUT 消费端做
 * 空间混合：勾选态基准 LUT（差值不施加）+ 边缘掩膜，掩膜像素在两套 LUT 间
 * 线性插值。
 *   · 掩膜 = 0（平坦区）：逐位 = 现行为（差值曲线原样）——用户拍板的 17:29
 *     观感不动；
 *   · 掩膜 = 1（强边缘 ±1px）：= 勾选态处理 ≈ SPP 对同样像素的行为（SPP
 * 没有这层差值）。注意这是「回到 线高」（场景固有细线仍在），
 *     不是去细线。
 * 阈值 X3F_OE_EDGE_THRESH="t0,t1"（log 亮度域 4 邻差软阈 + 3×3 膨胀，
 * 默认 0.35,0.70；log 差 0.35 ≈ 邻像素亮度跳 1.42×）。App 侧走显式 setter
 * （x3f_render_set_oe_edge_clip），env 只给 lab/renderlab A/B 用。 */
static int g_oe_edge_clip_force = -1;
void x3f_render_set_oe_edge_clip(int enable) { g_oe_edge_clip_force = enable ? 1 : 0; }
static int oe_edge_clip_on(void)
{
    if (g_oe_edge_clip_force >= 0) return g_oe_edge_clip_force;
    {
        const char *e = getenv("X3F_OE_EDGE_CLIP");
        return (e != NULL && *e != '\0' && *e != '0') ? 1 : 0;
    }
}

/* 边缘掩膜：log 亮度域 4 邻差 → 软阈 [t0,t1] → 3×3 膨胀（覆盖 ±1px 边带）。
 * 产物 u8 0..255；失败返回 NULL（调用方整体降级回现行为）。 */
static uint8_t *oe_edge_mask_build(const float *lin, int w, int h,
                                   double t0, double t1)
{
    const size_t np = (size_t)w * (size_t)h;
    float *ym = malloc(np * sizeof(float));
    float *gm = malloc(np * sizeof(float));
    uint8_t *mk = malloc(np);
    size_t i;
    int x, y;
    const double inv = 1.0 / (t1 - t0);
    if (ym == NULL || gm == NULL || mk == NULL) {
        free(ym); free(gm); free(mk);
        return NULL;
    }
    for (i = 0; i < np; i++) {
        const float *s = lin + i * 3;
        ym[i] = 0.2126f * s[0] + 0.7152f * s[1] + 0.0722f * s[2];
    }
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            const size_t idx = (size_t)y * w + x;
            const double yc = log((double)ym[idx] + 1e-4);
            double g = 0.0;
            if (x > 0) {
                double d = fabs(log((double)ym[idx - 1] + 1e-4) - yc);
                if (d > g) g = d;
            }
            if (x < w - 1) {
                double d = fabs(log((double)ym[idx + 1] + 1e-4) - yc);
                if (d > g) g = d;
            }
            if (y > 0) {
                double d = fabs(log((double)ym[idx - w] + 1e-4) - yc);
                if (d > g) g = d;
            }
            if (y < h - 1) {
                double d = fabs(log((double)ym[idx + w] + 1e-4) - yc);
                if (d > g) g = d;
            }
            g = (g - t0) * inv;
            gm[idx] = (float)(g < 0.0 ? 0.0 : (g > 1.0 ? 1.0 : g));
        }
    }
    /* 3×3 膨胀：边缘带扩到 ±1px（差值曲线对边缘两侧各 ~1px 都有感） */
    for (y = 0; y < h; y++) {
        const int y0 = (y > 0) ? y - 1 : 0;
        const int y1 = (y < h - 1) ? y + 1 : h - 1;
        for (x = 0; x < w; x++) {
            const int x0 = (x > 0) ? x - 1 : 0;
            const int x1 = (x < w - 1) ? x + 1 : w - 1;
            float m = 0.0f;
            int yy, xx;
            for (yy = y0; yy <= y1; yy++)
                for (xx = x0; xx <= x1; xx++) {
                    const float v = gm[(size_t)yy * w + xx];
                    if (v > m) m = v;
                }
            mk[(size_t)y * w + x] = (uint8_t)(m * 255.0f + 0.5f);
        }
    }
    free(ym); free(gm);
    return mk;
}
static inline double oe_tone_gain_in(double in)
{
    double e, f;
    int i;
    if (in <= 0.0) return 1.0;
    if (in >= 1.0) return kX3FOEToneGainIn[X3F_OE_TONE_IN_NA - 1];
    e = in * (X3F_OE_TONE_IN_NA - 1);
    i = (int)e;
    f = e - i;
    return kX3FOEToneGainIn[i] + (kX3FOEToneGainIn[i + 1] - kX3FOEToneGainIn[i]) * f;
}

static inline double apply_base_tone(double lin, double hl_keep)
{
    double e, f;
    int i;

    if (lin <= 0.0) return 0.0;

    e = (log2(lin) - X3F_BASE_LOG2_MIN) / X3F_BASE_LOG2_STEP;
    if (e <= 0.0) return X3F_BASE_TINY_SLOPE * lin;
    if (e >= X3F_BASE_NA - 1) return kX3FBaseCurve[X3F_BASE_NA - 1];

    i = (int)e;
    f = e - i;
    {
        double v = kX3FBaseCurve[i] + (kX3FBaseCurve[i + 1] - kX3FBaseCurve[i]) * f;
        /* 高光保留（hl_keep，见头文件）：把高光端那段"硬肩"换成有梯度的斜坡。
         *
         * 现状：最后 8 个锚点（4 档 EV）只有 0.987 → 0.999 的跨度，落到 8bit
         * 就是 252~255 —— RAW 里那 1~2 档余量全挤在这几个码值里，所以
         * "切得少了但看不出层次"（Quattro 实测标准差 1.8）。
         * 改法：同样这 8 个锚点拉成 0.90 → 1.00 的斜坡，余量就有地方分开了。
         * 代价：亮部整体会比相机直出暗一点（0.90 vs 0.987 起跳）。
         * hl_keep 就是"拉多少"这个旋钮：0 = 曲线原样，1 = 完全拉成斜坡。 */
        if (hl_keep > 0.0) {
            const int TOP = 8;
            int i0 = X3F_BASE_NA - 1 - TOP;
            if (i >= i0) {
                double t = (i - i0 + f) / (double)TOP;
                double lo;
                if (t < 0.0) t = 0.0;
                if (t > 1.0) t = 1.0;
                /* 起点在"曲线本来的值"和 0.90 之间按 hl_keep 插值 */
                lo = kX3FBaseCurve[i0] * (1.0 - hl_keep) + 0.90 * hl_keep;
                v = lo + (1.0 - lo) * t;
            }
        }
        return v;
    }
}

/* ---------- 显示编码与影调调整 ---------- */

/* X3I(SFD) 的显示曲线：直接查 kX3FSFDTone（同一套 log2 网格、线性内插）。
 * 表里**已经含了** SPP 那条高光肩，所以不再叠 hl_keep。 */
static inline double apply_sfd_tone(double lin)
{
    double e, f;
    int i;
    if (lin <= 0.0) return 0.0;
    e = (log2(lin) - X3F_BASE_LOG2_MIN) / X3F_BASE_LOG2_STEP;
    if (e <= 0.0) return kX3FSFDTone[0] * (lin / pow(2.0, X3F_BASE_LOG2_MIN));
    if (e >= X3F_BASE_NA - 1) return kX3FSFDTone[X3F_BASE_NA - 1];
    i = (int)e;
    f = e - i;
    return kX3FSFDTone[i] + (kX3FSFDTone[i + 1] - kX3FSFDTone[i]) * f;
}

/*一条线里选曲线：SFD 模式走 实测表，其它走标定出来的基础曲线 + hl_keep。*/
static inline double tone_curve(const x3f_edit_params_t *p, double lin)
{
    if (p != NULL && p->sfd_tone) return apply_sfd_tone(lin);
    return apply_base_tone(lin, p != NULL ? p->hl_keep : 0.5);
}

static inline double srgb_encode(double v)
{
    if (v <= 0.0) return 0.0;
    if (v <= 0.0031308) return 12.92 * v;
    return 1.055 * pow(v, 1.0 / 2.4) - 0.055;
}

static inline double srgb_decode(double v)
{
    if (v <= 0.0) return 0.0;
    if (v <= 0.04045) return v / 12.92;
    return pow((v + 0.055) / 1.055, 2.4);
}

/* ---------- 输出色彩空间：ProPhoto RGB（2026-11-04）-------------------
 *
 * 只做一件事：把"线性 sRGB（D65）"换成"线性 ProPhoto（D50）"。用的是三个
 * 公开色度学矩阵合并后的一个 3×3：
 *
 *     sRGB_to_XYZ(D65) · Bradford(D65→D50) · XYZ(D50)_to_ProPhoto(ROMM)
 *
 * 与 own_decoder.c 里 srgb_to_xyz / xyz_to_srgb 同一做法（标准常数直接写死，
 * 不含我们的标定资产，故不涉及加密合规）。
 *
 * 已验证：白点 (1,1,1)→(1,1,1)、中性灰 (v,v,v)→(v,v,v)。也就是说换了色域
 * **中性色的影调一点不变**，只有彩色的通道比例会变 —— 这正是我们要的。
 */
static const double kSRGBToProPhoto[9] = {
    0.5292864, 0.3300948, 0.1406435,
    0.0982955, 0.8735194, 0.0281722,
    0.0168516, 0.1176804, 0.8656226
};

static inline void srgb_lin_to_prophoto_lin(double *v)
{
    const double a = v[0], b = v[1], c = v[2];
    v[0] = kSRGBToProPhoto[0] * a + kSRGBToProPhoto[1] * b + kSRGBToProPhoto[2] * c;
    v[1] = kSRGBToProPhoto[3] * a + kSRGBToProPhoto[4] * b + kSRGBToProPhoto[5] * c;
    v[2] = kSRGBToProPhoto[6] * a + kSRGBToProPhoto[7] * b + kSRGBToProPhoto[8] * c;
}

/* ProPhoto 的传递函数是 γ1.8 纯幂（ROMM RGB 规格），没有 sRGB 那段线性趾。
 * 与 srgb_encode 一样**不夹上界**（>1 留给后面统一处理，保持两条链同构）。 */
static inline double pp_encode(double v)
{
    if (v <= 0.0) return 0.0;
    return pow(v, 1.0 / 1.8);
}

static inline double pp_decode(double v)
{
    if (v <= 0.0) return 0.0;
    return pow(v, 1.8);
}

/* ---------- 输出色彩空间：Adobe RGB (1998)（2026-11-04）----------------
 * 同样是标准常数合并：sRGB_to_XYZ(D65) · XYZ(D65)_to_AdobeRGB。
 * Adobe RGB 与 sRGB 同为 D65，不需要 Bradford。γ = 563/256 ≈ 2.1992（规格值；
 * 旧 x3f_process.c 用的是 2.2，这里取规格值，差 0.04% 可忽略）。
 * 白点 (1,1,1)→(1,1,1)、中性灰 (v,v,v)→(v,v,v)，同样成立。
 */
static const double kSRGBToAdobe[9] = {
    0.7151773, 0.2848856, 0.0000474,
   -0.0000812, 1.0000475, -0.0000000,
   -0.0000279, 0.0411633, 0.9587994
};

static inline void srgb_lin_to_adobe_lin(double *v)
{
    const double a = v[0], b = v[1], c = v[2];
    v[0] = kSRGBToAdobe[0] * a + kSRGBToAdobe[1] * b + kSRGBToAdobe[2] * c;
    v[1] = kSRGBToAdobe[3] * a + kSRGBToAdobe[4] * b + kSRGBToAdobe[5] * c;
    v[2] = kSRGBToAdobe[6] * a + kSRGBToAdobe[7] * b + kSRGBToAdobe[8] * c;
}

#define X3F_ADOBE_GAMMA (563.0 / 256.0)

static inline double adobe_encode(double v)
{
    if (v <= 0.0) return 0.0;
    return pow(v, 1.0 / X3F_ADOBE_GAMMA);
}

static inline double adobe_decode(double v)
{
    if (v <= 0.0) return 0.0;
    return pow(v, X3F_ADOBE_GAMMA);
}

/* ---------- 输出色彩空间：Display P3 / DCI-P3（2026-11-04）------------
 *
 * 两个空间共用同一组原色（P3：R(0.680,0.320) G(0.265,0.690) B(0.150,0.060)，
 * D65），区别只在传递函数：Display P3 用 sRGB 那条分段曲线，DCI-P3 用纯 γ2.6。
 *
 * 矩阵：sRGB_to_XYZ(D65) · XYZ(D65)_to_P3，同为 D65 不需要 Bradford。
 * 注意第三列前两项是 0 —— sRGB 的蓝原色 (0.150,0.060) 与 P3 蓝原色**同坐标**，
 * 所以蓝那一路不需要串色。行和精确为 1，(v,v,v)→(v,v,v) 成立。
 *
 * Display P3 是 sRGB 的**真超集**（原色更饱和、白点与曲线完全相同），所以
 * sRGB 色域内的颜色转过去全部落在 [0,1]，只有我们线性 sRGB 里的负值（相机
 * 超色域部分）会变成 P3 的正值 —— 这正是这档要保住的东西。
 *
 * DCI-P3 的白点：SMPTE RP 431-2 的影院白点其实是 ~6300K（x=0.314,y=0.351），
 * 但系统那个 `DCI(P3) RGB.icc` 的白点写的是 D50 适配值（即按 D65 处理）。
 * 这里跟着 ICC 走（P3-D65 矩阵 + γ2.6），保证「码值 ↔ 贴上去的 ICC」自洽；
 * 拿去做真正的数字影院母版之前要按 DCI 白点另算，别直接上 DCP。
 */
static const double kSRGBToP3[9] = {
    0.8224620, 0.1775380, 0.0000000,
    0.0331942, 0.9668058, 0.0000000,
    0.0170826, 0.0723974, 0.9105199
};

static inline void srgb_lin_to_p3_lin(double *v)
{
    const double a = v[0], b = v[1], c = v[2];
    v[0] = kSRGBToP3[0] * a + kSRGBToP3[1] * b + kSRGBToP3[2] * c;
    v[1] = kSRGBToP3[3] * a + kSRGBToP3[4] * b + kSRGBToP3[5] * c;
    v[2] = kSRGBToP3[6] * a + kSRGBToP3[7] * b + kSRGBToP3[8] * c;
}

#define X3F_DCI_GAMMA 2.6

static inline double dci_encode(double v)
{
    if (v <= 0.0) return 0.0;
    return pow(v, 1.0 / X3F_DCI_GAMMA);
}

static inline double dci_decode(double v)
{
    if (v <= 0.0) return 0.0;
    return pow(v, X3F_DCI_GAMMA);
}

/* ---------- 输出色彩空间：Rec.ITU-R BT.2020（2026-11-04）--------------
 *
 * 原色 R(0.708,0.292) G(0.170,0.797) B(0.131,0.046)，D65。矩阵同样是
 * sRGB_to_XYZ(D65) · XYZ(D65)_to_BT2020，行和 1，(v,v,v)→(v,v,v)。
 *
 * 传递函数是 BT.2020 的 **OETF 反函数**（不是 γ2.4，也不是 BT.1886）：
 *   V < d      →  L = c·V
 *   V ≥ d      →  L = (a·V + b)^g
 * 常数直接取系统 `ITU-2020.icc` 里 para type3 的那五个值 —— 必须与 ICC 逐位
 * 自洽，否则文件在别的软件里打开就是错的颜色（少 1% 的 α 就够看出来）。
 * 这等价于 BT.2020 规范的 α=1.0993、β=0.018（1/1.0993=0.9097，4.5β=0.081）。
 *
 * 为什么给 OETF 而不是显示用的 γ2.4：16 位 TIFF 是**素材母版**，OETF 是
 * 采集侧定义，转显示时由播放端的 EOTF（BT.1886）决定。把显示曲线焊死在
 * 文件里，等于替后期决定了亮度。
 */
static const double kSRGBTo2020[9] = {
    0.6274039, 0.3292830, 0.0433131,
    0.0690973, 0.9195404, 0.0113623,
    0.0163914, 0.0880133, 0.8955953
};

static inline void srgb_lin_to_2020_lin(double *v)
{
    const double a = v[0], b = v[1], c = v[2];
    v[0] = kSRGBTo2020[0] * a + kSRGBTo2020[1] * b + kSRGBTo2020[2] * c;
    v[1] = kSRGBTo2020[3] * a + kSRGBTo2020[4] * b + kSRGBTo2020[5] * c;
    v[2] = kSRGBTo2020[6] * a + kSRGBTo2020[7] * b + kSRGBTo2020[8] * c;
}

#define X3F_2020_A 0.909668    /* 1/1.0993 */
#define X3F_2020_B 0.090332
#define X3F_2020_G 2.222229    /* 1/0.45 */
#define X3F_2020_C 0.222229    /* 1/4.5 */
#define X3F_2020_D 0.080994    /* 4.5×0.018 */

/* 传递函数方向约定（**踩过一次，别再搞反**）：
 *   encode = 线性 → 码值（写文件时用），就是 OETF 本身
 *   decode = 码值 → 线性（读回来/回算时用），是 OETF 的反函数
 * ICC 里 para type3 描述的是 **decode**（X 是设备码值，Y 是线性值），所以从
 * ICC 抄来的 (a,b,g,c,d) 直接对应 decode；encode 要反解。
 * 2026-11-04 第一版把两边的公式写反了，导出的 Rec.2020 整幅几乎全黑
 * （均值 0.49/1.37/3.16），转回 sRGB 与其它档差 60 级 —— 就是这么发现的。 */
static inline double bt2020_encode(double v)
{
    if (v <= 0.0) return 0.0;
    if (v < X3F_2020_C * X3F_2020_D) return v / X3F_2020_C;      /* L < β=0.018 */
    return (pow(v, 1.0 / X3F_2020_G) - X3F_2020_B) / X3F_2020_A;
}

static inline double bt2020_decode(double v)
{
    if (v <= 0.0) return 0.0;
    if (v < X3F_2020_D) return X3F_2020_C * v;
    return pow(X3F_2020_A * v + X3F_2020_B, X3F_2020_G);
}

/* ---- 输出空间分派（0 = sRGB，逐位零回归；其余为广色域）-------------
 * 只在 apply_row 的最后一步调用：sRGB 显示值 → 线性 → 这里转域 → out_encode。
 * 输入可以是负值（sRGB 之外的真实颜色），矩阵对负值同样成立。 */
static inline void out_lin_convert(int space, double *v)
{
    switch (space) {
    case X3F_OUT_PROPHOTO: srgb_lin_to_prophoto_lin(v); break;
    case X3F_OUT_ADOBE:    srgb_lin_to_adobe_lin(v);    break;
    case X3F_OUT_P3:       srgb_lin_to_p3_lin(v);       break;
    case X3F_OUT_DCI_P3:   srgb_lin_to_p3_lin(v);       break;  /* 同原色 */
    case X3F_OUT_2020:     srgb_lin_to_2020_lin(v);     break;
    default: break;                                             /* sRGB 原样 */
    }
}

static inline double out_encode(int space, double v)
{
    switch (space) {
    case X3F_OUT_PROPHOTO: return pp_encode(v);
    case X3F_OUT_ADOBE:    return adobe_encode(v);
    case X3F_OUT_P3:       return srgb_encode(v);   /* P3 的 TRC 就是 sRGB 那条 */
    case X3F_OUT_DCI_P3:   return dci_encode(v);
    case X3F_OUT_2020:     return bt2020_encode(v);
    default:               return srgb_encode(v);
    }
}

static inline double out_decode(int space, double v)
{
    switch (space) {
    case X3F_OUT_PROPHOTO: return pp_decode(v);
    case X3F_OUT_ADOBE:    return adobe_decode(v);
    case X3F_OUT_P3:       return srgb_decode(v);
    case X3F_OUT_DCI_P3:   return dci_decode(v);
    case X3F_OUT_2020:     return bt2020_decode(v);
    default:               return srgb_decode(v);
    }
}

/* ---------- 有符号外推（2026-11-04，全链路广色域）-------------------
 *
 * 上面那组传输函数都在 v <= 0 处直接返回 0。对 sRGB 出图这是对的（负码值
 * 没有意义），但对**广色域**输出是致命的：
 *
 *   相机矩阵按 sRGB 原色分解，超出 sRGB 三角形的真实颜色就落在**负值**上
 *   （例如饱和青在 sRGB 里是 R < 0）。把这些负值夹成 0，等于在进门第一步
 *   就把 sRGB 之外的内容删掉 —— 之后无论怎么转域，ProPhoto/P3 容器里装的
 *   仍然只有 sRGB 的东西。这就是"宽容器、窄内容"的来历。
 *
 * 所以广色域路径改用奇对称外推 f(-x) = -f(x)：负激发按同等强度压缩、符号
 * 相反，转域矩阵之后自然变成目标域里的正值，色相和饱和度都是对的。
 *
 * **只有广色域路径用这几个函数**。sRGB 路径保持原来的 sm_* （v<=0 → 0），
 * 逐位零回归不受影响。
 */
static inline double srgb_encode_s(double v)
{
    if (v < 0.0) return -srgb_encode(-v);
    return srgb_encode(v);
}

static inline double srgb_decode_s(double v)
{
    if (v < 0.0) return -srgb_decode(-v);
    return srgb_decode(v);
}

static inline double out_encode_s(int space, double v)
{
    if (v < 0.0) return -out_encode(space, -v);
    return out_encode(space, v);
}

/* LUT 内部的"显示值 ↔ 线性"换算。
 *
 * 数值上 `lut_encode(0, v)` 就是 `srgb_encode(v)`（out_encode 的 default 分支），
 * 但**不能**在这里直调 srgb_encode：编译器对 switch 的内联与常量折叠不同，
 * 实测 sRGB 出图会在 0.06% 像素上差 1~2 级，零回归铁律直接破。所以 sRGB
 * （osp == 0）一律走原来的 out_encode / out_decode 调用形式。 */
static inline double lut_encode(int osp, double v)
{
    return (osp == X3F_OUT_SRGB) ? out_encode(osp, v) : srgb_encode(v);
}

static inline double lut_decode(int osp, double v)
{
    return (osp == X3F_OUT_SRGB) ? out_decode(osp, v) : srgb_decode(v);
}

/* ---------- SFD：逐帧 JPEG 高光融合 ----------
 *
 * 七帧容器每帧都带一张自己的内嵌 JPEG。合成以前只在**合成结果**上拿 0 EV 那一张
 * 去融合，于是高光里 0 EV JPEG 已经切白的地方，合成捡回来的颜色又被抹回白。
 * 这里把同一套融合放进逐帧循环：每帧先按自己的曝光把 JPEG 归一到基准帧，
 * 与这一帧的显示值融合，再交给合成累加。
 *
 * 融合在**显示域**里做（和 App 里单帧那条 `fuseJPEGHighlights` 同一个口径）：
 * `lin*k` 与 JPEG 都先归一到基准帧，过同一条基础曲线，再按亮度权重插值。
 * 插完显示值查一次反向曲线回到线性，除以 k 交还给合成。
 *
 * 不能直接在线性域插值：JPEG 的 sRGB 值反 gamma 后仍带着相机的色调曲线，
 * 与场景线性不是同一个刻度，直接把两者线性混合会把整幅顶亮（实测 ≥250
 * 从 2.7% 涨到 40%）。显示域插值才是"截断处趋向 JPEG"的原意。
 */
#define X3F_SFD_INV_NA 65536
#define X3F_SFD_TONE_NA 4096
long x3f_sfd_fuse_frame_linear(float *rgb, int w, int h,
                               const uint8_t *jpeg, int stride,
                               double k, double strength,
                               double threshold, double jpeg_floor)
{
    double decode_lut[256];
    double inv_tone[X3F_SFD_INV_NA + 1];
    double tone_lut[X3F_SFD_TONE_NA + 1];
    double enc_lut[X3F_SFD_TONE_NA + 1];
    int x, y, c, i;
    long touched = 0;
    double t0, f0, f1, st, g_jpeg = 1.0;

    if (rgb == NULL || jpeg == NULL || w <= 0 || h <= 0 ||
        (stride != 3 && stride != 4))
        return -1;
    if (!(k > 0.0)) k = 1.0;
    st = strength;
    if (st < 0.0) st = 0.0;
    if (st > 1.0) st = 1.0;
    if (st <= 0.0001) return 0;
    t0 = threshold;
    if (t0 < 0.0) t0 = 0.0;
    if (t0 > 0.999) t0 = 0.999;
    f0 = jpeg_floor;
    if (f0 < 0.0) f0 = 0.0;
    if (f0 > 1.0) f0 = 1.0;
    f1 = f0 + 0.25;
    if (f1 > 0.999) f1 = 0.999;
    if (f1 <= f0 + 1e-6) f1 = f0 + 1e-6;

    for (i = 0; i < 256; i++) decode_lut[i] = srgb_decode((double)i / 255.0);

    /* 每帧几十兆像素，逐像素调 log2/pow 会把一次合成拖慢半分钟。
     * 基础曲线按 sqrt(lin) 均匀取样（暗部密、亮部疏），sRGB 编码按显示值
     * 均匀取样；像素里只剩一次 sqrt + 两次查表。 */
    for (i = 0; i <= X3F_SFD_TONE_NA; i++) {
        double t = (double)i / X3F_SFD_TONE_NA;
        double lin = X3F_LIN_MAX * t * t;
        tone_lut[i] = apply_base_tone(lin, 0.5);
        enc_lut[i] = srgb_encode(t);
    }

    /* 反向基础曲线：y（显示线性 0…~0.999）→ lin。基础曲线单调，二分即可。
     * 表按 y 均匀取样，像素里只做一次线性插值。 */
    for (i = 0; i <= X3F_SFD_INV_NA; i++) {
        double yt = (double)i / X3F_SFD_INV_NA * 0.999171;
        double lo = 0.0, hi = X3F_LIN_MAX;
        int it;
        for (it = 0; it < 24; it++) {
            double mid = 0.5 * (lo + hi);
            if (apply_base_tone(mid, 0.5) < yt) lo = mid; else hi = mid;
        }
        inv_tone[i] = 0.5 * (lo + hi);
    }

    /* JPEG guide 的逐帧全局曝光对齐。
     *
     * 相机 JPEG 里的色调曲线不完全是我们的 `apply_base_tone`，直接做
     * "反曲线 → ×k → 再走曲线" 会系统性地偏亮（实测整幅 +14/255）。
     * 这里只用中间调估一个标量 g，让 guide 的中间调亮度与这一帧自己的显示值
     * 对齐；高光段的相对形状和颜色仍由 JPEG 决定。每 3×3 取一个样本就够。 */
    {
        double sd = 0.0, sj = 0.0;
        long n = 0;
        int sy, sx;
        for (sy = 0; sy < h; sy += 3) {
            for (sx = 0; sx < w; sx += 3) {
                size_t p = ((size_t)sy * (size_t)w + (size_t)sx) * 3;
                const uint8_t *jp = jpeg + ((size_t)sy * (size_t)w + (size_t)sx)
                                         * (size_t)stride;
                double ld = 0.0, lj = 0.0;
                for (c = 0; c < 3; c++) {
                    double wc = (c == 0) ? 0.2126 : (c == 1 ? 0.7152 : 0.0722);
                    double lin = (double)rgb[p + c];
                    double jd_lin, ft, jscene;
                    int idx;
                    if (lin < 0.0) lin = 0.0;
                    jd_lin = decode_lut[jp[c]];
                    if (jd_lin >= 0.999171) jd_lin = 0.999171;
                    ft = jd_lin / 0.999171 * X3F_SFD_INV_NA;
                    idx = (int)ft;
                    if (idx >= X3F_SFD_INV_NA) idx = X3F_SFD_INV_NA - 1;
                    ft -= idx;
                    jscene = inv_tone[idx] * (1.0 - ft) + inv_tone[idx + 1] * ft;
                    {
                        double lv = lin * k, jv = jscene * k;
                        double tt;
                        int ti;
                        if (lv < 0) lv = 0; if (lv > X3F_LIN_MAX) lv = X3F_LIN_MAX;
                        if (jv < 0) jv = 0; if (jv > X3F_LIN_MAX) jv = X3F_LIN_MAX;
                        tt = sqrt(lv / X3F_LIN_MAX) * X3F_SFD_TONE_NA;
                        ti = (int)tt; if (ti >= X3F_SFD_TONE_NA) ti = X3F_SFD_TONE_NA - 1;
                        tt -= ti;
                        ld += wc * enc_lut[(int)((tone_lut[ti] * (1.0 - tt)
                                                   + tone_lut[ti + 1] * tt)
                                                  * X3F_SFD_TONE_NA)];
                        tt = sqrt(jv / X3F_LIN_MAX) * X3F_SFD_TONE_NA;
                        ti = (int)tt; if (ti >= X3F_SFD_TONE_NA) ti = X3F_SFD_TONE_NA - 1;
                        tt -= ti;
                        lj += wc * enc_lut[(int)((tone_lut[ti] * (1.0 - tt)
                                                   + tone_lut[ti + 1] * tt)
                                                  * X3F_SFD_TONE_NA)];
                    }
                }
                if (ld > 0.05 && ld < 0.60 && lj > 0.01) {
                    sd += ld; sj += lj; n++;
                }
            }
        }
        if (n > 1000 && sj > 1e-9) {
            g_jpeg = sd / sj;
            if (g_jpeg < 0.5) g_jpeg = 0.5;
            if (g_jpeg > 2.0) g_jpeg = 2.0;
        }
    }

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            size_t p = ((size_t)y * (size_t)w + (size_t)x) * 3;
            const uint8_t *jp = jpeg + ((size_t)y * (size_t)w + (size_t)x) * (size_t)stride;
            double d[3], jd[3], yr, yj, a, b, wgt;
            int any = 0;
            for (c = 0; c < 3; c++) {
                double lin = (double)rgb[p + c];
                double jd_lin, jscene;
                int idx;
                double ft;
                if (lin < 0.0) lin = 0.0;
                {
                    double lt = lin * k;
                    double tt;
                    int ti;
                    if (lt < 0.0) lt = 0.0;
                    if (lt > X3F_LIN_MAX) lt = X3F_LIN_MAX;
                    tt = sqrt(lt / X3F_LIN_MAX) * X3F_SFD_TONE_NA;
                    ti = (int)tt;
                    if (ti >= X3F_SFD_TONE_NA) ti = X3F_SFD_TONE_NA - 1;
                    tt -= ti;
                    d[c] = tone_lut[ti] * (1.0 - tt) + tone_lut[ti + 1] * tt;
                }
                /* JPEG 的 sRGB 先解码成"显示线性"，再用反向基础曲线还原成场景线性，
                 * 乘 k 归一到基准帧，最后过同一条曲线。只做 sRGB 反 gamma 是不够的：
                 * 相机 JPEG 里还烤着一条色调曲线，直接在 gamma 域乘 k 会把中间调
                 * 提亮（实测 ≥250 从 2.7% 涨到 33%）。 */
                jd_lin = decode_lut[jp[c]];
                if (jd_lin >= 0.999171) jd_lin = 0.999171;
                ft = jd_lin / 0.999171 * X3F_SFD_INV_NA;
                idx = (int)ft;
                if (idx >= X3F_SFD_INV_NA) idx = X3F_SFD_INV_NA - 1;
                ft -= idx;
                jscene = (inv_tone[idx] * (1.0 - ft) + inv_tone[idx + 1] * ft)
                       * k * g_jpeg;
                {
                    double tt;
                    int ti;
                    if (jscene < 0.0) jscene = 0.0;
                    if (jscene > X3F_LIN_MAX) jscene = X3F_LIN_MAX;
                    tt = sqrt(jscene / X3F_LIN_MAX) * X3F_SFD_TONE_NA;
                    ti = (int)tt;
                    if (ti >= X3F_SFD_TONE_NA) ti = X3F_SFD_TONE_NA - 1;
                    tt -= ti;
                    jd[c] = tone_lut[ti] * (1.0 - tt) + tone_lut[ti + 1] * tt;
                }
                if (jd[c] > 0.0) any = 1;
            }
            if (!any) continue;
            {
                double e0, e1, e2;
                e0 = d[0] * X3F_SFD_TONE_NA;
                e1 = d[1] * X3F_SFD_TONE_NA;
                e2 = d[2] * X3F_SFD_TONE_NA;
                if (e0 < 0) e0 = 0; if (e0 > X3F_SFD_TONE_NA - 1) e0 = X3F_SFD_TONE_NA - 1;
                if (e1 < 0) e1 = 0; if (e1 > X3F_SFD_TONE_NA - 1) e1 = X3F_SFD_TONE_NA - 1;
                if (e2 < 0) e2 = 0; if (e2 > X3F_SFD_TONE_NA - 1) e2 = X3F_SFD_TONE_NA - 1;
                yr = 0.2126 * enc_lut[(int)e0] + 0.7152 * enc_lut[(int)e1]
                   + 0.0722 * enc_lut[(int)e2];
                e0 = jd[0] * X3F_SFD_TONE_NA;
                e1 = jd[1] * X3F_SFD_TONE_NA;
                e2 = jd[2] * X3F_SFD_TONE_NA;
                if (e0 < 0) e0 = 0; if (e0 > X3F_SFD_TONE_NA - 1) e0 = X3F_SFD_TONE_NA - 1;
                if (e1 < 0) e1 = 0; if (e1 > X3F_SFD_TONE_NA - 1) e1 = X3F_SFD_TONE_NA - 1;
                if (e2 < 0) e2 = 0; if (e2 > X3F_SFD_TONE_NA - 1) e2 = X3F_SFD_TONE_NA - 1;
                yj = 0.2126 * enc_lut[(int)e0] + 0.7152 * enc_lut[(int)e1]
                   + 0.0722 * enc_lut[(int)e2];
            }
            a = (yr - t0) / (1.0 - t0);
            if (a < 0.0) a = 0.0; else if (a > 1.0) a = 1.0;
            a = a * a * (3.0 - 2.0 * a);
            b = (yj - f0) / (f1 - f0);
            if (b < 0.0) b = 0.0; else if (b > 1.0) b = 1.0;
            b = b * b * (3.0 - 2.0 * b);
            /* JPEG 自己已经切平的地方不要再往里拉：那里只剩一片白，
             * 没有颜色也没有层次。SFD 的高光核心往往只有 1~2 帧可用，
             * 把它们的 RAW 梯度换成 JPEG 的白点，就会在核心中心留下
             * 一颗星/一团硬白（用户 2026-10-20 截图）。留一点渐变到
             * 0.9995 才完全放手，避免在 0.985 处又造出一条硬边。 */
            if (yj > 0.985) {
                double cg = (yj - 0.985) / (0.9995 - 0.985);
                if (cg > 1.0) cg = 1.0;
                cg = cg * cg * (3.0 - 2.0 * cg);
                b *= (1.0 - cg);
            }
            wgt = a * b * st;
            if (wgt <= 0.0001) continue;
            for (c = 0; c < 3; c++) {
                double dout = d[c] * (1.0 - wgt) + jd[c] * wgt;
                double ft, lin_out;
                int idx;
                if (dout <= 0.0) { rgb[p + c] = 0.0f; continue; }
                if (dout >= 0.999171) dout = 0.999171;
                ft = dout / 0.999171 * X3F_SFD_INV_NA;
                idx = (int)ft;
                if (idx >= X3F_SFD_INV_NA) idx = X3F_SFD_INV_NA - 1;
                ft -= idx;
                lin_out = inv_tone[idx] * (1.0 - ft) + inv_tone[idx + 1] * ft;
                rgb[p + c] = (float)(lin_out / k);
            }
            touched++;
        }
    }
    return touched;
}

/*对比度：相机算法算法（2026-10-22 闭环：CombineToneCurve →
 * CreateBasicCurve ， arm64）。
 *
 * ToneLUT 基础曲线是两段 logistic：inner(x) = x^(1/γ)·(Ae−As)+As 走进
 * sigmoid，陡度 S₁ = TCFSteepness·2^(co/2)、S₂ = TCFSteepness2·2^(co/2)
 * （GenerateF20ToneCurve 内 exp2(co/2) 实证；camfile 默认
 * TCFSteepness=3.0 / Steepness2=1.4~2.0，按色彩模式覆盖）。对比度滑杆**只缩放
 * 陡度**：co=±2 → ×2/×0.5；枢轴 inner=0（σ=0.5，输出中灰）固定不动。
 * 对任意基础曲线 f₀ 的等价形式即 logit 域陡度缩放：
 *     f_co(x) = σ( 2^(co/2) · logit(f₀(x)) )
 * 在 主段（piece1，无偏移）与真身逐点相等；肩部（piece2 带
 * σ₁(B)−σ₂(B) 连续性偏移）为自然推广。app 滑杆与 SPP 滑杆值 1:1。
 * 旧实现 identity↔smoothstep 插值：+1 档斜率增益仅 ~1.5（真身 2^0.5≈1.41 陡度
 * 增益作用在 logit 上，两端远强于中段），且枢轴在输入中点而非输出 0.5 ——
 * 已按真身替换。端点 0/1 固定（相机算法 f(0)>0 来自 TCFStart 锚，量化后同为
 * 黑/白不动点）。 */
static inline double apply_contrast(double d, double c)
{
    double k, u;
    if (c == 0.0) return d;
    if (d <= 0.0) return 0.0;
    if (d >= 1.0) return 1.0;
    k = exp2(c * 0.5);                 /* 陡度乘子，SPP: Steep·2^(co/2) */
    u = log(d / (1.0 - d));            /*logit(f₀) = inner(x)*/
    d = 1.0 / (1.0 + exp(-k * u));
    return d;
}

/*阴影/高光：局部幂曲线，作用在**亮度**上。
 *
 * 的 DoAdditinalAdjustment 不是"整条曲线加一个量"，而是：
 *   · 阴影只改曲线输出 0…25% 这一段，0 与 25% 两个端点固定；
 *   · 高光只改 75%…100% 这一段，75% 与 100% 两个端点固定；
 *   · 段内用 pow() 改变局部 gamma。
 *
 * 旧实现用 d += shadow·0.45·(1-d)² / d -= highlight·0.45·d²。它看似也是
 * "暗部/亮部权重"，实际范围铺满整条曲线：阴影 +1 会把 10% 灰抬到 46%，
 * 高光 +1 会把 50% 中灰压到 39%，所以拖动后整幅发灰、层次变平。幂曲线
 * 端点固定，也不会给暗部加一个共同的 RGB 偏置，颜色比加法版干净得多。
 * 现在这条曲线在 apply_spatial() 里对显示亮度 Y 执行，然后按 Y'/Y 同比缩放
 * RGB。逐通道分别查曲线会在高饱和暗部把通道比例拉开；按亮度缩放则保持色比，
 * 只改明暗。
 *
 * 方向/强度按 相机算法的符号（探针实测：TP+0x70 正=压暗暗部、TP+0x68 正=提亮
 * 高光）：app 滑杆阴影正值提亮暗部、负值压暗；高光正值压暗、负值提亮。
 * app 参数与 SPP 滑杆值 1:1（SPPWrite 十分位直写直读），阴影段公式与真身
 * 逐字一致（exponent = exp2(2·shadow_tp)，shadow_tp = −滑杆值）；高光段见
 * 下面分支注释。SPP 还会轻微挪动基础曲线两端的锚点，使全图有很小的同向
 * 亮度偏移；这里不复制那段副作用，正是为了"阴影不洗中间调、高光不压暗
 * 中间调"。
 */
static inline double apply_sh_d(double d, double shadow, double highlight)
{
    const double lo = 0.25;
    const double hi = 0.75;

    if (d < 0.0) d = 0.0;
    if (d > 1.0) d = 1.0;

    if (shadow != 0.0 && d > 0.0 && d < lo) {
        /* 正向指数 2^-2；反向指数 2^2。两者都只在 0…25% 内改变 gamma。 */
        double e = exp2(-2.0 * shadow);
        double x = pow(d / lo, e);
        d = lo * x;
    }
    if (highlight != 0.0 && d > hi && d < 1.0) {
        /*SPP DoAdditinalAdjustment 的高光段已读通 + 运行时探针
         * 实证，见 §2.2）：
         *   if (highlight < 0) highlight *= 0.25;   ← 压暗方向只有 1/4 强度
         *   exponent = exp2(-2.0 * highlight);
         * TP+0x68 正 = 提亮高光、负 = 压暗（探针实测）。app 滑杆正 = 压暗
         * （恢复）= TP 负方向，所以 h = -滑杆值；app 参数与 SPP 滑杆值
         * 1:1（SPPWrite 十分位直写直读，preset(fromSPP:) /10 无缩放）。
         * 2026-10-22 前压暗侧用的是 exp2(1.5h) 拟合值、丢了 ×0.25 —— 按真身
         * 改回。提亮侧 exp2(2·h) 与真身逐字一致，不动。 */
        double h = -highlight;
        if (h < 0.0) h *= 0.25;
        double e = exp2(-2.0 * h);
        double x = pow((d - hi) / (1.0 - hi), e);
        d = hi + (1.0 - hi) * x;
    }
    return d < 0.0 ? 0.0 : d > 1.0 ? 1.0 : d;
}

/* 把这一组参数折成一张 4097 项的显示值 LUT。
 * 索引做 sqrt 压缩，暗部才有足够分辨率（线性值在暗部非常小）。 */
#ifndef X3F_HL_RECOVER_KNEE
#define X3F_HL_RECOVER_KNEE 0.15   /* 线性 knee：这以上的高光才做恢复 */
#endif
/* 展开上限：线性值到 TOP 时收成纯白。相机曲线本来就在 lin≈1.4 之后全部
 * 趋近 255，所以把 [knee, TOP] 摊到高光段不会丢掉任何"原本还看得见"的
 * 层次，只是把挤在最顶端的那些摊开。 */
#ifndef X3F_HL_RECOVER_TOP
#define X3F_HL_RECOVER_TOP  1.60
#endif

/*实验：直接使用 相机算法导出的 tone LUT（hp_on / hp_off）。
 * 数据文件不入库；没有文件时这一项自动回落。 */
static uint16_t g_spp_lut_on[16385];
static uint16_t g_spp_lut_off[16385];
static int g_spp_lut_loaded = -1;

static int spp_lut_load(void)
{
    const char *path = getenv("X3F_TONE_SPP_FILE");
    FILE *f;
    char line[128];
    if (g_spp_lut_loaded >= 0) return g_spp_lut_loaded;
    if (path == NULL || *path == '\0') path = "/private/tmp/spp/lut_ref.txt";
    f = fopen(path, "r");
    if (f == NULL) { g_spp_lut_loaded = 0; return 0; }
    while (fgets(line, sizeof(line), f) != NULL) {
        char tag[16];
        int idx, val;
        if (sscanf(line, "%15s %d %d", tag, &idx, &val) != 3) continue;
        if (idx < 0 || idx > 16384) continue;
        if (strcmp(tag, "hp_on") == 0) g_spp_lut_on[idx] = (uint16_t)val;
        else if (strcmp(tag, "hp_off") == 0) g_spp_lut_off[idx] = (uint16_t)val;
    }
    fclose(f);
    g_spp_lut_loaded = (g_spp_lut_on[16384] != 0 && g_spp_lut_off[16384] != 0);
    return g_spp_lut_loaded;
}

static double spp_lut_value(double lin, int unchecked, double scale)
{
    const uint16_t *tab = unchecked ? g_spp_lut_on : g_spp_lut_off;
    double x = lin * scale;
    int i;
    if (x <= 0.0) return 0.0;
    if (x >= 16384.0) return tab[16384] / 65535.0;
    i = (int)x;
    return (tab[i] + (tab[i + 1] - tab[i]) * (x - i)) / 65535.0;
}

/* tone_off_override：<0 = 按 x3f_render_oe_tone_off()；0/1 = 强制（边缘限幅
 * 构建勾选态基准 LUT 用，见 oe_edge_clip_on 注释）。只能传字面量，不做并发
 * 协议 —— 无全局状态，天然可重入。 */
static void build_tone_lut(const x3f_edit_params_t *p, uint16_t *lut,
                           int tone_off_override)
{
    int i;
    /* 仅 SDQH 机内链生效时关掉产品基础曲线（其余机型 use_base 与原式全等） */
    const int native_disp = sdqh_native_display();
    const int use_base = g_base_curve && !classic_native_strict() && !native_disp;
    /* 输出色彩空间（见 x3f_render.h 的 out_space）。
     *
     * 2026-11-04（全链路广色域）：**LUT 一律输出 sRGB 显示值**，与 osp 无关。
     * 逐通道观感曲线只在一处有定义 —— sRGB 域，那是它标定的地方；把同一张表
     * 拿去目标域直接查会把色相压歪（DP2Q2661 一块黄压成灰绿，差 12 级）。
     * 所以曲线永远在 sRGB 域走，转域放到整条显示域链的**最后一步**
     * （apply_row 末尾 / apply_spatial 之后），那里是双精度、能带负值。
     *
     * master 现在只用于一件事：SPP 外部标定表（X3F_TONE_SPP）的口径核对，
     * 它与 sRGB 绑定，广色域下不启用。 */
    const int osp = (p != NULL) ? p->out_space : 0;
    const int master = (osp != X3F_OUT_SRGB);
    double gain = pow(2.0, p->exposure);
    int spp_lut = (getenv("X3F_TONE_SPP") != NULL && spp_lut_load());
    double spp_scale = 16384.0;
    int spp_unchecked = (tone_off_override >= 0) ? tone_off_override
                                                 : x3f_render_oe_tone_off();
    {
        const char *e = getenv("X3F_TONE_SPP_SCALE");
        if (e != NULL && *e != '\0') spp_scale = atof(e);
    }
    /* 「色调曲线」四段调节（x3f_tonecurve.c）。
     * 显示域 u16→u16；四滑杆全 0 返回 0（恒等），整段跳过 = 零回归。
     * X3F_TC="sh,dk,li,hi,r2,r3,r4" 可覆盖（lab A/B，参数优先级低于宿主）。 */
    static uint16_t tc_lut[X3F_TONECURVE_LUT_SIZE];
    int tc_on = 0;
    {
        const char *e = getenv("X3F_TC");
        if (e != NULL && e[0] != '\0') {
            double v[7] = {0, 0, 0, 0, 64, 128, 192};
            int n = 0;
            char *s = strdup(e), *q = s;
            while (n < 7 && q != NULL) {
                char *nx = strchr(q, ',');
                if (nx != NULL) *nx = '\0';
                v[n++] = atof(q);
                q = (nx != NULL) ? nx + 1 : NULL;
            }
            free(s);
            x3f_tone_curve_build(v[0], v[1], v[2], v[3], v[4], v[5], v[6], tc_lut);
            tc_on = 1;
        } else {
            tc_on = x3f_tone_curve_build(p->tc[0], p->tc[1], p->tc[2], p->tc[3],
                                         p->tc[4], p->tc[5], p->tc[6], tc_lut);
        }
    }

    /* 基础曲线标定表（显示域）。
     *
     * 来历：拿 SPP 自己的 FL=0 导出反解"线性亮度 → 显示值"的映射，和我们的基础
     * 曲线逐档比（DP0Q1070，ISO 100，同批像素，26 个对数分箱）：
     *   线性 0.03/0.07/0.11/0.18/0.28/0.43/0.67/1.05 → 我们比 SPP 高
     *    +4.8/+7.2/+8.1/+8.9/+12.1/+16.5/+15.3/+6.6 灰阶；到接近白点反过来
     *    −5.5（我们 249 封顶，SPP 走到 254）。
     * 也就是**中间偏高、白点偏低**：这里按实测差值把显示值压回去，并把顶上抬一点。
     * 曲线形状（逐档斜率）本来就是对的，所以这只是一条很浅的标定，不动形状。
     * 强度用 X3F_BASE_CAL 覆盖（0 = 关掉，回到原样）。 */
    static const double calx[12] = {0.000, 0.170, 0.250, 0.400, 0.550, 0.660,
                                    0.760, 0.850, 0.940, 0.975, 0.999, 1.000};
    static const double caly[12] = {0.000, -0.012, -0.020, -0.020, -0.031, -0.035,
                                    -0.047, -0.063, -0.010, 0.000, 0.000, 0.000};
    /*默认关：亮段锚点已经按 实测映射改对了（index 24…31），这张补丁表
     * 的历史使命结束。关掉它之后整条 lin→显示 曲线与 偏差 ≤4 灰阶
     * （0.073 处 97.4/97.4、0.277 处 181.5/181.7、0.431 处 204.7/203.8）。
     * 留着只为 A/B 复现（X3F_BASE_CAL=1）。 */
    double cal = 0.0;
    {
        const char *e = getenv("X3F_BASE_CAL");
        if (e != NULL && e[0] != '\0') cal = atof(e);
    }
    /* 高光恢复的肩部基准（knee 之上的显示值从这里开始对数摊开） */
    const double rk = X3F_HL_RECOVER_KNEE;
    double rtop = X3F_HL_RECOVER_TOP;
    {
        /* A/B：上限越大，火焰核心越多层次能留下来（但整体更平） */
        const char *e = getenv("X3F_HL_RECOVER_TOP");
        if (e != NULL && *e != '\0') {
            double v = atof(e);
            if (v > rk) rtop = v;
        }
    }
    /* 严格原生档：相机曲线/基础曲线都不吃，高光恢复的锚点也按纯
     * 线性 → sRGB 编码取，保持与 x3fuse 原生输出域一致。 */
    const double rbase = srgb_encode(use_base
                                     ? tone_curve(p, rk) : rk);
    /* 窄肩部（X3F_HL_SHOULDER=1）：只接管 lin>1 的一小段。
     *
     * 基础曲线在 lin≈1.4 就压到 1.0，而实测 DP2Q2501 的火焰核心
     * 线性值最高到 1.73（Y 标准差 0.2~0.3）。这段被压平就是"有数据、
     * 没层次"的来源。这里把 [1.0, top] 映射到 [d1, 1)，单调、连续，
     * 不动 1.0 以下。 */
    double sh_top = 0.0, sh_d1 = 0.0, sh_knee = 1.0;
    {
        const char *e = getenv("X3F_HL_SHOULDER");
        if (e != NULL && *e != '\0' && *e != '0') {
            const char *t = getenv("X3F_HL_SHOULDER_TOP");
            const char *k = getenv("X3F_HL_SHOULDER_KNEE");
            sh_top = (t != NULL && *t != '\0') ? atof(t) : 1.8;
            sh_knee = (k != NULL && *k != '\0') ? atof(k) : 0.8;
            if (sh_top < sh_knee + 0.05) sh_top = sh_knee + 0.05;
            sh_d1 = srgb_encode(use_base
                                ? tone_curve(p, sh_knee) : sh_knee);
            if (sh_d1 > 0.999) sh_d1 = 0.999;
        }
    }

    /* 严格原生档：相机 auto-tone 曲线（65 点表 / v2 / SD1 专属表）整体
     * 退出，回到 x3fuse 原生的「线性 → 显示编码」。
     * 2 档保留早期 65 点表（v2 在 classic_tone_v2_enabled 里已被关）。 */
    int classic_on = g_classic_tone && g_classic_active
                  && !classic_native_strict();
    /* 基础曲线关掉时，相机曲线（含 classic 那三条）也必须一起关。
     *
     * 2026-11-05（引擎前置验证）：classic 是**相机内的 auto-tone 曲线**，
     * 和 kX3FBaseCurve 是同一层东西 —— 引擎前置那条链里相机曲线由胶片
     * 引擎的印相曲线代替，第二段只该施加用户滑杆。原来 `disp_lin` 那段
     * 用 g_base_curve 二选一，但 classic 分支排在它**前面**、绕过了这个
     * 判断，于是 SD14/SD1 会在第二段再吃一次相机曲线（双重曲线，画面
     * 被压暗两遍）。
     *
     * g_base_curve 默认 1，所以默认路径逐位不变；只有显式
     * x3f_render_set_base_curve(0) 才受影响 —— 而那个开关的语义本来
     * 就是"不做相机曲线"。 */
    if (!g_base_curve) classic_on = 0;
    {
        const char *ce = getenv("X3F_CLASSIC_TONE");
        if (ce != NULL && *ce != '\0' && atoi(ce) == 0) classic_on = 0;
    }
    int classic_v2_on = classic_on && classic_tone_v2_enabled();
    /* SPP「过曝矫正」勾选/未勾选的差值增益（kX3FOEToneGainIn）是**相对基础曲线**
     * 标定的口径 —— 它的参照物就是 kX3FBaseCurve 那条"勾选态"曲线。
     * Classic-Foveon 走的是相机内 auto-tone 曲线（上面 classic 分支把基础曲线
     * **整条替换**掉了，注释里也写明"不能叠在 tone_curve 上，否则双重 S 曲线"），
     * 再乘这条比值就是同一个错误换了个来源：那几条 classic 表都是
     * `camera = T(our_lin)` 的**端到端**拟合（SD14 三滑块 / SD1 固定表），下游
     * 的整条映射已经被表吸收，再乘一次等于重复施加，曲线被整体推亮、高光顶到
     * 250 以上。
     * 实测（SD14 sdim0092，App 全参数口径，对相机内嵌 JPEG）：
     *   乘这条比值   高光 MAE 17.57  死白 0.092%
     *   不乘          高光 MAE 5.89   死白 0.006%（相机 0.011%）
     * 故 classic 机型不吃这条差值。tone_off_override >= 0 是 oe_edge_clip 构建
     * "勾选态基准 LUT"用的强制值，照旧尊重。A/B 复现：X3F_OE_TONE=1 强制打开
     * （环境变量显式给了就不拦）。 */
    int oe_tone_on = (tone_off_override >= 0) ? tone_off_override
                                              : x3f_render_oe_tone_off();
    if (classic_on && tone_off_override < 0 && getenv("X3F_OE_TONE") == NULL)
        oe_tone_on = 0;
    /* 原生链同样不吃「过曝矫正勾选/未勾选」差值增益：那张表是相对
     * kX3FBaseCurve 标定的，曲线既已不施，增益也就没有参照物。
     * tone_off_override >= 0 是 edge_clip 构建基准 LUT 的强制值，照旧尊重。 */
    if (g_classic_native == 1 && tone_off_override < 0
        && getenv("X3F_OE_TONE") == NULL)
        oe_tone_on = 0;
    for (i = 0; i <= X3F_TONE_LUT_SIZE; i++) {
        double t = (double)i / X3F_TONE_LUT_SIZE;
        double lin = t * t * X3F_LIN_MAX;
        double d;

        lin *= gain;                                   /* 曝光 */
        if (spp_lut && !master) {
            d = spp_lut_value(lin, spp_unchecked, spp_scale);
            if (d < 0.0) d = 0.0;
            if (d > 1.0) d = 1.0;
            lut[i] = (uint16_t)(d * 65535.0 + 0.5);
            continue;
        }
        {
            /* Classic-Foveon 机型：相机内 auto-tone 曲线**取代**基础曲线，
             * 与 vendor(process.rs apply_classic_tone_to_lut) 语义严格一致：
             *   显示值 = srgb_encode(classic_tone_curve(lin, 三滑块))
             * 不能叠在 tone_curve(p,lin) 之上（那会双重量化/双重 S 曲线，
             * 把已接近相机的基础曲线再推离相机 JPEG）。非 classic 机型走原
             * 基础曲线，观感与回归铁律不变。 */
            double disp_lin;
            /* 相机观感曲线：**sRGB 路径与广色域路径走同一条**。
             *
             * 以前广色域走"母版语义"（跳过曲线，直接线性 → 目标域 γ），
             * 结果是导出比预览暗中间调、发灰、饱和度掉（DP2Q2501 中间调
             * 低 25 级）。曲线搬去目标域又会让色相漂。两个都不对。
             *
             * 正确的做法是**顺序**：曲线留在它标定的 sRGB 域里走完，转域
             * 作为线性矩阵放到最后。见 x3f_render.h 的 x3f_render_reencode
             * 注释与 apply_row 末尾的转域段。 */
            if (classic_on && g_classic_sd1)
                disp_lin = classic_tone_sd1_curve(lin);
            else if (classic_v2_on)
                disp_lin = classic_tone_v2_curve(lin);
            else if (classic_on)
                disp_lin = classic_tone_curve(lin, g_classic_hilite, g_classic_shadow, g_classic_contrast);
            else
                /* 严格原生档：不叠任何对比度/相机影调曲线，等价于
                 * x3fuse 原生 convert_data 的 matrix → ev_scale → 显示 gamma。 */
                disp_lin = use_base
                         ? tone_curve(p, lin) : lin;
            d = lut_encode(osp, disp_lin);
        }

        /* 高光肩：锚点（rbase / sh_d1）是按 sRGB 域曲线标定的，而曲线现在
         * 无论输出哪个域都在 sRGB 域走（见上），所以广色域也照用。 */
        if (sh_top > 0.0 && lin > sh_knee) {
            double t = (lin - sh_knee) / (sh_top - sh_knee);
            double s;
            if (t > 1.0) t = 1.0;
            s = (1.0 - exp(-3.0 * t)) / (1.0 - exp(-3.0));
            d = sh_d1 + (1.0 - sh_d1) * s;
        }

        if (cal != 0.0) {                              /* 基础曲线标定 */
            int k = 0;
            while (k < 10 && d > calx[k + 1]) k++;
            if (d >= calx[11]) k = 10;
            {
                double t = (d - calx[k]) / (calx[k + 1] - calx[k]);
                if (t < 0.0) t = 0.0;
                if (t > 1.0) t = 1.0;
                d += cal * (caly[k] + (caly[k + 1] - caly[k]) * t);
            }
            if (d < 0.0) d = 0.0;
            if (d > 1.0) d = 1.0;
        }

        /* 高光恢复：把 [knee, TOP] 这段线性高光用 smoothstep 摊到
         * [rbase, 1] 的显示范围里——相机曲线把这段压成几个灰阶，这里把它
         * 铺开成几十个，过曝的云层层次就回来了。knee 以下完全不动。
         * 这就是 SPP「高光恢复」在做的事：不用整体压曝光也能把高光拉出来。 */
        if (p->hl_recover > 0.0 && lin > rk) {
            double ds = rbase + (1.0 - rbase) * sstep01(rk, rtop, lin);
            d += (ds - d) * p->hl_recover;
        }

        d = apply_contrast(d, p->contrast);            /* 对比度 */
        /* 阴影/高光不在这里逐通道做：apply_spatial() 在显示亮度上做一次，
         * 再同比缩放 RGB，既保留色比，也避免三通道各自在 25%/75% 边界
         * 跨段造成的偏色。 */

        /* SPP「过曝矫正」未勾选那条色调曲线（见表上方的说明）。
         * 增益表的口径是**线性色调值**（= SPP 色调 LUT 的输出），而流水线走到
         * 这里已经是显示值了（基础曲线、高光恢复、对比度都在显示域），所以先
         * 解回线性、乘增益、再编码回去 —— 直接乘显示值等于在 gamma 域乘，
         * 高光会被放大到两三倍（实测过）。classic 机型见 oe_tone_on 处说明。 */
        if (oe_tone_on) {
            /*两条 LUT 都是"输入 → 色调值"，比值就该按输入取；
             * 我们的线性域和 输入码只差一个饱和点标定，取 1.4
             * （基础曲线标定时用的相机白点，见上面对 kX3FBaseCurve 的说明）。 */
            double dl = lut_decode(osp, d) * oe_tone_gain_in(lin / X3F_OE_LIN_WHITE);
            d = lut_encode(osp, dl);
        }

        /* 「色彩模式」每档自带的那条色调曲线（表见 x3f_colormode_tone.h，
         * 生成文件；没有表时 tone_row 恒为 -1，这块整个不参与）。
         * 与上面那条同一个口径：增益按**输入**（线性值 / 1.4）取，
         * 在显示值与线性值之间换算一次再乘。标准/鲜艳/中性/肖像/FOV 黄 五档
         * 带自己的小 S 形，其余各档（含风景）增益恒为 1 —— 那几档逐字节不变。 */
        if ((g_classic_native == 0 || g_classic_native == 3)
            && p->color.tone_row >= 0) {
            double g = cmtone_gain(p->color.tone_row, p->color.tone_w,
                                   lin / X3F_OE_LIN_WHITE);
            if (g != 1.0) {
                double dl = lut_decode(osp, d) * g;
                d = lut_encode(osp, dl);
            }
        }

        /* SPP「色调曲线」：作用于整条色调链之后的显示值（与曲线编辑器
         * 预览语义一致 —— 编辑器直方图画的就是当前输出的显示值）。
         * 全 0 时 tc_on=0，一个分支都不进。 */
        if (tc_on) {
            int ti = (int)(d * 65535.0 + 0.5);
            if (ti < 0) ti = 0;
            if (ti > 65535) ti = 65535;
            d = tc_lut[ti] / 65535.0;
        }

        if (d < 0.0) d = 0.0;
        if (d > 1.0) d = 1.0;
        lut[i] = (uint16_t)(d * 65535.0 + 0.5);
    }
}

/* ---- 「机内直出对齐」校正曲线（显示域 in/out，逐通道）----------------
 *
 * 来历：5 张跨机型跨场景样张（DP2Q1884 濑户内海 / DP2Q2051、DP2Q2054 甘南 /
 * DP0Q1518 春季 / DP0Q1884 21:9）的产品默认出图 vs X3F 内嵌 JPEG（机内直出，
 * 几何逐像素配对）显示域分箱 median 拟合，再做单调化与轻平滑。
 * 拟合与验证工具：实验室 x3fdec-app/align/fit_prod_look.py，
 * 表由其产物 prod_look.json 生成（**勿手抄**，重拟合后用脚本重新生成）。
 *
 * 实测（5 样张，滑杆全 0 默认参数）：校正前总差 117 灰阶（全体一致偏暗
 * 1…3.5、对比/范围差 3…14），校正后 29 灰阶，逐样张残差全部 ≤2.4。
 * 校正量本身很小（中间调 +2…+8/255），作用是收掉与机内直出的系统性影调差。
 *
 * 口径：作用在整条色调链（基础曲线/高光恢复/对比度/过曝矫正/色彩模式曲线）
 * 之后的**显示值**上，逐通道一张表。滑杆非 0 时它仍然只是浅浅的锚——
 * 不改变滑杆的方向与手感。X3F_ALIGN_LOOK=0 整段关掉（回到原曲线）。 */
#define ALIGN_LOOK_NPT 17
static const double ALIGN_LOOK_X[ALIGN_LOOK_NPT] = {
    0.0, 15.9375, 31.875, 47.8125, 63.75, 79.6875, 95.625, 111.5625,
    127.5, 143.4375, 159.375, 175.3125, 191.25, 207.1875, 223.125,
    239.0625, 255.0 };
static const double ALIGN_LOOK_R[ALIGN_LOOK_NPT] = {
    0.0, 15.4296875, 30.5520833, 44.5104167, 58.1041667, 74.6614583,
    91.9270833, 109.9713542, 129.5208333, 149.6302083, 167.9895833,
    183.8411458, 198.6458333, 212.5234375, 225.8072917, 239.4453125, 255.0 };
static const double ALIGN_LOOK_G[ALIGN_LOOK_NPT] = {
    0.0, 13.9375, 28.5520833, 43.1614583, 57.59375, 74.4817708,
    92.2604167, 110.4895833, 129.7083333, 149.4869792, 167.65625,
    184.0364583, 198.1770833, 212.5885417, 225.8072917, 238.6796875, 255.0 };
static const double ALIGN_LOOK_B[ALIGN_LOOK_NPT] = {
    0.0, 19.9375, 35.21875, 48.6848958, 61.7708333, 77.8411458,
    93.9583333, 111.4895833, 131.0416667, 150.9635417, 169.796875,
    186.1744792, 200.375, 213.4557292, 227.2708333, 239.296875, 255.0 };

static int align_look_on(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("X3F_ALIGN_LOOK");
        /* 缺省开；只有显式设成数值 0（"0"/"00"…）才关 */
        v = (e != NULL && *e != '\0' && atoi(e) == 0) ? 0 : 1;
    }
    return v;
}

static double align_look_map(const double *y, double d)
{
    double x = d * 255.0;
    int k = 1;
    if (x <= 0.0) return y[0] / 255.0;
    if (x >= 255.0) return y[ALIGN_LOOK_NPT - 1] / 255.0;
    while (k < ALIGN_LOOK_NPT - 1 && ALIGN_LOOK_X[k] < x) k++;
    {
        double t = (x - ALIGN_LOOK_X[k - 1])
                 / (ALIGN_LOOK_X[k] - ALIGN_LOOK_X[k - 1]);
        return (y[k - 1] + (y[k] - y[k - 1]) * t) / 255.0;
    }
}

/* 把逐通道校正折进 LUT：src 是 build_tone_lut 的产物，三个输出各折各的。 */
static void fold_align_look(const uint16_t *src,
                            uint16_t *outr, uint16_t *outg, uint16_t *outb)
{
    int i;
    for (i = 0; i <= X3F_TONE_LUT_SIZE; i++) {
        double d = (double)src[i] / 65535.0;
        outr[i] = (uint16_t)(align_look_map(ALIGN_LOOK_R, d) * 65535.0 + 0.5);
        outg[i] = (uint16_t)(align_look_map(ALIGN_LOOK_G, d) * 65535.0 + 0.5);
        outb[i] = (uint16_t)(align_look_map(ALIGN_LOOK_B, d) * 65535.0 + 0.5);
    }
}

/* 线性值 -> LUT 索引（带小数，用于线性插值） */
static inline double lin_to_index(double v)
{
    double t;
    if (v <= 0.0) return 0.0;
    t = sqrt(v / X3F_LIN_MAX);
    if (t >= 1.0) return X3F_TONE_LUT_SIZE;
    return t * X3F_TONE_LUT_SIZE;
}

/* ---------- 逐像素套 LUT ---------- */

/* 8 位 / 16 位输出共用同一套读写（值域都归一化到 0..1）。
 * idx 是**采样序号**（= y*stride 之后按通道数数的那个下标），两种位深同一套
 * 下标，由 cast 处理宽度 —— 早先按"字节偏移"传过一次，16 位下每个像素写到
 * 了前一格，整行只写满一半，引以为戒。 */
static inline double px_get(const void *base, int bits, size_t idx)
{
    if (bits == 32) return (double)((const float *)base)[idx];
    if (bits == 8) return ((const uint8_t *)base)[idx] / 255.0;
    return ((const uint16_t *)base)[idx] / 65535.0;
}

/* bits == 32：把**未钳制的 float 场景线性**直接落到 float 缓冲（HDR 头子
 * 不被 [0,1] 切掉）。仅用于「观感延后」喂胶片引擎的路径（Fotufilm 的
 * halation 物理上由 >1.0 的高光溢出驱动，16 位钳制会让它长不出来）。
 * 其余路径（8 / 16 位）语义不变。 */
static inline void px_put(void *base, int bits, size_t idx, double v)
{
    if (bits == 32) { ((float *)base)[idx] = (float)v; return; }
    if (v < 0.0) v = 0.0;
    if (v > 1.0) v = 1.0;
    if (bits == 8) ((uint8_t *)base)[idx] = (uint8_t)(v * 255.0 + 0.5);
    else           ((uint16_t *)base)[idx] = (uint16_t)(v * 65535.0 + 0.5);
}

typedef struct {
    const float *lin;
    const float *clip;    /* 每像素 RAW 饱和分（0..3），可空 */
    void *out;
    int w, h, stride;
    int bits;             /* 8 或 16 */
    const uint16_t *lut;  /* 亮度索引处（fill/hl 保色分支）用 R 这张 */
    const uint16_t *lutg; /* 「机内直出对齐」逐通道 LUT（未开时 = lut） */
    const uint16_t *lutb;
    double sat;
    int use_sat;
    double hl_desat;      /* 高光去饱和强度 0..1 */
    double hl_recover;    /* 高光恢复强度（只给"保色版"用） */
    /* 高光保色（2026-10-31，X3F_HL_CHROMA，默认 1.0 = 逐位 no-op）：
     * 显示域亮部的色度乘子，见 apply_row 里那段注释。 */
    double hl_chroma;
    /* 上面那个乘子的作用窗（亮度 0..1，X3F_HL_CHROMA_WIN）与色相闸
     * （X3F_HL_CHROMA_HUE="lo,hi"，度，软边 10°）。默认窗 0.78..0.93、
     * 色相闸关 —— 两者都不设时与只有 hl_chroma 时逐位相同。 */
    double hc_win_lo, hc_win_hi;
    int    hc_hue_on;
    double hc_hue_lo, hc_hue_hi;
    /* 中亮带通保色（2026-11-05，**默认开 K=1.30**）：
     * 把「单调上升的高光窗」换成「钟形中亮窗」——屋檐下的黄墙恰恰在
     * Y 200..230 缺色度（偏粉/偏红），而 Y>=240 反而色度偏多。上升窗
     * 只会把已经过黄的极亮段推得更黄，所以不能复用。hc_band_k<=1.0 时
     * 整段不进（此时与加这个功能前逐位相同）。字段只在本文件内部，
     * 不动 x3f_edit_params_t 的 ABI。 */
    double hc_band_k;          /* >1.0 启用；默认 1.30 */
    double hc_band_lo1, hc_band_lo2, hc_band_hi1, hc_band_hi2;
    int    hc_band_hue_on;
    double hc_band_hue_lo, hc_band_hue_hi;
    const float *fillgain;/* 每像素的填充光增益，可空 */
    float gain[3];        /* 白平衡：线性空间的通道增益 */
    int use_gain;
    /* SPP 色彩模式的矩阵（线性 sRGB 域，9 个 double，行主序），NULL = 不乘。
     * 位置在 tone LUT 之前 —— 与 SPP 把 CMCM 折进白平衡矩阵是同一个环节。 */
    const double *cm;
    /*黑白（ B&W 页签），NULL = 彩色。见 x3f_render.h 里 mono 那段。*/
    const struct mono_ctx_s *mono;
    /* 色彩分离（参照 Lightroom Split Toning）：split_setup() 预换算好的
     * 色差向量与分区参数。use_split=0 时 apply_row 里整块不进 ——
     * 两路强度都为 0 就是逐位 no-op（铁律）。 */
    int use_split;
    double split_cs[3];    /* 暗部色差向量 (T_c − T_601)·sat，601 域保亮度 */
    double split_ch[3];    /* 亮部色差向量 */
    double split_center;   /* 明度分界中心（0..1） */
    double split_half;     /* 过渡半宽（0..0.5） */
    /* 【边缘限幅（X3F_OE_EDGE_CLIP / x3f_render_set_oe_edge_clip），2026-10-30】
     * 勾选态基准 LUT 三张（与 lut/lutg/lutb 同构，未勾选差值不施加）+ 边缘
     * 掩膜（u8 0..255，NULL = 功能关）。掩膜>0 的像素两套 LUT 线性插值：
     * m=0 逐位 = 现行为（铁律），m=1 = 勾选态处理 ≈ SPP。 */
    const uint16_t *elut;
    const uint16_t *elutg;
    const uint16_t *elutb;
    const uint8_t *edge_mask;
    /* 输出色彩空间（X3F_OUT_SRGB / X3F_OUT_PROPHOTO）。0 = 现行 sRGB 路径，
     * 逐位零回归；1 = ProPhoto：过 LUT 前**不夹**负值，先把线性 RGB 转到
     * ProPhoto 线性域（sRGB 色域外的颜色在那里就是正的），见 apply_row。 */
    int out_space;
    /* 观感延后（x3f_edit_params_t.look_defer）：apply_row 只做到**线性 sRGB**
     * 就写像素，跳过曲线 / 对比 / 饱和 / 色调曲线 / 分离色调 / 空间处理。
     * 0 = 现行为（逐位零回归）。见 x3f_render.h 的 look_defer 段。 */
    int look_defer;
    /* 曝光增益（2^exposure）。正常链里它被折进 tone LUT 的建表输入
     * （build_tone_lut 的 `lin *= gain`），延后模式下 LUT 整段不走，
     * 所以要把这个常数单独带进来，在写像素前乘上 —— 否则延后路径会
     * 静默丢掉曝光补偿。非延后路径不用它。 */
    float exp_gain;
} apply_ctx_t;

/* 高光恢复的保色版（X3F_HL_Y_ONLY=1）：按亮度查同一个 LUT，再按比例缩放
 * 三通道 —— 只改明暗、不改色比。逐通道查表会把高光的色相拉开（绿/粉环）。 */
static int hl_y_only(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("X3F_HL_Y_ONLY");
        v = (e != NULL && *e != '\0' && *e != '0') ? 1 : 0;
    }
    return v;
}

/* 填充光的保色版（X3F_FL_HUEPRES=1）：填充光把皮肤/木头推进高光肩以后，
 * 逐通道查同一个 tone LUT 会把 R 压得比 G/B 多 —— 色比被拉开，
 * 手上那块从暖肤色变成灰粉（G/R 从 0.66 升到 0.79，SPP 同区域仍是 0.65）。
 * 这里改为「按亮度查表、三通道同比例缩放」，色比保持不变；只有单通道
 * 将要越过 1.0 时才朝该亮度做最小必要的去饱和。fill=0 时 fillgain 为 NULL，
 * 这条分支根本不进，仍是严格 no-op。 */
static int fill_hue_preserve(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("X3F_FL_HUEPRES");
        v = (e != NULL && *e != '\0' && *e != '0') ? 1 : 0;
    }
    return v;
}

/*填充光的显示域应用（X3F_FL_DOMAIN=display，实验）： Retinex map 是
 * 在 Stage3 那一步消费的，也就是**已经过相机色调曲线之后**乘上去。我们先
 * 乘在线性域再查逐通道 LUT，色比会被高光肩重新拉开（手掌 G/R 0.66→0.79）；
 * 按 顺序改成显示域乘，三通道同倍率，色比保持。*/
static int fill_display_apply(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("X3F_FL_DOMAIN");
        v = (e != NULL && e[0] == 'd') ? 1 : 0;
    }
    return v;
}

/* 填充光的色差补偿（X3F_FL_SATLIFT，默认 0.015）：线性乘完填充增益再查
 * 逐通道 LUT，暗部的色比会被拉平（阳光橙木 B/R 0.041→0.18，SPP 0.070）。
 * 补偿量按等效增益大小走：中性色不动，提得越狠的暗部补得越多。
 * 2446 的 App 16 位导出实测 0.015 让橙木 B 11.3（SPP 12.8，补偿前 20.2），
 * 手掌区 B 51.8（SPP 49.8）；0337 整幅变化 <0.6 个码值。0 = 关。 */
static double fill_satlift(void)
{
    static double v = -1.0;
    if (v < 0.0) {
        const char *e = getenv("X3F_FL_SATLIFT");
        v = (e != NULL && e[0] != '\0') ? atof(e) : 0.015;
        if (v < 0.0) v = 0.0;
    }
    return v;
}

/* 当前这次 apply 的 hl_recover（apply_common 拿不到 params，用这个传） */
static double g_hl_recover_cur = 0.0;

/* 显示域 3×4 后校正（实验，X3F_COLOR_FIT=1）。
 * 由 base 渲染与 SPP TIFF 在同一裁切上拟合；默认关。 */
static int color_fit_enabled(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("X3F_COLOR_FIT");
        v = (e != NULL && *e != '\0' && *e != '0') ? 1 : 0;
    }
    return v;
}

/* 黑白那一步的预换算结果：
 *   w[3]    = 色彩混合器三个权重（已归一化，和为 1）
 *   tint[3] = 色调的保亮度三通道增益（t_c / (0.3·tR + 0.59·tG + 0.11·tB)） */
typedef struct mono_ctx_s {
    double w[3];
    double tint[3];
} mono_ctx_t;

/*由参数算 mono_ctx_t：mix 全 0 → 等权 1/3（与 XDoColorMixing 一致）；
 * tint 全 0 或全相等 → 增益 1（等价"不调整"）。 */
static void mono_setup(const x3f_edit_params_t *p, mono_ctx_t *m)
{
    double sum, lum;
    m->w[0] = p->mono_mix_r;
    m->w[1] = p->mono_mix_g;
    m->w[2] = p->mono_mix_b;
    sum = m->w[0] + m->w[1] + m->w[2];
    if (!(sum > 0.0)) {
        m->w[0] = m->w[1] = m->w[2] = 1.0 / 3.0;
    } else {
        m->w[0] /= sum; m->w[1] /= sum; m->w[2] /= sum;
    }
    {
        double t[3];
        t[0] = p->mono_tone_r;
        t[1] = p->mono_tone_g;
        t[2] = p->mono_tone_b;
        lum = 0.3 * t[0] + 0.59 * t[1] + 0.11 * t[2];
        if (!(lum > 0.0)) {
            m->tint[0] = m->tint[1] = m->tint[2] = 1.0;
        } else {
            m->tint[0] = t[0] / lum;
            m->tint[1] = t[1] / lum;
            m->tint[2] = t[2] / lum;
        }
    }
}

/* ---- 色彩分离（参照 Lightroom 的 Split Toning / Color Grading）---- */

/* 色相 → 全饱和 RGB（v=1, s=1）。h 单位度，负数/超 360 都按模处理。 */
static void split_hue_to_rgb(double h, double rgb[3])
{
    double f, p, q, t;
    int i;
    h = fmod(h, 360.0);
    if (h < 0.0) h += 360.0;
    h /= 60.0;
    i = (int)h;
    f = h - i;
    p = 0.0; q = 1.0 - f; t = 1.0 - (1.0 - f);
    switch (i % 6) {
        case 0: rgb[0] = 1.0; rgb[1] = t; rgb[2] = p; break;
        case 1: rgb[0] = q;   rgb[1] = 1.0; rgb[2] = p; break;
        case 2: rgb[0] = p;   rgb[1] = 1.0; rgb[2] = t; break;
        case 3: rgb[0] = p;   rgb[1] = q;   rgb[2] = 1.0; break;
        case 4: rgb[0] = t;   rgb[1] = p;   rgb[2] = 1.0; break;
        default: rgb[0] = 1.0; rgb[1] = p;  rgb[2] = q; break;
    }
}

/* 参数 → apply_ctx 的预换算。p 为 NULL（对照入口）或两路强度都为 0
 * → use_split=0（逐位 no-op）。 */
static void split_setup(const x3f_edit_params_t *p, apply_ctx_t *ctx)
{
    const double sh = p ? p->split_sh_sat : 0.0;
    const double hl = p ? p->split_hl_sat : 0.0;
    ctx->use_split = (sh > 0.0 || hl > 0.0) ? 1 : 0;
    if (!ctx->use_split) return;
    {
        double ts[3], th[3];
        double lys, lyh;
        int i;
        split_hue_to_rgb(p->split_sh_hue, ts);
        split_hue_to_rgb(p->split_hl_hue, th);
        /* 601 亮度 —— 与本链饱和度那一步同一个口径（SPP 用 601） */
        lys = 0.299 * ts[0] + 0.587 * ts[1] + 0.114 * ts[2];
        lyh = 0.299 * th[0] + 0.587 * th[1] + 0.114 * th[2];
        for (i = 0; i < 3; i++) {
            ctx->split_cs[i] = (ts[i] - lys) * sh;
            ctx->split_ch[i] = (th[i] - lyh) * hl;
        }
    }
    /* 分界中心按 balance 挪（±0.4：不至于把暗部压到只剩纯黑），
     * 过渡半宽按 blending 展开：0 = ±0.05 的窄过渡（近乎硬分界），
     * 1 = ±0.5（全程从黑到白渐变，两个色轮同时起作用）。 */
    ctx->split_center = 0.5 + 0.4 * p->split_balance;
    ctx->split_half = 0.05 + 0.45 * p->split_blending;
    if (ctx->split_center < 0.1) ctx->split_center = 0.1;
    if (ctx->split_center > 0.9) ctx->split_center = 0.9;
}

/* SD1 保色度施加的开关（与 build_tone_lut 的 classic_on 同一判据）。
 * env 只在首次调用读一次，别放进像素循环。 */
static int sd1_chroma_on(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("X3F_CLASSIC_TONE");
        v = (g_classic_sd1 && !(e != NULL && *e != '\0' && atoi(e) == 0)) ? 1 : 0;
    }
    return v;
}

/* 色相闸（软边 10°）：返回 0..1 的权重，无彩度像素返回 0（乘子没有意义）。
 * 算式与 2026-10-31 探测版逐位相同，抽出来给「中亮带通」和「高光上升窗」
 * 两段共用，避免同一段 hue 计算写两遍后慢慢走偏。 */
static double hue_gate_w(const double d[3], double lo, double hi)
{
    double mx = d[0] > d[1] ? (d[0] > d[2] ? d[0] : d[2])
                            : (d[1] > d[2] ? d[1] : d[2]);
    double mn = d[0] < d[1] ? (d[0] < d[2] ? d[0] : d[2])
                            : (d[1] < d[2] ? d[1] : d[2]);
    double ch = mx - mn;
    double hue;
    if (ch <= 1e-6) return 0.0;
    if (mx == d[0])      hue = 60.0 * (d[1] - d[2]) / ch;
    else if (mx == d[1]) hue = 60.0 * ((d[2] - d[0]) / ch + 2.0);
    else                 hue = 60.0 * ((d[0] - d[1]) / ch + 4.0);
    if (hue < 0.0) hue += 360.0;
    {
        double soft = 10.0;
        return sstep01(lo - soft, lo + soft, hue) *
               (1.0 - sstep01(hi - soft, hi + soft, hue));
    }
}

static void apply_row(void *vctx, size_t row)
{
    const apply_ctx_t *c = (const apply_ctx_t *)vctx;
    const float *src = c->lin + row * (size_t)c->w * 3;
    void *dst = (uint8_t *)c->out + row * (size_t)c->stride;
    int x;

    for (x = 0; x < c->w; x++) {
        double d[3];
        double yl = 0.0;
        int i;

        /* ① 线性域：白平衡增益 / 填充光 → 色彩模式矩阵 → tone LUT。
         * 色彩模式放在这里，是因为 CMCM 就是乘在**线性** RGB 上的
         * （`GetColorModeContrastCompensation` 把它折进白平衡矩阵，整条再进
         * 色调链）。矩阵会出负值，过 LUT 前按 0 夹掉 —— SPP 那边同样如此。 */
        {
            double lv[3];
            double fg = 1.0;
            for (i = 0; i < 3; i++) {
                double v = src[i];
                if (c->use_gain) v *= c->gain[i];
                if (c->fillgain) {
                    fg = c->fillgain[row * (size_t)c->w + (size_t)x];
                    if (!fill_display_apply()) v *= fg;
                }
                lv[i] = v;
            }
            if (c->cm != NULL) {
                double a = lv[0], b = lv[1], e = lv[2];
                lv[0] = a * c->cm[0] + b * c->cm[1] + e * c->cm[2];
                lv[1] = a * c->cm[3] + b * c->cm[4] + e * c->cm[5];
                lv[2] = a * c->cm[6] + b * c->cm[7] + e * c->cm[8];
                {
                    static int cm_method = -1;
                    if (cm_method < 0) {
                        const char *m = getenv("X3F_CM_METHOD");
                        cm_method = (m != NULL && m[0] == '0') ? 0 : 1;
                    }
                    if (cm_method == 1) {
                        /* 亮度失真门控：仅对"大幅挪动亮度"的离群矩阵施加保亮 +
                         * gamut 压缩。FCBlue 绿系数 2.85 会把自然绿顶到黄白、蓝压到
                         * 近黑 —— 即"画质崩坏"的 washout；把矩阵后的亮度还原到矩阵前
                         * 的亮度，只保留色相/饱和变化，与 平滑观感一致。
                         * 温和档（Vivid/Standard/Neutral/Cinema…）矩阵本就近似保亮，
                         * ld≤0.5 → 跳过 → 与原实现逐位一致，零回归。
                         * X3F_CM_METHOD=0 退回旧的"直接乘"行为（供 A/B）。 */
                        double g0 = 0.2126*c->cm[0] + 0.7152*c->cm[3] + 0.0722*c->cm[6];
                        double g1 = 0.2126*c->cm[1] + 0.7152*c->cm[4] + 0.0722*c->cm[7];
                        double g2 = 0.2126*c->cm[2] + 0.7152*c->cm[5] + 0.0722*c->cm[8];
                        double ld = fmax(fabs(g0-0.2126),
                                         fmax(fabs(g1-0.7152), fabs(g2-0.0722)));
                        if (ld > 0.5) {
                            double yin = 0.2126*a + 0.7152*b + 0.0722*e;
                            double yout = 0.2126*lv[0] + 0.7152*lv[1] + 0.0722*lv[2];
                            if (yout > 1e-6) {
                                double k = yin / yout;
                                lv[0] *= k; lv[1] *= k; lv[2] *= k;
                            }
                            /* 有意对所有 out_space 都收口，理由见函数头 2026-11-10
                             * 的核查记录：广色域档跳过会让强色彩模式画质崩坏。 */
                            cm_gamut_compress(&lv[0], &lv[1], &lv[2]);
                        }
                    }
                    /* sRGB 原行为：矩阵之后把负值夹 0（SPP 同样如此）。
                     * 位置保持在这里 —— 只在有矩阵时执行，零回归。 */
                    if (c->out_space == X3F_OUT_SRGB)
                        for (i = 0; i < 3; i++) if (lv[i] < 0.0) lv[i] = 0.0;
                }
            }
            /* 输出色彩空间：这里**不再**转域（2026-11-04，全链路广色域）。
             *
             * 以前在这个位置 out_lin_convert 到目标域，然后拿目标域的数值去
             * 查 sRGB 域标定的曲线 LUT —— 同一块颜色走的是另一组工作点，
             * 于是色相被压歪（DP2Q2661 一块黄压成灰绿，差 12 级），当时的
             * 补救是"广色域干脆跳过曲线"，结果观感与预览差 25 级。
             *
             * 正解是**顺序**：曲线留在 sRGB 域（它标定的地方）走完全链，
             * 转域作为线性矩阵放到最后一步（apply_row 末尾）。那里数据还是
             * double，负值 / >1 的线性余量能原样带过去 —— 这才是"宽容器里
             * 真的装了宽内容"。 */
            /* 填充光的保色补偿（X3F_FL_SATLIFT，实验）：等效增益越大，
             * 线性域里三通道被推得越高，逐通道 LUT 之后色比被拉平得越厉害
             * （阳光下的橙木 G/R 0.41→0.79，SPP 是 0.60）。这里按增益大小
             * 给线性色差一个反向补偿；中性色不动，暗部提升越大补得越多。 */
            if (c->fillgain != NULL && fg > 1.0) {
                double alpha = fill_satlift();
                if (alpha > 0.0) {
                    double y0 = 0.2126 * lv[0] + 0.7152 * lv[1] + 0.0722 * lv[2];
                    double k = 1.0 + alpha * (fg - 1.0);
                    for (i = 0; i < 3; i++) {
                        lv[i] = y0 + (lv[i] - y0) * k;
                        /* 广色域不夹负值：sRGB 原色下的负值就是色域外的真实
                         * 颜色，后面由奇对称外推 + 转域矩阵带到目标域去。 */
                        if (c->out_space == X3F_OUT_SRGB && lv[i] < 0.0) lv[i] = 0.0;
                    }
                }
            }
            yl = 0.2126 * lv[0] + 0.7152 * lv[1] + 0.0722 * lv[2];
            /* ---- 观感延后（2026-11-05，最小验证）---------------------------
             * 到此为止 lv[] 是**场景线性 sRGB**：WB 增益、填充光、色彩模式
             * 矩阵都做完了，还没碰任何显示域曲线。这一支就停在这里，把线性
             * 值直接写进像素（值 = 线性，不是显示值），供「先跑胶片引擎、
             * 再施加观感」的对照实验取中间结果。
             *
             * 上面那条 fill_hue_preserve 分支不能用：它要查 tone LUT（按亮度
             * 查表再按比例缩放），那已经是显示域的事了，延后模式下必须整体
             * 跳过 —— 否则等于把曲线偷偷施加了一次。
             *
             * px_put 会夹 [0,1]：16bit 线性存储的物理上限，高光余量在这里
             * 丢掉（引擎的输入本来就是 0..1 的场景光）。 */
            if (c->look_defer) {
                /* 【诊断/标定】env X3F_SF_GAIN=<倍数>：给喂进胶片的场景线性
                 * 加一个整体增益。用来判「引擎是不是自动测光」——如果加了
                 * 增益输出几乎不动，说明 auto_exposure 把曝光吃掉了，那么
                 * 「比 SpektraLab 暗」就不是输入电平的问题。默认 1.0。 */
                static double sf_gain = -1.0;
                if (sf_gain < 0.0) {
                    const char *ge = getenv("X3F_SF_GAIN");
                    sf_gain = (ge != NULL && *ge != '\0') ? atof(ge) : 1.0;
                    if (!(sf_gain > 0.0)) sf_gain = 1.0;
                }
                const double eg = (double)c->exp_gain * sf_gain;
                double v[3];
                for (i = 0; i < 3; i++) v[i] = lv[i] * eg;
                /* 广色域前置（2026-11-05）：先把场景线性从 sRGB 原色换到目标域
                 * 再落地。px_put 只认 [0,1]，而 sRGB 原色下"超出 sRGB 三角形"
                 * 的真实颜色是负值 —— 直接写就等于在门口删掉，之后再怎么转域
                 * 都是宽容器窄内容。先换域，那些颜色在 ProPhoto/P3 里本来就是
                 * 正的，夹持只剩 >1 的高光余量（引擎输入本就要 0..1）。 */
                if (c->out_space != X3F_OUT_SRGB)
                    out_lin_convert(c->out_space, v);
                for (i = 0; i < 3; i++)
                    px_put(dst, c->bits, (size_t)x * 3 + (size_t)i, v[i]);
                src += 3;
                continue;
            }
            if (c->fillgain != NULL && fill_hue_preserve()) {
                /* 填充光保色：LUT 只查一次（亮度），三通道按同比例缩放。
                 * lv 上已经有逐像素的 fillgain，所以这一步只改“查表方式”，
                 * 不再改动填充光本身的增益曲线。 */
                double idx = lin_to_index(yl);
                int i0 = (int)idx;
                double f = idx - i0, a, b, ymap, sc, mx = 0.0;
                if (i0 >= X3F_TONE_LUT_SIZE) { a = b = c->lut[X3F_TONE_LUT_SIZE]; }
                else {
                    a = c->lut[i0];
                    b = c->lut[i0 + 1];
                }
                ymap = (a + (b - a) * f) / 65535.0;
                sc = (yl > 1e-9) ? ymap / yl : 0.0;
                for (i = 0; i < 3; i++) {
                    d[i] = lv[i] * sc;
                    if (d[i] > mx) mx = d[i];
                }
                if (mx > 1.0) {
                    /* 单通道要出界时朝该亮度去饱和（保持 Y 不变、只收色差），
                     * 平滑过渡，不能在 255 附近硬夹出色相边。 */
                    double s = (ymap < 1.0) ? (1.0 - ymap) / (mx - ymap) : 0.0;
                    if (s < 0.0) s = 0.0;
                    for (i = 0; i < 3; i++)
                        d[i] = ymap + (d[i] - ymap) * s;
                    for (i = 0; i < 3; i++) if (d[i] > 1.0) d[i] = 1.0;
                }
            } else if (sd1_chroma_on()) {
                /* SIGMA SD1：实验支路——逐通道施加（相机 classic 引擎的曲线
                 * 是逐通道 LUT；WB 修准后再验）。临时实验，勿直接提交。 */
                for (i = 0; i < 3; i++) {
                    double t = classic_tone_sd1_curve(lv[i]);
                    double v = t;
                    if (v > 1.0) v = 1.0;
                    d[i] = srgb_encode(v);
                }
            } else {
                /* 边缘限幅：掩膜>0 时对「未勾选差值」按 m 收回（1=全收，
                 * = 勾选态基准 LUT；0 = 不进分支，逐位现行为）。见
                 * oe_edge_clip_on() 注释。 */
                double em = 0.0;
                if (c->edge_mask != NULL)
                    em = c->edge_mask[(size_t)row * (size_t)c->w + x] / 255.0;
                for (i = 0; i < 3; i++) {
                    const uint16_t *tab =
                        (i == 0) ? c->lut : (i == 1) ? c->lutg : c->lutb;
                    /* 负值：sRGB 原色下超出 sRGB 三角形的那部分真实颜色
                     * （2026-11-04 全链路广色域）。
                     *
                     * 曲线表按 |v| 查、结果取负号 —— 即奇对称外推
                     * f(-x) = -f(x)，与 x3f_render.c 里 srgb_encode_s 的口径
                     * 一致。符号必须留着：它进转域矩阵后会变成目标域里的
                     * **正值**，那才是 sRGB 之外的颜色。夹 0 等于把这块内容
                     * 在门口删掉，"宽容器窄内容"就是这么来的。
                     *
                     * sRGB 出图保持旧行为（负值当 0 查表），零回归。 */
                    double v = lv[i];
                    int negv = 0;
                    if (v < 0.0 && c->out_space != X3F_OUT_SRGB) { v = -v; negv = 1; }
                    double idx = lin_to_index(v);
                    int i0 = (int)idx;
                    double f = idx - i0, a, b;
                    if (i0 >= X3F_TONE_LUT_SIZE) { a = b = tab[X3F_TONE_LUT_SIZE]; }
                    else {
                        a = tab[i0];
                        b = tab[i0 + 1];
                    }
                    d[i] = (a + (b - a) * f) / 65535.0;
                    if (negv) d[i] = -d[i];
                    if (em > 0.0) {
                        const uint16_t *etab =
                            (i == 0) ? c->elut : (i == 1) ? c->elutg : c->elutb;
                        double ea, eb;
                        if (i0 >= X3F_TONE_LUT_SIZE) { ea = eb = etab[X3F_TONE_LUT_SIZE]; }
                        else {
                            ea = etab[i0];
                            eb = etab[i0 + 1];
                        }
                        d[i] += ((ea + (eb - ea) * f) / 65535.0 - d[i]) * em;
                    }
                }
            }
            if (c->fillgain != NULL && fill_display_apply() && !fill_hue_preserve()) {
                for (i = 0; i < 3; i++) {
                    d[i] *= fg;
                    if (d[i] > 1.0) d[i] = 1.0;
                }
            }
        }
        /* 保色高光恢复：同一个 LUT 查在亮度上，再按比例缩放三通道。
         * 逐通道查表会让三通道压缩量不同，高光色相被拉开（绿/粉环）。 */
        if (hl_y_only() && c->hl_recover > 0.0) {
            double yd = 0.2126 * d[0] + 0.7152 * d[1] + 0.0722 * d[2];
            if (yd > 1e-6) {
                double idx = lin_to_index(yl), a, b, f, ymap;
                int i0 = (int)idx;
                f = idx - i0;
                if (i0 >= X3F_TONE_LUT_SIZE) { a = b = c->lut[X3F_TONE_LUT_SIZE]; }
                else {
                    a = c->lut[i0];
                    b = c->lut[i0 + 1];
                }
                ymap = (a + (b - a) * f) / 65535.0;
                f = ymap / yd;
                {
                    /* 逐通道截断会把"只有 R 到顶"的边缘压成粉/绿。
                     * X3F_HL_HUEKEEP=1 改成按最大通道整体缩回（色相不变）：
                     * 亮度略降，但不会在边界上拉出假的色相。 */
                    static int hk = -1;
                    double mx = 0.0;
                    if (hk < 0) {
                        const char *e = getenv("X3F_HL_HUEKEEP");
                        hk = (e != NULL && *e != '\0' && *e != '0') ? 1 : 0;
                    }
                    for (i = 0; i < 3; i++) {
                        d[i] *= f;
                        if (d[i] > mx) mx = d[i];
                    }
                    if (hk && mx > 1.0) {
                        double g = 1.0 / mx;
                        for (i = 0; i < 3; i++) d[i] *= g;
                    } else {
                        for (i = 0; i < 3; i++)
                            if (d[i] > 1.0) d[i] = 1.0;
                    }
                }
            }
        }
        /*---- 黑白（ B&W 页签）---------------------------------
         * 位置照：色彩混合在**色调曲线之后**（stage3 的 48RGB 域），
         * 见 x3f_render.h 的 mono 字段说明。*/
        if (c->mono != NULL) {
            double g = c->mono->w[0] * d[0] + c->mono->w[1] * d[1]
                     + c->mono->w[2] * d[2];
            d[0] = g * c->mono->tint[0];
            d[1] = g * c->mono->tint[1];
            d[2] = g * c->mono->tint[2];
        }

        if (c->use_sat && c->mono == NULL) {
            /*饱和度： Rec.601 饱和度矩阵（TonalAdjustments §4.4，
             * 完整：priProcessF20Stage4 构建的 M(u)，经
             * ProcessToOutput 的 FColorMatrix 一次应用）。
             *   u = clamp(Sat,-2,2)/2 + 1，Sat=-2 → u=0 纯灰度、+2 → u=2
             * M(u) 每行系数和恒为 1，代数上严格等价于「按 601 亮度拉开色差」：
             *   out_c = Y601 + (c − Y601)·u，Y601 = 0.299R + 0.587G + 0.114B
             * 旧实现用的是 Rec.709 系数（0.2126/0.7152/0.0722），与
             * 601 不一致 —— 高饱和红/蓝的色差量会差几个百分点，这里换成
             * 601。app 参数 = SPP 滑杆值 1:1（SPPWrite 十分位直写
             * 直读），调用方传进来的 c->sat = 1 + 滑杆/2，正好就是 u
             * （SPP 定义 u = Sat/2 + 1），再夹一次 [0,2] 保险。 */
            double u = c->sat;
            if (u < 0.0) u = 0.0;
            if (u > 2.0) u = 2.0;
            double y = 0.299 * d[0] + 0.587 * d[1] + 0.114 * d[2];
            for (i = 0; i < 3; i++) d[i] = y + (d[i] - y) * u;
        }

        if (c->hl_desat > 0.0) {
            double w;
            int sdq_max = 0;   /* 补收命中 → 目标取最亮通道（该是白） */
            if (c->clip) {
                /* RAW 过曝驱动：只有两个以上通道顶到传感器上限（色彩不可信）
                 * 才收色，跟显示亮度无关——所以压低曝光/做高光恢复把层次调
                 * 出来时，品红也不会回来；单个通道饱和的纯色物体不受影响。 */
                double cs = c->clip[row * (size_t)c->w + (size_t)x];
                /* 门槛可调（默认值见上面的宏）：DP3M2109 的镀铬边缘上，
                 * 极亮处常常**只有一个通道先顶**，另两个已经掉下来 ——
                 * 饱和分只有 1，原来几乎不收色，于是矩阵把它变成黄边
                 * （相机直出没有这条黄边）。放宽后单通道也收一部分。 */
                /* 2026-11-01 性能修复：这两行原来是**每像素** getenv + atof
                 * （1962 万像素 ≈ 4000 万次 environ 字符串扫描），CPU apply 段
                 * 从 0.3s 被拖到十几分钟。GPU apply 链不经过这里，所以一直
                 * 没暴露；一旦回退 CPU（显卡不可用 / X3F_GPU_APPLY=0）就表现
                 * 成「卡死」。env 在进程内不变，读一次缓存即可。 */
                static int cl_ini = 0;
                static double cl_lo = 0.0, cl_hi = 0.0;
                double lo, hi;
                if (!cl_ini) {
                    const char *e = getenv("X3F_HL_CLIP_LO");
                    cl_lo = (e != NULL) ? atof(e) : X3F_HL_DESAT_CLIP_LO;
                    e = getenv("X3F_HL_CLIP_HI");
                    cl_hi = (e != NULL) ? atof(e) : X3F_HL_DESAT_CLIP_HI;
                    cl_ini = 1;
                }
                lo = cl_lo;
                hi = cl_hi;
                w = sstep01(lo, hi, cs) * c->hl_desat;
            } else {
                /* 回退：没有 RAW 信息时，按显示值是否近白触发 */
                double m = d[0];
                if (d[1] < m) m = d[1];
                if (d[2] < m) m = d[2];
                w = sstep01(X3F_HL_DESAT_KNEE, 1.0, m) * c->hl_desat;
            }
            /* ---- SDQ 补收（显示域，不依赖 clip）----------------------
             * SDQ/SDQH 的 mode8 通路用的是显示域 knee（门槛 0.80，按最小的
             * 那个通道算），于是 (233,228,170) 这种「B 先掉下来」的淡黄永远
             * 够不到门槛 —— 拉到 1 也就停在原地（实测 App 里 4769 个近白
             * 偏黄像素一个没动）。这里补一条只看显示域的判据：
             *
             *   亮度够高（Y ≥ 0.62 起、0.86 全量）且**不是浓彩色**
             *   （色差 dmx-dmn < 0.40 起、0.55 以上完全排除）
             *
             * 为什么用「色差小」而不是「min 高」：要收的正是 B 掉下来的那种，
             * 它的 min 天生就低（min 判据永远追不上自己）。而夕阳 / 灯光 /
             * 水面橙色反光的色差都在 0.5 以上，正好被挡掉（实测真橙反光
             * (224,171,95) 色差 0.505、yy 0.694 → 权重 0.05，基本不动）。
             *
             * dig == (1,1,1) 的机型（DP-Q / Merrill）整块不进 ⇒ 零回归；
             * 滑杆为 0 时也不进 ⇒ 未开箱逐位不变。X3F_SDQ_NEUT=0 可关。 */
            if (g_sdq_hl > 0.0 && sdq_dig_active()) {
                /* 同上：原来是每像素 getenv（Quattro + SDQ 滑杆时全程命中）。 */
                static int sdn = -1;
                if (sdn < 0) {
                    const char *e = getenv("X3F_SDQ_NEUT");
                    sdn = (e == NULL || atoi(e) != 0) ? 1 : 0;
                }
                if (sdn) {
                    double yy = 0.2126 * d[0] + 0.7152 * d[1] + 0.0722 * d[2];
                    double dmn = d[0], dmx = d[0];
                    double wd;
                    int k;
                    for (k = 1; k < 3; k++) {
                        if (d[k] < dmn) dmn = d[k];
                        if (d[k] > dmx) dmx = d[k];
                    }
                    (void)dmn;
                    /* 形状与强度统一走 sdq_weight_at（s>1 会下移门槛，
                     * 见该函数的注释）。 */
                    {
                        double cs = c->clip ? (double)c->clip[row * (size_t)c->w + (size_t)x]
                                            : -1.0;
                        wd = sdq_weight_scene(yy, dmx - dmn, g_sdq_hl, cs);
                    }
                    /* 判 sdq_max 要用 wd 自己，不能用「wd 比 w 大」——真高光
                     * 那里 w 本来就是 1，wd 也是 1，比较为假 → 目标取不到
                     * 最亮通道，B 会停在 204 这种「半收」状态。 */
                    if (wd > 0.5) sdq_max = 1;
                    if (wd > w) w = wd;
                }
            }
            if (w > 0.001) {
                /* 拉到哪个值，取决于 RAW 怎么说。
                 *
                 * 三通道全饱和的高光（DP3M2110 那个镜面高光：饱和比
                 * R 1.04 / G 0.99 / B 0.99）经矩阵后可能出负值
                 * （R 5.38 / G −0.99 / B 0.22）。过 tone LUT 时负值按 0 处理，
                 * 于是显示域变成"R 白、G 黑"，加权亮度只剩 0.21 ——
                 * 往它拉就把一块本该纯白的高光拉成深灰，就是**黑斑**。
                 * 显示域已经分不清"全通道过曝"和"只有 R 亮"了，
                 * 所以这里用 RAW 的饱和分来判：**两个以上通道饱和 = 该是白的**，
                 * 目标取三通道最大值。单通道饱和的纯色物体（饱和分≈1）不受影响，
                 * 仍旧只做轻度的去饱和。 */
                double dr = d[0] > 0.0 ? d[0] : 0.0;
                double dg = d[1] > 0.0 ? d[1] : 0.0;
                double db = d[2] > 0.0 ? d[2] : 0.0;
                double y = 0.2126 * dr + 0.7152 * dg + 0.0722 * db;
                /* 关键：过渡必须是**平滑**的。
                 * 第一版写成 `if (cs >= 2.0) 目标 = 最大值` 这种硬阈值，
                 * 于是饱和分刚好在 2 上下的像素目标值突变，屏幕上就框出
                 * 一条生硬的黄边（窗沿那道）。改成按饱和分在 1.6~2.4 之间
                 * 渐入，形状上是"越接近全通道饱和，越往最亮通道靠"。 */
                if (sdq_max) {
                    /* 显示域补收命中（sdq_neut_w）：这类点「确定该是白」，
                     * 目标直接取最亮通道 —— 走加权亮度会停在 B 掉下来的那个
                     * 高度上，看着还是淡黄（实测 B 只到 204/255）。
                     * 不依赖 clip：mode8 后段那条路传进来的 clip 是空的。 */
                    double mx = d[0];
                    if (d[1] > mx) mx = d[1];
                    if (d[2] > mx) mx = d[2];
                    if (mx > y) y = mx;
                } else if (c->clip != NULL) {
                    double cs = c->clip[row * (size_t)c->w + (size_t)x];
                    double t = sstep01(1.6, 2.4, cs);
                    if (t > 0.0) {
                        double mx = d[0];
                        if (d[1] > mx) mx = d[1];
                        if (d[2] > mx) mx = d[2];
                        if (mx > y) y += (mx - y) * t;
                    }
                }
                for (i = 0; i < 3; i++) d[i] += (y - d[i]) * w;
            }
        }

        /* ---- 高光保色（2026-10-31，X3F_HL_CHROMA；默认 1.0 = 逐位 no-op）----
         * 症状（DP2Q2443 麻园站黄墙，用户："有点红"）：亮部黄被冲淡成粉。
         * 三方实测同一块墙（显示域均值）：
         *   我方 R213 G202 B157（R−B 56）| 机内 JPEG R228 G213 B147（80）
         *   | SPP R217 G189 B118（99）
         * 也就是说我们**不是红太多，是蓝压不下去**——逐通道 tone LUT 把
         * R/G/B 一起往 1.0 推，越接近白，色度掉得越快，于是饱和黄变成
         * 低饱和暖白 = 肉眼粉色。低亮度段反而比 SPP 更艳（sat 0.284 vs
         * 0.183），所以全局加饱和度（滑块）是错药：实测 sat=0.6 才把
         * R−B 从 56 抬到 73，仍不到 80，且暗部会更过。
         *
         * 这里改成**按亮度门控的色度乘子**：只在中高亮度带把 (d − Y) 放大
         * k 倍，Y 不变（不动明度），k = 1 + (hl_chroma − 1)·w，
         * w = smoothstep(0.78, 0.93, Y) —— 显示域 199..237 渐入，再往上
         * 到死白区自然回落（色度本来就该收）。中性像素 d≈Y，乘多少都不变。
         *
         * 默认 1.0 = 不启用（逐位等于加这段之前）；标定/实验走
         * X3F_HL_CHROMA=1.4 这类。 */
        /* ---- 显示域色度补偿：两段互相独立，按乘子复合 ------------------
         * 色度缩放不改变 Y（out = Y + (d−Y)·k 的 luma 仍是 Y），所以两段
         * 谁先谁后结果相同，可以合成一个 kk 一次写完。
         *
         * A. 中亮带通 hc_band_k（**2026-11-05 起默认 1.30**）：钟形窗 +
         *    黄/橙色相闸。修「屋檐下黄墙偏粉/偏红」——对机内 JPEG 实测该段
         *    Y200–230 缺色度、Y≥240 反而色度偏多，钟形窗正好只补中间。
         * B. 高光上升窗 hl_chroma（App「亮部保色」滑杆 / X3F_HL_CHROMA）：
         *    旧语义原样保留，只有显式 >1.0 才进。A 关掉时这一支的浮点
         *    表达式与加 A 之前**逐位相同**（kk 初值 1.0，1.0·x == x）。
         */
        if (c->hc_band_k > 1.0 || c->hl_chroma > 1.0) {
            double Y = 0.299 * d[0] + 0.587 * d[1] + 0.114 * d[2];
            double kk = 1.0;

            if (c->hc_band_k > 1.0) {
                double w = sstep01(c->hc_band_lo1, c->hc_band_lo2, Y) *
                           (1.0 - sstep01(c->hc_band_hi1, c->hc_band_hi2, Y));
                if (w > 0.0 && c->hc_band_hue_on)
                    w *= hue_gate_w(d, c->hc_band_hue_lo, c->hc_band_hue_hi);
                if (w > 0.0) kk *= 1.0 + (c->hc_band_k - 1.0) * w;
            }

            if (c->hl_chroma > 1.0) {
                double wc = sstep01(c->hc_win_lo, c->hc_win_hi, Y);
                if (wc > 0.0 && c->hc_hue_on)
                    wc *= hue_gate_w(d, c->hc_hue_lo, c->hc_hue_hi);
                if (wc > 0.0) kk *= 1.0 + (c->hl_chroma - 1.0) * wc;
            }

            if (kk != 1.0) {
                for (i = 0; i < 3; i++) d[i] = Y + (d[i] - Y) * kk;
            }
        }

        /* 高光暖色修正（试验，X3F_HL_WARM=1）：
         * R/G 双通道到顶后，B 还会跟着顶层继续往白走，火焰、白炽灯这类
         * 本该偏暖的高光会变成冷白。按饱和分把 B 压到 R 的一个比例以内。
         * DP2Q2501 实测：我们近白 R246 G250 B253（偏冷），
         * SPP 同区域 R254 G239 B194（暖），k=0.80 正好落在 197 附近。 */
        if (c->clip != NULL) {
            static int warm = -1;
            static double warm_k = 0.80;
            if (warm < 0) {
                const char *e = getenv("X3F_HL_WARM");
                const char *k = getenv("X3F_HL_WARM_K");
                warm = (e != NULL && *e != '\0' && *e != '0') ? 1 : 0;
                if (k != NULL && *k != '\0') warm_k = atof(k);
            }
            if (warm) {
                double cs = c->clip[row * (size_t)c->w + (size_t)x];
                double t = sstep01(1.2, 1.8, cs);
                if (t > 0.0) {
                    double cap = warm_k * d[0];
                    if (d[2] > cap) {
                        /* 不能直接夹到 cap：那样超出部分的起伏全被抹平，
                         * 火焰核会变成一块死奶油色。只压掉 (1-keep) 的超出量，
                         * 留下 keep 的结构。 */
                        static double keep = -1.0;
                        double target;
                        if (keep < 0.0) {
                            const char *e = getenv("X3F_HL_WARM_KEEP");
                            keep = (e != NULL && *e != '\0') ? atof(e) : 0.35;
                        }
                        target = cap + (d[2] - cap) * keep;
                        d[2] += (target - d[2]) * t;
                    }
                }
            }
        }

        /* 相机色彩匹配是**色度**上的 2x2 变换：灰度图的 cb/cr 恒为 0，
         * 走这里反而会把查表出来的非零值当成颜色加回去（变成有色灰），
         * 所以黑白下直接跳过。 */
        if (g_color_match && c->mono == NULL) {
            /* 相机色彩匹配：按色相把色度做小 2x2 变换，亮度不动（见 kColorMatch） */
            double yy = 0.299 * d[0] + 0.587 * d[1] + 0.114 * d[2];
            double cb = d[2] - yy, cr = d[0] - yy;
            double fx = (cb + X3F_CM_R) * (X3F_CM_N - 1) / (2.0 * X3F_CM_R);
            double fy = (cr + X3F_CM_R) * (X3F_CM_N - 1) / (2.0 * X3F_CM_R);
            if (fx >= 0.0 && fy >= 0.0 && fx <= X3F_CM_N - 1 && fy <= X3F_CM_N - 1) {
                int ix = (int)fx, iy = (int)fy;
                int jx = ix + 1 < X3F_CM_N ? ix + 1 : ix;
                int jy = iy + 1 < X3F_CM_N ? iy + 1 : iy;
                double tx = fx - ix, ty = fy - iy, ncb, ncr, rr, bb, gg;
                ncb = (g_cm_cb[iy*X3F_CM_N+ix] + (g_cm_cb[iy*X3F_CM_N+jx] - g_cm_cb[iy*X3F_CM_N+ix]) * tx) * (1 - ty)
                    + (g_cm_cb[jy*X3F_CM_N+ix] + (g_cm_cb[jy*X3F_CM_N+jx] - g_cm_cb[jy*X3F_CM_N+ix]) * tx) * ty;
                ncr = (g_cm_cr[iy*X3F_CM_N+ix] + (g_cm_cr[iy*X3F_CM_N+jx] - g_cm_cr[iy*X3F_CM_N+ix]) * tx) * (1 - ty)
                    + (g_cm_cr[jy*X3F_CM_N+ix] + (g_cm_cr[jy*X3F_CM_N+jx] - g_cm_cr[jy*X3F_CM_N+ix]) * tx) * ty;
                rr = yy + ncr;
                bb = yy + ncb;
                gg = (yy - 0.299 * rr - 0.114 * bb) / 0.587;
                /* CM 高光门控（试验，X3F_CM_HL_GATE=1）：CM **施加**按亮度
                 * 坡道混合——hlo 以下全量 CM，hlo→hhi 平滑过渡到完全保留
                 * 原像素（等价 CM 跳过，而不是把修正量乘 0：那仍会重锚定
                 * 到等亮度 yy，破坏原色度）。实测（DP2Q1224 -1.78EV）：
                 * CM 在蓝白云过渡带把 R/B 相对 G 双抬（紫粉镶边 -8/+29，
                 * 跳过 CM 是 -13/+17 ≈ SPP -11/+16），中间调（绿树）CM
                 * 有益须保留。默认关 = 行为逐位不变。
                 * 阈值可用 X3F_CM_HL_LO / X3F_CM_HL_HI（0-255 显示域）覆盖。 */
                {
                    static int hlg = -1;
                    static double hlo = 170.0, hhi = 235.0;
                    if (hlg < 0) {
                        const char *e = getenv("X3F_CM_HL_GATE");
                        const char *lo = getenv("X3F_CM_HL_LO");
                        const char *hi = getenv("X3F_CM_HL_HI");
                        hlg = (e != NULL && *e != '\0' && *e != '0') ? 1 : 0;
                        if (lo != NULL && *lo != '\0') hlo = atof(lo);
                        if (hi != NULL && *hi != '\0') hhi = atof(hi);
                    }
                    if (hlg && hhi > hlo) {
                        double y255 = yy * 255.0;   /* 本段 d[] 为 0-1 域 */
                        double wcm = 1.0 - sstep01(hlo, hhi, y255);
                        if (wcm < 1.0) {
                            rr = d[0] + (rr - d[0]) * wcm;
                            gg = d[1] + (gg - d[1]) * wcm;
                            bb = d[2] + (bb - d[2]) * wcm;
                        }
                    }
                }
                d[0] = rr; d[1] = gg; d[2] = bb;
            }
        }

        /* 显示域 3×4 后校正（实验，X3F_COLOR_FIT=1）。
         * 矩阵由 base 渲染与 SPP TIFF 在同一裁切上拟合；
         * X3F_COLOR_FIT_GAMMA 可选，默认 1.0。 */
        if (color_fit_enabled() && c->mono == NULL) {
            static const double cf[4][3] = {
                { 0.7933, -0.0885, -0.2216 },
                { 0.3066,  1.0902,  0.0397 },
                {-0.0918, -0.0485,  0.9260 },
                { 2.2551 / 255.0, -0.0354 / 255.0, 9.1029 / 255.0 }
            };
            static double cfg = -1.0;
            double r = d[0], g = d[1], b = d[2];
            if (cfg < 0.0) {
                const char *e = getenv("X3F_COLOR_FIT_GAMMA");
                cfg = (e != NULL && *e != '\0') ? atof(e) : 1.0;
                if (cfg <= 0.0) cfg = 1.0;
            }
            for (i = 0; i < 3; i++) {
                /* cf 是按输入通道存的（4×3）：out_c = Σ_in cf[in][c]。 */
                double v = r * cf[0][i] + g * cf[1][i] + b * cf[2][i] + cf[3][i];
                if (v < 0.0) v = 0.0;
                if (v > 1.0) v = 1.0;
                if (cfg != 1.0) v = pow(v, cfg);
                d[i] = v;
            }
        }

        /* ---- 色彩分离（参照 Lightroom 的 Split Toning / Color Grading）----
         * 显示域链的最后一步：拿 601 亮度当分区权重（smoothstep，
         * 中心/半宽来自 balance/blending），往暗部/亮部注入「保亮度色差」：
         *   out_c = d_c + (T_c − T_601)·w
         * 色差向量的 601 加权和恒为 0，理论上这一步严格不动亮度。
         * 分量要越界时不能硬夹 —— 硬夹会破坏亮度守恒。改成把色差向量
         * 整体乘 k 收缩，k = 全部通道越界约束的最小值（见下），亮度严格
         * 不动、色相不变，只是注入量在色域边界上优雅衰减。
         * 两路强度都为 0 时整块不进，零滑杆逐位不变。 */
        if (c->use_split) {
            double y = 0.299 * d[0] + 0.587 * d[1] + 0.114 * d[2];
            double t = sstep01(c->split_center - c->split_half,
                               c->split_center + c->split_half, y);
            double ws = 1.0 - t, wh = t;
            double C[3], k = 1.0;
            int need = 0;
            for (i = 0; i < 3; i++) {
                C[i] = c->split_cs[i] * ws + c->split_ch[i] * wh;
                if (C[i] != 0.0) need = 1;
            }
            if (need) {
                /* k 取所有通道越界约束的最小值：
                 *   C_c > 0 → 上界 (1−d_c)/C_c；C_c < 0 → 上界 d_c/(−C_c)。
                 * 只看正分量不够 —— 暗部注入蓝色差时 R/G 是**负**分量，
                 * 硬夹 0 同样破坏亮度守恒（实测整图 ΔY 均值 +4.5 灰阶）。
                 * k=0（该像素纯黑/纯白，色差注不进）= 数学上的正确答案。 */
                for (i = 0; i < 3; i++) {
                    double lim;
                    if (C[i] > 0.0) lim = (1.0 - d[i]) / C[i];
                    else if (C[i] < 0.0) lim = d[i] / (-C[i]);
                    else continue;
                    if (lim < k) k = lim;
                }
                if (k < 0.0) k = 0.0;
                for (i = 0; i < 3; i++) d[i] += C[i] * k;
            }
        }

        /* ---- 全链路广色域：整幅换域（2026-11-04）-----------------------
         *
         * 到这里 d[] 是**完整的 sRGB 显示值** —— 相机观感曲线、高光恢复、
         * 对比度、oe_tone、色彩模式小 S、色调曲线、黑白、饱和度、高光保色、
         * 分离色调全部在 sRGB 域做完了（它们也只在这个域有定义）。
         *
         * 现在才转域，一步到位：
         *     sRGB 显示值 → sRGB 线性（可负） → 目标域线性 → 目标域编码
         *
         * 全程 double，负值按奇对称外推，所以 sRGB 之外的线性内容（相机矩阵
         * 出来的负值）会活着抵达目标域，成为那边**真实的正值**；线性高光
         * 余量同理。之后 px_put 才夹 [0,1]——那是 8/16bit 存储的物理上限，
         * 与色域无关。
         *
         * sRGB 输出整段跳过（out_space == 0），逐位零回归。
         */
        if (c->out_space != X3F_OUT_SRGB) {
            double tv[3];
            for (i = 0; i < 3; i++) tv[i] = srgb_decode_s(d[i]);
            out_lin_convert(c->out_space, tv);
            for (i = 0; i < 3; i++) d[i] = out_encode_s(c->out_space, tv[i]);
        }

        for (i = 0; i < 3; i++)
            px_put(dst, c->bits, (size_t)x * 3 + (size_t)i, d[i]);
        src += 3;
    }
}

/* ==================================================================
 * 黑白「胶片颗粒」（SPP FilmGrain，FilmGrainFactory_F23 编译单元）
 *
 * 2026-10-31 把 §5.1 卡住的最后一块补上了：四张表**就在 lib95 的
 * __TEXT,__const 里**（当mapped地址 0x1a4890/0x1a48b8/0x1a5628/0x1a5ce0，
 * 各表首尾相接：counts[10] → cdf double[10][43] → amp int32[10][43]
 * → T10 u32[10]）。提取与推导见工作区 grain_re/ 与
 * 研究笔记的补记。
 *
 * 逐像素模板（buildGrain / buildGrain_s，逐字复刻）：
 *   seed1 = col + 123456789 + T10[col % 10] / (row + 1)
 *   seed2 = col·row + 362437093 + T10[row % 10] / (col + 1)
 *   r     = XorShiftFixed(seed1, seed2)
 *           （e = a^(a<<11); r = ((b>>19)^(e>>8)) ^ (b^e)）
 *   idx   = 对 cdf[k][] 二分（SPP 的循环语义：相等即返回，
 *           否则取最后一个 cdf < r%10000 的桶；n≤3 直接桶 0）
 *   模板值 = (s16)((u16)amp[k][idx] · 1000)   —— u16 回绕语义照抄
 *
 * 外圈整形（SPP 的装配顺序读到了调用网，逐值接线待与 SPP 导出
 * 对质定标，先用最自洽的一套，全部留了 env 旁路）：
 *   档位  = clamp(floor(v·10 − 1), 0, 9)（fcvtzs 截断，v∈[0,1]）
 *   k     = 尺寸档     → 选 cdf/amp 行（43→5 桶，越粗越少）
 *   σ     = ga[粗糙档]（0.65→0.2，越糙越细）—— 可 X3F_GRAIN_SIGMA 覆盖
 *   幅度  = gb[k]（0.25→0.55）—— 可 X3F_GRAIN_AMP 覆盖
 *   总增益默认 0.1（X3F_GRAIN_GAIN 可调），位置确定（t38/t40 = 0）。
 * ================================================================== */

static uint32_t grain_xorshift(uint32_t a, uint32_t b)
{
    uint32_t e = a ^ (a << 11);
    return ((b >> 19) ^ (e >> 8)) ^ (b ^ e);
}

/* SPP 的二分：cdf[mid]==v 命中即返回 mid；否则收敛到
 * 「最后一个 cdf[i] < v」的 i。逐字照 0xb0408..0xb0474 的循环。 */
static int grain_pick(const unsigned int *cdf, int n, unsigned v)
{
    int hi = n - 1, lo = 0, mid;
    if (n - 1 < 2) return 0;
    for (;;) {
        unsigned c;
        mid = (hi + lo) >> 1;
        c = cdf[mid];
        if (c > v) {
            if (lo + 1 >= hi) return lo;
            hi = mid;
        } else if (c < v) {
            if (mid + 1 >= hi) return mid;
            lo = mid;
        } else {
            return mid;
        }
    }
}

/* 档位：fcvtzs(v·10 − 1)（向零截断）后夹 [0,9]（bic asr#31 的负数归 0） */
static int grain_level(double v)
{
    int l;
    if (!(v > 0.0)) return 0;
    if (v > 1.0) v = 1.0;
    l = (int)(v * 10.0 - 1.0);
    if (l < 0) l = 0;
    if (l > 9) l = 9;
    return l;
}

static double grain_env(const char *name, double dflt)
{
    const char *e = getenv(name);
    if (e != NULL && *e != '\0') {
        double v = atof(e);
        if (v > 0.0) return v;
    }
    return dflt;
}

/* 可分离高斯（σ = sigma；模板是 s16 的 delta 场，边界 clamp）。
 * 半径不足 1px 时近似恒等（SPP 小 σ 档也接近恒等）。 */
static void grain_gauss_blur(int16_t *img, int w, int h, double sigma)
{
    int r, i, x, y, n;
    double *kern;
    double sum;
    int16_t *tmp;

    if (sigma < 0.35) return;
    r = (int)ceil(sigma * 2.5);
    if (r < 1) r = 1;
    if (r > 32) r = 32;
    kern = (double *)malloc(sizeof(double) * (size_t)(2 * r + 1));
    tmp = (int16_t *)malloc(sizeof(int16_t) * (size_t)w * (size_t)h);
    if (kern == NULL || tmp == NULL) { free(kern); free(tmp); return; }
    sum = 0.0;
    for (i = -r; i <= r; i++) {
        kern[i + r] = exp(-0.5 * (double)i * (double)i / (sigma * sigma));
        sum += kern[i + r];
    }
    for (i = 0; i <= 2 * r; i++) kern[i] /= sum;

    n = 2 * r + 1;
    /* 横向 */
    for (y = 0; y < h; y++) {
        const int16_t *row = img + (size_t)y * w;
        int16_t *dst = tmp + (size_t)y * w;
        for (x = 0; x < w; x++) {
            double acc = 0.0;
            for (i = 0; i < n; i++) {
                int xx = x + i - r;
                if (xx < 0) xx = 0;
                if (xx >= w) xx = w - 1;
                acc += (double)row[xx] * kern[i];
            }
            dst[x] = (int16_t)lrint(acc);
        }
    }
    /* 纵向 */
    for (y = 0; y < h; y++) {
        int16_t *row = img + (size_t)y * w;
        for (x = 0; x < w; x++) {
            double acc = 0.0;
            for (i = 0; i < n; i++) {
                int yy = y + i - r;
                if (yy < 0) yy = 0;
                if (yy >= h) yy = h - 1;
                acc += (double)tmp[(size_t)yy * w + x] * kern[i];
            }
            row[x] = (int16_t)lrint(acc);
        }
    }
    free(kern);
    free(tmp);
}

/* 叠到已渲染的显示缓冲上（apply_common 的 out/stride/bits 布局）。
 * 黑白下三通道同值，同一个 delta 加到 R/G/B。关（mono_grain=0）不进。 */
static void mono_grain_apply(uint8_t *out, int w, int h, int stride, int bits,
                             double size, double rough)
{
    int k = grain_level(size);
    int lr = grain_level(rough);
    int n = mono_grain_counts[k];
    double sigma = grain_env("X3F_GRAIN_SIGMA", (double)mono_grain_ga[lr]);
    double amp   = grain_env("X3F_GRAIN_AMP",   (double)mono_grain_gb[k]);
    double gain  = grain_env("X3F_GRAIN_GAIN",  0.1) * amp;
    const unsigned int *cdf = mono_grain_cdf[k];
    const int *amptab = mono_grain_amp[k];
    int bytes = bits / 8;
    int16_t *tmpl;
    int x, y, ch;

    if (w <= 0 || h <= 0 || n <= 0) return;
    tmpl = (int16_t *)malloc(sizeof(int16_t) * (size_t)w * (size_t)h);
    if (tmpl == NULL) return;

    for (y = 0; y < h; y++) {
        uint32_t t10row = mono_grain_t10[(unsigned)y % 10u];
        uint32_t ry = (uint32_t)y;
        for (x = 0; x < w; x++) {
            uint32_t rx = (uint32_t)x;
            uint32_t seed1 = rx + 123456789u
                           + mono_grain_t10[rx % 10u] / (ry + 1u);
            uint32_t seed2 = rx * ry + 362437093u
                           + t10row / (rx + 1u);
            uint32_t r = grain_xorshift(seed1, seed2);
            int idx = grain_pick(cdf, n, r % 10000u);
            tmpl[(size_t)y * w + x] =
                (int16_t)(uint16_t)((uint16_t)amptab[idx] * 1000u);
        }
    }

    grain_gauss_blur(tmpl, w, h, sigma);

    for (y = 0; y < h; y++) {
        uint8_t *row = out + (size_t)y * (size_t)stride;
        const int16_t *trow = tmpl + (size_t)y * w;
        for (x = 0; x < w; x++) {
            int delta = (int)lrint((double)trow[x] * gain);
            if (delta == 0) continue;
            for (ch = 0; ch < 3; ch++) {
                if (bits == 16) {
                    uint16_t *p = (uint16_t *)(row + (size_t)x * 6 + (size_t)ch * 2);
                    int v = (int)*p + delta;
                    if (v < 0) v = 0;
                    if (v > 65535) v = 65535;
                    *p = (uint16_t)v;
                } else {
                    uint8_t *p = row + (size_t)x * 3 + (size_t)ch;
                    int v = (int)*p + (delta >> 8);
                    if (v < 0) v = 0;
                    if (v > 255) v = 255;
                    *p = (uint8_t)v;
                }
            }
        }
    }
    free(tmpl);
    (void)bytes;
}

/* ------------------------------------------------------------------
 * 明度层 finish 的胶片颗粒（2026-10-31，供 App 明度层 finish 调用）。
 * 与黑白 mono_grain_apply **同一套** SPP FilmGrain 模板/表/高斯/幅度，
 * 差别只在落点：黑白直接加在 8/16bit RGB 显示缓冲（8bit 走 delta>>8），
 * 这里加在 [0,255] 的 float Y 平面上。为与黑白 8bit 输出可比，
 * Y 增量 = delta/257（16bit→8bit 的线性比例，等效 delta>>8 的期望值）。
 *
 * 滑杆语义与黑白面板对齐（映射在 Swift 侧做）：
 *   size  0..1 → 档位 k（选 cdf/amp 行 + 幅度 gb[k]）
 *   rough 0..1 → 高斯 σ = ga[lr]
 * originX/originY：瓦片全局坐标偏移——种子吃绝对 (col,row)，
 * 保证 1:1 区域 finish 与整幅导出是同一张颗粒网。
 * ------------------------------------------------------------------ */
void x3f_film_grain_yplane(float *y, int w, int h,
                           int originX, int originY,
                           double size, double rough)
{
    int k = grain_level(size);
    int lr = grain_level(rough);
    int n = mono_grain_counts[k];
    double sigma = grain_env("X3F_GRAIN_SIGMA", (double)mono_grain_ga[lr]);
    double amp   = grain_env("X3F_GRAIN_AMP",   (double)mono_grain_gb[k]);
    double gain  = grain_env("X3F_GRAIN_GAIN",  0.1) * amp / 257.0;
    const unsigned int *cdf = mono_grain_cdf[k];
    const int *amptab = mono_grain_amp[k];
    int16_t *tmpl;
    int x, yy;

    if (y == NULL || w <= 0 || h <= 0 || n <= 0) return;
    tmpl = (int16_t *)malloc(sizeof(int16_t) * (size_t)w * (size_t)h);
    if (tmpl == NULL) return;

    for (yy = 0; yy < h; yy++) {
        uint32_t gy = (uint32_t)(yy + originY);
        uint32_t t10row = mono_grain_t10[gy % 10u];
        for (x = 0; x < w; x++) {
            uint32_t gx = (uint32_t)(x + originX);
            uint32_t seed1 = gx + 123456789u
                           + mono_grain_t10[gx % 10u] / (gy + 1u);
            uint32_t seed2 = gx * gy + 362437093u
                           + t10row / (gx + 1u);
            uint32_t r = grain_xorshift(seed1, seed2);
            int idx = grain_pick(cdf, n, r % 10000u);
            tmpl[(size_t)yy * w + x] =
                (int16_t)(uint16_t)((uint16_t)amptab[idx] * 1000u);
        }
    }

    grain_gauss_blur(tmpl, w, h, sigma);

    for (yy = 0; yy < h; yy++) {
        float *row = y + (size_t)yy * w;
        const int16_t *trow = tmpl + (size_t)yy * w;
        for (x = 0; x < w; x++) {
            float v = row[x] + (float)((double)trow[x] * gain);
            if (v < 0.0f) v = 0.0f;
            if (v > 255.0f) v = 255.0f;
            row[x] = v;
        }
    }
    free(tmpl);
}

static int apply_common(const uint16_t *lut, double sat, int use_sat,
                        double hl_desat, const float *clip, const float *fillgain,
                        const float *gain,
                        const mono_ctx_t *mono,
                        const x3f_color_mode_t *cm,
                        const float *lin, int w, int h,
                        void *out, int stride, int bits,
                        const uint16_t *lutg, const uint16_t *lutb,
                        const x3f_edit_params_t *ep)
{
    if (apply_t_on())
        fprintf(stderr, "[apply] %-10s          footprint=%.0f/%.0fMB\n", "ac.in",
                apply_t_footprint_mb(), apply_t_peak_mb());
    apply_ctx_t ctx;
    const int bytes = bits / 8;

    if (lut == NULL || lin == NULL || out == NULL) return X3FRENDER_ERR_PARAM;
    if (w <= 0 || h <= 0 || (bits != 8 && bits != 16 && bits != 32)) return X3FRENDER_ERR_PARAM;
    if (stride < w * 3 * bytes) return X3FRENDER_ERR_PARAM;

    /* 色彩模式的 LUT（色相偏移/饱和度那一路）只在模式或强度变了时重建：
     * 拖强度滑块时每帧都会走到这儿，而 (Cb,Cr) 表是 65×65 = 4225 格。 */
    if (g_color_match &&
        (!g_cm_built || g_cm_q_lut != g_cm_quattro ||
         !cm_mode_same(cm, g_cm_mode_valid ? &g_cm_mode : NULL))) {
        build_color_match_lut(cm);
    }

    /* 色彩分离的预换算（两路强度都为 0 → use_split=0，逐位 no-op） */
    split_setup(ep, &ctx);

    ctx.lin = lin;
    ctx.out = out;
    ctx.w = w;
    ctx.h = h;
    ctx.stride = stride;
    ctx.bits = bits;
    ctx.lut = lut;
    ctx.lutg = (lutg != NULL) ? lutg : lut;
    ctx.lutb = (lutb != NULL) ? lutb : lut;
    ctx.sat = sat;
    ctx.use_sat = use_sat;
    ctx.hl_desat = hl_desat;
    /* 高光保色乘子：参数为主（App 滑杆 / sidecar），env 覆盖（命令行 A/B）。
     * 每次 apply 读一次，不逐像素读。默认 k=1.0 = 整块不进（逐位不变）。 */
    {
        double k   = ep->hl_chroma;
        double wlo = ep->hl_chroma_win_lo, whi = ep->hl_chroma_win_hi;
        double hlo = ep->hl_chroma_hue_lo, hhi = ep->hl_chroma_hue_hi;
        /* 兜底：零初始化（老调用方 / 旧参数 dump）与 NaN 一律落回"关"。
         * 窗非法（含 0,0）→ 用默认窗；色相非法 → 不限色相。 */
        if (!(k > 1.0)) k = 1.0;
        if (k > 4.0) k = 4.0;
        if (!(whi > wlo) || wlo < 0.0 || whi > 1.0) { wlo = 0.78; whi = 0.93; }
        if (!(hhi > hlo)) { hlo = 0.0; hhi = 360.0; }
        ctx.hl_chroma = k;
        ctx.hc_win_lo = wlo; ctx.hc_win_hi = whi;
        ctx.hc_hue_on = (hhi - hlo < 359.5) ? 1 : 0;
        ctx.hc_hue_lo = hlo; ctx.hc_hue_hi = hhi;
        {
            const char *ehc = getenv("X3F_HL_CHROMA");
            if (ehc != NULL && *ehc != '\0') {
                ctx.hl_chroma = atof(ehc);
                if (ctx.hl_chroma < 1.0) ctx.hl_chroma = 1.0;
            }
        }
        {
            const char *ew = getenv("X3F_HL_CHROMA_WIN");
            if (ew != NULL && *ew != '\0') {
                double lo = 0.78, hi = 0.93;
                if (sscanf(ew, "%lf,%lf", &lo, &hi) == 2 && hi > lo) {
                    ctx.hc_win_lo = lo; ctx.hc_win_hi = hi;
                }
            }
        }
        {
            const char *eh = getenv("X3F_HL_CHROMA_HUE");
            if (eh != NULL && *eh != '\0') {
                double lo = 0.0, hi = 360.0;
                if (sscanf(eh, "%lf,%lf", &lo, &hi) == 2) {
                    ctx.hc_hue_on = (hi - lo < 359.5) ? 1 : 0;
                    ctx.hc_hue_lo = lo; ctx.hc_hue_hi = hi;
                }
            }
        }
        /* ---- 中亮带通保色（2026-11-05，默认开 K=1.30；X3F_MID_CHROMA 可覆盖/关闭）
         *
         * 症状：DP2Q2443 麻园站黄墙「屋檐下面有一些偏红」。对机内 JPEG 逐
         * 亮度带解出「要把 (d−Y) 放大多少倍才对得上」（k 的最小二乘解）：
         *
         *   相机 Y  140-160 160-180 180-200 200-210 210-220 220-230 230-240 240+
         *   dB       −0.6    +3.7   +12.6   +18.3   +21.8   +19.2   +4.3  −22
         *   需要 k    1.01    1.10    1.19    1.26    1.30    1.26    1.07  0.5↓
         *
         * 也就是说 **中亮段 Y200–230 缺色度（蓝多 20 级 → 暖白/粉 = 用户说的
         * 偏红），极亮段 Y≥240 反而色度偏多**。原来的「亮部保色」是单调
         * 上升窗 smoothstep(0.78,0.93,Y)，只作用在极亮段，方向正好相反：
         * 实测 hlChroma 1.5/1.8/2.2 让 yellow 区 B 比机内 JPEG 再低
         * 37.7/47.7/61.1 级。所以这里另开一条钟形中亮窗，不动旧滑杆语义：
         *
         *   w = smoothstep(0.62,0.85,Y) · (1 − smoothstep(0.88,0.95,Y))
         *   k = 1 + (K−1) · w · 色相闸(40°–70°，软边 10°)
         *   out = Y + (d − Y)·k          （Y 不变，中性像素天然不动）
         *
         * K=1.30 的标定（对机内 JPEG，相机黄像素 Y170–238）：
         *   MAE(dB)  15.42 → 4.86，MAE(dR−B) 16.11 → 4.13；
         *   Y≥240 段 dB −21.78 → −21.81（几乎不动）；
         *   非黄像素平均改动 0.010 码；
         *   固定 wall 区 B 153.1 → 143.8（机内 147.4），R−B 74.8 → 86.1（机内 80.4）。
         *
         * **默认开**（2026-11-05 起）：K=1.30、钟形窗 0.62/0.85/0.88/0.95、
         * 色相闸 40°–70°。三条锚点对各自机内 JPEG 全部净改善：
         *   2443 麻园 MAE(dB) 15.42→4.93；1224 寺庙改动像素 45.2% 更近 /
         *   19.1% 更远、全图 MAE 5.772→5.727；2501 火核 62.5% 更近 /
         *   10.6% 更远、全图 3.265→3.247。
         * 关闭：X3F_MID_CHROMA=0（或 =1.0）；改强度：X3F_MID_CHROMA=1.15；
         * 标定/实验可覆盖 X3F_MID_CHROMA_WIN="lo1,lo2,hi1,hi2" 与
         * X3F_MID_CHROMA_HUE="lo,hi"。env 只在 apply 时读一次，不逐像素读。
         * 不新增 x3f_edit_params_t 字段：renderlab 按字节前缀覆盖 P，结构体
         * 一律只许在末尾追加，这里连末尾都不动，旧 dump 完全兼容。 */
        ctx.hc_band_k = 1.30;
        ctx.hc_band_lo1 = 0.62; ctx.hc_band_lo2 = 0.85;
        ctx.hc_band_hi1 = 0.88; ctx.hc_band_hi2 = 0.95;
        ctx.hc_band_hue_on = 1;
        ctx.hc_band_hue_lo = 40.0; ctx.hc_band_hue_hi = 70.0;
        {
            const char *em = getenv("X3F_MID_CHROMA");
            if (em != NULL && *em != '\0') {
                double k = atof(em);
                if (!(k > 1.0)) {
                    ctx.hc_band_k = 1.0;      /* =0 / =1.0 / 非法 → 关 */
                } else {
                    double lo1 = 0.62, lo2 = 0.85, hi1 = 0.88, hi2 = 0.95;
                    double hlo = 40.0, hhi = 70.0;
                    const char *ew = getenv("X3F_MID_CHROMA_WIN");
                    const char *eh2 = getenv("X3F_MID_CHROMA_HUE");
                    if (k > 4.0) k = 4.0;
                    if (ew != NULL && *ew != '\0') {
                        double a, b, c2, d2;
                        if (sscanf(ew, "%lf,%lf,%lf,%lf", &a, &b, &c2, &d2) == 4 &&
                            a >= 0.0 && a < b && b <= c2 && c2 < d2 && d2 <= 1.0) {
                            lo1 = a; lo2 = b; hi1 = c2; hi2 = d2;
                        }
                    }
                    if (eh2 != NULL && *eh2 != '\0') {
                        double a, b;
                        if (sscanf(eh2, "%lf,%lf", &a, &b) == 2 && b > a) {
                            hlo = a; hhi = b;
                        }
                    }
                    ctx.hc_band_k = k;
                    ctx.hc_band_lo1 = lo1; ctx.hc_band_lo2 = lo2;
                    ctx.hc_band_hi1 = hi1; ctx.hc_band_hi2 = hi2;
                    ctx.hc_band_hue_on = (hhi - hlo < 359.5) ? 1 : 0;
                    ctx.hc_band_hue_lo = hlo; ctx.hc_band_hue_hi = hhi;
                }
            }
        }
    }
    ctx.hl_recover = g_hl_recover_cur;
    ctx.clip = clip;
    ctx.fillgain = fillgain;
    ctx.mono = mono;
    /* 输出色彩空间（2026-11-04）：只有显式指定才切，其余（含旧参数 dump 里
     * 没有这个字段的 0）一律 sRGB —— 默认路径逐位不变。 */
    ctx.out_space = (ep != NULL) ? ep->out_space : X3F_OUT_SRGB;
    if (ctx.out_space < X3F_OUT_SRGB || ctx.out_space > X3F_OUT_DCI_P3)
        ctx.out_space = X3F_OUT_SRGB;   /* 未知值一律回落 sRGB */
    /* 观感延后（最小验证，见 x3f_render.h 的 look_defer 段）：ep 为空
     * （x3f_render_apply_lut 那条对照入口）或旧参数 dump 里没有这个字段
     * 时都是 0 = 现行为。 */
    ctx.look_defer = (ep != NULL && ep->look_defer) ? 1 : 0;
    ctx.exp_gain = (float)pow(2.0, (ep != NULL) ? ep->exposure : 0.0);
    /*黑白下不参与（黑白页签里也没有色彩模式这一栏）。*/
    ctx.cm = (cm != NULL && !cm->unit && mono == NULL) ? cm->m : NULL;
    /* 观感延后时色彩模式也不在这里乘：它属于「相机风格 / 调色」，应该在
     * 引擎之后与曲线、对比度、饱和度一起施加（否则第二段再乘一次 = 双重）。
     * 见 x3f_render.h 的 look_defer 段。 */
    if (ctx.look_defer) ctx.cm = NULL;
    if (gain) {
        ctx.gain[0] = gain[0]; ctx.gain[1] = gain[1]; ctx.gain[2] = gain[2];
        ctx.use_gain = 1;
    } else {
        ctx.gain[0] = ctx.gain[1] = ctx.gain[2] = 1.0f;
        ctx.use_gain = 0;
    }
    /* 边缘限幅（X3F_OE_EDGE_CLIP / x3f_render_set_oe_edge_clip，opt-in）：
     * 只在「未勾选差值曲线」实际生效时才有意义。默认全 NULL = 逐位现行为。
     * 缓冲区在本函数内 malloc / free（ctx 只被下面的 parallel_for 使用）。 */
    {
        uint16_t *elut_buf = NULL;
        uint8_t *emask = NULL;
        ctx.elut = ctx.elutg = ctx.elutb = NULL;
        ctx.edge_mask = NULL;
        if (oe_edge_clip_on() && x3f_render_oe_tone_off()) {
            double t0 = 0.35, t1 = 0.70;
            const char *te = getenv("X3F_OE_EDGE_THRESH");
            if (te != NULL && *te != '\0') {
                double a = 0.0, b = 0.0;
                if (sscanf(te, "%lf,%lf", &a, &b) == 2 && a > 0.0 && b > a) {
                    t0 = a; t1 = b;
                }
            }
            elut_buf = malloc(sizeof(uint16_t) * (size_t)(X3F_TONE_LUT_SIZE + 1) * 4);
            emask = oe_edge_mask_build(lin, w, h, t0, t1);
            if (elut_buf != NULL && emask != NULL) {
                uint16_t *ebase = elut_buf;
                uint16_t *e3 = elut_buf + (X3F_TONE_LUT_SIZE + 1);
                build_tone_lut(ep, ebase, 0);      /* 勾选态基准 */
                if (align_look_on() && !classic_skip_align_look() && !sdqh_native_display()
                    && !ep->align_look_off) {
                    fold_align_look(ebase, e3,
                                    e3 + (X3F_TONE_LUT_SIZE + 1),
                                    e3 + 2 * (X3F_TONE_LUT_SIZE + 1));
                    ctx.elut = e3;
                    ctx.elutg = e3 + (X3F_TONE_LUT_SIZE + 1);
                    ctx.elutb = e3 + 2 * (X3F_TONE_LUT_SIZE + 1);
                } else {
                    ctx.elut = ctx.elutg = ctx.elutb = ebase;
                }
                ctx.edge_mask = emask;
            } else {
                /* 资源不足 → 整体降级回现行为（宁可不限幅，不出错图） */
                free(elut_buf); free(emask);
                elut_buf = NULL; emask = NULL;
            }
        }
        x3f_parallel_for((size_t)h, apply_row, &ctx);
        free(elut_buf);
        free(emask);
    }
    /* 黑白「胶片颗粒」：模板 + 高斯 + 叠加（mono_grain=0 时一个字节不动，
     * 彩色路径 / 关颗粒的黑白路径与加这功能之前逐字节相同）。 */
    if (ctx.mono != NULL && ep->mono_grain) {
        mono_grain_apply((uint8_t *)out, w, h, stride, bits,
                         ep->mono_grain_size, ep->mono_grain_roughness);
    }
    return X3FRENDER_OK;
}

/* ---------- 可分离盒式模糊（锐度/细节用） ---------- */

#define XCL(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

/* ===================================================================
 * SPP「过曝矫正」的对应实现
 * （结构照 ：DP23HLPsRemoval / F23HPRemoval，
 * 推导见 §9–§16）
 *
 * C 侧默认**关**（X3F_HLP_FIX=1 才跑）。App 里由 `RawEditParams.hlpFix` 显式设置，
 * 2026-10-15 起那个默认改成 **true**（SPP 参数文件里彩色的 `X3F_Hpc=1`，
 * 见 §14.1）。
 * X3F_HLP_DRYRUN=1 只统计不改；X3F_HLP_DEBUG=1 打计数。
 *
 * 复刻到的三段：
 *   a) 标记 = HLP 网格 ∧ 饱和
 *        列 % 4 == 0 且 行 % 16 == 0            （CAMF HighlightPixelsInfo=[0,0,16,4]）
 *        max(逐通道饱和比) >= 14300/16383 = 0.8728   （= MarkSaturationArea 的
 *        thr = 14300/ISOGain；在"归一化到 ISO100"的域里它与 ISO 无关）
 * b) 判据（F23HPRemoval–0x161710 的投票）
 *        sigma_c(k) = sqrt(|c0 + c1·|k| + c2·k²|)   k = 该通道的 RAW 码值
 *        K(k)       = sqrt(|b0 + b1·|k| + b2·k²|)   ≈ 4.5 倍标准差
 *   c) 取值（§16 的三选一）
 *        min3/max3/local = 该通道 3×3 邻域的最小/最大/中心
 *        thr  = sigma_c · K
 *        cand = (min3 <= thr) ? ((max3 - local) > thr ? max3 : local) : min3
 *
 * 噪声模型系数直接来自 的 rodata（ 三组三元组 / 0x19e944 一组），
 * 单位是 RAW 码值 0…16383。
 * =================================================================== */
#define X3F_HLP_SAT_RATIO 0.8728      /* 14300 / 16383 */
#define X3F_HLP_RAW_MAX   16383.0
/* 噪声模型的单位是 RAW 码值(0…16383)，而 scene-linear 是"归一化饱和比"(≈0…1)，
 * 所以阈值要除以 16383 才能和邻域的差值直接比。 */
#define X3F_HLP_RAW_TO_LIN (1.0 / X3F_HLP_RAW_MAX)

static const double g_hlp_sigma[3][3] = {
    { 24.362,   0.04835,   -1.25e-07 },   /* R */
    { 15.1643,  0.0275,    -5e-08    },   /* G */
    { 1.30269,  0.0271375,   5e-08   },   /* B */
};
static const double g_hlp_k[3] = { 20.843, 0.4342, 8e-07 };

static int g_hlp_fix = -1;

/* SPP「过曝矫正」勾选框的对应开关。可用 x3f_render_set_hlp_fix() 或环境变量
 * X3F_HLP_FIX=1 打开；C 侧默认关（App 里默认开，见上面的说明）。 */
void x3f_render_set_hlp_fix(int enable) { g_hlp_fix = enable ? 1 : 0; }

int x3f_render_hlp_fix_enabled(void)
{
    if (g_hlp_fix < 0) {
        const char *e = getenv("X3F_HLP_FIX");
        g_hlp_fix = (e && *e && *e != '0') ? 1 : 0;
    }
    return g_hlp_fix;
}

static inline double hlpfx_model(const double c[3], double k)
{
    double a = k < 0 ? -k : k;
    double v = c[0] + c[1] * a + c[2] * k * k;
    if (v < 0) v = -v;
    return sqrt(v);
}

/* 扫一遍全图：对被标记的点按 §16 的规则改值；dry_run 时只统计 */
static long apply_hlp_fix(float *rgb, unsigned w, unsigned h,
                          const double *rgb_to_cam, int dry_run)
{
    long marked = 0;
    unsigned x, y;
    if (rgb == NULL || w < 4 || h < 4 || rgb_to_cam == NULL) return 0;

    for (y = 1; y + 1 < h; y++) {
        if ((y % 16) != 0) continue;                 /* HLP 网格：行 */
        for (x = 1; x + 1 < w; x++) {
            float *px = rgb + ((size_t)y * w + x) * 3;
            double sar[3], mx = 0, out[3];
            int c, dy, dx;
            if ((x % 4) != 0) continue;              /* HLP 网格：列 */

            /* 逐通道饱和比（1.0 = 顶到传感器上限） */
            for (c = 0; c < 3; c++) {
                sar[c] = rgb_to_cam[c * 3 + 0] * px[0]
                       + rgb_to_cam[c * 3 + 1] * px[1]
                       + rgb_to_cam[c * 3 + 2] * px[2];
                if (sar[c] > mx) mx = sar[c];
            }
            if (mx < X3F_HLP_SAT_RATIO) continue;    /* MarkSaturationArea 的判据 */
            marked++;
            if (dry_run) continue;

            /* 注意：网格点之间相隔 ≥4 列 / 16 行，所以"边改边读"不会读到刚改过的点 */
            for (c = 0; c < 3; c++) {
                double mn = 1e30, mx3 = -1e30, local = px[c], k, sigma, thr, cand;
                for (dy = -1; dy <= 1; dy++)
                    for (dx = -1; dx <= 1; dx++) {
                        double v = rgb[((size_t)(y + dy) * w + (x + dx)) * 3 + c];
                        if (v < mn) mn = v;
                        if (v > mx3) mx3 = v;
                    }
                k = sar[c] * X3F_HLP_RAW_MAX;
                sigma = hlpfx_model(g_hlp_sigma[c], k);
                thr = sigma * hlpfx_model(g_hlp_k, k) * X3F_HLP_RAW_TO_LIN;
                /*照–0x161744 的分支：
                 *   ref <= thr ? ( ((s10 - s21) > thr) ? s10 : s21 ) : s23
                 * 其中 s23 = 邻域 MAX、s10 = 邻域 MIN、s21 = 中心值、s15 = thr。
                 * 我们这一侧没有 SPP 那个"参考源(ref)"，用本点值 local 顶替。 */
                if (local <= thr)
                    cand = ((mn - local) > thr) ? mn : local;
                else
                    cand = mx3;
                if (cand < 0) cand = 0;
                out[c] = cand;
            }
            px[0] = (float)out[0];
            px[1] = (float)out[1];
            px[2] = (float)out[2];
        }
    }
    return marked;
}

/* ---------- HLP 高光混合（实验，默认关） ----------
 *
 * 依据 2026-10-18 的抓包结论：OECVer3 入口主图封顶 14,000，
 * 与 post-gain HLP 图按
 *   t = clamp((mean*4 - 44000) / 12000, 0, 1)
 *   out = (1-t)*main + t*HLP_post*k
 * 混合，post-gain HLP 约为 pre-gain 的 3.8–4.6 倍。
 *
 * X3F_OE_HLP=1 打开；默认路径逐字节不变。
 * X3F_HLP_GUIDE=/path/guide.bin 可喂抓包 HLP（459*184*3 float，
 * 即 post-gain 码值域，内部 /16383 归一化）；不喂时从当前线性图
 * 按 16×4 块均值生成 guide，并按 ×4 近似空间增益。
 */
/* 显式 API（x3f_render_set_oe_hlp）优先于环境变量；-1 = 跟随环境变量。 */
static int g_oe_hlp = -1;

static int oe_hlp_enabled(void)
{
    if (g_oe_hlp >= 0) return g_oe_hlp != 0 ? 1 : 0;
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("X3F_OE_HLP");
        v = (e != NULL && *e != '\0' && *e != '0') ? 1 : 0;
    }
    return v;
}

static int load_hlp_guide(const char *path, int gw, int gh, float **out)
{
    FILE *f;
    size_t n = (size_t)gw * (size_t)gh * 3;
    float *g;
    if (path == NULL || *path == '\0' || out == NULL) return 0;
    f = fopen(path, "rb");
    if (f == NULL) return 0;
    g = (float *)malloc(n * sizeof(float));
    if (g == NULL) { fclose(f); return 0; }
    if (fread(g, sizeof(float), n, f) != n) {
        free(g); fclose(f); return 0;
    }
    fclose(f);
    *out = g;
    return 1;
}

static void make_hlp_guide(const float *rgb, unsigned w, unsigned h,
                           int gw, int gh, float *guide)
{
    int gx, gy;
    for (gy = 0; gy < gh; gy++) {
        unsigned y0 = (unsigned)((double)gy * h / gh);
        unsigned y1 = (unsigned)((double)(gy + 1) * h / gh);
        if (y1 <= y0) y1 = y0 + 1;
        if (y1 > h) y1 = h;
        for (gx = 0; gx < gw; gx++) {
            unsigned x0 = (unsigned)((double)gx * w / gw);
            unsigned x1 = (unsigned)((double)(gx + 1) * w / gw);
            double sum[3] = {0, 0, 0};
            unsigned x, y, cnt = 0;
            if (x1 <= x0) x1 = x0 + 1;
            if (x1 > w) x1 = w;
            for (y = y0; y < y1; y++) {
                for (x = x0; x < x1; x++) {
                    const float *p = rgb + ((size_t)y * w + x) * 3;
                    sum[0] += p[0]; sum[1] += p[1]; sum[2] += p[2];
                    cnt++;
                }
            }
            if (cnt == 0) cnt = 1;
            guide[((size_t)gy * gw + gx) * 3 + 0] = (float)(sum[0] / cnt);
            guide[((size_t)gy * gw + gx) * 3 + 1] = (float)(sum[1] / cnt);
            guide[((size_t)gy * gw + gx) * 3 + 2] = (float)(sum[2] / cnt);
        }
    }
}

static long apply_oe_hlp_blend(float *rgb, unsigned w, unsigned h)
{
    const double k[3] = {1.114, 1.142, 1.148};
    const char *gp = getenv("X3F_HLP_GUIDE");
    int gw = (int)w / 16, gh = (int)h / 4;
    float *guide = NULL;
    int external = 0;
    double scale;
    long touched = 0;
    unsigned x, y;
    if (rgb == NULL || w < 16 || h < 4 || gw < 1 || gh < 1) return 0;
    if (gp != NULL && *gp != '\0')
        external = load_hlp_guide(gp, gw, gh, &guide);
    if (guide == NULL) {
        guide = (float *)malloc((size_t)gw * gh * 3 * sizeof(float));
        if (guide == NULL) return 0;
        make_hlp_guide(rgb, w, h, gw, gh, guide);
        external = 0;
    }
    scale = external ? (1.0 / 16383.0) : 4.0;
    {
        const char *eg = getenv("X3F_OE_HLP_GAIN");
        if (eg != NULL && *eg != '\0') scale = atof(eg);
    }

    for (y = 0; y < h; y++) {
        double gy = (gh > 1) ? (double)y * (gh - 1) / (h - 1) : 0.0;
        int y0 = (int)gy, y1 = y0 + 1;
        double fy = gy - y0;
        if (y1 >= gh) { y1 = gh - 1; fy = 0.0; }
        for (x = 0; x < w; x++) {
            float *p = rgb + ((size_t)y * w + x) * 3;
            double gx = (gw > 1) ? (double)x * (gw - 1) / (w - 1) : 0.0;
            int x0 = (int)gx, x1 = x0 + 1;
            double fx = gx - x0, m, t;
            double hval[3];
            int c;
            if (x1 >= gw) { x1 = gw - 1; fx = 0.0; }
            m = (p[0] + p[1] + p[2]) / 3.0 * 14000.0;
            t = (m * 4.0 - 44000.0) / 12000.0;
            if (t <= 0.0) continue;
            if (t > 1.0) t = 1.0;
            for (c = 0; c < 3; c++) {
                double a = guide[((size_t)y0 * gw + x0) * 3 + c];
                double b = guide[((size_t)y0 * gw + x1) * 3 + c];
                double d = guide[((size_t)y1 * gw + x0) * 3 + c];
                double e = guide[((size_t)y1 * gw + x1) * 3 + c];
                hval[c] = (a * (1 - fx) + b * fx) * (1 - fy)
                        + (d * (1 - fx) + e * fx) * fy;
            }
            for (c = 0; c < 3; c++) {
                double v = (1.0 - t) * p[c] + t * hval[c] * scale * k[c];
                if (v < 0.0) v = 0.0;
                p[c] = (float)v;
            }
            touched++;
        }
    }
    free(guide);
    return touched;
}

static int oe_hlp_mode(void)
{
    if (g_oe_hlp >= 0) return g_oe_hlp;   /* 显式 API 优先于环境变量 */
    static int v = -2;
    if (v == -2) {
        const char *e = getenv("X3F_OE_HLP");
        v = (e != NULL && *e != '\0') ? atoi(e) : 0;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
    }
    return v;
}

/* ---------- SPP Stage2 HighlightNeutralization（实验） ----------
 *
 * 依据见 §19：
 *   p     = in * wb * PGAGain
 *   alpha = (p.r+p.g+p.b)/3
 *   t     = clamp((alpha - Low) / (High - Low), 0, 1)
 *   out   = ((1-t)*p + t*(L,L,L)) / PGAGain
 *
 * 这里不默认启用；X3F_OE_NEUT=1 在 scene-linear（CCM 之后）做，
 * X3F_OE_NEUT=2 先反解回相机域（CCM 之前）再做—— kernel 在
 * Stage2 的 CCM 之前，后者才是同口径。阈值可用
 * X3F_OE_NEUT_LO / X3F_OE_NEUT_HI 覆盖，便于和 A/B 定标。
 *
 * mode 3 才是 SPP kernel 的 p 层（conv⁻¹ 再乘 as-shot WB）；
 * 早先试过的"再乘一次 BMT⁻¹"会偏绿，已从代码里去掉。
 * 只在实验工具里用；默认路径（不设环境变量）逐字节不变。 */
/* App 的显式置位（x3f_render_set_oe_neutralize）优先于环境变量：
 * -1 = 跟随 X3F_OE_NEUT（进程内没被置过时的默认），≥0 = 强制。 */
static int g_oe_neut = -1;

void x3f_render_set_oe_neutralize(int mode) { g_oe_neut = mode; }

/* 「过曝矫正」HLP / OECVer3 阶段（环境变量 X3F_OE_HLP）：
 * 0 = 关；1 = 内部 guide 混合（apply_oe_hlp_blend）。
 * -1 = 跟随 X3F_OE_HLP 环境变量（进程内没被显式置位时的默认）。
 * 与 x3f_render_set_oe_neutralize 平行：App 在每次 load 之前显式置位，
 * 免得拿着上一次调用留下的全局值跑。 */
void x3f_render_set_oe_hlp(int mode) {
    if (mode < 0) mode = -1;
    if (mode > 1) mode = 1;
    g_oe_hlp = mode;
}

static int oe_neut_mode(void)
{
    if (g_oe_neut >= 0) {
        /* 实验旁路：CLI/App 显式置位优先，但 X3F_OE_NEUT_FORCE 可以强行改道
         * （不设或空 = 完全不动，默认路径逐字节不变）。 */
        const char *ef = getenv("X3F_OE_NEUT_FORCE");
        if (ef != NULL && *ef != '\0') {
            int m = atoi(ef);
            if (m >= 0) return m > 9 ? 9 : m;   /* 8 = SPP spec 直通 / 9 = mode8+p 层高光重建（实验） */
        }
        /* App/CLI 显式置位也放行 8/9（2026-10-25 起「过曝矫正」四档：
         * 4=关 / 7=逐通道重建 / 8=SPP Stage2 直通重放 / 9=mode8+p 层高光重建 v1）。
         * 10+ 视为非法回 7。 */
        return g_oe_neut > 9 ? 7 : g_oe_neut;
    }
    static int mode = -2;
    if (mode == -2) {
        const char *e = getenv("X3F_OE_NEUT");
        mode = (e != NULL && *e != '\0') ? atoi(e) : 0;
        if (mode < 0) mode = 0;
        if (mode > 9) mode = 9;   /* 9 = mode8+p 层高光重建（2026-10-26 env 放行，与 FORCE 分支一致） */
    }
    return mode;
}

/* 两段式第二段临时旁路 mode8 重放时用来保存/恢复当前档位。放在
 * oe_neut_mode() 之后，避免前向声明。 */
int x3f_render_get_oe_neutralize(void) { return oe_neut_mode(); }

/* 取这张照片的 as-shot 白平衡增益（和 x3f_decode_probe 打出来的是同一组）。 */
static int read_as_shot_gain(const char *path, double g[3])
{
    FILE *f;
    x3f_t *x3f;
    char *wb;
    int ok;
    if (path == NULL || g == NULL) return 0;
    f = fopen(path, "rb");
    if (f == NULL) return 0;
    x3f = x3f_new_from_file(f);
    if (x3f == NULL) { fclose(f); return 0; }
    x3f_load_data(x3f, x3f_get_camf(x3f));
    wb = x3f_get_wb(x3f);
    ok = x3f_get_gain(x3f, wb, g);
    x3f_delete(x3f);
    fclose(f);
    return ok;
}

static long apply_oe_neutralize(float *rgb, unsigned w, unsigned h,
                                const double *rgb_to_cam,
                                const double *cam_to_rgb,
                                int in_cam, double low, double high)
{
    long touched = 0;
    unsigned x, y;
    int c;
    if (rgb == NULL || w == 0 || h == 0) return 0;
    if (!(high > low)) return 0;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            float *px = rgb + ((size_t)y * w + x) * 3;
            double p[3], q[3], alpha, t;
            if (in_cam) {
                double s[3];
                s[0] = px[0]; s[1] = px[1]; s[2] = px[2];
                x3f_3x3_3x1_mul((double *)rgb_to_cam, s, p);
            } else {
                p[0] = px[0]; p[1] = px[1]; p[2] = px[2];
            }
            alpha = (p[0] + p[1] + p[2]) / 3.0;
            t = (alpha - low) / (high - low);
            if (t <= 0.0) continue;
            if (t > 1.0) t = 1.0;
            touched++;
            for (c = 0; c < 3; c++)
                p[c] = (1.0 - t) * p[c] + t * alpha;
            if (in_cam) {
                x3f_3x3_3x1_mul((double *)cam_to_rgb, p, q);
                px[0] = (float)q[0];
                px[1] = (float)q[1];
                px[2] = (float)q[2];
            } else {
                px[0] = (float)p[0];
                px[1] = (float)p[1];
                px[2] = (float)p[2];
            }
        }
    }
    return touched;
}

/*同一条 kernel，但严格按 域：相机 RAW → ×WB 增益（= kernel 的 p），
 * 中性化后再除回增益、走相机→显示矩阵。 HighlightNeutralization
 * 就在这个位置（Stage2、CCM 之前）。 */
static long apply_oe_neutralize_wb(float *rgb, unsigned w, unsigned h,
                                   const double *rgb_to_cam,
                                   const double *cam_to_rgb,
                                   const double gain[3],
                                   double low, double high)
{
    long touched = 0;
    unsigned x, y;
    int c;
    if (rgb == NULL || w == 0 || h == 0) return 0;
    if (!(high > low)) return 0;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            float *px = rgb + ((size_t)y * w + x) * 3;
            double s[3], cam[3], p[3], q[3], alpha, t;
            s[0] = px[0]; s[1] = px[1]; s[2] = px[2];
            x3f_3x3_3x1_mul((double *)rgb_to_cam, s, cam);
            for (c = 0; c < 3; c++) p[c] = cam[c] * gain[c];
            alpha = (p[0] + p[1] + p[2]) / 3.0;
            t = (alpha - low) / (high - low);
            if (t <= 0.0) continue;
            if (t > 1.0) t = 1.0;
            touched++;
            for (c = 0; c < 3; c++) {
                p[c] = (1.0 - t) * p[c] + t * alpha;
                q[c] = p[c] / gain[c];
            }
            x3f_3x3_3x1_mul((double *)cam_to_rgb, q, s);
            px[0] = (float)s[0];
            px[1] = (float)s[1];
            px[2] = (float)s[2];
        }
    }
    return touched;
}

/* mode 5：HLN 之后不再走 cam_to_rgb，而是走 SPP Stage2 的 CCMRawBlending
 * 矩阵（CAMF AutoCCMatrix）。oracle 已确认：HLN 留下的少量暖色会被这张
 * 矩阵强烈放大，而中性像素因为行和≈1 保持不变。
 * ⚠️ 这里用的是"全 CCM"，没有 CCMRawBlending 的 raw/CCM 亮度混合；
 * 实测会把 224–240 推成 R216/G246/B196。等 `fRawBlendMult` 定标后再接产品。 */
static long apply_oe_neutralize_ccm(float *rgb, unsigned w, unsigned h,
                                    const double *rgb_to_cam,
                                    const double *cam_to_rgb,
                                    const double *ccm,
                                    const double gain[3],
                                    double low, double high, double mix)
{
    static int neut2 = -1;
    static double lo2 = 0.25, hi2 = 0.50;
    if (neut2 < 0) {
        const char *e = getenv("X3F_OE_NEUT2");
        const char *l = getenv("X3F_OE_NEUT2_LO");
        const char *h = getenv("X3F_OE_NEUT2_HI");
        neut2 = (e != NULL && *e != '\0' && *e != '0') ? 1 : 0;
        if (l != NULL && *l != '\0') lo2 = atof(l);
        if (h != NULL && *h != '\0') hi2 = atof(h);
    }
    long touched = 0;
    unsigned x, y;
    int c;
    if (rgb == NULL || w == 0 || h == 0 || ccm == NULL) return 0;
    if (!(high > low)) return 0;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            float *px = rgb + ((size_t)y * w + x) * 3;
            double s[3], cam[3], p[3], a[3], b[3], alpha, t;
            s[0] = px[0]; s[1] = px[1]; s[2] = px[2];
            x3f_3x3_3x1_mul((double *)rgb_to_cam, s, cam);
            for (c = 0; c < 3; c++) p[c] = cam[c] * gain[c];
            alpha = (p[0] + p[1] + p[2]) / 3.0;
            t = (alpha - low) / (high - low);
            if (t <= 0.0) continue;
            if (t > 1.0) t = 1.0;
            touched++;
            for (c = 0; c < 3; c++) {
                p[c] = (1.0 - t) * p[c] + t * alpha;
                cam[c] = p[c] / gain[c];
            }
            x3f_3x3_3x1_mul((double *)ccm, cam, a);
            x3f_3x3_3x1_mul((double *)cam_to_rgb, cam, b);
            px[0] = (float)((1.0 - mix) * b[0] + mix * a[0]);
            px[1] = (float)((1.0 - mix) * b[1] + mix * a[1]);
            px[2] = (float)((1.0 - mix) * b[2] + mix * a[2]);
            if (neut2) {
                double q0 = px[0], q1 = px[1], q2 = px[2];
                double am = (q0 + q1 + q2) / 3.0;
                double t2 = (am - lo2) / (hi2 - lo2);
                if (t2 > 0.0) {
                    if (t2 > 1.0) t2 = 1.0;
                    px[0] = (float)((1.0 - t2) * q0 + t2 * am);
                    px[1] = (float)((1.0 - t2) * q1 + t2 * am);
                    px[2] = (float)((1.0 - t2) * q2 + t2 * am);
                }
            }
        }
    }
    return touched;
}

/* mode 6：把 two-sample 拟合里“去掉 CCM 后的固定部分”直接作用到 raw：
 * X ≈ M_fit · inv(AutoCCMatrix)，两张样张平均。
 * 这是 Stage3/输出 gamut 映射的一个线性近似，不依赖逐图 CCM。 */
static long apply_oe_neutralize_fixed(float *rgb, unsigned w, unsigned h,
                                      const double *rgb_to_cam,
                                      const double gain[3],
                                      double low, double high)
{
    static const double X[9] = {
        0.5275, 0.5380, -0.0515,
        0.3015, 0.6475,  0.0705,
        0.0990, 0.6970,  0.2055
    };
    long touched = 0;
    unsigned x, y;
    int c;
    if (rgb == NULL || w == 0 || h == 0) return 0;
    if (!(high > low)) return 0;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            float *px = rgb + ((size_t)y * w + x) * 3;
            double s[3], cam[3], p[3], out[3], alpha, t;
            s[0] = px[0]; s[1] = px[1]; s[2] = px[2];
            x3f_3x3_3x1_mul((double *)rgb_to_cam, s, cam);
            for (c = 0; c < 3; c++) p[c] = cam[c] * gain[c];
            alpha = (p[0] + p[1] + p[2]) / 3.0;
            t = (alpha - low) / (high - low);
            if (t <= 0.0) continue;
            if (t > 1.0) t = 1.0;
            touched++;
            for (c = 0; c < 3; c++) {
                p[c] = (1.0 - t) * p[c] + t * alpha;
                cam[c] = p[c] / gain[c];
            }
            x3f_3x3_3x1_mul((double *)X, cam, out);
            px[0] = (float)out[0];
            px[1] = (float)out[1];
            px[2] = (float)out[2];
        }
    }
    return touched;
}

static void box_blur(float *a, float *tmp, int w, int h, int r)
{
    const float norm = 1.0f / (float)(2 * r + 1);
    int x, y, i;

    for (y = 0; y < h; y++) {
        float *row = a + (size_t)y * w;
        float s = 0;
        for (i = -r; i <= r; i++) s += row[XCL(i, 0, w - 1)];
        for (x = 0; x < w; x++) {
            tmp[(size_t)y * w + x] = s * norm;
            s += row[XCL(x + r + 1, 0, w - 1)] - row[XCL(x - r, 0, w - 1)];
        }
    }
    for (x = 0; x < w; x++) {
        float s = 0;
        for (i = -r; i <= r; i++) s += tmp[(size_t)XCL(i, 0, h - 1) * w + x];
        for (y = 0; y < h; y++) {
            a[(size_t)y * w + x] = s * norm;
            s += tmp[(size_t)XCL(y + r + 1, 0, h - 1) * w + x]
               - tmp[(size_t)XCL(y - r, 0, h - 1) * w + x];
        }
    }
}

static void blur2(float *a, float *tmp, int w, int h, int r)
{
    box_blur(a, tmp, w, h, r);
    box_blur(a, tmp, w, h, r);
}

/* ---------- 线性 sRGB 载入 ---------- */



/* RAW 饱和分：把每个通道"是否顶到传感器上限"平滑地累加。
 * 0 = 都没顶；1 = 一个通道顶；2 = 两个；3 = 三个都顶。
 * 为什么用"几个通道顶"而不是"最亮通道顶没顶"：
 *   纯色物体（如亮黄花）只有一个通道顶，色彩仍然可信，不该去饱和；
 *   白/高光区往往两个以上通道同时顶，色彩不可信（会被白平衡推成品红），
 *   这时候才需要收色。 */
static inline double clip_score(const double *in)
{
    double s = 0.0;
    int i;
    for (i = 0; i < 3; i++) {
        /* 门槛 0.97 → 0.90：DP3M2109 的镀铬高光上，三通道是
         * (R 0.68, G 0.958, B 0.983) —— 真正快顶的是 G/B，但 G 差 0.012
         * 没进 0.97 的门，只数到 B 一个通道（饱和分 0.41），去饱和不启动，
         * 而 R 的通道增益 2.7 把线性值放大到 2.55、B 变负 → 一条黄带。
         * 放到 0.90 之后 G 也计入，饱和分 ≈1.5，去饱和正常触发。
         * 注意单通道饱和的纯色物体仍只算 1 个通道，不受影响。 */
        double t = (in[i] - 0.90) / 0.10;     /* 0.90..1.00 渐入 */
        if (t <= 0.0) continue;
        if (t > 1.0) t = 1.0;
        s += t * t * (3.0 - 2.0 * t);
    }
    return s;
}

/* 取一个源像素，归一化后过色彩矩阵。
 * 顺序与上游 convert_data() 一致：先归一化 → 乘彩色阴影增益 → 上矩阵。
 * 饱和分按**未加增益**的值算：饱和是传感器属性，跟标定增益无关。 */


/* 降采样版。
 *
 * 关键：**先在原始 uint16 上平均，再做归一化和色彩矩阵**。
 * 归一化是仿射、矩阵乘法是线性，两者都跟"取平均"可交换：
 *     M·avg(a·(v-b))  ==  avg(M·a·(v-b))
 * 所以结果和"先变换每个像素再平均"等价（实测最大差 2.2e-08），
 * 但只对 1/down² 的像素做矩阵乘法。 */

/* crop_x/crop_y/crop_w/crop_h：只取一块区域。crop_w <= 0 表示整幅。
 * 裁剪坐标是**传感器朝向**的像素，会被夹进图像范围。 */
/* ---------- RAW 域亮度图（高光恢复用） ----------
 *
 * 逐像素取三通道 u16 的最大值，按 4×4 块平均。读的就是 get_image 输出的
 * 那份 u16（矩阵与 spatial gain 之前），所以和"另外解一遍 RAW 再取 max"
 * 逐位相同 —— 区别只是它发生在同一次解码里，省掉一次完整解码。
 * （换 x3fuse 之后一次解码 2 秒上下，省一次很值。） */
#define X3F_RAWMAX_BLOCK 4



/* ---- 解码/色彩来源 -----------------------------------------------------
 *
 * 1 = x3fuse 的 scene-linear —— **默认**。用的就是写 DNG 时那套色彩变换：
 *     相机标定 + 白平衡(AsShotNeutral) + 相对数字 ISO 增益，全部照相机元数据
 *     算好，只差显示用的色调曲线。所以 JPEG/HEIC/预览的色彩与 DNG 一致。
 *
 * 0 = 旧路径：自己拼矩阵（XYZ→sRGB · bmt_to_xyz · diag(as-shot 增益)），
 *     再靠 kMerrillFix 那块拟合补丁补偏色。它少了相机标定那一步，
 *     所以和同一个文件写出的 DNG 对不上 —— 保留只为 A/B 与 load_ex3
 *     （那个要 RAW 域的亮度图，scene-linear 里没有）。
 */
/* 高光重建开关：x3fuse 自带的那一套（chroma LUT / RepairPix / 矩阵病态门），
 * DNG 写出用的就是它，在 BMT（传感器）域重建被压住的层。
 * 默认开；X3F_HL_RECOVER=0 关掉做 A/B（比如某张片子上重建帮倒忙时）。 */
static int scene_hlrecover(void)
{
    const char *e = getenv("X3F_HL_RECOVER");
    return (e != NULL && e[0] == '0') ? 0 : 1;
}

/* NLM 降噪强度（0..10，见头文件 —— 它是 x3f_render_load_ex3 的参数）。
 *
 * x3fuse 的这套 NLM **本来就是只压色度的**
 * （denoise_nlm → fast_nlm_denoise_controlled(..., [0.0, h, h], 3, 11)，
 * 亮度权重 0；Quattro 那条更狠 [0,h,2h]），正是我们要的色度降噪。
 *
 * ex / ex2 / crop 那几条入口不降噪（预览、放大不该背这个成本），但留一个
 * 环境变量 X3F_NLM 让 lab 工具能对拍 —— 应用自己不会设它。 */
static unsigned char nlm_clamp(int v)
{
    if (v < 0) v = 0;
    if (v > 10) v = 10;
    return (unsigned char)v;
}

static unsigned char nlm_from_env(void)
{
    const char *e = getenv("X3F_NLM");
    return (e != NULL) ? nlm_clamp(atoi(e)) : 0;
}

extern int x3f_scene_linear(const char *path, float **data, unsigned *w,
                            unsigned *h, unsigned short *orientation,
                            double *camera_to_rgb,
                            int highlight_recovery,
                            unsigned char denoise_intensity,
                            const x3f_denoise_t *denoise,
                            const char *wb_preset);
extern void x3f_scene_linear_free(float *ptr);
/* ---- 自研 Quattro 解码核心分发钩子（x3fcore-hybrid）----
 * 契约与 x3f_scene_linear 完全一致；denoise 用 void* 携带 x3f_denoise_t*。
 * Quattro 静态照与 SFD 七帧（X3I 已展开成独立帧）走自研核心，
 * 其余机型原样转发 x3fuse；X3F_DECODER=fuse/own 可强制。
 * 链接：把 own_decoder.c 加进 CX3FRaw target sources。 */
extern int  x3f_own_dispatch_scene_linear(const char *path, float **data,
                                          unsigned *w, unsigned *h,
                                          unsigned short *orientation,
                                          double *camera_to_rgb,
                                          int highlight_recovery,
                                          unsigned char denoise_intensity,
                                          const void *denoise,
                                          const char *wb_preset);
extern void x3f_own_dispatch_scene_linear_free(float *data);
/* 上一次解码分发真实落在哪条后端（1=fuse / 0=own）。渲染层里按解码域标定的东西
 * （mode8 直通表、mode4 的饱和度阈值）靠它分路，详见 oe_spec_set_cur()。
 * 线程局部 —— 与 own_decoder.c 里那份同规格（render/worker 多线程）。 */
extern int  own_dispatch_used_fuse(void);
/* 逐通道裁剪标记（own_decoder.h 同声明）：mode7 过曝修复的输入 */
extern void      own_request_clip_mask(void);
extern uint8_t  *own_take_clip_mask(int *w, int *h);
/* GPU 降噪探针（x3fuse 0.2.0）。见 x3f_render_gpu_available()。 */
extern int x3f_denoise_gpu_probe(char *name_out, int cap);

void x3f_denoise_default(x3f_denoise_t *dn)
{
    if (dn == NULL) return;
    /* 与 0.1.6 的行为等价：NLM、只压色度、倍率 1.0、3/3/11/11、后端 auto。
     * 这几个默认值必须和 x3f-sys 的 DenoiseOptions::DEFAULT 对上，
     * 否则"什么参数都不传"和"传默认struct"会走出两张图。
     *
     * luminance 这里仍然是 0（关）：这张表是 C ABI 上的"什么都别设"，
     * 而 AUTO 是**产品默认**、由宿主显式填 2（Swift 侧 DenoiseLumaMode.auto）。
     * 填 2 之前请先想清楚 —— 见 x3f_render.h 里那组常量的说明。 */
    dn->algorithm     = X3F_DN_ALGO_NLM;
    dn->luminance     = 0;
    dn->chroma        = 1;
    dn->strength      = 1.0;
    dn->luma_scale    = 1.0;   /* 1.0 = 亮度不额外收力（老行为） */
    dn->luma_patch    = 3;
    dn->chroma_patch  = 3;
    dn->luma_search   = 11;
    dn->chroma_search = 11;
    dn->backend       = X3F_DN_BACKEND_AUTO;
}

/* AUTO 的门槛，`X3F_NR_AUTO_LUMA_ISO` 可覆盖（lab 扫阈值用）。 */
static double nr_auto_luma_iso(void)
{
    const char *e = getenv("X3F_NR_AUTO_LUMA_ISO");
    if (e != NULL && atof(e) > 0.0) return atof(e);
    return X3F_DN_LUMA_AUTO_ISO;
}

/* 判据本身，只写一处：ISO 大于等于门槛就带亮度。ISO 读不到（0）时给 0，
 * 也就是"判据不成立时不动亮度" —— 与 0.1.6 的行为一致。 */
static int nr_auto_luma_for_iso(double iso)
{
    return (iso > 0.0 && iso >= nr_auto_luma_iso()) ? 1 : 0;
}

/* AUTO 亮度档的**渐进**系数（2026-11-14）。
 *
 * 门槛只看"开不开"，但亮度 NLM 的力度不该在门槛上一步踩满：噪声幅度按 √ISO
 * 长，而 h 是绝对值 —— 门槛设 400 时，ISO 640 直接吃满 luma_scale 的力度，
 * 实测高频（拉普拉斯方差）掉 46%，观感就是"涂抹/变不清晰"。用户报的就是这条。
 *
 * 所以门槛处给 0，4× 门槛（默认 1600）处给满，中间线性：
 *   ISO 640  → 0.20 倍力度（轻微）
 *   ISO 1600 → 1.00（与既有标定一致，ISO 1600 的观感不变）
 * 显式选「开」（luminance=1）不走进来 —— 那是用户自己的决定，给满力度。 */
static double nr_auto_luma_ramp(double iso, double thr)
{
    double t;
    if (thr <= 0.0) return 1.0;
    t = (iso - thr) / (3.0 * thr);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return t;
}

double x3f_render_shooting_iso(const char *path)
{
    return read_shooting_iso(path);
}

int x3f_render_auto_luma(const char *path)
{
    return nr_auto_luma_for_iso(read_shooting_iso(path));
}

int x3f_render_gpu_available(char *name_out, int cap)
{
    /* 探测一次要几十毫秒（要建 wgpu Instance 并请求适配器），而且结果在一个
     * 进程里不会变，所以缓存。0 = 还没探过，1/2 = 不能用/能用。 */
    static int cached = 0;
    static char cached_name[128];

    if (cached == 0) {
        cached_name[0] = '\0';
        cached = x3f_denoise_gpu_probe(cached_name, (int)sizeof(cached_name)) ? 2 : 1;
    }
    if (cached == 2) {
        if (name_out != NULL && cap > 0)
            snprintf(name_out, (size_t)cap, "%s", cached_name);
        return 1;
    }
    if (name_out != NULL && cap > 0) name_out[0] = '\0';
    return 0;
}

/* ---- mode7 v3：裁剪感知的高光去饱和（2026-10-22 傍晚） ----
 *
 * v1：smoothstep(lo=0.75,hi=1.5) 向 luma 灰混合 —— 死白 23.7%（相机 24.3%），
 *     但 hi 太窄（m≥1.5 全饱和 t=1）+ 二值 mask + 裁剪通道分类权重，
 *     白核与橙层之间是硬边（用户：不同亮度之间过渡生硬）。
 * v2：通道比重建（未裁邻域 (ch/Y) 比率 × 本像素 Y 提亮被裁通道）——
 *     造出假黄色团，比 v1 更差。教训：从被裁数据做重建是放大垃圾信号；
 * 重建用的是 Quattro 半层原始数据（融合时已消费），短期内做不到。
 *
 * v3 = v1 的去饱和 + 三个平滑化：
 *   1. 宽渐变：默认 hi 提到 2.2（过渡带 ~1.5 档），白核与橙层之间是连续坡；
 *   2. 软门控：把二值 mask（任一通道裁剪）box blur 成软覆盖度 g，
 *      权重 = t · smoothstep(g)，消掉「裁剪/未裁」的缝；
 *   3. 无分类权重：全裁/部分裁用同一条曲线（部分裁像素 luma 被未裁通道
 *      拉住，色相天然保留）。
 * 只碰 mask≠0 且 m>lo 的像素；lo/hi 默认 0.75/2.2，X3F_OE_NEUT_LO/HI 覆盖；
 * X3F_OE_RESTORE_R 调 mask 软化半径（默认 6）。返回 touch 像素数。 */

/* separable box blur，原地（tmp 为工作缓冲），两遍调用 ≈ 三角核 */
static void blur_box_f(float *buf, float *tmp, unsigned w, unsigned h, int r)
{
    if (r <= 0) return;
    double *pre = (double *)malloc((size_t)(w > h ? w : h) * sizeof(double));
    if (pre == NULL) return;
    for (unsigned y = 0; y < h; y++) {          /* 水平 */
        float *row = buf + (size_t)y * w;
        double acc = 0.0;
        for (unsigned x = 0; x < w; x++) { acc += row[x]; pre[x] = acc; }
        for (unsigned x = 0; x < w; x++) {
            int x0 = (int)x - r; if (x0 < 0) x0 = 0;
            int x1 = (int)x + r; if (x1 >= (int)w) x1 = (int)w - 1;
            double s = pre[x1] - (x0 > 0 ? pre[x0 - 1] : 0.0);
            tmp[x] = (float)(s / (double)(x1 - x0 + 1));
        }
        memcpy(row, tmp, w * sizeof(float));
    }
    for (unsigned x = 0; x < w; x++) {          /* 垂直 */
        double acc = 0.0;
        for (unsigned y = 0; y < h; y++) {
            float *row = buf + (size_t)y * w;
            acc += row[x]; pre[y] = acc;
        }
        for (unsigned y = 0; y < h; y++) {
            int y0 = (int)y - r; if (y0 < 0) y0 = 0;
            int y1 = (int)y + r; if (y1 >= (int)h) y1 = (int)h - 1;
            double s = pre[y1] - (y0 > 0 ? pre[y0 - 1] : 0.0);
            buf[(size_t)y * w + x] = (float)(s / (double)(y1 - y0 + 1));
        }
    }
    free(pre);
}

/* ---- fuse 域合成裁剪标记（mode7 对齐，2026-10-25） ----
 * own 解码在 raw 层产出逐通道裁剪标记（半层 R/G + 顶层 B，按修复前 raw 判），
 * fuse 解码（x3fuse）不产标记 → mode7 一直退回 mode4 口径，与 own 视觉脱节。
 * 从 fuse 输出的 scene-linear 反解逐通道饱和比：
 *   scene_linear = cam_to_rgb · sat_ratio  ⇒  sat_ratio = rgb_to_cam · lin
 * 任一通道 ratio ≥ 阈值即置 bit0。阈值默认 1.00（own 按 raw 饱和值 sat[c]
 * 判裁剪 = 比率 1.0，口径一致；2501 实测 1.00 的 own↔fuse 火核 MAE 最优，
 * 0.95/0.98/1.05 无显著差异）；X3F_OE_FUSE_CLIP_T 覆盖。bit3（hlbase 重建
 * 跳过）fuse 域无对应物，恒 0 —— 裁剪平台（通道被拉平）恰是需要去饱和的
 * 垃圾区，全量处理是对的。
 * 返回 malloc 的 w*h 标记（调用方 free），失败返回 NULL。 */
static uint8_t *fuse_clip_mask_from_lin(const float *src, unsigned w, unsigned h,
                                        const double rgb_to_cam[9])
{
    uint8_t *m = (uint8_t *)malloc((size_t)w * h);
    if (m == NULL || src == NULL) { free(m); return NULL; }
    double thr = 1.00;
    const char *et = getenv("X3F_OE_FUSE_CLIP_T");
    if (et != NULL && *et != '\0') {
        double v = atof(et);
        if (v > 0.0) thr = v;
    }
    for (size_t i = 0; i < (size_t)w * h; i++) {
        double r = src[i * 3], g = src[i * 3 + 1], b = src[i * 3 + 2];
        double r0 = rgb_to_cam[0] * r + rgb_to_cam[1] * g + rgb_to_cam[2] * b;
        double r1 = rgb_to_cam[3] * r + rgb_to_cam[4] * g + rgb_to_cam[5] * b;
        double r2 = rgb_to_cam[6] * r + rgb_to_cam[7] * g + rgb_to_cam[8] * b;
        m[i] = (r0 >= thr || r1 >= thr || r2 >= thr) ? 1 : 0;
    }
    return m;
}

static long apply_oe_restore(float *rgb, const uint8_t *mask,
                             unsigned w, unsigned h, double lo, double hi)
{
    size_t n = (size_t)w * h;
    if (rgb == NULL || mask == NULL || !(hi > lo) || n == 0) return 0;
    int r = 6;
    const char *er = getenv("X3F_OE_RESTORE_R");
    if (er != NULL && *er != '\0') { r = atoi(er); if (r < 0) r = 0; }

    float *Wt  = (float *)malloc(n * sizeof(float));
    float *tmp = (float *)malloc(n * sizeof(float));
    if (Wt == NULL || tmp == NULL) { free(Wt); free(tmp); return 0; }

    long touched = 0;
    int nomask = 0;
    {
        const char *nm = getenv("X3F_OE_NOMASK");
        nomask = (nm != NULL && *nm != '\0' && nm[0] != '0');
    }
    for (size_t i = 0; i < n; i++) {
        /* 门控 = 裁剪过且**未**被 hlbase 重建（bit3）：这些像素还是平顶 +
         * 垃圾通道比 → 去饱和成白。bit3 置位的块已用半层真实亮度/色彩
         * 提回，不再碰（SPP 式重建路径）。
         * X3F_OE_NOMASK=1 = 旧 v3 行为（凡 mask≠0 都按 m 渐变去饱和）。 */
        float w = 0.0f;
        if (nomask) w = (mask[i] != 0) ? 1.0f : 0.0f;
        else if (mask[i] != 0 && !(mask[i] & 8)) w = 1.0f;
        Wt[i] = w;
        if (w > 0.0f) touched++;                /* 粗计（含 t=0 的） */
    }
    if (r > 0) {
        blur_box_f(Wt, tmp, w, h, r);
        blur_box_f(Wt, tmp, w, h, r);
    }

    for (size_t i = 0; i < n; i++) {
        double g = Wt[i];
        if (g <= 0.003f) continue;
        if (g > 1.0f) g = 1.0f;
        g = g * g * (3.0 - 2.0 * g);            /* 软门控 smoothstep */
        double rr = rgb[i * 3], gg = rgb[i * 3 + 1], bb = rgb[i * 3 + 2];
        double m = rr > gg ? (rr > bb ? rr : bb) : (gg > bb ? gg : bb);
        double t = ((double)m - lo) / (hi - lo);
        if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
        t = t * t * (3.0 - 2.0 * t);
        Wt[i] = (float)(t * g);                 /* 先存权重场，下面统一平滑 */
    }
    /* 权重场空间平滑：把裁剪平台边缘（m 不连续）与 mask 缝一起摊开，
     * 消白核边沿的轮廓线。半径取 mask 软化半径的 2 倍。 */
    if (r > 0) {
        blur_box_f(Wt, tmp, w, h, 2 * r);
        blur_box_f(Wt, tmp, w, h, 2 * r);
    }
    for (size_t i = 0; i < n; i++) {
        double k = Wt[i];
        if (k <= 0.002) continue;
        double rr = rgb[i * 3], gg = rgb[i * 3 + 1], bb = rgb[i * 3 + 2];
        double luma = (rr + gg + bb) / 3.0;
        rgb[i * 3]     = (float)(rr + (luma - rr) * k);
        rgb[i * 3 + 1] = (float)(gg + (luma - gg) * k);
        rgb[i * 3 + 2] = (float)(bb + (luma - bb) * k);
    }
    free(Wt); free(tmp);
    /* 显示域肩部压缩（mode7 第三段；2026-10-22 深夜，**实验性、默认关**）：
     * 死白优势来自显示域（勾选态 ToneLUT 肩部 + 邻域替换），而本函数
     * 跑在色调 LUT 之前的 scene-linear 域 —— 实测这里压缩会被后面的 LOOK
     * 曲线重新顶回 255（22.4%→22.0%，无效）。要生效需把本段挪到色调映射
     * 之后（管线里 clip_out 平面已流出，改造点明确），待做。
     * X3F_OE_SHOULDER 调深度（0=关，默认 0）。 */
    {
        double depth = 0.0;
        const char *ed = getenv("X3F_OE_SHOULDER");
        if (ed != NULL && *ed != '\0') depth = atof(ed);
        if (depth > 0.0) {
            const double s0 = 0.8, s1 = 1.6;
            for (size_t i = 0; i < n; i++) {
                if (mask[i] == 0) continue;
                double rr = rgb[i * 3], gg = rgb[i * 3 + 1], bb = rgb[i * 3 + 2];
                double m = rr > gg ? (rr > bb ? rr : bb) : (gg > bb ? gg : bb);
                double t = (m - s0) / (s1 - s0);
                if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
                t = t * t * (3.0 - 2.0 * t);
                double k = 1.0 - depth * t;
                rgb[i * 3]     = (float)(rr * k);
                rgb[i * 3 + 1] = (float)(gg * k);
                rgb[i * 3 + 2] = (float)(bb * k);
            }
        }
    }
    return touched;
}

static int load_common_inner(const char *path, int down, int crop_x, int crop_y,
                             int crop_w, int crop_h, unsigned char nlm, const char *wb,
                             const x3f_denoise_t *dn, float **rgb_out, float **clip_out,
                             float **rawmax_out, int *rmw_out, int *rmh_out,
                             int *w_out, int *h_out, char *wb_out);

/* 读 PROPS 段的 CAMMODEL（如 "SIGMA SD1" / "SIGMA DP2 Merrill"）。1 = 拿到了。
 *
 * 只为 SD1 影调曲线的门控服务：SD1 没有 DBG_Auto* 块（x3f_camf_matrix_f64
 * 那条门控对它失效），而它的机型名只存在于 PROPS 段——process.rs 的
 * get_black_level 也是用 x3f_get_prop_entry 读同一个键（SIGMA SD1 黑电平
 * 修复同款读法），两边一致。Merrill/Quattro 渲染各自有专属路径，不会走到
 * 这里的调用点。 */
static int probe_cammodel(const char *path, char *out, int cap)
{
    FILE *f;
    x3f_t *x3f;
    int ok = 0;

    if (path == NULL || out == NULL || cap <= 0) return 0;
    out[0] = '\0';
    f = fopen(path, "rb");
    if (f == NULL) return 0;
    x3f = x3f_new_from_file(f);
    if (x3f == NULL) { fclose(f); return 0; }
    {
        x3f_directory_entry_t *prop = x3f_get_prop(x3f);
        if (prop != NULL && x3f_load_data(x3f, prop) == 0) {
            char *v = NULL;
            if (x3f_get_prop_entry(x3f, (char *)"CAMMODEL", &v) != 0 && v != NULL) {
                snprintf(out, (size_t)cap, "%s", v);
                ok = 1;
            }
        }
    }
    x3f_delete(x3f);
    fclose(f);
    return ok;
}

/* 墙钟（ms）——只给分阶段计时用（SFD 合成的耗时拆账） */
static double render_now_ms(void)
{
#if defined(_WIN32)
    LARGE_INTEGER f, c;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart * 1000.0 / (double)f.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
#endif
}

/* 上一次「解码分发」（x3f_own_dispatch_scene_linear）的耗时（ms），线程局部：
 * load_common_inner 在 dispatch 两端打点；x3f_sfd.c 的双槽流水线里每个解码
 * 线程读自己那份（随 job 结构带回主线程，不共享写）。 */
#if defined(_WIN32)
static __declspec(thread) double g_dispatch_ms = 0.0;
#else
static _Thread_local double g_dispatch_ms = 0.0;
#endif

double x3f_render_last_decode_ms(void)
{
    return g_dispatch_ms;
}

/* X3I(SFD) 容器：把选定帧落成一份临时独立 X3F 再走同一条链。渲染链下游
 * （CAMF 取数、ISO 判据、scene_linear）都是"按路径"的接口，一处物化全都受益；
 * 普通 X3F 时 x3f_standalone_path 原样返回 path，零开销。 */
static int load_common(const char *path, int down, int crop_x, int crop_y,
                       int crop_w, int crop_h, unsigned char nlm, const char *wb,
                       const x3f_denoise_t *dn, float **rgb_out, float **clip_out,
                       float **rawmax_out, int *rmw_out, int *rmh_out,
                       int *w_out, int *h_out, char *wb_out)
{
    char tmp[1024];
    const char *use = x3f_standalone_path(path, -1, tmp, sizeof(tmp));
    /* RCPS AUTO 门【暂关 2026-10-27 深夜】：用户实测反馈 Merrill 画质变差，
     * 停用默认开，回退到显式 X3F_RCPS=1 才跑。机制侧结论保留（SPP
     * 0x6a480 三参数>0 门控 = SPP 默认就跑；tif16 实测我们对墙/海等中反差
     * 纹理区改动过大——热力图呈内容形状，|v/box9(v)|<0.1 门在 9×9 尺度
     * 拦不住中反差纹理，随机邻采样在纹理里重排像素=视觉劣化）。
     * 待办：复刻对齐 调用位置/参数（SPP 是 F20 链内 mono/luma 域
     * 带参数 0.075/0.1/1.0 的调用，我们是 RGB lin 域全幅直上），或收紧
     * 门控后再评估默认开。 */
    /* x3f_rcps_set_auto(x3f_render_probe_is_merrill(use)); */
    x3f_rcps_set_auto(0);
    /* 探测 Classic-Foveon auto-tone 机型（SD14 等）：DBG_AutoContrast 只存在于
     * 这一代，是精确的 self-selecting gate。命中则读取三滑块（hilite/shadow/
     * contrast 即 DBG_AutoContrast 自身）供 build_tone_lut 施加经典 S 曲线。
     * 非该代文件（Merrill/Quattro）无此键，g_classic_active 保持 0，观感不变。
     *
     * 原版 SIGMA SD1 没有 DBG_Auto* 块，但同样需要相机影调曲线（它内嵌 JPEG
     * 走老一代引擎）—— 用 PROPS 段的 CAMMODEL 探测，走 SD1 专属实测曲线。
     * 注意 SD1 Merrill（升级款）带的是 Merrill 渲染路径，不能命中这里。 */
    {
        unsigned cd[3] = {1, 1, 1};
        double v = 0.0;
        g_classic_active = 0;
        g_classic_sd1 = 0;
        g_classic_exposure = 0.0;
        {
            char model[64] = "";
            if (probe_cammodel(use, model, (int)sizeof model) &&
                strcmp(model, "SIGMA SD1") == 0) {
                g_classic_active = 1;
                g_classic_sd1 = 1;
            }
        }
        if (!g_classic_active &&
            x3f_camf_matrix_f64(use, "DBG_AutoContrast", &v, 1, cd) == 1) {
            g_classic_active = 1;
            g_classic_contrast = v;
            x3f_camf_matrix_f64(use, "DBG_AutoHilite", &g_classic_hilite, 1, cd);
            x3f_camf_matrix_f64(use, "DBG_AutoShadow", &g_classic_shadow, 1, cd);
            x3f_camf_matrix_f64(use, "DBG_AutoExposureNA", &g_classic_exposure, 1, cd);
        }
    }
    /* Quattro 绿增强门控（2026-10-30）：探测当前文件是否 Quattro 世代，供
     * build_color_match_lut 决定是否对 +135° 绿箱施 +3% 增强。Merrill / Classic
     * 不施，不影响它们已交付的严格对齐。与 g_classic_active 同一套 per-load 套路。 */
    g_cm_quattro = x3f_render_probe_is_quattro(use);
    /* 「过曝矫正」mode 4 的机型门（见 g_oe_quattro_gen 注释）：Quattro 世代
     * 才允许跑 SPP Stage2 高光中性化，其余机型一律按「关」处理。 */
    g_oe_quattro_gen = x3f_render_probe_is_quattro_gen(use);
    /* 「皮肤卡粉」软压缩的机型门（见 g_hf_classic 注释）：只有经典 Foveon
     * 成品相对相机 JPEG 偏锐 1.3~1.9 倍，Merrill/Quattro 反而更软（0.65~0.78）。 */
    g_hf_classic = !x3f_render_probe_is_merrill(use)
                && !x3f_render_probe_is_quattro_gen(use);
    /* Classic 原生链机型门（与 g_hf_classic 同口径）：非 Merrill / 非 Quattro
     * 世代走 x3fuse 原生解码 + AutoRGBNeutral 白平衡修复，渲染层不叠后来的
     * 画质算子。Quattro / Merrill / X3I(SFD) 恒为 0 → 管线逐字节不变。 */
    g_classic_native = (!x3f_render_probe_is_merrill(use)
                     && !x3f_render_probe_is_quattro_gen(use)) ? 3 : 0;
    {
        /* 档位见 g_classic_native 注释：0 旧链 / 1 严格原生 / 2 早期相机影调
         * / 3 最小档。机型门优先：Quattro / Merrill 恒不进原生链。 */
        const char *cne = getenv("X3F_CLASSIC_NATIVE");
        if (g_classic_native && cne != NULL && *cne != '\0') {
            int v = atoi(cne);
            g_classic_native = (v >= 0 && v <= 3) ? v : 3;
        }
    }
    int rc = load_common_inner(use, down, crop_x, crop_y, crop_w, crop_h, nlm, wb, dn,
                               rgb_out, clip_out, rawmax_out, rmw_out, rmh_out,
                               w_out, h_out, wb_out);
    x3f_temp_release(tmp);
    return rc;
}

/* 保色高光恢复（实验，X3F_HL_CHROMA=1）：只修饱和像素的**色度**，不动亮度。
 *
 * 把图分成 16×16 块，用块内未饱和像素统计 R/G、B/G；空块由邻块迭代填补；
 * 饱和像素按所在块的比值重组颜色（亮度保持不变）。这样 hl-recover 把高光
 * 摊开时带出来的是邻域的橙色，而不是 L*p 那条路产生的绿/粉环。
 * X3F_HL_CHROMA_STR 调混合强度（默认 0.6）。 */
/* 保色高光恢复 v2：最近可信像素的色度填充（Voronoi 式）。
 *
 * 种子 = 未饱和（clip<0.5）且亮度>0.15 的像素；把它的色度 (R/Y, B/Y) 沿
 * 4 邻域/对角用两遍 chamfer 距离变换扩散进饱和区。背景暗像素被亮度门槛
 * 排除，所以火焰边缘补到的是最近的火焰本体颜色，而不是背景。
 * 只改色度、亮度 Y 原样保留。X3F_HL_VORONOI=1 打开，_STR 调强度（默认 0.85）。 */
static void hl_chroma_voronoi(float *rgb, const float *clip, int w, int h, double str)
{
    size_t np = (size_t)w * (size_t)h, i;
    float *dist = (float *)malloc(np * sizeof(float));
    float *cu = (float *)malloc(np * sizeof(float));
    float *cv = (float *)malloc(np * sizeof(float));
    int x, y;
    const float BIG = 1e9f;
    if (dist == NULL || cu == NULL || cv == NULL) {
        free(dist); free(cu); free(cv);
        return;
    }
    for (i = 0; i < np; i++) {
        double r = rgb[i * 3], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        double yv = 0.2126 * r + 0.7152 * g + 0.0722 * b;
        if (clip != NULL && clip[i] < 0.5 && yv > 0.15) {
            dist[i] = 0.0f;
            cu[i] = (float)(r / yv);
            cv[i] = (float)(b / yv);
        } else {
            dist[i] = BIG;
            cu[i] = cv[i] = 0.0f;
        }
    }
#define TRY(nx, ny, cost) do { \
        if ((nx) >= 0 && (ny) >= 0 && (nx) < w && (ny) < h) { \
            size_t q = (size_t)(ny) * (size_t)w + (size_t)(nx); \
            float nd = dist[q] + (cost); \
            if (nd < dist[(size_t)y * (size_t)w + (size_t)x]) { \
                dist[(size_t)y * (size_t)w + (size_t)x] = nd; \
                cu[(size_t)y * (size_t)w + (size_t)x] = cu[q]; \
                cv[(size_t)y * (size_t)w + (size_t)x] = cv[q]; \
            } \
        } \
    } while (0)
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x++) {
            TRY(x - 1, y, 1.0f);
            TRY(x, y - 1, 1.0f);
            TRY(x - 1, y - 1, 1.4142f);
            TRY(x + 1, y - 1, 1.4142f);
        }
    for (y = h - 1; y >= 0; y--)
        for (x = w - 1; x >= 0; x--) {
            TRY(x + 1, y, 1.0f);
            TRY(x, y + 1, 1.0f);
            TRY(x + 1, y + 1, 1.4142f);
            TRY(x - 1, y + 1, 1.4142f);
        }
#undef TRY
    for (i = 0; i < np; i++) {
        double yv, ur, vr, u2, v2, R, G, B;
        if (clip == NULL || clip[i] < 0.9 || dist[i] >= BIG * 0.5f) continue;
        yv = 0.2126 * rgb[i * 3] + 0.7152 * rgb[i * 3 + 1] + 0.0722 * rgb[i * 3 + 2];
        if (yv <= 1e-6) continue;
        ur = rgb[i * 3] / yv;
        vr = rgb[i * 3 + 2] / yv;
        u2 = ur + ((double)cu[i] - ur) * str;
        v2 = vr + ((double)cv[i] - vr) * str;
        R = u2 * yv;
        B = v2 * yv;
        G = (yv - 0.2126 * R - 0.0722 * B) / 0.7152;
        rgb[i * 3] = (float)R;
        rgb[i * 3 + 1] = (float)G;
        rgb[i * 3 + 2] = (float)B;
    }
    free(dist); free(cu); free(cv);
}

#define HLC_BLOCK 16
static void hl_chroma_recover(float *rgb, const float *clip, int w, int h, double str)
{
    int bw = (w + HLC_BLOCK - 1) / HLC_BLOCK;
    int bh = (h + HLC_BLOCK - 1) / HLC_BLOCK;
    int bx, by, i, pass;
    float *sr, *sb, *sr2, *sb2;
    unsigned char *have, *have2;
    size_t nblk = (size_t)bw * (size_t)bh, p;

    if (rgb == NULL || clip == NULL || w < 8 || h < 8 || nblk == 0) return;
    sr = (float *)calloc(nblk, sizeof(float));
    sb = (float *)calloc(nblk, sizeof(float));
    sr2 = (float *)calloc(nblk, sizeof(float));
    sb2 = (float *)calloc(nblk, sizeof(float));
    have = (unsigned char *)calloc(nblk, 1);
    have2 = (unsigned char *)calloc(nblk, 1);
    if (!sr || !sb || !sr2 || !sb2 || !have || !have2) {
        free(sr); free(sb); free(sr2); free(sb2); free(have); free(have2);
        return;
    }

    /* 1) 未饱和像素的色比，按块累加 */
    for (i = 0; i < w * h; i++) {
        double r = rgb[i * 3], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        size_t bi;
        if (clip[i] >= 0.5 || g <= 1e-4) continue;
        bi = (size_t)(i / w / HLC_BLOCK) * (size_t)bw
           + (size_t)((i % w) / HLC_BLOCK);
        sr[bi] += (float)(r / g);
        sb[bi] += (float)(b / g);
        if (have[bi] < 255) have[bi]++;
    }
    for (p = 0; p < nblk; p++) {
        if (have[p]) { sr[p] /= have[p]; sb[p] /= have[p]; }
    }

    /* 2) 空块用 8 邻域迭代填补（最多 16 轮） */
    for (pass = 0; pass < 16; pass++) {
        int filled = 0;
        memcpy(sr2, sr, nblk * sizeof(float));
        memcpy(sb2, sb, nblk * sizeof(float));
        memcpy(have2, have, nblk);
        for (by = 0; by < bh; by++)
            for (bx = 0; bx < bw; bx++) {
                size_t bi = (size_t)by * (size_t)bw + (size_t)bx;
                double r = 0.0, b = 0.0;
                int n = 0, dy, dx;
                if (have[bi]) continue;
                for (dy = -1; dy <= 1; dy++)
                    for (dx = -1; dx <= 1; dx++) {
                        int nx = bx + dx, ny = by + dy;
                        size_t ni;
                        if (nx < 0 || ny < 0 || nx >= bw || ny >= bh) continue;
                        ni = (size_t)ny * (size_t)bw + (size_t)nx;
                        if (!have[ni]) continue;
                        r += sr[ni]; b += sb[ni]; n++;
                    }
                if (n) { sr2[bi] = (float)(r / n); sb2[bi] = (float)(b / n); have2[bi] = 1; filled = 1; }
            }
        memcpy(sr, sr2, nblk * sizeof(float));
        memcpy(sb, sb2, nblk * sizeof(float));
        memcpy(have, have2, nblk);
        if (!filled) break;
    }

    /* 3) 饱和像素：按块的色比重组，亮度原样保留 */
    for (i = 0; i < w * h; i++) {
        size_t bi;
        double r, g, b, y, rr, bb, den, g2;
        if (clip[i] < 0.9) continue;
        bi = (size_t)(i / w / HLC_BLOCK) * (size_t)bw
           + (size_t)((i % w) / HLC_BLOCK);
        if (!have[bi]) continue;
        r = rgb[i * 3]; g = rgb[i * 3 + 1]; b = rgb[i * 3 + 2];
        y = 0.2126 * r + 0.7152 * g + 0.0722 * b;
        rr = sr[bi]; bb = sb[bi];
        den = 0.2126 * rr + 0.7152 + 0.0722 * bb;
        if (den <= 1e-4 || y <= 1e-6) continue;
        g2 = y / den;
        rgb[i * 3]     = (float)(r + (g2 * rr - r) * str);
        rgb[i * 3 + 1] = (float)(g + (g2 - g) * str);
        rgb[i * 3 + 2] = (float)(b + (g2 * bb - b) * str);
    }
    free(sr); free(sb); free(sr2); free(sb2); free(have); free(have2);
}

/* pre-fix lin 快照（2026-10-30 破案后接入）。
 * hole-fix 在 lin 缓存就位前施加。凡是**帧级判定统计**（mode8 apply 级
 * 过曝门控等）吃这条 lin 时，fix 拉亮天空会改变统计量、让整帧的渲染
 * 分支开关随机翻转——寺庙 DP2Q1224 实测：fix off 过曝占比 0.0000%
 * （mode8 跳过），fix on 1.93%（mode8 放行）→ 全帧走焰核重放曲线，
 * 黄墙按亮度分带整体移动（0-40 区 -9、40-80 区 -21、80-120 区 -44，
 * 渐变墙面上即用户所见条带）；关掉 mode8 后分带全部归零。
 * 这里在施加 fix 前快照一份 lin，帧级统计一律用快照——开关 fix 不再
 * 改变任何帧级分支判定，条带消失、天空修复保留。
 * 仅 fix 开启时占内存（全分辨率 ≈ w*h*12 字节）；fix 默认关，零开销。
 *
 * 【2026-10-30 并发竞态修复】这份缓冲是无同步的全局单例，而 App 的解码
 * 是多队列并发的（x3f.detail 细节裁剪 / 预览 / 导出池同时各解各的）。
 * 旧实现「尺寸变了就 free+malloc+memcpy」在 fix 开启时会被两条不同尺寸
 * 的解码交错打穿：A 线程 free 掉 B 线程正在 memcpy 写入的缓冲
 * （MALLOC_LARGE 释放即整块 munmap）→ SIGSEGV in memmove，实测
 * X3FExtractor-2026-10-30-163448.ips 崩在 load_common_inner 的快照
 * memcpy（x3f.detail 队列 + requestDetailCrop）；fix 关闭分支的
 * free+置NULL 也有 TOCTOU 双重 free。修法 = 快照的全部触碰点加互斥锁：
 * 写入侧（free/malloc/memcpy）与读取侧（门控统计扫描）都持锁。语义
 * 完全不变（仍是全局「最新者胜」、尺寸匹配才用快照），单线程路径锁
 * 无竞争。锁的移植写法与下方 apply_mtx_t（apply_cache_on 前）同款。 */
#if defined(_WIN32)
typedef volatile LONG prefix_mtx_t;
static void prefix_mtx_lock(prefix_mtx_t *m) {
    while (InterlockedCompareExchange(m, 1, 0) != 0) Sleep(0);
}
static void prefix_mtx_unlock(prefix_mtx_t *m) { InterlockedExchange(m, 0); }
#define PREFIX_MTX_INIT 0
#else
#include <pthread.h>
typedef pthread_mutex_t prefix_mtx_t;
static void prefix_mtx_lock(prefix_mtx_t *m) { pthread_mutex_lock(m); }
static void prefix_mtx_unlock(prefix_mtx_t *m) { pthread_mutex_unlock(m); }
#define PREFIX_MTX_INIT PTHREAD_MUTEX_INITIALIZER
#endif

static float *g_fill_prefix_lin = NULL;
static int    g_fill_prefix_w = 0, g_fill_prefix_h = 0;
static prefix_mtx_t g_fill_prefix_mtx = PREFIX_MTX_INIT;

/* pre-fix lin 选择器：帧级判定统计处使用（尺寸须匹配当前缓冲），
 * 不匹配时原样返回。 */
static const float *prefix_lin_input(const float *cur, int w, int h)
{
    if (g_fill_prefix_lin != NULL && g_fill_prefix_w == w && g_fill_prefix_h == h)
        return g_fill_prefix_lin;
    return cur;
}

/* 真正干活的实现：path 必须是独立 X3F */
static int load_common_inner(const char *path, int down,
                             int crop_x, int crop_y, int crop_w, int crop_h,
                             unsigned char nlm, const char *wb,
                             const x3f_denoise_t *dn,
                             float **rgb_out, float **clip_out,
                             float **rawmax_out, int *rmw_out, int *rmh_out,
                             int *w_out, int *h_out, char *wb_out)
{
    /* 解码分发计时桩（线程局部 g_dispatch_ms，见上面的访问器）。 */
    float *src = NULL;
    unsigned sw = 0, sh = 0, cols, rows, x0 = 0, y0 = 0;
    unsigned short orient = 1;
    double cam_to_rgb[9], rgb_to_cam[9];
    int ow, oh, ch;
    int rmw = 0, rmh = 0;
    float *rgb, *clip = NULL, *rm = NULL;
    double *rm_sum = NULL;
    unsigned *rm_cnt = NULL;
    unsigned oy, ox;

    if (path == NULL || rgb_out == NULL || w_out == NULL || h_out == NULL)
        return X3FRENDER_ERR_PARAM;
    if (down < 1) down = 1;

    /* 先把这张照片的降噪标定取出来（CAMF 段，只有解码层能解压）。
     * 放在这里是因为 apply 那一步拿不到文件路径；参数改动不重解 RAW，
     * 所以这份标定在整个「加载 → 拖滑块 → 导出」期间都有效。
     *
     * **必须在解码之前**：解码内那一步 NLM 要吃 `luminance = AUTO` 的判定，
     * 而那个判定按拍摄 ISO 走 —— ISO 就在这份标定的第一行里（见 nr_load_model）。 */
    nr_load_model(path);
    /* SDQ 高光归一：本文件的 DigitalISOGain（滑杆 >0 时才算） */
    sdq_load_dig(path);

    /* 亮度通道 AUTO → 按拍摄 ISO 定下来，只替换这一个字段。
     *
     * **dn_auto 必须活到 x3f_scene_linear 那次调用之后**：dn 会指向它，而那个调用
     * 在这个块**外面**。原来把它声明在块里 = 指针悬垂（UB）—— 今天编译器一改栈布局
     * 就现形：只要 luminance = AUTO，解码必返回 -4（NULL 或显式 0 不走这条路，
     * 所以看着只有"自动档"坏）。诊断工具：outputs/tools/lab/loadchk。 */
    x3f_denoise_t dn_auto;
    {
        if (dn != NULL && dn->luminance == X3F_DN_LUMA_AUTO) {
            dn_auto = *dn;
            dn_auto.luminance = nr_auto_luma_for_iso(g_shooting_iso);
            /* 过线不等于踩满：按 ISO 渐进（见 nr_auto_luma_ramp）。ramp 太小
             * （刚过门槛）就直接判成关 —— 省掉一整趟亮度 NLM，效果看不出来
             * 却贵 3 倍；同时避开 luma_scale 的合法下限 0.1。 */
            if (dn_auto.luminance) {
                double ramp = nr_auto_luma_ramp(g_shooting_iso, nr_auto_luma_iso());
                if (ramp < 0.05) {
                    dn_auto.luminance = 0;
                } else {
                    dn_auto.luma_scale *= ramp;
                    if (dn_auto.luma_scale < 0.1) dn_auto.luma_scale = 0.1;
                }
            }
            dn = &dn_auto;
            if (getenv("X3F_NR_DEBUG"))
                fprintf(stderr, "[nr] 亮度通道 auto → %s"
                                "（拍摄 ISO %.0f，门槛 %.0f，力度 ×%.2f）\n",
                        dn_auto.luminance ? "开" : "关",
                        g_shooting_iso, nr_auto_luma_iso(), dn_auto.luma_scale);
        }
        /* 记下"解码那一步到底有没有降亮度" —— 渲染链那根辉度滑块要靠它决定
         * 出多大力（见 nr_luma_stack_cap）。nlm 是强度，0 = 解码步整个跳过；
         * dn == NULL 时用的是 x3f_denoise_default()，那份是"只压色度"。 */
        g_nlm_luma_on = (nlm != 0 && dn != NULL && dn->luminance != 0) ? 1 : 0;
    }

    /* mode7 要逐通道裁剪标记：分发前先挂请求（own 解码时生成） */
    if (oe_neut_mode() == 7) own_request_clip_mask();
    {
        double d0 = render_now_ms();
        int drc = x3f_own_dispatch_scene_linear(path, &src, &sw, &sh, &orient,
                                                cam_to_rgb, scene_hlrecover(),
                                                nlm, dn, wb);
        g_dispatch_ms = render_now_ms() - d0;
        if (drc != 0 || src == NULL)
            return X3FRENDER_ERR_DECODE;
    }
    if (sw == 0 || sh == 0) { x3f_own_dispatch_scene_linear_free(src); return X3FRENDER_ERR_DECODE; }

    /* scene-linear = camera_to_rgb · 逐通道饱和比，所以求逆就能把饱和比反解出来。
     * 这正是旧路径 clip_score() 用的那个量（1.0 = 顶到传感器上限），
     * 比在 sRGB 上猜"超过 1.0"当代理准得多。 */
    x3f_3x3_inverse(cam_to_rgb, rgb_to_cam);

    /* ---- 冷调校正（2026-11-05）：把肤色从"红黄"拉回"冷白" ----
     *
     * 症状（用户）：内置 JPEG 人脸是冷白皮，我们的"有点黄有点红"。
     * 逐像素 Lab 对质（对齐 DY,DX,B = -2,4,20，sRGB 出口）：
     *
     *   样本        我们 a*   相机 a*   Δa      我们 b*  相机 b*
     *   SDIM0173     +7.43    +5.94   +1.49    +13.37   +15.07
     *   SDIM0092     +1.60    -0.12   +1.72    +13.74   +11.23
     *   DP1M1896       —        —     +2.79      —        —
     *   DP2M2115       —        —     +3.60      —        —
     *   DP2Q2501       —        —     +0.93      —        —
     *
     * **Δa 全为正，且与机型无关**（SD14 / Merrill / Quattro 同向）→ 不是
     * 肤色局部处理，是我们整条链的 R 偏高。全图中性像素也印证：
     * 平均 R/G 我们 1.000 vs 相机 0.989（SDIM0092，29 万像素）；B/G 两边
     * 都是 0.984 —— **只有 R 高，B 是对的**。
     *
     * 因此在线性域给 R 一个乘子（等价把色温略降），是最贴近白平衡本质的
     * 位置：后续所有曲线/矩阵看到的都是修正后的场景线性值，不会像在
     * 显示域乘那样随 gamma 变味。
     *
     * k 的标定：对 5 张样张在整图上网格搜「让全图 a* 与 b* 的均值最接近
     * 机内 JPEG」的解，得 0.950 / 0.975 / 0.960 / 0.958 / 0.987，取 0.96。
     * SDIM0173 肤色 a* 7.43 → 6.44（相机 5.94）、ΔE 2.58 → 2.40；
     * 24 张 SDIM 全图 Δa 中位 +0.37 → −0.67，24/24 张同向改善；
     * 目检过 0.94 / 0.96 / 0.98 / 1.00 四档：0.96 的肤色最接近相机，
     * 0.94 起嘴唇开始失血、背景绿叶发青。
     *
     * 关闭：X3F_COOL_R=0 或 =1.0；标定：X3F_COOL_R=<k>（0.5..1.5）。 */
    {
        /* classic 原生链不吃这条全局冷调：它是后来按 24 张样张整图 a* 拟合
         * 的产品观感修正，不是 x3fuse 原生白平衡。Quattro / Merrill 保持 0.96。
         * 显式 X3F_COOL_R 仍可覆盖（标定/对拍用）。 */
        double k = g_classic_native ? 1.0 : 0.96;
        const char *e = getenv("X3F_COOL_R");
        if (e != NULL && *e != '\0') {
            double v = atof(e);
            if (v > 0.5 && v < 1.5) k = v;
        }
        if (k != 1.0) {
            long n = (long)sw * (long)sh;
            long i;
            for (i = 0; i < n; i++) src[i * 3 + 0] = (float)(src[i * 3 + 0] * k);
        }
    }

    /* 结构化默认色彩变换（2026-10-26）：SPP Stage2 色彩步在我方 lin 域
     * 等价于逐通道增益 diag(g_color)，即 M_struct = diag(g_color/k0)
     * （k0=6.99e-5 跨解码器常数，ISO 因子与 v=k_iso·conv·in 的 k_iso 相消；
     * 推导与验证见 lab/fit_struct.py + test_transfer.py，6563/1884 双片）。
     * g_color 逐场景 3 标量（2501 rel=[1.062,0.998,1.0]、6563 rel≈[0.96,1.05,1.0]），
     * 元数据来源（疑似 HLN 高光归一化）尚未——先由 env 标定提供。
     * 默认 no-op；X3F_OE_STRUCT_GAIN="r,g,b" 启用。只在默认路径（om==0）生效：
     * mode7/8/9 的表已含该因子，重复施加会双重修正。 */
    if (oe_neut_mode() == 0) {
        const char *egsg = getenv("X3F_OE_STRUCT_GAIN");
        if (egsg != NULL && *egsg != '\0') {
            double sg[3] = { 0.0, 0.0, 0.0 };
            if (sscanf(egsg, "%lf,%lf,%lf", &sg[0], &sg[1], &sg[2]) == 3) {
                long sn = (long)sw * (long)sh;
                for (long i = 0; i < sn; i++) {
                    src[i * 3 + 0] = (float)(src[i * 3 + 0] * sg[0]);
                    src[i * 3 + 1] = (float)(src[i * 3 + 1] * sg[1]);
                    src[i * 3 + 2] = (float)(src[i * 3 + 2] * sg[2]);
                }
                if (getenv("X3F_OE_NEUT_DEBUG"))
                    fprintf(stderr, "[oe] struct gain = %.4f %.4f %.4f\n",
                            sg[0], sg[1], sg[2]);
            }
        }
    }


    /* 实验：SPP Stage2 的高光中性化（§19）。默认不跑。
     * mode 8/9 不在这里做 —— 都是 apply 级的全局点变换直通（见 apply_ex_common；
     * mode9 还要在 apply 里对**未中和**的原始 lin 做 p 层高光重建）。 */
    {
        int om = oe_neut_mode();
        /* mode 4 = 产品「关」档，但它跑的是 SPP Stage2 高光中性化 —— 那是
         * Quattro 引擎的步骤（kernel 在 Stage2、判据来自 own 解码的裁剪
         * mask）。非 Quattro 机型（SD14/SD1 走 fuse、Merrill）没有这一段，
         * 硬套会把正常曝光的高光当裁剪区抹成中性白，人脸高光因此失色。
         * 这里按机型把 mode 4 收成真「关」；显式的实验/产品档（1…3、5…9）
         * 不受影响。 */
        if (om == 4 && !g_oe_quattro_gen) om = 0;
        if (om != 0 && om != 8 && om != 9) {
            double lo, hi;
            if (om == 7) {
                /* mode7：lo 同 HighlightBlendingLow；hi 放宽到 2.2 做宽渐变
                 * （v1 的 1.5 让 t 提前饱和，白核与橙层之间出硬边） */
                lo = 0.75; hi = 2.2;
            } else if (om >= 2) {
                lo = (8000.0 / 16384.0);
                hi = (11000.0 / 16384.0);
            } else {
                lo = (8000.0 / 16384.0) * X3F_OE_LIN_WHITE;
                hi = (11000.0 / 16384.0) * X3F_OE_LIN_WHITE;
            }
            const char *el = getenv("X3F_OE_NEUT_LO");
            const char *eh = getenv("X3F_OE_NEUT_HI");
            long n;
            if (el != NULL && *el != '\0') lo = atof(el);
            if (eh != NULL && *eh != '\0') hi = atof(eh);
            if (om == 7) {
                int mw = 0, mh = 0;
                uint8_t *cmask = own_take_clip_mask(&mw, &mh);
                if (cmask == NULL && own_dispatch_used_fuse()) {
                    /* fuse 解码不产 raw 级裁剪标记：从 scene-linear 反解
                     * 饱和比合成，让 mode7 走同一个裁剪感知去饱和，
                     * 口径对齐 own（X3F_OE_FUSE_CLIP=0 退回 mode4 口径） */
                    const char *ec = getenv("X3F_OE_FUSE_CLIP");
                    if (ec == NULL || ec[0] != '0') {
                        cmask = fuse_clip_mask_from_lin(src, sw, sh, rgb_to_cam);
                        if (cmask) { mw = (int)sw; mh = (int)sh; }
                        if (getenv("X3F_OE_NEUT_DEBUG"))
                            fprintf(stderr, "[oe] mode7 fuse：合成裁剪标记\n");
                    }
                }
                if (cmask && mw == (int)sw && mh == (int)sh) {
                    n = apply_oe_restore(src, cmask, sw, sh, lo, hi);
                    free(cmask);
                } else {
                    /* 没拿到标记（x3fuse 回落 / 非 Quattro / 尺寸不符）：
                     * 退回 mode4 口径的 WB 中性化，宁可抹白也不要品红环 */
                    free(cmask);
                    double gain[3] = {1.0, 1.0, 1.0};
                    double lo4 = (8000.0 / 16384.0), hi4 = (11000.0 / 16384.0);
                    const char *e2;
                    if ((e2 = getenv("X3F_OE_NEUT_LO")) && *e2) lo4 = atof(e2);
                    if ((e2 = getenv("X3F_OE_NEUT_HI")) && *e2) hi4 = atof(e2);
                    if (read_as_shot_gain(path, gain))
                        n = apply_oe_neutralize_wb(src, sw, sh, rgb_to_cam,
                                                   cam_to_rgb, gain, lo4, hi4);
                    else
                        n = apply_oe_neutralize(src, sw, sh, rgb_to_cam,
                                                cam_to_rgb, 1, lo4, hi4);
                    if (getenv("X3F_OE_NEUT_DEBUG"))
                        fprintf(stderr, "[oe] mode7 无标记，退回 mode4 口径\n");
                }
            } else if (om >= 3) {
                double gain[3] = {1.0, 1.0, 1.0};
                if (!read_as_shot_gain(path, gain)) {
                    if (getenv("X3F_OE_NEUT_DEBUG"))
                        fprintf(stderr, "[oe] 读不到 as-shot WB，退回 mode 2\n");
                    n = apply_oe_neutralize(src, sw, sh, rgb_to_cam, cam_to_rgb,
                                            1, lo, hi);
                } else {
                    const char *eg = getenv("X3F_OE_NEUT_GAIN");
                    if (eg != NULL && *eg != '\0')
                        sscanf(eg, "%lf,%lf,%lf", &gain[0], &gain[1], &gain[2]);
                    if (om == 6) {
                        n = apply_oe_neutralize_fixed(src, sw, sh, rgb_to_cam,
                                                      gain, lo, hi);
                    } else if (om == 5) {
                        double ccm[9];
                        unsigned dims[3] = { 0, 0, 0 };
                        int got = x3f_camf_matrix_f64(path, "AutoCCMatrix",
                                                      ccm, 9, dims);
                        if (got == 9 && dims[0] == 3 && dims[1] == 3) {
                            /* 仅 sigma_dp0_quattro_25 单张定标；DP2Q2501 上
                             * 0.25 会过暖，说明它不是通用常数。 */
                            double mix = 0.25;
                            const char *em = getenv("X3F_OE_CCM_MIX");
                            if (em != NULL && *em != '\0') mix = atof(em);
                            if (mix < 0.0) mix = 0.0;
                            if (mix > 1.0) mix = 1.0;
                            n = apply_oe_neutralize_ccm(src, sw, sh, rgb_to_cam,
                                                       cam_to_rgb, ccm, gain,
                                                       lo, hi, mix);
                        } else {
                            if (getenv("X3F_OE_NEUT_DEBUG"))
                                fprintf(stderr, "[oe] 读不到 AutoCCMatrix（%d），退回 cam_to_rgb\n", got);
                            n = apply_oe_neutralize_wb(src, sw, sh, rgb_to_cam,
                                                       cam_to_rgb, gain, lo, hi);
                        }
                    } else {
                        n = apply_oe_neutralize_wb(src, sw, sh, rgb_to_cam, cam_to_rgb,
                                                   gain, lo, hi);
                    }
                    if (getenv("X3F_OE_NEUT_DEBUG"))
                        fprintf(stderr, "[oe] WB gain = %.5f %.5f %.5f\n",
                                gain[0], gain[1], gain[2]);
                }
            } else {
                n = apply_oe_neutralize(src, sw, sh, rgb_to_cam, cam_to_rgb,
                                        om == 2, lo, hi);
            }
            if (getenv("X3F_OE_NEUT_DEBUG")) {
                fprintf(stderr,
                        "[oe] cam_to_rgb = [%g %g %g; %g %g %g; %g %g %g]\n",
                        cam_to_rgb[0], cam_to_rgb[1], cam_to_rgb[2],
                        cam_to_rgb[3], cam_to_rgb[4], cam_to_rgb[5],
                        cam_to_rgb[6], cam_to_rgb[7], cam_to_rgb[8]);
                fprintf(stderr, "[oe] mode=%d lo=%.5f hi=%.5f touched=%ld (%.3f%%)\n",
                        om, lo, hi, n,
                        100.0 * (double)n / ((double)sw * (double)sh));
            }
        }
    }

    /*「过曝矫正」（勾选框 gate 的那一步）：默认关；
     * 网格是 Quattro 的专用像素布局，Merrill 不动（与 SPP 手册的机型限制一致）。 */
    if (x3f_render_hlp_fix_enabled() && !x3f_render_probe_is_merrill(path)) {
        const char *dbg = getenv("X3F_HLP_DEBUG");
        int dry = getenv("X3F_HLP_DRYRUN") != NULL;
        long n = apply_hlp_fix(src, sw, sh, rgb_to_cam, dry);
        if (dbg) fprintf(stderr, "[hlp] %s: marked=%ld (%.4f%%)\n",
                         dry ? "dryrun" : "applied", n,
                         100.0 * (double)n / ((double)sw * (double)sh));
    }

    /* HLP 高光混合（实验）：默认关。X3F_OE_HLP=1 走内部 guide。
     * （dylib / 的 OECVer3 分支已废除，仅保留内部 guide 混合。）*/
    if (oe_hlp_mode() != 0) {
        int hm = oe_hlp_mode();
        const char *dbg = getenv("X3F_OE_HLP_DEBUG");
        long n = apply_oe_hlp_blend(src, sw, sh);
        if (dbg) fprintf(stderr, "[oehlp] mode=%d touched=%ld (%.4f%%) guide=%s\n",
                         hm, n, 100.0 * (double)n / ((double)sw * (double)sh),
                         getenv("X3F_HLP_GUIDE") ? getenv("X3F_HLP_GUIDE")
                                                  : "internal");
    }

    /* SPP Stage2 MAC（MultiAxisCC）：NR 后、CCM 前的相机域逐像素色彩修正。
     * 实现独立在 x3f_mac.c；gate 不过 = 逐位 no-op（0337 类无池稿件不受扰动）。 */
    x3f_mac_stage(path, src, sw, sh, rgb_to_cam, cam_to_rgb);

    cols = sw; rows = sh;
    if (crop_w > 0 && crop_h > 0) {
        if (crop_x < 0) crop_x = 0;
        if (crop_y < 0) crop_y = 0;
        x0 = (unsigned)crop_x; y0 = (unsigned)crop_y;
        if (x0 >= cols) x0 = cols - 1;
        if (y0 >= rows) y0 = rows - 1;
        cols = (unsigned)crop_w; rows = (unsigned)crop_h;
        if (x0 + cols > sw) cols = sw - x0;
        if (y0 + rows > sh) rows = sh - y0;
    }
    ow = (int)((cols + (unsigned)down - 1) / (unsigned)down);
    oh = (int)((rows + (unsigned)down - 1) / (unsigned)down);

    rgb = (float *)malloc((size_t)ow * oh * 3 * sizeof(float));
    if (rgb == NULL) { x3f_own_dispatch_scene_linear_free(src); return X3FRENDER_ERR_NOMEM; }
    if (clip_out != NULL) {
        clip = (float *)calloc((size_t)ow * oh, sizeof(float));
        if (clip == NULL) {
            free(rgb); x3f_own_dispatch_scene_linear_free(src);
            return X3FRENDER_ERR_NOMEM;
        }
    }

    /* RAW 域亮度图（给"用 RAW 反推高光"用）：每 X3F_RAWMAX_BLOCK 一块，
     * 取块内「逐像素三通道最大值」的均值。旧路径用的是预处理后的 u16，
     * 这里用反解出来的逐通道饱和比 —— 同一个物理量，只是单位不同；
     * 消费端（RawHighlights.fromRawMax）会按 99.9 分位自己归一化，
     * 所以只有相对亮度要紧。 */
    if (rawmax_out != NULL) {
        if (down != 1) {
            free(rgb); free(clip); x3f_own_dispatch_scene_linear_free(src);
            return X3FRENDER_ERR_PARAM;
        }
        rmw = (ow + X3F_RAWMAX_BLOCK - 1) / X3F_RAWMAX_BLOCK;
        rmh = (oh + X3F_RAWMAX_BLOCK - 1) / X3F_RAWMAX_BLOCK;
        rm = (float *)calloc((size_t)rmw * rmh, sizeof(float));
        rm_sum = (double *)calloc((size_t)rmw * rmh, sizeof(double));
        rm_cnt = (unsigned *)calloc((size_t)rmw * rmh, sizeof(unsigned));
        if (rm == NULL || rm_sum == NULL || rm_cnt == NULL) {
            free(rgb); free(clip); free(rm); free(rm_sum); free(rm_cnt);
            x3f_own_dispatch_scene_linear_free(src);
            return X3FRENDER_ERR_NOMEM;
        }
        *rawmax_out = NULL;
    }

    for (oy = 0; oy < (unsigned)oh; oy++) {
        for (ox = 0; ox < (unsigned)ow; ox++) {
            double acc[3] = {0.0, 0.0, 0.0};
            int n = 0, dy, dx;
            for (dy = 0; dy < down; dy++) {
                unsigned sy = y0 + oy * (unsigned)down + (unsigned)dy;
                if (sy >= y0 + rows) break;
                for (dx = 0; dx < down; dx++) {
                    const float *p;
                    unsigned sx = x0 + ox * (unsigned)down + (unsigned)dx;
                    if (sx >= x0 + cols) break;
                    p = src + ((size_t)sy * sw + sx) * 3;
                    for (ch = 0; ch < 3; ch++) acc[ch] += p[ch];
                    n++;
                }
            }
            if (n == 0) n = 1;
            for (ch = 0; ch < 3; ch++)
                rgb[((size_t)oy * ow + ox) * 3 + ch] = (float)(acc[ch] / n);
            if (clip != NULL || rm != NULL) {
                double lin[3], sat[3];
                int clipped = 0;
                for (ch = 0; ch < 3; ch++) lin[ch] = acc[ch] / n;
                /* scene-linear = camera_to_rgb · 逐通道饱和比 ⇒ 求逆反解 */
                x3f_3x3_3x1_mul(rgb_to_cam, lin, sat);
                /* 试过在**传感器域**收色（矩阵之前），结果更差：这个矩阵把
                 * "传感器中性"(1,1,1) 映射成 (5.31, −0.93, 0.20)，也就是
                 * 矩阵之前的"中性"并不等于"白"，在那边收色出来照样是黄的。
                 * 所以高光这一步只能留在显示域（见下面 hl_desat 那段）。 */
                (void)clipped;
                if (clip != NULL) {
                    /* SDQ 高光归一（见 g_sdq_hl 的长注释）：把"相对本机通道
                     * 白点"的饱和比换算回统一白电平。dig=(1,1,1) 的文件乘
                     * 1.0 精确无变化；滑杆为 0 时走原路径。 */
                    if (sdq_dig_active()) {
                        double sc[3];
                        double cs, cmn, cmx;
                        for (ch = 0; ch < 3; ch++)
                            sc[ch] = sat[ch] * g_sdq_dig[ch];
                        cs = clip_score(sc);
                        /* ---- 补收：RAW 中性、显示偏色的「暗白」 ------------
                         * 上面那套只认「顶到白点」（0.90 起算），漏掉一类：
                         * SDQH0184 水面成片反光在 RAW 里归一后是 (0.577,
                         * 0.650, 0.584) —— **三通道几乎相等，本来就是中性
                         * 白**，只是亮度才到白点的 58%；tone 曲线把它推到
                         * 显示 0.85，矩阵又把 B 压到 0.82 → 成片淡黄。
                         * 相机 JPEG 同一片是纯白，且 RAW 饱和比才 0.5 出头，
                         * 所以 clip_score 恒为 0，拉到 1 也一点不动。
                         *
                         * 判据用「RAW 域的通道平坦度」而不是亮度：真正偏暖的
                         * 物体（水面橙色反光、河床）在 RAW 上就偏红，比值
                         * 1.5~1.8；这类伪色比值只有 1.1~1.2。实测同帧：
                         *   伪色 (3363,890) ratio 1.13、真橙 (3631,1747) 1.75。
                         * 再乘一个亮度渐入，暗部噪声（ratio 会乱跳）不参与。
                         * 只有显示域真有色偏才会被改（中性像素往 Y 拉不变）。 */
                        cmn = cmx = sc[0];
                        for (ch = 1; ch < 3; ch++) {
                            if (sc[ch] < cmn) cmn = sc[ch];
                            if (sc[ch] > cmx) cmx = sc[ch];
                        }
                        {
                            const char *e = getenv("X3F_SDQ_NEUT");
                            int neut_on = (e == NULL) ? 1 : (atoi(e) != 0);
                            if (neut_on && cmx > 1e-6) {
                                double ratio = cmx / (cmn > 1e-6 ? cmn : 1e-6);
                                double flat = 1.0 - sstep01(1.12, 1.40, ratio);
                                double lvl = sstep01(0.18, 0.45, cmx);
                                double cs2 = 3.0 * flat * lvl;
                                if (cs2 > cs) cs = cs2;
                            }
                        }
                        clip[(size_t)oy * ow + ox] = (float)cs;
                    } else {
                        clip[(size_t)oy * ow + ox] = (float)clip_score(sat);
                    }
                }
                if (rm != NULL) {
                    double m = sat[0];
                    size_t bi;
                    if (sat[1] > m) m = sat[1];
                    if (sat[2] > m) m = sat[2];
                    bi = (size_t)(oy / X3F_RAWMAX_BLOCK) * rmw
                       + (size_t)(ox / X3F_RAWMAX_BLOCK);
                    rm_sum[bi] += m;
                    rm_cnt[bi] += 1;
                }
            }
        }
    }
    x3f_own_dispatch_scene_linear_free(src);

    /* v6 裁剪孔洞修正（X3F_HOLE_FIX=1 开启，默认关；配帧级门控）。
     * 在 lin 域、播出前施加：G/B 按裁剪深度加增益，R 锚定不动。
     * 施加前快照 pre-fix lin（g_fill_prefix_lin），供帧级判定统计用——
     * 见该变量处的注释。快照随每次解码刷新，与当前 lin 缓存严格同步。
     * ★ 快照不止 fix 需要（2026-10-30 云边缘粉破案）：mode8/9 的帧门控
     *   统计也必须吃这份快照。fix 关闭时原本不生成快照 → prefix_lin_input
     *   回落到 apply 链当前 lin，而那个域被 apply 前置增益（实测 ≈×1.11）
     *   抬过 → 2443 阈 1.55 统计 1.81% 误放行、黄墙断裂回归。改为 fix 开
     *   **或** oe8/9 激活时都快照；两者都关时维持 NULL（零开销）。 */
    if (x3f_hole_fix_enabled() || oe_neut_mode() == 8 || oe_neut_mode() == 9) {
        const size_t np3 = (size_t)ow * oh * 3;
        prefix_mtx_lock(&g_fill_prefix_mtx);
        if (g_fill_prefix_w != ow || g_fill_prefix_h != oh) {
            free(g_fill_prefix_lin);
            g_fill_prefix_lin = (float *)malloc(np3 * sizeof(float));
            g_fill_prefix_w = ow;
            g_fill_prefix_h = oh;
        }
        if (g_fill_prefix_lin != NULL) {
            memcpy(g_fill_prefix_lin, rgb, np3 * sizeof(float));
            /* 临时诊断：dump 快照本体（2026-10-30 门控域对质用） */
            const char *pd = getenv("X3F_PREFIX_DUMP");
            if (pd != NULL) {
                FILE *pf = fopen(pd, "wb");
                if (pf != NULL) {
                    fwrite(g_fill_prefix_lin, sizeof(float), np3, pf);
                    fclose(pf);
                    fprintf(stderr, "[prefix] dump %dx%d -> %s\n", ow, oh, pd);
                }
            }
        }
        prefix_mtx_unlock(&g_fill_prefix_mtx);
        if (x3f_hole_fix_enabled())
            x3f_hole_fix_apply(rgb, ow, oh);
    } else {
        prefix_mtx_lock(&g_fill_prefix_mtx);
        free(g_fill_prefix_lin);
        g_fill_prefix_lin = NULL;
        g_fill_prefix_w = g_fill_prefix_h = 0;
        prefix_mtx_unlock(&g_fill_prefix_mtx);
    }

    if (rm != NULL) {
        size_t i, total = (size_t)rmw * rmh;
        for (i = 0; i < total; i++)
            rm[i] = rm_cnt[i] ? (float)(rm_sum[i] / rm_cnt[i]) : 0.0f;
        free(rm_sum);
        free(rm_cnt);
        *rawmax_out = rm;
        if (rmw_out) *rmw_out = rmw;
        if (rmh_out) *rmh_out = rmh;
    }

    /* 保色高光恢复（实验，已验证失败）：16×16 块色比会把背景的中性色
     * 串进火焰核心（出现蓝/白块、绿边），默认不启用，仅供后续对比。
     * 只认 X3F_HL_CHROMA=2。 */
    {
        const char *e = getenv("X3F_HL_CHROMA");
        if (e != NULL && atoi(e) >= 2) {
            const char *se = getenv("X3F_HL_CHROMA_STR");
            double s = (se != NULL && *se != '\0') ? atof(se) : 0.6;
            hl_chroma_recover(rgb, clip, ow, oh, s);
        }
    }
    {
        const char *e = getenv("X3F_HL_VORONOI");
        if (e != NULL && *e != '\0' && *e != '0') {
            const char *se = getenv("X3F_HL_VORONOI_STR");
            double s = (se != NULL && *se != '\0') ? atof(se) : 0.85;
            hl_chroma_voronoi(rgb, clip, ow, oh, s);
        }
    }
    *rgb_out = rgb;
    if (clip_out) *clip_out = clip;
    *w_out = ow;
    *h_out = oh;
    if (wb_out) snprintf(wb_out, 64, "dng");
    return X3FRENDER_OK;
}


int x3f_render_load(const char *path, float **rgb_out, int *w_out, int *h_out,
                    char *wb_out)
{
    return load_common(path, 1, 0, 0, 0, 0, nlm_from_env(), NULL, NULL,
                       rgb_out, NULL, NULL, NULL, NULL,
                       w_out, h_out, wb_out);
}

int x3f_render_load_ex(const char *path, int down,
                       float **rgb_out, int *w_out, int *h_out, char *wb_out)
{
    return load_common(path, down, 0, 0, 0, 0, nlm_from_env(), NULL, NULL,
                       rgb_out, NULL, NULL, NULL, NULL,
                       w_out, h_out, wb_out);
}

int x3f_render_load_ex2(const char *path, int down,
                        float **rgb_out, float **clip_out,
                        int *w_out, int *h_out, char *wb_out)
{
    if (clip_out) *clip_out = NULL;
    return load_common(path, down, 0, 0, 0, 0, nlm_from_env(), NULL, NULL,
                       rgb_out, clip_out, NULL, NULL, NULL,
                       w_out, h_out, wb_out);
}

int x3f_render_load_ex3(const char *path, int down, int nlm,
                        const x3f_denoise_t *dn,
                        float **rgb_out, float **clip_out,
                        float **rawmax_out, int *rmw_out, int *rmh_out,
                        int *w_out, int *h_out, char *wb_out)
{
    if (clip_out) *clip_out = NULL;
    if (rawmax_out) *rawmax_out = NULL;
    return load_common(path, down, 0, 0, 0, 0, nlm_clamp(nlm), NULL, dn,
                       rgb_out, clip_out,
                       rawmax_out, rmw_out, rmh_out, w_out, h_out, wb_out);
}

/* 带"相机白平衡预设"的那条入口（预览和导出都用它）。
 *
 * wb = 预设名（"Sunlight" / "Shade" / "Overcast" / "Incandescent" /
 * "Fluorescent" / "Flash" / "Auto" / "Custom"），NULL = 用文件里记录的那个。
 * 名字同时决定用哪一份**每光源色彩校正矩阵**，所以换 WB 是整条色彩标定跟着换，
 * 不是三个通道乘个系数 —— 这也是它为什么必须走解码那一步、只能在这里传。
 *
 * nlm / dn = 导出用的降噪（0..10 与参数表，见 x3f_render_load_ex3 的说明）；
 *            dn 可传 NULL（= 默认那套）。
 * rawmax 非空时行为同 ex3（只在 down==1 时给）。 */
int x3f_render_load_wb(const char *path, int down, const char *wb, int nlm,
                       const x3f_denoise_t *dn,
                       float **rgb_out, float **clip_out,
                       float **rawmax_out, int *rmw_out, int *rmh_out,
                       int *w_out, int *h_out, char *wb_out)
{
    if (clip_out) *clip_out = NULL;
    if (rawmax_out) *rawmax_out = NULL;
    return load_common(path, down, 0, 0, 0, 0, nlm_clamp(nlm), wb, dn,
                       rgb_out, clip_out,
                       rawmax_out, rmw_out, rmh_out, w_out, h_out, wb_out);
}

int x3f_render_load_crop(const char *path, int x0, int y0, int cw, int ch,
                         float **rgb_out, float **clip_out,
                         int *w_out, int *h_out, char *wb_out)
{
    if (clip_out) *clip_out = NULL;
    if (cw <= 0 || ch <= 0) return X3FRENDER_ERR_PARAM;
    return load_common(path, 1, x0, y0, cw, ch, nlm_from_env(), NULL, NULL,
                       rgb_out, clip_out, NULL, NULL, NULL,
                       w_out, h_out, wb_out);
}

/* 正立矩形 → 传感器矩形（EXIF 方向）。
 *
 * 推导（与 RawRender.applyOrientation 的 CGContext 变换互为逆运算）：
 * CGContext 原点在左下、y 向上，源像素 (sx,sy)【左上角原点、行优先】在用户空间是
 * (sx+0.5, H-sy-0.5)；过 CTM 之后落到设备空间，再换回"左上角原点"的行列号：
 *     1 : (u,v) = (sx, sy)
 *     3 : u = W-1-sx, v = H-1-sy
 *     6 : u = H-1-sy, v = sx          ← 6 = 顺时针 90°
 *     8 : u = sy,     v = W-1-sx      ← 8 = 逆时针 90°
 * 反过来解就得到下面四行。竖幅（6/8）时正立图的宽 = 传感器的高。
 * 这块换算是拿像素验过的：lab 的 chromanr_check rot 模式会把"裁剪后转正"
 * 和"整幅转正后裁剪"逐字节对比。 */
void x3f_render_upright_rect_to_sensor(int orientation, int sensor_w, int sensor_h,
                                       int x, int y, int w, int h,
                                       int *sx, int *sy, int *sw, int *sh)
{
    int swap = (orientation == 6 || orientation == 8);
    int uw = swap ? sensor_h : sensor_w;
    int uh = swap ? sensor_w : sensor_h;
    int u0, v0, uwid, vhei;

    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x > uw - 1) x = uw - 1;
    if (y > uh - 1) y = uh - 1;
    w = (x + w > uw) ? uw - x : w;
    h = (y + h > uh) ? uh - y : h;
    if (w < 1) w = 1;
    if (h < 1) h = 1;
    u0 = x; v0 = y; uwid = w; vhei = h;

    switch (orientation) {
    case 3:
        *sx = sensor_w - u0 - uwid; *sy = sensor_h - v0 - vhei;
        *sw = uwid; *sh = vhei;
        break;
    case 6:
        *sx = v0; *sy = sensor_h - u0 - uwid; *sw = vhei; *sh = uwid;
        break;
    case 8:
        *sx = sensor_w - v0 - vhei; *sy = u0; *sw = vhei; *sh = uwid;
        break;
    default:
        *sx = u0; *sy = v0; *sw = uwid; *sh = vhei;
        break;
    }
    if (*sx < 0) *sx = 0;
    if (*sy < 0) *sy = 0;
    if (*sx + *sw > sensor_w) *sw = sensor_w - *sx;
    if (*sy + *sh > sensor_h) *sh = sensor_h - *sy;
}

void x3f_render_free(float *rgb)
{
    free(rgb);
}

/* ---------------------------------------------------------------- */
/*「细节」= Detail 滑块：残差比值图 × 强度（2026-10 第三轮后重写）
 *
 * 全链路与逐条证据见 。要点：
 *
 *  1) 档位：X3F 字段 85/86（srResidualColor / Monochrome，tag +0x1f8 / +0x200）
 *     → IPM2+0x4a0 → Stage2B 请求 +0x52c
 *     → level = (int)(v·6 + 0.1) ∈ 0…6（7 档；能力表 step = 1/6 是旁证）
 *  2) 档位 → 残留强度 R（表 A）：0 / .1 / .2 / .3 / 1.0 / 1.25 / 1.5
 *  3) 强度 → 贴残差的增益 **g = 1 + (base − 1)·R**，base ≈ 1.4
 * （F23Stage2BMain–0xe2b04；另一条分支 base ≈ 1.8）
 * 4) 残差图（CreateLumaResidual，整段 580 字节）：
 *        φ(x) = (x/f)^(5/12)·f + 5          ; 0x3ed55555 = 5/12
 *        r = clamp( φ(一张亮度图) / φ(另一张), 0.001, 1000 )，src ≤ 0 → r = 1
 *     另一支（bool = 0）是 r = (a+5)/(b+5)，且亮部 ≥ 10992 时把 r 往 1 淡出。
 * 5) 贴回（ApplyResidual，逐像素乘性）：
 *        r > 1 : m = 1 + (r − 1)·g
 *        r ≤ 1 : m = 1 / ((1 − g) + g/r)
 *        R,G,B *= m
 *     两支在 r = 1 相接，并对 r ↔ 1/r、m ↔ 1/m 封闭（对数域对称）。
 *     **g = 1 → m ≡ r（残差原样贴上）；g = 0 → m ≡ 1（完全不贴）。**
 *
 * 我们的取舍（**与 SPP 不同，写清楚**）：
 *  * SPP 管线里残差是**一直**贴的（g ≥ 1）；我们的基线从来没有这一步，
 *    所以这里把 **level 0 定义成"整段关"**，level ≥ 1 才生成并贴残差 ——
 *    这样默认出图与以前逐字节相同。等对过 SPP 参考图再决定要不要常开。
 * * 比值的方向：里那两张图的角色我们没追到底，这里**按语义定**
 *    （r = φ(本像素) / φ(局部低通)），保证往 Crispy 推是**加细节** ——
 *    与 SPP UI 的说法一致。
 * * 局部低通用"多尺度盒式模糊"顶替 双边金字塔：SPP 那条
 *    （CreateBFLumaResidual + BilateralFilterMonoF23）还没读，属于建模部分。
 *
 * 标定开关：X3F_DETAIL_R（低通半径，默认 4）·X3F_DETAIL_OCT（尺度数，默认 2）
 * ·X3F_DETAIL_EPS（φ 里那个 +5 换算到显示域的等效值，默认 0.02·满量程）
 * ·X3F_DETAIL_BASE（base，默认 1.4）·X3F_DETAIL_LEVEL（直接点档，调试用）。 */

static const double x3f_detail_tbl_a[7] = { 0.0, 0.1, 0.2, 0.3, 1.0, 1.25, 1.5 };

/* 环境变量读数。**不**把 "0" 当"没设"（0 是有意义的值）。 */
static double detail_env(const char *name, double dflt)
{
    const char *s = getenv(name);
    return (s != NULL && s[0] != '\0') ? atof(s) : dflt;
}

/* 滑块值 → 档位 0…6（SPP 原式：(int)(v·6 + 0.1)，0x8ccc4–0x8ccec） */
static int x3f_detail_level(double v)
{
    int l;
    const char *forced = getenv("X3F_DETAIL_LEVEL");   /* 调试：直接点档 */
    if (forced != NULL && forced[0] != '\0') {
        l = atoi(forced);
    } else {
        if (v < 0.0) v = 0.0;
        if (v > 1.0) v = 1.0;
        l = (int)(v * 6.0 + 0.1);
    }
    return l < 0 ? 0 : (l > 6 ? 6 : l);
}

/* 档位 → **实际生效的**残留强度 R。
 *
 * 表 A（= SetParameterResidualSlider 里那张 5 项表）只在**高档位**（4~6）
 * 逐位对上；前三档在 SPP 里是**另一回事**，而且是 **ISO 相关**的 ——
 * 三张 16-bit 无损参考（DP0Q1000/ISO100、DP0Q1049/ISO160、DP0Q2315/ISO1600，
 * 各 7 档，`tools/lab/hfshape.py` + 线性口径）实测：
 *
 *   ISO1600：档 1~3 的输出与档 0 相比 —— 中位 |Δ| = 0、只有 0.45% 的像素动了
 *            ±1 个 16-bit 码值（= 纯舍入）⇒ **前三档是 no-op** ⇒ R(1..3) = 0
 * （ 里那张 `[0,0,0,1,1.25]` 的表确实被走到了；本文档早先
 *             "该分支走不到"的判断是错的）
 *   ISO100/160：档 1~3 是**真的在动**（平均 |Δ| = 121~562 码值），反解出的
 *            有效 R ≈ [0.24, 0.38, 0.53]（两张图互相吻合到 5%），
 *            比表 A 的 [0.1, 0.2, 0.3] 大 ~2.2×。
 *            等价说法：低 ISO 下响应对 R 是**凹**的（Δrel ∝ R^0.69），
 *            高档位（R ≥ 1）仍严格线性、且与表 A 逐位吻合。
 *
 * 分界取 ISO 800（与 §14.6 那条 arg7 阶梯的分界同源），可用
 * X3F_DETAIL_ISOHI / X3F_DETAIL_LOW1..3 覆盖。 */
static double x3f_detail_R(int level)
{
    const double low_default[3] = { 0.24, 0.38, 0.53 };
    const double iso = detail_env("X3F_DETAIL_ISO", x3f_file_iso());
    double r;
    if (level < 0) level = 0;
    if (level > 6) level = 6;
    if (level >= 4) return x3f_detail_tbl_a[level];
    if (level == 0) return 0.0;
    if (iso >= detail_env("X3F_DETAIL_ISOHI", 800.0)) return 0.0;   /* 高 ISO：前三档不做事 */
    r = detail_env(level == 1 ? "X3F_DETAIL_LOW1" :
                   level == 2 ? "X3F_DETAIL_LOW2" : "X3F_DETAIL_LOW3",
                   low_default[level - 1]);
    return r;
}

/*强度 → 增益 g。**这里是拟合值，不是值**：
 *
 * 那条是 g = 1 + (base − 1)·R（–0xe2b04，base ≈ 1.4），
 * 也就是 R = 0 时 g = 1（残差已贴）。但拿 七张参考图对**绝对**高频能量
 * （DP0Q1000，8 位码值）一看：
 *
 *   我们关掉残差 6.97 ｜ 我们贴残差(g=1) 12.57
 *   SPP 档 0      6.37 ｜ SPP 档 6        12.67
 *
 * —— **SPP 整个滑块的范围正好等于我们"不贴 → 贴"**，所以它们的档 0 在绝对值上
 * 就是"没有残差"。因此按 g = GMAX·(R/1.5)、GMAX 默认 1.1 定标：档 0 → g=0
 * （严格 no-op，默认出图不变），档 6 → g≈1.1 ≈ 档 6。
 *
 * 与那条的差别见文档 §10：两者只在"档 0 到底贴不贴"上分歧，
 * 而绝对量级支持"档 0 不贴"。 */
static double x3f_detail_gain(double R)
{
    /* 幂指数是**拟合**出来的：SPP 那七档的实测响应满足
     *
     *     (HF/HF0 − 1) = 0.741 · R^0.709        （七点全中，误差 ≤5%）
     *
     * 我们这条链的响应是**线性**于 g 的（实测 g 从 0 到 4.0，(rel−1)/g 恒定），
     * 所以 g = 系数·R^0.709 就能对上；系数在**参考图那个分辨率**（5424）上定，
     * 因为 HF 这个指标本身随分辨率变（2712 上斜率 0.72、5424 上是 0.81）。
     * 定出来 0.906。
     * 指数 ≈0.71 从哪来还没解释（那条 g = 1+(base−1)R 是线性的），
     * 但七点实测就是这样，见文档 §10.4/§10.5。 */
    const double p = detail_env("X3F_DETAIL_POW", 0.709);
    const double gs = detail_env("X3F_DETAIL_GSCALE", 1.162);   /*默认补齐相对 HF 残差缺口(≈0.861×)*/
    /*base 有**两条**，F23Stage2BMain–0xe2adc 逐行读下来是：
     *
     *   base1 = max( ((1 − ISO/100)·0.125 + 1)·0.3 + 1.1, 1.1 )
     *           ISO100 → 1.400 · 320 → 1.318 · 800 → 1.250 · ≥1600 → 1.100
     *   base2 = max( ((ISO/100 − 1)/(−15) + 1)·0.6 + 1.2, 1.2 )
     *           ISO100 → 1.800 · 160 → 1.712 · 320 → 1.652 · ≥1600 → 1.200
     *
     * 选哪条（–0xe2a88）：
     *   mode = params[0xf8]
     *     mode ≥ 2              → **base2**
     *     mode ≤ 1 且 ISO > 1600 且 p[0x101] == 0 → base = 1.0
     *     其余                   → base1
     * 另外 style 串 == "Portrait" 时 R 要除以 1.1402，
     * 走 base2 那条更狠：R ← R/2。
     *
     * 参考图（SPP 导出那七张）的 p[0xf8] = 3/4（§11.1 实证）⇒ **用的是 base2**。
     * 这一点还能被 ISO100 那份参考独立验证：Δrel = 0.9885 只有 base2（g−1 = 1.2）
     * 才自洽（c = 0.82），base1 会反解出 c = 1.65 > 1（物理上不可能）。
     * 所以我们默认 mode = 3（= base2），mode 用 X3F_DETAIL_MODE 改。 */
    {
        double iso = detail_env("X3F_DETAIL_ISO", x3f_file_iso());
        const int mode = (int)detail_env("X3F_DETAIL_MODE", 3.0);
        const int p101 = (int)detail_env("X3F_DETAIL_P101", 1.0);
        double base1, base2, base;
        if (iso > 0.0) {
            base1 = (1.0 - iso / 100.0) * 0.125;
            base1 = (base1 + 1.0) * 0.3 + 1.1;
            if (base1 < 1.1) base1 = 1.1;
            base2 = (iso / 100.0 - 1.0) / (-15.0) + 1.0;
            base2 = base2 * 0.6 + 1.2;
            if (base2 < 1.2) base2 = 1.2;
            if (mode >= 2)                    base = base2;
            else if (iso > 1600.0 && !p101)   base = 1.0;
            else                              base = base1;
        } else {
            base = 1.4;                    /* 拿不到 ISO：按 ISO 100 的 base 兜底 */
        }
        /* 系数 0.906/0.4 = 2.265：让 ISO 100（base−1 = 0.4）处与上一版定标**逐值相同**，
         * 这样 ISO 100 的文件出图不变，只有高 ISO 的会跟着 SPP 变弱。 */
        /*默认走**原式** g = 1 + (base − 1)·R（–0xe2b04）。
         * 语义：g = 1 时残差**原样**贴上（out = restored·r ≡ 原图，看不出变化），
         * g > 1 才是"放大残差"——也就是 整个滑块范围是 [原样, 原样+60%]。
         * 上一版那条 2.265·(base−1)·R^0.709 是为了补"低通不一样"的拟合，
         * 换成 保边核之后就不需要了；留着 X3F_DETAIL_GMODE=fit 做 A/B。*/
        {
            const char *gm = getenv("X3F_DETAIL_GMODE");
            if (gm == NULL || gm[0] != 'f')
                return 1.0 + (base - 1.0) * R * gs;
        }
        return detail_env("X3F_DETAIL_GMAX", 2.265) * (base - 1.0) * pow(R, p);
    }
}

/*保边低通（双边）。SPP 那条（BilateralFilterMonoF23）是
 * **表驱动 + 掩膜感知**的双边：开局先建一张 8192 项的范围权重表
 * （bzero 0x2000 后逐 16 项由常数表填，并把低于阈值的项清零），
 * 另有离群点剔除分支；逐项复刻的收益不大，这里只取它的**性质**：
 * 空间高斯 × 由 |Δ| 查表得到的 range 权重。
 * 两趟（横/竖）近似 —— 双边本身不可分，但对"低通"足够。
 * 输入必须是归一化 0..1 的亮度。 */
#define X3F_DBL_LUT 1024

static void detail_bilateral(const float *src, float *dst, float *tmp,
                             int w, int h, int r, double sigma_r)
{
    static float rl[X3F_DBL_LUT + 1];
    static int   rl_ready = 0;
    float sw[65];
    int i, x, y, d;
    const double ss = (r > 0 ? r * 0.5 : 0.5);

    if (r < 1) r = 1;
    if (r > 32) r = 32;

    if (!rl_ready) {
        for (i = 0; i <= X3F_DBL_LUT; i++) {
            const double t = (double)i / (double)X3F_DBL_LUT;
            rl[i] = (float)exp(-(t * t) / (2.0 * sigma_r * sigma_r));
        }
        rl_ready = 1;
    }
    for (i = -r; i <= r; i++)
        sw[i + r] = (float)exp(-(double)(i * i) / (2.0 * ss * ss));

    /* 横向 */
    for (y = 0; y < h; y++) {
        const float *sr = src + (size_t)y * w;
        float *dr = tmp + (size_t)y * w;
        for (x = 0; x < w; x++) {
            const float c = sr[x];
            float acc = 0.0f, wsum = 0.0f;
            for (d = -r; d <= r; d++) {
                int xx = x + d;
                float v, ww;
                int idx;
                if (xx < 0) xx = 0; else if (xx >= w) xx = w - 1;
                v = sr[xx];
                idx = (int)((double)(v - c >= 0 ? v - c : c - v) * X3F_DBL_LUT);
                if (idx > X3F_DBL_LUT) idx = X3F_DBL_LUT;
                ww = sw[d + r] * rl[idx];
                acc += ww * v; wsum += ww;
            }
            dr[x] = acc / wsum;
        }
    }
    /* 纵向 */
    for (y = 0; y < h; y++) {
        float *dr = dst + (size_t)y * w;
        for (x = 0; x < w; x++) {
            const float c = tmp[(size_t)y * w + x];
            float acc = 0.0f, wsum = 0.0f;
            for (d = -r; d <= r; d++) {
                int yy = y + d;
                float v, ww;
                int idx;
                if (yy < 0) yy = 0; else if (yy >= h) yy = h - 1;
                v = tmp[(size_t)yy * w + x];
                idx = (int)((double)(v - c >= 0 ? v - c : c - v) * X3F_DBL_LUT);
                if (idx > X3F_DBL_LUT) idx = X3F_DBL_LUT;
                ww = sw[d + r] * rl[idx];
                acc += ww * v; wsum += ww;
            }
            dr[x] = acc / wsum;
        }
    }
}

/*---- 保边低通（"restored" 图）。见文档 §13.3 / §13.4 ----------
 *
 * w(i,j) = exp(−d(i,j)) · 1 / (1 + L1 / T) (XDoCreateRestoredImg 起)
 *
 *   d  = 5×5 窗口，i,j ∈ {−2..2}（单位 = 整像素），**整体再加 ±0.25 像素的四相位偏移**
 * （相位由 (col,row) 的奇偶选；距离表 4 组 × 25 项在–0xe1c38 就地展开）
 *   L1 = |ΔR| + |ΔG| + |ΔB|（引导图的三通道绝对差之和）
 *   T  = 25/65535（max(R,G,B) ≤ 1000 码值）→ 1000/65535（≥ 10000 码值之间线性插值）
 * （–0xe1980；引导图是 16bit 码值，所以 25 是"16bit 码值的 25"，
 *          这里一律换算成满量程比例，与位深无关）
 *
 * 与 两点差别（写清楚）：
 * · 引导图（arg3）与被平均的值图（arg5）是**两张**图；我们只有一张，自引导
 *    （= 普通双边）。
 *  · 被平均的亮度取 `Y`（含锐度那一步），引导的 RGB 取还没被写过的工作缓冲 ——
 *    因为锐度在我们这里是先于细节做的（SPP 是 Stage2B 先、锐度后）。
 *
 * 阈值倍数 X3F_DETAIL_RANGE **默认 1.0 = 照抄的两个锚点**（25/65535 → 1000/65535）。
 * 实测（DP0Q2315，ISO1600，档6，见文档 §14）： Δrel = +0.1034，我们 +0.0289，
 * 即**我们的"restored"只吃掉了 19% 的高频，SPP 吃掉了 69%**。原因是 SPP 那条链里
 * 被除的 restored 是**降噪后的图**（BlendNR / 噪声模型双边，XDoCreateRestoredImg
 * 里 arg5 那张），这颗 5×5 核只是拿原图当引导去**精修**它 —— 我们少了那一步。
 * 把 RANGE 放宽到 25 能凑出幅度（+0.0952 ≈ SPP），但会破坏保边性：
 * 实测一个被 (221,162,162) 包围的孤立 (211,0,0) 像素，L1=0.63 vs 阈值 0.38
 * → 邻居权重 0.38，restored 被邻居拽走 → r=0.26 → m=0.235 → 该点被压成 (99,25,25)。
 * 所以这里**不做这个凑数**，保持保边；要补的是"restored 得是降噪图"这件事。
 * 另有 X3F_DETAIL_PHASE（0 = 关四相位，默认 1） */
#define X3F_GDF_R      2
#define X3F_GDF_N      (2 * X3F_GDF_R + 1)
#define X3F_GDF_T_LO   (1000.0 / 65535.0)
#define X3F_GDF_T_HI   (10000.0 / 65535.0)
#define X3F_GDF_RANGE_LO (25.0 / 65535.0)
#define X3F_GDF_RANGE_HI (1000.0 / 65535.0)

static float x3f_gdf_tbl[4][X3F_GDF_N * X3F_GDF_N];
static int   x3f_gdf_ready = 0;

static void x3f_gdf_init(void)
{
    /* 四个四分之一像素相位；组序按 §13.4 反出来的：0=(+,+) 1=(+,−) 2=(−,+) 3=(−,−) */
    static const double ph_x[4] = {  0.25,  0.25, -0.25, -0.25 };
    static const double ph_y[4] = {  0.25, -0.25,  0.25, -0.25 };
    int p, r, c;
    for (p = 0; p < 4; p++) {
        for (r = 0; r < X3F_GDF_N; r++) {
            for (c = 0; c < X3F_GDF_N; c++) {
                const double dx = (double)(c - X3F_GDF_R) + ph_x[p];
                const double dy = (double)(r - X3F_GDF_R) + ph_y[p];
                x3f_gdf_tbl[p][r * X3F_GDF_N + c] =
                    (float)exp(-sqrt(dx * dx + dy * dy));
            }
        }
    }
    x3f_gdf_ready = 1;
}

/* 线性域的「细节」（SPP Stage2B 的顺序：残差在 tone curve 之前算）。
 *
 * 形态（§13.3）可以整理成一句非常好用的等价式：
 *     out = restored · m,  m = 1 + (r−1)·g,  r = φ(原)/φ(restored)
 *   ⇒ out = 原 + (g−1)·(原 − restored)
 * 也就是「把**原图与降噪图之差**放大 (g−1) 倍、再加回原图」——
 * g = 1 时 out ≡ 原图（滑块最左端不改变任何东西），g > 1 才是加细节。
 * 这里取的就是这个等价式（线性域、有界、不会把 r 的极值放大成 m 的极值），
 * SPP 那条 log 对称的原始式留在 X3F_DETAIL_MFORM=spp 下做 A/B。
 *
 * restored 用**降噪图**：SPP 那条链里被除的 restored 就是 BlendNR / 噪声模型
 * 双边的结果，dl = 原 − 降噪 正好是"降噪抹掉的那部分"。
 * 注意：我们自己的基线（档 0）不做降噪，所以档 0 出图不含这一步 ——
 * 这条式子保证档 1 起是**连续**的（g−1 → 0 时逐像素回到原图）。 */
/* 线性域的保边低通（= SPP Stage2B 那条"restored"的等效物）。
 *
 * 核还是那颗 5×5 四相位核（§13.3/§13.4），但：
 *  · 在**线性域**做（噪声模型本来就是线性域的，显示域那个 gamma 会把量纲搞乱）；
 *  · 阈值改成**与信号成比例** —— 散粒噪声 σ ∝ √v，所以 range 阈值取
 *    T = nnk · √max(v, 0)（X3F_DETAIL_NNK 标定这个倍数，X3F_DETAIL_NNSHAPE=0
 *    则退化成 T = nnk·max(v,0)，=2 用 SPP 那条噪声模型多项式的形状
 * 20.843 + 0.4342x + 0.5x²（x = v/局部均值，§13.5，硬编码在 里）。
 *  · 只低通**亮度**（引导用 RGB 的 L1，跟 SPP 一样）。
 *
 * 为什么不用我们现成的 NR（nr_apply）：那条链要 CAMF 的噪声模型，而
 * **Quattro 的文件里没有**（见 nr_apply 里的注释）——在 DP0Q 上它静默不跑。
 * SPP 给 Quattro 的噪声模型藏在 （F23）里，就是 §13.5 那些常数。*/
/* 行 worker（2026-10-23 行并行）：每行只读 rgb/y_in、写 dst 的同一行，
 * 逐像素算式一字不改 —— 输出与串行逐位一致。 */
typedef struct {
    const float *rgb, *y_in;
    float *dst;
    int w, h;
    double nnk;
    int shape, use_phase;
    const float *mguide;    /* 非空 = 用这张单通道图算 L1（×3 保持量程），
                             * 即 SPP 那种「引导图与被平均图都是降噪图」的接法 */
} gdf_lin_ctx_t;

static void gdf_lin_row(void *v, size_t yi)
{
    const gdf_lin_ctx_t *cx = (const gdf_lin_ctx_t *)v;
    const float *rgb = cx->rgb;
    const float *y_in = cx->y_in;
    float *dst = cx->dst;
    const int w = cx->w, h = cx->h, shape = cx->shape, use_phase = cx->use_phase;
    const double nnk = cx->nnk;
    const float *mguide = cx->mguide;
    int y = (int)yi, x;
    {
        for (x = 0; x < w; x++) {
            const size_t ic = ((size_t)y * w + x) * 3;
            const double cr = rgb[ic], cg = rgb[ic + 1], cb = rgb[ic + 2];
            const double yc = y_in[(size_t)y * w + x];
            const float *tbl;
            double mx, thr;
            double acc = 0.0, ws = 0.0;
            int r, c;
            const double gc = (mguide != NULL)
                ? (double)mguide[(size_t)y * w + x] : 0.0;

            mx = cr > cg ? cr : cg;
            if (cb > mx) mx = cb;
            if (mx < 0.0) mx = 0.0;
            if (shape == 2) {
                double mean = (cr + cg + cb) / 3.0;
                double q = (mean > 1e-6) ? (mx / mean) : 1.0;
                thr = nnk * (20.843 + 0.4342 * q + 0.5 * q * q);
            } else if (shape == 0) {
                thr = nnk * mx;
            } else {
                thr = nnk * sqrt(mx);
            }
            if (thr < 1e-9) thr = 1e-9;

            tbl = x3f_gdf_tbl[use_phase ? (((y & 1) << 1) | (x & 1)) : 0];
            for (r = 0; r < X3F_GDF_N; r++) {
                int yy = y + r - X3F_GDF_R;
                if (yy < 0) yy = 0; else if (yy >= h) yy = h - 1;
                for (c = 0; c < X3F_GDF_N; c++) {
                    int xx = x + c - X3F_GDF_R;
                    size_t in;
                    double l1, wt;
                    if (xx < 0) xx = 0; else if (xx >= w) xx = w - 1;
                    if (mguide != NULL) {
                        /* 单通道引导：L1 = 3·|Δ引导|，量程与三通道求和一致 */
                        l1 = 3.0 * fabs((double)mguide[(size_t)yy * w + xx] - gc);
                    } else {
                        in = ((size_t)yy * w + xx) * 3;
                        l1 = fabs(rgb[in] - cr) + fabs(rgb[in + 1] - cg)
                           + fabs(rgb[in + 2] - cb);
                    }
                    wt = tbl[r * X3F_GDF_N + c] / (1.0 + l1 / thr);
                    acc += wt * (double)y_in[(size_t)yy * w + xx];
                    ws += wt;
                }
            }
            dst[(size_t)y * w + x] = (float)(ws > 0.0 ? acc / ws : yc);
        }
    }
}

static void x3f_gdf_lowpass_lin(const float *rgb, const float *y_in, float *dst,
                                int w, int h, double nnk, int shape, int use_phase)
{
    gdf_lin_ctx_t c = { rgb, y_in, dst, w, h, nnk, shape, use_phase, NULL };
    if (!x3f_gdf_ready) x3f_gdf_init();
    x3f_parallel_for((size_t)h, gdf_lin_row, &c);
}

/* 同上，但 L1 的引导图换成单通道的 mguide（SPP：引导 = 被平均 = 降噪图）。
 * mguide 为 NULL 时与上者逐位一致。 */
static void x3f_gdf_lowpass_lin_m(const float *rgb, const float *y_in, float *dst,
                                  int w, int h, double nnk, int shape, int use_phase,
                                  const float *mguide)
{
    gdf_lin_ctx_t c = { rgb, y_in, dst, w, h, nnk, shape, use_phase, mguide };
    if (!x3f_gdf_ready) x3f_gdf_init();
    x3f_parallel_for((size_t)h, gdf_lin_row, &c);
}


/* ---- SPP Stage2B 的 ①②③：造残差之前那道降噪（`BilateralFilterMonoF23` 0xdf034）
 *
 * ★ 它就是本文件那一堆 0.19 vs 0.69 的题眼（见 §13.5）。 5×5 四相位核（④）
 *   吃的不是原图，而是**这张双边降噪后的工作图**；而且引导图（算 L1 那张）也是
 *   它 —— 两张都是干净图。于是 L1 ≈ 0，值域项 1/(1+L1/T) ≈ 1，那颗 5×5 核
 *   退化成近乎均匀的 5×5 平均，一下吃掉 69% 高频。我们直接拿带噪原图自引导：
 *   L1 ≫ T → 权重被压到很小 → 核近似恒等，只吃掉 19%。
 *
 * （§13.5）：**纯值域**双边，内层没有空间权重。
 *     idx = trunc(s7·Δ² + 0.5) ; >0x7FF 跳过 ; w = exp(−idx/256) ; out = Σwv/Σw
 *     s7  = 255.875 / (arg7 · range · g²) ,  g = 1/guide
 *     range = table2[floor(clamp(guide,1,14382))] · |c0 + c1x + c2x²| , x = v/g
 *
 * ⚠️ 两个未知量没钉死：`guide` 是谁、`table2` 是什么（§13.5 的遗留）。
 *   钉死前这里用物理等价的散粒噪声律 **σ = nrs·√v** —— 与 5×5 核的
 *   T = nnk·√v 同源，也正是 §14.8 实测那条 NNK ∝ 1/√ISO 的律。
 *   绝对倍数留给 X3F_DETAIL_NRS 去扫（残差在 nrs 上有极大值：太弱则基底仍
 *   带噪、核不平均；太强则基底已平、无可平均的高频）。
 *
 * 逐条对得上的只有三条性质：**纯值域 + 11×11 + LUT 截断**。
 * 两趟（横/竖）近似 —— 值域双边不可分，与 detail_bilateral 同级近似；
 * 两趟各自行并行，趟间有栅栏。开关 X3F_DETAIL_NR=1（默认 0 = 不跑）。 */
#define X3F_BF23_LUTN 2048
#define X3F_BF23_R    5

static float x3f_bf23_lut[X3F_BF23_LUTN + 1];
static int   x3f_bf23_ready = 0;

static void x3f_bf23_init(void)
{
    int i;
    for (i = 0; i <= X3F_BF23_LUTN; i++)
        x3f_bf23_lut[i] = (float)exp(-(double)i / 256.0);
    x3f_bf23_ready = 1;
}

typedef struct {
    const float *src;
    float *dst;
    int w, h, r;
    double nrs;
    int    horiz;      /* 1 = 横向 src→dst，0 = 纵向 src→dst */
} bf23_ctx_t;

static void bf23_row(void *v, size_t yi)
{
    const bf23_ctx_t *cx = (const bf23_ctx_t *)v;
    const float *src = cx->src;
    float *dst = cx->dst;
    const int w = cx->w, h = cx->h, r = cx->r, horiz = cx->horiz;
    const double k1 = 128.0 / (cx->nrs * cx->nrs);   /* s7 = k1 / v；σ² = nrs²·v */
    int y = (int)yi, x, d;

    for (x = 0; x < w; x++) {
        const size_t oc = (size_t)y * w + x;
        const double c = src[oc];
        double s7, acc = 0.0, ws = 0.0;
        int idx;
        if (c <= 0.0) { dst[oc] = (float)c; continue; }
        s7 = k1 / c;
        for (d = -r; d <= r; d++) {
            int xx = x, yy = y;
            double v, dd, wt;
            if (horiz) { xx = x + d; if (xx < 0) xx = 0; else if (xx >= w) xx = w - 1; }
            else       { yy = y + d; if (yy < 0) yy = 0; else if (yy >= h) yy = h - 1; }
            v  = src[(size_t)yy * w + xx];
            dd = v - c;
            {
                /* 暗部 c→0 时 s7 = k1/c ≈ 1e20，先比阈值再 (int) 截断，
                 * 否则 s7·dd² 溢出 int 是 UB（散粒噪声里暗部是常态）。 */
                const double q = s7 * dd * dd;
                if (q < 0.0 || q > (double)X3F_BF23_LUTN) continue;
                idx = (int)(q + 0.5);
            }
            wt = x3f_bf23_lut[idx];
            acc += wt * v;
            ws  += wt;
        }
        dst[oc] = (float)(ws > 0.0 ? acc / ws : c);
    }
}

/* 两趟：src →（横）→ tmp →（竖）→ dst。tmp 由调用方提供（w*h float）。 */
static void x3f_bf23_mono(const float *src, float *dst, float *tmp,
                          int w, int h, int r, double nrs)
{
    bf23_ctx_t c1, c2;
    if (!x3f_bf23_ready) x3f_bf23_init();
    if (r < 1) r = 1;
    if (r > 16) r = 16;
    c1.src = src; c1.dst = tmp; c1.w = w; c1.h = h; c1.r = r;
    c1.nrs = nrs; c1.horiz = 1;
    c2.src = tmp; c2.dst = dst; c2.w = w; c2.h = h; c2.r = r;
    c2.nrs = nrs; c2.horiz = 0;
    x3f_parallel_for((size_t)h, bf23_row, &c1);
    x3f_parallel_for((size_t)h, bf23_row, &c2);
}

/* NR 三参数。缓存键与执行共用同一份，避免两处读 env 走岔。 */
static void x3f_detail_nr_params(int *on, int *nr_r, double *nr_s)
{
    const char *e = getenv("X3F_DETAIL_NR");
    *on   = (e != NULL && e[0] != '0' && e[0] != '\0');
    *nr_r = (int)detail_env("X3F_DETAIL_NRR", (double)X3F_BF23_R);
    *nr_s = detail_env("X3F_DETAIL_NRS", 0.015);
    if (*on && getenv("X3F_DETAIL_NRISO") != NULL) {
        const double iso = detail_env("X3F_DETAIL_ISO", x3f_file_iso());
        if (iso > 0.0)
            *nr_s *= sqrt(1600.0 / iso) * detail_env("X3F_DETAIL_NRISOK", 1.0);
    }
}

/* 造细节残差的基底：SPP 是「降噪图 → 5×5 核精修」，我们原来只有后半。
 *
 *   NR 开（X3F_DETAIL_NR=1）：ynr = BF23(ylin)；restored = 5×5 核(ynr)，
 *       L1 的引导也是 ynr；*base_out = ynr（调用方用完 free）。
 *   NR 关（默认 0）        ：restored = 5×5 核(ylin)，引导 = dsrc 的 RGB，
 *       *base_out = NULL —— 与改动前**逐位一致**。
 *
 * 返回 1 = 用了 NR 基底（这时 boost 必须传 base_luma，否则噪声会被当细节放大）。
 * OOM 时静默退回 NR 关的路径。 */
static int x3f_detail_substrate(const float *dsrc, const float *ylin, float *rlin,
                                float **base_out, int w, int h,
                                double nnk, int shape, int phase,
                                int nr_on, int nr_r, double nr_s)
{
    const size_t nn = (size_t)w * h;

    *base_out = NULL;
    if (!nr_on) {
        x3f_gdf_lowpass_lin(dsrc, ylin, rlin, w, h, nnk, shape, phase);
        return 0;
    }
    {
        float *ynr = (float *)malloc(nn * sizeof(float));
        float *tmp = (float *)malloc(nn * sizeof(float));
        if (ynr == NULL || tmp == NULL) {
            free(ynr); free(tmp);
            x3f_gdf_lowpass_lin(dsrc, ylin, rlin, w, h, nnk, shape, phase);
            return 0;
        }
        x3f_bf23_mono(ylin, ynr, tmp, w, h, nr_r, nr_s);
        free(tmp);
        x3f_gdf_lowpass_lin_m(dsrc, ynr, rlin, w, h, nnk, shape, phase, ynr);
        *base_out = ynr;
        return 1;
    }
}

/* restored 的噪声尺度（= F23Stage2BProcParams+0x58，BilateralFilterMonoF23 的 arg7）。
 *
 * （`SetParameterStage2BCPULow` 0xa7bc4 / `...CPUHi` 0xa7f14，跳转表在
 * 0x19d424，模式由 `fcvtzu w22, s1, #2` 给出 = (int)(4·clamp(IPM2+0x490,0,1))）：
 *
 *   mode 0 : arg7 = 0.2             （**不随 ISO 变**）
 *   mode 1 : ISO<800 ? 1.0 : 0.5    mode 2 : ISO<800 ? 2.0 : 1.0
 *   mode 3 : ISO<800 ? 0.05 : 3.0   mode ≥4: ISO<800 ? 4.0 : 2.0
 *
 * 这里的 ISO 就是 `params+0x3c` 那一路（request+0x7c8 = ISO/100）：setter 里
 * `fcmp s8, 8.0` 就是"ISO 800 分界"。所以**分界线在 ISO 800，低 ISO 侧 arg7 ×2**。
 *
 * 为什么要它：BilateralFilterMonoF23 的 range 权重是
 *     w = exp(−s7·Δ²/256),  s7 = 255.875 / (arg7 · range · g²)
 *   ⇒ σ² = 128/s7 = 0.5·arg7·range·g²  ⇒ **σ ∝ √arg7**。
 * 我们的 NNK 就是这个"相对 σ"，所以 NNK 要乘 √(arg7/arg7_锚点)。
 * 锚点 = ISO1600、mode 2（arg7 = 1.0）时 NNK = 0.36（DP0Q2315 对参考图定的）。
 *
 * 注意 mode 来自 IPM2+0x490（细节滑块 0x4a0 的邻居），**不是** ISO ——
 * 想试别的档位用 X3F_DETAIL_NRMODE=<0..4>。 */
/* NNK 的 ISO 律：**两条路**
 *
 * (1) 那条（arg7 阶梯）：σ ∝ √arg7，低 ISO 侧 arg7 ×2 → σ ×1.41。
 *     它是 BilateralFilterMonoF23 那**一颗**双边核的宽度，不是整条降噪链。
 * (2) 实测那条（默认）：我们用一颗相对阈值的保边核顶替 **整条降噪**
 *     （Stage1 NR + BlendNR + 这颗双边），所以要按实测标：
 *       ISO 100  → 要匹配参考的 Δrel = 0.9885 ⇒ NNK ≈ 0.18（扫 0.062/0.09/0.12/0.16
 *                  → Δrel 0.797/0.902/0.936/0.967，外推 ≈0.20 命中 0.9885）
 *       ISO 1600 → 要匹配 Δrel = 0.1035 ⇒ NNK = 0.044（锚点）
 *     比值 0.20/0.044 = 4.5，横跨 ISO 100→1600（16 倍）⇒ 指数 ln4.5/ln16 = 0.546
 *     ⇒ **NNK ∝ 1/√ISO** —— 正是散粒噪声的相对尺度（相对噪声 ∝ 1/√光子数）✓
 *     取 NNK(ISO) = 0.044·√(1600/ISO)，ISO>1600 也照这条式子往下走（噪声更大 →
 *     相对阈值更小，与 arg7 阶梯同向）。
 * 想用 (1) 那条：X3F_DETAIL_NNKRULE=arg7（再配 X3F_DETAIL_NRMODE）。 */
static double x3f_detail_nnk_scale(void)
{
    const double iso = detail_env("X3F_DETAIL_ISO", x3f_file_iso());
    const char *rule = getenv("X3F_DETAIL_NNKRULE");

    if (rule != NULL && rule[0] == 'a') {          /*那条*/
        const int lowiso = (iso > 0.0 && iso < 800.0);
        const int mode = (int)detail_env("X3F_DETAIL_NRMODE", 2.0);
        double a;
        switch (mode) {
        case 0:  a = 0.2;                    break;
        case 1:  a = lowiso ? 1.0  : 0.5;    break;
        case 3:  a = lowiso ? 0.05 : 3.0;    break;
        case 4:  a = lowiso ? 4.0  : 2.0;    break;
        default: a = lowiso ? 2.0  : 1.0;    break;   /* mode 2 */
        }
        return sqrt(a / 1.0);
    }
    if (iso > 0.0)
        return sqrt(1600.0 / iso) * detail_env("X3F_DETAIL_NNK_ISOK", 1.0);
    return 1.0;                                    /* 读不到 ISO：按锚点 */
}

/* 贴回：用**原图 − restored（亮度差）**做乘性/加性放大。
 *   mform=0（默认）: out = 原 + (g−1)·(Yo − Yr)          ← SPP 式的等价、有界
 *   mform=1        : m 用 SPP 那条 log 对称两段式，out = 原 + (m−1)·(Yo − Yr) */
/* 行并行（2026-10-23）：逐像素算式不变，输出与串行逐位一致。 */
typedef struct {
    const float *orig, *rest_luma;
    float *dst;
    int w;
    double g, eps;
    int mform, hl_soften;
    const float *clip;
    const float *base_luma;   /* 非空：残差取 (base_luma − yn)，见下 */
} dboost_ctx_t;

static void dboost_row(void *v, size_t yi)
{
    const dboost_ctx_t *c = (const dboost_ctx_t *)v;
    const float *orig = c->orig, *rest_luma = c->rest_luma;
    float *dst = c->dst;
    const int w = c->w;
    const double g = c->g, eps = c->eps;
    const int mform = c->mform, hl_soften = c->hl_soften;
    const float *clip = c->clip, *base_luma = c->base_luma;
    const size_t i0 = (size_t)yi * w, i1 = i0 + w;
    size_t i;
    for (i = i0; i < i1; i++) {
        const double or_ = orig[i * 3], og = orig[i * 3 + 1], ob = orig[i * 3 + 2];
        /* base_luma 非空 = 残差取（基底 − restored），不是（原图 − restored）。
         * 基底是降噪图时，BF23 抹掉的那份噪声**不进残差** —— 否则细节滑块
         * 会把噪声当细节再放大一遍。这是"在干净基底上提细节"与"连噪一起提"
         * 的分界；同时守住档 0（g−1 → 0 时逐像素回到 orig）。 */
        const double yo = (base_luma != NULL)
            ? (double)base_luma[i]
            : 0.2126 * or_ + 0.7152 * og + 0.0722 * ob;
        const double yn = rest_luma[i];
        double k;
        int c2;
        if (mform != 0) {
            double r;
            if (yo <= 0.0 || yn <= 0.0) r = 1.0;
            else {
                r = (yo + eps) / (yn + eps);
                if (r < 0.001) r = 0.001; else if (r > 1000.0) r = 1000.0;
            }
            k = ((r > 1.0) ? 1.0 + (r - 1.0) * g
                           : 1.0 / ((1.0 - g) + g / r)) - 1.0;
        } else {
            k = g - 1.0;
        }
        double kk = k;
        /* 过曝矫正关闭：RAW 已经截断的像素不做 Detail 放大 —— 那里只保留
         * 正常解码的基础结果，什么也不加。未截断的像素完全照旧。 */
        if (hl_soften && clip != NULL && clip[i] >= 0.5) kk = 0.0;
        if (hl_soften && clip == NULL) {
            const double s = 1.0 - sstep01(0.75, 0.88, yo);
            kk *= s;
        }
        for (c2 = 0; c2 < 3; c2++) {
            const double v = orig[i * 3 + c2] + kk * (yo - yn);
            dst[i * 3 + c2] = (float)(v < 0.0 ? 0.0 : v);
        }
    }
}

static float *x3f_detail_linear_boost(const float *orig, const float *rest_luma,
                                      int w, int h, double g, double eps, int mform,
                                      int hl_soften, const float *clip,
                                      const float *base_luma)
{
    float *dst = (float *)malloc((size_t)w * h * 3 * sizeof(float));
    dboost_ctx_t cx = { orig, rest_luma, dst, w, g, eps, mform, hl_soften, clip,
                        base_luma };
    if (dst == NULL) return NULL;
    x3f_parallel_for((size_t)h, dboost_row, &cx);
    return dst;
}

/* 引导滤波（自引导）：把 Y 保边低通进 dst。out/stride/bits 用来取引导图的 RGB。
 * 行并行（2026-10-23）：每行只读引导图/Y、写 dst 的同一行，逐位一致。 */
typedef struct {
    const void *out;
    int stride, bits;
    const float *Y;
    float *dst;
    int w, h, use_phase;
    double rlo, rhi;
} gdf_ctx_t;

static void gdf_row(void *v, size_t yi)
{
    const gdf_ctx_t *c = (const gdf_ctx_t *)v;
    const int y = (int)yi;
    const int w = c->w, h = c->h, bits = c->bits, stride = c->stride;
    const int use_phase = c->use_phase;
    const double rlo = c->rlo, rhi = c->rhi;
    const float *Y = c->Y;
    float *dst = c->dst;
    const void *out = c->out;
    const uint8_t *crow = (const uint8_t *)out + (size_t)y * stride;
    float *drow = dst + (size_t)y * w;
    int x;

    for (x = 0; x < w; x++) {
        const size_t ic = (size_t)x * 3;
        const double cr = px_get(crow, bits, ic);
        const double cg = px_get(crow, bits, ic + 1);
        const double cb = px_get(crow, bits, ic + 2);
        const float *tbl;
        double mx, t, thr;
        double acc = 0.0, ws = 0.0;
        int r, cc;

        mx = cr > cg ? cr : cg;
        if (cb > mx) mx = cb;
        t = (mx - X3F_GDF_T_LO) / (X3F_GDF_T_HI - X3F_GDF_T_LO);
        if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
        thr = rlo + (rhi - rlo) * t;

        tbl = x3f_gdf_tbl[use_phase ? (((y & 1) << 1) | (x & 1)) : 0];

        for (r = 0; r < X3F_GDF_N; r++) {
            int yy = y + r - X3F_GDF_R;
            const uint8_t *nrow;
            if (yy < 0) yy = 0; else if (yy >= h) yy = h - 1;
            nrow = (const uint8_t *)out + (size_t)yy * stride;
            for (cc = 0; cc < X3F_GDF_N; cc++) {
                int xx = x + cc - X3F_GDF_R;
                size_t in;
                double nr, ng, nb, l1, wt;
                if (xx < 0) xx = 0; else if (xx >= w) xx = w - 1;
                in = (size_t)xx * 3;
                nr = px_get(nrow, bits, in);
                ng = px_get(nrow, bits, in + 1);
                nb = px_get(nrow, bits, in + 2);
                l1 = fabs(nr - cr) + fabs(ng - cg) + fabs(nb - cb);
                wt = tbl[r * X3F_GDF_N + cc] / (1.0 + l1 / thr);
                acc += wt * (double)Y[(size_t)yy * w + xx];
                ws += wt;
            }
        }
        drow[x] = (float)(ws > 0.0 ? acc / ws : (double)Y[(size_t)y * w + x]);
    }
}

static void x3f_gdf_lowpass(const void *out, int stride, int bits,
                            const float *Y, float *dst, int w, int h, int use_phase)
{
    const double rng = detail_env("X3F_DETAIL_RANGE", 1.0);
    gdf_ctx_t c;
    c.out = out; c.stride = stride; c.bits = bits; c.Y = Y; c.dst = dst;
    c.w = w; c.h = h; c.use_phase = use_phase;
    c.rlo = X3F_GDF_RANGE_LO * rng;
    c.rhi = X3F_GDF_RANGE_HI * rng;
    if (!x3f_gdf_ready) x3f_gdf_init();
    x3f_parallel_for((size_t)h, gdf_row, &c);
}

/* 高光结构柔化的亮度掩膜：1 = 原样，0 = 完全不贴高频结构。
 * 参数在 apply_spatial 开头读一次（detail_env 每次都 getenv，放在逐像素
 * 循环里会白烧几秒）。 */
static inline double hl_soft_mask(double y, double k, double lo, double hi)
{
    if (k <= 0.0) return 1.0;
    if (k > 1.0) k = 1.0;
    return 1.0 - k * sstep01(lo, hi, y);
}

/* 该像素还保留多少"额外结构"（Detail/锐度）：
 *   - 过曝矫正开着：1（管线原样）
 *   - 关闭且有 RAW 饱和图：截断像素 0（回到正常解码，什么也不加）
 *   - 关闭但没有饱和图（SFD 合成）：按亮度掩膜淡出 */
static inline double hl_structure_keep(int hl_soften, const float *clip, size_t i,
                                       double y, double k, double lo, double hi)
{
    if (!hl_soften) return 1.0;
    if (clip != NULL) return clip[i] >= 0.5 ? 0.0 : 1.0;
    return hl_soft_mask(y, k, lo, hi);
}

/* ---- apply_spatial 行并行（2026-10-23）：各阶段逐像素算式一字不改，
 * 只是按行摊到多核；输出与串行逐位一致。 ---- */
typedef struct {
    void  *out;                 /* 显示缓冲（16/8 位交错 RGB） */
    int    stride, bits, w, h;
    float *Y, *Cb, *Cr, *buf, *low, *dgain;
    const float *clip;
    /* 阴影/高光 */
    double shadow, highlight;
    /* 锐度 */
    double delta, abs_cl;
    /* Detail 比值 */
    double dgain_g, dgain_eps;
    /* 高光结构掩膜参数 */
    double soft_k, soft_lo, soft_hi;
    int    hl_soften;
    /* 黑白锐度增强（解耦旋钮，默认关；见 apply_spatial 里 X3F_SHARP_*_BW）。
     * 当前黑白与彩色共用同一盒式高通当细节层，强调最细频率、平权所有亮度，
     * 与 SMono（中频结构 + 亮边更立）质感有别。这三个旋钮只作用于
     * mono 分支，彩色与零滑杆输出逐位不变。 */
    int    bw_r;       /* 细节层低通半径：1=当前盒式(3×3)；2~3=更宽→中频结构更"实" */
    double bw_band;    /* 亮度带增强：0=关；>0 亮边乘 (1+bw_band·max(0,Y-0.5))，
                        * 模拟 SPP「先锐化后 tone LUT」导致的亮边响应更强（立体感） */
    /* 亮度权重（2026-11-04 全链路广色域）。
     * 转域现在发生在 apply_row 内部，所以这个阶段拿到的**已经是目标域码值**，
     * Y/Cb/Cr 分解与反解的权重必须换成目标域原色的 Y 行系数；继续用 BT.709
     * 等于拿 sRGB 的亮度定义去量 ProPhoto 的三通道，阴影/高光的分区判据会偏
     * （ProPhoto 的 R 通道权重 0.288 vs 0.213，蓝几乎不贡献）。
     * sRGB 输出填 (0.2126, 0.7152, 0.0722)，逐位不变。 */
    double yw[3];
    int    yw_srgb;   /* 1 = 用原来的 BT.709 字面量算式（sRGB 输出，逐位零回归）。
                       * 直写 yw[] 会换成 double 运算、常量折叠也消失，实测
                       * 8bit sRGB 输出会在 0.06% 像素上差 1~2 级。 */
} spat_ctx_t;

static void spat_decomp_row(void *v, size_t yi)
{
    spat_ctx_t *c = (spat_ctx_t *)v;
    const int y = (int)yi, w = c->w, bits = c->bits;
    const void *row = (const uint8_t *)c->out + (size_t)y * c->stride;
    float *Y = c->Y, *Cr = c->Cr, *Cb = c->Cb;
    int x;
    for (x = 0; x < w; x++) {
        const size_t i = (size_t)y * w + x;
        float rr = (float)px_get(row, bits, (size_t)x * 3);
        float gg = (float)px_get(row, bits, (size_t)x * 3 + 1);
        float bb = (float)px_get(row, bits, (size_t)x * 3 + 2);
        float yy = c->yw_srgb
            ? (0.2126f*rr + 0.7152f*gg + 0.0722f*bb)
            : (float)(c->yw[0]*rr + c->yw[1]*gg + c->yw[2]*bb);
        Y[i] = yy; Cr[i] = rr - yy; Cb[i] = bb - yy;
    }
}

static void spat_zone_row(void *v, size_t yi)
{
    spat_ctx_t *c = (spat_ctx_t *)v;
    const size_t i0 = (size_t)yi * c->w, i1 = i0 + c->w;
    const double shadow = c->shadow, highlight = c->highlight;
    float *Y = c->Y, *Cr = c->Cr, *Cb = c->Cb;
    size_t i;
    for (i = i0; i < i1; i++) {
        double y0 = Y[i];
        double y1 = apply_sh_d(y0, shadow, highlight);
        if (y0 > 1e-9) {
            double s = y1 / y0;
            Cr[i] = (float)(Cr[i] * s);
            Cb[i] = (float)(Cb[i] * s);
        }
        Y[i] = (float)y1;
    }
}

static void spat_sharp_mono_row(void *v, size_t yi)
{
    spat_ctx_t *c = (spat_ctx_t *)v;
    const size_t i0 = (size_t)yi * c->w, i1 = i0 + c->w;
    const double delta = c->delta;
    const double soft_k = c->soft_k, soft_lo = c->soft_lo, soft_hi = c->soft_hi;
    const int hl_soften = c->hl_soften;
    const float *clip = c->clip, *buf = c->buf;
    float *Y = c->Y;
    size_t i;
    for (i = i0; i < i1; i++) {
        const double hs = hl_structure_keep(hl_soften, clip, i, buf[i],
                                            soft_k, soft_lo, soft_hi);
        double yv = Y[i];
        double band = 1.0;
        if (c->bw_band != 0.0) {
            /* 亮度带增强：亮边（Y 远离中灰）乘更大系数，模拟 SPP 先锐化后 tone
             * 的「亮边更立」。仅正偏方向（Y>0.5）生效，绝不削弱暗部边缘。 */
            double d = yv - 0.5;
            if (d > 0.0) band = 1.0 + c->bw_band * (d * 2.0);
        }
        Y[i] = (float)(yv + 0.25 * (yv - buf[i]) * delta * hs * band);
    }
}

static void spat_sharp_color_row(void *v, size_t yi)
{
    spat_ctx_t *c = (spat_ctx_t *)v;
    const size_t i0 = (size_t)yi * c->w, i1 = i0 + c->w;
    const double delta = c->delta, abs_cl = c->abs_cl;
    const double soft_k = c->soft_k, soft_lo = c->soft_lo, soft_hi = c->soft_hi;
    const int hl_soften = c->hl_soften;
    const float *clip = c->clip, *buf = c->buf;
    float *Y = c->Y, *Cr = c->Cr, *Cb = c->Cb;
    size_t i;
    for (i = i0; i < i1; i++) {
        const double hs = hl_structure_keep(hl_soften, clip, i, buf[i],
                                            soft_k, soft_lo, soft_hi);
        double r = Y[i] + Cr[i];          /* 由 Y/Cb/Cr 反解显示域 RGB */
        double b = Y[i] + Cb[i];
        double g = c->yw_srgb
            ? (Y[i] - 0.2126 * r - 0.0722 * b) / 0.7152
            : (Y[i] - c->yw[0] * r - c->yw[2] * b) / c->yw[1];
        double S = fabs(r) + fabs(g) + fabs(b);
        double detail = 0.25 * (Y[i] - buf[i]) * delta;
        if (detail > abs_cl) detail = abs_cl;
        else if (detail < -abs_cl) detail = -abs_cl;
        if (detail > 0.35 * S) detail = 0.35 * S;
        else if (detail < -0.35 * S) detail = -0.35 * S;
        if (S > 1e-9) {
            double f = 1.0 + detail / S * hs;
            Y[i]  = (float)(Y[i] * f);
            Cr[i] = (float)(Cr[i] * f);
            Cb[i] = (float)(Cb[i] * f);
        }
    }
}

static void spat_ratio_row(void *v, size_t yi)
{
    spat_ctx_t *c = (spat_ctx_t *)v;
    const size_t i0 = (size_t)yi * c->w, i1 = i0 + c->w;
    const double g = c->dgain_g, eps = c->dgain_eps;
    const double soft_k = c->soft_k, soft_lo = c->soft_lo, soft_hi = c->soft_hi;
    const int hl_soften = c->hl_soften;
    const float *clip = c->clip, *low = c->low;
    const float *Y = c->Y;
    float *dgain = c->dgain;
    size_t i;
    for (i = i0; i < i1; i++) {
        double yv = Y[i], lo = low[i], r, m;
        if (yv <= 0.0 || lo <= 0.0) { dgain[i] = 1.0f; continue; }
        r = (yv + eps) / (lo + eps);
        if (r < 0.001) r = 0.001; else if (r > 1000.0) r = 1000.0;
        if (r > 1.0) m = 1.0 + (r - 1.0) * g;
        else         m = 1.0 / ((1.0 - g) + g / r);
        /* 高光里把乘性残差收向 1（= 不贴结构），过渡与锐度那条共用掩膜。 */
        dgain[i] = (float)(1.0 + (m - 1.0)
                           * hl_structure_keep(hl_soften, clip, i, lo,
                                               soft_k, soft_lo, soft_hi));
    }
}

static void spat_recombine_row(void *v, size_t yi)
{
    spat_ctx_t *c = (spat_ctx_t *)v;
    const int y = (int)yi, w = c->w, bits = c->bits;
    void *row = (uint8_t *)c->out + (size_t)y * c->stride;
    const float *Y = c->Y, *Cr = c->Cr, *Cb = c->Cb;
    const float *low = c->low, *dgain = c->dgain;
    int x;
    for (x = 0; x < w; x++) {
        const size_t i = (size_t)y * w + x;
        const double base = (dgain != NULL) ? (double)low[i] : (double)Y[i];
        const double m = (dgain != NULL) ? (double)dgain[i] : 1.0;
        double rr = (base + Cr[i]) * m;
        double bb = (base + Cb[i]) * m;
        double gg = c->yw_srgb
            ? ((base - 0.2126 * (base + Cr[i]) - 0.0722 * (base + Cb[i])) / 0.7152) * m
            : ((base - c->yw[0] * (base + Cr[i]) - c->yw[2] * (base + Cb[i])) / c->yw[1]) * m;
        px_put(row, bits, (size_t)x * 3,     rr);
        px_put(row, bits, (size_t)x * 3 + 1, gg);
        px_put(row, bits, (size_t)x * 3 + 2, bb);
    }
}

static int apply_spatial(const x3f_edit_params_t *p, void *out,
                         int w, int h, int stride, int bits,
                         const float *clip)
{
    size_t n, i;
    float *Y, *Cb, *Cr, *buf, *tmp, *low = NULL, *lp = NULL, *dgain = NULL;
    int y, x;
    spat_ctx_t sc;
    int dlevel, nsc = 0;
    /* 高光结构去除参数：**只在过曝矫正关闭时**（hl_soften=1）生效。
     * 矫正开启时这条掩膜恒为 1，锐度/细节/最终像素与原来逐字节一致。
     * 关闭时：93→96 快速过渡，96% 以上细节/锐度完全退出，只留低通。 */
    const double soft_k  = p->hl_soften ? detail_env("X3F_HL_SOFT", 1.0) : 0.0;
    const double soft_lo = detail_env("X3F_HL_SOFT_LO", 0.93);
    const double soft_hi = detail_env("X3F_HL_SOFT_HI", 0.96);

    /* 这里只做锐度和细节（都是亮度通道）。降噪一律不在这里做：
     * 色度降噪走 x3fuse 的 NLM（解码那一步，只在导出时开，见 x3f_render.h），
     * 亮度降噪仍然不做 —— SPP 那套是噪声模型驱动的小波滤波，我们写不出
     * 同等效果的东西，就不摆到界面上。 */
    dlevel = x3f_detail_level(p->detail);
    /*X3F_DETAIL_FORCE：档 0 也照贴残差（行为，用于对参考图定标）。
     * 默认不开 —— 我们基线没有这一步，档 0 先留成关。 */
    const int has_zone = p->shadow != 0.0 || p->highlight != 0.0;
    if (dlevel == 0 && getenv("X3F_DETAIL_FORCE") == NULL &&
        p->sharpness <= 0 && !has_zone)
        return X3FRENDER_OK;

    n = (size_t)w * h;
    Y   = (float *)malloc(n * sizeof(float));
    Cb  = (float *)malloc(n * sizeof(float));
    Cr  = (float *)malloc(n * sizeof(float));
    buf = (float *)malloc(n * sizeof(float));
    tmp = (float *)malloc(n * sizeof(float));
    if (dlevel > 0 || getenv("X3F_DETAIL_FORCE") != NULL) {
        low = (float *)malloc(n * sizeof(float));
        lp  = (float *)malloc(n * sizeof(float));
    }
    if (!Y || !Cb || !Cr || !buf || !tmp ||
        ((dlevel > 0 || getenv("X3F_DETAIL_FORCE") != NULL) && (!low || !lp))) {
        free(Y); free(Cb); free(Cr); free(buf); free(tmp); free(low); free(lp);
        return X3FRENDER_ERR_NOMEM;
    }

    sc.out = out; sc.stride = stride; sc.bits = bits; sc.w = w; sc.h = h;
    sc.Y = Y; sc.Cb = Cb; sc.Cr = Cr; sc.buf = buf; sc.low = low; sc.dgain = NULL;
    sc.clip = clip;
    sc.shadow = p->shadow; sc.highlight = p->highlight;
    sc.delta = 0.0; sc.abs_cl = 1000.0 / 32768.0;
    sc.dgain_g = 0.0; sc.dgain_eps = 0.0;
    sc.soft_k = soft_k; sc.soft_lo = soft_lo; sc.soft_hi = soft_hi;
    sc.hl_soften = p->hl_soften;
    /* 亮度权重：按**输出域原色**取 Y 行系数（各矩阵 Y 行已归一化，和为 1）。
     * 这个阶段的数据已经是目标域码值（换域在 apply_row 末尾），继续拿 BT.709
     * 去量 ProPhoto/P3 会把阴影高光的分区判据算偏。sRGB 一行与旧行为逐位相同。 */
    switch (p->out_space) {
    case X3F_OUT_PROPHOTO: sc.yw[0] = 0.2880402; sc.yw[1] = 0.7118741; sc.yw[2] = 0.0000857; break;
    case X3F_OUT_ADOBE:    sc.yw[0] = 0.2973769; sc.yw[1] = 0.6273491; sc.yw[2] = 0.0752741; break;
    case X3F_OUT_P3:
    case X3F_OUT_DCI_P3:   sc.yw[0] = 0.2289746; sc.yw[1] = 0.6917385; sc.yw[2] = 0.0792869; break;
    case X3F_OUT_2020:     sc.yw[0] = 0.2627;    sc.yw[1] = 0.6780;    sc.yw[2] = 0.0593;    break;
    default:               sc.yw[0] = 0.2126;    sc.yw[1] = 0.7152;    sc.yw[2] = 0.0722;    break;
    }
    sc.yw_srgb = (p->out_space == X3F_OUT_SRGB);
    /* 黑白锐度增强旋钮：半径（仅 2/3 有效）与亮度带增强。
     * 半径取值优先级：env X3F_SHARP_BW_R > 同目录 bw_sharp_r.txt > 默认 2。
     * 仅作用于 mono 路径（彩色半径锁 1），零滑杆不走此分支，md5 不变。 */
    {
        double env = detail_env("X3F_SHARP_BW_R", 0.0);
        int r = (env > 0.0) ? (int)(env + 0.5) : g_bw_r_file;
        if (r == -2) { r = read_bw_r_from_file(); g_bw_r_file = r; }
        if (r != 2 && r != 3) r = 2;
        sc.bw_r = r;
    }
    sc.bw_band = detail_env("X3F_SHARP_BW_BAND", 0.0);

    /* 分解成显示空间的 Y / Cb / Cr（行并行） */
    x3f_parallel_for((size_t)h, spat_decomp_row, &sc);

    /* 阴影/高光先做，后面的锐度与 Detail 再基于调整后的亮度工作。 */
    if (has_zone) {
        x3f_parallel_for((size_t)h, spat_zone_row, &sc);
    }

    /* 高光结构柔化（2026-10-20，用户报「过曝矫正关闭时高光里的结构不自然」）。
     *
     * 基础曲线 + hl_keep 把高光端压成一段很窄的肩：线性域里差好几档的亮度，
     * 落到显示域只差 250~255 几个码值。锐度和 Detail 的残差在这里乘上去，
     * 就会变成一圈圈等高线（火焰中心最明显）。
     *
     * 掩膜在 0.94 起淡出、0.998 归零：94% 以下一个像素都不碰，
     * 纯白处只剩低通本身，过渡是 smoothstep。
     * X3F_HL_SOFT / _LO / _HI 只给 lab 做 A/B，应用不设。 */
    /*锐度：乘性残差锐化（TonalAdjustments §4.5，完整）。
     *
     * 相机算法（FPixelProc2::Sharpen ，彩色版逐像素）：
     *   S      = |R|+|G|+|B|
     *   detail = clamp(0.25·D·Δ, ±1000) → clamp(±0.35·S)   ← 两级限幅
     *   out_c  = clamp(c·(1 + detail/S))                    ← 三通道同系数
     * D 是 SMono 单色细节层（有符号残差），Δ = ComputeSharpenAmount：
     * 滑杆 × sqrt((sx+sy)/2)（分辨率缩放），负值经同一核就是平滑。
     * 等比作用 RGB ⇒ 色相严格不变；亮度-only 版等价于
     *   Y' = Y·(1+detail/S)，Cr/Cb 同乘 (1+detail/S)（保色比，和上面
     *   阴影/高光的处理一致）。D 用 Y − blur(Y) 代替 SMono 层。
     *
     * 常数按 32768 满量程标定：±1000 → 0..1 域 ±0.03052。
     * ±0.35·S 相对限幅就是 防光晕（单次最多改 ±35%）。
     * X3F_SHARP_GAIN：Δ 的标定乘数（我方 blur 高通 vs SPP SMono 细节层的
     * 幅度差，按固定倍数补）。
     * 定标史：
     *   19.2（2026-10-22）= 与旧 unsharp 同滑杆位置等强度（内部口径）。
     * 3.4 （2026-10-24）= **相机算法实测**：DP2Q1884 导出 sharp=0 /
     *        sharp=1.0 两档 16-bit TIFF（sRGB 全尺寸其余默认），差值法在
     *        双方非零纹理区（738 万 px）比响应，逐像素比中位数 3.40
     *        （分亮度带 2.6~3.2 互证；最小二乘 5.23 被模式差异拉高不取）。
     * 实测当前产品(19.2) 响应是 5.0 倍。
     *        残余不确定：SPP 导出档位若未精确落在 1.0（如默认非 0），
     *        本值按比例缩放；可补导 sharp=2.0 档验证线性后收紧。
     * X3F_SHARP_RES：sqrt((sx+sy)/2) 的预览缩放因子（1:1 输出 = 1）。
     * 现由参数 p->sharp_res 携带（app 预览按 渲染长边/全尺寸长边 传入），
     * 环境变量保留作 lab 覆盖；两者都没有 / 非正 = 1.0。 */
    if (p->sharpness > 0) {
        double sgain = detail_env("X3F_SHARP_GAIN", 3.4);
        if (p->mono) {
            /* 黑白锐度增益与彩色解耦：默认沿用 3.4（与彩色同一 SPP 标定），
             * 但可单独用 X3F_SHARP_GAIN_BW 调（黑白质感是单独的话题，见上）。 */
            const double sbw = detail_env("X3F_SHARP_GAIN_BW", -1.0);
            if (sbw > 0.0) sgain = sbw;
        }
        const char *sre = getenv("X3F_SHARP_RES");
        double sres = (sre != NULL && *sre != '\0') ? atof(sre)
                     : (p->sharp_res > 0.0 ? p->sharp_res : 1.0);
        if (!(sres > 0.0)) sres = 1.0;
        const double delta = p->sharpness * sgain * sres; /* Δ = SPP 滑杆值×标定 */
        const double abs_cl = 1000.0 / 32768.0;
        memcpy(buf, Y, n * sizeof(float));
        blur2(buf, tmp, w, h, p->mono ? sc.bw_r : 1);
        sc.delta = delta;
        if (p->mono) {
            /*BnW 版 ApplySharpLBnW （TonalAdjustments §4.5(e)）：
             *   img_c += 0.25·D·Δ     三个通道加**同一灰度增量**，输出只钳 [0,32768]
             * ——没有 ±1000 绝对限幅，也没有 ±0.35·S 相对（防光晕）限幅。
             * 在 YUV 域等价于 Y += detail（Cb/Cr 不动）：三通道同增量 ⇔ 亮度动
             * detail、色度不动。灰度段比彩色版强 ~3×（彩色 out=c·(1+detail/S)，
             * 中灰 S≈3Y ⇒ 每通道只动 detail/3），这正是 SPP 黑白锐化"更带劲"的
             * 出处，不是错觉。D 用 Y−blur(Y) 代替 SMono 层；黑白可经 X3F_SHARP_BW_R
             * 换更宽的低通（中频结构更"实"），经 X3F_SHARP_BW_BAND 加亮边增强。 */
            x3f_parallel_for((size_t)h, spat_sharp_mono_row, &sc);
        } else {
            x3f_parallel_for((size_t)h, spat_sharp_color_row, &sc);
        }
    }

    /*细节（ Detail）：保边低通 → 比值图 → 乘性贴回。见文件上方那一节。
     *
     * 这一版把上一版的"建模低通"换成 SPP 自己的核，并把贴回改成 形态：
     *   restored = x3f_gdf_lowpass()             ← 5×5 四相位保边核（§13.3/§13.4）
     *   r        = (Y + eps) / (restored + eps)  ← CreateLumaResidual 线性支
     *   out      = restored · m                  ← **m 乘在 restored 上**（上一版乘在原图上）
     * g = 1 + (base − 1)·R ← 原式（旧拟合：X3F_DETAIL_GMODE=fit）
     * g = 1 → out ≡ Y（原样）；g > 1 → out = Y + (g−1)·(Y − restored)（放大残差）。
     * eps 默认 = +5（16bit 码值的 5 → 归一化 5/65535）；旧版用的 0.02 是它 ×257。*/
    dgain = NULL;
    if ((dlevel > 0 || getenv("X3F_DETAIL_FORCE") != NULL) && !g_detail_done_linear) {
        const double g = x3f_detail_gain(x3f_detail_R(dlevel));
        const double eps = detail_env("X3F_DETAIL_EPS", 5.0 / 65535.0);
        const char *lpm = getenv("X3F_DETAIL_LP");
        const int use_box = (lpm != NULL && lpm[0] == 'b');
        const int use_phase = (int)detail_env("X3F_DETAIL_PHASE", 1.0) != 0;

        if (use_box) {
            /* 旧的多尺度盒式双边 —— 留着做 A/B（X3F_DETAIL_LP=box） */
            const int oct = (int)detail_env("X3F_DETAIL_OCT", 2.0);
            int rr = (int)detail_env("X3F_DETAIL_R", 4.0);
            const double sigr = detail_env("X3F_DETAIL_SIGR", 0.06);
            int k;
            if (rr < 1) rr = 1;
            if (rr > 4096) rr = 4096;
            memset(low, 0, n * sizeof(float));
            nsc = 0;
            for (k = 0; k < oct; k++) {
                const int r2 = (k == 0) ? rr : (rr << k);
                if (k > 0 && r2 > 32) break;
                memcpy(buf, Y, n * sizeof(float));
                detail_bilateral(buf, lp, tmp, w, h, r2, sigr);
                for (i = 0; i < n; i++) low[i] += lp[i];
                nsc++;
            }
            if (nsc < 1) nsc = 1;
            {
                const float inv = 1.0f / (float)nsc;
                for (i = 0; i < n; i++) low[i] *= inv;
            }
        } else {
            x3f_gdf_lowpass(out, stride, bits, Y, low, w, h, use_phase);
            nsc = 4 - (use_phase ? 3 : 4);        /* 相位开启=4，关闭=0（仅调试打印用） */
        }

        /* 逐像素：r = 比值，m = ApplyResidual 的两段式；m 落到 dgain（= lp） */
        dgain = lp;
        sc.low = low; sc.dgain = dgain;
        sc.dgain_g = g; sc.dgain_eps = eps;
        x3f_parallel_for((size_t)h, spat_ratio_row, &sc);
        if (getenv("X3F_DETAIL_DEBUG")) {
            double sd = 0, sm = 0, rmin = 1e9, rmax = -1e9, mmin = 1e9, mmax = -1e9;
            double sy = 0, sl = 0, dmax = 0;
            for (i = 0; i < n; i++) {
                double yv = Y[i], lo = low[i], r, m;
                if (yv <= 0.0 || lo <= 0.0) continue;
                r = (yv + eps) / (lo + eps);
                m = dgain[i];
                sd += fabs(yv - lo); sm += m;
                sy += yv; sl += lo;
                if (fabs(yv - lo) > dmax) dmax = fabs(yv - lo);
                if (r < rmin) rmin = r; if (r > rmax) rmax = r;
                if (m < mmin) mmin = m; if (m > mmax) mmax = m;
            }
            fprintf(stderr,
                    "[detail] level=%d g=%.3f eps=%.6f lp=%s phase=%d nsc=%d "
                    "|Y-low|均值=%.4f 最大=%.4f Y均值=%.4f low均值=%.4f "
                    "r=[%.4f,%.4f] m=[%.4f,%.4f] m均值=%.3f\n",
                    dlevel, g, eps, use_box ? "box" : "spp", use_phase, nsc,
                    sd / (double)n, dmax, sy / (double)n, sl / (double)n,
                    rmin, rmax, mmin, mmax, sm / (double)n);
        }
    }

    /*还原成 RGB：有细节时 out = restored·m（形态），否则原样（行并行）*/
    sc.dgain = dgain;
    x3f_parallel_for((size_t)h, spat_recombine_row, &sc);

    free(Y); free(Cb); free(Cr); free(buf); free(tmp); free(low); free(lp);
    return X3FRENDER_OK;
}


/* ---------------------------------------------------------------- */
/*填充光 = Fill Light（2026-10 第二次；完整管线）
 *
 * 对象：SIGMA Photo Pro 6.8.4 的 （arm64）。
 * 证据与全过程见 。
 *
 *   参数：X3F 的 srX3FillLightColor（有符号有理数，字段 21）→ double F，
 *         滑块 -2.0 … +2.0，步进 0.1。
 *
 * 增益表 FPixelProc2::GenerateRetinexGainLUT：
 *       k = 1.33·F
 *       s = 1 / (1024·(|k| + 0.0625)^0.25)
 *       lut[m] = round(256·exp(k·tanh(s·m)))          m ∈ [-32768, 32767]
 *   每像素：gain = lut[(int16)map]（负下标按 uint16 回绕），
 *          线性域三通道同乘（SPP 里是 (像素·gain) >> 8 的 Q8 定点）。
 *
 *   map 在**半分辨率**上算（MakeRetinexMap：宽高各 >>1），由
 *   GenerateRetinexLogGain(0x128104) 的四个 worker + 两次 PyramidFilter 组成。
 *   括号里是 dylib 内现成的日志字符串，直接印证每一步的语义：
 *
 *     1) XDoRetinexProcess1「log compress luma」
 *          v = (0.25·R + 0.65·G + 0.10·B)·(ISO/100)     ← 只是一个线性亮度
 *          v = clamp(v, 0, 262143)
 *          A = round(1024·ln(v + 30))                    ← 16 位定点的对数压缩
 *     2) PyramidFilter「PyramidFilter(pLogLum)」→ B = 多尺度低通(A)
 *     3) XDoRetinexProcess2「adjust the mean LogLum and calculate variance」
 *          k0  = (mean(A) + 6761.44)/2 + 8               ← 整幅的「锚点」
 *          A  -= k0、B -= k0（都就地改）
 *          var = clamp(((A-B)^2) >> 10, 0, 32767)
 *          PyramidFilter「PyramidFilter(pVar)」→ var = 多尺度低通(var)
 *     4) XDoRetinexProcess3「convert variance to Standard Deviation」
 *          C = 32·sqrt(max(var,0) + 64)
 *     5) XDoRetinexProcess4「generate z-score」
 *          map = k0 - A + (32·sqrt(mean(C)))·(A-B)/C
 *
 *   即：**整幅的「负对数亮度（相对锚点）」 + 一个局部 z-score**。
 *   前者给暗部正 map（提亮）、给亮部负 map（压暗）；后者是 Retinex 的那一半 ——
 *   比局部均值亮的像素再加一点、暗的再减一点，所以提亮暗部的同时
 *   局部对比没有被压平。
 *
 * PyramidFilter：2×2 盒式降采样建层到短边 ≤2（每层记均值），
 *   再从**最粗**一层往细走；第 i 层（i 从最粗数起）权重
 *       w_i = 0.9 · powf(0.65, i)            ← 0.65 的 i 次幂，不是 i^0.65
 *   融合写进更细的那一层：
 *       L_{i-1} ← (1-w)·up(L_i) + w·(mean_all/mean_{i-1})·L_{i-1}
 *   最细一层（没有更细的了）与**原始输入**混合：
 *       out ← (1-w)·up(L_0) + w·输入
 *   系数全为正且 ≤ 0.9 → 整条融合是**真低通**。旧版把权重读成 i^0.65
 *   （i≥2 时 >1），推出「Nyquist 有 11 倍增益」的怪结论，根源就是这里。
 *
 *   本节推翻旧实现的两处：① map 不是「局部对数比值」，而是
 *   「锚点 − 对数亮度 + 局部 z-score」；② 空间算子不是「两级盒式模糊」，
 *   而是这条多尺度低通 —— 尺度由图像自己决定，不再需要 X3F_FL_MAPSCL
 *   这类定标常数。
 */
#define X3F_FL_K       1.33      /*k = 1.33·F（147AE148）*/
#define X3F_FL_ANCHOR  6761.44   /*1024·ln(738.2)，SPP 硬编码（5FF7CA94）*/
#define X3F_FL_VSCALE  2048.0    /*线性 → SPP 内部尺度。里 v = luma·(CaptureISO/100)：
                                  * F23 亮度按固定参考归一化，再乘 ISO/100。
                                  * 这一档是拿 DP2Q0337（ISO 320）定的：**符号**对拍
                                  * （F=±1 时高光该跟暗部同向）在 2048 附近翻正，
                                  * 4095 时高光会反向（−1 反而推亮高光 = 蒙磨玻璃）。
                                  * 换算过来 K ≈ 2048/(320/100) ≈ 640，即 F23 白点
                                  * 大约是 640（待另一张不同 ISO 的 SPP 参考图确认）。
                                  * 可用 X3F_FL_VSCALE 覆盖。 */
#define X3F_FL_MAPGAIN 0.65      /* map 的整体系数：**实证值，来源未解**。
                                  * 2026-10-13 重定：JPEG 编码档提到 4:4:4/质量 1.0
                                  * 之后（编码不再抹掉细节），在 DP2Q0337 的 SPP
                                  * 五档 + DP0Q0995(FL=−2) 上重扫，0.65 明显好于
                                  * 0.80：平均影调误差 4.87 → 3.91 灰阶，且 +1/+2
                                  * 两档从 6.5/8.2 收到 3.8/6.9；纹理也从系统性偏锐
                                  * （1.07~1.42）回到 0.86~0.96。0.5 又反过来变差
                                  * （9.7），所以 0.6~0.7 是这个口径下的甜点。
                                  * 旧的 0.80 是在 0.95 档编码下拟合的。
                                  * 读出来的公式到 LUT 为止是闭合的，但直接按公式
                                  * 算出来的 map 比 SPP 实际用的大 ~1.4 倍（暗部过亮、
                                  * 高光压得过狠）。两种独立估计都指向同一个系数：
                                  * ① 同一批像素上配对比较"每一档提升量"：0.85 时
                                  *    F=-2/-1/+1/+2 四档的最差箱偏差都在 ±25% 以内，
                                  *    而 0.70 时最暗那几箱只有 0.33~0.61（差 2~2.5 倍）；
                                  * ② 逐像素把 输出反解回 map，回归斜率 ≈0.7~0.8
                                  *    （暗部因增益超出 tanh 可逆区间被删失，偏小）。
                                  * 待查：是不是 F23 的输入不是纯线性、或者 map 在
                                  * GenerateRetinexLogGain 之后（MakeRetinexMap 的
                                  * ResizeImage / F23 compose）被重新定标过一次。 */
#define X3F_FL_MAXLEV  32

/*增益表：lut[m + 32768] 对应 map = m（m ∈ [-32768, 32767]）*/
static double fill_env(const char *name, double dflt);   /* 定义在下面 */
/* ===================================================================
 * apply 中间结果缓存（2026-10-22，拖滑杆提速）
 *
 * 量账（X3F_APPLY_TIME，1808×1206 预览帧）：
 *   Fill Light 拖一格 = build_fill_gain 全重算 ≈ 6 s（几乎单线程），
 *   但其中步骤 1a–4（降采样→tone→对数→低通→金字塔→方差）**完全不依赖
 *   主滑杆 F**；Detail 的 gdf 低通不依赖 detail 滑杆；NR 的辉度段不依赖
 *   色度滑杆。这些"参数无关基座"每帧都重算，就是拖杆慢一拍的根源。
 *
 * 设计：
 *   · 单槽缓存，键 = 图像内容指纹 + 参数位。**指纹而不是裸指针**：
 *     Swift 侧的 lin 是 malloc 出来又 free 的，地址复用会让"同指针"
 *     变成假命中，指纹（FNV 采样 ~3 万个 float）杜绝这一类错位。
 *   · 命中后 memcpy 到调用方私有缓冲——缓存内的缓冲**永不外借**，
 *     随便驱逐都不用加读锁。导出多路并发只是互相挤掉（不提速），
 *     不会崩；预览是串行的，稳定命中。
 *   · 只缓存 ≤ APPLY_CACHE_MAX_PIX 像素的帧：预览口径（2712 长边
 *     ≈ 4.9M）命中，全分辨率导出（19.6M）一次性不缓存不浪费内存。
 *   · X3F_APPLY_CACHE=0 总开关（A/B 逐字节对照用）。
 *   · 前提：X3F_FL_* / X3F_NR_* 等环境变量在进程内不变（App 与 lab
 *     工具都是启动时设一次）；键里已含随滑杆变化的那些（lmean 等）。
 * =================================================================== */
#define APPLY_CACHE_MAX_PIX 8000000

#if defined(_WIN32)
typedef volatile LONG apply_mtx_t;
static void apply_mtx_lock(apply_mtx_t *m) {
    while (InterlockedCompareExchange(m, 1, 0) != 0) Sleep(0);
}
static void apply_mtx_unlock(apply_mtx_t *m) { InterlockedExchange(m, 0); }
#else
#include <pthread.h>
typedef pthread_mutex_t apply_mtx_t;
static void apply_mtx_lock(apply_mtx_t *m) { pthread_mutex_lock(m); }
static void apply_mtx_unlock(apply_mtx_t *m) { pthread_mutex_unlock(m); }
#define APPLY_MTX_INIT PTHREAD_MUTEX_INITIALIZER
#endif

static int apply_cache_on(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("X3F_APPLY_CACHE");
        v = (e != NULL && *e != '\0' && *e == '0') ? 0 : 1;
    }
    return v;
}

/* 内容指纹：FNV-1a 采样（头 4096 + 每隔 997 + 尾 4096 个 float）。
 * p 是 float 序列（n 个），喂的是每个 float 的位型。 */
static uint64_t apply_fp(const float *p, size_t n)
{
    uint64_t h = 1469598103934665603ULL;
    size_t i;
    if (p == NULL || n == 0) return h;
    for (i = 0; i < n && i < 4096; i++) {
        uint32_t b; memcpy(&b, &p[i], 4);
        h = (h ^ b) * 1099511628211ULL;
    }
    for (i = 4096; i < n; i += 997) {
        uint32_t b; memcpy(&b, &p[i], 4);
        h = (h ^ b) * 1099511628211ULL;
    }
    for (i = (n > 4096) ? n - 4096 : 0; i < n; i++) {
        uint32_t b; memcpy(&b, &p[i], 4);
        h = (h ^ b) * 1099511628211ULL;
    }
    return h;
}

static uint64_t apply_fp_d(double v)
{
    uint64_t b; memcpy(&b, &v, 8);
    return b;
}

/* 大帧 NR 缓存帽（像素数）。默认 24M px —— 覆盖 Quattro 原生 3616×5424
 * ≈ 19.6M（槽成本 ≈ 288MB，只在 NR 开着且真渲过原生大帧才落地）。
 * X3F_NR_CACHE_MAX_PIX=0 关掉、=N 调整；比 APPLY_CACHE_MAX_PIX 小的帧
 * 走原有的辉度/色度分级槽，不走这条。 */
static int nr_cache_max_pix(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("X3F_NR_CACHE_MAX_PIX");
        v = (e != NULL && *e != '\0') ? atoi(e) : 24000000;
        if (v < 0) v = 0;
    }
    return v;
}

/* 单槽缓存。k1..k3 的含义随用途而定；w/h/n 是缓冲形状（n = float 个数）。
 * d1..d3 是随缓冲一起存取的标量伴随值（fill 基座存 meanA/meanC/scale；
 * 不用的调用方传 NULL/0）。 */
typedef struct {
    int valid;
    uint64_t k1, k2, k3;
    int w, h;
    size_t n;
    double d1, d2, d3;
    float *buf;
} apply_slot_t;

static int apply_slot_get(apply_slot_t *s, apply_mtx_t *mtx,
                          uint64_t k1, uint64_t k2, uint64_t k3,
                          int w, int h, size_t n, float *dst,
                          double *d1, double *d2, double *d3)
{
    int hit = 0;
    if (dst == NULL || n == 0) return 0;
    apply_mtx_lock(mtx);
    if (s->valid && s->buf != NULL && s->k1 == k1 && s->k2 == k2 && s->k3 == k3
        && s->w == w && s->h == h && s->n == n) {
        memcpy(dst, s->buf, n * sizeof(float));
        if (d1) *d1 = s->d1;
        if (d2) *d2 = s->d2;
        if (d3) *d3 = s->d3;
        hit = 1;
    }
    apply_mtx_unlock(mtx);
    return hit;
}

static void apply_slot_put(apply_slot_t *s, apply_mtx_t *mtx,
                           uint64_t k1, uint64_t k2, uint64_t k3,
                           int w, int h, size_t n, const float *src,
                           double d1, double d2, double d3)
{
    if (src == NULL || n == 0) return;
    apply_mtx_lock(mtx);
    if (s->n != n) {
        float *nb = (float *)malloc(n * sizeof(float));
        if (nb == NULL) { apply_mtx_unlock(mtx); return; }
        free(s->buf);
        s->buf = nb; s->n = n;
    }
    memcpy(s->buf, src, n * sizeof(float));
    s->k1 = k1; s->k2 = k2; s->k3 = k3;
    s->w = w; s->h = h; s->d1 = d1; s->d2 = d2; s->d3 = d3;
    s->valid = 1;
    apply_mtx_unlock(mtx);
}

/* ---- 槽位：fill 基座（A/B/C）、detail 低通、NR 两级、NR 整链（大帧） ---- */
static apply_slot_t g_fl_slot_a, g_fl_slot_b, g_fl_slot_c;
#ifndef _WIN32
static apply_mtx_t g_fl_lock   = APPLY_MTX_INIT;
static apply_mtx_t g_det_lock  = APPLY_MTX_INIT;
static apply_mtx_t g_nr1_lock  = APPLY_MTX_INIT;
static apply_mtx_t g_nr2_lock  = APPLY_MTX_INIT;
static apply_mtx_t g_nrf_lock  = APPLY_MTX_INIT;
#else
static apply_mtx_t g_fl_lock, g_det_lock, g_nr1_lock, g_nr2_lock, g_nrf_lock;
#endif
static apply_slot_t g_det_slot, g_nr1_slot, g_nr2_slot, g_nrf_slot;

/* ---- NR 整链缓存池（2026-11-04）----
 *
 * 【病根实测】1:1 放大态下调一个跟 NR 无关的滑杆（曝光/对比度/曲线），
 * CPU 立刻满核几秒。`X3F_SPP_PROFILE` + `X3F_APPLY_TIME` 拆开看，全部是
 * `[apply] nr 0.9s (5424x512)` —— 每条带都在把整条双边滤波从头重跑。
 *
 * 原因不是"没缓存"，而是**槽不够**：
 *   · 整链槽 `g_nrf_slot` 只有一个，且原来只在 `w*h > 8M` 才启用；
 *   · 1:1 的行带是 5424×512 = 2.78M、视口瓦片 3053×2040 = 6.23M，
 *     双双落在 8M 以下 → 走的是辉度/色度**分级单槽**（`g_nr1_slot`）；
 *   · 8 条带内容各不相同、尺寸还和瓦片不同 → 每次 put 都 `free + malloc`
 *     （74.8MB 抖动），且下一次 get 必然 miss。
 * 于是「调一次参」= 瓦片 2.16s + 8 带 × 0.9s ≈ 9s 全核空转。
 *
 * 这里给块级帧（2M..8M px，正是 1:1 的带与瓦片）开一个**小池**：按
 * (内容指纹, 形状) 精确匹配，命中就直接 memcpy 出来。同一块第二次调参
 * 就只剩 chrmed/spatial 的几十毫秒。
 *
 * 内存：槽按需分配（put 过才 malloc），6 槽 × 2.78M×3×4B ≈ 200MB 封顶。
 * 大帧（>8M，导出/整幅）仍走原来的单槽，不参与池化 —— 否则 19.6M 一块
 * 就是 235MB，6 槽直接 1.4GB。
 * 纯缓存，不改变任何输出像素。 */
#define NR_F_MIN_PIX 2000000
/* 槽数必须 ≥ 一次填充可能touch的块数，否则顺序访问会让轮转淘汰变成
 * 「100% miss」：6 槽装 8 条带，第二轮从带 0 扫回来刚好每条都被上一轮
 * 自己挤掉（实测 2nd pass 反而更慢）。带级惰性后实际只有「可见 1~2 带 +
 * 上下预取 + 视口瓦片」≈ 4~6 块活跃，8 槽留出余量。 */
#define NR_F_POOL 8
static apply_slot_t g_nrf_pool[NR_F_POOL];
static int g_nrf_pool_rr = 0;

static int apply_pool_get(apply_slot_t *pool, int np, apply_mtx_t *mtx,
                          uint64_t k1, uint64_t k2, uint64_t k3,
                          int w, int h, size_t n, float *dst)
{
    int i;
    for (i = 0; i < np; i++)
        if (apply_slot_get(&pool[i], mtx, k1, k2, k3, w, h, n, dst,
                           NULL, NULL, NULL))
            return 1;
    return 0;
}

static void apply_pool_put(apply_slot_t *pool, int np, apply_mtx_t *mtx,
                           int *rr, uint64_t k1, uint64_t k2, uint64_t k3,
                           int w, int h, size_t n, const float *src)
{
    int i, slot = -1;
    /* 同键覆盖优先（幂等），否则轮转淘汰。查重不加锁：最坏是两个线程
     * 同时写同一块，结果仍是同一份像素，只是多占一个槽。 */
    for (i = 0; i < np; i++) {
        if (pool[i].valid && pool[i].k1 == k1 && pool[i].k2 == k2
            && pool[i].k3 == k3 && pool[i].w == w && pool[i].h == h
            && pool[i].n == n) { slot = i; break; }
    }
    if (slot < 0) { slot = *rr; *rr = (*rr + 1) % np; }
    apply_slot_put(&pool[slot], mtx, k1, k2, k3, w, h, n, src, 0, 0, 0);
}

static void fill_gain_lut(double F, float *lut)
{
    /* 标定开关 X3F_FL_KSCALE：整体缩放 k = 1.33·F。
     * 用它扫"曲线够不够陡"：k 变大 = 负档压得更狠、局部斜率更大
     * （实测 FL=−2 在最暗档压得比我们狠、且中间调局部对比比我们高
     * 25%，2026-10-13）。默认 1.0 = 出来的原值。*/
    const double k = X3F_FL_K * fill_env("X3F_FL_KSCALE", 1.0) * F;
    const double s = 1.0 / (1024.0 * pow(fabs(k) + 0.0625, 0.25));
    int n;
    for (n = -32768; n < 32768; n++)
        lut[n + 32768] = (float)exp(k * tanh(s * (double)n));
}

/* SPP 6.9.0 GenerateRetinexGainLUT：Q8 整数表，平台后冻结。
 * 与直调结果见 WorkBuddy 的§3.3。*/
static void fill_gain_lut_69(double F, float *lut)
{
    const double k = X3F_FL_K * F;
    const int scale = 256;
    int n, cap, vp = 0, vn = 0;
    double s;

    if (k == 0.0) {
        for (n = 0; n < 65536; n++) lut[n] = 1.0f;
        return;
    }

    s = 1.0 / (1024.0 * pow(fabs(k) + 0.0625, 0.25));
    cap = (int)(exp(fabs(k)) * (double)scale);
    if (cap < 1) cap = 1;
    lut[32768] = 1.0f;

    for (n = 1; n < 32768; n++) {
        if (vp != cap && vn != cap) {
            double ep = exp(k * tanh(s * (double)n));
            vp = (int)(ep * (double)scale + 0.5);
            vn = (int)((1.0 / ep) * (double)scale + 0.5);
            if (vp < 1) vp = 1;
            if (vn < 1) vn = 1;
        }
        lut[32768 + n] = (float)vp / (float)scale;
        lut[32768 - n] = (float)vn / (float)scale;
    }
}

/* 定标/调试用的环境变量（正数才认） */
static double fill_env(const char *name, double dflt)
{
    const char *e = getenv(name);
    if (e != NULL && e[0] != '\0') {
        double v = atof(e);
        if (v > 0.0) return v;
    }
    return dflt;
}

/* 同 fill_env，但允许 0（"明确关掉"和"没设"要区分开）。 */
static double fill_env0(const char *name, double dflt)
{
    const char *e = getenv(name);
    if (e != NULL && e[0] != '\0') return atof(e);
    return dflt;
}

/*2×2 盒式降采样，越界钳到边缘像素（ Downscale2x2）；顺带给该层均值*/
static void fl_down2(const float *s, int sw, int sh,
                     float *d, int dw, int dh, double *mean_out, int exact)
{
    double acc = 0.0;
    int x, y;
    for (y = 0; y < dh; y++) {
        int y0 = 2 * y, y1 = y0 + 1;
        if (y1 >= sh) y1 = sh - 1;
        for (x = 0; x < dw; x++) {
            int x0 = 2 * x, x1 = x0 + 1;
            double v;
            if (x1 >= sw) x1 = sw - 1;
            v = 0.25 * (s[(size_t)y0 * sw + x0] + s[(size_t)y0 * sw + x1] +
                        s[(size_t)y1 * sw + x0] + s[(size_t)y1 * sw + x1]);
            if (exact) v = floor(v);
            d[(size_t)y * dw + x] = (float)v;
            acc += v;
        }
    }
    if (mean_out)
        *mean_out = ((double)dw * dh > 0.0) ? acc / ((double)dw * dh) : 0.0;
}

/* 6.9.0 的 Retinex map 在每一步都走 16 位整数平面。保持浮点存储便于复用
 * 后面的路径，但在每个可观察的整数边界处按 方式取整。*/
static void fl_round_i16(float *p, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++) {
        double v = floor((double)p[i] + 0.5);
        if (v < -32768.0) v = -32768.0;
        if (v > 32767.0) v = 32767.0;
        p[i] = (float)v;
    }
}

/* 2× 上采样（双线性、像素中心对齐）。对应 SmoothDoubleBlend 里的插值 ——
 * 那个内核是「[1,2,1]/4 插值 + 相位相关的 [25,9,-1,-1]/32 轻微锐化」，
 * 对多尺度低通的影响只有百分之几，这里用双线性等价替代。 */
static void fl_up2(const float *s, int sw, int sh, float *d, int dw, int dh)
{
    int x, y;
    if (sw < 2 || sh < 2) {                      /* 退化：直接复制 */
        for (y = 0; y < dh; y++)
            for (x = 0; x < dw; x++)
                d[(size_t)y * dw + x] = s[0];
        return;
    }
    for (y = 0; y < dh; y++) {
        double fy = ((double)y + 0.5) * sh / dh - 0.5;
        int y0 = (int)floor(fy);
        double ty = fy - y0;
        if (y0 < 0) { y0 = 0; ty = 0.0; }
        if (y0 > sh - 2) { y0 = sh - 2; if (ty > 1.0) ty = 1.0; }
        for (x = 0; x < dw; x++) {
            double fx = ((double)x + 0.5) * sw / dw - 0.5;
            int x0 = (int)floor(fx);
            double tx = fx - x0, a, b, c, e;
            if (x0 < 0) { x0 = 0; tx = 0.0; }
            if (x0 > sw - 2) { x0 = sw - 2; if (tx > 1.0) tx = 1.0; }
            a = s[(size_t)y0 * sw + x0];
            b = s[(size_t)y0 * sw + x0 + 1];
            c = s[(size_t)(y0 + 1) * sw + x0];
            e = s[(size_t)(y0 + 1) * sw + x0 + 1];
            d[(size_t)y * dw + x] = (float)
                ((a + (b - a) * tx) * (1.0 - ty) + (c + (e - c) * tx) * ty);
        }
    }
}

/*PyramidFilter：多尺度低通（out 可以和 src 同址）*/
static int fl_pyramid(const float *src, int w, int h, float *out, int exact)
{
    int lw[X3F_FL_MAXLEV], lh[X3F_FL_MAXLEV], m = 0, i, j;
    double lmean[X3F_FL_MAXLEV], meanall = 0.0;
    float *lev[X3F_FL_MAXLEV], *cur = NULL, *up = NULL;
    const float *pin = src;
    int cw = w, ch = h, npix = w * h;

    if (w < 2 || h < 2) return 0;               /* SPP 在这两种尺寸下直接跳过 */

    /* 建层：降到「上一层宽 ≥3 且这一层高 >2」不成立为止（与 SPP 同） */
    while (m < X3F_FL_MAXLEV) {
        int dw = (cw + 1) / 2, dh = (ch + 1) / 2;
        float *d = (float *)malloc((size_t)dw * dh * sizeof(float));
        if (d == NULL) goto fail;
        fl_down2(pin, cw, ch, d, dw, dh, &lmean[m], exact);
        lev[m] = d; lw[m] = dw; lh[m] = dh; m++;
        if (!(cw >= 3 && dh > 2)) break;
        pin = d; cw = dw; ch = dh;
    }

    for (j = 0; j < npix; j++) meanall += src[j];
    meanall /= (double)npix;

    up = (float *)malloc((size_t)w * h * sizeof(float));
    if (up == NULL) goto fail;

    /* 从最粗一层往细融合：w_i = 0.9·0.65^i（i 从最粗数起） */
    cur = lev[m - 1];
    for (i = 0; i < m; i++) {
        int base = m - 1 - i;
        double wgt = 0.9 * pow(0.65, (double)i);
        if (exact) wgt = floor(256.0 * wgt + 0.5) / 256.0;
        if (base > 0) {
            int dw = lw[base - 1], dh = lh[base - 1];
            float *dstl = lev[base - 1];
            fl_up2(cur, lw[base], lh[base], up, dw, dh);
            for (j = 0; j < dw * dh; j++)
                dstl[j] = (float)((1.0 - wgt) * up[j] +
                                  wgt * (meanall / lmean[base - 1]) * (double)dstl[j]);
            if (exact) fl_round_i16(dstl, (size_t)dw * dh);
        } else {
            fl_up2(cur, lw[0], lh[0], up, w, h);
            for (j = 0; j < npix; j++)
                out[j] = (float)((1.0 - wgt) * up[j] + wgt * (double)src[j]);
            if (exact) fl_round_i16(out, (size_t)npix);
        }
        free(cur);
        cur = (base > 0) ? lev[base - 1] : NULL;
    }
    free(up);
    return 1;

fail:
    free(up);
    for (i = 0; i < m; i++) free(lev[i]);
    return 0;
}

/* 实验用：固定半径的可分离盒式均值（tmps 至少 w*h）。X3F_FL_LMEANR 用它
 * 替换 fl_pyramid 得到的 B。之所以要这个开关：dump 出来的 B 在手掌这种
 * 亮物体附近只到全图均值的量级（半径几百像素），A−B 于是变成"全局对比项"，
 * 亮物体边缘一圈没有邻域压住，正档 Fill Light 就出现粉白描边。
 * 半径按 **map 像素** 算（map 是半分辨率），全分辨率半径是它的两倍。 */
static void fl_boxmean1(const float *src, int w, int h, int r,
                        float *tmp, float *out)
{
    int x, y;
    double inv = 1.0 / (2.0 * (double)r + 1.0);
    for (y = 0; y < h; y++) {
        const float *row = src + (size_t)y * w;
        float *dst = tmp + (size_t)y * w;
        double acc = 0.0;
        for (x = -r; x <= r; x++)
            acc += row[x < 0 ? 0 : (x >= w ? w - 1 : x)];
        for (x = 0; x < w; x++) {
            double add = row[x + r + 1 >= w ? w - 1 : x + r + 1];
            double sub = row[x - r < 0 ? 0 : x - r];
            dst[x] = (float)(acc * inv);
            acc += add - sub;
        }
    }
    for (x = 0; x < w; x++) {
        double acc = 0.0;
        for (y = -r; y <= r; y++)
            acc += tmp[(y < 0 ? 0 : (y >= h ? h - 1 : y)) * w + x];
        for (y = 0; y < h; y++) {
            double add = tmp[(y + r + 1 >= h ? h - 1 : y + r + 1) * w + x];
            double sub = tmp[(y - r < 0 ? 0 : y - r) * w + x];
            out[(size_t)y * w + x] = (float)(acc * inv);
            acc += add - sub;
        }
    }
}

/* B（局部均值）与 V（局部方差）共用的低通。
 * X3F_FL_LMEANR=R（map 像素）> 0 时用固定半径盒式均值（默认跑两遍 =
 * 三角核），否则走原来的 fl_pyramid。V 这里也必须跟着切：金字塔把方差
 * 抹到接近全图 RMS，噪声大的暗区 C 被抬得过高，局部项（scale·(A−B)/C）
 * 在那里被稀释，亮物体旁边的暗区就压不住——右侧橙色光带就是这么来的。 */
static int fl_lowpass(const float *src, int w, int h, float *out, double r,
                      int exact)
{
    if (r >= 1.0) {
        int rad = (int)(r + 0.5), pass, npass = 2;
        const char *lmp = getenv("X3F_FL_LMEANP");
        float *tmp;
        if (lmp != NULL && lmp[0] != '\0') npass = atoi(lmp);
        if (npass < 1) npass = 1;
        tmp = (float *)malloc((size_t)w * h * sizeof(float));
        if (tmp == NULL) return 0;
        fl_boxmean1(src, w, h, rad, tmp, out);
        for (pass = 1; pass < npass; pass++)
            fl_boxmean1(out, w, h, rad, tmp, out);
        free(tmp);
        return 1;
    }
    return fl_pyramid(src, w, h, out, exact);
}

static float *build_fill_gain(const float *lin, int w, int h,
                              double F, double expo,
                              int tune, double tune_zred,
                              double tune_lmean, double tune_msat,
                              double tune_mapgain, double tune_k0off,
                              double tune_zgain,
                              uint64_t lin_fp)
{
    int down, gw, gh, x, y, i, np;
    int sd, sw, sh, cw, ch;
    int use69;
    double sx, sy, canon, amax;
    float *A = NULL, *B = NULL, *V = NULL, *C = NULL, *lut = NULL, *gain = NULL;
    int16_t *mapbuf = NULL;
    double meanA = 0.0, meanC = 0.0, k0 = 0.0, scale = 0.0, vscale, fgam, mapgain, vmax, zgain, fblack;
    double mshape, logadd;
    double vdm = 0.0;          /* 显示域均亮（TONEDIN 时用于定 vscale），调试用 */
    int tonedin;
    double luma_sum = 0.0;
    double cfloor;
    int nog;
    /* 6.9.0 的精确 Retinex 流程先作为可验证路径保留，默认仍走已经按
     * 当前渲染器输入域标定过的兼容路径。X3F_FL_69=1 启用 6.9 原始流程。
     *
     * 不能直接默认开启的原因：MakeRetinexMap 接收的是 SPP Stage1 之后的
     * 传感器域 RGB，而本渲染器送进来的是 scene-linear sRGB。公式本身已经
     * 复刻，但输入域换算仍未闭合；直接切换会把 DP2Q0337 FL=+2 明显提亮。 */
    use69 = fill_env0("X3F_FL_69", 0.0) >= 0.5;
    /* 这些是早期为了补输入域/低通差异加的标定旋钮。精确路径默认全部
     * 归零；环境变量仍可显式覆盖，继续做实验。 */
    double eff_zred  = fill_env0("X3F_FL_HLZRED",
                                 use69 ? 0.0 : (tune ? tune_zred : 0.95));
    double eff_lmean = fill_env0("X3F_FL_LMEANR",
                                 tune ? tune_lmean : 0.0);
    double eff_msat  = fill_env0("X3F_FL_MSAT",
                                 use69 ? 0.0 : (tune ? tune_msat : 0.0));
    double eff_mapgain = fill_env0("X3F_FL_MAPGAIN",
                                   use69 ? 1.0 : (tune ? tune_mapgain : 0.75));
    double eff_k0off   = fill_env0("X3F_FL_K0OFF",
                                   use69 ? 0.0 : (tune ? tune_k0off : 0.0));
    (void)expo;   /* 全局曝光已被「锚点 − A」吸收，不再折进引导值 */

    if (eff_zred < 0.0) eff_zred = 0.0;
    if (eff_zred > 1.0) eff_zred = 1.0;
    if (eff_lmean < 0.0) eff_lmean = 0.0;
    if (eff_msat < 0.0) eff_msat = 0.0;
    if (eff_mapgain < 0.05) eff_mapgain = 0.05;
    if (eff_mapgain > 1.5) eff_mapgain = 1.5;

    if (F == 0.0) return NULL;

    /* TONEDIN 默认开（X3F_FL_NOTONEDIN 可关）：SPP 那条链交给 Retinex 的是一张
     * 已过相机色调曲线的图（Stage3 的 16 位归一化输出），不是裸线性。喂裸线性
     * 等于把中间调压平 → 局部项在中尺度变弱 → 那个 0.65 的"map 系数"就是硬顶
     * 这个形状误差的。改显示域输入后它可以回到 1.0。 */
    tonedin = getenv("X3F_FL_TONEDIN") != NULL || getenv("X3F_FL_NOTONEDIN") == NULL;

    /*工作分辨率 = 输入的 1/down。 MakeRetinexMap 固定在**传感器**半分辨率上
     * 算 map（ 起 `lsr w21, w9, #1`），所以整幅导出用 down=2；
     * 对拍时（手里就是半分辨率线性数据）用 X3F_FL_DOWN=1 对齐尺度。
     *
     * 但 Retinex 的半径是**像素**，不是场景比例：预览按 1600 长边解出来时，
     * 同一组 down=2 只得到 600×900 的 map，局部项几乎退化成整体提亮，
     * 预览于是比全分辨率导出亮一档、发灰（2446 的脸和手一眼能看出来）。
     * 所以这里把 map 的**工作分辨率固定到 2712 长边**（= 5424 的一半，
     * 即 5424 输入下原来的 2712×1808），小图先上采样到这条线再算，
     * 算完按比例采样回输入尺寸。全分辨率导出仍是原来的 2712×1808，
     * 逐字节不变。X3F_FL_CANON=0 回到旧的“按输入尺寸降采样”。 */
    canon = fill_env("X3F_FL_CANON", 2712.0);
    amax = (double)(w > h ? w : h);
    down = (int)fill_env("X3F_FL_DOWN", 2.0);
    if (down < 1) down = 1;
    if (w / down < 8 || h / down < 8) down = 1;
    if (canon > 0.0 && amax > 0.0) {
        double k = canon / amax;
        cw = (int)((double)w * k + 0.5);
        ch = (int)((double)h * k + 0.5);
        sd = (int)(amax / canon + 0.5);
        if (sd < 1) sd = 1;
        if (w / sd < 8 || h / sd < 8) sd = 1;
        if (cw < 4) cw = 4;
        if (ch < 4) ch = 4;
    } else {
        sd = down;
        cw = gw = (w + sd - 1) / sd;
        ch = gh = (h + sd - 1) / sd;
    }
    sw = (w + sd - 1) / sd;
    sh = (h + sd - 1) / sd;
    gw = cw;
    gh = ch;
    if (gw < 4 || gh < 4) return NULL;
    /*输入尺度。是 v = luma·(CaptureISO/100)（SPP 内部按固定参考归一化），
     * 但拿三张片子对拍（DP2Q0337 ISO320、DP0Q1013 ISO100、DP2Q2135 ISO100）发现：
     * 拟合出的尺度并不只随 ISO 走，而是跟着**场景平均亮度**走 ——
     *   场景平均线性亮度 ≈ 0.073 / 0.114 / 0.430  →  拟合尺度 ≈ 2200 / 3000 / 4100
     * 这里用 vscale = 5584·mean^0.35（过这三点的经验拟合，钳在 1200…8000）同时照顾三张。
     * **这是实证式，不是出来的**；X3F_FL_VSCALE 可以强制覆盖。
     * 待查：SPP F23 预处理里的 CalcMedianOfHLPsGain（每图一个曝光归一化因子）
     * 很可能就是"每张图不一样"的来源。 */
    vscale = fill_env("X3F_FL_VSCALE", 0.0);
    fgam = fill_env("X3F_FL_GAMMA", 1.0);
    /* 显示域输入（TONEDIN，默认开）下不再需要那个经验系数：拟合值是 1.0。
     * 保留非 TONEDIN 的老值 0.65，只给 A/B 复现用。 */
    /* TONEDIN 路径的整体系数：1.0 会把暗部/中间调的 A 依赖拉得过陡
     * （2446 的阳光橙木被推到 ≈+3.1 EV，SPP 只有 ≈+2.2 EV；同图亮部却
     * 又靠 HLZRED 压回去）。0.75 让 A 越低提得越多的斜率回到 量级，
     * 0337 的 F=+2/−1/−2 也一并更接近。 */
    mapgain = use69 ? fill_env("X3F_FL_MAPGAIN", 1.0)
                    : (tune ? eff_mapgain
                            : fill_env("X3F_FL_MAPGAIN",
                                       tonedin ? 0.75 : X3F_FL_MAPGAIN));
    vmax = fill_env("X3F_FL_VMAX", 0.0);
    zgain = fill_env("X3F_FL_ZGAIN", use69 ? 1.0 : (tune ? tune_zgain : 1.0));
    mshape = fill_env("X3F_FL_MSHAPE", 1.0);
    logadd = fill_env("X3F_FL_LOGADD", 30.0);
    fblack = fill_env("X3F_FL_BLACK", 0.0);
    /*C 的下限。**按是 32·sqrt(64)=256**，但实测 256 会让局部项在平滑区
     * 被放大 2.76 倍（= scale/C），把高光的局部对比"熨平"——F=-1 时我们的高光
     * 局部对比掉 20%，而 SPP 是涨 40%（像蒙了一层磨玻璃的来源）。
     * 512 时：高光掉 12%，暗部各档的配对比值也从 0.87~0.91 抬到 0.93~1.02。
     * 最可能的根因是我们的局部方差（C）系统性偏小（B 太钝 / 金字塔权重），
     * 用这个下限先顶住；要深挖就改金字塔那一层。 */
    cfloor = fill_env("X3F_FL_CFLOOR", 512.0);
    nog = getenv("X3F_FL_NOGLOBAL") != NULL;
    np = gw * gh;

    A = (float *)malloc((size_t)np * sizeof(float));
    B = (float *)malloc((size_t)np * sizeof(float));
    V = (float *)malloc((size_t)np * sizeof(float));
    C = (float *)malloc((size_t)np * sizeof(float));
    if (A == NULL || B == NULL || V == NULL || C == NULL) goto done;

    /* ==== 步骤 1a–4 的缓存查找 ====
     * 这一段只依赖 (lin 内容, w, h, eff_lmean, 环境常量)，与 F / 其余
     * tune 旋钮无关。命中就直接跳到步骤 5（查表 + 上采样）。
     * 键里的 lmean 是 fl_lmean_r 滑杆（影响两处低通半径）；k0off 不进
     * 键——k0 在缓存外用 meanA 重算（见 fl_base_ready 之后）。 */
    if (apply_cache_on() && lin_fp != 0 && np <= APPLY_CACHE_MAX_PIX
        ? (apply_slot_get(&g_fl_slot_a, &g_fl_lock,
                          lin_fp, apply_fp_d(eff_lmean),
                          ((uint64_t)use69 << 1) | (uint64_t)tonedin,
                          w, h, (size_t)np, A,
                          &meanA, &meanC, &scale) &&
           apply_slot_get(&g_fl_slot_b, &g_fl_lock,
                          lin_fp, apply_fp_d(eff_lmean),
                          ((uint64_t)use69 << 1) | (uint64_t)tonedin,
                          w, h, (size_t)np, B, NULL, NULL, NULL) &&
           apply_slot_get(&g_fl_slot_c, &g_fl_lock,
                          lin_fp, apply_fp_d(eff_lmean),
                          ((uint64_t)use69 << 1) | (uint64_t)tonedin,
                          w, h, (size_t)np, C, NULL, NULL, NULL))
        : 0) {
        goto fl_base_ready;
    }

    /* 1a) 半分辨率线性亮度（先只存亮度，因为自适应尺度要用全图均值）。
     * 先在输入上盒式降到 sw×sh，再（需要时）双线性到固定的 cw×ch。
     * 全分辨率导出时 sw==cw、sh==ch，这一步就是逐元素拷贝。 */
    {
        float *S = (float *)malloc((size_t)sw * sh * sizeof(float));
        double sumS = 0.0;
        if (S == NULL) goto done;
        for (y = 0; y < sh; y++) {
            int y0 = y * sd, y1 = y0 + sd;
            if (y1 > h) y1 = h;
            for (x = 0; x < sw; x++) {
                int x0 = x * sd, x1 = x0 + sd;
                double s = 0.0;
                int cnt = 0, yy, xx;
                if (x1 > w) x1 = w;
                for (yy = y0; yy < y1; yy++) {
                    const float *p = lin + ((size_t)yy * w + x0) * 3;
                    for (xx = x0; xx < x1; xx++, p += 3)
                        s += 0.25 * p[0] + 0.65 * p[1] + 0.10 * p[2];
                    cnt += (x1 - x0);
                }
                s = (cnt ? s / cnt : 0.0);
                S[(size_t)y * sw + x] = (float)s;
                sumS += s;
            }
        }
        if (sw == cw && sh == ch) {
            memcpy(A, S, (size_t)np * sizeof(float));
            luma_sum = sumS;          /* 与改之前逐字节一致 */
        } else {
            fl_up2(S, sw, sh, A, cw, ch);
            luma_sum = 0.0;
            for (i = 0; i < np; i++) luma_sum += A[i];
        }
        free(S);
    }

    /* 1b) 输入尺度。
     *
     * 老路（TONEDIN 关）= 线性亮度进对数：**影调是对的**（三张干净参照上误差
     * 4~5 灰阶），但中间调被压平 → 局部项弱 → 中尺度对比只有 0.73~0.81，
     * 也就是那层"朦胧"。它那个 5584·mean^0.35 的尺度就是为影调拟合出来的。
     *
     * 新路（TONEDIN 开）= 显示域亮度进对数：**形状对了**（1070 上 r=4 0.807→0.956、
     * r=16 0.726→1.000，且不再需要 0.65 那个"map 系数"），但整体水平会随图漂
     * （0995 直接暗 27 灰阶）——因为 vscale 只等价于给 map 加一个常数偏移，
     * 而这个偏移该是多少取决于"我们这条替身曲线"和 SPP 那条差多少，逐图不同。
     *
     * 所以这里不再猜一个"倍数"，而是**拿老路的 map 水平当基准**：
     * 两条路的 map 均值（局部项在整幅上近似零均值）分别是
     *    老：MAPGAIN·((ANCHOR − m_old)/2 + 8)
     *    新：       ((ANCHOR − m_new)/2 + 8)
     * 令两者相等解出 m_new，再反解 vscale 使 mean(1024·ln(vscale·disp + 30)) = m_new。
     * 即"形状取新路、水平取老路"。X3F_FL_VSCALE 仍然可以直接覆盖。 */
    {
        double mlv = (np > 0) ? luma_sum / (double)np : 1e-4;
        double vs_old;
        if (!(mlv > 1e-6)) mlv = 1e-6;
        vs_old = 5584.0 * pow(mlv, 0.35);
        if (vs_old < 300.0) vs_old = 300.0;
        if (vs_old > 8000.0) vs_old = 8000.0;
        if (vscale <= 0.0) {
            if (tonedin) {
                double dm = 0.0;
                for (i = 0; i < np; i++) {
                    A[i] = (float)apply_base_tone(A[i] > 0.0 ? (double)A[i] : 0.0, 0.5);
                    dm += A[i];
                }
                dm = (np > 0) ? dm / (double)np : 1e-4;
                vdm = dm;
                if (!(dm > 1e-9)) dm = 1e-9;
                /* vscale = VMEAN · dm^0.75（dm = 增益前的显示域均亮）。
                 * 三点标定（FL=−2、锐度 0.70，对 SPP 同档参照扫 mid 过零点）：
                 *   DP0Q1070  dm 0.3823 → 最优 1763   （3620·0.3823^0.75 = 3612）
                 *   DP0Q0995  dm 0.1877 → 最优 1036   （3620·0.1877^0.75 = 3632）
                 *   DP0Q1013  dm 0.2221 → 最优  895   （3620·0.2221^0.75 = 1171 ← 高 31%）
                 * 前两点吻合到 0.5%，1013 高 31%：说明"水平由图像均亮决定"这条
                 * 只能算近似 —— 真正的水平取决于**我们这条替身曲线和 SPP 那条
                 * Stage3 曲线差多少**，那个差是逐图不同的。所以这条律先顶着用
                 * （1070/0995 基本归零，1013 仍偏亮 ~15 灰阶），要根治得回去读
                 * priStage3 那一级：交给 Retinex 的那张 Flt32RGB 到底是谁产出的。
                 * "map 均值对齐老路"那条也试过：形状不同，均值对上但亮度差 30 灰阶。 */
                /* 这个倍数每次动基础影调都要重对：亮段锚点按 SPP 改对（index 24…31）
                 * 之后基础整体变暗，水平线跟着偏低（1070 上 mid 从 −2.3 掉到 −18.1），
                 * 重扫得 5300（1070：mid +0.2、影调误差 3.00，优于改锚点前的 4.54）。
                 * 上次的值：3620（标定表前）→ 3780（标定表后）。 */
                vscale = fill_env("X3F_FL_VMEAN", 5300.0) * pow(dm, 0.75);
            } else {
                vscale = vs_old;
            }
            if (vscale < 300.0) vscale = 300.0;
            if (vscale > 8000.0) vscale = 8000.0;
        } else if (tonedin) {
            for (i = 0; i < np; i++)
                A[i] = (float)apply_base_tone(A[i] > 0.0 ? (double)A[i] : 0.0, 0.5);
        }
    }

    /* 1c) 亮度 → 对数压缩 A = 1024·ln(v+30)（SPP 走 18 位入、16 位出的定点表） */
    /* 标定用开关 X3F_FL_TONEDIN：进对数之前先把亮度过一遍**基础曲线**
     * （也就是「显示域亮度」而不是线性光）。假设来自 2026-10-13 的反解：
     * 把 SPP 五档输出反解成它真正的 map 之后，我们的 map 在亮部斜率是它的
     * 4~5 倍、过零点也低 3 倍 —— 那正是「输入被压过」的形状，而压它的应该
     * 就是相机色调曲线（F23 交进 Retinex 的是 Stage3 之后的图，不是裸线性）。 */
    for (y = 0; y < gh * gw; y++) {
        double v = A[y];
        {
            /*v 到底是「线性亮度」还是「已编码亮度」还没定论：
             * 若是已编码（gamma），对数压缩后的动态范围会小得多 ——
             * 用 X3F_FL_GAMMA=2.2 试验（默认 1 = 线性）。 */
            if (fgam != 1.0 && v > 0.0) v = pow(v, 1.0 / fgam);
            v = v * vscale + fblack;   /*fblack：黑电平（X3F_FL_BLACK），传感器域不是从 0 起*/
            /* SPP 那边的 v 是传感器域（白点 ~4095，过曝就顶住）；我们的线性数据
             * 在高光处可以远超白点。X3F_FL_VMAX 用来试"顶白点"这个假设。 */
            if (vmax > 0.0 && v > vmax) v = vmax;
            if (!(v > 0.0)) v = 0.0;
            if (v > 262143.0) v = 262143.0;
            /*标定开关 X3F_FL_LOGADD：对数压缩里的常数项（是 +30）。
             * 它的副作用是**暗端被压平**：v→0 时 A 趋于 1024·ln(30) 而不是 −∞，
             * 于是 map = k0−A 的峰值不在最暗处、而是落进低中间调 ——
             * 这正是"负档把暗部局部对比压掉"（gain 随亮度非单调）的来源。 */
            if (use69) {
                double q = floor(1024.0 * log(v + logadd) + 0.5);
                if (q < 0.0) q = 0.0;
                if (q > 32767.0) q = 32767.0;
                A[y] = (float)(int)q;
            } else {
                A[y] = (float)(1024.0 * log(v + logadd));
            }
        }
    }

    /* 2) B = 低通(A)；A 的全图均值 → 锚点 k0。eff_lmean 见 fl_lowpass()。 */
    if (!fl_lowpass(A, gw, gh, B, eff_lmean, use69)) goto done;
    for (i = 0; i < np; i++) meanA += A[i];
    meanA /= (double)np;

    /* 3) 局部方差：var = ((A-B)^2)>>10，然后再过一次同样的低通 */
    for (i = 0; i < np; i++) {
        double d = (double)A[i] - (double)B[i];
        double v = floor(d * d / 1024.0);
        if (v < 0.0) v = 0.0;
        if (v > 32767.0) v = 32767.0;
        V[i] = (float)v;
    }
    /* V 默认仍走金字塔：跟着 B 一起换成盒式均值，左侧木棚的抑制会被冲淡
     * （实测 1.2→1.7，SPP 1.4），右侧反而更亮。X3F_FL_LMEANV=1 可开。 */
    if (getenv("X3F_FL_LMEANV") != NULL) {
        if (!fl_lowpass(V, gw, gh, V, eff_lmean, use69)) goto done;
    } else if (!fl_pyramid(V, gw, gh, V, use69)) {
        goto done;
    }

    /* 4) 方差 → 标准差（含 256 的底噪，保证 z-score 不会爆） */
    for (i = 0; i < np; i++) {
        double v = V[i] > 0.0 ? V[i] : 0.0;
        if (use69) {
            /* 6.9.0：sqrtTable[i] = round(32·sqrt(i))，索引从 64 开始，
             * 且索引上限为 8447。 */
            int ci = (int)floor(v + 0.5);
            if (ci < 0) ci = 0;
            if (ci > 8447) ci = 8447;
            C[i] = (float)(int)floor(32.0 * sqrt((double)(64 + ci)) + 0.5);
        } else {
            /* 旧标定路径：X3F_FL_CFLOOR 用来放大 sqrt 表的下限。 */
            C[i] = (float)sqrt(v * 1024.0 + cfloor * cfloor);
        }
    }
    /* X3F_FL_CCAP：C 的上限（实验）。边缘处 C 会被撑大，局部项
     * scale·(A−B)/C 反而在最该压的强边上被稀释——掌右侧那条光带就是
     * 这样：同一条手边，左侧过渡缓、压得住，右侧陡、压不住。
     * 0 = 不封顶。 */
    {
        const double ccap = fill_env0("X3F_FL_CCAP", 0.0);
        if (ccap > 1.0) {
            for (i = 0; i < np; i++)
                if (C[i] > ccap) C[i] = (float)ccap;
        }
    }
    for (i = 0; i < np; i++) meanC += C[i];
    meanC /= (double)np;
    scale = 32.0 * sqrt(meanC);

    /* 步骤 1a–4 完成 → 存缓存（键同查找）。 */
    if (apply_cache_on() && lin_fp != 0 && np <= APPLY_CACHE_MAX_PIX) {
        const uint64_t fk2 = apply_fp_d(eff_lmean);
        const uint64_t fk3 = ((uint64_t)use69 << 1) | (uint64_t)tonedin;
        apply_slot_put(&g_fl_slot_a, &g_fl_lock, lin_fp, fk2, fk3, w, h, (size_t)np, A,
                       meanA, meanC, scale);
        apply_slot_put(&g_fl_slot_b, &g_fl_lock, lin_fp, fk2, fk3, w, h, (size_t)np, B,
                       0, 0, 0);
        apply_slot_put(&g_fl_slot_c, &g_fl_lock, lin_fp, fk2, fk3, w, h, (size_t)np, C,
                       0, 0, 0);
    }
fl_base_ready:
    /* 锚点 k0：从（缓存或现算的）meanA 重算——eff_k0off 是每帧可变的
     * tune 旋钮，不进缓存键（原代码把它夹在步骤 2 里，现挪到缓存外）。 */
    k0 = (meanA + X3F_FL_ANCHOR) / 2.0 + (use69 ? 512.0 : 8.0);
    /* 锚点偏移：环境变量优先，否则用面板值（默认 0）。 */
    k0 += eff_k0off;

    /* 5) map = k0 − A + scale·(A−B)/C → 查增益表（线性域三通道同乘） */
    lut = (float *)malloc(65536 * sizeof(float));
    gain = (float *)malloc((size_t)w * h * sizeof(float));
    if (use69) mapbuf = (int16_t *)malloc((size_t)np * sizeof(int16_t));
    if (lut == NULL || gain == NULL || (use69 && mapbuf == NULL)) goto done;
    if (use69) {
        fill_gain_lut_69(F, lut);
        /* SPP 先在缩小图上生成有符号 int16 map，再由 ResizeImage 放大到
         * 全幅；不能把 A/B/C 分别插值后再算 map。 */
        for (i = 0; i < np; i++) {
            double a = A[i], b = B[i], c = C[i];
            double m = mapgain * (k0 - a) +
                       zgain * scale * (a - b) / (c > 0.0 ? c : 1.0);
            int q = (int)m;
            if (q < -32768) q = -32768;
            if (q > 32767) q = 32767;
            mapbuf[i] = (int16_t)q;
        }
    } else {
        fill_gain_lut(F, lut);
    }
    sx = (double)gw / (double)w;
    sy = (double)gh / (double)h;
    /* 亮部回拉的标定旋钮：原来是**逐像素** fill_env0（getenv），2.2M 像素 ×
     * 每像素最多 7 次 ≈ 每帧白烧 2~4 秒。全是环境常量，提到循环外读一次。
     * 逐字节输出不变。 */
    const double env_hl0    = fill_env0("X3F_FL_HLSTART", 0.0);
    const double env_hlw    = fill_env0("X3F_FL_HLWIDTH", 1200.0);
    const double env_hlmode = fill_env0("X3F_FL_HLMODE", 0.0);
    const double env_hlb    = fill_env0("X3F_FL_HLBASE", 1100.0);
    const double env_xb     = fill_env0("X3F_FL_HLXBASE", 1600.0);
    const double env_xw     = fill_env0("X3F_FL_HLXWIDTH", 400.0);
    const double env_xs     = fill_env0("X3F_FL_HLXTRA", 0.45);
    for (y = 0; y < h; y++) {
        double fy = ((double)y + 0.5) * sy - 0.5;
        int y0 = (int)floor(fy);
        double ty = fy - y0;
        if (y0 < 0) { y0 = 0; ty = 0.0; }
        if (y0 > gh - 2) { y0 = gh - 2; if (ty > 1.0) ty = 1.0; }
        for (x = 0; x < w; x++) {
            double fx = ((double)x + 0.5) * sx - 0.5;
            int x0 = (int)floor(fx);
            double tx = fx - x0, a, b, c, m;
            int i0, j0, j1;
            if (x0 < 0) { x0 = 0; tx = 0.0; }
            if (x0 > gw - 2) { x0 = gw - 2; if (tx > 1.0) tx = 1.0; }
            j0 = y0 * gw + x0;
            j1 = (y0 + 1) * gw + x0;
#define FL_SAMP(arr) (((arr[j0] * (1.0 - tx) + arr[j0 + 1] * tx) * (1.0 - ty)) + \
                      ((arr[j1] * (1.0 - tx) + arr[j1 + 1] * tx) * ty))
            a = FL_SAMP(A);
            b = FL_SAMP(B);
            c = FL_SAMP(C);
            if (use69) {
                double ms = FL_SAMP(mapbuf);
                i0 = (int)floor(ms + 0.5);
                if (i0 < -32768) i0 = -32768;
                if (i0 > 32767) i0 = 32767;
                gain[(size_t)y * w + x] = lut[i0 + 32768];
                continue;
            }
            /* 诊断开关：整体项与局部项可以分开加权（X3F_FL_MAPGAIN / X3F_FL_ZGAIN），
             * 用来判定"是整体项太强还是局部项太弱"，或 X3F_FL_NOGLOBAL 只看局部项。 */
            {
                /* 亮部局部项收敛（X3F_FL_HLZRED，先做 A/B）：
                 * 亮而平滑的区域（手掌、白衣服）里，局部 Retinex 项会把
                 * k0−A 的压暗抵消掉，正档 Fill Light 于是只提暗部、不压高光。
                 * 这里按“比整幅锚点亮多少”连续地把局部项收向 0——暗部和
                 * 负档不动，亮部交还给整幅压暗项。用 smoothstep 而不是阈值，
                 * 避免亮部出现块状跳变。 */
                double zw = zgain;
                /* 0.8 会把亮皮肤一路压到 177/137/112（SPP 211/172/154），
                 * 0.7 在 16 位导出那条路上是 216/178/156、≥250 0.5%
                 * （SPP 212/172/154、2.4%）。renderlab 不带降噪，A/B 输入
                 * 不同、同一参数会偏暗，标定以 App 的 16 位导出为准。 */
                const double zred = eff_zred;
                if (zred > 0.0 && F > 0.0) {
                    /* 回拉的起点/宽度：默认从 k0 起、1200 宽（旧行为）。
                     * 起点抬到 k0+400 后只压真正顶到白的那一档（掌心），
                     * 前臂这种“亮但没顶白”的皮肤不会被一起压暗。 */
                    const double hl0 = env_hl0;
                    const double hlw = env_hlw;
                    /* X3F_FL_HLMODE=1：按**局部加成** A−B 而不是亮度 A 判据。
                     * 掌心和前臂的 A 只差 73（7171 vs 7098），按 A 分不开；
                     * 但 A−B 是 2000 vs 1090，正好能只压顶白的那一块。 */
                    const double hlmode = env_hlmode;
                    const double hlb = env_hlb;
                    double t = (hlmode >= 0.5)
                        ? ((a - b) - hlb) / (hlw > 100.0 ? hlw : 100.0)
                        : (a - (k0 + hl0)) / (hlw > 100.0 ? hlw : 100.0);
                    double fs = F < 2.0 ? F * 0.5 : 1.0;
                    if (t < 0.0) t = 0.0;
                    if (t > 1.0) t = 1.0;
                    t = t * t * (3.0 - 2.0 * t);
                    zw *= 1.0 - zred * fs * t;
                    /* X3F_FL_HLMODE=2：保留上面的常规回拉，再对**局部加成极端**
                     * 的那一小块（A−B > 1600，只有顶白的掌心）额外收一刀。
                     * 前臂 A−B≈1090，完全不受影响。 */
                    if (hlmode >= 1.5) {
                        const double xb = env_xb;
                        const double xw = env_xw;
                        const double xs = env_xs;
                        double e = ((a - b) - xb) / (xw > 50.0 ? xw : 50.0);
                        if (e < 0.0) e = 0.0;
                        if (e > 1.0) e = 1.0;
                        e = e * e * (3.0 - 2.0 * e);
                        zw *= 1.0 - xs * e;
                    }
                }
                m = (nog ? 0.0 : mapgain * (k0 - a)) +
                    zw * scale * (a - b) / (c > 1e-6 ? c : 1e-6);
            }
            /* 标定开关 X3F_FL_MSAT=S：对 map 做软饱和 m' = S·tanh(m/S)，S=0 关。
             * 依据： retinex map 在暗端明显更平（用 retinex_call 喂均匀图
             * 实测 dM/dln v 从 100 涨到 495），而我们的 map 随亮度线性走，
             * 深阴影和亮部两端都比 SPP 更极端——F=+2 时掌边暗木提 3~5 倍
             * （SPP 1.4~2.0），手掌只剩 0.3~0.5（SPP 0.8~0.9）。 */
            {
                const double msat = eff_msat;
                if (msat > 1.0) m = msat * tanh(m / msat);
            }
            /* 标定开关 X3F_FL_MSHAPE：map 的非线性整形（>1 = 两端放大、中间压缩）。
             * 依据：显示域逐段误差表说"两端太弱"（F=−2 高光差 −26、F=+2 暗部差 −26），
             * 而实测纹理在 F=−2 掉到 0.72×SPP —— 那是增益曲线在中间调"斜率太大"把
             * 局部对比压掉了。幂律 m' = M0·sign(m)·(|m|/M0)^γ 同时干这两件事：
             * 中间（|m|≪M0）斜率降到 γ 倍以下，两端（|m|>M0）放大。 */
            if (mshape != 1.0) {
                double sgn = (m < 0.0) ? -1.0 : 1.0, am = fabs(m);
                m = sgn * 2000.0 * pow(am / 2000.0, mshape);
            }
            if (!(m == m)) m = 0.0;                      /* NaN → 不改变 */
            i0 = (int)m;                                 /* SPP: fcvtzs 截断 + int16 */
            if (i0 < -32768) i0 = -32768;
            if (i0 > 32767) i0 = 32767;
            gain[(size_t)y * w + x] = lut[i0 + 32768];
        }
    }
#undef FL_SAMP

    /* 标定开关 X3F_FL_GAINSCALE：把最终增益 1+(g−1)·s 整体收一点，
     * 用来判断"暗部/中间调到底是不是整体偏强"，不改变 map 的形状。 */
    {
        const char *gs = getenv("X3F_FL_GAINSCALE");
        if (gs != NULL && gs[0] != '\0') {
            double s = atof(gs);
            size_t k, nn = (size_t)w * h;
            for (k = 0; k < nn; k++) gain[k] = (float)(1.0 + ((double)gain[k] - 1.0) * s);
        }
    }

done:
    if (getenv("X3F_FL_DEBUG")) {
        fprintf(stderr, "[fl] mode=%s down=%d src=%dx%d (sd=%d) map=%dx%d  meanA=%.2f k0=%.2f scale=%.2f  tonedin=%d mapgain=%.2f vscale=%.1f dm=%.4f\n",
                use69 ? "6.9" : "legacy", down, sw, sh, sd, gw, gh,
                meanA, k0, scale, tonedin, mapgain, vscale, vdm);
    }
    /* 定标用：把 A / B / C 三张中间图导出（float32，gw*gh），供离线拿 SPP
     * 参考图反推 map 做对比。X3F_FL_DUMP=/path/base → base_<A|B|C>.f32 + base.txt */
    {
        const char *dp = getenv("X3F_FL_DUMP");
        if (dp && A && B && C) {
            const char *nm[3] = { "A", "B", "C" };
            const float *arr[3];
            char fn[1024];
            FILE *fp;
            int k;
            arr[0] = A; arr[1] = B; arr[2] = C;
            for (k = 0; k < 3; k++) {
                snprintf(fn, sizeof fn, "%s_%s.f32", dp, nm[k]);
                fp = fopen(fn, "wb");
                if (fp) { fwrite(arr[k], sizeof(float), (size_t)np, fp); fclose(fp); }
            }
            snprintf(fn, sizeof fn, "%s.txt", dp);
            fp = fopen(fn, "w");
            if (fp) { fprintf(fp, "%d %d %d %d %d %.6f %.6f %.6f\n", gw, gh, down, w, h, meanA, k0, scale); fclose(fp); }
            /* 最终增益图（全分辨率 float）：量"这块皮肤到底被乘了几倍"。
             * 只给标定用，不影响渲染。 */
            if (gain != NULL) {
                snprintf(fn, sizeof fn, "%s_gain.f32", dp);
                fp = fopen(fn, "wb");
                if (fp) {
                    fwrite(gain, sizeof(float), (size_t)w * h, fp);
                    fclose(fp);
                }
            }
            if (mapbuf != NULL) {
                snprintf(fn, sizeof fn, "%s_map.i16", dp);
                fp = fopen(fn, "wb");
                if (fp) {
                    fwrite(mapbuf, sizeof(int16_t), (size_t)np, fp);
                    fclose(fp);
                }
            }
        }
    }
    free(A); free(B); free(V); free(C); free(lut); free(mapbuf);
    return gain;
}

/* ---------------------------------------------------------------- */
/* SPP 降噪：噪声模型 + 条纹杂讯（方向性双边）
 *
 * 依据 。这一刀只做**条纹杂讯** ——
 * SPP 里只有 Merrill 有这个滑块，参数也最完整、且与辉度/色度解耦；
 * 后两者要等这一层验完再叠上去。
 *
 * 标定来自 CAMF 段（Huffman 压缩，只有解码层能读，所以走 x3f_camf_matrix_f64）：
 *   ColorNoiseModel   σ² 的 (a,b,c)。**存储是系数优先**：
 *                     (aR,aG,aB, bR,bG,bB, cR,cG,cB)，逐通道取
 *                     T[ch] + T[3+ch]·v + T[6+ch]·v²
 *                     —— 和 SPP 自己 CalculateNoiseVar 的取法一致
 *                     （它读 T[0]/T[3]/T[6] 当同一通道的三个系数）。
 *   DenoisingSlidersStep  5 档 → 13 个乘子
 *   ISONoiseSettings      逐 ISO 的 12 个参数
 *
 * 条纹的算法（SPP：ProcessTiles → MatchColOffsets → DirectionalBilateral）：
 *   权重  w = exp( −strength · Σ_c Δ_c²/σ²_c )       Δ = 邻居 − 中心
 *   窗口半宽 = {15,21,27,33,39}[档位]
 *   strength = isoScale · {0.625,0.5,0.375,0.25,0.125}[档位]
 *   只在**一个方向**上取邻居（名字里的 Directional）；SPP 用 2048 项
 *   exp(−k/256) 查表，这里直接算 exp，数值等价。
 *   σ² 是逐通道的二次噪声模型 —— 所以"差多少算噪声"随亮度自适应。
 */

#define X3F_NR_ISO_MAX 16

typedef struct {
    int    valid;
    double cn[9];                    /* ColorNoiseModel 原始顺序（系数优先） */
    double cn_iso[9];                /* 同一组系数，按拍摄 ISO 增益缩放后的（见 nr_build_iso_coef） */
    double cn_iso_c[9];              /* 色度那条用的一组：ISO 缩放更温和（见 nr_build_iso_coef） */
    double step[5][13];              /* DenoisingSlidersStep（行 = 档位） */
    double iso[X3F_NR_ISO_MAX][13];  /* ISONoiseSettings（行 = ISO，列 0 = ISO） */
    int    iso_n;
    double sensor_iso;               /* 这张照片的 ISO（CAMF SensorISO） */
    double capture_iso;              /* CAMF CaptureISO —— 真正的拍摄 ISO（SensorISO 在高 ISO 下只给模拟增益那一档：DP0Q2315 是 400 vs 1600） */
    int    quattro_model;            /*用的是烘在 里的那组 Quattro 系数（CAMF 没有 ColorNoiseModel）*/
} x3f_nr_model_t;

static x3f_nr_model_t g_nr;
static int g_nr_bflut;        /*用 bfLUT 放开门限（X3F_NR_BFLUT=1）*/
/* 解码那一步的 NLM 有没有**真的连亮度一起降**（load_common 里按 AUTO/ON 定下来）。
 * 见下面 nr_luma_stack_cap()：两处亮度降噪叠起来时，渲染链这根滑块的上半段是纯亏。 */
static int g_nlm_luma_on;
/* 多尺度时**逐层**替换 σ² 系数（SPP：`F23Stage1ProcParams.m_fBFNoiseScaling[L]`
 * 逐层给双边的噪声缩放；我们原来三层共用一个 σ）。非 NULL 时覆盖 cn_iso。 */
static const double *g_nr_level_coef;
/* 色度多尺度（X3F_NR_CHROMA_PYR，2026-10-27）：逐层 σ 乘子（SPP NR3 的
 * base[L]/BFS[L] 语义，§22.1）+ 色度模式旗标（跳过 ②~⑥ 亮度步、
 * 噪声系数换 cn_iso_c —— 色度滑杆不许碰亮度）。 */
static const double *g_pyr_bfs_mul;
static int g_pyr_chroma_mode;

/* 解码层提供的 CAMF 取数口（见 x3f-ffi-c/src/lib.rs）。 */
/* dims **必须**给 3 个元素的位置：FFI 那边会写 dims[0]/[1] 之外，还写一个
 * dims[2]（恒为 1）。写 2 个就是越界 4 字节 —— 栈上正好压到保护金丝雀，
 * 函数返回时 abort（2026-10-15 在 Windows 那条链上实测踩到过，见 C 调用点注释）。 */
extern int x3f_camf_matrix_f64(const char *path, const char *name,
                               double *out, int cap, unsigned *dims);

/* 给 x3f_camf_matrix_f64 传 dims 的地方一律用这个尺寸，别写字面量 2。 */
#define X3F_CAMF_DIMS 3

/* 噪声模型的自变量单位是**原始 DN**，而本渲染链的线性值是 0..1（1.0 ≈ 白点）。
 * 这个常数是两者的换算（白点对应的 DN）。Merrill 是 14bit，先按 16383 试，
 * 拿 SPP 参考图定标后可以改；X3F_NR_FS 可覆盖（同 X3F_FL_VSCALE 的做法）。 */
#define X3F_NR_FULLSCALE 16383.0

/* 噪声模型信号域 —— 和 **图像域** 不是一回事，别混。
 *
 * `Δ²/σ²` 本身对尺度免疫（Δ 和 σ 一起缩放），但噪声模型是
 * `σ² = a + b·v + c·v²`，那个**常数项 a**（读出噪声²）不跟着缩放 ——
 * 所以域越小，a 占比越大、σ 相对 Δ 越大、权重越接近 1、压得越狠。域是模型的
 * 一部分，不能随便定。
 *
 * 实测标定（2026-10-13，DP3M1295 + SPP 三张导出）：SPP 满档把**列间条纹**
 * 降 23.8%，而按 16383 这个域算出来只降 0.2% —— 差约 125×。
 * 扫下来等效域 ≈960~1024（域=1024 时我们降 22.7%）。取 **1024**：
 * 它是 2 的幂，像是固定点归一化的域，而且落点在参考值 5% 以内；
 * 为了单张图去凑 960 属于过拟合。**这只是单张图的结果**，多几张参考图再复核。
 *
 * 注意这个域**只管噪声模型**；`GenerateKernelMap` 那张半径图用的阈值
 * （150/300/500/700）比的是**图像值**，还是按 16383 那个域算的，所以
 * 两者在代码里分开：`nr_fullscale()` 给半径图，`nr_nmfs()` 给噪声模型。 */
#define X3F_NR_NMFS 1024.0

/* 分阶段计时（X3F_NR_TIME=1 时打印）。墙钟——并行下它才是"用户等了多久"。 */
/* Windows 没有 clock_gettime/CLOCK_MONOTONIC（见下面的 nr_now）。x3f_parallel.h
 * 在 _WIN32 下已经带过 windows.h，这里再写一次是不想依赖那层传递包含 —— 有
 * include guard 挡着，重复包含是空操作。 */
#if defined(_WIN32)
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#endif

static double nr_now(void)
{
#if defined(_WIN32)
    /* QueryPerformanceCounter 同样单调、与系统时间调整无关，分辨率还比
     * 毫秒级时钟高，正好是这里要的"墙钟"。 */
    LARGE_INTEGER freq, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&now);
    return (double)now.QuadPart / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#endif
}

static double nr_fullscale(void)
{
    const char *e = getenv("X3F_NR_FS");
    if (e != NULL) {
        double v = atof(e);
        if (v > 1.0) return v;
    }
    return X3F_NR_FULLSCALE;
}

static double nr_nmfs(void)
{
    const char *e = getenv("X3F_NR_NMFS");
    if (e != NULL) {
        double v = atof(e);
        if (v > 1.0) return v;
    }
    return X3F_NR_NMFS;
}

/* ★ 辉度/色度那条双边用的是**另一个**域（2026-10-14 标定）。
 *
 * 起因：拿 DP0Q1000（Quattro）跟用户在 SPP 里导的三张参考（smooth/mid/more =
 * 档 0/2/4，辉度+色度一起动）逐档在**平坦区**比（掩膜取自 SPP 参考、
 * 3×3 亮度极差 ≤2、60.3% 像素、16 位浮点量），亮度 HF 是
 *
 *      档位      SPP     我们(域 1024)     我们(域 768)
 *      档 0    0.5586    0.7429 (1.33×)    0.6325 (1.13×)
 *      档 2    0.2932    0.4376 (1.49×)    0.3325 (1.13×)
 *      档 4    0.1945    0.2821 (1.45×)    0.1612 (0.83×)
 *
 * —— 域 1024 下每一档都稳定偏弱 ~1.4×，收到 768 就落回 1.1× 上下。
 * 色度同向（档 2：0.0858 → 0.0818，SPP 0.0840；档 4：0.0953 → 0.0613，SPP 0.0652）。
 *
 * 但**条纹**那边不是这个数：同一套代码在 DP3M1295（Merrill）上量列条纹降幅，
 *      SPP 关→满 −23.8% · 我们 域 1024 −22.6% · 我们 域 768 −26.7%（过头）
 * 条纹的标定要 1024。两个数对不上，不能共用一个常数 —— 大概是 Stage1
 * （辉度/色度）与 Stage2B（条纹）本来就在不同的域里跑（Stage1 还有逐层 scales[]），
 * 我们把它压成一个 0..1 线性域，就只能各自标定。于是拆成两个：
 *
 *      X3F_NR_NMFS      默认 1024   条纹（Stage2B 那条）
 *      X3F_NR_NMFS_LC   默认  768   辉度/色度（Stage1 那条）
 *
 * 两边都还是单张参考图标出来的；而且域只能改整体强度、改不了曲线形状
 * （档 0 想更狠、档 4 想更弱 ⇒ 我们的曲线比 SPP 平，多半是缺它那套多尺度结构）。 */
/* ★ 2026-10-15：Quattro 那条链要**另一个**域（实测标定，见下）。
 *
 * 起因：拿 DP0Q1000（Quattro，ISO100）和我们自己的出图，对 三张参考
 * （`DP0Q1000-smooth/mid/more.jpg` = 档 0/2/4）比"平坦区亮度 HF 的档位响应"。
 * 口径：掩膜取自 SPP 参考（3×3 亮度极差 ≤2/255）、先 4× 盒式降采样，
 * 工具 `outputs/tools/lab/nrcmp.py`。响应是 **档2 −36.3%、档4 −57.2%**。
 *
 *   域     档2      档4
 *   768   −60.1%   −68.2%      ← 原来的默认（Merrill 上标出来的）
 *  1280   −49.1%   −64.7%
 *  2048   −37.9%   −57.2%      ← ★ 命中
 *  3072   −29.7%   −48.3%
 *  4096   −24.1%   −41.8%
 * 12288    −5.9%   −17.8%
 *
 * 所以 CAMF 标定那条（Merrill）保持 768，** 那组 Quattro 系数用 2048**。
 * 物理上两边也对得上：把实测噪声与 σ(v) 曲线对齐（`outputs/tools/lab/nrfit.c`）
 * 给出的也是这个量级 —— Quattro 的系数来自 SPP 自己的 DN 域，不是 Merrill 的。 */
#define X3F_NR_NMFS_LC   768.0   /* CAMF ColorNoiseModel（Merrill） */
#define X3F_NR_NMFS_LC_Q 2048.0  /*的 Quattro 系数*/
/* 色度那条还要**再大**（实测，见 §17.13）：同一组系数、同一张 DP0Q1000，
 * 亮度按 2048 命中 档位响应，而色度要用 8192 才命中 —— 色度本来就
 * 走另一条（Stage1 里的 UV pre-filter，见 §6），两侧的 σ 不是一套。
 * 这里不重写那条链，先给它一个独立的域把幅度对上。 */
#define X3F_NR_NMFS_C_Q  6144.0

static double nr_nmfs_lc(void)
{
    const char *e = getenv("X3F_NR_NMFS_LC");
    if (e != NULL) {
        double v = atof(e);
        if (v > 1.0) return v;
    }
    return g_nr.quattro_model ? X3F_NR_NMFS_LC_Q : X3F_NR_NMFS_LC;
}

static double nr_nmfs_c(void)
{
    const char *e = getenv("X3F_NR_NMFS_C");
    if (e != NULL) {
        double v = atof(e);
        if (v > 1.0) return v;
    }
    return g_nr.quattro_model ? X3F_NR_NMFS_C_Q : X3F_NR_NMFS_LC;
}

/* Quattro 的 σ² 系数**不在 CAMF 里**（它的 .meta 里连 "noise" 这个词都没有，
 * 见 §10.6）。它们烘在 的常量区：
 * 文件偏移 处的 9 个 double，被 `F23Stage1NoiseReduction`
 * 与它的 Mono 变体、以及 `XDoNDCR` 三处引用，直接抄进 F23Stage1ProcParams
 * 的 +0xe0…+0x128（正好对上 CalculateNoiseVar 读的那个系数块）。
 * 布局与 ColorNoiseModel 完全一致：系数优先 aR,aG,aB,bR,bG,bB,cR,cG,cB。
 * 取证见 §11。
 *
 * 域：**不能沿用 Merrill 那个 768**（2026-10-15 实测，见 X3F_NR_NMFS_LC 上面那段：
 * 用 768 时档位响应是 −60%/−68%，SPP 是 −36%/−57%；域 2048 才逐点对上）。
 * 两个模型的读出噪声数值确实同量级，但**系数所在的域**不是一回事：
 * Merrill 那份来自 CAMF（和我们链路一起标定的），Quattro 这份是从 抄的
 * SPP 自己的 DN 域。所以按"这份系数从哪来"选域：见 nr_nmfs_lc()。 */
static const double nr_quattro_model[9] = {
    302.25516, 156.5858, 24.5292,                     /* a: R G B */
      0.25684,   0.22657, 0.13639,                    /* b: R G B */
    7.6665e-06, 7.9716e-06, 5.2763680601690054e-05    /* c: R G B */
};

/* σ² 的 ISO 缩放（定义在下面，紧跟 nr_var；nr_load_model 里要用，先声明） */
static double nr_iso_gain(void);
static void   nr_build_iso_coef(void);

/* 拍摄 ISO：CAMF 的 CaptureISO，退回 SensorISO，都没有给 0。
 *
 * **CaptureISO 不是 SensorISO**：DP0Q2315 的 SensorISO 只有 400（模拟增益那一段），
 * CaptureISO 才是 1600。降噪这一路的 ISO 一律用这个（见 nr_iso_gain 的说明）。 */
static double read_shooting_iso(const char *path)
{
    int d[X3F_CAMF_DIMS] = { 1, 1, 1 };   /* FFI 会写 3 个 u32，别缩成 2 */
    double v = 0.0;
    char tmp[1024];
    const char *use;
    if (path == NULL) return 0.0;
    /* CAMF 取数是"按路径"的接口：X3I 容器先物化该帧（0 EV 基准帧）。七帧的 ISO
     * 本来就一样，用基准帧即可 —— 而且 load_common 传进来的已经是物化后的路径，
     * 这里不会再物化第二次。 */
    use = x3f_standalone_path(path, -1, tmp, sizeof(tmp));
    if (x3f_camf_matrix_f64(use, "CaptureISO", &v, 1, d) == 1) {
        x3f_temp_release(tmp);
        return v;
    }
    if (x3f_camf_matrix_f64(use, "SensorISO", &v, 1, d) == 1) { x3f_temp_release(tmp); return v; }
    x3f_temp_release(tmp);
    return 0.0;
}

static void nr_load_model(const char *path)
{
    /* ISO 先读：它与噪声模型无关，而且 Quattro 的 CAMF 里**没有**噪声模型
     * （见 §10.6），下面会提前 return ——
     * 所以不能把 ISO 的读取放在噪声模型之后。
     * 优先 CaptureISO（拍摄 ISO， base 用的就是它），退回 SensorISO。*/
    g_shooting_iso = read_shooting_iso(path);
    unsigned d[X3F_CAMF_DIMS];   /* 尺寸见上面的宏：FFI 写满 3 个 */
    int n;

    g_nr.valid = 0;
    g_nr_bflut = nr_flag("X3F_NR_BFLUT", 0);
    if (path == NULL) return;
    memset(&g_nr, 0, sizeof g_nr);

    n = x3f_camf_matrix_f64(path, "ColorNoiseModel", g_nr.cn, 9, d);
    if (n != 9) {
        /*Quattro：CAMF 里没有这份标定，换用烘在 里的那 9 个数。
         * 只对非 Merrill 生效 —— Merrill 缺模型说明这文件有问题，
         * 别拿 Quattro 的数去套它。 */
        if (x3f_render_probe_is_merrill(path)) return;
        memcpy(g_nr.cn, nr_quattro_model, sizeof nr_quattro_model);
        g_nr.quattro_model = 1;
        if (getenv("X3F_NR_DEBUG"))
            fprintf(stderr, "[nr] CAMF 里没有 ColorNoiseModel → "
                            "用 里那组 Quattro 系数（a=%.4g/%.4g/%.4g）\n",
                    g_nr.cn[0], g_nr.cn[1], g_nr.cn[2]);
    }
    n = x3f_camf_matrix_f64(path, "DenoisingSlidersStep", g_nr.step[0], 5 * 13, d);
    if (n != 65) return;
    n = x3f_camf_matrix_f64(path, "ISONoiseSettings", g_nr.iso[0],
                            X3F_NR_ISO_MAX * 13, d);
    g_nr.iso_n = (n > 0) ? n / 13 : 0;
    n = x3f_camf_matrix_f64(path, "SensorISO", &g_nr.sensor_iso, 1, d);
    if (n != 1) g_nr.sensor_iso = 0.0;
    /* 拍摄 ISO（真 ISO）。SensorISO 在高 ISO 下只给到模拟增益那一段 ——
     * DP0Q2315：SensorISO=400、CaptureISO=1600，而 EXIF 的 ISO 也是 1600。
     * 噪声模型的 ISO 增益要用这个（见 nr_build_iso_coef）。 */
    n = x3f_camf_matrix_f64(path, "CaptureISO", &g_nr.capture_iso, 1, d);
    if (n != 1 || g_nr.capture_iso <= 0.0) g_nr.capture_iso = g_nr.sensor_iso;
    nr_build_iso_coef();          /* ISO 100 → 等于 cn，输出与改动前逐字节相同 */
    g_nr.valid = 1;

    if (getenv("X3F_NR_DEBUG"))
        fprintf(stderr, "[nr] 标定 %s: a=%.4g/%.4g/%.4g b=%.4g/%.4g/%.4g "
                        "c=%.4g/%.4g/%.4g iso_rows=%d ISO增益=%.3g(Capture=%.0f/Sensor=%.0f)\n",
                path, g_nr.cn[0], g_nr.cn[1], g_nr.cn[2],
                g_nr.cn[3], g_nr.cn[4], g_nr.cn[5],
                g_nr.cn[6], g_nr.cn[7], g_nr.cn[8], g_nr.iso_n,
                nr_iso_gain(), g_nr.capture_iso, g_nr.sensor_iso);
}

/* σ² 的逐通道取值（v 用原始 DN）。存储系数优先：T[ch], T[3+ch], T[6+ch]。 */
static inline double nr_var(const x3f_nr_model_t *m, int ch, double v)
{
    return m->cn[ch] + m->cn[3 + ch] * v + m->cn[6 + ch] * v * v;
}

/* σ² 随 ISO 的缩放（【实证 + 物理】2026-10-15）——
 *
 * 起因：ISO 1600（DP0Q2315）上我们比 SPP 脏 3 倍、而且三根滑块几乎不动作。
 * 量线性域的真实噪声：ISO 100 的相邻差中位数 3.2e-3、我们模型的 σ 是 1.7e-2
 * （门全开，所以 ISO 100 那条曲线标得准）；ISO 1600 的真实噪声 **4.2e-2**，
 * 而模型的 σ 还是 1.7e-2 —— **真噪声比模型 σ 还大 2.5 倍**，于是
 * u = k·fs²·Δ²/σ² ≈ 3，权重 exp(−3)≈0.05，滤波器把噪声当细节挡在门外。
 *
 * 物理上正确的关系（也是 SPP `CalculateNoiseVar` 那套 `s²·(a + b·(v/s) + c·(v/s)²)`
 * 的同一个形式，只是 s 取 ISO 增益）：
 *     读出噪声² ∝ g²（随模拟增益放大）
 *     散粒噪声² ∝ g （DN 域里 σ² = v·g·k）
 *     PRNU/固定图案项不随增益
 * 所以 a×g²、b×g、c×1。**g = max(1, CaptureISO/100)**（100 是模型标定所在的 ISO），
 * 于是 ISO 100 时 g=1、与改动前**逐字节相同**，高 ISO 才起作用。
 *
 * 注意 ISO 要取 **CaptureISO**，不是 SensorISO：DP0Q2315 的 SensorISO=400（只有
 * 模拟增益那一段），CaptureISO=1600 才是真 ISO；SPP 用的是它自己估的 EstimatedISO。 */
static double nr_iso_gain(void)
{
    const char *e = getenv("X3F_NR_ISOGAIN");
    double g;
    if (e != NULL) {
        double v = atof(e);
        if (v > 0.0) return v;         /* 调试：直接给增益（1 = 关） */
    }
    g = (g_nr.capture_iso > 0.0 ? g_nr.capture_iso : g_nr.sensor_iso) / 100.0;
    if (g < 1.0) g = 1.0;
    return g;
}

static void nr_build_iso_coef(void)
{
    double g = nr_iso_gain();
    /*色度那条用**更温和**的 ISO 缩放：拿 DP0Q2315（ISO 1600）对 三张
     * 降噪档参考（smooth/mid/more = 0/0.5/1.0）量出来的 —— 辉度用 g 时响应是
     * −23%/−56%，SPP 是 −34%/−48%（同量级 ✓）；而同一条链的色度是 −36%/−73%，
     * SPP 只有 −21%/−25%（**过强 3 倍**）。把色度的 ISO 增益取一半（g/2）后
     * 落回 −14%/−31%，对得上 斜率。ISO 100 时 g = 1 ⇒ 两个增益都是 1，
     * 输出与改动前逐字节相同。`X3F_NR_ISOGAIN_C` 可覆盖（见工具/笔记 §17.16）。 */
    const char *e = getenv("X3F_NR_ISOGAIN_C");
    /*只对 那组 Quattro 系数生效 —— Merrill 那条链是在自己的参考图上标定的，
     * 不能拿一张 Quattro 的量测去改它（改了就回不到原样了）。 */
    double gc = g;
    if (g_nr.quattro_model) gc = (g > 2.0) ? g * 0.5 : 1.0;
    if (e != NULL && atof(e) > 0.0) gc = atof(e);
    int i;
    for (i = 0; i < 3; i++) {
        g_nr.cn_iso[i]     = g_nr.cn[i]     * g * g;
        g_nr.cn_iso[3 + i] = g_nr.cn[3 + i] * g;
        g_nr.cn_iso[6 + i] = g_nr.cn[6 + i];
        g_nr.cn_iso_c[i]     = g_nr.cn[i]     * gc * gc;
        g_nr.cn_iso_c[3 + i] = g_nr.cn[3 + i] * gc;
        g_nr.cn_iso_c[6 + i] = g_nr.cn[6 + i];
    }
}

static inline double nr_var_c(const double *c, int ch, double v)
{
    return c[ch] + c[3 + ch] * v + c[6 + ch] * v * v;
}

/* 滑块 0..1 → 5 档（SPP：floor(clamp(v,0,1)·4)） */
static inline int nr_level(double s)
{
    int l;
    if (s <= 0.0) return 0;
    if (s >= 1.0) return 4;
    l = (int)(s * 4.0);
    return (l < 0) ? 0 : ((l > 4) ? 4 : l);
}

/* ISO 缩放：SPP 用估计 ISO 落档；这里直接按文件记录的 ISO 与
 * ISONoiseSettings 的 ISO 列做线性插值，取第一个参数列（列 1）当"档位缩放"。
 * 这一刀只用来把强度调到与 ISO 相称的量级，后续再逐列对齐。 */
/* 这张照片的 ISO（CAMF 的 CaptureISO/SensorISO；0 = 没读到）——「细节」要用 */
static double x3f_file_iso(void) { return g_shooting_iso; }

static double nr_iso_scale(const x3f_nr_model_t *m, double iso)
{
    int i;
    double lo, hi, t;

    if (m->iso_n <= 0 || iso <= 0.0) return 1.0;
    if (iso <= m->iso[0][0]) return m->iso[0][1];
    for (i = 1; i < m->iso_n; i++) {
        if (iso <= m->iso[i][0]) {
            lo = m->iso[i - 1][0];
            hi = m->iso[i][0];
            t = (hi > lo) ? (iso - lo) / (hi - lo) : 0.0;
            return m->iso[i - 1][1] * (1.0 - t) + m->iso[i][1] * t;
        }
    }
    return m->iso[m->iso_n - 1][1];
}

/* ---- 辉度 / 色度：二维噪声归一化双边 ----
 *
 * SPP（Merrill）走的是「自适应半径双边 + 各向异性扩散 + 残差混合」那一串，
 * 逐块算半径（GenerateKernelMap）、range 核是加窗 tanh（GetTanhLUT）。
 * 这里先立**结构**：同一套噪声模型 + 双边，区别只在窗宽和 σ 缩放 ——
 *   · 辉度：逐通道，小窗，σ 缩放 = DenoisingSlidersStep[档位][0]（0.25…4）
 *   · 色度：只在 Cb/Cr 上做，窗更大、σ 缩放更狠（列 8：0.5…1.5）
 * range 权重用经典的 u = 0.5·Δ²/σ²（等价于 range σ = 噪声 σ），
 * 也就是"差得比噪声大就别动它"。等有 SPP 参考图再把半径和缩放拟合准
 * （X3F_NR_LUMA_R/LUMA_K/CHROMA_R/CHROMA_K 是给这一步留的旋钮）。
 */

static inline void nr_rgb2ycc(const float *p, double ycc[3])
{
    double r = p[0], g = p[1], b = p[2];
    ycc[0] = 0.2126 * r + 0.7152 * g + 0.0722 * b;
    ycc[1] = b - ycc[0];
    ycc[2] = r - ycc[0];
}

/*亮度（ F23 链里到处用的就是 (R+G+B)/3，见 §17.9.2 的实测）*/
static inline double nr_lum3(const float *p)
{
    return ((double)p[0] + (double)p[1] + (double)p[2]) / 3.0;
}

/* 把一张 RGB 图拆成亮度平面（六步的"亮度专用"版在这上面跑，§17.19） */
static void nr_s1_luma_plane(float *Y, const float *rgb, int w, int h)
{
    size_t i, n = (size_t)w * h;
    for (i = 0; i < n; i++) Y[i] = (float)nr_lum3(rgb + i * 3);
}

static inline void nr_ycc2rgb(const double ycc[3], float *p)
{
    double y = ycc[0], cb = ycc[1], cr = ycc[2];
    p[0] = (float)(y + cr);
    p[1] = (float)(y - (0.2126 * cr + 0.0722 * cb) / 0.7152);
    p[2] = (float)(y + cb);
}

/* 每个中心像素的 inv_i = k·fs²/σ²_i，只算一次（内层只留乘加）。
 *
 * 门限用的是 **cn_iso**（含 ISO 增益的那组系数，见 nr_build_iso_coef）——
 * 只有辉度/色度这条双边走这里。**条纹不走这里**：它另有 ISONoiseSettings 的
 * iso_scale，用原始系数（`nr_var`）就够，两边都乘会重复计。
 * g = 1（ISO 100）时 cn_iso == cn，所以 ISO 100 的输出与改动前逐字节相同。 */
/* 这里计算逐通道的 1/σ² 权重（×fs²k）：σ² 用哪组系数由调用方给 ——
 * 辉度走 `cn_iso`（ISO 按 g 缩放），色度走 `cn_iso_c`（更温和，见 nr_build_iso_coef）。 */
static inline void nr_inv3(const float *p, double fs, double k, double inv[3],
                           const double *coef)
{
    int i;
    for (i = 0; i < 3; i++) {
        double sig = nr_var_c(coef, i, (double)p[i] * fs);
        if (sig < 1e-6) sig = 1e-6;
        inv[i] = k * fs * fs / sig;
        /*★ `bfLUT`（2026-10-15 读出来的用法，见 §17.21/§17.25）：
         *
         *   `BilateralFilterRGBF23` 里逐通道做 `σ²_c ← σ²_c · bfLUT[idx]`，
         *   而 **idx 是"中心像素的值"**（汇编 `0xd5a6c`：`s11 = max(中心.B, 0)`，
         *   再 `s11·s6` 夹到 14384），表是
         *       `bfLUT[i] = 31·(i/14407)^4 + 1`   （`0xd8970..0xd8a24`，14385 项）
         *   ⇒ 按亮度放开门限：暗端 ≈ ×1（不变）、亮端最多 ×32。
         *
         * 换算到我们这边：v 是"白点=1"的归一值，SPP 那边 v 除以白点再乘 14384
         * 进表，所以这里是 `idx = v·14384`。
         * `X3F_NR_BFLUT=1` 打开（默认关 = 与改动前逐字节相同）。 */
        if (g_nr_bflut) {
            double idx = (double)p[i] * 14384.0;
            double t;
            if (idx < 0.0) idx = 0.0;
            if (idx > 14384.0) idx = 14384.0;
            t = idx / 14407.0;
            inv[i] /= (31.0 * t * t * t * t + 1.0);
        }
    }
}

/* ---------- SPP「亮部软门」：WFA 的 LUT_A（默认关） ----------
 *
 * 来自 compose 阶段（`SFDComposeWFA`，见
 * §2.1），常数是硬编码的：
 *
 *   LUT_A[i] = 1 / (1 + exp(0.005·(i − C)))，         C = 9000.0
 *   LUT_B[i] = max(1/(1+exp(0.015·(300 − i))), 1e-10)  （只在 SPP 多帧路径上用）
 *
 * 索引就是**信号值本身**（原始 DN，clamp 到 65535 —— 表 65536 项，
 * 索引前 clamp 的常数是 = 65535.0f，不是 65504）。
 *
 * 形状：C=9000 附近有个 ±1000 的软过渡，往暗端→1、往亮端→0。
 * 所以它本质是一条**「这个像素是不是快饱和了」的软门**，而不是"暗部提亮"：
 * 越接近饱和权重越小（i≥12000 时已到 1e-7 量级）。
 *
 * 用在哪：我们把它当**邻居门**乘进双边权重 ——
 * `w_q = exp(−Σ Δ²/σ²) · LUT_A[V_q]`。
 * （只乘中心像素的话会被归一化约掉，一定要按"贡献者自己"的 V 取。）
 * 效果上等价于"快饱和的邻居不许被拉进暗部"，
 * 和 SPP 那道 `CalcLWeight_ReduceHighlightArtifact` 是同一个动机。
 *
 * ⚠️ 两点口径说明（都是还没钉死的，所以留了旋钮）：
 *   ① 索引用的域。SPP 那个 V 的域我们没定死，这里按**图像域**
 *      （`nr_fullscale()` = 16383）算，于是门压在 9000/16383 ≈ 55%。
 *      `X3F_NR_WFA_FS` 可覆盖。
 *   ② 我们只接了 LUT_A，没接 LUT_B（SPP 那边是 `count != 1` 才乘，
 *      单张文件本来就不乘）。
 *
 * 默认关（`X3F_NR_WFA=1` 打开），保证默认出图逐字节不变。
 */
#define X3F_WFA_LUT_N 65536
#define X3F_WFA_C     9000.0

static float *g_wfa_lut;         /* LUT_A，懒建 */

static const float *nr_wfa_lut(void)
{
    int i;
    if (g_wfa_lut != NULL) return g_wfa_lut;
    g_wfa_lut = (float *)malloc(X3F_WFA_LUT_N * sizeof(float));
    if (g_wfa_lut == NULL) return NULL;
    for (i = 0; i < X3F_WFA_LUT_N; i++)
        g_wfa_lut[i] = (float)(1.0 / (1.0 + exp(0.005 * ((double)i - X3F_WFA_C))));
    if (getenv("X3F_NR_WFA_DEBUG"))
        fprintf(stderr, "[nr] WFA 门 LUT_A: [0]=%.4f [4500]=%.4f [9000]=%.4f"
                        " [12000]=%.3g [16383]=%.3g\n",
                g_wfa_lut[0], g_wfa_lut[4500], g_wfa_lut[9000],
                g_wfa_lut[12000], g_wfa_lut[16383]);
    return g_wfa_lut;
}

/* 逐像素门图：gate[px] = LUT_A[ clamp(max(R,G,B)·fs, 0, 65535) ] */
static void nr_build_wfa_gate(const float *src, int w, int h, double fs,
                              float *gate)
{
    const float *lut = nr_wfa_lut();
    size_t i, n = (size_t)w * h;
    if (lut == NULL) { for (i = 0; i < n; i++) gate[i] = 1.0f; return; }
    for (i = 0; i < n; i++) {
        const float *p = src + i * 3;
        double v = p[0];
        int idx;
        if (p[1] > v) v = p[1];
        if (p[2] > v) v = p[2];
        v *= fs;
        if (!(v > 0.0)) v = 0.0;
        if (v > 65535.0) v = 65535.0;
        idx = (int)v;
        gate[i] = lut[idx];
    }
}

typedef struct {
    float       *dst;
    const float *src;
    int          w, h, radius, mode;
    double       k, fs;
    const double *coef;          /* 用哪组 σ² 系数：辉度 cn_iso / 色度 cn_iso_c */
    int          uvgate;         /* 色度那条用 UV 距离做门（而不是 RGB 距离，见 worker 里注释） */
    double       shape;          /*>0 时用 **四次幂**权重 1/(1+31·(u/shape)^4) 取代 exp(−u)*/
    const float *gate;           /* 亮部门图（可空；非空时乘进每个邻居的权重） */
    const unsigned char *rmap;   /* 逐块半径图（可空；非空时按块取半径） */
    int          block;          /* 块边长 */
    int          bw;             /* 半径图的宽（块数） */
} nr_bil_ctx_t;

/*逐块自适应半径 = GenerateKernelMap（` `，3364 字节）。已解出：
 *
 *   1) 32×32 分块（`GenerateKernelMap(src, map, 32, …)`，块边长由 ProcessTiles 传 32）
 *   2) 块值 = 块内 **min(R,G,B) 的均值**（原始 DN）——
 *      注意是**逐像素取三通道最小**再平均，不是先平均再取最小；
 *      边缘的碎块按**实际像素数**除（代码里对末行/末列单独走了一遍）
 *   3) 对块值图做 **3×3 最小值滤波**（取邻域最暗 → 保守）
 *   4) 按亮度阶梯给**半径**（阈值 150/300/500/700 DN，四次独立比较 + 累加）：
 *
 *        ≥700 → 3 / 5 / 6 / 7        （ISO > 400 的表）
 *        500–700 → 5 / 3
 *        300–500 → 6 / 4
 *        150–300 → 7 / 5
 *        <150 → 9 / 9
 *
 *      两根列分别是 **ISO>400** 与 **ISO≤400**（判据是 `NoiseParams+0x4` 与 400.0
 *      比较，那格是估计 ISO）。即**越暗半径越大**（暗部噪声大要多压），
 *      而 ISO 高时整档再大一号。
 *
 * 顺带一个重要的旁证：这些阈值是拿 150…700 去比**图像值**，所以
 * **SPP 在这一步的像素域就是原始 DN（0..16383）** —— 和噪声模型
 * （ColorNoiseModel 的 a/b/c 量级）对得上，`X3F_NR_FULLSCALE` 取 16383 是对的。
 */
static void nr_build_radius_map(const float *src, int w, int h, int block,
                                double iso, double fs,
                                unsigned char *rmap, int *bw_out)
{
    static const double thr[4] = { 700.0, 500.0, 300.0, 150.0 };
    static const int    rad_hi[5] = { 3, 5, 6, 7, 9 };   /* ISO > 400 */
    static const int    rad_lo[5] = { 2, 3, 4, 5, 9 };   /* ISO <= 400 */
    const int *rad = (iso > 400.0) ? rad_hi : rad_lo;
    int bw = (w + block - 1) / block;
    int bh = (h + block - 1) / block;
    double *blk;
    int by, bx;

    *bw_out = bw;
    blk = (double *)malloc((size_t)bw * bh * sizeof(double));
    if (blk == NULL) return;

    for (by = 0; by < bh; by++) {
        for (bx = 0; bx < bw; bx++) {
            int x0 = bx * block, y0 = by * block;
            int x1 = (x0 + block > w) ? w : x0 + block;
            int y1 = (y0 + block > h) ? h : y0 + block;
            double acc = 0.0;
            int n = 0, x, y;

            for (y = y0; y < y1; y++) {
                const float *row = src + (size_t)y * w * 3;
                for (x = x0; x < x1; x++) {
                    const float *p = row + (size_t)x * 3;
                    double mn = p[0];
                    if (p[1] < mn) mn = p[1];
                    if (p[2] < mn) mn = p[2];
                    acc += mn * fs;                   /* scene → DN */
                    n++;
                }
            }
            blk[(size_t)by * bw + bx] = (n > 0) ? acc / (double)n : 0.0;
        }
    }
    /* 3×3 最小值滤波 + 阶梯查表 */
    if (getenv("X3F_NR_DEBUG")) {
        /* 块值（min(RGB) 的块均值，DN）的分布 —— 用来核对"这个域到底是不是 DN" */
        double *cp = (double *)malloc((size_t)bw * bh * sizeof(double));
        if (cp != NULL) {
            memcpy(cp, blk, (size_t)bw * bh * sizeof(double));
            /* 排序取分位（块数不多，插入排序够了） */
            int n = bw * bh, a, b;
            for (a = 1; a < n; a++) {
                double v = cp[a];
                for (b = a - 1; b >= 0 && cp[b] > v; b--) cp[b + 1] = cp[b];
                cp[b + 1] = v;
            }
            fprintf(stderr, "[nr]   块值(DN) 10%%=%.0f 50%%=%.0f 90%%=%.0f  max=%.0f\n",
                    cp[n / 10], cp[n / 2], cp[n * 9 / 10], cp[n - 1]);
            free(cp);
        }
    }
    for (by = 0; by < bh; by++) {
        for (bx = 0; bx < bw; bx++) {
            double mn = blk[(size_t)by * bw + bx];
            int dy, dx, k;
            for (dy = -1; dy <= 1; dy++) {
                int yy = by + dy;
                if (yy < 0 || yy >= bh) continue;
                for (dx = -1; dx <= 1; dx++) {
                    int xx = bx + dx;
                    double v;
                    if (xx < 0 || xx >= bw) continue;
                    v = blk[(size_t)yy * bw + xx];
                    if (v < mn) mn = v;
                }
            }
            if      (mn >= thr[0]) k = 0;
            else if (mn >= thr[1]) k = 1;
            else if (mn >= thr[2]) k = 2;
            else if (mn >= thr[3]) k = 3;
            else                   k = 4;
            rmap[(size_t)by * bw + bx] = (unsigned char)rad[k];
        }
    }
    free(blk);
}

/* mode 0 = 逐通道（辉度）；mode 1 = 只动色度（YCbCr 的 Cb/Cr，Y 不变） */
static void nr_bilateral2d_row(void *vctx, size_t yy)
{
    const nr_bil_ctx_t *c = (const nr_bil_ctx_t *)vctx;
    const int w = c->w, h = c->h, radius = c->radius, mode = c->mode;
    const float *src = c->src;
    const float *gate = c->gate;
    float *dst = c->dst;
    double k = c->k, fs = c->fs;
    int y = (int)yy, x, dy, dx;

    {
        for (x = 0; x < w; x++) {
            const float *p = src + ((size_t)y * w + x) * 3;
            float *o = dst + ((size_t)y * w + x) * 3;
            double inv[3], ycc0[3], acc[3] = { 0.0, 0.0, 0.0 }, sw = 0.0;
            int radius = c->radius;
            int y0 = (y - radius < 0) ? 0 : y - radius;
            int y1 = (y + radius >= h) ? h - 1 : y + radius;
            int x0 = (x - radius < 0) ? 0 : x - radius;
            int x1 = (x + radius >= w) ? w - 1 : x + radius;

            if (c->rmap != NULL) {
                radius = c->rmap[(size_t)(y / c->block) * c->bw + (x / c->block)];
                y0 = (y - radius < 0) ? 0 : y - radius;
                y1 = (y + radius >= h) ? h - 1 : y + radius;
                x0 = (x - radius < 0) ? 0 : x - radius;
                x1 = (x + radius >= w) ? w - 1 : x + radius;
            }
            nr_inv3(p, fs, k, inv, c->coef);
            if (mode) nr_rgb2ycc(p, ycc0);
            for (dy = y0; dy <= y1; dy++) {
                for (dx = x0; dx <= x1; dx++) {
                    const float *q = src + ((size_t)dy * w + dx) * 3;
                    double u = 0.0, wt;
                    int i;
                    if (mode && c->uvgate) {
                        /* **色度**距离做门（`X3F_NR_CH_UVGATE=1`）。
                         *
                         * 原来两支共用 RGB 距离：而 RGB 差被**亮度**主导，于是
                         * "亮度噪声大的地方色度权重≈0" —— 色度噪声被当细节挡在门外，
                         * 只在亮度平坦的地方才抹色度，等于**磨色度结构**。
                         * 实测（DP0Q2315，ISO1600，色度滑块）：
                         *   可去色（逐像素噪声）只降 15%/22%，而 down=4 的平坦区
                         *   色度 HF 降了 28%/62% ⇒ 确实在磨结构、不去噪声。
                         * 换成 UV 距离（σ 用 σ_R²+σ_G² 的量级）后色度门才对得上
                         * "色度像不像"。 */
                        double c[3];
                        double sc = 0.5 * (inv[0] + inv[1]);
                        nr_rgb2ycc(q, c);
                        u = sc * ((c[1] - ycc0[1]) * (c[1] - ycc0[1])
                                + (c[2] - ycc0[2]) * (c[2] - ycc0[2]));
                    } else {
                        for (i = 0; i < 3; i++) {
                            double d = (double)q[i] - (double)p[i];
                            u += inv[i] * d * d;
                        }
                    }
                    if (u > 700.0) continue;
                    if (c->shape > 0.0) {
                        /*range LUT 形状：`bfLUT[i] = 31·(i/14407)^4 + 1`
                         *（ `0xd8970..0xd8a24`，14385 项，末项强制 32.0）。
                         * 当惩罚用 ⇒ 权重 = 1/(1+LUT)：u 小的时候几乎不衰减、
                         * 到 shape 附近才**陡降**，和 exp(−u) 那种"一路平缓衰减"
                         * 是两种门限（见 §17.20.1）。 */
                        double t = u / c->shape;
                        wt = 1.0 / (1.0 + 31.0 * t * t * t * t);
                    } else {
                        wt = exp(-u);
                    }
                    /*亮部软门：按**邻居自己**的信号收权（见 LUT_A 注释）。
                     * 只乘中心会被归一化约掉，所以必须取邻居的值；
                     * 但**中心自己不能门** —— 否则一片饱和区里中心和各邻居的门
                     * 同时趋近 0，权重和塌成 0，那个像素会被"留一"成黑点
                     * （实测 max|Δ| 会飙到 248/255，就是这批点）。 */
                    if (gate != NULL && (dx != x || dy != y))
                        wt *= (double)gate[(size_t)dy * w + dx];
                    sw += wt;
                    if (mode) {
                        double c[3];
                        nr_rgb2ycc(q, c);
                        acc[1] += wt * c[1];
                        acc[2] += wt * c[2];
                    } else {
                        acc[0] += wt * q[0];
                        acc[1] += wt * q[1];
                        acc[2] += wt * q[2];
                    }
                }
            }
            if (sw <= 1e-9) sw = 1.0;
            if (mode) {
                ycc0[1] = acc[1] / sw;
                ycc0[2] = acc[2] / sw;
                nr_ycc2rgb(ycc0, o);
            } else {
                o[0] = (float)(acc[0] / sw);
                o[1] = (float)(acc[1] / sw);
                o[2] = (float)(acc[2] / sw);
            }
        }
    }
}

static void nr_bilateral2d(float *dst, const float *src, int w, int h,
                           int radius, double k, double fs, int mode,
                           const unsigned char *rmap, int block, int bw)
{
    nr_bil_ctx_t c;
    float *gate = NULL;
    double gfs = fs;

    /* 亮部软门（WFA 的 LUT_A）。默认关；`X3F_NR_WFA=1` 打开。
     * 索引域默认用**图像域**（和 σ² 用的噪声域 fs 不是一回事，别混）——
     * 见 nr_wfa_lut() 上面的说明。 */
    if (nr_flag("X3F_NR_WFA", 0)) {
        const char *e = getenv("X3F_NR_WFA_FS");
        gfs = (e != NULL && atof(e) > 1.0) ? atof(e) : nr_fullscale();
        gate = (float *)malloc((size_t)w * h * sizeof(float));
        if (gate != NULL) {
            nr_build_wfa_gate(src, w, h, gfs, gate);
        } else if (getenv("X3F_NR_DEBUG")) {
            fprintf(stderr, "[nr] WFA 门：内存不足，按关处理\n");
        }
        if (getenv("X3F_NR_DEBUG") && gate != NULL)
            fprintf(stderr, "[nr] 亮部门 on：%s 域=%.0f 模式=%s\n",
                    (e != NULL) ? "X3F_NR_WFA_FS" : "图像域", gfs,
                    mode ? "色度" : "辉度");
    }
    c.coef = mode ? g_nr.cn_iso_c
                  : (g_nr_level_coef != NULL ? g_nr_level_coef : g_nr.cn_iso);
    /* UV 门：实测（DP0Q2315）与 RGB 门**没有差别**（−6.1% vs −5.8%），默认关。
     * 留着是因为它对应的"物理直觉"（色度该按色度距离判相似）仍然是对的，
     * 换成别的机身/别的尺度时值得再试一次。 */
    c.uvgate = mode ? nr_flag("X3F_NR_CH_UVGATE", 0) : 0;
    c.shape = nr_env("X3F_NR_BFSHAPE", 0.0);
    c.dst = dst; c.src = src; c.w = w; c.h = h;
    c.radius = radius; c.mode = mode; c.k = k; c.fs = fs;
    c.gate = gate;
    c.rmap = rmap; c.block = block; c.bw = bw;
    x3f_parallel_for((size_t)h, nr_bilateral2d_row, &c);
    free(gate);
}

/* ---------------------------------------------------------------- */
/* ===================== SPP F23Stage1 的六步（2026-10-15） =====================
 *
 * 公式全部来自 §17.9（那一节是" +
 * 实测"双重确认的，§17.10 有对拍数据）：
 *
 *   ① 双边      —— 已有 `nr_bilateral2d`（每层的 σ 缩放 + 半径）
 *   ② 边缘图    —— r×r 邻域里"差异不超过 3σ_c"的**邻居占比** ∈[0,1]（r = 11）
 *   ③ 局部均值  —— **边缘加权的对称归一 EMA**：W = 0.9·边缘图，
 *                  num/den 各扫一遍正反，out = (F + B − x)/(dF + dB − 1)
 *   ④ 权重图    —— `clamp(10·(max(r,1/r) − 1), 0, 1)`，r = 8 邻域亮度均值/中心亮度
 *   ⑤ 权重混合  —— 权重图的 3×3 帐篷模糊 + 亮/暗软门，把 A 混向 P
 *   ⑥ 大混合    —— `BlendImages`：按亮度 LUT 的逐像素混合
 *
 * **单位**：SPP 那几个常数（9000 / 2000 / 14448 截断 / 451.5 偏置）都在它自己的
 * DN 域里；我们这边按"scene 白点 = 14448"换算（`nr_s1_dn()`），所以这些常数可以
 * 原样抄 —— 这是本文件里唯一一处**没有实测校准**的换算，若整段偏亮/偏暗就先调它。
 * 域里的 3σ² 阈值则用噪声模型自己的域（`fs`），和双边那条一致。 */

#define NR_S1_R       11          /* 边缘图窗口（实测 r = 11、分母 120） */
#define NR_S1_SIGMA   0.9         /* 局部均值的每步保留率（params+0x50，实测） */
#define NR_S1_LUT_N   14344       /* 亮表项数（实测：0xe020 字节 / 4） */
#define NR_S1_DN      14448.0     /*亮度索引上限*/

static inline double nr_s1_dn(double v) { return v * NR_S1_DN; }

/* LUT_A[i] = (1 − i/14346.9)^64（实测：表长与索引上限都指向这个函数） */
static double nr_s1_lut_a(int i)
{
    double t;
    if (i < 0) i = 0;
    if (i > NR_S1_LUT_N - 1) i = NR_S1_LUT_N - 1;
    t = 1.0 - (double)i / 14346.9;
    if (t < 0.0) t = 0.0;
    return pow(t, 64.0);
}

/* LUT_B = tanh(−3i/1022)（1024 项，实测；§17.5 的差分支用它） */
static double nr_s1_lut_b(int i)
{
    if (i < 0) i = 0;
    if (i > 1023) i = 1023;
    return tanh(-3.0 * (double)i / 1022.0);
}

/* ② 边缘图：每像素一个 [0,1]，平坦 = 1、有纹理/边 = 0。
 *    coef = 该层的噪声系数（**已按 s² 缩放过**，见 nr_s1_level），v 用噪声域。 */
static void nr_s1_edgemap(float *mv, const float *rgb, int w, int h,
                          const double *coef, double fs, double scale)
{
    const int RR = NR_S1_R / 2, NN = NR_S1_R * NR_S1_R - 1;
    int x, y, dy, dx, c;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            const float *p = rgb + ((size_t)y * w + x) * 3;
            double v[3], t[3];
            int cnt = 0;
            for (c = 0; c < 3; c++) {
                double q;
                v[c] = (double)p[c] * fs;
                q = coef[c] + coef[3 + c] * v[c] + coef[6 + c] * v[c] * v[c];
                t[c] = scale * (q < 0.0 ? -q : q);
            }
            for (dy = -RR; dy <= RR; dy++) {
                int yy = y + dy;
                if (yy < 0) yy = 0;
                if (yy > h - 1) yy = h - 1;
                for (dx = -RR; dx <= RR; dx++) {
                    int xx = x + dx;
                    const float *q;
                    if (dx == 0 && dy == 0) continue;
                    if (xx < 0) xx = 0;
                    if (xx > w - 1) xx = w - 1;
                    q = rgb + ((size_t)yy * w + xx) * 3;
                    for (c = 0; c < 3; c++) {
                        double d = (double)q[c] * fs - v[c];
                        if (d * d > t[c]) { cnt++; break; }
                    }
                }
            }
            mv[(size_t)y * w + x] = (float)(1.0 - (double)cnt / (double)NN);
        }
    }
}

/* ③ 局部均值：沿一条轴做"边缘加权的对称归一 EMA"。
 *    递推 y ← x + W·y_prev（W = σ_param·mask），正反各一遍，
 *    out = (F + B − x)/(dF + dB − 1) —— 就是 §17.9.4 那三行（实测冲激响应吻合）。 */
static void nr_s1_ema_axis(float *dst, const float *src, const float *mask,
                           int w, int h, double sigma, int vertical)
{
    int n = vertical ? h : w, L = vertical ? w : h, i, l, c;
    float *fn = (float *)malloc((size_t)n * 3 * sizeof(float));
    float *fd = (float *)malloc((size_t)n * sizeof(float));
    if (fn == NULL || fd == NULL) { free(fn); free(fd); return; }
    for (l = 0; l < L; l++) {
        double num[3] = { 0, 0, 0 }, den = 0.0;
        for (i = 0; i < n; i++) {
            size_t q = vertical ? ((size_t)i * w + l) : ((size_t)l * w + i);
            const float *px = src + q * 3;
            double W = sigma * (double)mask[q];
            for (c = 0; c < 3; c++) {
                num[c] = (double)px[c] + W * num[c];
                fn[i * 3 + c] = (float)num[c];
            }
            den = 1.0 + W * den;
            fd[i] = (float)den;
        }
        num[0] = num[1] = num[2] = 0.0;
        den = 0.0;
        for (i = n - 1; i >= 0; i--) {
            size_t q = vertical ? ((size_t)i * w + l) : ((size_t)l * w + i);
            const float *px = src + q * 3;
            float *o = dst + q * 3;
            double W = sigma * (double)mask[q];
            for (c = 0; c < 3; c++) {
                num[c] = (double)px[c] + W * num[c];
                o[c] = (float)(((double)fn[i * 3 + c] + num[c] - (double)px[c])
                               / ((double)fd[i] + den - 1.0));
            }
            den = 1.0 + W * den;
        }
    }
    free(fn);
    free(fd);
}

/* ③ 两遍（先横后竖，掩膜同一张 —— 实测两次调用的 mask1 就是掩膜的转置拷贝） */
static void nr_s1_localmean(float *dst, const float *src, const float *mask,
                            int w, int h, double sigma)
{
    float *tmp = (float *)malloc((size_t)w * h * 3 * sizeof(float));
    if (tmp == NULL) { memcpy(dst, src, (size_t)w * h * 3 * sizeof(float)); return; }
    nr_s1_ema_axis(tmp, src, mask, w, h, sigma, 0);
    nr_s1_ema_axis(dst, tmp, mask, w, h, sigma, 1);
    free(tmp);
}

/* ④ 权重图 = 局部对比度：r = 8 邻域亮度均值 / 中心亮度，w = clamp(10·(max(r,1/r)−1)) */
static void nr_s1_w1(float *w1, const float *lum, int w, int h)
{
    int x, y;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            double sum = 0.0, lc = 0.0, r, q, wv;
            int dy, dx;
            lc = lum[(size_t)y * w + x];
            for (dy = -1; dy <= 1; dy++) {
                int yy = y + dy;
                if (yy < 0) yy = 0;
                if (yy > h - 1) yy = h - 1;
                for (dx = -1; dx <= 1; dx++) {
                    int xx = x + dx;
                    if (dx == 0 && dy == 0) continue;
                    if (xx < 0) xx = 0;
                    if (xx > w - 1) xx = w - 1;
                    sum += lum[(size_t)yy * w + xx];
                }
            }
            sum /= 8.0;
            if (sum <= 0.0 || lc <= 0.0) { w1[(size_t)y * w + x] = 0.0f; continue; }
            r = sum / lc;
            q = (r > 1.0 / r) ? r : 1.0 / r;
            wv = 10.0 * (q - 1.0);
            if (wv < 0.0) wv = 0.0;
            if (wv > 1.0) wv = 1.0;
            w1[(size_t)y * w + x] = (float)wv;
        }
    }
}

/* ⑤ 权重图的帐篷模糊 + 软门混合：out = w·A + (1−w)·P
 *    w = clamp(ramp(luma(O)) + min(conv(B), 1 − LUT_A[luma(A)]), 0, 1) */
static void nr_s1_w2(float *a, const float *b, const float *p, const float *o,
                     int w, int h)
{
    int x, y, dy, dx, c;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            size_t i = (size_t)y * w + x;
            double conv = 0.0, la, lo, ramp, s, wv, o3[3];
            static const double tent[3][3] = { { .25, .5, .25 }, { .5, 1.0, .5 }, { .25, .5, .25 } };
            for (dy = -1; dy <= 1; dy++) {
                int yy = y + dy;
                if (yy < 0) yy = 0;
                if (yy > h - 1) yy = h - 1;
                for (dx = -1; dx <= 1; dx++) {
                    int xx = x + dx;
                    if (xx < 0) xx = 0;
                    if (xx > w - 1) xx = w - 1;
                    conv += tent[dy + 1][dx + 1] * (double)b[(size_t)yy * w + xx];
                }
            }
            conv *= 0.25;
            if (conv < 0.0) conv = 0.0;
            if (conv > 1.0) conv = 1.0;
            la = nr_s1_dn(nr_lum3(a + i * 3));
            s = 1.0 - nr_s1_lut_a((int)(la < 1.0 ? 1.0 : la));
            s = (s < 0.0) ? 0.0 : ((s > 1.0) ? 1.0 : s);
            lo = nr_s1_dn(nr_lum3(o + i * 3));
            ramp = (lo - 9000.0) / 2000.0;
            ramp = (ramp < 0.0) ? 0.0 : ((ramp > 1.0) ? 1.0 : ramp);
            wv = ramp + ((conv < s) ? conv : s);
            wv = (wv < 0.0) ? 0.0 : ((wv > 1.0) ? 1.0 : wv);
            for (c = 0; c < 3; c++) {
                o3[c] = wv * (double)a[i * 3 + c] + (1.0 - wv) * (double)p[i * 3 + c];
            }
            for (c = 0; c < 3; c++) a[i * 3 + c] = (float)o3[c];
        }
    }
}

/* ⑥ BlendImages（§17.5）：先按**目的图**的亮度查表得权重，再按两张图的差修正。
 *    这里只做第一级（亮度 LUT）—— 差分支在 §17.5 里就带着"索引尺度与表长对不上"
 *    的疑点（49152 vs 1024），先不引进来；等那一条定了再补。 */
static void nr_s1_blend_images(float *dst, const float *src, int w, int h)
{
    size_t i, n = (size_t)w * h;
    for (i = 0; i < n; i++) {
        double mx = dst[i * 3], wv;
        int k, idx, c;
        if (dst[i * 3 + 1] > mx) mx = dst[i * 3 + 1];
        if (dst[i * 3 + 2] > mx) mx = dst[i * 3 + 2];
        k = (int)(nr_s1_dn(mx) - 451.5);
        if (k < 1) k = 1;
        if (k > NR_S1_DN) k = (int)NR_S1_DN;
        idx = k - 1;
        wv = nr_s1_lut_a(idx);
        if (wv < 0.0) wv = 0.0;
        if (wv > 1.0) wv = 1.0;
        for (c = 0; c < 3; c++)
            dst[i * 3 + c] = (float)(wv * (double)src[i * 3 + c] + (1.0 - wv) * (double)dst[i * 3 + c]);
    }
}

/* ===== 亮度专用版（§17.19）：②~⑥ 只作用在亮度上，色度保持双边那条的产物 =====
 *
 * 动机：逐通道 RGB 跑整套（旧版）会把色度顺带抹两遍 —— 实测 DP0Q2315 档 0.5
 * 色度响应 −41.6%，而 SPP 只有 −21.1%（§17.18）。六步大概只作用在亮度上。*/

/* ② 亮度版边缘图：σ² 用"三通道在同一个 v 上求值再平均 /9"（亮度 = (R+G+B)/3） */
static void nr_s1_edgemap_y(float *mv, const float *lum, int w, int h,
                            const double *coef, double fs, double scale)
{
    const int RR = NR_S1_R / 2, NN = NR_S1_R * NR_S1_R - 1;
    int x, y, dy, dx, c;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            size_t i = (size_t)y * w + x;
            double v = (double)lum[i] * fs, t = 0.0;
            int cnt = 0;
            for (c = 0; c < 3; c++)
                t += coef[c] + coef[3 + c] * v + coef[6 + c] * v * v;
            t = scale * (t < 0.0 ? -t : t) / 9.0;
            for (dy = -RR; dy <= RR; dy++) {
                int yy = y + dy;
                if (yy < 0) yy = 0;
                if (yy > h - 1) yy = h - 1;
                for (dx = -RR; dx <= RR; dx++) {
                    int xx = x + dx;
                    double d;
                    if (dx == 0 && dy == 0) continue;
                    if (xx < 0) xx = 0;
                    if (xx > w - 1) xx = w - 1;
                    d = (double)lum[(size_t)yy * w + xx] * fs - v;
                    if (d * d > t) cnt++;
                }
            }
            mv[i] = (float)(1.0 - (double)cnt / (double)NN);
        }
    }
}

/* ③ 亮度版局部均值（单通道 EMA，正反两遍；公式同 3 通道版） */
static void nr_s1_ema_y(float *dst, const float *src, const float *mask,
                        int w, int h, double sigma, int vertical)
{
    int n = vertical ? h : w, L = vertical ? w : h, i, l;
    float *fn = (float *)malloc((size_t)n * sizeof(float));
    float *fd = (float *)malloc((size_t)n * sizeof(float));
    if (fn == NULL || fd == NULL) { free(fn); free(fd); return; }
    for (l = 0; l < L; l++) {
        double num = 0.0, den = 0.0, W;
        for (i = 0; i < n; i++) {
            size_t q = vertical ? ((size_t)i * w + l) : ((size_t)l * w + i);
            W = sigma * (double)mask[q];
            num = (double)src[q] + W * num;
            fn[i] = (float)num;
            den = 1.0 + W * den;
            fd[i] = (float)den;
        }
        num = 0.0;
        den = 0.0;
        for (i = n - 1; i >= 0; i--) {
            size_t q = vertical ? ((size_t)i * w + l) : ((size_t)l * w + i);
            W = sigma * (double)mask[q];
            num = (double)src[q] + W * num;
            den = 1.0 + W * den;
            dst[q] = (float)(((double)fn[i] + num - (double)src[q])
                             / ((double)fd[i] + den - 1.0));
        }
    }
    free(fn);
    free(fd);
}

static void nr_s1_localmean_y(float *dst, const float *src, const float *mask,
                              int w, int h, double sigma)
{
    float *tmp = (float *)malloc((size_t)w * h * sizeof(float));
    if (tmp == NULL) { memcpy(dst, src, (size_t)w * h * sizeof(float)); return; }
    nr_s1_ema_y(tmp, src, mask, w, h, sigma, 0);
    nr_s1_ema_y(dst, tmp, mask, w, h, sigma, 1);
    free(tmp);
}

/* ⑤ 亮度版：out = w·A + (1−w)·P，权重同 RGB 版（帐篷模糊 + 亮/暗软门） */
static void nr_s1_w2_y(float *a, const float *b, const float *p, const float *o,
                       int w, int h)
{
    static const double tent[3][3] = { { .25, .5, .25 }, { .5, 1.0, .5 }, { .25, .5, .25 } };
    int x, y, dy, dx;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            size_t i = (size_t)y * w + x;
            double conv = 0.0, la, lo, ramp, s, wv;
            for (dy = -1; dy <= 1; dy++) {
                int yy = y + dy;
                if (yy < 0) yy = 0;
                if (yy > h - 1) yy = h - 1;
                for (dx = -1; dx <= 1; dx++) {
                    int xx = x + dx;
                    if (xx < 0) xx = 0;
                    if (xx > w - 1) xx = w - 1;
                    conv += tent[dy + 1][dx + 1] * (double)b[(size_t)yy * w + xx];
                }
            }
            conv *= 0.25;
            if (conv < 0.0) conv = 0.0;
            if (conv > 1.0) conv = 1.0;
            la = nr_s1_dn((double)a[i]);
            s = 1.0 - nr_s1_lut_a((int)(la < 1.0 ? 1.0 : la));
            s = (s < 0.0) ? 0.0 : ((s > 1.0) ? 1.0 : s);
            lo = nr_s1_dn((double)o[i]);
            ramp = (lo - 9000.0) / 2000.0;
            ramp = (ramp < 0.0) ? 0.0 : ((ramp > 1.0) ? 1.0 : ramp);
            wv = ramp + ((conv < s) ? conv : s);
            wv = (wv < 0.0) ? 0.0 : ((wv > 1.0) ? 1.0 : wv);
            a[i] = (float)(wv * (double)a[i] + (1.0 - wv) * (double)p[i]);
        }
    }
}

/* ⑥ 亮度版 BlendImages（同一级：亮度 LUT 的逐像素混合） */
static void nr_s1_blend_images_y(float *dst, const float *src, int w, int h)
{
    size_t i, n = (size_t)w * h;
    for (i = 0; i < n; i++) {
        int k = (int)(nr_s1_dn((double)dst[i]) - 451.5);
        double wv;
        if (k < 1) k = 1;
        if (k > (int)NR_S1_DN) k = (int)NR_S1_DN;
        wv = nr_s1_lut_a(k - 1);
        if (wv < 0.0) wv = 0.0;
        if (wv > 1.0) wv = 1.0;
        dst[i] = (float)(wv * (double)src[i] + (1.0 - wv) * (double)dst[i]);
    }
}

/* 把亮度增益乘回 RGB（保色度**比**）。
 *
 * 试过"在 YCbCr 里只换 Y、Cb/Cr 原样"那版：数学上更干净，但暗部会变成**加性偏移**
 * 再被 sRGB 曲线放大 —— 实测亮度响应从 −38% 掉到 −58%（过强），所以用乘性增益。 */
static void nr_s1_apply_gain(float *rgb, const float *y0, const float *y1,
                             int w, int h)
{
    size_t i, n = (size_t)w * h;
    for (i = 0; i < n; i++) {
        double g = (y0[i] > 1e-7f) ? (double)y1[i] / (double)y0[i] : 1.0;
        if (g < 0.0) g = 0.0;
        rgb[i * 3 + 0] = (float)(rgb[i * 3 + 0] * g);
        rgb[i * 3 + 1] = (float)(rgb[i * 3 + 1] * g);
        rgb[i * 3 + 2] = (float)(rgb[i * 3 + 2] * g);
    }
}

/*多尺度（ F23Stage1NoiseReduction）—— **原型，X3F_NR_PYR=1 打开**
 *
 * Stage1 不是一次双边（见 §3、§6）：
 *
 * 尺度表={ 1.0, 0.5, 0.25 } → 三层
 *   每层：ComputeResampleParams(scale) 降采样 → 迭代{双边 → 边缘图 → 局部均值 →
 *         权重滤波 → 混合} → ResampleImage 升采样
 * 合并：逐层按 1/σ² 累加（`F23Stage1NoiseReduction` 起那段 fdiv/fmul）
 * 每层的 σ 用 `CalculateNoiseVar`的同一形式：
 *         σ²_L = s²·(a + b·(v/s) + c·(v/s)²) = s²a + s·b·v + c·v²
 *
 * 这一版**只做"三层双边 + 按 1/σ² 合并"**：每层的边缘图/局部均值/混合那几步
 * 还没读透（§8.3 的老账），所以这里没有它们 —— 也就是这是**结构的近似**，
 * 不是逐行还原。目的：先量出"多尺度到底能不能把高 ISO 的 1.33× 与
 * 低 ISO 的曲线形状一起收掉"，能不能再决定要不要把那几步也读下来。
 *
 * 为什么值得试：单程双边在门全开时退化成"盒平均"（高 ISO 干净但糊，§16.5）；
 * 多尺度是把"粗层多平均、细层保边"分开，天然缓解这个矛盾。
 */
static void nr_pyr_down2(const float *src, int w, int h, float *dst, int *dwo, int *dho)
{
    int dw = (w + 1) / 2, dh = (h + 1) / 2, x, y, dx, dy;
    *dwo = dw; *dho = dh;
    for (y = 0; y < dh; y++) {
        for (x = 0; x < dw; x++) {
            double acc[3] = { 0, 0, 0 };
            int n = 0;
            for (dy = 0; dy < 2; dy++) {
                int yy = y * 2 + dy;
                if (yy >= h) break;
                for (dx = 0; dx < 2; dx++) {
                    const float *p;
                    int xx = x * 2 + dx;
                    if (xx >= w) break;
                    p = src + ((size_t)yy * w + xx) * 3;
                    acc[0] += p[0]; acc[1] += p[1]; acc[2] += p[2];
                    n++;
                }
            }
            if (n == 0) n = 1;
            dst[((size_t)y * dw + x) * 3 + 0] = (float)(acc[0] / n);
            dst[((size_t)y * dw + x) * 3 + 1] = (float)(acc[1] / n);
            dst[((size_t)y * dw + x) * 3 + 2] = (float)(acc[2] / n);
        }
    }
}

/* 双线性升采样（像素中心对齐，和 nr_chroma_half 里那套同一个口径） */
static void nr_pyr_up2(const float *src, int sw, int sh, float *dst, int w, int h)
{
    int x, y;
    for (y = 0; y < h; y++) {
        double fy = (y - 0.5) * 0.5, ty;
        int y0, y1;
        if (fy < 0) fy = 0;
        if (fy > sh - 1) fy = sh - 1;
        y0 = (int)fy; y1 = (y0 + 1 < sh) ? y0 + 1 : y0;
        ty = fy - y0;
        for (x = 0; x < w; x++) {
            double fx = (x - 0.5) * 0.5, tx;
            int x0, x1, c;
            const float *a, *b, *cc, *d;
            float *o = dst + ((size_t)y * w + x) * 3;
            if (fx < 0) fx = 0;
            if (fx > sw - 1) fx = sw - 1;
            x0 = (int)fx; x1 = (x0 + 1 < sw) ? x0 + 1 : x0;
            tx = fx - x0;
            a = src + ((size_t)y0 * sw + x0) * 3;
            b = src + ((size_t)y0 * sw + x1) * 3;
            cc = src + ((size_t)y1 * sw + x0) * 3;
            d = src + ((size_t)y1 * sw + x1) * 3;
            for (c = 0; c < 3; c++) {
                double t = (a[c] * (1 - tx) + b[c] * tx) * (1 - ty)
                         + (cc[c] * (1 - tx) + d[c] * tx) * ty;
                o[c] = (float)t;
            }
        }
    }
}

/* 三层双边 + 逐像素 1/σ² 合并。coef = cn_iso（含 ISO 增益）。
 * 层 σ：σ²_L = s²·a + s·b·v + c·v²（ CalculateNoiseVar 形式）。*/
static int nr_pyramid_luma(float *dst, const float *src, int w, int h,
                           int radius, double k, double fs,
                           const unsigned char *rmap, int block, int bw)
{
    static const double sc[3] = { 1.0, 0.5, 0.25 };
    float *lv[3] = { NULL, NULL, NULL }, *fil[3] = { NULL, NULL, NULL };
    float *up[3] = { NULL, NULL, NULL };
    const float *lvin[3] = { src, NULL, NULL };
    int lw[3] = { w, 0, 0 }, lh[3] = { h, 0, 0 };
    int L, x, y, ok = 0;
    /*合并权重模式：0 = 1/σ²（累加形式，粗层权重大 → 实测过糊）；
     * 1 = 均匀；2 = σ²（等价于"按方差加权"，粗层权重小 → 接近只用细层）。
     * 做成开关是为了量出"多尺度的收益到底在哪一段" —— 正式实现应该照
     * SPP 每层里的边缘图/局部均值/混合来做，而不是靠调这个权重。 */
    /* 合并权重 = σ²^(2t−1) 的连续插值：t=0 → 1/σ²（粗层主导，实测 −59%）、
     * t=0.5 → 均匀（−44%）、t=1 → σ²（细层主导，−19.5%）；SPP 参考是 −34%/−21%
     * ⇒ 拟合落在 0.5~0.9 之间（§17.17.2、§17.18）。老写法 0/1/2 依次对应 t=0/0.5/1。 */
    double wmode = nr_env("X3F_NR_PYR_W", 0.7);
    if (wmode < 0.0) wmode = 0.0;
    if (wmode > 1.0) wmode = 1.0;

    lv[0] = (float *)malloc((size_t)w * h * 3 * sizeof(float));
    fil[0] = (float *)malloc((size_t)w * h * 3 * sizeof(float));
    up[0] = (float *)malloc((size_t)w * h * 3 * sizeof(float));
    if (lv[0] == NULL || fil[0] == NULL || up[0] == NULL) goto done;
    memcpy(lv[0], src, (size_t)w * h * 3 * sizeof(float));

    for (L = 1; L < 3; L++) {
        int dw, dh;
        dw = (lw[L - 1] + 1) / 2; dh = (lh[L - 1] + 1) / 2;
        if (dw < 8 || dh < 8) break;               /* 太小的层不要 */
        lv[L] = (float *)malloc((size_t)dw * dh * 3 * sizeof(float));
        fil[L] = (float *)malloc((size_t)dw * dh * 3 * sizeof(float));
        up[L] = (float *)malloc((size_t)w * h * 3 * sizeof(float));
        if (lv[L] == NULL || fil[L] == NULL || up[L] == NULL) goto done;
        nr_pyr_down2(lv[L - 1], lw[L - 1], lh[L - 1], lv[L], &lw[L], &lh[L]);
    }

    for (L = 0; L < 3; L++) {
        double s2 = sc[L] * sc[L], coef_s[9];
        int c9;
        float *ev = NULL, *lm = NULL, *w1 = NULL, *y0 = NULL, *ya = NULL;
        size_t np = (size_t)lw[L] * lh[L];
        int c;
        if (lv[L] == NULL) break;
        for (c9 = 0; c9 < 9; c9++)
            coef_s[c9] = (g_pyr_chroma_mode ? g_nr.cn_iso_c[c9] : g_nr.cn_iso[c9]) * s2;
        /* ① 双边（每层一套 σ：半径按像素给 → 覆盖随层数翻倍） */
        /* ★ 逐层 σ（`X3F_NR_PYR_LSIG=1`）：用该层缩放的系数（s²a, s·b, c）跑双边，
         * 与 `m_fBFNoiseScaling[L]` 同一个意思 —— 粗层自己的噪声被放大，
         * 所以它的 σ 也该更大。（原来三层共用一个 σ。） */
        g_nr_level_coef = nr_flag("X3F_NR_PYR_LSIG", 1) ? coef_s : NULL;
        /* ★ 色度模式：逐层 σ 乘子 m_L（SPP NR3 的 base[L]/BFS[L]，§22.1）——
         * range = k/σ_L，σ_L = σ·m_L → 传 k/m_L。粗层 σ 相对更大 =
         * 色斑在粗层磨掉、细层保纹理（SPP"色度滑杆高档去斑不糊"的结构根源）。 */
        {
            double kL = k;
            if (g_pyr_bfs_mul != NULL) kL = k / g_pyr_bfs_mul[L];
            /* 色度模式必须走 mode=1（只动 U/V）——mode=0 连亮度一起磨，
             * 实测细节保留 100.9%→59%（2026-10-27 首版踩坑）。 */
            nr_bilateral2d(fil[L], lv[L], lw[L], lh[L], radius, kL, fs,
                           g_pyr_chroma_mode ? 1 : 0,
                           (L == 0) ? rmap : NULL, block, bw);
        }
        g_nr_level_coef = NULL;
        if (nr_flag("X3F_NR_S1", 1) && !g_pyr_chroma_mode) {
            ev = (float *)malloc(np * sizeof(float));
            lm = (float *)malloc(np * 3 * sizeof(float));
            w1 = (float *)malloc(np * sizeof(float));
            y0 = (float *)malloc(np * sizeof(float));
            ya = (float *)malloc(np * sizeof(float));
        }
        if (ev != NULL && lm != NULL && w1 != NULL && y0 != NULL && ya != NULL) {
            /* ② 边缘图（系数按该层 s² 缩：实测整块 9 个 double 一起乘；coef_s 已算好） */
            if (nr_flag("X3F_NR_S1_LUMA", 1)) {
                /* **亮度专用**（默认）：②~⑥ 只在亮度上跑，色度保持双边那条的产物
                 * （实测：逐通道跑整套会让色度响应比 SPP 强 2~3 倍，见 §17.18） */
                nr_s1_luma_plane(y0, fil[L], lw[L], lh[L]);
                nr_s1_edgemap_y(ev, y0, lw[L], lh[L], coef_s, fs, 3.0);
                nr_s1_localmean_y(lm, y0, ev, lw[L], lh[L], NR_S1_SIGMA);
                nr_s1_w1(w1, lm, lw[L], lh[L]);
                /* ⚠️ 2026-10-15 试过把 A/P 与 ramp 源一起翻过来（"亮部保持原样、
                 * 暗部换成均值"）：**两个指标同时变差**（纹理 3.54→2.56、
                 * 平坦暗部 1.23→8.18），所以维持这一版接法（A = 局部均值、
                 * P = 双边后的层图、ramp 源 = 局部均值）。见 §17.25。 */
                memcpy(ya, y0, np * sizeof(float));
                nr_s1_w2_y(lm, w1, y0, lm, lw[L], lh[L]);
                nr_s1_blend_images_y(lm, y0, lw[L], lh[L]);
                nr_s1_apply_gain(fil[L], y0, lm, lw[L], lh[L]);
            } else {
                /* 旧版：整套逐通道 RGB（`X3F_NR_S1_LUMA=0`，留作对照） */
                nr_s1_edgemap(ev, fil[L], lw[L], lh[L], coef_s, fs, 3.0);
                nr_s1_localmean(lm, fil[L], ev, lw[L], lh[L], NR_S1_SIGMA);
                nr_s1_w1(w1, lm, lw[L], lh[L]);
                nr_s1_w2(lm, w1, fil[L], lm, lw[L], lh[L]);
                nr_s1_blend_images(lm, fil[L], lw[L], lh[L]);
                memcpy(fil[L], lm, np * 3 * sizeof(float));
            }
        }
        free(ev); free(lm); free(w1); free(y0); free(ya);
        if (L == 0) memcpy(up[0], fil[0], (size_t)w * h * 3 * sizeof(float));
        else        nr_pyr_up2(fil[L], lw[L], lh[L], up[L], w, h);
    }

    /* 逐像素合并：w_L = 1/σ²_L（σ²_L 用该层的缩放系数，v 取全分辨率的像素值） */
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            size_t i3 = ((size_t)y * w + x) * 3;
            double sw = 0.0, acc[3] = { 0, 0, 0 };
            int c;
            for (L = 0; L < 3; L++) {
                double s = sc[L], wl = 0.0;
                if (up[L] == NULL) continue;
                for (c = 0; c < 3; c++) {
                    double v = (double)src[i3 + c] * fs;
                    const double *cnm = g_pyr_chroma_mode ? g_nr.cn_iso_c : g_nr.cn_iso;
                    double a = cnm[c], b = cnm[3 + c], cc = cnm[6 + c];
                    double var = s * s * a + s * b * v + cc * v * v;
                    if (var < 1e-9) var = 1e-9;
                    wl += pow(var, 2.0 * wmode - 1.0);   /* t=0→1/σ²、0.5→均匀、1→σ² */
                }
                wl /= 3.0;
                sw += wl;
                for (c = 0; c < 3; c++) acc[c] += wl * (double)up[L][i3 + c];
            }
            if (sw <= 1e-12) { dst[i3] = src[i3]; dst[i3 + 1] = src[i3 + 1]; dst[i3 + 2] = src[i3 + 2]; continue; }
            dst[i3 + 0] = (float)(acc[0] / sw);
            dst[i3 + 1] = (float)(acc[1] / sw);
            dst[i3 + 2] = (float)(acc[2] / sw);
            if (g_pyr_chroma_mode) {
                /* 色度模式：把**源的亮度**锚回（709，与 nr_rgb2ycc/nrab 口径
                 * 一致）。三层各通道平滑程度不同，逐通道合并会连带磨亮度
                 * （实测细节保留 100.9%→59.8%，2026-10-27）——SPP 色度滑杆
                 * 不碰亮度（§20.2：chroma 只进 m_fBFNoiseScaling）。
                 *
                 * 等色相去饱和（gamut mapping）：out = Y_src + t·(merged−Y_m)，
                 * t 取保持全通道 ∈[0,1] 的最大值。Y 逐位精确；色域边界处只降
                 * 饱和、色相不变。前两版踩坑记录：①601 注光剩 27% 损失（601/709
                 * 串扰）；②709 等量平移 dy 有 27% 像素越界，下游裁剪吃掉亮度
                 * HF，dy 夹持又在边界像素上反过来注入亮度噪声（暗部亮度 HF
                 * 33.6→38.9）。 */
                double ys = 0.2126 * src[i3] + 0.7152 * src[i3 + 1] + 0.0722 * src[i3 + 2];
                double ym = 0.2126 * dst[i3] + 0.7152 * dst[i3 + 1] + 0.0722 * dst[i3 + 2];
                double t = 1.0;
                int cc;
                for (cc = 0; cc < 3; cc++) {
                    double dv = (double)dst[i3 + cc] - ym;
                    if (dv > 1e-12) { double tt = (1.0 - ys) / dv; if (tt < t) t = tt; }
                    else if (dv < -1e-12) { double tt = ys / (-dv); if (tt < t) t = tt; }
                }
                if (t < 0.0) t = 0.0;
                dst[i3]     = (float)(ys + t * ((double)dst[i3]     - ym));
                dst[i3 + 1] = (float)(ys + t * ((double)dst[i3 + 1] - ym));
                dst[i3 + 2] = (float)(ys + t * ((double)dst[i3 + 2] - ym));
            }
        }
    }
    ok = 1;
    if (g_pyr_chroma_mode && getenv("X3F_NR_PYR_CHK")) {
        /* 临时校验：合并+注回后 Y709 应与源同族（float 误差 ~1e-7 量级）。 */
        double mx = 0, acc = 0;
        size_t n = (size_t)w * h, i, oob = 0;
        for (i = 0; i < n; i++) {
            size_t i3 = i * 3;
            int c;
            double ys = 0.2126 * src[i3] + 0.7152 * src[i3 + 1] + 0.0722 * src[i3 + 2];
            double yo = 0.2126 * dst[i3] + 0.7152 * dst[i3 + 1] + 0.0722 * dst[i3 + 2];
            double d = fabs(yo - ys);
            if (d > mx) mx = d;
            acc += d;
            for (c = 0; c < 3; c++)
                if (dst[i3 + c] < 0.0f || dst[i3 + c] > 1.0f) { oob++; break; }
        }
        fprintf(stderr, "[nr][chk] Y709 注回后偏差 max=%.3g mean=%.3g 越界像素=%.2f%% (n=%zu)\n",
                mx, acc / (double)n, 100.0 * (double)oob / (double)n, n);
    }
done:
    for (L = 0; L < 3; L++) { free(lv[L]); free(fil[L]); free(up[L]); }
    if (!ok) memcpy(dst, src, (size_t)w * h * 3 * sizeof(float));
    return ok;
}

/*色度在**半分辨率**上做 —— Stage1 就是这么干的
 * （里成对出现的 `DownSample` / `UpSample` + "Scaled BF kernel"）。
 *
 * 两个理由：
 * ① 更贴原物：色度工作本来就在降采样后的图上做；
 *   ② 色度窗最大（半径到 10 ≈ 21×21），半分辨率直接省 4× —— 预览拖滑块才不会顿。
 *
 * 步骤：2×2 盒式降采样 → 在低分辨率的 Cb/Cr 上跑同一套噪声归一化双边 →
 * 双线性升采样回来。**只替换 Cb/Cr，Y 用原始全分辨率的那份** ——
 * 这样亮度细节一点没丢，色度该模糊的本来也不需要那么高的分辨率。
 */
static void nr_chroma_half(float *dst, const float *src, int w, int h,
                           int radius, double k, double fs)
{
    int dw = (w + 1) / 2, dh = (h + 1) / 2;
    float *half, *sm, *cc;
    int x, y, dy, dx;

    /* 噪声方差要跟着降采样走：2×2 盒式平均把**方差**压到 1/4（σ 减半），
     * 而邻域差 Δ 也跟着变小 —— 同一个 k 直接搬过去会变成"u 只有原来的 1/4"，
     * 权重整片贴近 1，等于把色度糊平（实测 Cb 高频掉 47%，全分辨率才 2%）。
     * 所以这里把 k 乘 4 抵消掉：u = 4k·Δ_half²/σ² 与全分辨率的 k·Δ_full²/σ² 同量级。
     * SPP 那边是 `SetNoiseScalingForISO*` 一族在干同样的事（按缩放级别调噪声参数）。 */
    k *= 4.0;
    /* 窗宽也要按半分辨率折算：半径 10 在半分辨率上是 20 像素的覆盖（全分辨率的
     * 两倍），配上上面那个 4 倍会让权重饱和成"纯盒式平均" —— 实测把 k 从 0.01
     * 扫到 8 结果纹丝不动（Cb 一律 −47%），就是这么饱和的。
     * 折半之后覆盖与全分辨率一致，k×4 才真正起到补偿噪声的作用。 */
    radius = (radius + 1) / 2;
    if (radius < 1) radius = 1;

    if (dw < 2 || dh < 2) {
        /* 太小的图不值得走这条路 */
        nr_bilateral2d(dst, src, w, h, radius, k, fs, 1, NULL, 0, 0);
        return;
    }
    half = (float *)malloc((size_t)dw * dh * 3 * sizeof(float));
    sm   = (float *)malloc((size_t)dw * dh * 3 * sizeof(float));
    cc   = (float *)malloc((size_t)dw * dh * 2 * sizeof(float));
    if (half == NULL || sm == NULL || cc == NULL) {
        free(half); free(sm); free(cc);
        nr_bilateral2d(dst, src, w, h, radius, k, fs, 1, NULL, 0, 0);
        return;
    }

    /* 2×2 盒式降采样 */
    for (y = 0; y < dh; y++) {
        for (x = 0; x < dw; x++) {
            double acc[3] = { 0.0, 0.0, 0.0 };
            int n = 0;
            float *o = half + ((size_t)y * dw + x) * 3;
            for (dy = 0; dy < 2; dy++) {
                int yy = y * 2 + dy;
                if (yy >= h) break;
                for (dx = 0; dx < 2; dx++) {
                    int xx = x * 2 + dx;
                    const float *p;
                    if (xx >= w) break;
                    p = src + ((size_t)yy * w + xx) * 3;
                    acc[0] += p[0]; acc[1] += p[1]; acc[2] += p[2];
                    n++;
                }
            }
            if (n == 0) n = 1;
            o[0] = (float)(acc[0] / n);
            o[1] = (float)(acc[1] / n);
            o[2] = (float)(acc[2] / n);
        }
    }

    /* 半分辨率上的色度双边 */
    nr_bilateral2d(sm, half, dw, dh, radius, k, fs, 1, NULL, 0, 0);

    /* 先把低分辨率的 Cb/Cr 摊平（后面双线性取样就不用反复 RGB→YCC） */
    for (y = 0; y < dh * dw; y++) {
        double c[3];
        nr_rgb2ycc(sm + (size_t)y * 3, c);
        cc[(size_t)y * 2]     = (float)c[1];
        cc[(size_t)y * 2 + 1] = (float)c[2];
    }

    /* 升采样：Cb/Cr 取双线性，Y 取原图 */
    for (y = 0; y < h; y++) {
        double fy = ((double)y + 0.5) / 2.0 - 0.5;
        int y0 = (int)floor(fy);
        double ty = fy - y0;
        if (y0 < 0) { y0 = 0; ty = 0.0; }
        if (y0 > dh - 2) { y0 = dh - 2; if (ty > 1.0) ty = 1.0; }
        for (x = 0; x < w; x++) {
            double fx = ((double)x + 0.5) / 2.0 - 0.5;
            int x0 = (int)floor(fx);
            double tx = fx - x0, cb, cr, ycc0[3];
            const float *p = src + ((size_t)y * w + x) * 3;
            float *o = dst + ((size_t)y * w + x) * 3;
            size_t i00, i10;
            if (x0 < 0) { x0 = 0; tx = 0.0; }
            if (x0 > dw - 2) { x0 = dw - 2; if (tx > 1.0) tx = 1.0; }
            i00 = ((size_t)y0 * dw + x0) * 2;
            i10 = ((size_t)(y0 + 1) * dw + x0) * 2;
            cb = ((cc[i00]     * (1.0 - tx) + cc[i00 + 2] * tx) * (1.0 - ty) +
                  (cc[i10]     * (1.0 - tx) + cc[i10 + 2] * tx) * ty);
            cr = ((cc[i00 + 1] * (1.0 - tx) + cc[i00 + 3] * tx) * (1.0 - ty) +
                  (cc[i10 + 1] * (1.0 - tx) + cc[i10 + 3] * tx) * ty);
            nr_rgb2ycc(p, ycc0);          /* 只借它的 Y */
            ycc0[1] = cb;
            ycc0[2] = cr;
            nr_ycc2rgb(ycc0, o);
        }
    }
    free(half); free(sm); free(cc);
}

/*方向性双边（条纹）。dir: 0 = 沿行（匹配列偏移， MatchColOffsets），
 * 1 = 沿列。dst 可为 NULL 表示就地写回 src。 */
typedef struct {
    float       *dst;
    const float *src;
    int          w, h, half, dir;
    double       strength, fs;
} nr_band_ctx_t;

static void nr_banding_row(void *vctx, size_t yy)
{
    const nr_band_ctx_t *c = (const nr_band_ctx_t *)vctx;
    const int w = c->w, h = c->h, half = c->half, dir = c->dir;
    const float *src = c->src;
    float *dst = c->dst;
    double strength = c->strength, fs = c->fs;
    int y = (int)yy, x, k;

    {
        for (x = 0; x < w; x++) {
            const float *p = src + ((size_t)y * w + x) * 3;
            double sig[3], inv[3], sw = 0.0, acc[3];
            int i;

            for (i = 0; i < 3; i++) {
                sig[i] = nr_var(&g_nr, i, (double)p[i] * fs);
                if (sig[i] < 1e-6) sig[i] = 1e-6;
                /* 单位要一致：σ² 是按**原始 DN** 标定的，而这里的 Δ 是 scene(0..1)。
                 * 所以 Δ²/σ² 要么把 σ² 除 fs²、要么把 Δ 乘 fs —— 等价于把 fs² 提到
                 * inv 上（第一版漏了这一步，结果整幅被当噪声抹平）。 */
                inv[i] = strength * fs * fs / sig[i];
                acc[i] = 0.0;
            }
            for (k = -half; k <= half; k++) {
                int xx = x, yy = y;
                const float *q;
                double u = 0.0, wt;
                if (dir == 0) { xx = x + k; if (xx < 0 || xx >= w) continue; }
                else          { yy = y + k; if (yy < 0 || yy >= h) continue; }
                q = src + ((size_t)yy * w + xx) * 3;
                for (i = 0; i < 3; i++) {
                    double d = (double)q[i] - (double)p[i];
                    u += inv[i] * d * d;
                }
                if (u > 700.0) continue;          /* exp(−700) 已是 0 */
                wt = exp(-u);
                sw += wt;
                acc[0] += wt * q[0];
                acc[1] += wt * q[1];
                acc[2] += wt * q[2];
            }
            {
                float *o = dst + ((size_t)y * w + x) * 3;
                if (sw > 1e-9) {
                    o[0] = (float)(acc[0] / sw);
                    o[1] = (float)(acc[1] / sw);
                    o[2] = (float)(acc[2] / sw);
                } else {
                    o[0] = p[0]; o[1] = p[1]; o[2] = p[2];
                }
            }
        }
    }
}

static void nr_banding_pass(float *dst, const float *src, int w, int h,
                            int half, double strength, double fs, int dir)
{
    nr_band_ctx_t c;
    c.dst = dst; c.src = src; c.w = w; c.h = h;
    c.half = half; c.dir = dir; c.strength = strength; c.fs = fs;
    x3f_parallel_for((size_t)h, nr_banding_row, &c);
}

/* ---------------------------------------------------------------- */
/* Stage2 的色度中值（SPP：F23Stage2NoiseReduction 里那条 **无条件** 的处理）
 *
 * 【实证】`F23Stage2NoiseReduction`的流程是
 *      高光中和 → RawBlending(ColorDQ) → **XDoYUVMedian**
 * 三条都只受一个"取消"标志影响（`F23Stage2NoiseReduction(): Cancelled.`），
 * **不吃任何滑块、也不看 UI 设置** —— 也就是说它是 SPP 出图基线的一部分。
 *
 * `XDoYUVMedian`读下来是：
 *   RGB→YUV（`XDoRGB2YUV`）→ 对 **U/V 逐通道做 ±3（7×7）中值**，
 *   **Y 原样穿过** → YUV→RGB 写回：
 *     · 窗口：行/列各 ±3（`subs w8, w9, #3` / `add w11, w11, #1` 那组），
 *       边界把索引 clamp 回图内；
 * · 通道 0（Y）在进窗口前只读了一次（`ldp s1, s2, [x0]` ），
 *       循环里只更新 U/V 的序统计量，末尾重建：
 *           out0 = Y + 1.13983·V
 *           out1 = Y − 0.39465·U − 0.58060·V
 *           out2 = Y + 2.03211·U
 *       三个常数就是从字面量拼出来的 YUV→RGB 系数（1.13983 / 0.39465 /
 *       0.58060 / 2.03211），所以**亮度一点没动**，动的只有色度。
 * · （Merrill）里**没有** `XDoYUVMedian` / `UV_ImageFilter` / `XDoRGB2YUV`
 *     这三个符号 —— 这条是 F23（Quattro）专属。
 *
 * 为什么要抄它：拿 DP0Q1000 在**平坦区**（掩膜取自 SPP 参考，60.3% 像素）量，
 * 色度高频 mean|Δ(R−G)| 是
 *       SPP more(档4) 0.065 · 我们 色度满档 0.84 · 加这条中值后 0.10
 * 也就是说"色度差一个数量级"里的大头，就是这一条我们一直没做的东西。
 * （整幅指标追不平是另一回事：那被真实彩色边缘主导，见 §10 的视觉结论。）
 *
 * ★ 一个实测出来的、必须写下来的差异：**放在哪一段做差别巨大**。
 *   按 SPP 原样放在**线性域**（tone curve 之前）：
 *       平坦区色度 HF 0.77 → 0.54   （线性域自身的色度高频降了 7~9×，但成图几乎没动）
 *   放在**显示域**（tone curve 之后、锐化之前）：
 *       平坦区色度 HF 0.77 → 0.09   （SPP 参考 0.065）
 *   把线性域的中值半径从 7×7 加到 15×15，成图只从 0.76 动到 0.71 —— 也就是说
 *   成图里的色度高频**不是**从线性域的色度高频换算过来的，它是 tone curve
 *   逐通道作用时与色度互调出来的（曲线在暗部斜率随亮度抖，把亮度噪声乘进色度）。
 * 推论：在线性域再怎么滤都够不着 平坦度，所以这一版**放在显示域**。
 *   （SPP 敢放在它的 Stage2 线性域，多半是因为它那套 FColorFloat 工作域本来就
 *   接近显示域 —— 噪声模型域标定出来是 ~1024、S 曲线中心 9000，都不是我们这种
 *   0..1 线性量纲。这条只是【推断】，没有直接证据。）
 *
 * 位置：tone curve 之后、锐化之前 —— 与 SPP「Stage2 中值 → Stage2B 锐化」同序，
 * 而且锐化只动 Y，不会把色度高频再加回来。
 *
 * 默认开（半径 3 = 7×7，与 SPP 一致）。`X3F_CHRMED=0` 关，
 * `X3F_CHRMED_R=<n>` 换半径，用来做 A/B。 */
#define CHRMED_MAX_R 7                       /* ±7 → 15×15，够用了 */
#define CHRMED_BUF   ((2 * CHRMED_MAX_R + 1) * (2 * CHRMED_MAX_R + 1))

typedef struct {
    const float *src;                        /* 归一化 0..1，RGB 交错 */
    float       *dst;
    /* 提速（2026-10-22）：预计算的色度平面 U=B−Y / V=R−Y（逐像素只算一次
     * 亮度，值与旧版逐邻域重算逐位一致——同一表达式）。行内循环不再做
     * 2×(2r+1)² 次 Rec709 点积，只剩窗口收集 + 两次中值选择。 */
    const float *upl;                        /* U = B − Y，w*h */
    const float *vpl;                        /* V = R − Y，w*h */
    int          w, h, r;
    /* 行并行搬运（2026-10-23）用的显示缓冲指针/步长/位深 */
    void        *src_out;
    int          stride, bits;
} chrmed_ctx_t;

/* GPU 重建行 worker 的上下文（chrmed_rebuild_row） */
typedef struct {
    const float *src, *omu, *omv;
    void        *buf;
    int w, stride, bits;
} chrmed_rebuild_ctx_t;

/* 就地 nth_element（只要中位数，n 很小，双指针分区足够） */
static float chrmed_select(float *a, int n, int k)
{
    int lo = 0, hi = n - 1;
    while (lo < hi) {
        float pivot = a[(lo + hi) >> 1];
        int i = lo, j = hi;
        while (i <= j) {
            while (a[i] < pivot) i++;
            while (a[j] > pivot) j--;
            if (i <= j) { float t = a[i]; a[i] = a[j]; a[j] = t; i++; j--; }
        }
        if (k <= j)      hi = j;
        else if (k >= i) lo = i;
        else             break;
    }
    return a[k];
}

static void chrmed_row(void *vctx, size_t yy)
{
    const chrmed_ctx_t *c = (const chrmed_ctx_t *)vctx;
    const int w = c->w, h = c->h, r = c->r;
    const float *upl = c->upl, *vpl = c->vpl;
    const float *src = c->src;
    float *dst = c->dst;
    int y = (int)yy, x, dy, dx;
    float bu[CHRMED_BUF], bv[CHRMED_BUF];

    for (x = 0; x < w; x++) {
        const float *p = src + ((size_t)y * w + x) * 3;
        double y0 = 0.2126 * (double)p[0] + 0.7152 * (double)p[1] + 0.0722 * (double)p[2];
        double cb, cr;
        int y0r = (y - r < 0) ? 0 : y - r, y1r = (y + r >= h) ? h - 1 : y + r;
        int x0r = (x - r < 0) ? 0 : x - r, x1r = (x + r >= w) ? w - 1 : x + r;
        int n = 0;
        float *o;

        for (dy = y0r; dy <= y1r; dy++) {
            const float *urow = upl + (size_t)dy * w;
            const float *vrow = vpl + (size_t)dy * w;
            for (dx = x0r; dx <= x1r; dx++) {
                bu[n] = urow[dx];              /* U = B − Y */
                bv[n] = vrow[dx];              /* V = R − Y */
                n++;
            }
        }
        cb = chrmed_select(bu, n, n >> 1);
        cr = chrmed_select(bv, n, n >> 1);

        o = dst + ((size_t)y * w + x) * 3;
        o[0] = (float)(y0 + cr);
        o[2] = (float)(y0 + cb);
        o[1] = (float)(y0 - (0.2126 * cr + 0.0722 * cb) / 0.7152);
    }
}

/*半径：0 = 关。默认 3（±3 = 7×7，窗口）。*/
static int chrmed_radius(void)
{
    int r;
    if (!nr_flag("X3F_CHRMED", 1)) return 0;
    r = (int)nr_env("X3F_CHRMED_R", 3.0);
    /* ★ ISO 自适应（2026-10-15 实测）——低 ISO 保持 3，高 ISO 加大。
     *
     * 起因：用户实测"辉度滑块拉到最左（=关）细节可以，但彩噪满天；往右一点就涂抹"。
     * 量下来彩噪那半是**色度中值半径不够**：DP0Q2315（ISO 1600）、滑块全关、
     * 只动中值半径 ——
     *      r=3(7×7)  暗部彩噪 7.28
     *      r=6(13×13)           3.42
     *      r=10(21×21)          2.88
     * 而**纹理处亮度 HF 一个点没动**（23.75 → 23.84）—— 这条是色度专用，
     * 加大半径在"亮度细节"上是免费的（代价是小面积彩色特征会被抹，这个要靠看图定）。
     * 所以：ISO ≤ 400 用 3（与改动前逐字节相同），≥ 800 用 6，≥ 3200 用 8。 */
    if (r == 3 && getenv("X3F_CHRMED_R") == NULL) {
        double iso = g_shooting_iso;
        if (iso >= 3200.0) r = 8;
        else if (iso >= 800.0) r = 6;
    }
    if (r < 1) r = 1;
    if (r > CHRMED_MAX_R) r = CHRMED_MAX_R;
    return r;
}

/* 显示域色度中值：在**已经过 tone curve / 色彩匹配**的成图上做，
 * 原地改（bits = 8 或 16），Y 不动、只换 U/V。 */
/* ---- 行并行搬运（2026-10-23）：逐像素算式不变，输出逐位一致 ---- */
static void chrmed_load_row(void *v, size_t yi)
{
    chrmed_ctx_t *c = (chrmed_ctx_t *)v;
    const int y = (int)yi;
    const void *row = (const uint8_t *)c->src_out + (size_t)y * c->stride;
    float *s = (float *)c->src;      /* 只在本 worker 写，读侧仍是只读语义 */
    const int w = c->w, bits = c->bits;
    int x;
    for (x = 0; x < w; x++) {
        size_t o = ((size_t)y * w + x) * 3;
        s[o]     = (float)px_get(row, bits, (size_t)x * 3);
        s[o + 1] = (float)px_get(row, bits, (size_t)x * 3 + 1);
        s[o + 2] = (float)px_get(row, bits, (size_t)x * 3 + 2);
    }
}

static void chrmed_uv_row(void *v, size_t yi)
{
    chrmed_ctx_t *c = (chrmed_ctx_t *)v;
    const int y = (int)yi, w = c->w;
    const float *src = c->src;
    float *upl = c->upl, *vpl = c->vpl;
    int x;
    for (x = 0; x < w; x++) {
        size_t o = ((size_t)y * w + x) * 3, i = (size_t)y * w + x;
        double yq = 0.2126 * (double)src[o] + 0.7152 * (double)src[o + 1]
                  + 0.0722 * (double)src[o + 2];
        upl[i] = (float)((double)src[o + 2] - yq);   /* U = B − Y */
        vpl[i] = (float)((double)src[o]     - yq);   /* V = R − Y */
    }
}

/* GPU 重建行 worker（原 gdone 分支的循环体） */
static void chrmed_rebuild_row(void *v, size_t yi)
{
    chrmed_rebuild_ctx_t *c = (chrmed_rebuild_ctx_t *)v;
    const int y = (int)yi, w = c->w, bits = c->bits;
    const float *src = c->src, *omu = c->omu, *omv = c->omv;
    void *row = (uint8_t *)c->buf + (size_t)y * c->stride;
    int x;
    for (x = 0; x < w; x++) {
        size_t o = ((size_t)y * w + x) * 3, i = (size_t)y * w + x;
        double y0 = 0.2126 * (double)src[o] + 0.7152 * (double)src[o + 1]
                  + 0.0722 * (double)src[o + 2];
        double cb = omu[i], cr = omv[i];
        px_put(row, bits, (size_t)x * 3,     (float)(y0 + cr));
        px_put(row, bits, (size_t)x * 3 + 2, (float)(y0 + cb));
        px_put(row, bits, (size_t)x * 3 + 1,
               (float)(y0 - (0.2126 * cr + 0.0722 * cb) / 0.7152));
    }
}

/* CPU 回退路径的 dst 回写行 worker（原 !gdone 分支的循环体） */
static void chrmed_back_row(void *v, size_t yi)
{
    chrmed_ctx_t *c = (chrmed_ctx_t *)v;
    const int y = (int)yi, w = c->w;
    const float *dst = c->dst;
    void *row = (uint8_t *)c->src_out + (size_t)y * c->stride;
    int x;
    for (x = 0; x < w; x++) {
        size_t o = ((size_t)y * w + x) * 3;
        px_put(row, c->bits, (size_t)x * 3,     dst[o]);
        px_put(row, c->bits, (size_t)x * 3 + 1, dst[o + 1]);
        px_put(row, c->bits, (size_t)x * 3 + 2, dst[o + 2]);
    }
}

/* ── 行带版色度中值（2026-11-08）────────────────────────────────────────
 *
 * 算的东西与整帧版**逐个像素相同**（同一套 U/V 定义、同一个窗口、同一个
 * 第 n/2 小值、同一句重建），区别只在内存布局：
 *
 *   整帧版：float src（n*3*4 = 1.2GB@100MP）+ 两条整幅 U/V 平面（0.8GB）
 *           + 输出 dst（已改为复用 src）。导出时调用方的线性帧（1.2GB）
 *           还躺在那儿，一段就顶到 5GB。
 *   行带版：一条带（bh 行 + 上下 r 行 halo）的 RGB 与 U/V，约 60–100MB。
 *
 * 为什么可以分带：重建只读**本像素**的 RGB（y0）与中值后的 U/V；窗口只
 * 向下/上扩 r 行。所以一条带只要带上 halo 就自洽。唯一的坑是「本带写完
 * 后，下一带还要用已写过的行当 halo」—— 每带写之前把尾部 r 行原始 RGB
 * 存进 snap，下一带从左 halo 取它，逐位不受影响。
 */
typedef struct {
    const float *rgb;      /* 带内原始 RGB（未修改） */
    const float *upl, *vpl;
    void *buf;
    int stride, bits, w, h, r;
    int origin;            /* 带第 0 行对应的图像行号 */
    int ky0, ky1;          /* 本带要产出的行 [ky0, ky1)（图像坐标） */
} chrmed_band_ctx_t;

static void chrmed_band_row(void *vctx, size_t kk)
{
    const chrmed_band_ctx_t *c = (const chrmed_band_ctx_t *)vctx;
    const int w = c->w, h = c->h, r = c->r, origin = c->origin;
    const float *rgb = c->rgb, *upl = c->upl, *vpl = c->vpl;
    const int y = c->ky0 + (int)kk;
    const int ly = y - origin;                 /* 带内行号 */
    float bu[CHRMED_BUF], bv[CHRMED_BUF];
    int x;

    for (x = 0; x < w; x++) {
        const float *p = rgb + ((size_t)ly * w + x) * 3;
        const double y0 = 0.2126 * (double)p[0] + 0.7152 * (double)p[1]
                        + 0.0722 * (double)p[2];
        int y0r = (y - r < 0) ? 0 : y - r, y1r = (y + r >= h) ? h - 1 : y + r;
        int x0r = (x - r < 0) ? 0 : x - r, x1r = (x + r >= w) ? w - 1 : x + r;
        int n = 0, dy, dx;
        double cb, cr;

        for (dy = y0r; dy <= y1r; dy++) {
            const float *urow = upl + (size_t)(dy - origin) * w;
            const float *vrow = vpl + (size_t)(dy - origin) * w;
            for (dx = x0r; dx <= x1r; dx++) {
                bu[n] = urow[dx];
                bv[n] = vrow[dx];
                n++;
            }
        }
        cb = chrmed_select(bu, n, n >> 1);
        cr = chrmed_select(bv, n, n >> 1);
        {
            void *row = (uint8_t *)c->buf + (size_t)y * c->stride;
            px_put(row, c->bits, (size_t)x * 3,     (float)(y0 + cr));
            px_put(row, c->bits, (size_t)x * 3 + 2, (float)(y0 + cb));
            px_put(row, c->bits, (size_t)x * 3 + 1,
                   (float)(y0 - (0.2126 * cr + 0.0722 * cb) / 0.7152));
        }
    }
}

/* GPU 分带的「要不要试」缓存：-1 未试 / 1 可用 / 0 已确认不可用。
 * 一旦某带失败就永久回 CPU，避免每带都白试一遍。 */
static int chrmed_gpu_state = -1;

static int chrmed_gpu_wanted(void)
{
    if (chrmed_gpu_state >= 0) return chrmed_gpu_state;
#if defined(HAVE_OWN_GPU) && HAVE_OWN_GPU
    {
        /* X3F_CHRMED_IMPL=cpu 时强制纯 CPU，用于与 GPU 版对拍 */
        const char *e = getenv("X3F_CHRMED_IMPL");
        const int force_cpu = (e != NULL && (*e == 'c' || *e == 'C'));
        chrmed_gpu_state = (!force_cpu && x3f_render_gpu_chrmed_enabled()) ? 1 : 0;
    }
#else
    chrmed_gpu_state = 0;
#endif
    return chrmed_gpu_state;
}

typedef struct {
    const float *rgb, *omu, *omv;
    void *buf;
    int stride, bits, w, origin, ky0;
} chrmed_band_gpu_ctx_t;

/* 重建：中值 U/V 已由 GPU 算好。注意两套行号不能混：
 *   rgb 是**带内**数组（从 origin 起，含上下 halo）→ 用 y − origin
 *   omu/omv 只回传 [ry0,ry1) 那段，第 0 行 = 图像行 ky0 → 用 y − ky0
 * 混用会让中间带整体错开 r 行（首带因为 origin == ky0 才侥幸正确）。 */
static void chrmed_band_row_gpu(void *vctx, size_t kk)
{
    const chrmed_band_gpu_ctx_t *c = (const chrmed_band_gpu_ctx_t *)vctx;
    const int w = c->w;
    const int y = c->ky0 + (int)kk;
    const int lr = y - c->origin;   /* 带内 RGB 行 */
    const int lm = y - c->ky0;      /* 中值行 */
    const float *rp  = c->rgb + (size_t)lr * w * 3;
    const float *gu  = c->omu + (size_t)lm * w;
    const float *gv  = c->omv + (size_t)lm * w;
    void *row = (uint8_t *)c->buf + (size_t)y * c->stride;
    int x;

    for (x = 0; x < w; x++) {
        const float *p = rp + (size_t)x * 3;
        const double y0 = 0.2126 * (double)p[0] + 0.7152 * (double)p[1]
                        + 0.0722 * (double)p[2];
        const double cr = (double)gv[x];   /* R − Y */
        const double cb = (double)gu[x];   /* B − Y */
        px_put(row, c->bits, (size_t)x * 3,     (float)(y0 + cr));
        px_put(row, c->bits, (size_t)x * 3 + 2, (float)(y0 + cb));
        px_put(row, c->bits, (size_t)x * 3 + 1,
               (float)(y0 - (0.2126 * cr + 0.0722 * cb) / 0.7152));
    }
}

static int apply_chroma_median_banded(int r, void *buf, int w, int h, int stride, int bits)
{
    if (w <= 0 || h <= 0 || r <= 0) return X3FRENDER_OK;
    const size_t per_row = (size_t)w * (3 + 2) * sizeof(float);
    size_t want = (size_t)(96u << 20) / (per_row ? per_row : 1);
    if (want < (size_t)(2 * r + 16)) want = (size_t)(2 * r + 16);
    int bh = (int)want - 2 * r;
    if (bh < 16) bh = 16;
    /* 对齐 16：chrmed_sel kernel 是 16×16 的 threadgroup，靠 threadgroup_barrier
     * 协作加载窗口瓦片。整幅调用时 h 是 8192/4096（天然 16 的倍数），带高却是
     * 随预算算出来的 403 这种数 —— 末列 threadgroup 里 g.y >= bh 的线程如果
     * 在 barrier 前提前返回，瓦片就少一块，出来的中值会偏。整幅版从没暴露
     * 过这个问题，分带必须自己保证。 */
    bh &= ~15;
    if (bh < 16) bh = 16;
    /* 诊断：X3F_CHRMED_BAND=4096 把带高钉死（设成 ≥ 图高即退化成整幅单带），
     * 用来分辨"分带边界算错"还是"GPU 调用本身不对"。 */
    {
        const char *be = getenv("X3F_CHRMED_BAND");
        if (be != NULL && *be != '\0') {
            int v = atoi(be);
            if (v >= 16) { bh = v & ~15; if (bh < 16) bh = 16; }
        }
    }

    const int cap = bh + 2 * r;
    float *rgb = (float *)malloc((size_t)cap * w * 3 * sizeof(float));
    float *upl = (float *)malloc((size_t)cap * w * sizeof(float));
    float *vpl = (float *)malloc((size_t)cap * w * sizeof(float));
    float *snap = (float *)malloc((size_t)r * w * 3 * sizeof(float));
    /* 中值结果按**带**分配（bh 行），不是按带 + halo：GPU 只回传要用的行。 */
    float *omu = NULL, *omv = NULL;
    if (chrmed_gpu_wanted()) {
        omu = (float *)malloc((size_t)bh * w * sizeof(float));
        omv = (float *)malloc((size_t)bh * w * sizeof(float));
        if (omu == NULL || omv == NULL) { free(omu); free(omv); omu = omv = NULL; }
    }
    if (rgb == NULL || upl == NULL || vpl == NULL || snap == NULL) {
        /* 与整帧版一致：分配失败静默跳过这一级（不影响出图） */
        free(rgb); free(upl); free(vpl); free(snap); free(omu); free(omv);
        return X3FRENDER_OK;
    }

    int y0 = 0;
    while (y0 < h) {
        int y1 = y0 + bh; if (y1 > h) y1 = h;
        const int left = (y0 < r) ? y0 : r;            /* 左 halo 行数 */
        const int bot = (y1 + r > h) ? h : y1 + r;     /* 带 + 右 halo 的止行 */
        const int origin = y0 - left;
        const int nrows = bot - origin;
        int y;

        for (y = origin; y < bot; y++) {
            const float *srcrow;
            if (y < y0) srcrow = snap + (size_t)(y - origin) * w * 3;   /* 上一带存的原始行 */
            else        srcrow = NULL;
            if (srcrow != NULL) {
                memcpy(rgb + (size_t)(y - origin) * w * 3, srcrow,
                       (size_t)w * 3 * sizeof(float));
            } else {
                const uint8_t *brow = (const uint8_t *)buf + (size_t)y * stride;
                float *d = rgb + (size_t)(y - origin) * w * 3;
                int x;
                for (x = 0; x < w; x++) {
                    d[x * 3]     = (float)px_get(brow, bits, (size_t)x * 3);
                    d[x * 3 + 1] = (float)px_get(brow, bits, (size_t)x * 3 + 1);
                    d[x * 3 + 2] = (float)px_get(brow, bits, (size_t)x * 3 + 2);
                }
            }
        }
        /* U/V 平面（带内） */
        for (y = origin; y < bot; y++) {
            const float *sp = rgb + (size_t)(y - origin) * w * 3;
            float *up = upl + (size_t)(y - origin) * w;
            float *vp = vpl + (size_t)(y - origin) * w;
            int x;
            for (x = 0; x < w; x++) {
                const size_t o = (size_t)x * 3;
                const double yq = 0.2126 * (double)sp[o] + 0.7152 * (double)sp[o + 1]
                                + 0.0722 * (double)sp[o + 2];
                up[x] = (float)((double)sp[o + 2] - yq);   /* U = B − Y */
                vp[x] = (float)((double)sp[o]     - yq);   /* V = R − Y */
            }
        }
        /* 写之前把尾部 r 行的**原始** RGB 存进 snap（下一带的左 halo） */
        {
            const int keep = (bot - y1 < r) ? (bot - y1) : r;
            const int start = y1 - keep;
            if (keep > 0)
                memcpy(snap, rgb + (size_t)(start - origin) * w * 3,
                       (size_t)keep * w * 3 * sizeof(float));
        }
        /* 重建本带：优先 GPU 算中值（一带来回，几十 MB），失败切 CPU 选择。 */
        {
            int gpu_ok = 0;
            if (omu != NULL && omv != NULL) {
                const int ry0 = y0 - origin, ry1 = y1 - origin;
                if (own_gpu_chrmed_plane_rows(upl, omu, w, nrows, ry0, ry1, r) == 0
                    && own_gpu_chrmed_plane_rows(vpl, omv, w, nrows, ry0, ry1, r) == 0) {
                    gpu_ok = 1;
                } else {
                    chrmed_gpu_state = 0;   /* 本进程后续全走 CPU，不再白试 */
                    if (getenv("X3F_NR_DEBUG"))
                        fprintf(stderr, "[nr] 色度中值 GPU 分带失败，回 CPU 选择\n");
                }
            }
            if (gpu_ok) {
                chrmed_band_gpu_ctx_t gc;
                gc.rgb = rgb; gc.omu = omu; gc.omv = omv;
                gc.buf = buf; gc.stride = stride; gc.bits = bits;
                gc.w = w; gc.origin = origin; gc.ky0 = y0;
                x3f_parallel_for((size_t)(y1 - y0), chrmed_band_row_gpu, &gc);
            } else {
                chrmed_band_ctx_t bc;
                bc.rgb = rgb; bc.upl = upl; bc.vpl = vpl;
                bc.buf = buf; bc.stride = stride; bc.bits = bits;
                bc.w = w; bc.h = h; bc.r = r; bc.origin = origin;
                bc.ky0 = y0; bc.ky1 = y1;
                x3f_parallel_for((size_t)(y1 - y0), chrmed_band_row, &bc);
            }
        }
        y0 = y1;
    }

    free(rgb); free(upl); free(vpl); free(snap); free(omu); free(omv);
    return X3FRENDER_OK;
}

/* 旧的整帧实现（只在 GPU 色度中值开启时使用，见下面的分发） */
static int apply_chroma_median_full(int r, void *buf, int w, int h, int stride, int bits)
{
    chrmed_ctx_t c;
    float *src, *dst, *upl, *vpl;
    size_t n3 = (size_t)w * h * 3, np = (size_t)w * h;
    int y, x;

    src = (float *)malloc(n3 * sizeof(float));
    /* dst 曾经是**另一整帧** n3 float（100MP = 1.2GB）。重建是逐像素的
     * （chrmed_row 只读本像素的 src、只写本像素的 dst），所以 dst 直接复用
     * src：写进去的就是同一块内存，逐位结果不变。GPU 路径不经过 dst（直接
     * 写 buf），CPU 路径原地重建后由 chrmed_back_row 回写。 */
    dst = src;
    upl = (float *)malloc(np * sizeof(float));
    vpl = (float *)malloc(np * sizeof(float));
    if (src == NULL || upl == NULL || vpl == NULL) {
        free(src); free(upl); free(vpl); return X3FRENDER_OK;
    }

    c.src = src; c.dst = dst; c.upl = upl; c.vpl = vpl;
    c.w = w; c.h = h; c.r = r;
    c.src_out = buf; c.stride = stride; c.bits = bits;
#define CHRMED_TAP(tag) do { if (apply_t_on()) \
        fprintf(stderr, "[chrmed] %-8s footprint=%.0f/%.0fMB\n", tag, apply_t_footprint_mb(), apply_t_peak_mb()); } while (0)
    CHRMED_TAP("alloc");
    x3f_parallel_for((size_t)h, chrmed_load_row, &c);
    CHRMED_TAP("load");
    /* 色度平面：每像素一次亮度点积，表达式与旧版逐邻域重算完全一致
     * （double 域相减后再 cast float），保证输出逐位不变。 */
    x3f_parallel_for((size_t)h, chrmed_uv_row, &c);
    CHRMED_TAP("uv");
    int gdone = 0;   /* GPU 路径已直接写 buf 时跳过 dst 回写 */
#if defined(HAVE_OWN_GPU) && HAVE_OWN_GPU
    /* GPU 提速（2026-10-22）：GPU 只算 U/V 两个窗口中值平面。
     * 选中值 = 同一浮点多重集合的第 n/2 小值，与选择算法无关，
     * 所以与 CPU chrmed_select **值精确一致**；重建仍在 CPU 用
     * double 亮度（表达式同 chrmed_row），整条链输出逐位不变。
     * X3FDEC_GPU=0 关整层；**2026-10-31 起色度中值段默认回 CPU**
     * （X3FDEC_GPU_CHRMED=1 才走 GPU，Intel UHD 上 sel 二分 31 趟
     * kernel 反而慢）；GPU 失败静默回退 CPU 并行路径。
     * 注意：GPU 路径直接写 buf，不走下面的 dst 回写循环。 */
    /* GPU 段默认关（X3FDEC_GPU_CHRMED=1 才用）。omu/omv 以前无条件分配，
     * 100MP 就是 2×402MB 白占 —— 只在真要跑 GPU 时才申请。 */
    const int gpu_chrmed = x3f_render_gpu_chrmed_enabled() ? 1 : 0;
    float *omu = gpu_chrmed ? (float *)malloc(np * sizeof(float)) : NULL;
    float *omv = gpu_chrmed ? (float *)malloc(np * sizeof(float)) : NULL;
    int g1 = (omu && gpu_chrmed)
        ? own_gpu_chrmed_plane(upl, omu, w, h, r) : -1;
    int g2 = (g1 == 0 && omv) ? own_gpu_chrmed_plane(vpl, omv, w, h, r) : -1;
    if (getenv("X3FDEC_GPUDBG"))
        fprintf(stderr, "[chrmed] r=%d w=%d h=%d g1=%d g2=%d "
                "upl[0..1]=%.5f,%.5f omu[0..1]=%.5f,%.5f omv[0..1]=%.5f,%.5f\n",
                r, w, h, g1, g2, upl[0], upl[1], omu ? omu[0] : -1, omu ? omu[1] : -1,
                omv ? omv[0] : -1, omv ? omv[1] : -1);
    if (omu && omv && g1 == 0 && g2 == 0) {
        chrmed_rebuild_ctx_t rb;
        gdone = 1;
        rb.src = src; rb.omu = omu; rb.omv = omv;
        rb.buf = buf; rb.w = w; rb.stride = stride; rb.bits = bits;
        x3f_parallel_for((size_t)h, chrmed_rebuild_row, &rb);
    }
    free(omu); free(omv);
    if (!gdone) {
        /* GPU 没接上，整帧的 src / U/V 平面这时已经没有意义：先把它们放掉，
         * 再交给行带实现（数值逐像素相同，临时内存从 ~2GB 掉到几十 MB）。
         * 不这么做的话，full 版会继续跑整帧 CPU 路径，既慢又白占内存。 */
        CHRMED_TAP("gpu-miss");
        free(src); if (dst != src) free(dst); free(upl); free(vpl);
        if (getenv("X3F_NR_DEBUG"))
            fprintf(stderr, "[nr] 显示域色度中值 %d×%d（GPU 未接，行带兜底）\n",
                    2 * r + 1, 2 * r + 1);
        return apply_chroma_median_banded(r, buf, w, h, stride, bits);
    }
    CHRMED_TAP("select");
#else
    x3f_parallel_for((size_t)h, chrmed_row, &c);
    CHRMED_TAP("select");
#endif
    if (!gdone) {
        x3f_parallel_for((size_t)h, chrmed_back_row, &c);
    }
    CHRMED_TAP("back");
#undef CHRMED_TAP
    free(src); if (dst != src) free(dst); free(upl); free(vpl);
    if (getenv("X3F_NR_DEBUG"))
        fprintf(stderr, "[nr] 显示域色度中值 %d×%d（SPP Stage2 XDoYUVMedian 的等价物）\n",
                2 * r + 1, 2 * r + 1);
    return X3FRENDER_OK;
}

/* 分发。
 *
 * 默认走**行带版**：它内部把中值交给 GPU（own_gpu_chrmed_plane_rows，一带来回
 * 几十 MB），GPU 不可用时才退回 CPU 选择。这样 100MP 的临时内存从 3.2GB
 * 降到几十 MB，而中值仍是 GPU 速度。
 *
 * `X3F_CHRMED_IMPL=full` 强制旧的整帧实现（内存高，只作对拍）；
 * `X3F_CHRMED_IMPL=cpu` 强制行带里的纯 CPU 选择（不碰 GPU）。 */
static int apply_chroma_median(int r, void *buf, int w, int h, int stride, int bits)
{
    static int impl_full = -1;
    if (impl_full < 0) {
        const char *e = getenv("X3F_CHRMED_IMPL");
        impl_full = (e != NULL && (*e == 'f' || *e == 'F')) ? 1 : 0;
    }
#if defined(HAVE_OWN_GPU) && HAVE_OWN_GPU
    if (impl_full)
        return apply_chroma_median_full(r, buf, w, h, stride, bits);
#endif
    return apply_chroma_median_banded(r, buf, w, h, stride, bits);
}

/* ---- 显示域色度低通（2026-10-24）：暗部色噪的第二级 ----
 *
 * 实测（DP2Q6563 vs 内嵌 JPEG，逐像素分箱 + 自相关）：暗部残余色噪是
 * 3~6px 的高频斑点（自相关 ~5px），色度中值（上面那级，r=3）只能收一部分
 * —— X3F_CHRMED_R 扫到 12 也压不动（斑点成簇）。而内嵌 JPEG 的暗部色度
 * 被相机 NR + 4:2:0 子采样平滑到 ~48px 相关长度 —— 相机自己就把暗部
 * 色度抹得很狠，我们把暗部色度低通到 σ≈4 是向相机看齐，不是过度处理。
 *
 * 所以这一级做**亮度门控的色度高斯低通**（Y 不动、只低通 U/V）：
 *   · 门控 w = clamp((128 − Y)/80, 0, 1)：L<48 全量，L≥128 一丝不碰
 *     —— 中亮带的饱和度（真实颜色）零影响；
 *   · 高斯用 3 趟 box（半径 b）近似，σ_eff = √(b(b+1))；
 *   · 强度由色度降噪滑杆线性映射 σ = 8·nr_chroma（0.5 → b=2, σ≈4.2），
 *     滑杆为 0 时不调用 —— 零滑杆输出与接入前逐位一致（md5 铁律）。
 * 原型实证（6563，σ=4 + 门控，scipy 高斯）：0-16 带 satR 2.96→1.99，
 * 32-64 带 ~1.0，64-128 带 1.00→0.93，亮度梯度完全不动（Y 未触碰）。 */
typedef struct {
    void *buf; int stride, bits, w, h;
    float *y, *u, *v;          /* load 行写：亮度 + U=B−Y + V=R−Y */
    const float *ul, *vl;      /* 回写行读：低通后的 U/V */
} cshlp_io_ctx_t;

typedef struct {
    const float *in;
    float *out;
    int w, h, b, vertical;     /* vertical=0 水平 / 1 垂直 box */
} cbox_ctx_t;

static void cbox_row(void *v, size_t yi)
{
    const cbox_ctx_t *c = (const cbox_ctx_t *)v;
    const int y = (int)yi, w = c->w, b = c->b;
    int x, k;
    if (c->vertical) {
        const float *in = c->in;
        float *out = c->out + (size_t)y * w;
        for (x = 0; x < w; x++) {
            double s = 0.0;
            for (k = y - b; k <= y + b; k++) {
                int yy = k < 0 ? 0 : (k >= c->h ? c->h - 1 : k);
                s += in[(size_t)yy * w + x];
            }
            out[x] = (float)(s / (2 * b + 1));
        }
    } else {
        const float *in = c->in + (size_t)y * w;
        float *out = c->out + (size_t)y * w;
        for (x = 0; x < w; x++) {
            double s = 0.0;
            for (k = x - b; k <= x + b; k++) {
                int xx = k < 0 ? 0 : (k >= w ? w - 1 : k);
                s += in[xx];
            }
            out[x] = (float)(s / (2 * b + 1));
        }
    }
}

static void cshlp_load_row(void *v, size_t yi)
{
    cshlp_io_ctx_t *c = (cshlp_io_ctx_t *)v;
    const int y = (int)yi, w = c->w, bits = c->bits;
    const void *row = (const uint8_t *)c->buf + (size_t)y * c->stride;
    float *ypl = c->y, *upl = c->u, *vpl = c->v;
    int x;
    for (x = 0; x < w; x++) {
        size_t o = (size_t)x * 3, i = (size_t)y * w + x;
        double r = px_get(row, bits, o);
        double g = px_get(row, bits, o + 1);
        double b = px_get(row, bits, o + 2);
        double yq = 0.2126 * r + 0.7152 * g + 0.0722 * b;
        ypl[i] = (float)yq;
        upl[i] = (float)(b - yq);   /* U = B − Y */
        vpl[i] = (float)(r - yq);   /* V = R − Y */
    }
}

static void cshlp_back_row(void *v, size_t yi)
{
    cshlp_io_ctx_t *c = (cshlp_io_ctx_t *)v;
    const int y = (int)yi, w = c->w, bits = c->bits;
    const float *ypl = c->y, *upl = c->u, *vpl = c->v;
    const float *ul = c->ul, *vl = c->vl;
    void *row = (uint8_t *)c->buf + (size_t)y * c->stride;
    int x;
    for (x = 0; x < w; x++) {
        size_t o = (size_t)x * 3, i = (size_t)y * w + x;
        const double y0 = ypl[i];
        /* 亮度门控：L<48 全量低通，48..128 线性衰减，≥128 原样 */
        double w8 = (128.0 - y0) / 80.0;
        double cb, cr;
        if (w8 < 0.0) w8 = 0.0; else if (w8 > 1.0) w8 = 1.0;
        cb = (double)upl[i] + w8 * ((double)ul[i] - (double)upl[i]);
        cr = (double)vpl[i] + w8 * ((double)vl[i] - (double)vpl[i]);
        px_put(row, bits, o,     (float)(y0 + cr));
        px_put(row, bits, o + 2, (float)(y0 + cb));
        px_put(row, bits, o + 1,
               (float)(y0 - (0.2126 * cr + 0.0722 * cb) / 0.7152));
    }
}

/* 滑杆 → box 半径：σ = 8·nr_chroma。
 * 单趟 box（window 2b+1）σ²=b(b+1)/3，三趟合成 σ²=b(b+1) ⇒ b≈(−1+√(1+4σ²))/2 */
static int chroma_shlp_box(double nrchroma)
{
    const double sigma = 8.0 * nrchroma;
    int b = (int)(0.5 * (-1.0 + sqrt(1.0 + 4.0 * sigma * sigma)) + 0.5);
    if (b < 1) b = 1;
    if (b > 8) b = 8;
    return b;
}

static int apply_chroma_shlp(int b, void *buf, int w, int h,
                             int stride, int bits)
{
    cshlp_io_ctx_t io;
    cbox_ctx_t cb;
    size_t np = (size_t)w * h;
    float *y, *u, *v, *u2, *v2;
    int pass;

    if (b <= 0 || buf == NULL || w <= 0 || h <= 0) return X3FRENDER_OK;
    y  = (float *)malloc(np * sizeof(float));
    u  = (float *)malloc(np * sizeof(float));
    v  = (float *)malloc(np * sizeof(float));
    u2 = (float *)malloc(np * sizeof(float));
    v2 = (float *)malloc(np * sizeof(float));
    if (y == NULL || u == NULL || v == NULL || u2 == NULL || v2 == NULL) {
        free(y); free(u); free(v); free(u2); free(v2);
        return X3FRENDER_OK;
    }
    io.buf = buf; io.stride = stride; io.bits = bits; io.w = w; io.h = h;
    io.y = y; io.u = u; io.v = v; io.ul = NULL; io.vl = NULL;
    x3f_parallel_for((size_t)h, cshlp_load_row, &io);
    cb.in = NULL; cb.out = NULL; cb.w = w; cb.h = h; cb.b = b; cb.vertical = 0;
    for (pass = 0; pass < 3; pass++) {
        cb.in = u;  cb.out = u2; cb.vertical = 0;
        x3f_parallel_for((size_t)h, cbox_row, &cb);
        cb.in = u2; cb.out = u;  cb.vertical = 1;
        x3f_parallel_for((size_t)h, cbox_row, &cb);
        cb.in = v;  cb.out = v2; cb.vertical = 0;
        x3f_parallel_for((size_t)h, cbox_row, &cb);
        cb.in = v2; cb.out = v;  cb.vertical = 1;
        x3f_parallel_for((size_t)h, cbox_row, &cb);
    }
    io.ul = u; io.vl = v;
    x3f_parallel_for((size_t)h, cshlp_back_row, &io);
    free(y); free(u); free(v); free(u2); free(v2);
    if (getenv("X3F_NR_DEBUG"))
        fprintf(stderr, "[nr] 显示域暗部色度低通 b=%d（σ≈%.1f，亮度门控 128/80）\n",
                b, sqrt((double)b * (b + 1)));
    return X3FRENDER_OK;
}

/* 档位 → σ 缩放 / 窗半径。
 * 辉度用 DenoisingSlidersStep 的列 0（0.25…4，等比），照抄。
 *
 * **色度那一列不是照抄的**（2026-10-14 对 SPP 三张参考标定）：表给的是
 * 0.5/0.75/1/1.25/1.5，但拿它跑出来**远远不够** —— SPP 最低档就已经把色度收到
 * 相机直出的水平（DP0Q1000 @2712：SPP 0.282 / 相机 0.213），我们照原表跑是
 * 4.21，拉到满档才 2.29。原因是 色度降噪在它整条链里有别的东西在出力，
 * 单靠这一格 σ 对不上。
 *
 * （2026-10-14 补：那个"别的东西"找到了 —— 就是上面这条 Stage2 的色度中值。
 * 它在 SPP 里是**基线**、不吃滑块，所以我们之前无论怎么调 σ 都追不上。）
 *
 * 所以色度这套是**按视觉另标的**：以"档 4 用 σ=6、半径 10"为锚点（1:1 对比
 * more，暗部色斑消失、又没糊成一片），整条按同比例放大 4 倍。
 * 注意：这个"整幅 R−G 高频"指标**对不平**（SPP 0.19 vs 我们哪怕 91×91 的
 * 纯平均也停在 ~0.7），所以只按视觉定；换参考图觉得过头/不够就改这一行。
 *
 * ★ 2026-10-22 重标（ §20/§22）：
 * 色度滑杆 = 的 m_fBFNoiseScaling（逐层噪声 σ 缩放），5 档入口实测
 * less/mid/more = 0.06/0.5/1.5（层1/2；层0 = 0.02/0.1/1.0），导出遍再 ×
 * clamp(2−strength,1.1,2.5)=1.5。**档间曲线强非线性**（less→mid ×8.3、
 * mid→more ×3），旧线性表 {2,3,4,5,6} 的低半段比 SPP 狠得多 —— 这就是
 * "往右一点就涂抹"在滑杆侧的另一半根源（另一半是缺 常开 UV 预滤波
 * 基线，我们靠解码步 NLM 顶着）。
 * 新表：σ_detent = 6 × BFS(detent)/BFS(more)（锚定档 4 = σ6 不变），
 * 档 1/3 按几何插值（实测只有 3 档）。要回旧表：X3F_NR_CHROMA_LEGACY=1。 */
static const double nr_luma_sig[5]   = { 0.25, 0.5, 1.0, 2.0, 4.0 };
static const double nr_chroma_sig[5] = { 0.24, 0.69, 2.0, 3.46, 6.0 };
static const double nr_chroma_sig_legacy[5] = { 2.0, 3.0, 4.0, 5.0, 6.0 };
static const int    nr_luma_rad[5]   = { 2, 3, 4, 5, 6 };
static const int    nr_chroma_rad[5] = { 3, 4, 6, 8, 10 };

/* ---- SPP Stage1 UV 预滤波（分类器版，§19；X3F_NR_CHROMA_SPP=1，默认关）----
 *
 * §19/§20/§22：SPP 色度基线的"暗部干净、彩色无损"
 * 不是双边调出来的，是 Stage1 的一个**分类器**——只动深阴影里的近灰像素：
 *   1. 暗部门：box 均值 R ≤ 6·√(读出噪声²) → 不动（黑电平附近信号与噪声不可分）
 *   2. 中性门：box 均值比 mean_R/mean_B、mean_G/mean_B ∈ [0.95,1.1] 才处理
 *      —— 彩色结构天然豁免（§19.4：这就是"平坦区色噪磨 3×、彩色细节不受损"
 *      能同时成立的根源）
 *   3. 邻居门：3×3 内 |R_j−boxR|² > bfLUT[14444−clamp(R_c,1,14384)]·σ²_R(R_c)
 *      的邻居数 > 6 → 旗标。bfLUT 反向索引 = 越暗阈值越宽（32×→1×）；
 *      bfLUT[i] = 31·(i/14407)⁴+1（14385 项，双边那张同款）。
 *   4. 替换：旗标像素按亮度斜坡混入 3×3 box 均值 —— mR ≤ 200·max3 全量，
 *      [200,225]·max3 线性过渡，≥225·max3 不动（max3 = 逐层参数
 *      params+0x240/0x248/0x250 的最大值，实测 d240=[1.2205,1,1.0844] 恒定
 *      → 阈 ≈ 244..275 DN ≈ 1.7% 灰度 —— SPP 这级只洗最深的影子）。
 * 滑杆 → BFS（§20 实测 3 档 less/mid/more = 0.02·0.06·1.0（层0），几何插值补
 * 2/4 档）× 入口调制 k=clamp(2−strength,1.1,2.5)（导出遍 strength=0.5 → 1.5）。
 * σ² 有效值 = 模型 σ²/(BFS·k)²（§22.1 的 s12 = s8/BFS）→ 滑杆越大阈值越低、
 * 越多邻居超阈 —— 动态范围 ~50×，与 SPP"色度滑杆曲线陡"一致。
 *
 * 【与 两处已知偏差】①旗标 1 斜坡的方向：§19.2 里 w 的 A/B 语义
 * 存在两读，这里取"越暗替换越重"（与暗部门/旗标 2 的亮度序一致）；
 * ②G/B 以补数计的技巧按文档结论直接实现为 R 路单通道判据。
 * 本级跑在色度双边**之前**；中调行为仍由双边负责（对应 NR3 多尺度）。*/
typedef struct {
    const float *src; float *dst;
    int w, h;
    double fs, max3, a_r, c1, c2;      /* DN 域换算 + R 通道噪声模型 (c0=a_r,c1,c2) */
    double bfs_r;                      /* BFS[0]·k（有效值） */
    double t200, t2125, t225;          /* 亮度斜坡三阈值（DN） */
} uvp_ctx_t;

static void uvp_row(void *v, size_t yi)
{
    const uvp_ctx_t *c = (const uvp_ctx_t *)v;
    const int y = (int)yi, w = c->w, h = c->h;
    const double fs = c->fs;
    int x, dx, dy;
    for (x = 0; x < w; x++) {
        double sR = 0, sG = 0, sB = 0;
        double Rc = 0;
        size_t o = ((size_t)y * w + x) * 3;
        /* 3×3 box 均值（边缘 clamp）+ 中心 R（DN） */
        for (dy = -1; dy <= 1; dy++) {
            int yy = y + dy; if (yy < 0) yy = 0; if (yy >= h) yy = h - 1;
            for (dx = -1; dx <= 1; dx++) {
                int xx = x + dx; if (xx < 0) xx = 0; if (xx >= w) xx = w - 1;
                size_t p = ((size_t)yy * w + xx) * 3;
                double r = (double)c->src[p] * fs;
                sR += r; sG += (double)c->src[p + 1] * fs; sB += (double)c->src[p + 2] * fs;
            }
        }
        sR /= 9.0; sG /= 9.0; sB /= 9.0;
        Rc = (double)c->src[o] * fs;
        /* 门 1：暗部门（读出噪声 6σ 以内不分信号噪声） */
        if (sR <= 6.0 * sqrt(c->a_r)) goto keep;
        /* 门 2：中性门（只动近灰区） */
        if (sB <= 0.0) goto keep;
        { double rB = sR / sB, gB = sG / sB;
          if (rB < 0.95 || rB > 1.1 || gB < 0.95 || gB > 1.1) goto keep; }
        /* 门 3：邻居门 —— R 路超阈计数 */
        {
            int idx = 14444 - (int)(Rc + 0.5);
            double bfl, var, T2, cnt = 0;
            if (idx < 1) idx = 1; if (idx > 14384) idx = 14384;
            { double q = (double)idx / 14407.0; bfl = 31.0 * q * q * q * q + 1.0; }
            var = (c->c2 * Rc + c->c1) * Rc + c->a_r;
            T2 = bfl * var / (c->bfs_r * c->bfs_r);
            for (dy = -1; dy <= 1; dy++) {
                int yy = y + dy; if (yy < 0) yy = 0; if (yy >= h) yy = h - 1;
                for (dx = -1; dx <= 1; dx++) {
                    int xx = x + dx; if (xx < 0) xx = 0; if (xx >= w) xx = w - 1;
                    double d = (double)c->src[((size_t)yy * w + xx) * 3] * fs - sR;
                    if (d * d > T2) cnt += 1.0;
                }
            }
            if (cnt <= 6.0) goto keep;
        }
        /* 旗标落像素：亮度斜坡（越暗替换越重） */
        {
            double wr;
            if (sR <= c->t200) wr = 1.0;
            else if (sR >= c->t225) goto keep;
            else wr = (c->t225 - sR) / (c->t225 - c->t200);
            c->dst[o]     = (float)((double)c->src[o]     * wr + (sR / fs) * (1.0 - wr));
            c->dst[o + 1] = (float)((double)c->src[o + 1] * wr + (sG / fs) * (1.0 - wr));
            c->dst[o + 2] = (float)((double)c->src[o + 2] * wr + (sB / fs) * (1.0 - wr));
            continue;
        }
keep:
        c->dst[o] = c->src[o]; c->dst[o + 1] = c->src[o + 1]; c->dst[o + 2] = c->src[o + 2];
    }
}

static void nr_uv_prefilter(float *dst, const float *src, int w, int h,
                            double fs, int cv)
{
    uvp_ctx_t c;
    /* 滑杆 5 档 → BFS（§20 实测 1/3/5 档，2/4 几何插值）× 入口调制 k=1.5 */
    static const double bfs0[5]  = { 0.02, 0.045, 0.1, 0.32, 1.0 };
    double km = nr_env("X3F_NR_UV_K", 1.5);          /* clamp(2−strength,1.1,2.5)，导出遍 */
    double max3 = nr_env("X3F_NR_UV_MAX3", 1.2205);  /* d240 三层最大值（实测恒定） */
    c.src = src; c.dst = dst; c.w = w; c.h = h; c.fs = fs;
    c.a_r = g_nr.cn_iso_c[0];
    c.c1 = g_nr.cn_iso_c[3]; c.c2 = g_nr.cn_iso_c[6];
    c.bfs_r = bfs0[cv] * km;
    c.t200 = 200.0 * max3; c.t2125 = 212.5 * max3; c.t225 = 225.0 * max3;
    (void)c.t2125;
    x3f_parallel_for((size_t)h, uvp_row, &c);
    if (getenv("X3F_NR_DEBUG"))
        fprintf(stderr, "[nr] UV预滤波(SPP分类器) 档=%d BFS_r=%.3f(k=%.2f) "
                        "暗部门=%.0fDN 斜坡=%.0f..%.0fDN max3=%.3f\n",
                cv + 1, c.bfs_r, km, 6.0 * sqrt(c.a_r), c.t200, c.t225, max3);
}

/* ---- SPP NR3 层内 UV 执行器（XDo_UV_ImageFilter 复刻，§25；X3F_NR_CHROMA_UVF=1，默认关）----
 *
 * §25.2：SPP 把 RGB→YUV 后做 3×3 盒滤波，再按亮度门把色度整块替换回 RGB：
 *   Y<200·max3 或 Y≥225·max3 → 无条件用滤波色度（整块替换）；
 *   [200,225]·max3 → (225−Y)/25 斜坡。六σ超阈计数与中性比值 [0.955,1.05]
 *   只决定 mask 分类（2/1 低段行为相同、0 与 2 相同），原型先省略。
 *
 * 【RGB 域等价】线性可交换性免掉显式 YUV 往返：目标 out_YUV=(Y_orig, U_box, V_box)
 * ⟺ ΔRGB = d − dY·(1,1,1)，其中 d = box−orig（RGB 逐通道）、dY = 709 亮度差。
 * Y(out)=Y(orig) 逐位精确（零 601/709 串扰，§24.2 坑 2 的正解同族），
 * U/V(out)=U/V(box)（U=R−Y 型线性基下精确）。
 *
 * 门语义（§25.2 有方向二义，X3F_NR_UVF_DIR 裁决）：
 *   dir=0（单调连续读法）：Yc≤LO → g=1 全替换；Yc≥HI → g=0 豁免；
 *         带内 g=(Yc−LO)/(HI−LO)。
 *   dir=1（字面读法）：两端都 g=1（高光也替换）、带内 g=1−(Yc−LO)/(HI−LO)。
 * X3F_NR_UVF_S21X：s21 缩放疑点（s21=中心+Σk·邻≈2Y → 阈值实为 Y∈[100,112.5]·max3），
 *   门比较用 Yc·S21X；对质后钉死。
 */
typedef struct {
    const float *src; float *dst;
    int w, h;
    double fs, lo, hi;
    int dir;
} uvf_ctx_t;

static void uvf_row(void *v, size_t yi)
{
    const uvf_ctx_t *c = (const uvf_ctx_t *)v;
    const int y = (int)yi, w = c->w, h = c->h;
    int x, dx, dy;
    for (x = 0; x < w; x++) {
        size_t o = ((size_t)y * w + x) * 3;
        double r0 = (double)c->src[o] * c->fs;
        double g0 = (double)c->src[o + 1] * c->fs;
        double b0 = (double)c->src[o + 2] * c->fs;
        double sR = 0, sG = 0, sB = 0;
        double g, yc, dY, dr, dg, db;
        for (dy = -1; dy <= 1; dy++) {
            int yy = y + dy; if (yy < 0) yy = 0; if (yy >= h) yy = h - 1;
            for (dx = -1; dx <= 1; dx++) {
                int xx = x + dx; if (xx < 0) xx = 0; if (xx >= w) xx = w - 1;
                size_t p = ((size_t)yy * w + xx) * 3;
                sR += (double)c->src[p]     * c->fs;
                sG += (double)c->src[p + 1] * c->fs;
                sB += (double)c->src[p + 2] * c->fs;
            }
        }
        sR /= 9.0; sG /= 9.0; sB /= 9.0;
        /* 亮度门（709，DN 域；SPP 用自家矩阵，域一致即可比较） */
        yc = 0.2126 * r0 + 0.7152 * g0 + 0.0722 * b0;
        if (c->dir == 0) {
            if (yc <= c->lo) g = 1.0;
            else if (yc >= c->hi) g = 0.0;
            else g = (yc - c->lo) / (c->hi - c->lo);
        } else {
            if (yc <= c->lo) g = 1.0;
            else if (yc >= c->hi) g = 1.0;
            else g = 1.0 - (yc - c->lo) / (c->hi - c->lo);
        }
        if (g <= 0.0) {
            c->dst[o] = c->src[o]; c->dst[o + 1] = c->src[o + 1];
            c->dst[o + 2] = c->src[o + 2];
            continue;
        }
        dr = sR - r0; dg = sG - g0; db = sB - b0;
        dY = 0.2126 * dr + 0.7152 * dg + 0.0722 * db;
        /* out = orig + g·(Δ − dY)：Y 精确保持、色度=盒滤波值 */
        c->dst[o]     = (float)((r0 + g * (dr - dY)) / c->fs);
        c->dst[o + 1] = (float)((g0 + g * (dg - dY)) / c->fs);
        c->dst[o + 2] = (float)((b0 + g * (db - dY)) / c->fs);
    }
}

static void nr_chroma_uvf(float *dst, const float *src, int w, int h,
                          double fs, int cv)
{
    uvf_ctx_t c;
    double max3 = nr_env("X3F_NR_UV_MAX3", 1.2205);   /* 与 §19 分类器同源 */
    double s21x = nr_env("X3F_NR_UVF_S21X", 1.0);     /* s21≈2Y 疑点：2 时阈值折半 */
    c.src = src; c.dst = dst; c.w = w; c.h = h; c.fs = fs;
    c.lo = 200.0 * max3 / s21x;
    c.hi = 225.0 * max3 / s21x;
    c.dir = (int)nr_env("X3F_NR_UVF_DIR", 0);
    x3f_parallel_for((size_t)h, uvf_row, &c);
    if (getenv("X3F_NR_DEBUG"))
        fprintf(stderr, "[nr] UVF执行器(NR3层0) 档=%d 门=%.0f..%.0fDN(max3=%.3f ×%.2f) dir=%d\n",
                cv + 1, c.lo, c.hi, max3, s21x, c.dir);
}

/* 解码那一步的 NLM 已经带亮度时（高 ISO 的默认），**辉度滑块封顶在档 2**。
 *
 * 起因（2026-10-15 实测，DP0Q2315 ISO 1600，口径 = Y 五点高通 RMS 0..255）：
 * NLM 亮度先跑，滑块再叠上去，两段各测三块区域
 *
 *   滑块档   平坦暗部   电扇（高对比）  布料（低对比）
 *   关        3.25       5.90          6.52
 *   2 (0.5)   1.80       3.94          4.51
 *   4 (1.0)   0.57       1.86          1.83
 *
 * **0 → 2 是有选择性的**（噪声掉 45%、结构只掉 31~33%，暗部那几点残渣正好收掉）；
 * **2 → 4 基本是 1:1 地一起抹**（噪声 −68%、结构 −53~59%），1:1 看图也能看出糊。
 * 也就是说 NLM 上台之后，这根滑块的有效区间缩到了下半段，上面两档是纯亏。
 *
 * 所以：解码步降了亮度（`g_nlm_luma_on`）时把档位夹到 2，其余情况（低 ISO、
 * 或者用户自己把 NLM 关掉）**一个字节都不动** —— 那两种场景下滑块就是唯一的
 * 亮度降噪，档位曲线是按 SPP 标定过的（见 §13 / §16），不能碰。
 * `X3F_NR_STACK=0` 关掉这个夹取（lab A/B 用）。 */
static int nr_luma_stack_cap(int lv)
{
    if (!g_nlm_luma_on) return lv;
    if (!nr_flag("X3F_NR_STACK", 1)) return lv;
    return (lv > 2) ? 2 : lv;
}

static double nr_env(const char *name, double dflt)
{
    const char *e = getenv(name);
    if (e != NULL) {
        double v = atof(e);
        if (v > 0.0) return v;
    }
    return dflt;
}

/* 布尔开关要单独一个：`nr_env` 把 0 当成"没设"（那是给倍率用的），
 * 所以 X3F_NR_ADAPT=0 会静默失效 —— 踩过一次。 */
static int nr_flag(const char *name, int dflt)
{
    const char *e = getenv(name);
    if (e == NULL || e[0] == '\0') return dflt;
    return atoi(e) != 0;
}

/* Quattro 基线亮度降噪（2026-11-01，用户报"楼顶上方细竖线"）。
 *
 * 背景：Quattro 的 CAMF 里**没有**噪声模型（见 §10.6），g_nr 走的是烘进去的
 * nr_quattro_model；而 `nr_luma = 0`（UI 语义"不额外降噪"）时，整条渲染链
 * **一级亮度降噪都不跑**，解码链那边 own 也只跑 NLM。实测 DP3Q 09
 * （伦敦金融城，强逆光蓝天 + 深色玻璃尖顶楼）：楼顶每根亮竖框正上方都留一条
 * 1~2 级、5~10px 高的淡竖纹，越往上越淡；SIGMA 自家内嵌 JPEG 和 SPP 都没有
 * —— 因为 SPP 的降噪链默认就在跑，我们的默认是"什么都不做"。
 *
 * 这里给 Quattro 一个基线档（0.8 = 第 3 档）。同一区域实测：
 * 竖纹列剖面 p95 2.45 → 1.32（原厂 JPEG 1.29），而窗格立面细节只掉 1.0%、
 * 小黄瓜格栅掉 1.2% —— 属于"抹掉低于感知阈值的结构"，不是"糊细节"。
 *
 * ⚠️ 2026-11-04 回退：**默认关**。后来查明那条细竖纹的真正根因是显示域
 * 解码侧 unsharp（apply_decode_unsharp 跨强边的贴边暗下冲），不是噪声；
 * 基线降噪只压 1~2px 随机列纹，压不动 4~12px 相干结构，加大还会削掉
 * 真实细节与掩蔽噪声 —— 用户实测「只能轻微降低、加大损失画质」。
 * 真正治本已改为把 X3FDEC_SHARP 默认关掉，这条弯路不再默认生效。
 * 能力与 UI 开关都保留，需要时手动勾或 X3F_NR_Q_BASELINE=1 开启。
 *
 * 用户把滑块拉到 >0 时按用户的档走（不叠加）；改档：X3F_NR_Q_BASE。
 * Merrill 不适用：它有自己的 CAMF 噪声模型，默认值一个字都不许动。 */
static double nr_luma_baseline(double v, const x3f_edit_params_t *p, int *from_base)
{
    const char *e;
    double base;
    int on;
    if (from_base != NULL) *from_base = 0;
    if (!g_nr.quattro_model) return v;
    /* 口径：App 面板传下来的参数是主源；环境变量只做 CLI / 标定覆盖。
     * （X3F_NR_Q_BASELINE 用 atoi 解释，所以 =0 是关 —— A/B 时也可以直接 unset。） */
    on = (p != NULL) ? p->nr_q_baseline : 0;
    base = (p != NULL && p->nr_q_base > 0.0) ? p->nr_q_base : 0.8;
    e = getenv("X3F_NR_Q_BASELINE");
    if (e != NULL && e[0] != '\0') on = atoi(e) != 0;
    e = getenv("X3F_NR_Q_BASE");
    if (e != NULL && e[0] != '\0') base = atof(e);
    if (!on || base <= 0.0) return v;
    /* 取 max：滑块在基线以下等于不动（单调，不会出现"往上拉反而更脏"），
     * 基线以上按用户档走。 */
    if (v >= base) return v;
    if (from_base != NULL) *from_base = 1;
    if (getenv("X3F_NR_DEBUG") && v <= 0.0)
        fprintf(stderr, "[nr] Quattro 基线亮度降噪：滑块 %.2f → %.2f（第 %d 档）\n",
                v, base, nr_level(base));
    return base;
}

/* 结构门控的三个旋钮：参数是主源，环境变量只做 CLI / 标定覆盖。
 * 返回 1 = 开。keep_max = 纹理区最多还回多少比例的残差。 */
static int nr_gate_cfg(const x3f_edit_params_t *p,
                       double *keep, double *lo, double *hi)
{
    const char *e;
    int on = (p != NULL) ? (p->nr_q_gate != 0) : 1;
    *keep = (p != NULL && p->nr_q_gate_k > 0.0) ? p->nr_q_gate_k : 0.75;
    /* 阈值按 DP3Q 09 的实测分布定的（见 x.raw 标定）：
     *   纯天空 x 中位 0.82 / p75 1.07 → lo=1.1 时天空基本不进门
     *   树冠   x 中位 1.28 / p75 2.30 → hi=2.6 时 p75 以上全保
     * 实测 h1（1~2px）相对"门控关"：纯天空 +1%、屋顶条带 +2%、
     * 树冠 +12%、窗格 +4%、小黄瓜 +5%、玻璃塔 +2%。 */
    *lo = 1.1;
    *hi = 2.6;
    e = getenv("X3F_NR_Q_GATE");
    if (e != NULL && e[0] != '\0') on = atoi(e) != 0;
    e = getenv("X3F_NR_Q_GATE_K");
    if (e != NULL && e[0] != '\0') *keep = atof(e);
    e = getenv("X3F_NR_Q_GATE_LO");
    if (e != NULL && e[0] != '\0') *lo = atof(e);
    e = getenv("X3F_NR_Q_GATE_HI");
    if (e != NULL && e[0] != '\0') *hi = atof(e);
    if (*keep > 1.0) *keep = 1.0;
    if (*keep < 0.0) *keep = 0.0;
    return on && (*keep > 0.0) && (*hi > *lo);
}

/* ---- 基线降噪的**结构门控**（2026-11-01，见 x3f_render.h 的字段注释）----
 *
 * 记法：io = 双边输出（原地改），orig = 双边输入。
 *   1) r = orig − io；e2 = mean_c(r_c²)（逐像素残差能量，线性域）
 *   2) E = 3×3 盒均值(e2)                    ← 邻域判决（单像素分不开纹理与噪声）
 *   3) σ² = mean_c(cn_iso 在 v=orig_c·fs 处的值) ← 相机自己的噪声模型（DN 域→线性域）
 *   4) x = E/σ²，s = clamp((x−lo)/(hi−lo),0,1)，keep = keep_max·s
 *   5) io += keep·r
 *
 * 为什么用**残差**而不是梯度：噪声本身就带梯度，按梯度门控会把"噪声多的
 * 地方"当成结构、把 sky 的竖纹（1~2 级、正好在噪声量级）当成纹理留下来 ——
 * 两个方向上都是错的。残差能量除以噪声模型才是"这块到底是噪声还是东西"。
 * 平坦区 E≈σ² ⇒ s≈0 ⇒ 一个像素都不还。纹理区 E 是 σ² 的几倍 ⇒ 大部分还回去。
 *
 * 代价：一次 w·h 的 float 临时数组（20MP ≈ 78MB），失败就整段跳过（= 退回
 * 纯基线，宁可不门控也不出半张图）。 */
static void nr_gate_mix(float *io, const float *orig, int w, int h,
                        double fs, double keep_max, double lo, double hi)
{
    const size_t nn = (size_t)w * h;
    float *e2;
    size_t i;
    int x, y, c;
    double fs2 = fs * fs;
    /* 诊断（X3F_NR_DEBUG）：x = E/σ² 的分档计数 + 平均 keep */
    static long hist[5];
    static double kpsum;
    static long kpn;
    /* 标定用（X3F_NR_Q_GATE_DUMP=<前缀>）：落盘 x（uint16，×512）与 keep（uint8，×255） */
    const char *dumpn = getenv("X3F_NR_Q_GATE_DUMP");
    const int dbg = (getenv("X3F_NR_DEBUG") != NULL);   /* 循环里 getenv 太贵（逐个 environ 扫） */
    uint16_t *xmap = NULL;
    unsigned char *kmap = NULL;
    if (keep_max <= 0.0 || hi <= lo) return;
    e2 = (float *)malloc(nn * sizeof(float));
    if (e2 == NULL) return;
    if (dumpn != NULL && dumpn[0] != '\0') {
        xmap = (uint16_t *)calloc(nn, sizeof(uint16_t));
        kmap = (unsigned char *)calloc(nn, 1);
    }

    /* pass 1：逐像素残差能量（三通道平均） */
    for (i = 0; i < nn; i++) {
        const float *po = orig + i * 3;
        const float *pi = io + i * 3;
        double e = 0.0;
        for (c = 0; c < 3; c++) {
            const double r = (double)po[c] - (double)pi[c];
            e += r * r;
        }
        e2[i] = (float)(e / 3.0);
    }

    /* pass 2：3×3 盒均值 → 判决 → 逐通道回注 */
    for (y = 0; y < h; y++) {
        const int y0 = (y > 0) ? y - 1 : 0;
        const int y1 = (y < h - 1) ? y + 1 : h - 1;
        for (x = 0; x < w; x++) {
            const int x0 = (x > 0) ? x - 1 : 0;
            const int x1 = (x < w - 1) ? x + 1 : w - 1;
            const size_t idx = (size_t)y * w + x;
            const float *po = orig + idx * 3;
            float *pi = io + idx * 3;
            double e = 0.0, s2 = 0.0, sv, t, kp;
            int yy, xx;
            for (yy = y0; yy <= y1; yy++)
                for (xx = x0; xx <= x1; xx++)
                    e += e2[(size_t)yy * w + xx];
            e /= (double)((y1 - y0 + 1) * (x1 - x0 + 1));
            for (c = 0; c < 3; c++) {
                const double v0 = (double)po[c] * fs;
                s2 += g_nr.cn_iso[c] + g_nr.cn_iso[3 + c] * v0
                    + g_nr.cn_iso[6 + c] * v0 * v0;
            }
            s2 /= 3.0;
            /* σ² 折回线性域（cn_iso 是 DN 域，见 nr_build_iso_coef） */
            if (fs2 > 1e-12) s2 /= fs2;
            if (s2 < 1e-12) continue;          /* 噪声模型退化：不动这一块 */
            sv = e / s2;
            if (xmap != NULL) {
                double xv = sv * 512.0;
                xmap[idx] = (uint16_t)(xv < 0.0 ? 0 : (xv > 65535.0 ? 65535.0 : xv));
            }
            if (dbg) {
                int b = (sv < 0.5) ? 0 : (sv < 1.0) ? 1 : (sv < 2.0) ? 2 : (sv < 4.0) ? 3 : 4;
                hist[b]++;
            }
            t = (sv - lo) / (hi - lo);
            if (t <= 0.0) continue;            /* 平坦区：一个像素都不还 */
            if (t > 1.0) t = 1.0;
            kp = keep_max * t;
            if (kmap != NULL) kmap[idx] = (unsigned char)(kp * 255.0 + 0.5);
            if (dbg) { kpsum += kp; kpn++; }
            for (c = 0; c < 3; c++)
                pi[c] = (float)((double)pi[c]
                                + kp * ((double)po[c] - (double)pi[c]));
        }
    }
    if (dbg) {
        const double n = (double)nn;
        fprintf(stderr, "[nr]   门控 x=E/σ² 分布 <0.5:%.1f%% 0.5-1:%.1f%% 1-2:%.1f%% "
                        "2-4:%.1f%% >4:%.1f%% | 回注像素 %.1f%% 平均 keep %.3f\n",
                100.0 * hist[0] / n, 100.0 * hist[1] / n, 100.0 * hist[2] / n,
                100.0 * hist[3] / n, 100.0 * hist[4] / n,
                (kpn > 0 ? 100.0 * (double)kpn / n : 0.0),
                (kpn > 0 ? kpsum / (double)kpn : 0.0));
        memset(hist, 0, sizeof hist); kpsum = 0.0; kpn = 0;
    }
    if (xmap != NULL) {
        char path[1024];
        FILE *f;
        snprintf(path, sizeof path, "%s.x.raw", dumpn);
        f = fopen(path, "wb");
        if (f != NULL) { fwrite(xmap, sizeof(uint16_t), nn, f); fclose(f); }
        snprintf(path, sizeof path, "%s.k.raw", dumpn);
        f = fopen(path, "wb");
        if (f != NULL) { fwrite(kmap, 1, nn, f); fclose(f); }
        fprintf(stderr, "[nr]   门控落盘 %s.{x,k}.raw (%dx%d)\n", dumpn, w, h);
    }
    free(xmap);
    free(kmap);
    free(e2);
}

/* 返回 1 = 真的动过图（调用方据此决定是否用副本）。
 * 顺序按 ProcessTiles：双边（辉度/色度）在前，条纹在后。*/
static int nr_apply(double luma, double chroma, double banding,
                    const float *lin, float **work_io, int w, int h,
                    int gate, double gate_keep, double gate_lo, double gate_hi)
{
    static const int    half_tab[5] = { 15, 21, 27, 33, 39 };
    static const double k_tab[5]    = { 0.625, 0.5, 0.375, 0.25, 0.125 };
    const char *dir_env;
    double fs, nmfs, nmfs_lc, nmfs_c, iso_scale;
    float *a, *b;
    int did = 0;
    double t0 = 0.0;

    if (getenv("X3F_NR_TIME")) t0 = nr_now();

    /* 标定表没读进来就整段跳过。
     *
     * 【实证 2026-10-13】**Quattro（DP0Q/DPQ…）的文件里没有噪声模型**：
     * 把 CAMF 全导出来对比（`x3f_extract -meta`），Merrill 有
     * ColorNoiseModel / ISONoiseSettings(Fast) / AdditionalISONoiseSettings /
     * Min·MaxNoiseScalingFactor，而 Quattro 一份都没有 —— 整个 .meta 里连
     * "noise" 这个词都不出现，只有 DenoisingSlidersStep(5×13) + SensorISO
     * + EstimatedISO{reference,AWBGains,NumOutliers}。
     * 于是走这条路的三个滑块在 Quattro 上**静默无效**（不是没生效，是没跑）。
     * Quattro 的 σ 模型应该在 （F23）里，
     * 还没挖 —— 详见 §10.6。*/
    if (!g_nr.valid || lin == NULL || w <= 0 || h <= 0) return 0;
    if (luma <= 0.0 && chroma <= 0.0 && banding <= 0.0) return 0;

    fs = nr_fullscale();
    nmfs = nr_nmfs();          /* 条纹那条的噪声模型域（见 X3F_NR_NMFS） */
    nmfs_lc = nr_nmfs_lc();    /* 辉度（见 X3F_NR_NMFS_LC，2026-10-14 拆分标定；2026-10-15 再按系数来源分域） */
    nmfs_c = nr_nmfs_c();      /* 色度 —— 比辉度更大，见 X3F_NR_NMFS_C_Q 的注释 */
    iso_scale = nr_iso_scale(&g_nr, g_nr.sensor_iso);

    a = (float *)malloc((size_t)w * h * 3 * sizeof(float));
    b = (float *)malloc((size_t)w * h * 3 * sizeof(float));
    if (a == NULL || b == NULL) { free(a); free(b); return 0; }
    memcpy(a, lin, (size_t)w * h * 3 * sizeof(float));

    /* 整链 NR 输出槽（2026-10-23，**只服务大帧**）。
     *
     * 【病根实测】辉度/色度两个分级槽都在 ≤8M px 才工作（APPLY_CACHE_MAX_PIX），
     * 而 1:1 显示缓存那一路是原生 Quattro（≈19.6M px）——永远超帽。于是
     * 拖一个跟 NR 无关的滑杆（曝光/色调/细节…），原生 apply 就把整条双边
     * 滤波从头重跑（6 线程好几分钟）：sample 空闲进程抓到 6 个 utility 线程
     * 全泡在 nr_bilateral2d_row，「什么也没做 CPU 也在烧」+ 拖杆不跟手都是它。
     *
     * 这里给大帧加一个**整链**槽：键 = (fp(lin), 三滑杆)。非 NR 滑杆 →
     * 键不变 → 命中 → NR 整段跳过；拖 NR 滑杆 → 键变 → 重算（应该的）。
     * 小帧（≤8M）维持原有分级槽不动——预览拖 NR 滑杆时按级复用更省。
     * 帽子 X3F_NR_CACHE_MAX_PIX（默认 24M px），置 0 回到原行为。 */
    const size_t n3f = (size_t)w * h * 3;
    /* 大帧（>8M）走单槽（导出/整幅，一块就 235MB，池化会炸内存）；
     * 块级（2M..8M）走小池 —— 1:1 的行带 2.78M 与视口瓦片 6.23M 都在这一档，
     * 这正是「放大后调参 CPU 打满」的现场，见 g_nrf_pool 处的注释。 */
    const int nr_big_frame = w * h > APPLY_CACHE_MAX_PIX;
    int comb_ok = apply_cache_on() && w * h <= nr_cache_max_pix()
        && (nr_big_frame || w * h >= NR_F_MIN_PIX);
    uint64_t cfk1 = 0, cfk2 = 0;
    if (comb_ok) {
        cfk1 = apply_fp(lin, n3f);
        comb_ok = cfk1 != 0;
        if (comb_ok)
            cfk2 = apply_fp_d(luma)    * 0x9E3779B97F4A7C15ULL
                 ^ apply_fp_d(chroma)  * 0xC2B2AE3D27D4EB4FULL
                 ^ apply_fp_d(banding) * 0x165667B19E3779F9ULL;
    }
    if (comb_ok && (nr_big_frame
            ? apply_slot_get(&g_nrf_slot, &g_nrf_lock, cfk1, cfk2, 0,
                             w, h, n3f, a, NULL, NULL, NULL)
            : apply_pool_get(g_nrf_pool, NR_F_POOL, &g_nrf_lock, cfk1, cfk2, 0,
                             w, h, n3f, a))) {
        if (getenv("X3F_NR_TIME"))
            fprintf(stderr, "[nr] 整链缓存命中（%dx%d）%.2fs\n", w, h, nr_now() - t0);
        free(b);
        if (*work_io != NULL) free(*work_io);
        *work_io = a;
        return 1;
    }

    /* 两级缓存的键基座：内容指纹（防地址复用假命中）。
     * 辉度段键 = (fp(lin), luma)；色度段键 = (fp(lin), luma, chroma)
     * —— 色度的输入是辉度的输出，而那是 (lin, luma) 的确定函数，
     * 用键组合等价且省一次全图指纹。 */
    {
        const size_t n3 = (size_t)w * h * 3;
        const int cache_ok = apply_cache_on() && w * h <= APPLY_CACHE_MAX_PIX;
        const uint64_t lfp = cache_ok ? apply_fp(lin, n3) : 0;
        const int can_cache = cache_ok && lfp != 0;

    if (luma > 0.0) {
        const uint64_t nlk = apply_fp_d(luma);
        int nr_hit = 0;
        if (can_cache)
            nr_hit = apply_slot_get(&g_nr1_slot, &g_nr1_lock,
                                    lfp, nlk, 0, w, h, n3, a, NULL, NULL, NULL);
        if (!nr_hit) {
        int lv_raw = nr_level(luma);
        int lv = nr_luma_stack_cap(lv_raw);
        if (lv != lv_raw && getenv("X3F_NR_DEBUG"))
            fprintf(stderr, "[nr] 辉度滑块 档 %d → %d（解码那步已降亮度，叠着跑的上半段纯亏）\n",
                    lv_raw, lv);
        /* ★ EC 模式（X3F_NR_LUMA_EC=1，2026-10-27）：辉度滑杆改挂 SPP Stage2B
         * §23，本轮静态+抓包）。
         *
         * 实证：SPP 辉度滑杆**不进 Stage1**、也不调 Stage2B 双边的 σ —— 双边
         * （XDoBilateralFilter2B，LUT3）和 BlendNR（LUT1）都由逐遍强度 cc /
         * 噪声模型自动驱动；滑杆唯一消费点是 XDoApplyResid 的 PixAdaptLUT
         * （15 结点全等 = d8）：
         *     d8 = 1 + (base−1)·ec
         *     ec = {0(less), 0.1, 0.2, 0.3, 1.0, 1.25(more)}[档位]（本机类 floatC 表）
         *     base = max(0.3·(1−0.125·(p3c−1)) + 1.1, 1.1)   （ISO 派生；
         *           DP2Q0337 抓包 p3c=3.2 → base=1.3175；Portrait 匹配时另有 ÷2/÷1.1404）
         * 低档每格只 +0.1·(base−1)≈+0.03 —— SPP 滑杆「低档平缓、高档渐陡」的
         * 非线性就是这条插值自然给出的，不需要任何查表。
         *
         * 本地一阶近似：双边跑固定 auto 档（X3F_NR_EC_AUTO_LV，默认 2 = 现行
         * 0.5 档；滑杆不再直接抬 σ），双边滤掉的残差按 keep = 2−d8 线性混回。
         * SPP 原式是逐像素 mix（XDoApplyResid 对残差图按值整形：小残差压得更
         * 狠、大残差保住——即"同噪点、细节更多"的出处），标量版先复刻响应
         * 形状；待一次 抓包（XDoApplyResid 入/出各一）定 buffer 极性后
         * 升级成逐像素。keep 想换式子：X3F_NR_EC_KEEP 幂（keep=1 时不动）。
         * 默认关 —— 不设 X3F_NR_LUMA_EC 时与现行行为逐位一致。 */
        double ec_keep = 1.0;
        if (nr_flag("X3F_NR_LUMA_EC", 0)) {
            static const double ec_tab[4] = { 0.1, 0.2, 0.3, 1.0 };
            double base = nr_env("X3F_NR_EC_BASE", 1.3175);
            double more = nr_env("X3F_NR_EC_MORE", 1.0);   /* SPP more=1.25（>5 才到） */
            double ec   = (lv_raw >= 4) ? more : ec_tab[lv_raw - 1];
            double d8   = 1.0 + (base - 1.0) * ec;
            ec_keep = nr_env("X3F_NR_EC_KEEP", 0.0);
            if (ec_keep <= 0.0) ec_keep = 2.0 - d8;
            if (ec_keep < 0.0) ec_keep = 0.0; else if (ec_keep > 1.0) ec_keep = 1.0;
            lv = (int)nr_env("X3F_NR_EC_AUTO_LV", 2.0);
            if (lv < 0) lv = 0; else if (lv > 4) lv = 4;
            if (getenv("X3F_NR_DEBUG"))
                fprintf(stderr, "[nr] EC 模式 档=%d ec=%.3f base=%.4f d8=%.4f keep=%.4f "
                                "auto_lv=%d\n", lv_raw, ec, base, d8, ec_keep, lv);
        }
        int rmin = (int)nr_env("X3F_NR_LUMA_R", nr_luma_rad[lv]);
        int force_r = (int)nr_env("X3F_NR_R", 0.0);
        unsigned char *rmap = NULL;
        int block = 32, bw = 0, bh;
        double sig = nr_env("X3F_NR_LUMA_SIG", nr_luma_sig[lv]);
        double k = nr_env("X3F_NR_LUMA_K", 0.0);
        if (k <= 0.0) k = nr_env("X3F_NR_RANGE", 128.0) / 256.0;  /* SPP: k = rangeScale/256 */
        if (force_r > 0) {
            rmin = force_r;                       /* X3F_NR_R=<n> 强制固定半径 */
        } else if (nr_flag("X3F_NR_ADAPT", 1)) {
            /*自适应半径 = GenerateKernelMap（默认开；X3F_NR_ADAPT=0 关）*/
            bh = (h + block - 1) / block;
            rmap = (unsigned char *)malloc((size_t)((w + block - 1) / block) * bh);
            if (rmap != NULL)
                nr_build_radius_map(a, w, h, block, g_nr.sensor_iso, fs, rmap, &bw);
        }
        /* 多尺度原型（X3F_NR_PYR=1）：三层双边 + 1/σ² 合并，见 nr_pyramid_luma。
         * 默认关 —— 它是**结构近似**（每层的边缘图/局部均值/混合还没读透）。 */
        if (nr_flag("X3F_NR_PYR", 0))
            nr_pyramid_luma(b, a, w, h, rmin, k / sig, nmfs_lc, rmap, block, bw);
        else
            nr_bilateral2d(b, a, w, h, rmin, k / sig, nmfs_lc, 0, rmap, block, bw);
        if (rmap != NULL && getenv("X3F_NR_DEBUG")) {
            /*半径分布：阶梯给的是 2/3/4/5/9，看它到底落在哪几档*/
            int hist[16] = { 0 }, t, n = bw * ((h + block - 1) / block);
            for (t = 0; t < n; t++) hist[rmap[t] & 15]++;
            fprintf(stderr, "[nr]   半径分布");
            for (t = 0; t < 16; t++) if (hist[t]) fprintf(stderr, " %d:%d", t, hist[t]);
            fprintf(stderr, "\n");
        }
        free(rmap);
        { float *t = a; a = b; b = t; }
        /* EC 模式的残差回注（此时 a=双边结果，b=原始 lin）：
         * keep=1 → 原样（无 NR）、keep=0 → 纯 auto 双边。
         * X3F_NR_EC_MIX=1 时走下面的逐像素 mix（公式见块内注释），默认标量。 */
        if (ec_keep < 1.0) {
            if (nr_flag("X3F_NR_EC_MIX", 0)) {
                /* ★ 逐像素 mix（X3F_NR_EC_MIX=1，2026-10-27）：SPP XDoApplyResid
                 * 「按残差值整形」的工程等价（§26 抓包：小残差压得更狠、大残差
                 * 保住； A 分支与 LUT 是配套设计无法逐位拆解，这里复刻
                 * 响应形状）。判别量用 **3×3 局部残差能量** —— 单像素 r/σ 分
                 * 不开平坦与纹理（NLM3+双边后纹理残差幅度已落到噪声量级，
                 * 实测网格全过狠），而纹理像素的邻域能量高、孤立噪声低：
                 *   E   = 3×3 均值(Σ_c r_c²/3)          （局部残差能量，lin²）
                 *   s_c = E / (E + τ²σ_c²)              （σ_c² 折回 lin 域）
                 *   keep_px = kmin + (keep − kmin)·s_c
                 * σ²(v)=c0+c1·v+c2·v² 用 cn_iso（v=原始像素×fs=DN 域）。
                 * 旋钮：X3F_NR_EC_MIX_KMIN（默认 0.5）/ X3F_NR_EC_MIX_TAU（默认 1.2）
                 * —— DP0Q2315 网格最优平衡点 11.61/1.69/0.70（SPP mid
                 * 14.90/1.77/1.62：平坦暗正中目标、彩噪更好、纹理 −17%；
                 * §27.5 前沿：kmin0.7/τ1.5 = 12.47/2.03/0.80 更保纹理）。
                 * 默认关 —— 不设时走标量路径，逐位 no-op。 */
                const double kmin = nr_env("X3F_NR_EC_MIX_KMIN", 0.5);
                double tau2 = nr_env("X3F_NR_EC_MIX_TAU", 1.2);
                const size_t nn = (size_t)w * h;
                float *e2 = (float *)malloc(nn * sizeof(float));
                size_t i;
                int x, y, c;
                tau2 *= tau2;
                if (e2 != NULL) {
                    /* pass 1：逐像素通道平均残差能量 */
                    for (i = 0; i < nn; i++) {
                        const float *pb = b + i * 3;
                        const float *pa = a + i * 3;
                        double e = 0.0;
                        for (c = 0; c < 3; c++) {
                            const double r = (double)pb[c] - (double)pa[c];
                            e += r * r;
                        }
                        e2[i] = (float)(e / 3.0);
                    }
                    /* pass 2：3×3 盒均值 → E，逐通道整形回注 */
                    for (y = 0; y < h; y++) {
                        const int y0 = (y > 0) ? y - 1 : 0;
                        const int y1 = (y < h - 1) ? y + 1 : h - 1;
                        for (x = 0; x < w; x++) {
                            const int x0 = (x > 0) ? x - 1 : 0;
                            const int x1 = (x < w - 1) ? x + 1 : w - 1;
                            double e = 0.0;
                            int yy, xx;
                            for (yy = y0; yy <= y1; yy++)
                                for (xx = x0; xx <= x1; xx++)
                                    e += e2[(size_t)yy * w + xx];
                            e /= (double)((y1 - y0 + 1) * (x1 - x0 + 1));
                            {
                                float *pa = a + ((size_t)y * w + x) * 3;
                                const float *pb = b + ((size_t)y * w + x) * 3;
                                for (c = 0; c < 3; c++) {
                                    const double r = (double)pb[c] - (double)pa[c];
                                    const double v0 = (double)pb[c] * fs;
                                    const double s2 = g_nr.cn_iso[c]
                                                    + g_nr.cn_iso[3 + c] * v0
                                                    + g_nr.cn_iso[6 + c] * v0 * v0;
                                    double s, kp;
                                    if (s2 > 1e-12) {
                                        const double sl = s2 / (fs * fs);
                                        s = e / (e + tau2 * sl);
                                    } else {
                                        s = 1.0;       /* σ 退化：保住标量行为 */
                                    }
                                    kp = kmin + (ec_keep - kmin) * s;
                                    if (kp < 0.0) kp = 0.0; else if (kp > 1.0) kp = 1.0;
                                    pa[c] = (float)((double)pa[c] + kp * r);
                                }
                            }
                        }
                    }
                    free(e2);
                } else {
                    const size_t nn3 = nn * 3;         /* 内存不足：退标量 */
                    for (i = 0; i < nn3; i++)
                        a[i] = (float)((double)a[i] + ec_keep * ((double)b[i] - (double)a[i]));
                }
            } else {
                const size_t nn3 = (size_t)w * h * 3;
                size_t i;
                for (i = 0; i < nn3; i++)
                    a[i] = (float)((double)a[i] + ec_keep * ((double)b[i] - (double)a[i]));
            }
        }
        /* 结构门控（2026-11-01）：只给"基线档"用 —— 用户自己把滑块拉过基线时
         * gate 传 0，按用户要求原样跑满（见 nr_luma_baseline 的 from_base）。
         * 此时 a = 双边输出、b = 双边输入（上面刚 swap 过）。
         * EC 模式自带逐像素整形，两者叠在一起口径不清 —— EC 开时门控让路。 */
        if (gate && ec_keep >= 1.0) {
            nr_gate_mix(a, b, w, h, fs, gate_keep, gate_lo, gate_hi);
            if (getenv("X3F_NR_DEBUG"))
                fprintf(stderr, "[nr] 结构门控 on keep_max=%.2f lo=%.2f hi=%.2f\n",
                        gate_keep, gate_lo, gate_hi);
        }
        if (can_cache)
            apply_slot_put(&g_nr1_slot, &g_nr1_lock, lfp, nlk, 0,
                           w, h, n3, a, 0, 0, 0);
        if (getenv("X3F_NR_DEBUG"))
            fprintf(stderr, "[nr] 辉度 滑块=%.3f 档=%d 半径=%d σ缩放=%.3f "
                            "k=%.4f(rangeScale=%.0f) 半径图=%s(iso=%.0f) 域=%.0f\n",
                    luma, lv, rmin, sig, k, k * 256.0,
                    (rmap != NULL) ? "on" : ((force_r > 0) ? "fixed" : "off"),
                    g_nr.sensor_iso, nmfs_lc);
        }   /* !nr_hit */
        did = 1;
        if (getenv("X3F_NR_TIME"))
            fprintf(stderr, "[nr]   辉度耗时 %.2fs%s\n", nr_now() - t0,
                    nr_hit ? "（缓存）" : ""), t0 = nr_now();
    }
    if (chroma > 0.0) {
        const uint64_t nck = (apply_fp_d(luma) << 1) ^ apply_fp_d(chroma);
        int nr_hit = 0;
        if (can_cache)
            nr_hit = apply_slot_get(&g_nr2_slot, &g_nr2_lock,
                                    lfp, nck, 0, w, h, n3, a, NULL, NULL, NULL);
        if (!nr_hit) {
        int cv = nr_level(chroma);
        int r = (int)nr_env("X3F_NR_CHROMA_R", nr_chroma_rad[cv]);
        double k = nr_env("X3F_NR_CHROMA_K", 0.0);
        if (k <= 0.0) k = nr_env("X3F_NR_RANGE", 128.0) / 256.0;
        /* 2026-10-22 起 σ 表按 SPP 实测 BFS 曲线（见上面的表注）；
         * X3F_NR_CHROMA_LEGACY=1 回 2026-10-14 视觉标定旧表。 */
        const double *ctab = nr_flag("X3F_NR_CHROMA_LEGACY", 0)
                           ? nr_chroma_sig_legacy : nr_chroma_sig;
        double sig = nr_env("X3F_NR_CHROMA_SIG", ctab[cv]);
        /* **默认全分辨率**。半分辨率那条（X3F_NR_CH_HALF=1）快 12 倍，但它把色度
         * 降到半分辨率再升回来 —— 实测图像差是全分辨率的 85 倍（3.38 vs 0.040/255），
         * 本质是"给色度做 2×2 抽样"，边缘会带彩色晕。当加速选项可以，
         * 当默认不行。（ Stage1 确实在降采样上做，但那是它整条链的约定，
         * 我们这边只对色度做就等于单方面加了一层抽样。） */
        /* SPP Stage1 UV 预滤波（分类器，§19）：先洗深阴影近灰区，
         * 中调仍交给下面的双边（对应 NR3）。默认关。*/
        /* SPP NR3 层内 UV 执行器（§25，暗部色度整块替换的真身）。
         * 默认关；开着时替代双边（对应 SPP 层内它也跑在双边之前）。 */
        if (nr_flag("X3F_NR_CHROMA_UVF", 0)) {
            nr_chroma_uvf(b, a, w, h, fs, cv);
            { float *t = a; a = b; b = t; }
        } else if (nr_flag("X3F_NR_CHROMA_SPP", 0)) {
            nr_uv_prefilter(b, a, w, h, fs, cv);
            { float *t = a; a = b; b = t; }
        }
        if (nr_flag("X3F_NR_CHROMA_PYR", 0)) {
            /* 色度多尺度（SPP NR3 对应物，X3F_NR_CHROMA_PYR=1，默认关）：
             * 三层金字塔 + 逐层 σ 乘子 m_L = (base[L]/base[0])·(BFS0/BFS_L)。
             * base = {0.3,0.75,0.75}§22.1），BFS = 色度滑杆实测
             * m_fBFNoiseScaling（§20，1/3/5 档 + 几何插值；入口调制 k 在
             * BFS0/BFS_L 里约掉）。m_1=m_2 = 2.5·BFS0/BFS12：
             * 档5 → 1.67（粗层狠磨色斑）、档1 → 0.83（低档整体温和）——
             * "高档去斑不糊、低档碰不坏"就是这条比值给的。 */
            static const double base[3]  = { 0.3, 0.75, 0.75 };
            static const double bfs0[5]  = { 0.02, 0.045, 0.1, 0.32, 1.0 };
            static const double bfs12[5] = { 0.06, 0.17, 0.5, 0.87, 1.5 };
            double mul[3];
            mul[0] = 1.0;
            mul[1] = mul[2] = (base[1] / base[0]) * (bfs0[cv] / bfs12[cv]);
            g_pyr_bfs_mul = mul;
            g_pyr_chroma_mode = 1;
            nr_pyramid_luma(b, a, w, h, r, k / sig, nmfs_c, NULL, 0, 0);
            g_pyr_bfs_mul = NULL;
            g_pyr_chroma_mode = 0;
            if (getenv("X3F_NR_DEBUG"))
                fprintf(stderr, "[nr] 色度金字塔 m=[%.2f %.2f %.2f] r=%d σ=%.2f\n",
                        mul[0], mul[1], mul[2], r, sig);
        } else if (nr_flag("X3F_NR_CH_HALF", 0))
            nr_chroma_half(b, a, w, h, r, k / sig, nmfs_c);
        else
            nr_bilateral2d(b, a, w, h, r, k / sig, nmfs_c, 1, NULL, 0, 0);
        { float *t = a; a = b; b = t; }
        if (can_cache)
            apply_slot_put(&g_nr2_slot, &g_nr2_lock, lfp, nck, 0,
                           w, h, n3, a, 0, 0, 0);
        if (getenv("X3F_NR_DEBUG")) {
            const char *ec = getenv("X3F_NR_ISOGAIN_C");
            double g_l = nr_iso_gain();
            double g_c = (ec != NULL && atof(ec) > 0.0) ? atof(ec)
                       : (g_nr.quattro_model ? ((g_l > 2.0) ? g_l * 0.5 : 1.0) : g_l);
            fprintf(stderr, "[nr] 色度 滑块=%.3f 档=%d 半径=%d σ缩放=%.3f k=%.3f 域=%.0f "
                            "ISO增益=%.2f\n",
                    chroma, cv, r, sig, k, nmfs_c, g_c);
        }
        }   /* !nr_hit */
        did = 1;
        if (getenv("X3F_NR_TIME"))
            fprintf(stderr, "[nr]   色度耗时 %.2fs%s\n", nr_now() - t0,
                    nr_hit ? "（缓存）" : ""), t0 = nr_now();
    }
    }   /* 缓存键作用域（n3/cache_ok/lfp/can_cache） */
    if (banding > 0.0) {
        int lv = nr_level(banding);
        double strength = iso_scale * k_tab[lv];
        strength = nr_env("X3F_NR_K", 0.0) > 0.0 ? iso_scale * nr_env("X3F_NR_K", 0.0)
                                                 : strength;
        dir_env = getenv("X3F_NR_DIR");
        nr_banding_pass(a, a, w, h, half_tab[lv], strength, nmfs,
                        (dir_env != NULL && dir_env[0] == 'v') ? 1 : 0);
        did = 1;
        if (getenv("X3F_NR_TIME"))
            fprintf(stderr, "[nr]   条纹耗时 %.2fs\n", nr_now() - t0), t0 = nr_now();
        if (getenv("X3F_NR_DEBUG"))
            fprintf(stderr, "[nr] 条纹 滑块=%.3f 档=%d 半宽=%d 强度=%.4f(iso×%.3f) dir=%s\n",
                    banding, lv, half_tab[lv], strength, iso_scale,
                    (dir_env != NULL && dir_env[0] == 'v') ? "v" : "h");
    }
    /* 整链结果落槽（在条纹段之后 —— 条纹也改 a，属于"链"的一部分）。
     * 落槽的拷贝 ≈ 288MB（原生 Quattro），后台路几十毫秒，可接受。
     * 块级落小池（见上方 NR_F_POOL 注释），大帧仍落单槽。 */
    if (comb_ok) {
        if (nr_big_frame)
            apply_slot_put(&g_nrf_slot, &g_nrf_lock, cfk1, cfk2, 0,
                           w, h, n3f, a, 0, 0, 0);
        else
            apply_pool_put(g_nrf_pool, NR_F_POOL, &g_nrf_lock, &g_nrf_pool_rr,
                           cfk1, cfk2, 0, w, h, n3f, a);
    }
    free(b);

    if (getenv("X3F_NR_DEBUG")) {
        size_t i, n3 = (size_t)w * h * 3;
        double acc = 0.0;
        for (i = 0; i < n3; i++) acc += fabs((double)a[i] - (double)lin[i]);
        fprintf(stderr, "[nr]   Δ均值 %.4f/255（scene %.3g）图像域=%.0f 噪声域=%.0f\n",
                acc / (double)n3 * 255.0, acc / (double)n3, fs, nmfs);
    }

    if (*work_io != NULL) free(*work_io);
    *work_io = a;
    return did;
}

int x3f_render_apply_lut(const uint16_t *tone_lut,
                         const float *lin, int w, int h,
                         uint8_t *out, int stride)
{
    /* 曲线拟合对照用：不做高光去饱和，保持纯 LUT 语义。
     * 这条对照入口没有 x3f_edit_params_t，色彩分离固定不参与（传 NULL）。 */
    return apply_common(tone_lut, 1.0, 0, 0.0, NULL, NULL, NULL, NULL, NULL,
                        lin, w, h, out, stride, 8, NULL, NULL, NULL);
}

/* ---------- apply 分段计时（X3F_APPLY_TIME=1 打开）----------
 * 只为定位「拖滑杆慢」用：把 apply_ex_common 的每个阶段单独计时，
 * 顺序与真实执行顺序一致，每行打的是**这一段自身**的耗时。 */
static double apply_t_now(void)
{
#if defined(_WIN32)
    /* 与上面 nr_now 同一套：Windows 没有 clock_gettime，用 QPC（同样单调）。 */
    LARGE_INTEGER freq, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&now);
    return (double)now.QuadPart / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#endif
}
/* 进程内 running peak（phys_footprint 的最大值），给分段打点用。 */
static double apply_t_peak_mb(void)
{
    static double pk = 0.0;
    const double v = apply_t_footprint_mb();
    if (v > pk) pk = v;
    return pk;
}
static int apply_t_on(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("X3F_APPLY_TIME");
        v = (e != NULL && *e != '\0' && *e != '0') ? 1 : 0;
    }
    return v;
}

/* 与耗时同一个开关：每段跑完后进程的**峰值 RSS**（MB）。
 * 100MP 导出的 5GB 峰值不在 Swift 侧，而在显示域后段（detail / chrmed /
 * spatial）的整帧临时缓冲里 —— 这个打点就是用来指认究竟哪一段。 */
static double apply_t_footprint_mb(void)
{
#if defined(__APPLE__)
    /* phys_footprint 才是活动监视器那个数，也是内存压缩/GPU 缓冲之后的值；
     * ru_maxrss 只算已驻留页，两者在大图上能差 1GB 以上。 */
    task_vm_info_data_t vi;
    mach_msg_type_number_t n = TASK_VM_INFO_COUNT;
    if (task_info(mach_task_self(), TASK_VM_INFO, (task_info_t)&vi, &n) == KERN_SUCCESS)
        return (double)vi.phys_footprint / 1048576.0;
    return -1;
#elif defined(_WIN32)
    return -1;
#else
    struct rusage ru;
    if (getrusage(RUSAGE_SELF, &ru) != 0) return -1;
    return (double)(ru.ru_maxrss / 1024);
#endif
}

/* ── 预览 apply 的 GPU 链（2026-10-23）────────────────────────────────
 * 8 位路的 lutloop / 色度中值 / down8 三段上 Metal（own_gpu_apply_lut16 /
 * own_gpu_chrmed_u16 / own_gpu_down8）。Mac/Metal 下与 CPU double 链差 ±1 LSB 级
 * （float 精度，非逐位）。Windows/Vulkan（Intel UHD）2026-10-31 曾"整图错图"
 * （max=65535、99.8% 不一致），当日定位根因：Vulkan 对 storage/uniform 缓冲的
 * f32 类型读取整体返回 0（nlm3 全 u32 字节级通过佐证）——apply 链三个 shader
 * （own_apply_lut / own_chrmed_sel / own_chrmed_u16）全部改 array<u32> +
 * bitcast<f32> 还原后修复，gpu-cmp 复测 max=2、0.4% 像素 ±1~2（float 顺序差异
 * 经中值链放大，不可见）。默认恢复 ON；X3F_GPU_APPLY=0 仍可强制回 CPU。
 * **色度中值段例外（2026-10-31）**：默认回 CPU（Intel UHD 上 sel 二分 31 趟
 * kernel 反而慢），X3FDEC_GPU_CHRMED=1 才上 GPU，见 gpu_chrmed_enabled。 */
static int g_gpu_apply = -1;   /* -1 = 未显式设置，看 X3F_GPU_APPLY（默认开） */
void x3f_render_set_gpu_apply(int enable) { g_gpu_apply = enable ? 1 : 0; }
int  x3f_render_gpu_apply_enabled(void)
{
    if (g_gpu_apply >= 0) return g_gpu_apply;   /* 显式 set 优先 */
    {
        const char *e = getenv("X3F_GPU_APPLY");
        if (e != NULL && *e != '\0')
            return (*e == '0') ? 0 : 1;          /* 0 = 强制 CPU；其余 = 打开 */
#if defined(__APPLE__)
        /* macOS/Metal：apply 链与 CPU double 链差 ±1 LSB（float 精度，非逐位），
         * 而 macOS 侧的验收基准是「零滑杆出图与改前逐位一致」（md5 对质），
         * 所以这里仍默认 CPU —— 要试 GPU 请显式 X3F_GPU_APPLY=1 或
         * x3f_render_set_gpu_apply(1)。Windows/Vulkan 侧 2026-10-31 修完
         * f32 缓冲读取的根因后默认开，见上。 */
        return 0;
#else
        return 1;                                /* 2026-10-31 根因修复后默认开 */
#endif
    }
}

/* GPU apply 对拍校验开关（X3F_GPU_APPLY_CMP）。默认关；UI/CLI 设 1 打开。
 * 走 DLL 内部全局（g_gpu_apply_cmp），不依赖 getenv —— 因为壳层 /MT 与 DLL /MD
 * 的 CRT 环境副本错配，运行时 SetEnvironmentVariableA 的改动 DLL 的 getenv 读不到。 */
static int g_gpu_apply_cmp = 0;
void x3f_render_set_gpu_apply_cmp(int enable) { g_gpu_apply_cmp = enable ? 1 : 0; }

/* 色度中值段单独门控（2026-10-31 修订）：**默认 GPU**。
 *
 * 上午那版曾因「Intel UHD 上 chrmed_u16 实测 14.3s」默认回 CPU —— 那个结论
 * 是错的：慢的不是 GPU，是算法本身。旧 kernel 走键域二分 32 趟，每趟都把
 * 整个窗口从 workgroup 内存重读一遍（32 × N 次 shared 读），带宽成了唯一
 * 瓶颈；核显共享内存带宽本来就低，于是看起来像「GPU 不如 CPU」。换成独立
 * 显卡只是把这个错误结论掩盖过去，问题依旧存在。
 *
 * 现行实现改成**单趟排序网络**（own_nlm3.rs build_chrmed_med_wgsl）：窗口
 * N 个键一次读进寄存器，bitonic 网络排完直接取第 N>>1 小，shared 读
 * 1568 → 49 次。5424×3616 r=3 单平面实测：
 *    排序网络 0.26s  |  键域二分 2.67s  |  CPU 并行 ~4s
 * 与 CPU chrmed_select **逐位一致**（1962 万像素 bad=0，见 chrmed_med_bench.c）。
 *
 * r ≥ 4（窗口 169/225 元素）网络过大，仍走二分；X3FDEC_CHRMED_MED=0 可强制
 * 回退做 A/B。整段关 GPU：X3FDEC_GPU_CHRMED=0 或 x3f_render_set_gpu_chrmed(0)。 */
static int g_gpu_chrmed = -1;  /* -1 = 未显式设置，看 X3FDEC_GPU_CHRMED（默认开） */
void x3f_render_set_gpu_chrmed(int enable) { g_gpu_chrmed = enable ? 1 : 0; }
int  x3f_render_gpu_chrmed_enabled(void)
{
    if (g_gpu_chrmed >= 0) return g_gpu_chrmed;  /* 显式 set 优先 */
    {
        const char *e = getenv("X3FDEC_GPU_CHRMED");
        if (e != NULL && *e != '\0')
            return (*e == '0') ? 0 : 1;
        return 1;                                /* 默认 GPU */
    }
}

/* 对拍结果回报：默认走 stderr（与 Mac 端一致）；若 host 注册了回报回调，
 * 额外把结论行推给 UI（Windows 版用于面板内直接显示 [gpu-cmp] 结论）。 */
static void (*g_gpu_cmp_report)(const char *) = NULL;
void x3f_render_set_gpu_cmp_report(void (*cb)(const char *)) { g_gpu_cmp_report = cb; }

#if defined(HAVE_OWN_GPU) && HAVE_OWN_GPU
/* kernel 内硬编码了 SIZE=4096 / CM_N=65（MSL 看不见这里的宏），改宏必须
 * 同步 own_gpu.m 的 apply_lut / apply_lut16。LIN_MAX=8.0 / CM_R=0.6 同理
 * （预处理器不吃浮点常数，这里只能靠注释盯住）。 */
#if X3F_TONE_LUT_SIZE != 4096 || X3F_CM_N != 65
#error "GPU apply_lut 假定 X3F_TONE_LUT_SIZE=4096 / X3F_CM_N=65，改了请同步 own_gpu.m"
#endif
/* 前置条件：不满足 = 回 CPU（不是错误）。fill/mono/hl_y_only/实验项不复刻，
 * 这些支路要么带逐像素状态要么本来就是 env 实验项，保持 CPU 独占。 */
static int gpu_apply_gate(const x3f_edit_params_t *p, const mono_ctx_t *mono,
                          const float *fillgain)
{
    const char *e;
    if (!x3f_render_gpu_apply_enabled()) return 0;
    if (!own_gpu_available()) return 0;
    if (fillgain != NULL) return 0;        /* 填充光三支（display/huepres/satlift）不复刻 */
    if (mono != NULL) return 0;            /* 黑白支不复刻 */
    if (hl_y_only()) return 0;             /* 高光恢复保色支（env 实验项） */
    /* 广色域直出（2026-11-04 全链路广色域）：换域在 apply_row 末尾的 CPU
     * 逐像素段里做（decode → 矩阵 → 目标域编码，带负值奇对称外推）。GPU
     * 那条 kernel 只复刻 sRGB 码值，接了就等于把 sRGB 码值贴上 ProPhoto
     * 标签 —— 颜色全错，而且比不做还糟。 */
    if (p->out_space != X3F_OUT_SRGB) return 0;
    if (color_fit_enabled()) return 0;     /* X3F_COLOR_FIT（实验项） */
    if (g_classic_sd1) return 0;           /* SD1 保色度施加在 CPU 逐像素支路里 */
    if (p->split_sh_sat > 0.0 || p->split_hl_sat > 0.0)
        return 0;                          /* 色彩分离不复刻：回 CPU，预览/导出同一结果 */
    e = getenv("X3F_HL_WARM");
    if (e != NULL && *e != '\0' && *e != '0') return 0;   /* X3F_HL_WARM（实验项） */
    return 1;
}
/* 去饱和 clip 门槛：CPU 逐像素读 env（历史行为），GPU 每次调用读一次 */
static void gpu_desat_range(double *lo, double *hi)
{
    const char *e = getenv("X3F_HL_CLIP_LO");
    *lo = (e != NULL) ? atof(e) : X3F_HL_DESAT_CLIP_LO;
    e = getenv("X3F_HL_CLIP_HI");
    *hi = (e != NULL) ? atof(e) : X3F_HL_DESAT_CLIP_HI;
}
#endif /* HAVE_OWN_GPU */

/* 渲染主体：8 位与 16 位只差 bits（LUT、色彩匹配、去饱和、饱和度、空间处理
 * 全都共用一份代码，避免两条路慢慢走偏）。 */
/* 白平衡细调增益（色温/色调滑杆）：正常链与 mode8 直通两条路共用一份，
 * 保证语义不漂。这三行和 Eyedropper.swift 的反解是一对：改这里就要改那里。 */
static void wb_gain_calc(const x3f_edit_params_t *p, float g[3])
{
    g[0] = (float)(1.0 + 0.30*p->wb_temp + 0.12*p->wb_tint);
    g[1] = (float)(1.0 - 0.15*p->wb_tint);
    g[2] = (float)(1.0 - 0.30*p->wb_temp + 0.12*p->wb_tint);
    if (g[0] < 0.1f) g[0] = 0.1f;
    if (g[1] < 0.1f) g[1] = 0.1f;
    if (g[2] < 0.1f) g[2] = 0.1f;
    /* WB RGB 三通道强度（Mac wbRed/Green/Blue）：temp/tint 之上再乘线性域
     * 乘子（1.0 = 不动）。乘在 0.1 钳制之后 —— 它自己允许 0（通道关掉）。 */
    g[0] = (float)(g[0] * p->wb_red);
    g[1] = (float)(g[1] * p->wb_green);
    g[2] = (float)(g[2] * p->wb_blue);
}

/* ---------- mode 8：SPP「过曝矫正开」全局点变换直通（实验） ----------
 * 语义 = stage2_domain_spec.json + 残差 3D LUT：w = M·v + bias →
 * src_c = T_c(w_c) + Trilin(kOeSpecRes, clamp(src/31744)) →
 * out = combined[clamp(src_c, 31744)]。整条显示链换 ，
 * 不是在现有色调段上打补丁；滑杆全零时是 SPP 开的全局近似
 * （DP2Q2501 端到端 8bit MAE 2.84，参考真值对比；残差 LUT 拟合细节
 * 见 x3f_oe_spec_res.h 头注。单场景拟合，跨图泛化未验证）。 */
#include "x3f_oe_spec.h"
#include "x3f_oe_spec_res.h"
#include "x3f_oe_spec_dark2.h"   /* dark2 槽位（分支 m8-darkrecal）：Tc 暗段 Δ，默认关 */
/* fuse 解码 lin 域的另一套 M/T_c/残差 LUT（2026-10-25 重拟合）。combined
 * （kOeSpecCombX/Y）是 全局固定层（flut5 层，跨帧逐位相同），两路共用，
 * 由 x3f_oe_spec_fuse.h 间接 include own 那份，不重复生成。
 * 运行时按解码后端选路：oe_spec_set_cur(own_dispatch_used_fuse())。 */
#include "x3f_oe_spec_fuse.h"
#include "x3f_oe_spec_res_fuse.h"
/* mode9 专属表（own 域，v = **高光重建后** 的 lin 重拟合，2026-10-25 初版）。
 * mode8 表是按未重建 lin 拟合的：重建后的高 lin 值经 M·v 外推顶进 T_c 肩部
 * 钳制段 → 输出死白反而更多（实测 m9 用 mode8 表火核死白 14.6% vs m8
 * 11.3%）。combined 仍是 SPP 全局固定层（m9 头 include own 头共用）。
 * 拟合：fit_fuse.py m9（8×8 块 + 热启动 refine2）→ 初版全图 MAE 4.012、
 * 火核死白 11.2% vs SPP 6.56%；残差 450/13824 格（重建平台集中），未调优。 */
#include "x3f_oe_spec_m9.h"
#include "x3f_oe_spec_res_m9.h"
#include "x3f_oe_hl9corr.h"
#include "x3f_oe_hl9corr2.h"
/* v8 Δ 场（按 8bit 显示域判据重标定，2026-10-28）—— X3F_HL9_DELTA_V8=1 启用。
 * 默认不启用 → 逐位零回归。详见该头文件的注释。 */
#include "x3f_oe_hl9corr_v8.h"
/* 输出域 Δ 表（s 域架构改造：Δ 注入点移到 combined 之后，2026-10-29）——
 * X3F_HL9_ODOMAIN=1 启用。表值单位 = 8bit 级；标定 fit_delta_od.py。
 * 默认不启用 → 逐位零回归。详见该头文件的注释。 */
#include "x3f_oe_hl9od.h"
/* mode8 基线输出域 Δ 表（X3F_HL9_ODM8=1）：基线/标定集不同的姊妹表，
 * 见该头文件注释。 */
#include "x3f_oe_hl9od_m8.h"
#include "x3f_oec3.h"

typedef struct {
    const float *lin;
    void *out;            /* u8/u16 直写（显示域后段不参与时的路） */
    float *disp;          /* float 显示值（0..1）输出：喂给恒等 LUT 的 apply_common */
    int w, stride, bits;
    /* ---- 线性域前段（2026-10-24：让全部滑杆在 mode8 生效）----
     * gain     = WB 细调 × 曝光（逐通道，SPP 里都在过曝矫正之前）
     * fillgain = 填充光逐像素增益图（线性域乘性；display 域实验路不接）
     * cm       = 色彩模式**相对 Standard 的差**矩阵（直通的拟合吸收了
     *            Standard 档，见 g_cm_standard_m；恒等/无表 = NULL）
     * 全空 = 滑杆全零，逐位不变。 */
    const float *gain;
    const float *fillgain;
    const double *cm;
    /* ---- 显示域行内后段（与正常链 build_tone_lut 内的顺序一致）---- */
    double contrast;      /* 对比度（apply_contrast，显示域 logit） */
    double hl_recover;    /* 高光恢复（线性值驱动的肩部 blend，近似） */
    double rbase;         /* hl_recover 的 knee 显示锚点（= srgb_encode(tone_curve(rk))） */
    int cmtone_row;       /* 色彩模式色调曲线（除回 Standard 档 = 差） */
    int cmtone_row_std;
    double cmtone_w;
    /* 输出色彩空间（2026-11-10）：mode8 直写路以前把 SPP Stage2 的
     * **sRGB 显示码值**原样写进 8/16bit 缓冲，完全不看 ep->out_space，
     * 于是 Display P3 / ProPhoto 档贴着宽域 ICC 装的却是 sRGB 码值
     * ——「预览（恒 sRGB）正常、导出偏色」。这里把 out_space 带进
     * worker，在 put 汇合点按 apply_row 末尾同款换一次域。
     * disp 路（c->out == NULL）不走这里：它交给 apply_common 换域。 */
    int out_space;
} oe_spec_ctx_t;

static float oe_spec_interp1(const float *xs, const float *ys, int n, float x)
{
    int lo = 0, hi = n - 1;
    float t;
    if (x <= xs[0]) return ys[0];
    if (x >= xs[hi]) return ys[hi];
    while (hi - lo > 1) {
        int mid = (lo + hi) >> 1;
        if (xs[mid] <= x) lo = mid; else hi = mid;
    }
    t = (x - xs[lo]) / (xs[hi] - xs[lo]);
    return ys[lo] + t * (ys[hi] - ys[lo]);
}

/* ---- mode8 逐像素表的运行时分路 ----
 * mode8 是逐像素点变换，表严格绑定解码 lin 域：own 表（历史基线，
 * 零滑杆 md5 铁律约束，数值与选表前的编译期常量逐位一致）+ fuse 表
 * （x3f_oe_spec_fuse.h）。渲染线程在 x3f_parallel_for 之前调
 * oe_spec_set_cur(own_dispatch_used_fuse()) 一次；worker 内只读，
 * 无锁。own_dispatch_used_fuse() 与分发钩子同为线程局部，同线程
 * 先解码后渲染，读到的就是本次解码真实走的后端。 */
typedef struct {
    const double (*M)[3];         /* [4][3]：前三行系数，末行 bias */
    const float *tcX[3], *tcY[3]; /* 逐通道 w→s 曲线 */
    int         tcN[3];
    const int16_t (*res)[3];      /* 残差 3D LUT，index=(ix*G+iy)*G+iz */
    int         res_g;
    float       res_scale;        /* u = clamp(s / res_scale, 0, 1) */
} oe_spec_tabs_t;

static const oe_spec_tabs_t oe_spec_tabs_own = {
    kOeSpecM,
    { kOeSpecTc0X, kOeSpecTc1X, kOeSpecTc2X },
    { kOeSpecTc0Y, kOeSpecTc1Y, kOeSpecTc2Y },
    { OE_SPEC_TC0_N, OE_SPEC_TC1_N, OE_SPEC_TC2_N },
    kOeSpecRes, OE_SPEC_RES_G, 31744.0f,
};
static const oe_spec_tabs_t oe_spec_tabs_fuse = {
    kOeSpecFuseM,
    { kOeSpecFuseTc0X, kOeSpecFuseTc1X, kOeSpecFuseTc2X },
    { kOeSpecFuseTc0Y, kOeSpecFuseTc1Y, kOeSpecFuseTc2Y },
    { OE_SPEC_FUSE_TC0_N, OE_SPEC_FUSE_TC1_N, OE_SPEC_FUSE_TC2_N },
    kOeSpecResFuse, OE_SPEC_FUSE_RES_G, OE_SPEC_FUSE_RES_SCALE,
};
/* mode9 专属（own 域重建 lin 拟合）。fuse 域的 m9 表未拟合：mode9+fuse
 * 解码时回落 fuse 表（重建 lin 喂 fuse 表，行为未标定，先跑通）。 */
static const oe_spec_tabs_t oe_spec_tabs_m9 = {
    kOeSpecM9M,
    { kOeSpecM9Tc0X, kOeSpecM9Tc1X, kOeSpecM9Tc2X },
    { kOeSpecM9Tc0Y, kOeSpecM9Tc1Y, kOeSpecM9Tc2Y },
    { OE_SPEC_M9_TC0_N, OE_SPEC_M9_TC1_N, OE_SPEC_M9_TC2_N },
    kOeSpecResM9, OE_SPEC_M9_RES_G, OE_SPEC_M9_RES_SCALE,
};
/* 非 thread-local：oe_spec_row 跑在 parallel_for 的 worker 线程里，必须
 * 全局可见。写点只有 oe_spec_set_cur()（渲染线程、parallel_for 之前），
 * 渲染期间只读；产品同一时刻只有一路渲染，无竞争。 */
static const oe_spec_tabs_t *g_oespec = &oe_spec_tabs_own;

static void oe_spec_set_cur(int fuse)
{
    if (getenv("X3F_OE_SEL_DEBUG"))
        fprintf(stderr, "[oe8] set_cur fuse=%d thread=%p\n", fuse, (void*)pthread_self());
    g_oespec = fuse ? &oe_spec_tabs_fuse : &oe_spec_tabs_own;
}

/* ---------- mode 9：p 层高光重建 v1（B 驱动结构展开，实验） ----------
 * 依据 ）：
 * SPP 在 Stage2 前用 F23SigmaHighlightRestoration2 展开传感器裁剪区；
 * 实测（DP2Q2501）我方裁剪区内 R/G 是平台，**B 层保留真实结构**
 * （Foveon B 饱和最深），SPP 展开值可由 B_μ7 幂律预测：
 *     est_c = C_c + A_c · (B_mu7 / WHITE)^g_c      （仅替换裁剪区）
 * mode9 = 在喂 oe_spec_row（mode8 链）之前对 lin 的裁剪区做该重建，
 * 其余与 mode8 完全一致 —— 用于与 mode7/mode8 并列对比。
 *
 * 域说明：v1 参数在 16384 满阱域拟合（SPP in.bin 域）；产品 lin 是归一化
 * 域（own 火核平台 ~2.4 / fuse ~1.58，见下方门控注释）。这里用
 * HL9_LIN_TO_WB 逐通道增益把 lin 抬回满阱域算 est，再除回去写回 lin，
 * 参数即可原样使用。增益按 2501 门控平台值反推（16384/平台），
 * 精确标定待 X3F_OE_SPEC_DUMP 对齐回归（§13.13 后续①）。 */
#define HL9_WHITE   16384.0
#define HL9_SIGMA   7          /* B 通道平滑窗（μ7，与 hl_restore_v1.py 一致） */
/* lin 域逐通道白点 = own 域的**裁剪平台值**（2026-10-28 v6 重标定）。
 *
 * ★ v6 的关键修正（此前 v3/v4/v5 全错在这一步）：
 *   域系数必须**分区域**标定。SPP 与 own 是两条独立解码链，
 *   各自的"白点"定义与裁剪行为不同：
 *     - SPP 在裁剪区把 R/G 拉到平台 14335（14-bit 满阱）
 *     - own 解码器在裁剪区 R/G 平台只有 1.771 / 0.799（own 归一化域）
 *   ⇒ 裁剪区专用系数 Kc = SPP平台/own平台 =
 *       [8094.3, 17941.2, 4984.3]，与非裁剪区系数 Kn =
 *       [10610.8, 12474.7, 10796.5] 相差 −24% / +44% / −54%。
 *   用非裁剪区系数外推裁剪区真值 → 拟合目标全错 →
 *   端到端 bias 爆掉（G −7697）。**这是域外推的铁律。**
 *
 * own 域白点 = own 裁剪平台众数 = [1.771, 0.799, 2.869]
 *   （mode9 的 mx = max_c(lin/wlin) 判"是否裁剪"，尺度必须与 own 域一致）
 * fuse 域未标定（暂借 own）。 */
static const double hl9_wlin[2][3] = {
    { 0.99516, 1.11761, 1.80162 },   /* own 域：v3（现产品默认） */
    { 0.99516, 1.11761, 1.80162 },   /* fuse 域（未标定，暂借 own） */
};
/* 2501 lin 域重拟合参数（2026-10-28 v6 重标定）。
 *
 * 标定链（三代踩坑的最终正确版）：
 *   ① SPP 帧(2944×1836) 的 active 起于 (108,14) 尺寸 2712×1808；
 *      own 全分辨率 active 5424×3616，**无暗边界**；
 *      映射 = SPP(i,j) ↔ own(2i,2j)，**无额外偏移**（实测相关 r=0.9933）。
 *      ★ 此前记忆里"偏移 216"是误记，勿再沿用。
 *   ② 域系数**分区域**：裁剪区 Kc（平台比）+ 非裁剪区 Kn（分位比）。
 *   ③ bn 必须在**全分辨率**上算 box7(B)（同 C 侧 hl9_box7）。
 *      ★ v4 曾在半分辨率上算（≈全分辨率 box13），bn 差 12% → 参数含义漂移。
 * ④ 标定完**立刻端到端自检**（同脚本内复刻 C 侧 hl9_restore → 对 参考真值算 bias）。
 *      ★ 拟合集 MAE 好 ≠ 端到端好，v4/v5 都栽在这。
 *
 * v6 结果（标定集 91250 px = own∩SPP 裁剪区）：
 *   R: g=10.0 A=2.653344 C=2.425572  MAE 0.9775 (基线 2.1202, 提升 2.17×)
 *   G: g= 7.0 A=1.252153 C=0.903530  MAE 0.3296 (基线 0.7912, 提升 2.40×)
 *   B: knee=0.80 blend_w=0.20，两段 MAE 0.7988 (基线 1.8094, 提升 2.27×)
 *
 * ★★ 端到端自检（复刻 C 侧 hl9_restore，对 SPP post 真值，SPP 域单位）：
 *      v3(现产品)  bias R−10567  G+14705  B−6424  | MAE 12570/15005/7170
 *      v5(错误域)  bias R−12965  G  +741  B−10247 | MAE 14500/ 9832/10461
 *      v6(本次)   bias R    −2  G    −1  B  −284  | MAE  7914/ 5915/ 3983
 *      ⇒ v6 的三通道 bias 几乎归零，MAE 全面优于 v3。
 *
 * ⚠️ 已知局限（下一个会话的主攻点）：
 *   * R 通道仍不可由 B 驱动特征重建（留出检验泛化比 >1.0）——
 *     own R 在裁剪区有结构但特征选不对；正解 = 先重建 G 再用通道比例恢复 R。
 *   * 纯幂律在 **SPP 自己的域**上也只有 MAE 5430（白点 39%）→ SPP 用的是
 *     NewBloomingCorr 空间邻域投票 + PreBloomingCorr 梯度生长，
 *     点变换的**形式**上限有限，这是"等高线/空间错位"的根因。
 */
/* ★★ 参数组选择：v3(现产品，默认) vs v6(2026-10-28 重标定)。
 *
 * ⚠️ v6 **默认关闭**，由 X3F_HL9_V6=1 开启。原因是 v6 只重标了**重建层**，
 *   下游 Δ 场表（hl9c_delta / hl9c2_delta / kHl9HaloLUT / HL9_FLOOR_*）
 *   仍是 2026-10-26 在 v3 参数下标的 —— Δ 表查表坐标 (bn,dens,mG) 与
 *   wlin/bn 定义绑定，v6 改了 wlin ⇒ 查表坐标全错位 ⇒ 开箱即用会引入新误差。
 *   ⇒ 必须等 Δ 场重标定完成后才切默认（见 hl9_restore 参数组逻辑）。
 *
 * 端到端自检（复刻 C 侧 hl9_restore，对 SPP post 真值，SPP 域单位）：
 *   v3(现产品)  bias R−10567  G+14705  B−6424  | MAE 12570/15005/7170
 *   v6(本次)   bias R    −2  G    −1  B  −284  | MAE  7914/ 5915/ 3983
 *   梯度比（等高线强度）1.990 → 1.034；8bit G 中位 700 → 409（SPP 381）。 */
static const double hl9_wlin_v3[3] = { 0.99516, 1.11761, 1.80162 };
static const double hl9_wlin_v6[3] = { 1.771000, 0.799000, 2.869000 };
static const double hl9_params_v3[3][3] = {
    { 8.0, 0.051413, 1.317236 },
    { 4.5, 0.290185, 1.101550 },
    { 2.5, 0.872239, 0.892128 },
};
static const double hl9_params_v6[3][3] = {
    { 10.0, 2.653344, 2.425572 },
    {  7.0, 1.252153, 0.903530 },
    {  1.5, 1.586313, 1.754839 },
};
/* B 通道两段膝点：v3 / v6 两套 */
static const double hl9_b_v3[5] = { 1.40, 0.15, 1.0, 1.1048, 0.7305 };  /* knee,bw,lo_g,lo_A,lo_C */
static const double hl9_b_v3_hi[3] = { 6.0, 0.1652, 1.1140 };
static const double hl9_b_v6[5] = { 0.80, 0.20, 1.5, 1.586313, 1.754839 };
static const double hl9_b_v6_hi[3] = { 10.0, 3.075528, 2.587645 };

/* B 通道两段膝点模型（2026-10-25 v4 引入结构，2026-10-28 v6 重标定）：
 * 参考真值呈两段结构——低段 B 压低保黄（HLP 补色机制），高段 B 陡升变白；
 * 单段幂律在过渡带过冲 +26~31% → 输出 B 被推到 251 死白。
 *
 * ★ v6 重标定：膝点与两段参数均按 SPP F23Sigma 出口真值重标（裁剪区系数 Kc），
 *   目标域 = own 归一化（白点 = own 裁剪平台），标定集 91250 px。
 *   knee=0.80 blend_w=0.20 → MAE 0.7988（基线 1.8094，提升 2.27×）。
 *   lo 段 g=1.5 A=1.586313 C=1.754839；hi 段 g=10.0 A=3.075528 C=2.587645。
 *   膝点处两段跳变仅 0.028（可忽略），mix 严格单调 ✓
 *
 * ⚠️ hi 段是 g=10 的陡幂律，外推极猛（bn=1.7 → est 622 = 白点 43 倍）。
 *   实际 bn p90=1.00，风险低；但仍加输出上限保护（见 HL9_EST_MAX）。
 *   （v3/v6 两套参数见上方 hl9_b_v3 / hl9_b_v6，运行时由 X3F_HL9_V6 选择。） */
/* 重建值上限（相对 own 白点）：SPP 实测最高 G = 64700 = 白点 4.62 倍。
 * 取 8.0 留足余量，只兜外推爆掉，不影响正常区间。 */
#define HL9_EST_MAX 8.0
/*软掩膜羽化：展开过渡带很宽（s∈[0.90,1.35] 比值 1.00→1.37 渐进），
 * 硬 0.98 掩膜在边界产生亮度/色度台阶。羽化带 [HL9_FEATHER_LO,0.98)：
 * w=smoothstep 渐入，est·w + raw·(1−w)；≥0.98 全替换。 */
#define HL9_FEATHER_LO 0.92

/* B 通道 7×7 box blur（可分离，边界 replicate）。 */
static void hl9_box7(const float *src, float *dst, int w, int h)
{
    const int R = HL9_SIGMA / 2;
    float *tmp = (float *)malloc((size_t)w * h * sizeof(float));
    if (tmp == NULL) { memset(dst, 0, (size_t)w * h * sizeof(float)); return; }
    int x, y, k;
    for (y = 0; y < h; y++) {
        const float *s = src + (size_t)y * w;
        float *d = tmp + (size_t)y * w;
        for (x = 0; x < w; x++) {
            float acc = 0.0f;
            for (k = -R; k <= R; k++) {
                int xx = x + k;
                if (xx < 0) xx = 0; if (xx >= w) xx = w - 1;
                acc += s[xx];
            }
            d[x] = acc / HL9_SIGMA;
        }
    }
    for (y = 0; y < h; y++) {
        float *d = dst + (size_t)y * w;
        for (x = 0; x < w; x++) {
            float acc = 0.0f;
            for (k = -R; k <= R; k++) {
                int yy = y + k;
                if (yy < 0) yy = 0; if (yy >= h) yy = h - 1;
                acc += tmp[(size_t)yy * w + x];
            }
            d[x] = acc / HL9_SIGMA;
        }
    }
    free(tmp);
}

/* lin（w×h×3，显示方向 RGB）裁剪区高光重建 v1（lin 域直接算，无域往返）。
 * 返回 malloc 的重建副本；失败返回 NULL（调用方回落 mode8 行为）。
 * 同时产出空间修正的输入图（mode9 专用，见 x3f_oe_hl9corr.h）：
 *   g_hl9_bn   = 每像素 bn（=box7(B)/wlin_B，未钳原值）
 *   g_hl9_mask = 重建作用区（max_c(lin/wlin) ≥ HL9_FEATHER_LO）
 *   g_hl9_dens = mask 的 31×31 box 均值（bloom 深度代理）
 *   g_hl9_mg   = 重建前 G/wlin_G（v6 第三维：掩膜内约 72% 像素 G 未裁，
 *                仍带真实场景信息）
 * 四者在 oe_spec_row 里被读（只读），调用方在链路结束后释放。 */
static float   *g_hl9_bn   = NULL;
static uint8_t *g_hl9_mask = NULL;
static float   *g_hl9_dens = NULL;
static float   *g_hl9_mg   = NULL;
static int      g_hl9_w = 0, g_hl9_h = 0;
/* [诊断] X3F_OE9_S_DUMP=路径：倒出空间修正注入后的 s（combined 之前），
 * 与 python 离线重放逐位对质用（浮点，w*h*3，与 lin dump 同方向）。 */
static float   *g_hl9_sdump = NULL;
static const char *g_hl9_sdump_path = NULL;
static void hl9_build_delta_fields(int w, int h);   /* 定义在 hl9_restore 之后 */
static void hl9_free_delta_fields(void);

/* [dark2 拟合 dump 2026-11-01，分支 m8-darkrecal] X3F_DARK2_FITDUMP=路径：
 * oe_spec_row 内把 (w0..w2, s_pre0..2, s_post0..2) 九元组 ::2 抽样落盘，
 * 供 fit_dark2.py 拟合 Tc 暗段 Δ（s_pre = Tc 输出未注入前；s_post = 残差
 * 之后 combined 之前 —— 含残差与 dark2 Δ。拟合须在 dark2 关（基线）下采）。
 * 索引 (row>>1, x>>1)，缓冲 fw=(w+1)/2, fh=(h+1)/2。 */
static float       *g_dark2_fit = NULL;
static int          g_dark2_fw = 0, g_dark2_fh = 0;
static const char  *g_dark2_fit_path = NULL;
static void dark2_fit_init(int w, int h)
{
    const char *e;
    if (g_dark2_fit != NULL) return;
    e = getenv("X3F_DARK2_FITDUMP");
    if (e == NULL || e[0] == '\0') return;
    g_dark2_fit_path = e;
    g_dark2_fw = (w + 1) / 2;
    g_dark2_fh = (h + 1) / 2;
    g_dark2_fit = (float *)malloc((size_t)g_dark2_fw * (size_t)g_dark2_fh * 9 * sizeof(float));
    if (g_dark2_fit != NULL)
        memset(g_dark2_fit, 0, (size_t)g_dark2_fw * (size_t)g_dark2_fh * 9 * sizeof(float));
}
static void dark2_fit_flush(void)
{
    FILE *f;
    if (g_dark2_fit == NULL || g_dark2_fit_path == NULL) return;
    f = fopen(g_dark2_fit_path, "wb");
    if (f != NULL) {
        fwrite(g_dark2_fit, sizeof(float), (size_t)g_dark2_fw * (size_t)g_dark2_fh * 9, f);
        fclose(f);
        fprintf(stderr, "[dark2] fit dump -> %s (%dx%d ::2)\n",
                g_dark2_fit_path, g_dark2_fw, g_dark2_fh);
    }
    free(g_dark2_fit);
    g_dark2_fit = NULL;
}

/* v7.1 去等高线抖动：重建区是 box7(B) 的光滑函数（无颗粒），8bit 输出在
 * 平滑梯度上量化出 1 级台阶并连成同心等高线（用户可见的"橙圈"）。
 * 天然颗粒正好起了 dither 作用。这里用 8×8 Bayer 有序抖动（±0.5 个 8bit
 * 级，加在 combined 输出域 o 上）打散量化边界——视觉等效颗粒、幅度低于
 * 可见阈、不影响 MAE（+0.09）。只作用于 oe9 路径，m7/m8 逐位不变。 */
static const unsigned char kHl9Bayer8[64] = {
     0, 32,  8, 40,  2, 34, 10, 42,
    48, 16, 56, 24, 50, 18, 58, 26,
    12, 44,  4, 36, 14, 46,  6, 38,
    60, 28, 52, 20, 62, 30, 54, 22,
     3, 35, 11, 43,  1, 33,  9, 41,
    51, 19, 59, 27, 49, 17, 57, 25,
    15, 47,  7, 39, 13, 45,  5, 37,
    63, 31, 55, 23, 61, 29, 53, 21,
};

static void hl9_free_space(void)
{
    free(g_hl9_bn); free(g_hl9_mask); free(g_hl9_dens); free(g_hl9_mg);
    g_hl9_bn = NULL; g_hl9_mask = NULL; g_hl9_dens = NULL; g_hl9_mg = NULL;
    g_hl9_w = 0; g_hl9_h = 0;
    hl9_free_delta_fields();
}

/* 可分离 K×K box 均值（边界 replicate），src/dst/tmp 均 w×h float。 */
static void hl9_boxk(const float *src, float *dst, float *tmp, int w, int h, int K)
{
    const int R = K / 2;
    int x, y, k;
    for (y = 0; y < h; y++) {
        const float *s = src + (size_t)y * w;
        float *d = tmp + (size_t)y * w;
        for (x = 0; x < w; x++) {
            float acc = 0.0f;
            for (k = -R; k <= R; k++) {
                int xx = x + k;
                if (xx < 0) xx = 0; if (xx >= w) xx = w - 1;
                acc += s[xx];
            }
            d[x] = acc / K;
        }
    }
    for (y = 0; y < h; y++) {
        float *d = dst + (size_t)y * w;
        for (x = 0; x < w; x++) {
            float acc = 0.0f;
            for (k = -R; k <= R; k++) {
                int yy = y + k;
                if (yy < 0) yy = 0; if (yy >= h) yy = h - 1;
                acc += tmp[(size_t)yy * w + x];
            }
            d[x] = acc / K;
        }
    }
}

/* X3F_HL9_ODM8=1：od Δ 挂 mode8 基线（2026-10-29）。
 * 依据（fit_od_m8.py 全链模拟，2501 焰核）：mode8 曲线即 SPP Stage2 重放，
 * 过渡带（180-220 bias −0.31）远优于 m9 重建链；od Δ 在 m8 基线上重标
 * （cal=施加域全体 mask>0，修 fit_delta_od 的「标定域≠施加域」+43 推亮 bug）
 * 后亮区子集 2.431 / 全窗 4.768 / 全图 3.104，全面优于 od-on-m9（2.877 /
 * 8.593 / 3.405）。启用时 hl9_restore 只建特征场与 od 场（fields-only，
 * 返回 NULL），lin 不重建；注入门控见 oe_spec_row。默认关零回归。 */
/* App 显式开关（2026-10-30 接 App UI）：优先于 env；-1 = 未设置，走 env。
 * ⚠️ env 判据只看非空——X3F_HL9_ODM8=0 也是「开」，做 A/B 必须 unset
 * （2026-10-30 renderlab 对质踩实）。App 一律走 setter。 */
static int g_hl9_odm8_force = -1;
void x3f_render_set_hl9_odm8(int enable) { g_hl9_odm8_force = enable ? 1 : 0; }

static int hl9_odm8(void)
{
    if (g_hl9_odm8_force >= 0) return g_hl9_odm8_force;
    static int init = 0, v = 0;
    if (!init) {
        const char *e = getenv("X3F_HL9_ODM8");
        v = (e != NULL && *e != '\0');
        init = 1;
    }
    return v;
}

static float *hl9_restore(const float *lin, int w, int h)
{
    /* ★ 2026-10-30 崩溃修复：特征场/od 场的生命周期改为「下一次解码前释放」。
     * 旧实现把 hl9_free_space() 放在渲染完成后——App 是一帧多次渲染 + 多队列
     * 并发（预览 vs x3f.detail 裁剪 vs 导出），第一轮渲染完 free 掉 od/bn 等
     * 场（MALLOC_LARGE 释放即 munmap），并发中的另一路 oe_spec_row 再读就是
     * SIGSEGV（X3FExtractor-2026-10-30-202838.ips 实锤）；即便不并发，第二次
     * 渲染也会因指针已 NULL 而静默丢 Δ/去等高线修正。场是解码期产物、渲染期
     * 只读，持有到下一次 hl9_restore 入口（或进程退出）是安全且正确的。
     * 内存代价：一帧的场常驻（od int16 3 平面 + bn/mask/dens/mg），可接受。 */
    hl9_free_space();
    const size_t nn = (size_t)w * h;
    float *out = (float *)malloc(nn * 3 * sizeof(float));
    float *bmu = (float *)malloc(nn * sizeof(float));
    uint8_t *msk = (uint8_t *)malloc(nn);
    float *dens = (float *)malloc(nn * sizeof(float));
    float *blur = (float *)malloc(nn * sizeof(float));
    float *mg = (float *)malloc(nn * sizeof(float));
    const int fuse = (g_oespec == &oe_spec_tabs_fuse);
    /* ★ 参数组选择（2026-10-28）：v3(默认) / v6(X3F_HL9_V6=1)。
     * v6 的重建层已对齐 SPP F23Sigma 真值（端到端 bias 归零），
     * 但下游 Δ 场表仍是 v3 口径标的 → 默认关闭，待 Δ 场重标定后切换。 */
    const int v6 = (getenv("X3F_HL9_V6") != NULL);
    /* ★ B 通道可独立开关（2026-10-28 第 2 会话，SPP 8bit 真值实测）：
     *   X3F_HL9_B_V3=1 → B 用 v3 的两段膝点参数（配 v6 的 wlin）
     *   X3F_HL9_NO_B=1  → B 通道**完全不重建**（保留原始 lin）
     *   背景：裁剪区 bias vs SPP 8bit 真值（cap2501_on.tif）：
     *     mode8(不重建) B bias +8.27 MAE 11.11  ← 最好
     *     mode9 v3      B bias −11.54 MAE 26.12
     *     mode9 v6      B bias +26.53 MAE 26.75
     *   ⇒ **B 通道重建整体有害**，是「粉红块」的真正来源（不是缺绿）。
     *   R/G 在 v3/v6 都已达标（bias ±1 级别）。
     *   注意：B 参数组与 wlin 绑定（bn 定义不同），不能随意混用 ——
     *   实测 v6(R/G)+v3(B) 会让 G bias 恶化到 −12.82（M 矩阵 3×3 耦合）。
     */
    const int b_v3 = (getenv("X3F_HL9_B_V3") != NULL);
    const int no_b = (getenv("X3F_HL9_NO_B") != NULL);
    const double *wlin = v6 ? hl9_wlin_v6 : hl9_wlin[fuse ? 1 : 0];
    const double (*params)[3] = v6 ? hl9_params_v6 : hl9_params_v3;
    const int use_b_v6 = (v6 && !b_v3);
    const double b_knee    = use_b_v6 ? hl9_b_v6[0] : hl9_b_v3[0];
    const double b_blend_w = use_b_v6 ? hl9_b_v6[1] : hl9_b_v3[1];
    const double b_lo_g    = use_b_v6 ? hl9_b_v6[2] : hl9_b_v3[2];
    const double b_lo_a    = use_b_v6 ? hl9_b_v6[3] : hl9_b_v3[3];
    const double b_lo_c    = use_b_v6 ? hl9_b_v6[4] : hl9_b_v3[4];
    const double b_hi_g    = use_b_v6 ? hl9_b_v6_hi[0] : hl9_b_v3_hi[0];
    const double b_hi_a    = use_b_v6 ? hl9_b_v6_hi[1] : hl9_b_v3_hi[1];
    const double b_hi_c    = use_b_v6 ? hl9_b_v6_hi[2] : hl9_b_v3_hi[2];
    int y, x, c;
    if (out == NULL || bmu == NULL || msk == NULL || dens == NULL || blur == NULL || mg == NULL) {
        free(out); free(bmu); free(msk); free(dens); free(blur); free(mg);
        return NULL;
    }
    if (!hl9_odm8())
        memcpy(out, lin, nn * 3 * sizeof(float));
    /* B 通道（第 3 通道，Foveon 最深层）7×7 平滑 */
    for (y = 0; y < h; y++) {
        const float *s = lin + (size_t)y * w * 3 + 2;
        float *d = bmu + (size_t)y * w;
        for (x = 0; x < w; x++) d[x] = s[(size_t)x * 3];
    }
    hl9_box7(bmu, bmu, w, h);
    /* [TEMP-DEBUG] X3F_OE9_BMU_DUMP=路径 倒出 box7 后的 bmu（定位表/渲染失配用） */
    {
        const char *eb = getenv("X3F_OE9_BMU_DUMP");
        if (eb != NULL && *eb != '\0') {
            FILE *fb = fopen(eb, "wb");
            if (fb != NULL) {
                fwrite(bmu, sizeof(float), nn, fb);
                fclose(fb);
                fprintf(stderr, "[oe9] dump bmu -> %s\n", eb);
            }
        }
    }
    /* 空间修正输入图：mask 图 → 31×31 box 均值 = bloom 深度代理 */
    for (y = 0; y < h; y++) {
        const float *l = lin + (size_t)y * w * 3;
        uint8_t *m = msk + (size_t)y * w;
        for (x = 0; x < w; x++) {
            double m0 = l[x*3]   / wlin[0];
            double m1 = l[x*3+1] / wlin[1];
            double m2 = l[x*3+2] / wlin[2];
            double mx = m0 > m1 ? (m0 > m2 ? m0 : m2) : (m1 > m2 ? m1 : m2);
            m[x] = (mx >= HL9_FEATHER_LO) ? 1 : 0;
        }
    }
    for (y = 0; y < h; y++) {
        const uint8_t *m = msk + (size_t)y * w;
        float *d = dens + (size_t)y * w;
        for (x = 0; x < w; x++) d[x] = (float)m[x];
    }
    hl9_boxk(dens, dens, blur, w, h, 31);
    free(blur);
    /* 裁剪区替换：mask = max_c(lin_c/wlin_c) >= 0.98（逐通道白点归一，
     * 与 v1 的 16k 域 max≥0.98·16384 同构；拟合时 mask 128714 px）。
     * X3F_HL9_ODM8=1 时整段跳过（fields-only：只建特征场与 od 场）。 */
    if (!hl9_odm8()) {
        const double bnorm = 1.0 / wlin[2];
        for (y = 0; y < h; y++) {
            const float *br = bmu + (size_t)y * w;
            float *o = out + (size_t)y * w * 3;
            const float *l = lin + (size_t)y * w * 3;
            for (x = 0; x < w; x++) {
                double m0 = l[x*3]   / wlin[0];
                double m1 = l[x*3+1] / wlin[1];
                double m2 = l[x*3+2] / wlin[2];
                double mx = m0 > m1 ? (m0 > m2 ? m0 : m2) : (m1 > m2 ? m1 : m2);
                if (mx < HL9_FEATHER_LO) continue;
                double bn = br[x] * bnorm;
                if (bn < 0) bn = 0;   /* v7.1：负值钳 0（est 在 bn=0 处连续：
                                       * R=1.317/G=1.102/B=0.731 黄核色）。
                                       * 旧版 continue 跳过会在黄核里留下
                                       * 未替换孔洞——bn 在 0 附近抖动时相邻
                                       * 像素有的替换有的不替换，输出跳变
                                       * mean 1851/65535，表现为核心孤立橙斑。 */
                if (bn > 1.70) bn = 1.70;   /* 覆盖拟合域上限（真实 max 1.64），不产生平台 */
                /* 羽化权重：[0.92,0.98) smoothstep 渐入，≥0.98 全替换 */
                double w9 = 1.0;
                if (mx < 0.98) {
                    double t = (mx - HL9_FEATHER_LO) / (0.98 - HL9_FEATHER_LO);
                    w9 = t * t * (3.0 - 2.0 * t);
                }
                for (c = 0; c < 3; c++) {
                    double est;
                    if (c == 2) {
                        /* B：两段膝点 smoothstep 混合 */
                        double t = (bn - (b_knee - b_blend_w * 0.5))
                                 / b_blend_w;
                        if (t < 0.0) t = 0.0;
                        if (t > 1.0) t = 1.0;
                        t = t * t * (3.0 - 2.0 * t);
                        double lo = b_lo_c + b_lo_a * pow(bn, b_lo_g);
                        double hi = b_hi_c + b_hi_a * pow(bn, b_hi_g);
                        est = (1.0 - t) * lo + t * hi;
                        /* ★ B 通道不重建（X3F_HL9_NO_B=1）：保留原始 lin。
                         * 依据：mode8（完全不重建）的 B bias +8.27 / MAE 11.11
                         * 优于 mode9 v3 的 −11.54 / 26.12 —— **B 重建整体有害**，
                         * 是用户看到的「粉红块」真因（不是缺绿）。 */
                        if (no_b) est = l[x*3+2];
                    } else {
                        est = params[c][2]
                            + params[c][1] * pow(bn, params[c][0]);
                    }
                    /* v6：重建值上限保护。R/B 的幂律在高 bn 处外推极猛
                     * （B 的 g=10 段在 bn=1.7 处 est≈622 = 白点 43 倍）。
                     * SPP 实测最高 G = 白点 4.62 倍，取 8.0 留足余量——
                     * 只兜外推爆掉，正常区间（bn p90=1.0）完全不受影响。 */
                    if (est > HL9_EST_MAX) est = HL9_EST_MAX;
                    o[x*3+c] = (float)(w9 * est + (1.0 - w9) * l[x*3+c]);
                }
            }
        }
    }
    /* bmu 原地转 bn（=box7(B)/wlin_B），留给 oe_spec_row 的空间修正用 */
    {
        const double bnorm2 = 1.0 / wlin[2];
        for (y = 0; y < h; y++) {
            float *d = bmu + (size_t)y * w;
            for (x = 0; x < w; x++) d[x] = (float)(d[x] * bnorm2);
        }
    }
    /* mG：重建前 G/wlin_G（第三维修正轴）。必须在写回 out 之前抓，
     * 因为 out 会覆盖裁剪区的 G。 */
    for (y = 0; y < h; y++) {
        const float *l = lin + (size_t)y * w * 3;
        float *d = mg + (size_t)y * w;
        for (x = 0; x < w; x++) d[x] = (float)(l[x*3+1] / wlin[1]);
    }
    g_hl9_bn = bmu;
    g_hl9_mask = msk;
    g_hl9_dens = dens;
    g_hl9_mg = mg;
    g_hl9_w = w; g_hl9_h = h;
    hl9_build_delta_fields(w, h);
    if (hl9_odm8()) {
        /* fields-only：特征场/od 场已建好挂到 g_hl9_*，mode8 链用原 lin。 */
        free(out);
        return NULL;
    }
    return out;
}

/* ---------- v7.2 Δ 场预计算 + 归一化卷积低通（去等高线） ----------
 *
 * 离线实测（cache2 重放）：Δ 场逐像素查表 → 三线性输出在掩膜内高频粗糙
 * （核心 |dx| 1.57 vs SPP 0.82），量化后连成用户可见的同心等高线。高频源
 * 不只 mG 逐像素噪声（只平滑 mG 无效：核心仅 1.47），还有 bn(0.05 步)/
 * dens(0.1 步) 的 bin 边界陡坡——必须对 Δ 场本身低通：
 *   nc5（归一化卷积）：MAE 4.84→4.85、死白 7.28→6.99%、梯度 2.90→2.72、
 *   核心 1.57→0.90（SPP 0.83）全线正向。
 * 注意 1）这与被否掉的"表值平滑"（跨格平均、伤总量）完全不同——空间低通
 * 不改修正总量，只磨掉 bin 边界陡坡与特征噪声。
 * 注意 2）用归一化卷积 box(Δ·m)/box(m) 而不是裸 box5：裸 box5 把掩膜外的
 * 0 平均进来，边界内侧 2px 的 Δ 被稀释，出现可见的暗带（黑边）；归一化
 * 卷积用掩膜内均值延拓填充，边界无暗带。
 *
 * 实现：hl9_restore 末尾预计算两阶段 Δ 场（int16 平面 [c*nn+o]，量化
 * ±0.5 s 单位无影响），oe_spec_row 直读。峰值临时内存 ~240MB（plane/tmp/
 * den 各 79MB），稳态两场共 ~235MB，随 g_hl9_* 一起释放。 */
#define HL9_DELTA_SMOOTH 5
static int16_t *g_hl9_d1 = NULL;   /* 阶段一 Δ 场（nc5 后），平面 [c*nn+o] */
static int16_t *g_hl9_d2 = NULL;   /* 阶段二 Δ 场（nc5 后） */
/* 输出域 Δ 场（X3F_HL9_ODOMAIN=1）：combined 之后的 16bit 修正，
 * 单位 = 16bit 级（表值 8bit 级 × 257），平面 [c*nn+o]。s 域改造见
 * hl9od_delta 表头注释：same bin 网格、标定无反查/无天花板。 */
static int16_t *g_hl9_od = NULL;

/* X3F_HL9_ODOMAIN=1：输出域 Δ 路线开关（默认关，逐位零回归）。 */
static int hl9_odomain(void)
{
    static int init = 0, v = 0;
    if (!init) {
        const char *e = getenv("X3F_HL9_ODOMAIN");
        v = (e != NULL && *e != '\0');
        init = 1;
    }
    return v;
}

/* X3F_HL9_NO_DELTA=1：跳过 Δ 场注入（s 域与输出域都跳，分离实验用）。 */
static int hl9_nodelta(void)
{
    static int init = 0, v = 0;
    if (!init) {
        const char *e = getenv("X3F_HL9_NO_DELTA");
        v = (e != NULL && *e != '\0');
        init = 1;
    }
    return v;
}

/* ---------- v8 掩膜外光晕 + 逐通道下限（2026-10-26，own 域重标）----------
 * 用户报：火焰外部与掩膜凹陷深色区仍有硬边。实锤（own 域同 build 口径）：
 * 高光重建外溢到裁剪区之外——负光晕剖面 外2-8px luma -3.0、外8-32px
 * -3.4、外32-128px -1.5（B 通道主导，掩膜凹陷深缝处 -20~-36），我方掩膜外
 * 严格 =0 → 与 差异在掩膜边界外形成可见台阶。
 * 建模：halo_c = LUT_c[bin(box511(mask))]，24 bin 绝对宽度 0.02764，
 * LUT 在 2501 掩膜外像素上按 SPP-m8(own) 修正量分桶均值（3 点平滑）。
 * 只加在掩膜外、comb 输出域。离线环带 MAE：外2-8 4.47→4.12、外8-32
 * 4.67→3.94、外32-128 3.39→3.14。
 * 同时：掩膜内逐通道下限 sf ≥ κ_c·s2（κ={0.70,0.55,0.02}，own 域目标/s2
 * p01 = R 0.701 / G 0.382 / B 0.015）。Δ 场在掩膜边界突然截止是边界台阶
 * 主因（+Δ表 p99 38→65），floor 把它救回（p99 40.3、p99.9 78.5，SPP
 * 8.9/21.2）；同时消崩塌（own 域新表崩塌 4352px→0）。
 * ★标定铁律：dump 必须 X3F_DECODER=own（缺省 fuse 域，lin 差 max 1.8、
 * mask 差 5 万 px）——v4 期忘设此变量把整套重标标到 fuse 域全部作废。
 * 实测 JPEG（2501，own）：死白 6.64%（v7.3 6.85 / SPP 6.56）、梯度 2.968
 * （v7.3 3.189）、外2-8px 5.09（v7.3 6.18）、外8-32px 5.37（5.97）。 */
#define HL9_FLOOR_R 0.70f
#define HL9_FLOOR_G 0.65f
#define HL9_FLOOR_B 0.02f
#define HL9_HALO_BINS 24
#define HL9_HALO_BIN_W 0.0276435f   /* own 域 2501 掩膜外 box511 最大值/24 */
static const float kHl9HaloLUT[3][HL9_HALO_BINS] = {
    { -3.3774f, -5.4559f, -6.3872f, -6.3296f, -6.1225f, -6.4877f, -7.1439f, -7.6541f, -8.4232f, -9.3569f, -9.4762f, -9.0982f, -10.0119f, -12.4196f, -13.8844f, -13.3464f, -12.4263f, -12.0865f, -11.6019f, -10.8734f, -8.1246f, -2.8247f, +0.0000f, -0.8443f },
    { +2.1669f, +0.3870f, +0.1618f, +0.7839f, +1.3112f, +1.6917f, +1.4841f, +1.7367f, +3.8684f, +6.0806f, +7.5258f, +9.1581f, +10.9371f, +12.6402f, +14.2867f, +14.8515f, +14.1079f, +13.7804f, +13.8604f, +12.7829f, +8.3795f, +2.4871f, +0.0000f, +0.5417f },
    { -0.9196f, -3.4641f, -4.6643f, -4.7805f, -4.5850f, -4.3109f, -4.4539f, -4.6990f, -4.6363f, -4.8187f, -4.7903f, -4.7204f, -5.4161f, -6.3066f, -7.3473f, -8.7125f, -10.0794f, -11.4116f, -13.6663f, -17.3609f, -14.9968f, -5.2665f, +0.0000f, -0.2299f }
};
static uint8_t *g_hl9_hbin = NULL /*TEST*/; /* 掩膜外光晕 bin 图（box511(mask) 分桶） */

/* 大窗 box（O(1)/像素前缀和，edge 填充与 hl9_boxk/离线 box2d 同语义）。
 * K 大时 hl9_boxk 的 O(K) 逐像素直算不可用（511 窗 ≈ 200 亿次加法）。 */
static void hl9_box_big(const float *src, float *dst, float *tmp, int w, int h, int K)
{
    const int R = K / 2;
    int x, y;
    float *csum = (float *)malloc(((w > h ? w : h) + 1) * sizeof(float));
    if (csum == NULL) return;
    for (y = 0; y < h; y++) {
        const float *s = src + (size_t)y * w;
        float *d = tmp + (size_t)y * w;
        csum[0] = 0.0f;
        for (x = 0; x < w; x++) csum[x + 1] = csum[x] + s[x];
        for (x = 0; x < w; x++) {
            int lo = x - R; if (lo < 0) lo = 0;
            int hi = x + R; if (hi > w - 1) hi = w - 1;
            d[x] = (csum[hi + 1] - csum[lo]) / (float)K;
        }
    }
    for (x = 0; x < w; x++) {
        csum[0] = 0.0f;
        for (y = 0; y < h; y++) csum[y + 1] = csum[y] + tmp[(size_t)y * w + x];
        for (y = 0; y < h; y++) {
            int lo = y - R; if (lo < 0) lo = 0;
            int hi = y + R; if (hi > h - 1) hi = h - 1;
            dst[(size_t)y * w + x] = (csum[hi + 1] - csum[lo]) / (float)K;
        }
    }
    free(csum);
}

static void hl9_free_delta_fields(void)
{
    free(g_hl9_d1); free(g_hl9_d2); free(g_hl9_hbin); free(g_hl9_od);
    g_hl9_d1 = NULL; g_hl9_d2 = NULL; g_hl9_hbin = NULL; g_hl9_od = NULL;
}

static void hl9_build_delta_fields(int w, int h)
{
    const size_t nn = (size_t)w * h;
    int16_t *f1 = (int16_t *)malloc(nn * 3 * sizeof(int16_t));
    int16_t *f2 = (int16_t *)malloc(nn * 3 * sizeof(int16_t));
    float *plane = (float *)malloc(nn * sizeof(float));
    float *tmp = (float *)malloc(nn * sizeof(float));
    float *den = (float *)malloc(nn * sizeof(float));
    if (f1 == NULL || f2 == NULL || plane == NULL || tmp == NULL || den == NULL) {
        free(f1); free(f2); free(plane); free(tmp); free(den);
        return;   /* 失败则 oe_spec_row 回落 v7 逐像素查表路径 */
    }
    /* 归一化卷积的分母 = mask 的 box5（掩膜内延拓权场），算一次 */
    for (size_t o = 0; o < nn; o++) den[o] = g_hl9_mask[o] ? 1.0f : 0.0f;
    hl9_boxk(den, den, tmp, w, h, HL9_DELTA_SMOOTH);
    /* 表插值参数与 oe_spec_row 的旧内联代码一致：pb=bn/step（先钳），
     * pd=dens/0.1，pg=(mG-0.25)/0.125；掩膜外 Δ=0。 */
    for (int c = 0; c < 3; c++) {
        /* 阶段一 → plane → box5 → f1[c] */
        /* ★ v8 表选择：X3F_HL9_DELTA_V8=1 → 用按 8bit 显示域判据重标定的表。
         * 老表（2026-10-26）是按**线性域**标的，在 8bit 显示域把 B 压低 −95
         * （近饱和带 [192,224)），是用户看到的焰心偏色来源。
         * 默认不启用 → 逐位零回归。 */
        static int d8_init = 0;
        static int use_d8 = 0;
        if (!d8_init) { use_d8 = (getenv("X3F_HL9_DELTA_V8") != NULL); d8_init = 1; }
        for (int y = 0; y < h; y++) {
            const uint8_t *m = g_hl9_mask + (size_t)y * w;
            const float *bv = g_hl9_bn + (size_t)y * w;
            const float *dv = g_hl9_dens + (size_t)y * w;
            const float *gv = g_hl9_mg + (size_t)y * w;
            float *pl = plane + (size_t)y * w;
            for (int x = 0; x < w; x++) {
                if (!m[x]) { pl[x] = 0.0f; continue; }
                float pb = bv[x] / (float)HL9C_BN_STEP;
                float pd = dv[x] / (float)HL9C_DN_STEP;
                float pg = (gv[x] - (float)HL9C_G_LO) / (float)HL9C_G_STEP;
                if (pb < 0) pb = 0; if (pb > HL9C_BN_N - 1) pb = HL9C_BN_N - 1;
                if (pd < 0) pd = 0; if (pd > HL9C_DN_N - 1) pd = HL9C_DN_N - 1;
                if (pg < 0) pg = 0; if (pg > HL9C_GN_N - 1) pg = HL9C_GN_N - 1;
                const int i0b = (int)pb, i0d = (int)pd, i0g = (int)pg;
                const int i1b = (i0b + 1 < HL9C_BN_N) ? i0b + 1 : i0b;
                const int i1d = (i0d + 1 < HL9C_DN_N) ? i0d + 1 : i0d;
                const int i1g = (i0g + 1 < HL9C_GN_N) ? i0g + 1 : i0g;
                const float fb = pb - (float)i0b, fd = pd - (float)i0d, fg = pg - (float)i0g;
                float acc = 0.0f;
                for (int tg = 0; tg < 2; tg++)
                    for (int td = 0; td < 2; td++)
                        for (int tb = 0; tb < 2; tb++)
                            acc += (use_d8 ? hl9c_delta_v8[tb ? i1b : i0b][td ? i1d : i0d][tg ? i1g : i0g][c]
                                           : hl9c_delta[tb ? i1b : i0b][td ? i1d : i0d][tg ? i1g : i0g][c])
                                 * (tb ? fb : 1.0f - fb) * (td ? fd : 1.0f - fd) * (tg ? fg : 1.0f - fg);
                pl[x] = acc;
            }
        }
        hl9_boxk(plane, plane, tmp, w, h, HL9_DELTA_SMOOTH);
        for (size_t o = 0; o < nn; o++) {
            float v = g_hl9_mask[o] ? plane[o] / (den[o] < 1e-6f ? 1.0f : den[o]) : 0.0f;
            f1[c * nn + o] = (int16_t)(v >= 0 ? v + 0.5f : v - 0.5f);
        }
        /* 阶段二 → f2[c]（表不同、特征同，重查一遍，省一块 79MB 平面） */
        for (int y = 0; y < h; y++) {
            const uint8_t *m = g_hl9_mask + (size_t)y * w;
            const float *bv = g_hl9_bn + (size_t)y * w;
            const float *dv = g_hl9_dens + (size_t)y * w;
            const float *gv = g_hl9_mg + (size_t)y * w;
            float *pl = plane + (size_t)y * w;
            for (int x = 0; x < w; x++) {
                if (!m[x]) { pl[x] = 0.0f; continue; }
                float p2b = bv[x] / (float)HL9C2_BN_STEP;
                float p2d = dv[x] / (float)HL9C2_DN_STEP;
                float p2g = (gv[x] - (float)HL9C2_G_LO) / (float)HL9C2_G_STEP;
                if (p2b < 0) p2b = 0; if (p2b > HL9C2_BN_N - 1) p2b = HL9C2_BN_N - 1;
                if (p2d < 0) p2d = 0; if (p2d > HL9C2_DN_N - 1) p2d = HL9C2_DN_N - 1;
                if (p2g < 0) p2g = 0; if (p2g > HL9C2_GN_N - 1) p2g = HL9C2_GN_N - 1;
                const int j0b = (int)p2b, j0d = (int)p2d, j0g = (int)p2g;
                const int j1b = (j0b + 1 < HL9C2_BN_N) ? j0b + 1 : j0b;
                const int j1d = (j0d + 1 < HL9C2_DN_N) ? j0d + 1 : j0d;
                const int j1g = (j0g + 1 < HL9C2_GN_N) ? j0g + 1 : j0g;
                const float qb = p2b - (float)j0b, qd = p2d - (float)j0d, qg = p2g - (float)j0g;
                float acc = 0.0f;
                for (int ug = 0; ug < 2; ug++)
                    for (int ud = 0; ud < 2; ud++)
                        for (int ub = 0; ub < 2; ub++)
                            acc += (use_d8 ? hl9c2_delta_v8[ub ? j1b : j0b][ud ? j1d : j0d][ug ? j1g : j0g][c]
                                           : hl9c2_delta[ub ? j1b : j0b][ud ? j1d : j0d][ug ? j1g : j0g][c])
                                 * (ub ? qb : 1.0f - qb) * (ud ? qd : 1.0f - qd) * (ug ? qg : 1.0f - qg);
                pl[x] = acc;
            }
        }
        hl9_boxk(plane, plane, tmp, w, h, HL9_DELTA_SMOOTH);
        for (size_t o = 0; o < nn; o++) {
            float v = g_hl9_mask[o] ? plane[o] / (den[o] < 1e-6f ? 1.0f : den[o]) : 0.0f;
            f2[c * nn + o] = (int16_t)(v >= 0 ? v + 0.5f : v - 0.5f);
        }
    }
    /* v8 光晕 bin 图：box511(mask) → 24 bin（绝对宽度 HL9_HALO_BIN_W）。
     * 只在掩膜外用；掩膜内存 0 省一次查表判断。 */
    {
        uint8_t *hb = (uint8_t *)malloc(nn);
        if (hb != NULL) {
            for (size_t o = 0; o < nn; o++) plane[o] = g_hl9_mask[o] ? 1.0f : 0.0f;
            hl9_box_big(plane, plane, tmp, w, h, 511);
            for (size_t o = 0; o < nn; o++) {
                if (g_hl9_mask[o]) { hb[o] = 0; continue; }
                int b = (int)(plane[o] / (float)HL9_HALO_BIN_W);
                if (b < 0) b = 0; if (b > HL9_HALO_BINS - 1) b = HL9_HALO_BINS - 1;
                hb[o] = (uint8_t)b;
            }
            g_hl9_hbin = hb;
        }
    }
    /* [TEMP-DEBUG] X3F_OE9_HBIN_DUMP=路径 倒出光晕 bin 图（对质用） */
    {
        const char *ehb = getenv("X3F_OE9_HBIN_DUMP");
        if (ehb != NULL && *ehb != '\0' && g_hl9_hbin != NULL) {
            FILE *fhb = fopen(ehb, "wb");
            if (fhb != NULL) {
                fwrite(g_hl9_hbin, 1, nn, fhb);
                fclose(fhb);
                fprintf(stderr, "[oe9] dump hbin -> %s\n", ehb);
            }
        }
    }
    free(plane); free(tmp);   /* den 保留给下方输出域 Δ 场（od）复用 */
    g_hl9_d1 = f1;
    g_hl9_d2 = f2;
    /* 输出域 Δ 场（X3F_HL9_ODOMAIN=1 才构建）：hl9od_delta 表（bin 网格与
     * HL9C_* 相同：bn 0.1 步 / dens 0.1 步 / mG 0.25+0.125 步），同样的
     * 三线性 + nc5 归一化卷积；量化到 16bit 级（8bit 表值 × 257）。 */
    if (hl9_odomain() || hl9_odm8()) {
        int16_t *fo = (int16_t *)malloc(nn * 3 * sizeof(int16_t));
        float *pl2 = (float *)malloc(nn * sizeof(float));
        float *tm2 = (float *)malloc(nn * sizeof(float));
        if (fo == NULL || pl2 == NULL || tm2 == NULL) {
            free(fo); free(pl2); free(tm2);
            free(den);
            return;   /* fo 保持 NULL → oe_spec_row 走原 s 域路径 */
        }
        for (int c = 0; c < 3; c++) {
            for (int y = 0; y < h; y++) {
                const uint8_t *m = g_hl9_mask + (size_t)y * w;
                const float *bv = g_hl9_bn + (size_t)y * w;
                const float *dv = g_hl9_dens + (size_t)y * w;
                const float *gv = g_hl9_mg + (size_t)y * w;
                float *pl = pl2 + (size_t)y * w;
                for (int x = 0; x < w; x++) {
                    if (!m[x]) { pl[x] = 0.0f; continue; }
                    float pb = bv[x] / (float)HL9C_BN_STEP;
                    float pd = dv[x] / (float)HL9C_DN_STEP;
                    float pg = (gv[x] - (float)HL9C_G_LO) / (float)HL9C_G_STEP;
                    if (pb < 0) pb = 0; if (pb > HL9C_BN_N - 1) pb = HL9C_BN_N - 1;
                    if (pd < 0) pd = 0; if (pd > HL9C_DN_N - 1) pd = HL9C_DN_N - 1;
                    if (pg < 0) pg = 0; if (pg > HL9C_GN_N - 1) pg = HL9C_GN_N - 1;
                    const int i0b = (int)pb, i0d = (int)pd, i0g = (int)pg;
                    const int i1b = (i0b + 1 < HL9OD_BN_N) ? i0b + 1 : i0b;
                    const int i1d = (i0d + 1 < HL9OD_DN_N) ? i0d + 1 : i0d;
                    const int i1g = (i0g + 1 < HL9OD_GN_N) ? i0g + 1 : i0g;
                    const float fb = pb - (float)i0b, fd = pd - (float)i0d, fg = pg - (float)i0g;
                    /* 表选择：ODM8 用 mode8 基线表，其余用 mode9 基线表 */
                    const float (*odtab)[HL9OD_DN_N][HL9OD_GN_N][3]
                        = hl9_odm8() ? hl9od_m8_delta : hl9od_delta;
                    float acc = 0.0f;
                    for (int tg = 0; tg < 2; tg++)
                        for (int td = 0; td < 2; td++)
                            for (int tb = 0; tb < 2; tb++)
                                acc += odtab[tb ? i1b : i0b][td ? i1d : i0d][tg ? i1g : i0g][c]
                                     * (tb ? fb : 1.0f - fb) * (td ? fd : 1.0f - fd) * (tg ? fg : 1.0f - fg);
                    pl[x] = acc;
                }
            }
            hl9_boxk(pl2, pl2, tm2, w, h, HL9_DELTA_SMOOTH);
            for (size_t o = 0; o < nn; o++) {
                float v = g_hl9_mask[o] ? pl2[o] / (den[o] < 1e-6f ? 1.0f : den[o]) : 0.0f;
                v *= 257.0f;   /* 8bit 级 → 16bit 级 */
                fo[c * nn + o] = (int16_t)(v >= 0 ? v + 0.5f : v - 0.5f);
            }
        }
        free(pl2); free(tm2);
        g_hl9_od = fo;
    }
    free(den);
}

/* 残差 3D LUT 三线性插值（u 各分量已 clamp 到 [0,1]）。
 * index = (ix*G + iy)*G + iz，ix = u_R 轴（与拟合脚本一致）。 */
static void oe_spec_res_trilin(float u0, float u1, float u2, float r[3])
{
    const int G = g_oespec->res_g;
    float g0 = u0 * (G - 1), g1 = u1 * (G - 1), g2 = u2 * (G - 1);
    int i0 = (int)g0, i1 = (int)g1, i2 = (int)g2;
    float f0 = g0 - i0, f1 = g1 - i1, f2 = g2 - i2;
    if (i0 > G - 2) { i0 = G - 2; f0 = 1.0f; } if (i0 < 0) { i0 = 0; f0 = 0.0f; }
    if (i1 > G - 2) { i1 = G - 2; f1 = 1.0f; } if (i1 < 0) { i1 = 0; f1 = 0.0f; }
    if (i2 > G - 2) { i2 = G - 2; f2 = 1.0f; } if (i2 < 0) { i2 = 0; f2 = 0.0f; }
    float acc[3] = {0, 0, 0};
    int dx, dy, dz;
    for (dx = 0; dx <= 1; dx++) {
        float wx = dx ? f0 : 1.0f - f0;
        for (dy = 0; dy <= 1; dy++) {
            float wy = dy ? f1 : 1.0f - f1;
            for (dz = 0; dz <= 1; dz++) {
                float wz = dz ? f2 : 1.0f - f2;
                int ix = i0 + dx, iy = i1 + dy, iz = i2 + dz;
                const int16_t *e = g_oespec->res[(ix * G + iy) * G + iz];
                float wgt = wx * wy * wz;
                acc[0] += e[0] * wgt;
                acc[1] += e[1] * wgt;
                acc[2] += e[2] * wgt;
            }
        }
    }
    r[0] = acc[0]; r[1] = acc[1]; r[2] = acc[2];
}

static void oe_spec_row(void *vctx, size_t row)
{
    const oe_spec_ctx_t *c = (const oe_spec_ctx_t *)vctx;
    const float *src = c->lin + row * (size_t)c->w * 3;
    /* [bug fix 2026-11-01，分支 m8-darkrecal] 此处原来是 `const float *fillg =
     * c->fillgain` + 行内 `fillg[x]` —— 丢了 row 偏移，每行都在读 fill 增益图
     * 的第 0 行（典型=天空，近常数）→ mode8 的 fill 退化为全图标量增益，
     * 2D Retinex map 整张被丢弃。默认链 apply_row 一直是
     * fillgain[row*w+x]（2526 行）。后果：① mode8 fill 空间分布全错（暗部
     * 提不上来）；② 墙区工作点被压在残差 LUT 野值首格里，列带被非线性放大
     * 成竖条纹（DP2Q4019 的可见条纹主源）。 */
    const float *fillg = c->fillgain;
    uint8_t *dst8 = (c->out != NULL)
        ? (uint8_t *)c->out + row * (size_t)c->stride : NULL;
    float *disp = (c->disp != NULL)
        ? c->disp + row * (size_t)c->w * 3 : NULL;
    const int inline_post = (c->contrast != 0.0) || (c->hl_recover > 0.0)
                         || (c->cmtone_row >= 0);
    int x;
    for (x = 0; x < c->w; x++) {
        double v0 = src[0], v1 = src[1], v2 = src[2];
        if (fillg != NULL) {
            const float fg = fillg[(size_t)row * (size_t)c->w + (size_t)x];
            v0 *= fg; v1 *= fg; v2 *= fg;
        }
        if (c->gain != NULL) {
            v0 *= c->gain[0]; v1 *= c->gain[1]; v2 *= c->gain[2];
        }
        if (c->cm != NULL) {
            double a = v0, b = v1, e = v2;
            v0 = a * c->cm[0] + b * c->cm[1] + e * c->cm[2];
            v1 = a * c->cm[3] + b * c->cm[4] + e * c->cm[5];
            v2 = a * c->cm[6] + b * c->cm[7] + e * c->cm[8];
            {
                static int cm_method = -1;
                if (cm_method < 0) {
                    const char *m = getenv("X3F_CM_METHOD");
                    cm_method = (m != NULL && m[0] == '0') ? 0 : 1;
                }
                if (cm_method == 1) {
                    /* 与 apply_row 同款：亮度失真门控，仅离群矩阵（FCBlue）施加
                     * 保亮 + gamut 压缩；温和档跳过 → 零回归。 */
                    double g0 = 0.2126*c->cm[0] + 0.7152*c->cm[3] + 0.0722*c->cm[6];
                    double g1 = 0.2126*c->cm[1] + 0.7152*c->cm[4] + 0.0722*c->cm[7];
                    double g2 = 0.2126*c->cm[2] + 0.7152*c->cm[5] + 0.0722*c->cm[8];
                    double ld = fmax(fabs(g0-0.2126),
                                     fmax(fabs(g1-0.7152), fabs(g2-0.0722)));
                    if (ld > 0.5) {
                        double yin = 0.2126*a + 0.7152*b + 0.0722*e;
                        double yout = 0.2126*v0 + 0.7152*v1 + 0.0722*v2;
                        if (yout > 1e-6) {
                            double k = yin / yout;
                            v0 *= k; v1 *= k; v2 *= k;
                        }
                        cm_gamut_compress(&v0, &v1, &v2);
                    }
                }
            }
            if (v0 < 0) v0 = 0;
            if (v1 < 0) v1 = 0;
            if (v2 < 0) v2 = 0;
        }
        const double (*oeM)[3] = g_oespec->M;
        double w0 = oeM[0][0] * v0 + oeM[0][1] * v1 + oeM[0][2] * v2 + oeM[3][0];
        double w1 = oeM[1][0] * v0 + oeM[1][1] * v1 + oeM[1][2] * v2 + oeM[3][1];
        double w2 = oeM[2][0] * v0 + oeM[2][1] * v1 + oeM[2][2] * v2 + oeM[3][2];
        float s0 = oe_spec_interp1(g_oespec->tcX[0], g_oespec->tcY[0], g_oespec->tcN[0], (float)w0);
        float s1 = oe_spec_interp1(g_oespec->tcX[1], g_oespec->tcY[1], g_oespec->tcN[1], (float)w1);
        float s2 = oe_spec_interp1(g_oespec->tcX[2], g_oespec->tcY[2], g_oespec->tcN[2], (float)w2);
        const float d2f_s0 = s0, d2f_s1 = s1, d2f_s2 = s2;  /* 拟合 dump：Tc 输出基线 */
        /* [dark2 槽位 2026-11-01，分支 m8-darkrecal] mode8 Tc 暗段重标定 Δ：
         * s_i += interp(kOeSpecDark2Wx, kOeSpecDark2Dy[i], w_i)。节点端点 Δ=0
         * = 与现有表平滑衔接，w>60（中高光）逐位不变；表全 0 时开/关逐位同。
         * 默认关（X3F_OE_DARK2 非空且非 '0' 才开）；X3F_OE_DARK2_GAIN 调整体
         * 混合比例（默认 1）。拟合管线见 outputs/tools/lab/fit_dark2.py。 */
        {
            static int d2i = 0;
            static int d2_on = 0;
            static float d2_gain = 1.0f;
            if (!d2i) {
                const char *e = getenv("X3F_OE_DARK2");
                if (e != NULL && e[0] != '\0' && e[0] != '0') d2_on = 1;
                const char *g = getenv("X3F_OE_DARK2_GAIN");
                if (g != NULL && g[0] != '\0') d2_gain = (float)atof(g);
                d2i = 1;
            }
            if (d2_on) {
                s0 += d2_gain * oe_spec_interp1(kOeSpecDark2Wx, kOeSpecDark2Dy[0], DARK2_NN, (float)w0);
                s1 += d2_gain * oe_spec_interp1(kOeSpecDark2Wx, kOeSpecDark2Dy[1], DARK2_NN, (float)w1);
                s2 += d2_gain * oe_spec_interp1(kOeSpecDark2Wx, kOeSpecDark2Dy[2], DARK2_NN, (float)w2);
                if (s0 < 0) s0 = 0;
                if (s1 < 0) s1 = 0;
                if (s2 < 0) s2 = 0;
            }
        }
        /* 残差 3D LUT：u = clamp(src/res_scale,0,1)（own 31744 / fuse 16384），
         * 修正 Stage2 的色相/局部项残差。
         * 空格 = 0，域外自动退化回纯 T_c 基线（= 旧 mode8 行为）。 */
        {
            const float rs = 1.0f / g_oespec->res_scale;
            float u0 = s0 * rs, u1 = s1 * rs, u2 = s2 * rs;
            if (u0 < 0) u0 = 0; if (u0 > 1) u0 = 1;
            if (u1 < 0) u1 = 0; if (u1 > 1) u1 = 1;
            if (u2 < 0) u2 = 0; if (u2 > 1) u2 = 1;
            float r[3];
            oe_spec_res_trilin(u0, u1, u2, r);
            /* [暗端渐入 2026-11-01] 残差 LUT 是单场景（DP2Q2501）拟合，暗角
             * (u≈0) 无数据支撑：cnt<8 空格与野值格（G 通道 −643..+494）交错，
             * 三线性跨格跳变把暗部 s 打 0 / 巨幅调制 —— DP2Q4019 暗墙竖条纹
             * 的注入源（暗墙 s≈20..100 全落第一格距，r1≈−30 vs s1 同量级）。
             * u 域低端把残差渐入为 0 = 退化回纯 T_c 基线（架构本意「空格=0
             * 优雅退化」的连续版）；中高光（u≥hi）逐位不变。
             * 默认开，X3F_RES_DARKGATE=0 关；X3F_RES_DARK_LO/HI 调窗。 */
            {
                static int dgi = 0;
                static int dg_on = 1;
                static float dg_lo = 0.002f, dg_hi = 0.012f;
                if (!dgi) {
                    const char *e = getenv("X3F_RES_DARKGATE");
                    if (e != NULL && e[0] == '0') dg_on = 0;
                    const char *l = getenv("X3F_RES_DARK_LO");
                    const char *h = getenv("X3F_RES_DARK_HI");
                    if (l != NULL && l[0] != '\0') dg_lo = (float)atof(l);
                    if (h != NULL && h[0] != '\0') dg_hi = (float)atof(h);
                    dgi = 1;
                }
                if (dg_on) {
                    float um = u0;
                    if (u1 > um) um = u1;
                    if (u2 > um) um = u2;
                    float w = (um - dg_lo) / (dg_hi - dg_lo);
                    if (w < 0) w = 0;
                    if (w > 1) w = 1;
                    w = w * w * (3.0f - 2.0f * w);   /* smoothstep */
                    r[0] *= w; r[1] *= w; r[2] *= w;
                }
            }
            s0 += r[0]; s1 += r[1]; s2 += r[2];
            if (s0 < 0) s0 = 0; if (s1 < 0) s1 = 0; if (s2 < 0) s2 = 0;
        }
        /* dark2 拟合采集（::2）：s_pre = d2f_*（Tc 基线），s_post = 当前 s */
        if (g_dark2_fit != NULL && (row & 1) == 0 && (x & 1) == 0) {
            float *dp = g_dark2_fit + ((size_t)(row >> 1) * (size_t)g_dark2_fw + (size_t)(x >> 1)) * 9;
            dp[0] = (float)w0; dp[1] = (float)w1; dp[2] = (float)w2;
            dp[3] = d2f_s0;    dp[4] = d2f_s1;    dp[5] = d2f_s2;
            dp[6] = s0;        dp[7] = s1;        dp[8] = s2;
        }
        /*mode9 空间修正：Δs(bn, density, mG)——点变换链表达不了 空间
         * 自适应高光展开（同 w_B 下环形/深核/光晕目标非单调，反解实锤），
         * 用重建阶段算好的 bn、bloom 深度代理、未裁 G 在输出端逐像素补。
         * v7.1：Δ 场在 hl9_restore 末尾预计算并 box5 空间低通（去等高线），
         * 这里直接读场；场构建失败（OOM）时回落旧逐像素查表路径。 */
        if (g_oespec == &oe_spec_tabs_m9 && g_hl9_bn != NULL && g_hl9_mg != NULL) {
            const size_t o9 = (size_t)row * (size_t)c->w + (size_t)x;
            /* ★ X3F_HL9_NO_DELTA=1：跳过 Δ 场注入（保留重建层）。
             * 用途（2026-10-28 第 2 会话）：分离「重建层」与「Δ 场」各自的
             * 净贡献 —— 实测 mode8（两者都无）全 MAE 4.65，mode9 全有 10.57，
             * 需要知道是重建层还是 Δ 场在拖后腿。 */
            static const char *nd_env = NULL;
            static int nd_init = 0;
            if (!nd_init) { nd_env = getenv("X3F_HL9_NO_DELTA"); nd_init = 1; }
            if (nd_env != NULL && *nd_env != '\0') {
                /* 不注入 Δ 场，直接进 s dump */
                if (g_hl9_sdump != NULL) {
                    float *sp = g_hl9_sdump + ((size_t)row * (size_t)c->w + (size_t)x) * 3;
                    sp[0] = s0; sp[1] = s1; sp[2] = s2;
                }
            } else if (hl9_odomain()) {
                /* 输出域 Δ 路线（X3F_HL9_ODOMAIN=1）：s 域不注入，Δ 在
                 * combined 之后加（见下方 g_hl9_od）。s dump = 无 Δ 基线。 */
                if (g_hl9_sdump != NULL) {
                    float *sp = g_hl9_sdump + ((size_t)row * (size_t)c->w + (size_t)x) * 3;
                    sp[0] = s0; sp[1] = s1; sp[2] = s2;
                }
            } else if (g_hl9_d1 != NULL && g_hl9_d2 != NULL) {
                const size_t nn9 = (size_t)g_hl9_w * (size_t)g_hl9_h;
                /* v8 逐通道下限基准 = Δ 注入前的模型 s（崩塌防护，见
                 * kHl9HaloLUT 注释）：低 bn/低 mG bin 被暗缝人群主导，
                 * 无下限时边界羽化带像素 s 被推到 0 → 橙描边。 */
                const float f0 = HL9_FLOOR_R * s0;
                const float f1 = HL9_FLOOR_G * s1;
                const float f2 = HL9_FLOOR_B * s2;
                s0 += (float)g_hl9_d1[o9];
                s1 += (float)g_hl9_d1[nn9 + o9];
                s2 += (float)g_hl9_d1[2 * nn9 + o9];
                if (s0 < 0) s0 = 0; if (s1 < 0) s1 = 0; if (s2 < 0) s2 = 0;
                s0 += (float)g_hl9_d2[o9];
                s1 += (float)g_hl9_d2[nn9 + o9];
                s2 += (float)g_hl9_d2[2 * nn9 + o9];
                if (s0 < 0) s0 = 0; if (s1 < 0) s1 = 0; if (s2 < 0) s2 = 0;
                if (s0 < f0) s0 = f0;
                if (s1 < f1) s1 = f1;
                if (s2 < f2) s2 = f2;
                if (g_hl9_sdump != NULL) {
                    float *sp = g_hl9_sdump + ((size_t)row * (size_t)c->w + (size_t)x) * 3;
                    sp[0] = s0; sp[1] = s1; sp[2] = s2;
                }
            } else if (g_hl9_mask[o9]) {
                float pb = g_hl9_bn[o9] / (float)HL9C_BN_STEP;
                float pd = g_hl9_dens[o9] / (float)HL9C_DN_STEP;
                float pg = (g_hl9_mg[o9] - (float)HL9C_G_LO) / (float)HL9C_G_STEP;
                if (pb < 0) pb = 0;
                if (pb > HL9C_BN_N - 1) pb = HL9C_BN_N - 1;
                if (pd < 0) pd = 0;
                if (pd > HL9C_DN_N - 1) pd = HL9C_DN_N - 1;
                if (pg < 0) pg = 0;
                if (pg > HL9C_GN_N - 1) pg = HL9C_GN_N - 1;
                {
                    const int i0b = (int)pb, i0d = (int)pd, i0g = (int)pg;
                    const int i1b = (i0b + 1 < HL9C_BN_N) ? i0b + 1 : i0b;
                    const int i1d = (i0d + 1 < HL9C_DN_N) ? i0d + 1 : i0d;
                    const int i1g = (i0g + 1 < HL9C_GN_N) ? i0g + 1 : i0g;
                    const float fb = pb - (float)i0b;
                    const float fd = pd - (float)i0d;
                    const float fg = pg - (float)i0g;
                    float a0 = 0.0f, a1 = 0.0f, a2 = 0.0f;
                    int tb, td, tg;
                    for (tg = 0; tg < 2; tg++) {
                        const int jg = tg ? i1g : i0g;
                        const float wg = tg ? fg : (1.0f - fg);
                        for (td = 0; td < 2; td++) {
                            const int jd = td ? i1d : i0d;
                            const float wdg = (td ? fd : (1.0f - fd)) * wg;
                            for (tb = 0; tb < 2; tb++) {
                                const int jb = tb ? i1b : i0b;
                                const float w = (tb ? fb : (1.0f - fb)) * wdg;
                                a0 += hl9c_delta[jb][jd][jg][0] * w;
                                a1 += hl9c_delta[jb][jd][jg][1] * w;
                                a2 += hl9c_delta[jb][jd][jg][2] * w;
                            }
                        }
                    }
                    s0 += a0; s1 += a1; s2 += a2;
                    if (s0 < 0) s0 = 0; if (s1 < 0) s1 = 0; if (s2 < 0) s2 = 0;
                    /* 阶段二：残差表（bn 步长 0.05，标定目标 = 阶段一注入并
                     * 钳位后的残差），见 x3f_oe_hl9corr2.h。两阶段离线
                     * 掩膜 MAE 5.38→4.98、火核死白 7.47→7.25%；棋盘留出
                     * train/test 均 5.03，非过拟合。 */
                    {
                        float p2b = g_hl9_bn[o9] / (float)HL9C2_BN_STEP;
                        float p2d = g_hl9_dens[o9] / (float)HL9C2_DN_STEP;
                        float p2g = (g_hl9_mg[o9] - (float)HL9C2_G_LO) / (float)HL9C2_G_STEP;
                        if (p2b < 0) p2b = 0;
                        if (p2b > HL9C2_BN_N - 1) p2b = HL9C2_BN_N - 1;
                        if (p2d < 0) p2d = 0;
                        if (p2d > HL9C2_DN_N - 1) p2d = HL9C2_DN_N - 1;
                        if (p2g < 0) p2g = 0;
                        if (p2g > HL9C2_GN_N - 1) p2g = HL9C2_GN_N - 1;
                        {
                            const int j0b = (int)p2b, j0d = (int)p2d, j0g = (int)p2g;
                            const int j1b = (j0b + 1 < HL9C2_BN_N) ? j0b + 1 : j0b;
                            const int j1d = (j0d + 1 < HL9C2_DN_N) ? j0d + 1 : j0d;
                            const int j1g = (j0g + 1 < HL9C2_GN_N) ? j0g + 1 : j0g;
                            const float qb = p2b - (float)j0b;
                            const float qd = p2d - (float)j0d;
                            const float qg = p2g - (float)j0g;
                            float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;
                            int ub, ud, ug;
                            for (ug = 0; ug < 2; ug++) {
                                const int kg = ug ? j1g : j0g;
                                const float wgg = ug ? qg : (1.0f - qg);
                                for (ud = 0; ud < 2; ud++) {
                                    const int kd = ud ? j1d : j0d;
                                    const float wdd = (ud ? qd : (1.0f - qd)) * wgg;
                                    for (ub = 0; ub < 2; ub++) {
                                        const int kb = ub ? j1b : j0b;
                                        const float w = (ub ? qb : (1.0f - qb)) * wdd;
                                        b0 += hl9c2_delta[kb][kd][kg][0] * w;
                                        b1 += hl9c2_delta[kb][kd][kg][1] * w;
                                        b2 += hl9c2_delta[kb][kd][kg][2] * w;
                                    }
                                }
                            }
                            s0 += b0; s1 += b1; s2 += b2;
                            if (s0 < 0) s0 = 0; if (s1 < 0) s1 = 0; if (s2 < 0) s2 = 0;
                        }
                    }
                    if (g_hl9_sdump != NULL) {
                        float *sp = g_hl9_sdump + ((size_t)row * (size_t)c->w + (size_t)x) * 3;
                        sp[0] = s0; sp[1] = s1; sp[2] = s2;
                    }
                }
            }
        }
        float o0 = oe_spec_interp1(kOeSpecCombX, kOeSpecCombY, OE_SPEC_COMB_N, s0);
        float o1 = oe_spec_interp1(kOeSpecCombX, kOeSpecCombY, OE_SPEC_COMB_N, s1);
        float o2 = oe_spec_interp1(kOeSpecCombX, kOeSpecCombY, OE_SPEC_COMB_N, s2);
        if ((g_oespec == &oe_spec_tabs_m9 || hl9_odm8()) && g_hl9_od != NULL && !hl9_nodelta()
            && (size_t)c->w == (size_t)g_hl9_w && row < (size_t)g_hl9_h) {
            /* 尺寸护栏（2026-10-30）：od 场是解码期全尺寸（lin active）产物，
             * 预览/裁剪等小于解码尺寸的渲染路索引会对错位——直接跳过注入
             * （spec 曲线与分辨率无关，mode8 基线不受影响）。 */
            /* 输出域 Δ（X3F_HL9_ODOMAIN=1）：combined 之后的 16bit 修正场。
             * 在 dither/halo 之前加——halo 只作用于掩膜外（od 场掩膜外为 0），
             * 两者互不干扰。 */
            const size_t nn9 = (size_t)g_hl9_w * (size_t)g_hl9_h;
            const size_t o9 = (size_t)row * (size_t)c->w + (size_t)x;
            o0 += (float)g_hl9_od[o9];
            o1 += (float)g_hl9_od[nn9 + o9];
            o2 += (float)g_hl9_od[2 * nn9 + o9];
        }
        if (g_oespec == &oe_spec_tabs_m9 && g_hl9_bn != NULL) {
            /* v7.1 去等高线抖动（见 kHl9Bayer8 注释）：输出域 ±0.5 8bit 级 */
            float dv = ((float)kHl9Bayer8[((row & 7) << 3) | (x & 7)] * (1.0f / 64.0f) - 0.5f) * 257.0f;
            o0 += dv; o1 += dv; o2 += dv;
            /* v8 掩膜外光晕：SPP 高光重建外溢，负 halo 随 box511(mask) 邻域
             * 密度衰减（LUT 拟合 SPP-m8 修正量，见 kHl9HaloLUT 注释）。
             * 掩膜外像素 Δ 场为 0，这里补上外溢项，消火焰外硬边。 */
            if (g_hl9_hbin != NULL) {
                const size_t o9 = (size_t)row * (size_t)c->w + (size_t)x;
                if (!g_hl9_mask[o9]) {
                    const int hb = g_hl9_hbin[o9];
                    o0 += kHl9HaloLUT[0][hb] * 257.0f;
                    o1 += kHl9HaloLUT[1][hb] * 257.0f;
                    o2 += kHl9HaloLUT[2][hb] * 257.0f;
                }
            }
            /* halo 可为负且深缝处很强，o 可能 <0；uint32 回绕会钳成 65535 白点 */
            if (o0 < 0) o0 = 0; if (o1 < 0) o1 = 0; if (o2 < 0) o2 = 0;
        }
        uint32_t r0 = (uint32_t)(o0 + 0.5f), r1 = (uint32_t)(o1 + 0.5f), r2 = (uint32_t)(o2 + 0.5f);
        if (r0 > 65535) r0 = 65535;
        if (r1 > 65535) r1 = 65535;
        if (r2 > 65535) r2 = 65535;
        if (inline_post || disp != NULL) {
            /* 行内显示域后段：高光恢复 → 对比度 → 色彩模式色调差。
             * 顺序照 build_tone_lut（基础曲线 → hl_recover → contrast →
             * cmtone）；「除回 Standard」是因为直通的拟合吸收了 Standard 档
             * 的曲线。全 no-op 时 r→d→r 往返在 double 里精确还原，字节不变。 */
            double d0 = r0 * (1.0 / 65535.0), d1 = r1 * (1.0 / 65535.0),
                   d2 = r2 * (1.0 / 65535.0);
            const double vv[3] = { v0, v1, v2 };
            double *dd[3] = { &d0, &d1, &d2 };
            int ci;
            if (c->hl_recover > 0.0) {
                for (ci = 0; ci < 3; ci++)
                    if (vv[ci] > X3F_HL_RECOVER_KNEE) {
                        double ds = c->rbase + (1.0 - c->rbase)
                            * sstep01(X3F_HL_RECOVER_KNEE, X3F_HL_RECOVER_TOP, vv[ci]);
                        *dd[ci] += (ds - *dd[ci]) * c->hl_recover;
                    }
            }
            if (c->contrast != 0.0) {
                d0 = apply_contrast(d0, c->contrast);
                d1 = apply_contrast(d1, c->contrast);
                d2 = apply_contrast(d2, c->contrast);
            }
            if (c->cmtone_row >= 0) {
                for (ci = 0; ci < 3; ci++) {
                    double g = cmtone_gain(c->cmtone_row, c->cmtone_w,
                                           vv[ci] / X3F_OE_LIN_WHITE);
                    if (c->cmtone_row_std >= 0) {
                        double gs = cmtone_gain(c->cmtone_row_std, 1.0,
                                                vv[ci] / X3F_OE_LIN_WHITE);
                        if (gs > 1e-9) g /= gs;
                    }
                    if (g != 1.0)
                        *dd[ci] = srgb_encode(srgb_decode(*dd[ci]) * g);
                }
            }
            if (d0 < 0) d0 = 0; if (d0 > 1) d0 = 1;
            if (d1 < 0) d1 = 0; if (d1 > 1) d1 = 1;
            if (d2 < 0) d2 = 0; if (d2 > 1) d2 = 1;
            if (disp != NULL) {
                disp[0] = (float)d0; disp[1] = (float)d1; disp[2] = (float)d2;
                disp += 3;
            } else {
                r0 = (uint32_t)(d0 * 65535.0 + 0.5);
                r1 = (uint32_t)(d1 * 65535.0 + 0.5);
                r2 = (uint32_t)(d2 * 65535.0 + 0.5);
                if (r0 > 65535) r0 = 65535;
                if (r1 > 65535) r1 = 65535;
                if (r2 > 65535) r2 = 65535;
                goto put;
            }
        } else {
        put:;
            /* SDQ 补收（显示域）：这条直写路不经过 apply_common，clip 驱动的
             * 那套收不到这里 —— mode8 在 App 里默认就走这条路，于是
             * (233,228,170) 这种淡黄拉满也不动。判据见 sdq_neut_w。 */
            if (g_sdq_hl > 0.0) {
                double q0 = r0 * (1.0 / 65535.0), q1 = r1 * (1.0 / 65535.0),
                       q2 = r2 * (1.0 / 65535.0);
                double wq = sdq_neut_w(q0, q1, q2);
                if (wq > 0.001) {
                    double mx = q0, t;
                    if (q1 > mx) mx = q1;
                    if (q2 > mx) mx = q2;
                    t = mx;   /* 该是白 → 收到最亮通道 */
                    q0 += (t - q0) * wq;
                    q1 += (t - q1) * wq;
                    q2 += (t - q2) * wq;
                    r0 = (uint32_t)(q0 * 65535.0 + 0.5);
                    r1 = (uint32_t)(q1 * 65535.0 + 0.5);
                    r2 = (uint32_t)(q2 * 65535.0 + 0.5);
                }
            }
            /* 【广色域 2026-11-10】mode8 直写路：SPP Stage2 的直通表是
             * sRGB 显示码值标定，旧代码把它原样写进 8/16bit 缓冲、贴的却是
             * 目标域 ICC —— 预览（出口恒 sRGB）正常，导出 P3/ProPhoto 就
             * 成了「宽标签 + sRGB 码值」的错配，观感整个跑掉。
             *
             * 与 apply_row 末尾（:3768）同款换一次域：
             *     sRGB 显示值 → sRGB 线性 → 目标域线性 → 目标域编码
             * 放在 SDQ 补收**之后**：补收是 sRGB 显示域判据，保持原行为。
             * 只对直写路生效（c->out != NULL）；disp 路交给 apply_common
             * 换域，重复做会错。sRGB 档整段跳过 → 预览逐位零回归。
             * 这里保留宽域容器并让码值与 ICC 自洽，不新增任何收窄步骤。 */
            if (c->out != NULL && c->out_space != X3F_OUT_SRGB) {
                double tv[3], ec[3];
                int ci;
                tv[0] = srgb_decode_s((double)r0 * (1.0 / 65535.0));
                tv[1] = srgb_decode_s((double)r1 * (1.0 / 65535.0));
                tv[2] = srgb_decode_s((double)r2 * (1.0 / 65535.0));
                out_lin_convert(c->out_space, tv);
                for (ci = 0; ci < 3; ci++) {
                    ec[ci] = out_encode_s(c->out_space, tv[ci]);
                    if (ec[ci] < 0.0) ec[ci] = 0.0;   /* 存储物理上限，与 px_put 同口径 */
                    if (ec[ci] > 1.0) ec[ci] = 1.0;
                }
                r0 = (uint32_t)(ec[0] * 65535.0 + 0.5);
                r1 = (uint32_t)(ec[1] * 65535.0 + 0.5);
                r2 = (uint32_t)(ec[2] * 65535.0 + 0.5);
            }
            if (c->bits == 16) {
                uint16_t *d = (uint16_t *)dst8;
                d[0] = (uint16_t)r0; d[1] = (uint16_t)r1; d[2] = (uint16_t)r2;
                dst8 += 6;
            } else {
                dst8[0] = (uint8_t)(r0 >> 8);
                dst8[1] = (uint8_t)(r1 >> 8);
                dst8[2] = (uint8_t)(r2 >> 8);
                dst8 += 3;
            }
        }
        src += 3;
    }
}

/* 3×3 求逆（行主序；行列式≈0 返回 0）。mode8 色彩模式差矩阵用。 */
static int oe8_inv3x3(const double *m, double *o)
{
    const double a = m[0], b = m[1], c = m[2];
    const double d = m[3], e = m[4], f = m[5];
    const double g = m[6], h = m[7], i = m[8];
    const double A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    const double det = a * A + b * B + c * C;
    double id;
    if (fabs(det) < 1e-12) return 0;
    id = 1.0 / det;
    o[0] = A * id;           o[1] = -(b * i - c * h) * id;  o[2] = (b * f - c * e) * id;
    o[3] = B * id;           o[4] = (a * i - c * g) * id;   o[5] = -(a * f - d * e) * id;
    o[6] = C * id;           o[7] = -(a * h - d * b) * id;  o[8] = (a * e - d * b) * id;
    return 1;
}

/* 恒等显示 LUT：mode8 的显示域后段（黑白/饱和度/高光去饱和）直接复用
 * apply_common，让"逐像素套 LUT"那一步变成恒等映射。构造是 lin_to_index
 * 的反函数：lut[j] = clamp((j/S)²·LIN_MAX, 0..1)·65535。二次函数的线性
 * 插值误差 ≈ LIN_MAX·2/(8S²)·65535 < 0.01 LSB，远小于 1。 */
static uint16_t g_oe8_ident[X3F_TONE_LUT_SIZE + 1];
static const uint16_t *oe8_identity_lut(void)
{
    static int built = 0;
    if (!built) {
        int i;
        for (i = 0; i <= X3F_TONE_LUT_SIZE; i++) {
            double v = (double)i / X3F_TONE_LUT_SIZE;
            v = v * v * X3F_LIN_MAX;
            if (v > 1.0) v = 1.0;
            g_oe8_ident[i] = (uint16_t)(v * 65535.0 + 0.5);
        }
        built = 1;
    }
    return g_oe8_ident;
}

/* ----------------------------------------------------------------------
 * 解码侧边缘锐化(unsharp): 在**显示空间**(apply_common 输出之后)做,
 * 因为此时所有逐通道色调/饱和度曲线都已跑完, 把同量高频加回三通道
 * 不会再被曲线弯成色相偏移 —— 这正是早期把 unsharp 放在解码融合阶段
 * (线性域)会"明显发红 + 黄花去饱和"的根因(实测旧版整图 R−B 均值
 * 由 −3.8 漂到 +2.8, 且黄花区彩度掉 34%)。
 * 做法: 真 Rec709 亮度做 unsharp 源; 叠加**色度梯度蒙版**: 黄/紫彩边
 * 是突变色度台阶(高 |∇chr|)→ 锐化增益压到 0(不放大彩边); 均匀着色
 * 纹理与中性边缘→满锐化补回 NLM 柔掉的那线细节。
 * 旋钮: X3FDEC_SHARP 强度(默认 0.4, 0.3~0.8 起调; 显式 <=0 关闭);
 *   【2026-10-26 实测校准】0.4 保留不动。曾试提到 1.0 想追 SPP，CLI 真机
 *   A/B（X3FDEC_SHARP 扫 0.4~0.8）发现整机输出本就比 SPP 锐：亮度匹配后
 *   Laplacian HF 比值 0.4 档=1.48、0.8 档=1.62（SPP=1.0 基准）—— sp 杠杆很弱，
 *   画面锐度主要来自 Sharpen 滑杆那条 spat_sharp 链。提到 1.0 会过冲（比值
 *   1.65，样张出现轻微亮边）。此前"8bit 出口比 SPP 软 2×(0.49)"的结论出自
 *   一张旧脚本导出的过期对照图，不代表当前管线。FPN 防护（v2 局部窗行列
 *   去均值，见实现处注释）在乘 sp 之前剥掉本地持续的 1D 行/列偏移，故调
 *   sp 不会引入横/竖条纹。
 * X3FDEC_SHARP_R 模糊半径(默认 2); X3FDEC_SHARP_G 色度梯度抑制阈值
 * (显示域原始单位, 默认 0.3·maxval, 调小=压更狠);
 * X3FDEC_SHARP_FPNW 防护窗宽(默认 65, <3 = 关防护用纯 hf)。 */
/* ================================================================
 * X3Lumina「辉光」(glow) —— 高光辉光 Bloom
 * SPP 没有这一项，X3Lumina 独有显示域效果，存 sidecar、不写 SECs 段。
 * 作用在主渲染路径的最终显示像素（out 写定后）之上：
 *   1) 提取高光：亮度 > glow_threshold 的区域（smoothstep 软过渡）才发光；
 *   2) 降采样 + 分离高斯模糊生成柔光图层；
 *   3) 以 screen 混合加回原图（强度 glow 控制），暗部/低光区域不受影响。
 * glow == 0 时不调用本函数，输出逐位不变（零滑杆 md5 铁律）。
 * ================================================================ */
static void glow_gauss_h(const float *src, float *dst, int w, int h, int ch,
                         float w0, float w1)
{
    int y, x, c;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            int xl = x > 0 ? x - 1 : 0, xr = x < w - 1 ? x + 1 : w - 1;
            for (c = 0; c < ch; c++)
                dst[(y*w + x)*ch + c] =
                    src[(y*w + xl)*ch + c]*w1 + src[(y*w + x)*ch + c]*w0 +
                    src[(y*w + xr)*ch + c]*w1;
        }
    }
}
static void glow_gauss_v(const float *src, float *dst, int w, int h, int ch,
                         float w0, float w1)
{
    int y, x, c;
    for (y = 0; y < h; y++) {
        int yu = y > 0 ? y - 1 : 0, yd = y < h - 1 ? y + 1 : h - 1;
        for (x = 0; x < w; x++)
            for (c = 0; c < ch; c++)
                dst[(y*w + x)*ch + c] =
                    src[(yu*w + x)*ch + c]*w1 + src[(y*w + x)*ch + c]*w0 +
                    src[(yd*w + x)*ch + c]*w1;
    }
}

static void apply_glow(void *vout, int w, int h, int stride, int bits,
                       const x3f_edit_params_t *p)
{
    const int ch = 3;
    const size_t npix = (size_t)w * h;
    int y, x, c;

    float *src = (float *)malloc(npix * ch * sizeof(float));
    if (src == NULL) return;
    /* 1) 反量化显示值到 0..1（stride 行跨距可能大于 w*3 字节） */
    if (bits == 8) {
        const uint8_t *o = (const uint8_t *)vout;
        for (y = 0; y < h; y++) {
            const uint8_t *row = o + (size_t)y * stride;
            for (x = 0; x < w; x++)
                for (c = 0; c < ch; c++)
                    src[((size_t)y*w + x)*ch + c] = row[x*ch + c] / 255.0f;
        }
    } else {
        const uint16_t *o = (const uint16_t *)vout;
        int sb = stride / 2;
        for (y = 0; y < h; y++) {
            const uint16_t *row = o + (size_t)y * sb;
            for (x = 0; x < w; x++)
                for (c = 0; c < ch; c++)
                    src[((size_t)y*w + x)*ch + c] = row[x*ch + c] / 65535.0f;
        }
    }

    /* 2) 高光掩膜 + 取出发光源（只高光区参与发光） */
    const float thr = (float)p->glow_threshold;
    float *glow = (float *)malloc(npix * ch * sizeof(float));
    if (glow == NULL) { free(src); return; }
    for (size_t i = 0; i < npix; i++) {
        float r = src[i*ch+0], g = src[i*ch+1], b = src[i*ch+2];
        float lum = 0.2126f*r + 0.7152f*g + 0.0722f*b;
        float t = (lum - thr) / (1.0f - thr);
        if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
        float m = t*t*(3.0f - 2.0f*t);   /* smoothstep */
        glow[i*ch+0] = r * m;
        glow[i*ch+1] = g * m;
        glow[i*ch+2] = b * m;
    }

    /* 3) 降采样 + 分离高斯模糊 */
    int ds = (int)(2.0f + (float)p->glow_radius * 14.0f);
    if (ds < 2) ds = 2; else if (ds > 32) ds = 32;
    int dw = (w + ds - 1) / ds, dh = (h + ds - 1) / ds;
    if (dw < 1) dw = 1; if (dh < 1) dh = 1;
    float *dbuf = (float *)malloc((size_t)dw * dh * ch * sizeof(float));
    if (dbuf == NULL) { free(glow); free(src); return; }
    for (int yy = 0; yy < dh; yy++) {
        for (int xx = 0; xx < dw; xx++) {
            float fx = ((float)xx + 0.5f) * w / dw - 0.5f;
            float fy = ((float)yy + 0.5f) * h / dh - 0.5f;
            int x0 = (int)floorf(fx), y0 = (int)floorf(fy);
            float tx = fx - x0, ty = fy - y0;
            int x1 = x0+1, y1 = y0+1;
            if (x0 < 0) x0 = 0; if (x0 > w-1) x0 = w-1; if (x1 > w-1) x1 = w-1;
            if (y0 < 0) y0 = 0; if (y0 > h-1) y0 = h-1; if (y1 > h-1) y1 = h-1;
            for (c = 0; c < ch; c++) {
                float v00 = glow[((size_t)y0*w + x0)*ch + c];
                float v01 = glow[((size_t)y0*w + x1)*ch + c];
                float v10 = glow[((size_t)y1*w + x0)*ch + c];
                float v11 = glow[((size_t)y1*w + x1)*ch + c];
                float top = v00*(1-tx) + v01*tx;
                float bot = v10*(1-tx) + v11*tx;
                dbuf[((size_t)yy*dw + xx)*ch + c] = top*(1-ty) + bot*ty;
            }
        }
    }
    float sigma = 1.0f + (float)p->glow_radius * 2.0f;
    float k = expf(-1.0f / (2.0f * sigma * sigma));
    float w1 = k / (1.0f + 2.0f*k), w0 = 1.0f - 2.0f*w1;
    float *dwork = (float *)malloc((size_t)dw * dh * ch * sizeof(float));
    if (dwork == NULL) { free(dbuf); free(glow); free(src); return; }
    for (int it = 0; it < 2; it++) {
        glow_gauss_h(dbuf, dwork, dw, dh, ch, w0, w1);
        glow_gauss_v(dwork, dbuf, dw, dh, ch, w0, w1);
    }
    free(dwork);

    /* 上采样回全分辨率（双线性） */
    float *blur = (float *)malloc(npix * ch * sizeof(float));
    if (blur == NULL) { free(dbuf); free(glow); free(src); return; }
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            float fx = ((float)x + 0.5f) * dw / w - 0.5f;
            float fy = ((float)y + 0.5f) * dh / h - 0.5f;
            int x0 = (int)floorf(fx), y0 = (int)floorf(fy);
            float tx = fx - x0, ty = fy - y0;
            int x1 = x0+1, y1 = y0+1;
            if (x0 < 0) x0 = 0; if (x0 > dw-1) x0 = dw-1; if (x1 > dw-1) x1 = dw-1;
            if (y0 < 0) y0 = 0; if (y0 > dh-1) y0 = dh-1; if (y1 > dh-1) y1 = dh-1;
            for (c = 0; c < ch; c++) {
                float v00 = dbuf[((size_t)y0*dw + x0)*ch + c];
                float v01 = dbuf[((size_t)y0*dw + x1)*ch + c];
                float v10 = dbuf[((size_t)y1*dw + x0)*ch + c];
                float v11 = dbuf[((size_t)y1*dw + x1)*ch + c];
                float top = v00*(1-tx) + v01*tx;
                float bot = v10*(1-tx) + v11*tx;
                blur[((size_t)y*w + x)*ch + c] = top*(1-ty) + bot*ty;
            }
        }
    }
    free(dbuf);

    /* 4) screen 混合加回（强度 glow，暗部不受累） */
    const float s = (float)p->glow;
    for (size_t i = 0; i < npix * ch; i++) {
        float base = src[i];
        float add = blur[i] * s;
        float v = 1.0f - (1.0f - base) * (1.0f - add);
        src[i] = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    }
    free(blur);

    /* 5) 量化回 out */
    if (bits == 8) {
        uint8_t *o = (uint8_t *)vout;
        for (y = 0; y < h; y++) {
            uint8_t *row = o + (size_t)y * stride;
            for (x = 0; x < w; x++)
                for (c = 0; c < ch; c++) {
                    float v = src[((size_t)y*w + x)*ch + c];
                    int iv = (int)(v * 255.0f + 0.5f);
                    row[x*ch + c] = (uint8_t)(iv < 0 ? 0 : iv > 255 ? 255 : iv);
                }
        }
    } else {
        uint16_t *o = (uint16_t *)vout;
        int sb = stride / 2;
        for (y = 0; y < h; y++) {
            uint16_t *row = o + (size_t)y * sb;
            for (x = 0; x < w; x++)
                for (c = 0; c < ch; c++) {
                    float v = src[((size_t)y*w + x)*ch + c];
                    int iv = (int)(v * 65535.0f + 0.5f);
                    row[x*ch + c] = (uint16_t)(iv < 0 ? 0 : iv > 65535 ? 65535 : iv);
                }
        }
    }
    free(glow);
    free(src);
}

/* 解码侧锐化（显示域, 色相安全版）。旋钮：
 * X3FDEC_SHARP     强度 sp（默认 0.4；<=0 = 关）
 * X3FDEC_SHARP_R   模糊半径（默认 2）
 * X3FDEC_SHARP_G   色度梯度抑制阈（显示域原始单位, 默认 0.3·maxval, 调小=压更狠）
 * X3FDEC_SHARP_FPNW 条纹防护窗宽（默认 65, <3 = 关防护）
 * X3FDEC_SHARP_CLAMP halo 钳制允许量（8bit 口径 /255, 默认 0=硬钳；
 *                  >0 保留合法过冲, <0=关钳制） */
/* ── apply_decode_unsharp 的并行作业（2026-10-31）──────────────────────
 * 原实现十来遍全图遍历全是单线程，而且逐像素做 i/W、i%W 除法、垂直方向
 * 按列跨行访问（每步一个 cache miss）。实测在 5424×3616 上吃掉导出约 3.7s
 * （X3L_EXPORT_TIME 打点里它落在 apply 之外，一直没被算进任何分段）。
 *
 * 拆成按行 / 按列块两类作业：每个迭代只写自己的行（或自己的列块），
 * 没有跨迭代的累加，前缀和仍是同一列/行、同一顺序 → 与串行**逐位相同**。
 * 水平方向按行并行，垂直方向按列块并行（块内每行只碰 blk 个连续 float）。 */
#define US_NCHUNK 64

typedef struct {
    void *vout;
    float *Y, *Yb, *chr, *msk;
    const float *ref;
    float *Ymin, *Ymax, *Tmm, *boxv;
    int W, H, r, fr, bits, stride, maxv;
    double sp, ghi, glo, allow;
} us_job_t;

/* 读显示域 RGB → Y / Yb / chr（按行） */
static void us_row_luma(void *vctx, size_t yy)
{
    const us_job_t *j = (const us_job_t *)vctx;
    const int W = j->W;
    const size_t base = yy * (size_t)W;
    int x;
    for (x = 0; x < W; x++) {
        float R, G, B, yv;
        if (j->bits == 16) {
            const uint16_t *p = (const uint16_t *)j->vout
                              + yy * (size_t)(j->stride / 2) + (size_t)x * 3;
            R = (float)p[0]; G = (float)p[1]; B = (float)p[2];
        } else {
            const uint8_t *p = (const uint8_t *)j->vout
                             + yy * (size_t)j->stride + (size_t)x * 3;
            R = (float)p[0]; G = (float)p[1]; B = (float)p[2];
        }
        yv = 0.2126f * R + 0.7152f * G + 0.0722f * B;
        j->Y[base + x]  = yv;
        j->Yb[base + x] = yv;
        j->chr[base + x] = fabsf(R - B) + fabsf(G - B);
    }
}

/* 水平可分离 box：Y → Yb（按行，前缀和精确 clamp 窗） */
static void us_row_hbox(void *vctx, size_t yy)
{
    const us_job_t *j = (const us_job_t *)vctx;
    const int W = j->W, r = j->r;
    const float *src = j->Y + yy * (size_t)W;
    float *dst = j->Yb + yy * (size_t)W;
    double *P = (double *)malloc(((size_t)W + 1) * sizeof(double));
    int x;
    if (P == NULL) return;
    P[0] = 0.0;
    for (x = 0; x < W; x++) P[x + 1] = P[x] + src[x];
    for (x = 0; x < W; x++) {
        int lo = x - r, hi = x + r;
        int ilo = lo < 0 ? 0 : lo;
        int ihi = hi > W - 1 ? W - 1 : hi;
        double s = P[ihi + 1] - P[ilo];
        int cnt = ihi - ilo + 1;
        if (lo < 0) { s += (double)(-lo) * src[0]; cnt += -lo; }
        if (hi > W - 1) { s += (double)(hi - (W - 1)) * src[W - 1]; cnt += hi - (W - 1); }
        dst[x] = (float)(s / cnt);
    }
    free(P);
}

/* 垂直可分离 box：Yb 就地（按列块） */
static void us_col_vbox(void *vctx, size_t blk)
{
    const us_job_t *j = (const us_job_t *)vctx;
    const int W = j->W, H = j->H, r = j->r;
    const int x0 = (int)((size_t)W * blk / US_NCHUNK);
    const int x1 = (int)((size_t)W * (blk + 1) / US_NCHUNK);
    double *P;
    int x, y;
    if (x0 >= x1) return;
    P = (double *)malloc(((size_t)H + 1) * sizeof(double));
    if (P == NULL) return;
    for (x = x0; x < x1; x++) {
        P[0] = 0.0;
        for (y = 0; y < H; y++) P[y + 1] = P[y] + j->Yb[(size_t)y * W + x];
        for (y = 0; y < H; y++) {
            int lo = y - r, hi = y + r;
            int ilo = lo < 0 ? 0 : lo;
            int ihi = hi > H - 1 ? H - 1 : hi;
            double s = P[ihi + 1] - P[ilo];
            int cnt = ihi - ilo + 1;
            if (lo < 0) { s += (double)(-lo) * j->Yb[x]; cnt += -lo; }
            if (hi > H - 1) { s += (double)(hi - (H - 1)) * j->Yb[(size_t)(H - 1) * W + x]; cnt += hi - (H - 1); }
            j->Yb[(size_t)y * W + x] = (float)(s / cnt);
        }
    }
    free(P);
}

/* 水平窗 min/max：ref → Tmm(垫底) / Ymax（按行） */
static void us_row_minmax(void *vctx, size_t yy)
{
    const us_job_t *j = (const us_job_t *)vctx;
    const int W = j->W, r = j->r;
    const float *src = j->ref + yy * (size_t)W;
    float *dmn = j->Tmm + yy * (size_t)W;
    float *dmx = j->Ymax + yy * (size_t)W;
    int x;
    for (x = 0; x < W; x++) {
        int lo = x - r, hi = x + r, k;
        float mn, mx;
        if (lo < 0) lo = 0;
        if (hi > W - 1) hi = W - 1;
        mn = src[lo]; mx = src[lo];
        for (k = lo + 1; k <= hi; k++) {
            float v = src[k];
            if (v < mn) mn = v; else if (v > mx) mx = v;
        }
        dmn[x] = mn; dmx[x] = mx;
    }
}

/* 垂直窗 min/max：Tmm/Ymax → Ymin/Tmm（按列块） */
static void us_col_minmax(void *vctx, size_t blk)
{
    const us_job_t *j = (const us_job_t *)vctx;
    const int W = j->W, H = j->H, r = j->r;
    const int x0 = (int)((size_t)W * blk / US_NCHUNK);
    const int x1 = (int)((size_t)W * (blk + 1) / US_NCHUNK);
    int x, y;
    for (x = x0; x < x1; x++) {
        for (y = 0; y < H; y++) {
            int lo = y - r, hi = y + r, k;
            float mn, mx;
            if (lo < 0) lo = 0;
            if (hi > H - 1) hi = H - 1;
            mn = j->Tmm[(size_t)lo * W + x];
            mx = j->Ymax[(size_t)lo * W + x];
            for (k = lo + 1; k <= hi; k++) {
                float v1 = j->Tmm[(size_t)k * W + x];
                float v2 = j->Ymax[(size_t)k * W + x];
                if (v1 < mn) mn = v1;
                if (v2 > mx) mx = v2;
            }
            j->Ymin[(size_t)y * W + x] = mn;
            j->Tmm[(size_t)y * W + x] = mx;   /* 覆写安全: Ymax 垂直 pass 只读 */
        }
    }
}

/* 色度梯度蒙版：|∇chr| → msk（按行） */
static void us_row_msk(void *vctx, size_t yy)
{
    const us_job_t *j = (const us_job_t *)vctx;
    const int W = j->W, H = j->H;
    const double ghi = j->ghi, glo = j->glo;
    const size_t base = yy * (size_t)W;
    int x;
    for (x = 0; x < W; x++) {
        const size_t i = base + (size_t)x;
        float cL = j->chr[x > 0      ? i - 1 : i];
        float cR = j->chr[x < W - 1  ? i + 1 : i];
        float cU = j->chr[yy > 0     ? i - W : i];
        float cD = j->chr[yy < (size_t)H - 1 ? i + W : i];
        float g = sqrtf((cR - cL) * (cR - cL) + (cD - cU) * (cD - cU));
        double t = (ghi - g) / (ghi - glo);
        if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
        j->msk[i] = (float)(t * t * (3.0 - 2.0 * t));
    }
}

/* hf 生成：Yb ← Y − Yb（按行） */
static void us_row_hf(void *vctx, size_t yy)
{
    const us_job_t *j = (const us_job_t *)vctx;
    const int W = j->W;
    const size_t base = yy * (size_t)W;
    int x;
    for (x = 0; x < W; x++) j->Yb[base + x] = j->Y[base + x] - j->Yb[base + x];
}

/* FPN 防护列方向：hf − box_y(hf) → boxv（按列块） */
static void us_col_fpn(void *vctx, size_t blk)
{
    const us_job_t *j = (const us_job_t *)vctx;
    const int W = j->W, H = j->H, fr = j->fr;
    const int x0 = (int)((size_t)W * blk / US_NCHUNK);
    const int x1 = (int)((size_t)W * (blk + 1) / US_NCHUNK);
    double *P;
    int x, y;
    if (x0 >= x1) return;
    P = (double *)malloc(((size_t)H + 1) * sizeof(double));
    if (P == NULL) return;
    for (x = x0; x < x1; x++) {
        P[0] = 0.0;
        for (y = 0; y < H; y++) P[y + 1] = P[y] + j->Yb[(size_t)y * W + x];
        for (y = 0; y < H; y++) {
            int lo = y - fr, hi = y + fr;
            int ilo = lo < 0 ? 0 : lo;
            int ihi = hi > H - 1 ? H - 1 : hi;
            double s = P[ihi + 1] - P[ilo];
            int cnt = ihi - ilo + 1;
            if (lo < 0) { s += (double)(-lo) * j->Yb[x]; cnt += -lo; }
            if (hi > H - 1) { s += (double)(hi - (H - 1)) * j->Yb[(size_t)(H - 1) * W + x]; cnt += hi - (H - 1); }
            j->boxv[(size_t)y * W + x] = (float)(s / cnt);
        }
    }
    free(P);
}

/* FPN 防护行方向：hf − box_x(hf) → boxv（按行） */
static void us_row_fpn(void *vctx, size_t yy)
{
    const us_job_t *j = (const us_job_t *)vctx;
    const int W = j->W, fr = j->fr;
    const float *src = j->Yb + yy * (size_t)W;
    float *dst = j->boxv + yy * (size_t)W;
    double *P = (double *)malloc(((size_t)W + 1) * sizeof(double));
    int x;
    if (P == NULL) return;
    P[0] = 0.0;
    for (x = 0; x < W; x++) P[x + 1] = P[x] + src[x];
    for (x = 0; x < W; x++) {
        int lo = x - fr, hi = x + fr;
        int ilo = lo < 0 ? 0 : lo;
        int ihi = hi > W - 1 ? W - 1 : hi;
        double s = P[ihi + 1] - P[ilo];
        int cnt = ihi - ilo + 1;
        if (lo < 0) { s += (double)(-lo) * src[0]; cnt += -lo; }
        if (hi > W - 1) { s += (double)(hi - (W - 1)) * src[W - 1]; cnt += hi - (W - 1); }
        dst[x] = (float)(s / cnt);
    }
    free(P);
}

/* hf ← hf − boxv（按行） */
static void us_row_sub(void *vctx, size_t yy)
{
    const us_job_t *j = (const us_job_t *)vctx;
    const int W = j->W;
    const size_t base = yy * (size_t)W;
    int x;
    for (x = 0; x < W; x++) j->Yb[base + x] -= j->boxv[base + x];
}

/* unsharp 回写三通道（按行） */
static void us_row_apply(void *vctx, size_t yy)
{
    const us_job_t *j = (const us_job_t *)vctx;
    const int W = j->W, maxv = j->maxv;
    const size_t base = yy * (size_t)W;
    const double sp = j->sp, allow = j->allow;
    int x;
    for (x = 0; x < W; x++) {
        const size_t i = base + (size_t)x;
        float hf = j->Yb[i];                       /* 已是去条纹高通 */
        float add = (float)(sp * j->msk[i] * hf);
        if (j->Ymin != NULL) {
            float ynew = j->Y[i] + add;
            float himx = j->Tmm[i] + (float)allow, lomn = j->Ymin[i] - (float)allow;
            if (ynew > himx) ynew = himx;
            else if (ynew < lomn) ynew = lomn;
            add = ynew - j->Y[i];
        }
        if (j->bits == 16) {
            uint16_t *p = (uint16_t *)j->vout + yy * (size_t)(j->stride / 2) + (size_t)x * 3;
            double v0 = (double)p[0] + add, v1 = (double)p[1] + add, v2 = (double)p[2] + add;
            p[0] = v0 < 0 ? 0 : (v0 > maxv ? maxv : (uint16_t)v0);
            p[1] = v1 < 0 ? 0 : (v1 > maxv ? maxv : (uint16_t)v1);
            p[2] = v2 < 0 ? 0 : (v2 > maxv ? maxv : (uint16_t)v2);
        } else {
            uint8_t *p = (uint8_t *)j->vout + yy * (size_t)j->stride + (size_t)x * 3;
            double v0 = (double)p[0] + add, v1 = (double)p[1] + add, v2 = (double)p[2] + add;
            p[0] = v0 < 0 ? 0 : (v0 > maxv ? maxv : (uint8_t)v0);
            p[1] = v1 < 0 ? 0 : (v1 > maxv ? maxv : (uint8_t)v1);
            p[2] = v2 < 0 ? 0 : (v2 > maxv ? maxv : (uint8_t)v2);
        }
    }
}

static void apply_decode_unsharp(void *vout, int w, int h, int stride, int bits)
{
    const char *e;
    /* 默认**关**（2026-11-04 回退）。
     * 历史：2026-10-25 为补 NLM_H=200 柔掉的高频、修线性域锐化发红，
     *       把这一步放在显示域并固化默认 0.4（commit 69603f9）。
     *       此后它连续造出 FPN 横竖带、颈下黑带、0877 天空竖纹、贴边
     *       halo、以及 2026-11-04 定案的「楼顶细线深入天空」——全部是
     *       同一个跨强边暗下冲，历次只加防护、没重估默认是否该开。
     *       10-26 已实测其锐度杠杆很弱（0.4→0.8 的 HF 仅 1.48→1.62）。
     * 现默认关；需要时 X3FDEC_SHARP=0.4（或任意 >0 值）显式开启。 */
    double sp = 0.0;
    int r = 2;
    if ((e = getenv("X3FDEC_SHARP"))) {
        double v = atof(e);
        sp = (v > 0.0) ? v : 0.0;   /* 显式 >0 = 开启; <=0 = 关闭 */
    }
    if ((e = getenv("X3FDEC_SHARP_R"))) { int rr = (int)atof(e); if (rr > 0) r = rr; }
    if (sp <= 0.0 || r <= 0) return;
    int maxv = (bits == 16) ? 65535 : 255;
    double ghi = 0.3 * (double)maxv;
    if ((e = getenv("X3FDEC_SHARP_G"))) { double gg = atof(e); if (gg > 0) ghi = gg; }
    /* halo 钳制允许量（8bit 口径）：0=硬钳, <0=关。
     * 默认 0（硬钳）——2026-10-28 用户实测 1.5 仍「轻微可见」贴边线；
     * 硬钳后 x2137 暗侧残余 0.37/255、亮侧 0.0（纹理仍比无锐化 +19%）。 */
    double clamp_allow = 0.0;
    int clamp_on = 1;
    if ((e = getenv("X3FDEC_SHARP_CLAMP"))) {
        double cv = atof(e);
        if (cv < 0.0) clamp_on = 0; else clamp_allow = cv;
    }
    /* 钳制参考源：0=原始亮度（默认，唯一正确语义——原始 Y 本来就在自己
     * 窗口 min/max 内，只剪纯过冲）；1=模糊后亮度（实验否决：box 的
     * min/max 地板比原始纹理自身峰值更紧，会把原始暗/亮纹理峰一起削，
     * 纹理 RMS 27.2→13.6 崩掉且暗侧反向变亮 +9/255）。 */
    int clamp_src_blur = 0;
    if ((e = getenv("X3FDEC_SHARP_CLAMPSRC"))) clamp_src_blur = atoi(e) != 0;
    long N = (long)w * h;
    float *Y  = malloc((size_t)N * sizeof(float));   /* 原始亮度(Rec709) */
    float *Yb = malloc((size_t)N * sizeof(float));   /* 模糊亮度 → 后作 hf */
    float *chr = malloc((size_t)N * sizeof(float));  /* 色度 |R−B|+|G−B| */
    float *msk = malloc((size_t)N * sizeof(float));  /* 色度梯度蒙版 */
    us_job_t jb;
    int x, y;
    const int W = w, H = h;

    if (!Y || !Yb || !chr || !msk) { free(Y); free(Yb); free(chr); free(msk); return; }

    memset(&jb, 0, sizeof(jb));
    jb.vout = vout;
    jb.Y = Y; jb.Yb = Yb; jb.chr = chr; jb.msk = msk;
    jb.W = W; jb.H = H; jb.r = r; jb.bits = bits; jb.stride = stride; jb.maxv = maxv;
    jb.sp = sp; jb.ghi = ghi; jb.glo = ghi / 4.0; jb.allow = clamp_allow * maxv / 255.0;

    /* 读显示域 RGB → Y 与 chr(原始亮度顺带存进 Yb 供模糊) */
    x3f_parallel_for((size_t)H, us_row_luma, &jb);

    /* 可分离 box 模糊(前缀和, 边界 clamp 精确)。
     * ⚠旧滑动和实现有真 bug：首窗把 src[0] 记三次(k=−2,−1,0 都映射到
     * 下标 0)，滑动中多出的一份永不移除、cnt 停在 2r+2 —— 整行/列的
     * 模糊都携带 (边缘像素−局部均值)/6 的常数偏移。左边缘是亮窗时暗
     * 区域的高通全为负，被 sp 放大成区域性压暗（2501 领口下方贯穿黑带，
     * 实测锐化贡献 −2~−10/255；随后的行列去均值只能减掉每行/列的均值，
     * 压不掉这种区域性结构）。改为前缀和精确求 clamp 窗口均值。
     * 并行版：水平按行、垂直按列块（每列前缀和顺序不变 → 逐位相同）。 */
    x3f_parallel_for((size_t)H, us_row_hbox, &jb);
    x3f_parallel_for((size_t)US_NCHUNK, us_col_vbox, &jb);

    /* halo 钳制（2026-10-28）：强边缘的锐化过冲/下冲 = 用户可见的贴边
     * 亮/暗细线（0877 招牌带顶边实测：暗侧下冲比无锐化深 ~7/255、
     * 宽 1-2px；亮侧 +7.5）。做法：锐化后的亮度不得超出「参考亮度的
     * (2r+1)² 局部 min/max」±允许量——真实细节在局部范围内本来就有
     * 对应起伏、不受影响；只有被推出局部范围的纯过冲被剪掉。min/max
     * 用可分离两趟（水平→Tmm/Ymax，垂直→Ymin/Tmm），窗半径取模糊
     * 半径 r（过冲只在 ±r 内出现）。允许量 8bit 口径默认 0（硬钳）：
     * 用户实测 1.5 仍轻微可见；0 时 0877 x2137 暗侧残余 0.37/255、
     * 亮侧 0.0，纹理较无钳制 −12%（仍比无锐化 +19%）。
     * 参考源必须是**原始亮度**：raw Y 本来就在自己窗口 min/max 内，
     * 钳制只剪「推出原始局部范围」的纯过冲。模糊源实验否决（见上）。 */
    if (clamp_on) {
        jb.ref = clamp_src_blur ? (const float *)Yb : (const float *)Y;
        jb.Ymin = malloc((size_t)N * sizeof(float));
        jb.Ymax = malloc((size_t)N * sizeof(float));
        jb.Tmm  = malloc((size_t)N * sizeof(float));
        if (jb.Ymin && jb.Ymax && jb.Tmm) {
            x3f_parallel_for((size_t)H, us_row_minmax, &jb);
            x3f_parallel_for((size_t)US_NCHUNK, us_col_minmax, &jb);
        } else {
            free(jb.Ymin); free(jb.Ymax); free(jb.Tmm);
            jb.Ymin = jb.Ymax = jb.Tmm = NULL;
        }
    }
    /* 色度梯度蒙版: g=|∇chr|(4 邻域); 彩边(高 g)→msk→0 关锐化,
     * 中性/均匀着色(低 g)→msk=1 满锐化。smoothstep(ghi→ghi/4)。 */
    if (getenv("X3FDEC_SHARP_DBG")) {
        const double hmax = ghi * 1.5;
        enum { HN = 64 };
        long hb[HN];
        int k;
        for (k = 0; k < HN; k++) hb[k] = 0;
        for (y = 0; y < H; y++) {
            for (x = 0; x < W; x++) {
                size_t i = (size_t)y * W + x;
                float cL = chr[x > 0     ? i - 1 : i];
                float cR = chr[x < W - 1 ? i + 1 : i];
                float cU = chr[y > 0     ? i - W : i];
                float cD = chr[y < H - 1 ? i + W : i];
                float g = sqrtf((cR - cL) * (cR - cL) + (cD - cU) * (cD - cU));
                int b = (int)(g / hmax * HN); if (b >= HN) b = HN - 1; if (b < 0) b = 0;
                hb[b]++;
            }
        }
        fprintf(stderr, "[sharp] |grad chr| hist (display, ghi=%.0f):\n", ghi);
        for (k = 0; k < HN; k += 4)
            fprintf(stderr, "  g>=%-7.0f : %ld\n", (double)k / HN * hmax, hb[k]);
    }
    x3f_parallel_for((size_t)H, us_row_msk, &jb);

    /* unsharp: add = sp·msk·(orig_luma − blur) 同量回加三通道。
     * 显示空间、无后续曲线 ⇒ 加同量到 RGB 是真正的亮度提升, 色相不变。
     *
     * FPN 防护 v2（2026-10-28）：**局部窗行列去均值**。
     * v1（全图行/列均值, 2026-10-2x）有系统性缺陷：列相干的 hf 不全是
     * FPN，还有跨数百行的真实竖直结构——0877 中地带 x1594 的白色窗框
     * 板实测 B 列均值 −22/255 全部来自内容。全图平均把内容列纹当成
     * FPN 减除，等价于把它的负片印进"没有这个结构的区域"：0877 天空
     * x1594 的 +4.353/255 竖纹 = −0.4·colm(x1594)（numpy 仿真逐位吻
     * 合），锐化关时天空本底只有 0.18。
     * v2 改为 hf 减自己的 65 窗 box（先列后行，clamp 窗）：只剥离"本
     * 地区域内持续存在的 1D 偏移"——FPN 整柱/整行存在照样被剥掉；
     * 紧凑 2D 细节（真实锐化对象）基本不动；天空拿到 ≈0 的局部估计，
     * 不再被注入。窗 65 仿真：天空列纹 4.35→0.49/255（无防护 0.22），
     * 窗框板锐化保留 ~80%。窗可用 X3FDEC_SHARP_FPNW 覆盖（0=关防护
     * 用纯 hf）。 */
    x3f_parallel_for((size_t)H, us_row_hf, &jb);   /* Yb ← hf */
    {
        int fw = 65;
        if ((e = getenv("X3FDEC_SHARP_FPNW"))) fw = (int)atof(e);
        if (fw >= 3) {
            jb.fr = fw / 2;
            jb.boxv = malloc((size_t)N * sizeof(float));
            if (jb.boxv != NULL) {
                /* 列方向: hf − box_y(hf) —— 剥离本地区域内持续的列偏移 */
                x3f_parallel_for((size_t)US_NCHUNK, us_col_fpn, &jb);
                x3f_parallel_for((size_t)H, us_row_sub, &jb);
                /* 行方向: 再减 box_x(·) —— 剥离持续的行偏移 */
                x3f_parallel_for((size_t)H, us_row_fpn, &jb);
                x3f_parallel_for((size_t)H, us_row_sub, &jb);
            }
            free(jb.boxv);
            jb.boxv = NULL;
        }
        /* fw < 3: 防护关闭, hf 原样参与锐化 */
    }
    x3f_parallel_for((size_t)H, us_row_apply, &jb);

    free(Y); free(Yb); free(chr); free(msk); free(jb.Ymin); free(jb.Ymax); free(jb.Tmm);
}

/* ── 高频软压缩 hf_soften（2026-11-05）────────────────────────────────
 *
 * 解决「皮肤卡粉」：同场景对相机内嵌 JPEG，SD14 皮肤区 σ1.0 的亮度高频是
 * 相机的 1.4~2.2 倍（左颊/下巴最明显），视觉上就是满脸细碎明暗颗粒。
 *
 * 已排除的两条路：
 *   · apply_decode_unsharp —— 默认早就关（sp=0），不是它；
 *   · NLM 亮度 —— `luminance=auto` 的 ISO 门槛 800，低 ISO 根本不开；硬开到
 *     luma_scale 2.5 也只能压到 1.1~1.6 倍，而且会把结构一起抹（它按 patch
 *     相似度判定，压不住「有空间结构的微起伏」）。相机那边是 ISP 直接磨平。
 *
 * 做法：Wiener 型软阈值，只作用在亮度上
 *     Yb = 三角核(Y, [1,2,1]/4 横竖各一遍 ≈ σ1.0)
 *     hp = Y - Yb
 *     Y' = Yb + hp · hp²/(hp² + t²)
 * |hp| ≫ t 时系数趋近 1（睫毛、唇纹、边缘原样保留），|hp| ≪ t 时趋近 0
 * （颗粒被吃进低频）。回投按 k = Y'/Y 缩放三通道 —— 只改亮度，色度坐标不变。
 *
 * t 是 8bit 码值口径，按机型门取值（非经典 Foveon 恒 0 = 不跑 pass、逐字节
 * 不变）。X3F_HF_SOFTEN=<t> 覆盖（<=0 显式关，lab 对拍用）。
 *
 * 实测（SDIM0173 皮肤 σ1.0 高频，相机=1.00）：
 *     额头 1.37→1.10  左颊 1.89→1.28  下巴 2.18→1.50  脖子 1.34→1.15
 * 非皮肤区同步回到 1.0~1.2（树叶 1.30→1.02、花 1.33→1.19、头发 1.49→1.34），
 * 因为相机 JPEG 是**整体**做了降噪，不是只磨皮肤。
 */
typedef struct {
    void *vout;
    int w, h, bits, stride;
    float *Y;    /* 原始亮度 */
    float *tmp;  /* 水平结果 */
    float *blur; /* 垂直结果（= Yb） */
    float t;
} hfs_job_t;

static void hfs_read_luma(void *vctx, size_t y)
{
    const hfs_job_t *j = (const hfs_job_t *)vctx;
    const int W = j->w;
    float *row = j->Y + y * (size_t)W;
    int x;
    for (x = 0; x < W; x++) {
        float R, G, B;
        if (j->bits == 16) {
            const uint16_t *p = (const uint16_t *)j->vout
                              + y * (size_t)(j->stride / 2) + (size_t)x * 3;
            R = (float)p[0]; G = (float)p[1]; B = (float)p[2];
        } else {
            const uint8_t *p = (const uint8_t *)j->vout
                             + y * (size_t)j->stride + (size_t)x * 3;
            R = (float)p[0]; G = (float)p[1]; B = (float)p[2];
        }
        row[x] = 0.2126f * R + 0.7152f * G + 0.0722f * B;
    }
}

/* 水平 [1,2,1]/4：Y → tmp（边界 clamp，各行独立） */
static void hfs_hblur(void *vctx, size_t y)
{
    const hfs_job_t *j = (const hfs_job_t *)vctx;
    const int W = j->w;
    const float *s = j->Y + y * (size_t)W;
    float *d = j->tmp + y * (size_t)W;
    int x;
    for (x = 0; x < W; x++) {
        const float l = s[x > 0 ? x - 1 : 0];
        const float c = s[x];
        const float r = s[x < W - 1 ? x + 1 : W - 1];
        d[x] = (l + 2.0f * c + r) * 0.25f;
    }
}

/* 垂直 [1,2,1]/4：tmp → blur（读 tmp、写 blur，两缓冲不同，可按行并行） */
static void hfs_vblur(void *vctx, size_t y)
{
    const hfs_job_t *j = (const hfs_job_t *)vctx;
    const int W = j->w;
    const size_t H = (size_t)j->h;
    const float *rm = j->tmp + (y > 0 ? y - 1 : 0) * (size_t)W;
    const float *r0 = j->tmp + y * (size_t)W;
    const float *rp = j->tmp + (y < H - 1 ? y + 1 : H - 1) * (size_t)W;
    float *d = j->blur + y * (size_t)W;
    int x;
    for (x = 0; x < W; x++) d[x] = (rm[x] + 2.0f * r0[x] + rp[x]) * 0.25f;
}

/* 软阈值后按 k = Y'/Y 回投三通道 */
static void hfs_apply(void *vctx, size_t y)
{
    const hfs_job_t *j = (const hfs_job_t *)vctx;
    const int W = j->w;
    const float *Y = j->Y + y * (size_t)W;
    const float *Yb = j->blur + y * (size_t)W;
    const float t2 = j->t * j->t;
    const int maxv = (j->bits == 16) ? 65535 : 255;
    int x;
    for (x = 0; x < W; x++) {
        const float yv = Y[x];
        const float hp = yv - Yb[x];
        const float h2 = hp * hp;
        const float yn = Yb[x] + hp * (h2 / (h2 + t2));
        float k;
        if (yv > 0.5f) k = yn / yv; else k = 1.0f;
        if (k < 0.0f) k = 0.0f;
        if (k > 4.0f) k = 4.0f;
        if (j->bits == 16) {
            uint16_t *p = (uint16_t *)j->vout
                        + y * (size_t)(j->stride / 2) + (size_t)x * 3;
            int i;
            for (i = 0; i < 3; i++) {
                float v = (float)p[i] * k;
                p[i] = (uint16_t)(v < 0.0f ? 0.0f : (v > (float)maxv ? (float)maxv : v + 0.5f));
            }
        } else {
            uint8_t *p = (uint8_t *)j->vout + y * (size_t)j->stride + (size_t)x * 3;
            int i;
            for (i = 0; i < 3; i++) {
                float v = (float)p[i] * k;
                p[i] = (uint8_t)(v < 0.0f ? 0.0f : (v > (float)maxv ? (float)maxv : v + 0.5f));
            }
        }
    }
}

static double hf_soften_t(void)
{
    const char *e = getenv("X3F_HF_SOFTEN");
    if (e != NULL && *e != '\0') {
        double v = atof(e);
        if (v <= 0.0) return 0.0;              /* 显式关（lab 对拍用） */
        return (v < 64.0) ? v : 64.0;
    }
    /* classic 原生链不做软压缩 —— 那道 Wiener 阈值会把皮肤小尺度高频压掉，
     * 正是「质感不如 x3fuse」的主因。显式 X3F_HF_SOFTEN 仍可强制开做 A/B。 */
    if (g_classic_native) return 0.0;
    /* 默认值按机型门走：经典 Foveon 才开，其余恒 0（不跑 pass、逐字节不变）。 */
    return g_hf_classic ? 3.5 : 0.0;
}

static void apply_hf_soften(void *vout, int w, int h, int stride, int bits)
{
    const double t8 = hf_soften_t();
    hfs_job_t jb;
    long N;
    if (t8 <= 0.0 || w <= 2 || h <= 2) return;
    N = (long)w * h;
    memset(&jb, 0, sizeof(jb));
    jb.vout = vout; jb.w = w; jb.h = h; jb.bits = bits; jb.stride = stride;
    /* t 换算到当前位深口径 */
    jb.t = (float)(bits == 16 ? t8 * 257.0 : t8);
    jb.Y    = malloc((size_t)N * sizeof(float));
    jb.tmp  = malloc((size_t)N * sizeof(float));
    jb.blur = malloc((size_t)N * sizeof(float));
    if (!jb.Y || !jb.tmp || !jb.blur) {
        free(jb.Y); free(jb.tmp); free(jb.blur);
        return;
    }
    x3f_parallel_for((size_t)h, hfs_read_luma, &jb);
    x3f_parallel_for((size_t)h, hfs_hblur, &jb);
    x3f_parallel_for((size_t)h, hfs_vblur, &jb);
    x3f_parallel_for((size_t)h, hfs_apply, &jb);
    free(jb.Y); free(jb.tmp); free(jb.blur);
}

/* 线性域的「SDQ 高光收色」—— 把 X3Lumina 的**显示域公式**原样搬到
 * look_defer 的出口（apply_row / apply_common 的显示域阶段在 defer 下整段不跑，
 * 所以 sdq_hl 在 Develop DAG 的馈送里原本一个像素也动不了）。
 *
 * 判据与权重完全照抄原版（x3f_render.c 的 hl_desat + sdq_neut_w，
 * Views.swift 的 SDQ 高光滑杆）：
 *
 *     Y 够高（显示 Y ≥ 0.58 起、0.84 全量）
 *     且不是浓彩色（显示色差 dmx−dmn < 0.40 起收、0.55 以上排除）
 *     两路取并集：上面这条显示判据，加上 RAW 饱和分那条（clip 0.40→1.20 渐入，
 *     与显示亮度无关 —— 原版 hl_desat 的主路，两个通道顶到饱和就收）
 *     目标 = 显示 Y；判「确定是白」时改用最亮通道（否则只停在 B 掉下来那档）
 *
 * 唯一的改写是"显示值"的来源：defer 出口还没有 tone 曲线，于是用
 *     e = srgb_encode(clamp(lin / X3F_OE_LIN_WHITE, 0, 1))
 * 当显示域的代理 —— X3F_OE_LIN_WHITE 就是传感器饱和（= 出图白点），所以
 * e 与 16 位出口的码值同量级同形状。**上一版把判据直接算在场景线性上，
 * 色差门槛 0.40/0.55 是显示域的数：线性域里一个 (233,228,170) 的淡黄相对
 * 色差就有 0.5，于是被"浓彩色"项全部挡掉 —— 那正是滑杆"怎么拖都没反应"的
 * 原因。**
 *
 * 落回线性时朝"同显示亮度的中性值"插值：w=1 时三通道严格相等（真收白），
 * 通道值不低于 0，也不引入任何裁剪 —— 关掉滑杆（w=0）逐位等于原输出，
 * 广色域的原生数据照旧。
 *
 * 只对 SDQ/SDQH 生效（DigitalISOGain != 1）；滑杆 0 时整块不进。 */
static void sdq_linear_desat(float *out, const float *clip, int w, int h,
                             int stride, double strength)
{
    const double kWhite = X3F_OE_LIN_WHITE;  /* 传感器饱和 = 出图白点 */
    int          y;

    if (out == NULL || w <= 0 || h <= 0 || strength <= 0.0) return;
    if (strength > 2.0) strength = 2.0;

    for (y = 0; y < h; y++) {
        float *d = (float *)((char *)out + (size_t)y * (size_t)stride);
        const float *c = clip ? clip + (size_t)y * (size_t)w : NULL;
        int x;
        for (x = 0; x < w; x++) {
            float *px = d + (size_t)x * 3;
            double r = px[0], g = px[1], b = px[2];
            double yl = 0.2126 * r + 0.7152 * g + 0.0722 * b;
            double e0, e1, e2, dmn, dmx, ye, wd, ww, wc, cs, t, nlin;
            int k;

            /* ① RAW 饱和分那一路（原版的主路）：两个通道顶到传感器上限就
             * 说明这个高光的颜色不可信，按饱和分在 0.40~1.20 之间渐入收色，
             * 与显示亮度无关 —— 所以压曝光把层次拉出来以后它照样在。门槛与
             * 原版 X3F_HL_DESAT_CLIP_LO/HI 一致。 */
            cs = (c != NULL) ? (double)c[x] : -1.0;
            wc = (cs >= 0.0) ? sstep01(X3F_HL_DESAT_CLIP_LO,
                                       X3F_HL_DESAT_CLIP_HI, cs) * strength : 0.0;
            if (wc > 1.0) wc = 1.0;

            /* ② 显示域判据那一路只在显示亮度够高时才可能非零（Y 0.58 起，
             * s > 1 时门槛下移到 0.48）。两路都够不到就直接跳过，省掉 pow。 */
            if (yl < 0.24 && wc <= 0.001) continue;

            e0 = srgb_encode(r / kWhite > 1.0 ? 1.0 : (r > 0.0 ? r / kWhite : 0.0));
            e1 = srgb_encode(g / kWhite > 1.0 ? 1.0 : (g > 0.0 ? g / kWhite : 0.0));
            e2 = srgb_encode(b / kWhite > 1.0 ? 1.0 : (b > 0.0 ? b / kWhite : 0.0));

            dmn = dmx = e0;
            if (e1 < dmn) dmn = e1;
            if (e1 > dmx) dmx = e1;
            if (e2 < dmn) dmn = e2;
            if (e2 > dmx) dmx = e2;

            ye = 0.2126 * e0 + 0.7152 * e1 + 0.0722 * e2;
            wd = sdq_weight_at(ye, dmx - dmn, strength);

            ww = (wc > wd) ? wc : wd;
            if (ww <= 0.001) continue;

            /* 目标：先取显示亮度（= 去饱和到同亮度的中性），"确定是白"的点
             * 改成最亮通道 —— 否则 B 会停在掉下来的那一档，看着还是淡黄。 */
            t = ye;
            if (wd > 0.5) {
                if (dmx > t) t = dmx;
            } else if (cs >= 0.0) {
                double u = sstep01(1.6, 2.4, cs);
                if (u > 0.0) t += (dmx - t) * u;
            }

            nlin = srgb_decode(t) * kWhite;
            for (k = 0; k < 3; k++) px[k] = (float)(px[k] + (nlin - px[k]) * ww);
        }
    }
}

static int apply_ex_common(const x3f_edit_params_t *p,
                           const float *lin, const float *clip,
                           int w, int h, void *out, int stride, int bits)
{
    uint16_t lut[X3F_TONE_LUT_SIZE + 1];
    /* 「机内直出对齐」逐通道 LUT（X3F_ALIGN_LOOK=0 时不用，见 fold_align_look） */
    uint16_t alut[3][X3F_TONE_LUT_SIZE + 1];
    const uint16_t *alookg = NULL, *alookb = NULL;
    const float *p_wb_gain;
    /* 白平衡增益的**存储**必须在函数作用域上 —— 见下面那一段的注释：
     * 指针要活到 5472 / 5500 两处 apply_common 调用，块内的数组做不到。 */
    float wb_gain[3];
    float *fillgain = NULL;
    /*黑白（ B&W 页签）：只有开关打开时才把 mono_ctx_t 传给 apply_common，
     * 关掉时传 NULL —— 保证"彩色 = 与加这个功能之前逐字节相同"。 */
    mono_ctx_t mono_buf;
    const mono_ctx_t *mono = NULL;
    /* 注意这里是 **double**，不是 int —— 写成 int 的话 0.5 会被截成 0，
     * 于是「色度杂讯」除了正好拉到 1.0 以外**整条滑块都是死的**
     * （2026-10-14 标定 Quattro 降噪时抓到的第二个真 bug：p->nr_chroma=0.9
     *  与 0 逐字节相同，只有 1.0 会跑）。 */
    double nr_chroma_use;
    /* 分段计时（X3F_APPLY_TIME=1）：t_stage = 上一打点时刻 */
    double t_stage = apply_t_on() ? apply_t_now() : 0.0;
#define APPLY_TAP(label) do { \
    if (apply_t_on()) { \
        double _t = apply_t_now(); \
        fprintf(stderr, "[apply] %-10s %.3fs (%dx%d) footprint=%.0f/%.0fMB\n", label, _t - t_stage, w, h, apply_t_footprint_mb(), apply_t_peak_mb()); \
        t_stage = _t; \
    } } while (0)

    if (p == NULL) return X3FRENDER_ERR_PARAM;
    g_hl_recover_cur = p->hl_recover;

    /* 观感延后（2026-11-05，最小验证用）。打开时 apply_common 只做到线性
     * sRGB 就写像素，本函数后面所有**显示域**后处理（色度中值、色度低通、
     * 空间增益、解码 unsharp、辉光）全部跳过 —— 它们的前提是"数据已经是
     * 调好色的显示值"，在线性数据上施加等于在错误的域里做。
     *
     * 默认 0：所有分支的 `!look_defer` 都成立，逐位零回归。 */
    const int look_defer = (p->look_defer != 0);

    /* mode 8：SPP spec 直通。2026-10-24 起不再整链旁路 —— 全部滑杆接入：
     *
     *   线性域前段（与正常链同源同序）：NR → 填充光 → 细节，然后
     *   WB 细调 × 曝光 × 色彩模式差矩阵折进 replay 输入端；
     *   replay 行内（显示域）：高光恢复 → 对比度 → 色彩模式色调差；
     *   显示域后段：黑白 / 饱和度 / 高光去饱和走 apply_common（恒等 LUT），
     *   相机色彩匹配被 replay 的拟合吸收，在这里关掉防双重；
     *   尾段：色度中值 / spatial（阴影、高光、锐度）/ 8 位 Bayer 抖动，
     *   与正常链同一批函数。
     *
     * 滑杆全零（且无环境变量实验项）时走直写老路，与加后段之前逐字节一致。 */
    /* 诊断：X3F_OE_SPEC_DUMP=路径 把进 apply 的 lin 原样倒出来
     * （float32 interleaved，w×h×3，显示方向），供离线对质/重拟合。
     * 【2026-10-24】从 mode8 分支内挪到分支前 —— 两条渲染路共用同一
     * 份 lin 输入，dump 不该只在 mode8 路触发（排查暗部粉纱时发现
     * 默认路径的解码 NR 状态可能与 m8 运行不同，需要抓默认路的 lin）。 */
    {
        const char *ed = getenv("X3F_OE_SPEC_DUMP");
        if (ed != NULL && *ed != '\0') {
            FILE *fd = fopen(ed, "wb");
            if (fd != NULL) {
                fwrite(lin, sizeof(float), (size_t)w * (size_t)h * 3, fd);
                fclose(fd);
                fprintf(stderr, "[oe8] dump lin %dx%d -> %s\n", w, h, ed);
            }
        }
    }

    /* 诊断：X3F_CLIP_DUMP=路径 倒出 RAW 逐通道饱和分平面（float32 w×h，
     * 0..3，无 header）。给「裁剪掩膜区域中和」定标用：查某处红边的 cs
     * 分布，判断该处到底有没有进掩膜（cs≥1）还是只有部分通道顶到。 */
    if (clip != NULL) {
        const char *ecd = getenv("X3F_CLIP_DUMP");
        if (ecd != NULL && *ecd != '\0') {
            FILE *fc = fopen(ecd, "wb");
            if (fc != NULL) {
                fwrite(clip, sizeof(float), (size_t)w * (size_t)h, fc);
                fclose(fc);
                fprintf(stderr, "[clip] dump %dx%d -> %s\n", w, h, ecd);
            }
        }
    }

    /* ---- 帧级过曝门控（2026-10-25，DP2Q2443 黄墙断裂） ----
     * 两套表的 T_c[R] 肩部都是 2501 火核单场景拟合的硬钳制段（fuse：
     * 输入 16217..27885 → 输出钉 ~16399；own 同构）。正常曝光帧的
     * 中间调（DP2Q2443 黄墙）横跨该段时 R 通道被钳平，渐变断裂成
     * 等高线、色相跑飞。表泛化未验证，先按 SPP A4 同思路做内容门控：
     * 过曝像素占比低于阈值 → 本帧没有可矫正的过曝，跳过 mode8 重放，
     * 走默认链。
     * gate_lin 只在显式设 X3F_OE_GATE_LIN 时生效（A/B 兼容）；默认走
     * 下方统计段里的双带判据（2026-10-30 云边缘粉破案，见该处注释）。
     * 历史：fuse 域平台 ~1.5833（单阈值 1.55 在 fuse 域仍成立：
     * 2501=1.36%/2443=0.032%）；own 域旧单阈值 2.4 恰卡在裁剪平台顶端，
     * 使 mode8 对所有非重度过曝帧（含 1224）失效——寺庙天空粉由此来。 */
    /* 【两段式 2026-11-10】look_defer=1 是「引擎前置」的第一段：它只该
     * 取**场景线性**交给胶片引擎，不能在这里做 mode8 的 SPP Stage2 直通
     * 重放 —— 重放输出的是 sRGB 显示码值，被 applySpektra16 当成线性
     * 场景光吃进去，引擎 auto_exposure 与印相曲线整个跑飞（实测同域下
     * 开胶片引擎比不开亮约 30%，且颜色被洗白）。
     *
     * 解码层的 mode8 口径（om==8/9 时跳过高光中和）不受影响：那里直接
     * 读 oe_neut_mode()，与本变量无关。第二段 look_defer=0 仍照常重放，
     * 所以「胶片引擎关 + mode8」的 SPP 直出观感逐位不变。 */
    int oe8_active = (oe_neut_mode() == 8 || oe_neut_mode() == 9) && !look_defer;
    int oe9_active = (oe_neut_mode() == 9) && !look_defer;
    /* 门控否决标志（2026-10-31 步骤二）：否决 = 本帧 mode8 重放不跑，
     * 而解码段又因 om==8/9 跳过了高光中和 → 高光零处理，见下面默认链
     * hl_desat 处补的区域中和。 */
    int oe8_veto = 0;
    if (oe8_active) {
        int used_fuse = own_dispatch_used_fuse();
        /* 默认单阈值分域：fuse 域平台 1.5833 → 1.55（原标定）；own 域的
         * 单阈值仅作 env 显式覆盖时的 A/B 入口（默认走双带判据）。 */
        double gate_lin = used_fuse ? 1.55 : 2.4, gate_frac = 0.003;
        const char *gl = getenv("X3F_OE_GATE_LIN");
        const char *gf = getenv("X3F_OE_GATE_FRAC");
        const char *gd = getenv("X3F_OE_GATE");
        if (gl != NULL && *gl != '\0') gate_lin = atof(gl);
        if (gf != NULL && *gf != '\0') gate_frac = atof(gf);
        if (gd == NULL || (gd[0] != '\0' && gd[0] != '0')) {
            /* X3F_OE_GATE=0 强制关掉门控（实验）。⚠️ 原写法 `*gd != '\0'` 把 "0"
             * 也当「开」—— 设了 =0 门照跑（2026-10-30 云边缘粉排查时实测踩坑，
             * 折腾了一轮才靠 X3F_OE_GATE_LIN=0 降阈值绕过）。 */
            /* 统计必须用 pre-fix lin（prefix_lin_input）：fix 拉亮天空会把
             * 占比推过阈值、让 mode8 整帧开关随机翻转 → 全帧色调重放、
             * 渐变区按亮度分带（寺庙/麻园站条带破案，见快照处注释）。
             * 选源+扫描全程持锁：glin 指向共享快照，扫描期间不能被其他
             * 解码线程 free/重写（2026-10-30 SIGSEGV in memmove 修复）。
             *
             * ★ 判据（2026-10-30 云边缘粉破案）：own 域快照域实测
             * （X3F_PREFIX_DUMP，三帧 0EV，2026-10-30 20:30）——
             *   1224 天空：准裁剪堆 [2.0,2.2)=4.10%，≥2.2 断崖（0.041%）
             *   2443 黄墙：渐变 [2.0,2.4) 稳定 ~1.3-1.4%，≥2.4 断崖
             *   2501 火核：真饱和连续延伸，≥2.4=1.03%
             * 单阈值原理上分不开 1224（要过门修粉）与 2443（要跳过保墙）：
             * 低阈值三帧全过（2443 断裂回归），高阈值 1224 永远跳过。
             * 双带判据按「分布形状」分：低位准裁剪带 [2.0,2.2) 堆积
             * （软钳制生效=SPP 会做高光中性化的帧）或高位真饱和 ≥2.4
             * （硬裁剪）任一 ≥gate_frac 即过门。三帧实测：
             *   1224: 带 4.10% ✓过 | 2443: 带 0.088% + 0.0006% ✗跳 |
             *   2501: 带 0.069% + 1.03% ✓过（零滑杆 md5 铁律保持）。
             * 设 X3F_OE_GATE_LIN 强制退回旧单阈值行为（A/B 兼容）。 */
            size_t npix = (size_t)w * (size_t)h, clipn = 0, band_lo = 0, band_hi = 0, i;
            double frac;
            prefix_mtx_lock(&g_fill_prefix_mtx);
            const float *glin = prefix_lin_input(lin, w, h);
            if (getenv("X3F_OE_GATE_DEBUG"))
                fprintf(stderr, "[oe8] gate lin=%dx%d 快照=%s(%dx%d)\n",
                        w, h,
                        g_fill_prefix_lin != NULL ? "有" : "无",
                        g_fill_prefix_w, g_fill_prefix_h);
            for (i = 0; i < npix; i++) {
                const float *p3 = glin + i * 3;
                float m = p3[0] > p3[1] ? (p3[0] > p3[2] ? p3[0] : p3[2])
                                        : (p3[1] > p3[2] ? p3[1] : p3[2]);
                if ((double)m >= gate_lin) clipn++;
                if (m >= 2.0f && m < 2.2f) band_lo++;
                else if (m >= 2.4f) band_hi++;
            }
            prefix_mtx_unlock(&g_fill_prefix_mtx);
            frac = (double)clipn / (double)npix;
            if (gl != NULL || used_fuse) {
                /* 显式单阈值（env 覆盖，或 fuse 域回退帧——fuse 平台 1.5833，
                 * 双带常量 2.0/2.2/2.4 是 own 域标定，对 fuse 域全 0 误跳）：
                 * 只看 ≥gate_lin 占比 */
                if (frac < gate_frac) {
                    oe8_active = 0;
                    oe8_veto = 1;
                    if (getenv("X3F_OE_SEL_DEBUG") || getenv("X3F_OE_GATE_DEBUG"))
                        fprintf(stderr, "[oe8] 门控：过曝占比 %.4f%% < %.2f%%，本帧跳过 mode8 (%s)\n",
                                frac * 100.0, gate_frac * 100.0, used_fuse ? "fuse" : "own");
                } else if (getenv("X3F_OE_GATE_DEBUG")) {
                    fprintf(stderr, "[oe8] 门控放行：过曝占比 %.4f%% >= %.2f%%\n",
                            frac * 100.0, gate_frac * 100.0);
                }
            } else {
                /* 默认双带判据 */
                double f_lo = (double)band_lo / (double)npix;
                double f_hi = (double)band_hi / (double)npix;
                if (f_lo < gate_frac && f_hi < gate_frac) {
                    oe8_active = 0;
                    oe8_veto = 1;
                    if (getenv("X3F_OE_SEL_DEBUG") || getenv("X3F_OE_GATE_DEBUG"))
                        fprintf(stderr, "[oe8] 门控：准裁剪带 [2.0,2.2)=%.4f%% 真饱和≥2.4=%.4f%% 均低于 %.2f%%，本帧跳过 mode8 (%s)\n",
                                f_lo * 100.0, f_hi * 100.0, gate_frac * 100.0,
                                used_fuse ? "fuse" : "own");
                } else if (getenv("X3F_OE_GATE_DEBUG")) {
                    fprintf(stderr, "[oe8] 门控放行：准裁剪带 [2.0,2.2)=%.4f%% 真饱和≥2.4=%.4f%%\n",
                            f_lo * 100.0, f_hi * 100.0);
                }
            }
        }
    }

    if (oe8_active) {
        const float *src8 = lin;
        float *nrwork8 = NULL;
        float *dwork8 = NULL;
        float *fillgain8 = NULL;
        /* mode9：lin 裁剪区高光重建 v1（§13.13）—— 必须在 NR/细节之前
         *（对应 SPP 里展开发生在 Stage2 之前）。失败回落 mode8。 */
        float *hl9buf = NULL;
        if (oe9_active) {
            static const char *sdp = NULL;
            static int sdp_init = 0;
            if (!sdp_init) { sdp = getenv("X3F_OE9_S_DUMP"); sdp_init = 1; }
            if (sdp != NULL && *sdp != '\0' && g_hl9_sdump == NULL) {
                g_hl9_sdump = (float *)malloc((size_t)w * (size_t)h * 3 * sizeof(float));
                g_hl9_sdump_path = sdp;
            }
            hl9buf = hl9_restore(lin, w, h);
            if (hl9buf != NULL) {
                src8 = hl9buf;
                if (getenv("X3F_OE_SEL_DEBUG"))
                    fprintf(stderr, "[oe9] hl_restore_v1 applied %dx%d (%s)\n",
                            w, h, own_dispatch_used_fuse() ? "fuse" : "own");
                /* OECVer3 机制移植（X3F_OEC3=1，own 路径）：主图走
                 * clip(lin*K) 裁剪世界、est 降级为 thumb 增益源、投票回注。
                 * 失败/未启用保持 est 直通（v8b 行为）。 */
                {
                    const char *oc3 = getenv("X3F_OEC3");
                    if (oc3 != NULL && *oc3 != '\0' && *oc3 != '0'
                        && !own_dispatch_used_fuse()) {
                        float *oec3buf = x3f_oec3_envelope(lin, hl9buf, w, h);
                        if (oec3buf != NULL) {
                            free(hl9buf);
                            hl9buf = oec3buf;
                            src8 = oec3buf;
                            if (getenv("X3F_OE_SEL_DEBUG"))
                                fprintf(stderr, "[oe9] OECVer3 envelope applied\n");
                        }
                    }
                }
                /* 诊断：X3F_OE9_DUMP=路径 倒出重建后的 lin（与
                 * X3F_OE_SPEC_DUMP 的原始 lin 对质/重拟合 mode9 表用） */
                {
                    const char *ed9 = getenv("X3F_OE9_DUMP");
                    if (ed9 != NULL && *ed9 != '\0') {
                        FILE *fd9 = fopen(ed9, "wb");
                        if (fd9 != NULL) {
                            fwrite(hl9buf, sizeof(float), (size_t)w * h * 3, fd9);
                            fclose(fd9);
                            fprintf(stderr, "[oe9] dump restored lin -> %s\n", ed9);
                        }
                    }
                }
            } else {
                fprintf(stderr, "[oe9] hl_restore alloc failed, fallback mode8\n");
            }
        } else if (hl9_odm8()) {
            /* mode8 基线 + 输出域 Δ（X3F_HL9_ODM8=1）：只算特征场与 od 场
             * （fields-only，返回 NULL 不动 src8），lin 与 mode8 完全一致，
             * 特征与 v3 dump 口径相同（NR 前）。 */
            (void)hl9_restore(lin, w, h);
        }
        /* ⚠️ wb8 必须活到 x3f_parallel_for 之后 —— 放在本分支作用域。
         * 2026-10-23 第一版把它写在行内块里，块一结束 c8.gain 就是悬垂
         * 指针，读回来的是栈垃圾：用户拖一下白平衡整幅变成极端蓝
         * （与正常链 wb_gain[] 的历史事故同款，见下）。 */
        float wb8[3];
        int use_gain8 = 0;
        double cmd[9];
        int has_cmd = 0;
        double hl_desat8 = g_hl_desat;
        {
            const char *e = getenv("X3F_HL_DESAT");
            if (e != NULL && *e != '\0') hl_desat8 = atof(e);
        }
        /* SDQ 高光收色：与 hl_desat 相乘叠加，两者都 0 时结果恒 0。 */
        if (g_sdq_hl > 0.0 && sdq_dig_active())
            /* g_sdq_hl 可到 2（滑杆上限）：并集公式只认 0..1，夹一下。 */
            { double sg = g_sdq_hl > 1.0 ? 1.0 : g_sdq_hl;
              hl_desat8 = 1.0 - (1.0 - hl_desat8) * (1.0 - sg); }
        /* 色度中值（2026-10-24 接入）：原设计把 mode8 的色度降噪交给
         * nr_chroma → nr_apply（"① 已接"），但 nr_apply 需要 CAMF 噪声模型
         * 而 **Quattro 的文件里没有**（见 nr_apply 开头注释）——三个降噪滑杆
         * 在 DP2Q 上静默无效（实测 nrchroma 0.5/1/2/4 输出 MD5 全等），且
         * 6563 实测暗部色度杂讯是内嵌 JPEG 的 ~2.9 倍。分支头注释写的
         * "尾段：色度中值 / spatial" 此前从未兑现，现在与正常链同一批函数：
         * 显示域色度中值（chrmed_radius()，ISO 自适应半径），由色度降噪滑杆
         * **门控**——滑杆为 0 时不调用，零滑杆输出与接入前逐字节一致。
         * 端到端拟合吸收的是 **系统性**色度处理；残余的**随机**暗部
         * 色噪拟合吸收不了，这正是滑杆该干的活。 */
        const int cmr8 = (p->mono || (p->mono ? 0.0 : p->nr_chroma) <= 0.0)
            ? 0 : chrmed_radius();
        /* 暗部色度低通（第二级）：与正常链同策略，滑杆门控 */
        const int clpb8 = (p->mono || (p->mono ? 0.0 : p->nr_chroma) <= 0.0)
            ? 0 : chroma_shlp_box(p->nr_chroma);
        const int spat8 = (x3f_detail_level(p->detail) > 0) || p->sharpness > 0
            || p->shadow != 0.0 || p->highlight != 0.0;
        /* 显示域后段是否需要 apply_common（恒等 LUT）：
         * 饱和度 / 黑白 / 高光去饱和 / 色彩分离。相机色彩匹配（g_color_match）被
         * replay 的端到端拟合吸收，不重复做。 */
        const int split_on = (p->split_sh_sat > 0.0 || p->split_hl_sat > 0.0);
        const int disp_path = ((p->saturation != 0.0 && !p->mono) || p->mono
            || hl_desat8 > 0.0 || split_on);
        /* 8 位输出且有后段/尾段 → 中间走 16 位（与正常链同一理由：
         * 在量化到 8 位的底图上做 unsharp 会把量化台阶一起放大；
         * 色度中值同理 —— 与正常链一样在 16 位中间上做）。 */
        const int use_tmp16 = (bits == 8) && (disp_path || spat8 || cmr8 > 0);
        g_detail_done_linear = 0;

        /* ---- ① NR（线性域，与正常链同一实现）---- */
        int nr_base8 = 0;
        double nr_gk8 = 0.0, nr_glo8 = 0.0, nr_ghi8 = 0.0;
        const double nr_luma_eff8 = nr_luma_baseline(p->nr_luma, p, &nr_base8);
        const int nr_gate8 = nr_base8
            && nr_gate_cfg(p, &nr_gk8, &nr_glo8, &nr_ghi8);
        if (nr_luma_eff8 > 0.0 || (p->mono ? 0.0 : p->nr_chroma) > 0.0
            || p->nr_banding > 0.0) {
            float *nw = NULL;
            if (nr_apply(nr_luma_eff8, p->mono ? 0.0 : p->nr_chroma,
                         p->nr_banding, src8, &nw, w, h,
                         nr_gate8, nr_gk8, nr_glo8, nr_ghi8)) {
                nrwork8 = nw;
                src8 = nw;
            }
        }
        /* ---- ② 填充光（线性域乘性增益图）---- */
        if (p->fill_light != 0.0) {
            const uint64_t fp8 = apply_cache_on()
                ? apply_fp(src8, (size_t)w * h * 3) : 0;
            fillgain8 = build_fill_gain(src8, w, h, p->fill_light, p->exposure,
                                        p->fl_tune, p->fl_hlzred,
                                        p->fl_lmean_r, p->fl_msat,
                                        p->fl_mapgain, p->fl_k0off,
                                        p->fl_zgain, fp8);
            /* [诊断 2026-11-01] X3F_FILLGAIN8_DUMP=path：oe8 块自建 fill 增益
             * （与默认链 X3F_FL_DUMP 那份域不同）落盘，查 4019 暗墙列带。 */
            {
                const char *fg8p = getenv("X3F_FILLGAIN8_DUMP");
                if (fg8p != NULL && *fg8p != '\0') {
                    FILE *ffg = fopen(fg8p, "wb");
                    if (ffg != NULL) {
                        fwrite(fillgain8, sizeof(float), (size_t)w * h, ffg);
                        fclose(ffg);
                        fprintf(stderr, "[oe8] fillgain8 -> %s\n", fg8p);
                    }
                }
            }
        }
        /* ---- ③ 细节（线性域，与正常链同一实现；基准 = 当前 src8）----
         * 与正常链的细节段同构（保边低通 + 乘性 boost），改动需两边同步。 */
        {
            const int dlv = x3f_detail_level(p->detail);
            if (dlv > 0) {
                const size_t nn = (size_t)w * h;
                float *ylin = (float *)malloc(nn * sizeof(float));
                float *rlin = (float *)malloc(nn * sizeof(float));
                if (ylin != NULL && rlin != NULL) {
                    const double g = x3f_detail_gain(x3f_detail_R(dlv));
                    const double eps = detail_env("X3F_DETAIL_EPS", 5.0 / 65535.0);
                    const double nnk = detail_env("X3F_DETAIL_NNK", 0.044)
                                       * x3f_detail_nnk_scale();
                    const int shape = (int)detail_env("X3F_DETAIL_NNSHAPE", 1.0);
                    const int phase = (int)detail_env("X3F_DETAIL_PHASE", 1.0) != 0;
                    const char *mf = getenv("X3F_DETAIL_MFORM");
                    const int mform = (mf != NULL && mf[0] == 's') ? 1 : 0;
                    const float *dsrc = src8;
                    float *base_luma = NULL;
                    int nr_on, nr_r; double nr_s;
                    x3f_detail_nr_params(&nr_on, &nr_r, &nr_s);
                    size_t i;
                    const uint64_t dfp =
                        (dsrc == lin && apply_cache_on())
                            ? apply_fp(lin, nn * 3)
                            : (apply_cache_on() ? apply_fp(dsrc, nn * 3) : 0);
                    const uint64_t dk3 =
                        ((uint64_t)(unsigned)shape << 32) ^ (uint64_t)phase;
                    const int det_hit =
                        (apply_cache_on() && dfp != 0 && nn <= APPLY_CACHE_MAX_PIX)
                            ? apply_slot_get(&g_det_slot, &g_det_lock,
                                             dfp, apply_fp_d(nnk), dk3,
                                             w, h, nn, rlin, NULL, NULL, NULL)
                            : 0;
                    for (i = 0; i < nn; i++)
                        ylin[i] = (float)(0.2126 * dsrc[i * 3]
                                        + 0.7152 * dsrc[i * 3 + 1]
                                        + 0.0722 * dsrc[i * 3 + 2]);
                    if (nr_on) {
                        /* 降噪基底 + 降噪引导：restored 吃干净图，残差不裹噪声。
                         * 这条路径不走缓存（rlin 依赖 NR 参数，与缓存键不一致）。 */
                        x3f_detail_substrate(dsrc, ylin, rlin, &base_luma,
                                             w, h, nnk, shape, phase,
                                             nr_on, nr_r, nr_s);
                    } else if (!det_hit) {
                        x3f_gdf_lowpass_lin(dsrc, ylin, rlin, w, h, nnk, shape, phase);
                        if (apply_cache_on() && dfp != 0 && nn <= APPLY_CACHE_MAX_PIX)
                            apply_slot_put(&g_det_slot, &g_det_lock,
                                           dfp, apply_fp_d(nnk), dk3, w, h, nn, rlin,
                                           0, 0, 0);
                    }   /* else: det_hit → rlin 已从缓存取，NR 关逐位一致 */
                    dwork8 = x3f_detail_linear_boost(dsrc, rlin, w, h, g, eps, mform,
                                                     p->hl_soften, clip, base_luma);
                    if (dwork8 != NULL) {
                        src8 = dwork8;
                        g_detail_done_linear = 1;
                    }
                    free(ylin); free(rlin); free(base_luma);
                }
            }
        }
        /* ---- ④ WB 细调 × 曝光：折进 replay 输入端（SPP 里都在过曝矫正
         *      之前）。滑杆全零不乘，逐位不变。 ---- */
        if (p->wb_temp != 0.0 || p->wb_tint != 0.0 || p->exposure != 0.0
            || p->wb_red != 1.0 || p->wb_green != 1.0 || p->wb_blue != 1.0) {
            wb_gain_calc(p, wb8);
            if (p->exposure != 0.0) {
                const double eg = pow(2.0, p->exposure);
                wb8[0] = (float)(wb8[0] * eg);
                wb8[1] = (float)(wb8[1] * eg);
                wb8[2] = (float)(wb8[2] * eg);
            }
            use_gain8 = 1;
        }
        /* ---- ④b 实验（2026-10-31，X3F_OE8_WB="r,g,b"，默认不设 = 不变）----
         * 用户反馈「mode8 开启后整体偏青绿」。实测同一帧中性参照（显示域均值）：
         *   白曲墙 G−R：默认链 +5.95 → mode8 +9.08（+3.1）
         *   灰牌匾 G−R：默认链 +13.9 → mode8 +18.1（+4.2）
         * 逐通道 1D 重放表做不出矩阵，拟合时 G 通道整体抬了一点 —— 中性轴
         * 偏绿。这里给 m8 输入端一个三通道微调（与 WB 细调同一位置、同样
         * 折进 linear），用来把中性轴拉回默认链水平；表本身不动。
         * 标定见 outputs 报告：G≈0.96~0.97 可消掉 +3 的绿。 */
        {
            const char *e8wb = getenv("X3F_OE8_WB");
            if (e8wb != NULL && *e8wb != '\0') {
                double t[3] = { 1.0, 1.0, 1.0 };
                if (sscanf(e8wb, "%lf,%lf,%lf", &t[0], &t[1], &t[2]) == 3
                    && (t[0] != 1.0 || t[1] != 1.0 || t[2] != 1.0)) {
                    int k;
                    if (!use_gain8) wb_gain_calc(p, wb8);
                    for (k = 0; k < 3; k++) wb8[k] = (float)(wb8[k] * t[k]);
                    use_gain8 = 1;
                    if (getenv("X3F_OE_SEL_DEBUG"))
                        fprintf(stderr, "[oe8] WB 微调 %.3f/%.3f/%.3f\n",
                                t[0], t[1], t[2]);
                }
            }
        }
        /* ---- ⑤ 色彩模式差矩阵：直通的拟合吸收了 Standard 档 ——
         *      非 Standard 档按 M_当前 · M_Standard⁻¹ 折进线性段。 ---- */
        if (!p->color.unit && g_cm_standard_has) {
            double inv[9];
            if (oe8_inv3x3(g_cm_standard_m, inv)) {
                cm3_mul(p->color.m, inv, cmd);
                if (!cm3_is_unit(cmd)) has_cmd = 1;
            }
        }
        {
            oe_spec_ctx_t c8;
            const double rk = X3F_HL_RECOVER_KNEE;
            int tail_chrmed_spat;
            /* 本次解码真实后端 → 选 mode8 表（own/fuse 两套 lin 域各自标定）。
             * 必须在 x3f_parallel_for 之前、渲染同线程调用（见 g_oespec 注）。
             * mode9：重建成功且 own 域 → 换 m9 专属表（fuse 域未拟合，回落）。 */
            oe_spec_set_cur(own_dispatch_used_fuse());
            if (oe9_active && hl9buf != NULL && !own_dispatch_used_fuse())
                g_oespec = &oe_spec_tabs_m9;
            c8.lin = src8;
            /* [诊断] X3F_OE8_SRC_DUMP=路径：倒出真正进 oe 链的 src8（NR/填充光
             * 之后），离线重放必须以它为准——用重建后的 lin 会漏掉默认降噪。 */
            {
                static const char *sp8 = NULL;
                static int sp8_init = 0;
                if (!sp8_init) { sp8 = getenv("X3F_OE8_SRC_DUMP"); sp8_init = 1; }
                if (sp8 != NULL && *sp8 != '\0') {
                    FILE *f8 = fopen(sp8, "wb");
                    if (f8 != NULL) {
                        fwrite(src8, sizeof(float), (size_t)w * (size_t)h * 3, f8);
                        fclose(f8);
                        fprintf(stderr, "[oe8] dump src8(进链) -> %s\n", sp8);
                        sp8 = NULL;
                    }
                }
            }
            c8.out = NULL;
            c8.disp = NULL;
            c8.w = w; c8.stride = stride; c8.bits = bits;
            /* 只有直写路（c8.out != NULL）用它；见 oe_spec_row 的 put 段。
             * 未知档位一律按 sRGB 处理，与 apply_ex_common 的 ctx 同款。 */
            c8.out_space = (p->out_space >= X3F_OUT_SRGB
                            && p->out_space <= X3F_OUT_DCI_P3)
                ? p->out_space : X3F_OUT_SRGB;
            c8.gain = use_gain8 ? wb8 : NULL;
            c8.fillgain = fillgain8;
            c8.cm = has_cmd ? cmd : NULL;
            c8.contrast = p->contrast;
            c8.hl_recover = p->hl_recover;
            c8.rbase = srgb_encode((g_base_curve && !classic_native_strict())
                                   ? tone_curve(p, rk) : rk);
            c8.cmtone_row = p->color.tone_row;
            c8.cmtone_row_std = cmtone_row_for("Standard");
            c8.cmtone_w = p->color.tone_w;

            tail_chrmed_spat = spat8;
            if (!disp_path && !use_tmp16) {
                /* 直写路：replay → out（行内后段照做），色度中值/spatial 按需补。
                 * 滑杆全零时（cmr8==0 且无后段）= 与加尾段之前逐字节一致。 */
                c8.out = out;
                dark2_fit_init(w, h);
                x3f_parallel_for((size_t)h, oe_spec_row, &c8);
                dark2_fit_flush();
                if (cmr8 > 0)
                    apply_chroma_median(cmr8, out, w, h, stride, bits);
                if (clpb8 > 0)
                    apply_chroma_shlp(clpb8, out, w, h, stride, bits);
                if (tail_chrmed_spat) {
                    int rc8 = X3FRENDER_OK;
                    if (spat8)
                        rc8 = apply_spatial(p, out, w, h, stride, bits, clip);
                    free(nrwork8); free(dwork8); free(fillgain8); free(hl9buf);
            if (g_hl9_sdump != NULL) {
                FILE *fs = fopen(g_hl9_sdump_path, "wb");
                if (fs != NULL) {
                    fwrite(g_hl9_sdump, sizeof(float), (size_t)w * (size_t)h * 3, fs);
                    fclose(fs);
                    fprintf(stderr, "[oe9] dump s(修正后) -> %s\n", g_hl9_sdump_path);
                }
                free(g_hl9_sdump); g_hl9_sdump = NULL;
            }
            /* hl9_free_space() 已移入 hl9_restore 入口（2026-10-30 崩溃修复）：
             * 此处释放会打掉并发渲染路正在读的 od/bn 场（202838.ips）。 */
                    return rc8;
                }
                free(nrwork8); free(dwork8); free(fillgain8); free(hl9buf);
            if (g_hl9_sdump != NULL) {
                FILE *fs = fopen(g_hl9_sdump_path, "wb");
                if (fs != NULL) {
                    fwrite(g_hl9_sdump, sizeof(float), (size_t)w * (size_t)h * 3, fs);
                    fclose(fs);
                    fprintf(stderr, "[oe9] dump s(修正后) -> %s\n", g_hl9_sdump_path);
                }
                free(g_hl9_sdump); g_hl9_sdump = NULL;
            }
            hl9_free_space();
                return X3FRENDER_OK;
            }
            /* 后段路：replay → float 显示值 → apply_common（恒等 LUT）
             * → 色度中值 → spatial →（8 位）Bayer 抖动降位。 */
            {
                float *disp8 = (float *)malloc((size_t)w * h * 3 * sizeof(float));
                uint16_t *tmp16 = use_tmp16
                    ? (uint16_t *)malloc((size_t)w * h * 3 * sizeof(uint16_t)) : NULL;
                /* [诊断 2026-11-01] 后段路各阶段 dump 开关（块级作用域） */
                static const char *sdp8 = NULL;
                static int sdp8_i = 0;
                if (!sdp8_i) { sdp8 = getenv("X3F_OE8_STAGE_DUMP"); sdp8_i = 1; }
                int rc8 = (disp8 != NULL && (!use_tmp16 || tmp16 != NULL))
                    ? X3FRENDER_OK : X3FRENDER_ERR_NOMEM;
                if (rc8 == X3FRENDER_OK) {
                    void *dstB = use_tmp16 ? (void *)tmp16 : out;
                    int strideB = use_tmp16 ? w * 6 : stride;
                    const int saved_cm = g_color_match;
                    mono_ctx_t mono_buf8;
                    const mono_ctx_t *mono8 = NULL;
                    x3f_color_mode_t unit_cm8;
                    color_mode_unit(&unit_cm8);
                    if (p->mono) {
                        mono_setup(p, &mono_buf8);
                        mono8 = &mono_buf8;
                    }
                    c8.disp = disp8;
                    dark2_fit_init(w, h);
                    x3f_parallel_for((size_t)h, oe_spec_row, &c8);
                    dark2_fit_flush();
                    /* [诊断 2026-11-01] X3F_OE8_STAGE_DUMP=前缀：把后段路各阶段
                     * 产物落盘（replay 输出 disp f32；apply_common/色度中值/
                     * spatial 后的 16bit u16），用于定位 DP2Q4019 竖条纹注入段。 */
                    if (sdp8 != NULL && *sdp8 != '\0') {
                        char pp[512]; FILE *ff;
                        snprintf(pp, sizeof pp, "%s_disp.f32", sdp8);
                        ff = fopen(pp, "wb");
                        if (ff != NULL) {
                            fwrite(disp8, sizeof(float), (size_t)w * h * 3, ff);
                            fclose(ff);
                            fprintf(stderr, "[oe8] stage dump disp -> %s\n", pp);
                        }
                    }
                    /* 高光恢复在 replay 行内做过了；色彩匹配被拟合吸收 ——
                     * 都不许在 apply_common 里再来一遍。
                     * X3F_OE8_CM=1（实验，2026-10-31）：保留色彩匹配跑一遍，
                     * 用来判断「黄墙在 mode8 下偏淡」是否就是这段被关掉造成的
                     * ——m8 表是逐通道 1D，做不出矩阵对饱和黄压蓝的效果。
                     * 默认仍然关（拟合口径不变，逐位一致）。 */
                    g_hl_recover_cur = 0;
                    {
                        const char *e8cm = getenv("X3F_OE8_CM");
                        if (e8cm == NULL || e8cm[0] != '1')
                            g_color_match = 0;
                        else if (getenv("X3F_OE_SEL_DEBUG"))
                            fprintf(stderr, "[oe8] CM 保留（实验 X3F_OE8_CM=1）\n");
                    }
                    rc8 = apply_common(oe8_identity_lut(),
                                       1.0 + p->saturation * 0.5,
                                       p->saturation != 0.0 && mono8 == NULL,
                                       hl_desat8, clip, NULL, NULL, mono8,
                                       &unit_cm8, disp8, w, h,
                                       dstB, strideB, 16, NULL, NULL, p);
                    g_color_match = saved_cm;
                    /* [诊断] 阶段 dump：apply_common 后 */
                    if (sdp8 != NULL && *sdp8 != '\0') {
                        char pp[512]; FILE *ff;
                        snprintf(pp, sizeof pp, "%s_ac.u16", sdp8);
                        ff = fopen(pp, "wb");
                        if (ff != NULL) {
                            fwrite(dstB, sizeof(uint16_t), (size_t)w * h * 3, ff);
                            fclose(ff);
                        }
                    }
                    if (rc8 == X3FRENDER_OK && cmr8 > 0)
                        apply_chroma_median(cmr8, dstB, w, h, strideB, 16);
                    /* [诊断] 阶段 dump：色度中值后 */
                    if (sdp8 != NULL && *sdp8 != '\0') {
                        char pp[512]; FILE *ff;
                        snprintf(pp, sizeof pp, "%s_chrmed.u16", sdp8);
                        ff = fopen(pp, "wb");
                        if (ff != NULL) {
                            fwrite(dstB, sizeof(uint16_t), (size_t)w * h * 3, ff);
                            fclose(ff);
                        }
                    }
                    if (rc8 == X3FRENDER_OK && clpb8 > 0)
                        apply_chroma_shlp(clpb8, dstB, w, h, strideB, 16);
                    if (rc8 == X3FRENDER_OK && spat8)
                        rc8 = apply_spatial(p, dstB, w, h, strideB, 16, clip);
                    /* [诊断] 阶段 dump：spatial 后 */
                    if (sdp8 != NULL && *sdp8 != '\0') {
                        char pp[512]; FILE *ff;
                        snprintf(pp, sizeof pp, "%s_spat.u16", sdp8);
                        ff = fopen(pp, "wb");
                        if (ff != NULL) {
                            fwrite(dstB, sizeof(uint16_t), (size_t)w * h * 3, ff);
                            fclose(ff);
                            fprintf(stderr, "[oe8] stage dump ac/chrmed/spat -> %s_*\n", sdp8);
                        }
                    }
                    if (rc8 == X3FRENDER_OK && use_tmp16) {
                        /* 8 位降位：与正常链尾段同一循环（含 hl_soften 抖动） */
                        static const int bayer4[16] = {
                            0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5
                        };
                        int y;
                        for (y = 0; y < h; y++) {
                            const uint16_t *s = tmp16 + (size_t)y * w * 3;
                            uint8_t *d = (uint8_t *)out + (size_t)y * stride;
                            int x;
                            for (x = 0; x < w * 3; x++) {
                                const int dth = p->hl_soften
                                    ? (bayer4[((y & 3) << 2) | ((x / 3) & 3)] - 8) * 16
                                    : 0;
                                unsigned v = ((unsigned)s[x] + 128u + (unsigned)dth) >> 8;
                                d[x] = (uint8_t)(v > 255u ? 255u : v);
                            }
                        }
                    }
                }
                free(tmp16);
                free(disp8);
                free(nrwork8); free(dwork8); free(fillgain8); free(hl9buf);
            if (g_hl9_sdump != NULL) {
                FILE *fs = fopen(g_hl9_sdump_path, "wb");
                if (fs != NULL) {
                    fwrite(g_hl9_sdump, sizeof(float), (size_t)w * (size_t)h * 3, fs);
                    fclose(fs);
                    fprintf(stderr, "[oe9] dump s(修正后) -> %s\n", g_hl9_sdump_path);
                }
                free(g_hl9_sdump); g_hl9_sdump = NULL;
            }
            hl9_free_space();
                return rc8;
            }
        }
    }

    build_tone_lut(p, lut, -1);
    const uint16_t *main_lut = lut;
    /* align_look 是按 Quattro 参考图标定的逐通道输出码重映射（暗部 B+4/G−2
     * 这种量级）。SD1 走的是自己的实测曲线，再叠这张表等于二次塑形 —— 实测
     * SDIM4203 中性区 Δa +2.45 → +3.55、Δb −0.73 → −2.75，SD1 跳过。
     * 两段式的第二段同理跳过（p->align_look_off，见 x3f_render.h）——
     * 引擎已经给过观感，这里再叠一次是二次渲染，2026-11-11 实测偏青蓝。 */
    if (align_look_on() && !classic_skip_align_look() && !sdqh_native_display()
        && !p->align_look_off) {
        fold_align_look(lut, alut[0], alut[1], alut[2]);
        main_lut = alut[0];
        alookg = alut[1];
        alookb = alut[2];
    }
    APPLY_TAP("lut");

    if (p->mono) {
        mono_setup(p, &mono_buf);
        mono = &mono_buf;
    }

    /*黑白分支把「色度噪讯」那一格写死成 0（priSetup：
     * str xzr,[x19,#0x488]），也就是黑白下没有色度降噪可做。 */
    nr_chroma_use = p->mono ? 0.0 : p->nr_chroma;

    /* 图像内容指纹：fill 基座 / detail 低通 / NR 缓存的键基座。
     * 一次 apply 只算一遍（~3 万个采样 float，几十微秒）。 */
    const uint64_t lin_fp = apply_cache_on()
        ? apply_fp(lin, (size_t)w * (size_t)h * 3) : 0;

    /* 白平衡：色温正=更暖（加红减蓝），色调正=偏品红（加红蓝减绿） */
    {
        int use = (p->wb_temp != 0.0 || p->wb_tint != 0.0
                   || p->wb_red != 1.0 || p->wb_green != 1.0 || p->wb_blue != 1.0);
        /* 增益算在块外的 wb_gain[]（函数作用域，防悬垂指针 —— 历史事故见下）。
         * 公式收在 wb_gain_calc()，mode8 直通那条路共用同一份。 */
        wb_gain_calc(p, wb_gain);
        p_wb_gain = use ? wb_gain : NULL;
    }

    /* 饱和度 -1..+1 -> 0..2 的倍率 */
    {
        int rc;
        const float *use = lin;              /* 给填充光用（旧行为，不变） */
        const float *render_src = lin;       /* 真正进 tone curve 的那张 */
        float *nrwork = NULL;
        float *dwork = NULL;
        /* 显示域色度中值（SPP Stage2 XDoYUVMedian 的等价物，见 chrmed_row 上面）。
         * 黑白不做：R=G=B 时 U=V=0，数学上是空操作。 */
        const int cmr = p->mono ? 0 : chrmed_radius();
        /* 暗部色度低通（第二级，见 apply_chroma_shlp 头注释）：滑杆门控，
         * 0 = 不跑（与接入前逐位一致）；0.5 → b=4（σ_eff=√20≈4.5）。 */
        const int clpb = (p->mono || nr_chroma_use <= 0.0)
            ? 0 : chroma_shlp_box(nr_chroma_use);
        g_detail_done_linear = 0;
        int nr_base = 0;
        double nr_gk = 0.0, nr_glo = 0.0, nr_ghi = 0.0;
        const double nr_luma_eff = nr_luma_baseline(p->nr_luma, p, &nr_base);
        const int nr_gate = nr_base && nr_gate_cfg(p, &nr_gk, &nr_glo, &nr_ghi);
        if (nr_luma_eff > 0.0 || nr_chroma_use > 0.0 || p->nr_banding > 0.0) {
            /* ⚠️ 产物必须同时喂给 render_src。只写 use 的话，降噪只参与
             * fill light 的增益计算，**永远进不到出图** —— 2026-10-14 实测：
             * 三个滑块开满，输出与关掉时逐字节相同（mean|Δ| = 0.000）。
             * 这是这次「落地 + 验证」里抓到的第一个真 bug。 */
            if (nr_apply(nr_luma_eff, nr_chroma_use, p->nr_banding,
                         lin, &nrwork, w, h,
                         nr_gate, nr_gk, nr_glo, nr_ghi)) {
                use = nrwork;
                render_src = nrwork;
            }
        }
        APPLY_TAP("nr");
        if (p->fill_light != 0.0) {
            /* 键必须跟**实际输入**（use，可能是 NR 后的缓冲）走：
             * 用 fp(lin) 的话，改 NR 滑杆会让 fill 基座假命中旧内容。 */
            const uint64_t fill_fp = (use == lin)
                ? lin_fp
                : (apply_cache_on() ? apply_fp(use, (size_t)w * h * 3) : 0);
            fillgain = build_fill_gain(use, w, h, p->fill_light, p->exposure,
                                       p->fl_tune, p->fl_hlzred,
                                       p->fl_lmean_r, p->fl_msat,
                                       p->fl_mapgain, p->fl_k0off,
                                       p->fl_zgain, fill_fp);
        }
        APPLY_TAP("fill");

        /*细节（ Detail）：**线性域**做 —— 顺序与 SPP Stage2B 一致，
         * 残差在 tone curve 之前算。
         *   restored = 线性域保边低通（x3f_gdf_lowpass_lin，阈值 ∝ 信号）
         *   out      = 原 + (g−1)·(原 − restored)
         * g = 1 → out ≡ 原（滑块最左端不改变任何东西）。走显示域那条：
         * X3F_DETAIL_DOMAIN=display（旧实现，留着 A/B）。 */
        {
            const int dlv = x3f_detail_level(p->detail);
            const char *dom = getenv("X3F_DETAIL_DOMAIN");
            const int want_linear = (dom == NULL || dom[0] != 'd');
            if (dlv > 0 && want_linear) {
                const size_t nn = (size_t)w * h;
                float *ylin = (float *)malloc(nn * sizeof(float));
                float *rlin = (float *)malloc(nn * sizeof(float));
                if (ylin != NULL && rlin != NULL) {
                    const double g = x3f_detail_gain(x3f_detail_R(dlv));
                    const double eps = detail_env("X3F_DETAIL_EPS", 5.0 / 65535.0);
                    /* NNK = 噪声阈值倍数（相对 √v）。锚点 0.044：base2（mode3）下
                     * DP0Q2315（ISO1600）档 6 实测 Δrel = +0.105 vs SPP +0.1035；
                     * 扫过的点（g−1 = 0.30）：0.02→+0.068, 0.03→+0.086,
                     * 0.045→+0.105, 0.06→+0.120。
                     * ISO 侧再乘 x3f_detail_nnk_scale()（arg7 的 ISO 阶梯：
                     * ISO<800 时 arg7 ×2 → σ ×√2 → NNK ×1.41）。 */
                    const double nnk = detail_env("X3F_DETAIL_NNK", 0.044)
                                       * x3f_detail_nnk_scale();
                    const int shape = (int)detail_env("X3F_DETAIL_NNSHAPE", 1.0);
                    const int phase = (int)detail_env("X3F_DETAIL_PHASE", 1.0) != 0;
                    const char *mf = getenv("X3F_DETAIL_MFORM");
                    const int mform = (mf != NULL && mf[0] == 's') ? 1 : 0;
                    /* 基准取**当前链的输出**（render_src），不是原始 lin。
                     * 原来这里硬写 lin —— 于是「NR + 细节」同时开时，细节会拿
                     * 没降过噪的底图重算一遍并整个盖回去，NR 白做。
                     * 顺序是先降噪、残差/细节在其后（Stage1/2 之后再 Stage2B），
                     * 所以这里必须跟着 render_src 走。 */
                    const float *dsrc = render_src;
                    float *base_luma = NULL;
                    int nr_on, nr_r; double nr_s;
                    x3f_detail_nr_params(&nr_on, &nr_r, &nr_s);
                    size_t i;
                    /* 保边低通（rlin）不依赖 detail 滑杆——只依赖底图和
                     * 噪声阈值（nnk，ISO 定）。缓存住，拖 detail 只重跑
                     * 下面的乘性 boost。 */
                    const uint64_t dfp =
                        (dsrc == lin && lin_fp != 0)
                            ? lin_fp
                            : (apply_cache_on() ? apply_fp(dsrc, nn * 3) : 0);
                    const uint64_t dk3 =
                        ((uint64_t)(unsigned)shape << 32) ^ (uint64_t)phase;
                    const int det_hit =
                        (apply_cache_on() && dfp != 0 && nn <= APPLY_CACHE_MAX_PIX)
                            ? apply_slot_get(&g_det_slot, &g_det_lock,
                                             dfp, apply_fp_d(nnk), dk3,
                                             w, h, nn, rlin, NULL, NULL, NULL)
                            : 0;
                    for (i = 0; i < nn; i++)
                        ylin[i] = (float)(0.2126 * dsrc[i * 3] + 0.7152 * dsrc[i * 3 + 1]
                                          + 0.0722 * dsrc[i * 3 + 2]);
                    if (nr_on) {
                        /* 降噪基底 + 降噪引导：restored 吃干净图，残差不裹噪声。
                         * 这条路径不走缓存（rlin 依赖 NR 参数，与缓存键不一致）。 */
                        x3f_detail_substrate(dsrc, ylin, rlin, &base_luma,
                                             w, h, nnk, shape, phase,
                                             nr_on, nr_r, nr_s);
                    } else if (!det_hit) {
                        x3f_gdf_lowpass_lin(dsrc, ylin, rlin, w, h, nnk, shape, phase);
                        if (apply_cache_on() && dfp != 0 && nn <= APPLY_CACHE_MAX_PIX)
                            apply_slot_put(&g_det_slot, &g_det_lock,
                                           dfp, apply_fp_d(nnk), dk3, w, h, nn, rlin,
                                           0, 0, 0);
                    }   /* else: det_hit → rlin 已从缓存取，NR 关逐位一致 */
                    /* ylin 到这里已经用完（rlin / base_luma 才是 boost 的输入）。
                     * 100MP 它是 402MB，早放一步就少一段与 dwork 叠加的峰值。 */
                    free(ylin); ylin = NULL;
                    dwork = x3f_detail_linear_boost(dsrc, rlin, w, h, g, eps, mform,
                                                    p->hl_soften, clip, base_luma);
                    if (dwork != NULL) {
                        render_src = dwork;
                        g_detail_done_linear = 1;
                    }
                    if (getenv("X3F_DETAIL_DEBUG"))
                        fprintf(stderr, "[detail] 线性域 level=%d g=%.3f eps=%.6f "
                                "nnk=%.3f shape=%d phase=%d mform=%s dwork=%p\n",
                                dlv, g, eps, nnk, shape, phase,
                                mform ? "spp" : "diff", (void *)dwork);
                    free(ylin); free(rlin); free(base_luma);
                }
            }
        }
        APPLY_TAP("detail");

        /* 高光去饱和强度：A/B 用环境变量覆盖（X3F_HL_DESAT=0..1）。 */
        double hl_desat_use = g_hl_desat;
        {
            const char *e = getenv("X3F_HL_DESAT");
            if (e != NULL && *e != '\0') hl_desat_use = atof(e);
        }
        /* SDQ 高光收色：与 hl_desat 相乘叠加，两者都 0 时结果恒 0。 */
        if (g_sdq_hl > 0.0 && sdq_dig_active())
            /* g_sdq_hl 可到 2（滑杆上限）：并集公式只认 0..1，夹一下。 */
            { double sg = g_sdq_hl > 1.0 ? 1.0 : g_sdq_hl;
              hl_desat_use = 1.0 - (1.0 - hl_desat_use) * (1.0 - sg); }
        if (getenv("X3F_SDQ_DEBUG"))
            fprintf(stderr, "[sdq] apply: hl_desat_use=%.3f\n", hl_desat_use);
        /* ---- 门控否决 mode8 时的「裁剪掩膜区域中和」（2026-10-31 步骤二）----
         * 语义漏洞：om==8/9 时解码段**故意跳过**高光中和（假定 apply 级重放
         * 会做，见 load_common 里 `om != 0 && om != 8 && om != 9` 的判据）。
         * 一旦内容门控否决 mode8，两道处理就都落空——高光在这帧里零处理，
         * 屋檐下那种「准裁剪 + 通道比失真」的像素按默认链直接出红边，
         * 而 SPP 在这块是平滑的黄渐变（DP2Q2443 用户指的明暗交界）。
         *
         * 这里复用 hl_desat 那条 **clip 驱动**的去饱和路径，不再另造一张
         * 全局曲线：
         *   · 掩膜 = 解码段的 RAW 逐通道饱和分（c->clip），与显示亮度无关；
         *   · 权重 sstep01(0.40,1.20,cs)，cs≥1（有通道顶到上限）才显著；
         *     中间调黄墙 cs≈0 → 权重恒 0，一个像素都不动；
         *   · cs∈[1.6,2.4] 时额外把目标从 luma 抬到最亮通道（全通道过曝
         *     该是白的，别拉暗）。
         * 即"只中和掩膜覆盖的那一小圈"，铺开半径由 raw 决定，不需要模糊，
         * 也不会像全局曲线那样把整帧色调重放一遍（那正是 mode8 表的坏处）。
         * 强度取 max(用户滑杆, 0.6)：滑杆已经更高就不降。X3F_OE8_VETO_DESAT
         * 覆盖（=0 关闭，A/B 用）。 */
        if (oe8_veto) {
            double vd = 0.6;
            const char *ev = getenv("X3F_OE8_VETO_DESAT");
            if (ev != NULL && *ev != '\0') vd = atof(ev);
            if (vd > 0.0 && vd > hl_desat_use) {
                if (getenv("X3F_OE_SEL_DEBUG") || getenv("X3F_OE_GATE_DEBUG"))
                    fprintf(stderr, "[oe8] 门控否决补中和：hl_desat %.3f → %.3f\n",
                            hl_desat_use, vd);
                hl_desat_use = vd;
            }
        }
        /* 空间类（锐度/细节）放在最后：在显示空间上做，符合视觉感受。
         * 但它是个 unsharp —— 在**已经量化到 8 位**的底图上做，等于把量化台阶
         * 一起乘上 amt，高光那片渐变立刻变成肉眼可见的"断层"。
         * 所以 8 位输出时中间走 16 位，做完空间处理再一次性降到 8 位。
         * 实测（DP0Q1070, FL=−2, 锐度 0.70）高光"台阶≥2级"占比：
         *   8 位路径 38.3% → 16 位中间 34.5%（SPP 参照 33.7%），
         * 而纹理没变（texcmp 0.96 → 0.95）。16 位输出（TIFF）本来就没这问题。 */
        /* X3F_SPATIAL8=1 强制走老路（在 8 位底图上锐化），只给 A/B 复现用 */
        if (bits == 8 && getenv("X3F_SPATIAL8") == NULL) {
            uint16_t *tmp = (uint16_t *)malloc((size_t)w * h * 3 * sizeof(uint16_t));
            int y;
            if (tmp == NULL) {
                free(fillgain); free(nrwork); free(dwork);
                return X3FRENDER_ERR_NOMEM;
            }
            /* GPU 预览链（2026-10-23）：lutloop + 色度中值 + down8 三段上
             * Metal，spatial 仍走 CPU（邻域语义多、占比小）。任一环节失败
 * 自动回 CPU 对应段，输出仍正确只是慢。±1 LSB 级差异（已 CMP 校验）：
 * 默认开（2026-10-30），预览与导出/CLI 同样生效；X3F_GPU_APPLY=0 强制回 CPU。 */
            int gpu_lut = 0;
#if defined(HAVE_OWN_GPU) && HAVE_OWN_GPU
            if (gpu_apply_gate(p, mono, fillgain)) {
                const x3f_color_mode_t *cmv = &p->color;
                float cm3f[9];
                int has_cm3 = (!cmv->unit) ? 1 : 0;
                double dlo, dhi;
                int ci;
                if (has_cm3)
                    for (ci = 0; ci < 9; ci++) cm3f[ci] = (float)cmv->m[ci];
                /* 表重建条件与 apply_common 逐字同一套（拖强度滑块每帧命中） */
                if (g_color_match &&
                    (!g_cm_built || g_cm_q_lut != g_cm_quattro ||
                     !cm_mode_same(cmv, g_cm_mode_valid ? &g_cm_mode : NULL)))
                    build_color_match_lut(cmv);
                gpu_desat_range(&dlo, &dhi);
                gpu_lut = (own_gpu_apply_lut16(
                              render_src, clip,
                              main_lut, alookg ? alookg : main_lut,
                              alookb ? alookb : main_lut,
                              p_wb_gain,
                              has_cm3 ? cm3f : NULL,
                              g_color_match ? g_cm_cb : NULL,
                              g_color_match ? g_cm_cr : NULL,
                              (float)(1.0 + p->saturation * 0.5),
                              p->saturation != 0.0,      /* mono==NULL 由 gate 保证 */
                              (float)hl_desat_use,
                              (float)dlo, (float)dhi,
                              tmp, w, h) == 0);
                APPLY_TAP("lutloopG");
            }
#endif
            if (gpu_lut) {
                /* GPU lut 已写 tmp；色度中值**默认也走 GPU**（2026-10-31 修订，
                 * 见 x3f_render_gpu_chrmed_enabled 注释：单趟排序网络把 31 趟
                 * 二分换成 1 趟，5424×3616 单平面 2.67s → 0.26s，且与 CPU 逐位
                 * 一致）。r≥4 的大窗口仍走二分。GPU 中值失败时对产物回 CPU
                 * 中值兜底 */
                int chrmed_ok = X3FRENDER_OK;
#if defined(HAVE_OWN_GPU) && HAVE_OWN_GPU
                if (cmr > 0)
                    chrmed_ok = x3f_render_gpu_chrmed_enabled()
                        ? own_gpu_chrmed_u16(tmp, w, h, w * 6, cmr)
                        : -1;   /* 默认 CPU：非 OK → 走下面的 CPU 中值 */
#endif
                if (chrmed_ok != X3FRENDER_OK)
                    chrmed_ok = apply_chroma_median(cmr, tmp, w, h, w * 6, 16);
                rc = chrmed_ok;
                APPLY_TAP("chrmed");
#if defined(HAVE_OWN_GPU) && HAVE_OWN_GPU
                /* 自校验（X3F_GPU_APPLY_CMP=1）：同参数 CPU 链（lut+chrmed）
                 * 再算一遍，与 GPU 产物比逐字节差异。放在中值**之后**——
                 * 早期版本放在 lut 后、chrmed 前，比的是两个不同阶段，误报。 */
                if (g_gpu_apply_cmp || getenv("X3F_GPU_APPLY_CMP") != NULL) {
                    uint16_t *tmp2 =
                        (uint16_t *)malloc((size_t)w * h * 3 * sizeof(uint16_t));
                    if (tmp2 != NULL) {
                        size_t np3 = (size_t)w * h * 3, k, nd = 0;
                        unsigned mx = 0;
                        double sum = 0.0;
                    if (apply_common(main_lut, 1.0 + p->saturation * 0.5,
                                     p->saturation != 0.0 && mono == NULL,
                                     hl_desat_use, clip, fillgain, p_wb_gain,
                                     mono, &p->color, render_src, w, h,
                                     tmp2, w * 6, 16, alookg, alookb, p)
                                == X3FRENDER_OK && cmr > 0)
                            apply_chroma_median(cmr, tmp2, w, h, w * 6, 16);
                        for (k = 0; k < np3; k++) {
                            int dd = (int)tmp[k] - (int)tmp2[k];
                            if (dd < 0) dd = -dd;
                            if (dd) {
                                if (nd < 5)
                                    fprintf(stderr, "[gpu-cmp]  首差 #%zu px=%zu c=%zu gpu=%u cpu=%u\n",
                                            nd, k / 3, k % 3, (unsigned)tmp[k],
                                            (unsigned)tmp2[k]);
                                nd++;
                                sum += (double)dd;
                                if ((unsigned)dd > mx) mx = (unsigned)dd;
                            }
                        }
                        {
                            char _s[256];
                            snprintf(_s, sizeof _s,
                                     "[gpu-cmp] lut+chrmed %u/%zu 不一致 max=%u mean=%.4f",
                                     (unsigned)nd, np3, mx,
                                     nd ? sum / (double)nd : 0.0);
                            fprintf(stderr, "%s\n", _s);
                            if (g_gpu_cmp_report) g_gpu_cmp_report(_s);
                        }
                        free(tmp2);
                    }
                }
#endif
            } else {
                /* 黑白下饱和度不参与（SPP 手册 §9-4：饱和度不能用于黑白处理） */
                rc = apply_common(main_lut, 1.0 + p->saturation * 0.5,
                                  p->saturation != 0.0 && mono == NULL,
                                  hl_desat_use, clip, fillgain, p_wb_gain, mono,
                                  &p->color,
                                  render_src, w, h, tmp, w * 6, 16, alookg, alookb, p);
                APPLY_TAP("lutloop");
                if (!look_defer && rc == X3FRENDER_OK && cmr > 0)
                    rc = apply_chroma_median(cmr, tmp, w, h, w * 6, 16);
                APPLY_TAP("chrmed");
            }
            if (!look_defer && rc == X3FRENDER_OK && clpb > 0)
                apply_chroma_shlp(clpb, tmp, w, h, w * 6, 16);
            if (!look_defer && rc == X3FRENDER_OK)
                rc = apply_spatial(p, tmp, w, h, w * 6, 16, clip);
            APPLY_TAP("spatial");
            if (rc == X3FRENDER_OK) {
                /* 关闭过曝矫正时，8 位量化加 4×4 Bayer 抖动（±0.5 码值）。
                 * 高光肩把一大段亮度压进 250~255，直接截断会出现一圈圈
                 * "等高线"；抖动只打散量化台阶，不加结构、不改平均亮度。
                 * 矫正开启时 dth 恒为 0，输出与原来逐字节一致。 */
                static const int bayer4[16] = {
                    0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5
                };
                int d8_done = 0;
#if defined(HAVE_OWN_GPU) && HAVE_OWN_GPU
                if (gpu_lut)   /* 同一条 GPU 链；失败回下面的 CPU 循环 */
                    d8_done = (own_gpu_down8(tmp, out, w, h, stride,
                                             p->hl_soften) == 0);
#endif
                if (!d8_done) {
                    for (y = 0; y < h; y++) {
                        const uint16_t *s = tmp + (size_t)y * w * 3;
                        uint8_t *d = (uint8_t *)out + (size_t)y * stride;
                        int x;
                        for (x = 0; x < w * 3; x++) {
                            const int dth = p->hl_soften
                                ? (bayer4[((y & 3) << 2) | ((x / 3) & 3)] - 8) * 16
                                : 0;
                            unsigned v = ((unsigned)s[x] + 128u + (unsigned)dth) >> 8;
                            d[x] = (uint8_t)(v > 255u ? 255u : v);
                        }
                    }
                }
            }
            free(tmp);
            free(fillgain);
            free(nrwork);
            free(dwork);
            APPLY_TAP("down8");
            /* 解码侧边缘锐化: 此时 out 已是最终 8 位显示图, 所有曲线跑完,
             * 加同量高频到三通道不会再生色偏。必须在 return 前、out 写定后。 */
            if (!look_defer) {
                apply_decode_unsharp(out, w, h, stride, bits);
                apply_hf_soften(out, w, h, stride, bits);
        APPLY_TAP("hfsoft");
                if (p->glow > 0.0) apply_glow(out, w, h, stride, bits, p);
            }
            return rc;
        }

        rc = apply_common(main_lut, 1.0 + p->saturation * 0.5,
                          p->saturation != 0.0 && mono == NULL,
                          hl_desat_use, clip, fillgain,
                          p_wb_gain, mono, &p->color,
                          render_src, w, h, out, stride, bits, alookg, alookb, p);
        APPLY_TAP("lutloop");
        free(fillgain);
        free(nrwork);
        free(dwork);
        if (rc != X3FRENDER_OK) return rc;
        if (!look_defer && cmr > 0)
            rc = apply_chroma_median(cmr, out, w, h, stride, bits);
        APPLY_TAP("chrmed");
        if (rc != X3FRENDER_OK) return rc;
        if (!look_defer && clpb > 0)
            apply_chroma_shlp(clpb, out, w, h, stride, bits);
        APPLY_TAP("chrshlp");
    }

    /* 观感延后：到这里 out 已经是线性 sRGB 码值，显示域后处理一律不做。 */
    if (look_defer) {
        /* SDQ 高光在线性馈送里没有显示域可用，改用 RAW 饱和分掩膜收色
         * （见 sdq_linear_desat）。只在 32 位 float 出口做，16 位那条
         * look_defer 出口维持原样。 */
        if (bits == 32 && g_sdq_hl > 0.0 && sdq_dig_active())
            sdq_linear_desat((float *)out, clip, w, h, stride, g_sdq_hl);
        return X3FRENDER_OK;
    }

    /* 空间类放在最后：在显示空间上做，符合视觉感受 */
    {
        /* RCPS（X3F_RCPS=1，默认关）：Merrill 行/列 FPN 置换抑制。§30 定案
         * 接在**显示域** tone 后、unsharp 前（opt-in 诊断入口——§30 已证
         * 0877 可见竖纹是 unsharp 防护自印，非 RCPS 类问题，默认 no-op）。
         * 调试对质：X3F_RCPS_DEBUG>=2 落盘 /tmp/rcps_disp_{in,out}.f32。 */
        x3f_rcps_apply_disp(out, bits, w, h);
        APPLY_TAP("rcps");
        /* 解码侧边缘锐化(补 NLM 柔掉的那线细节, 不放大彩边) —— 必须在
         * 所有色调/饱和度曲线之后, 显示空间上做, 否则会被曲线弯成色偏。 */
        apply_decode_unsharp(out, w, h, stride, bits);
        APPLY_TAP("unsharp");
        int rc2 = apply_spatial(p, out, w, h, stride, bits, clip);
        APPLY_TAP("spatial");
        /* 软压缩必须排在 spatial 之后：spatial 会重新抬高频，先压等于白压 */
        apply_hf_soften(out, w, h, stride, bits);
        if (p->glow > 0.0) apply_glow(out, w, h, stride, bits, p);
        return rc2;
    }
}

int x3f_render_apply_ex(const x3f_edit_params_t *p,
                        const float *lin, const float *clip,
                        int w, int h, uint8_t *out, int stride)
{
    /* RCPS lin 域接线已撤（§30.3）：显示域条纹与 lin 列纹 corr 仅 0.02-0.18，
     * lin 域施压无效；RCPS disp（tone 后 unsharp 前）保留 opt-in 诊断。 */
    return apply_ex_common(p, lin, clip, w, h, out, stride, 8);
}

int x3f_render_apply_ex16(const x3f_edit_params_t *p,
                          const float *lin, const float *clip,
                          int w, int h, uint16_t *out, int stride)
{
    return apply_ex_common(p, lin, clip, w, h, out, stride, 16);
}

/* 同 x3f_render_apply_ex16，但输出**未钳制的 float 场景线性**（每通道 float，
 * RGB 交错，stride = w*3*sizeof(float)）。
 *
 * 用途：胶片引擎（Fotufilm host）要的是"场景光"而不是"调好色的照片"，而它的
 * halation（背面散射辉光）物理上由 >1.0 的高光溢出驱动。16 位出口把 HDR 钳死
 * 在 [0,1]，高光头子没了，光晕就长不出来。这条出口走的是**同一条管线**
 * （WB / 填充光 / 细节 / 色彩模式矩阵全在），只是最后一步不钳、直接写 float，
 * 于是 HDR 余量一路带到引擎。
 *
 * 只与 `look_defer=1` 配套使用（否则后面整条显示链仍会把 >1 钳掉，毫无意义）。
 * 调用方负责把 RGB 展开成宿主要的 RGBA、并把负值（sRGB 三角外的外推颜色）夹 0。 */
int x3f_render_apply_ex_linear(const x3f_edit_params_t *p,
                               const float *lin, const float *clip,
                               int w, int h, float *out)
{
    return apply_ex_common(p, lin, clip, w, h, out, w * 3 * (int)sizeof(float), 32);
}

int x3f_render_apply(const x3f_edit_params_t *p,
                     const float *lin, int w, int h,
                     uint8_t *out, int stride)
{
    return x3f_render_apply_ex(p, lin, NULL, w, h, out, stride);
}

/* 只跑显示域空间算子（锐度 / 细节 / 阴影高光分区）。见 x3f_render.h 的说明：
 * 两段式第二段直通时用它补回锐化，其余观感一律不施加。 */
int x3f_render_sharpen16(const x3f_edit_params_t *p, uint16_t *px,
                         int w, int h, int stride)
{
    if (p == NULL || px == NULL || w <= 0 || h <= 0) return X3FRENDER_ERR_PARAM;
    return apply_spatial(p, px, w, h, stride, 16, NULL);
}

/* ---------- 整幅换域（观感链留在 sRGB，成品搬进宽容器）-----------------
 *
 * 背景：胶片模拟 / 数字印像 / 片边（SpektraLab）是在 **sRGB 显示码值**上标定
 * 的，输出也是 sRGB。用户想把这份成品放进 ProPhoto / P3 / 2020 的 16 位 TIFF
 * 里时，只有两种可能：
 *   a) 把广色域码值直接喂给那张按 sRGB 标定的表 —— 串色（γ 与通道比例都错），
 *   b) 先在 sRGB 域把观感做完，再把成品整幅搬到目标空间 —— 本函数。
 * 走 (b)。语义上讲得通：胶片观感本来就是"显示域的效果"，容器只负责装。
 */
static void x3f__mat3_inv(const double *m, double *r)
{
    const double a = m[0], b = m[1], c = m[2];
    const double d = m[3], e = m[4], f = m[5];
    const double g = m[6], h = m[7], i = m[8];
    const double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if (fabs(det) < 1e-12) {          /* 只在传进来非可逆矩阵时发生，回单位阵 */
        r[0] = 1.0; r[1] = 0.0; r[2] = 0.0;
        r[3] = 0.0; r[4] = 1.0; r[5] = 0.0;
        r[6] = 0.0; r[7] = 0.0; r[8] = 1.0;
        return;
    }
    const double id = 1.0 / det;
    r[0] = (e * i - f * h) * id;
    r[1] = (c * h - b * i) * id;
    r[2] = (b * f - c * e) * id;
    r[3] = (f * g - d * i) * id;
    r[4] = (a * i - c * g) * id;
    r[5] = (c * d - a * f) * id;
    r[6] = (d * h - e * g) * id;
    r[7] = (b * g - a * h) * id;
    r[8] = (a * e - b * d) * id;
}

/* sRGB 线性 → 目标空间线性的矩阵（NULL = sRGB 自己，单位阵） */
static const double *x3f__out_matrix(int space)
{
    switch (space) {
    case X3F_OUT_PROPHOTO: return kSRGBToProPhoto;
    case X3F_OUT_ADOBE:    return kSRGBToAdobe;
    case X3F_OUT_P3:       return kSRGBToP3;
    case X3F_OUT_DCI_P3:   return kSRGBToP3;   /* 同原色，只差 TRC */
    case X3F_OUT_2020:     return kSRGBTo2020;
    default:               return NULL;
    }
}

int x3f_render_reencode16(const uint16_t *src, int w, int h,
                          int from_space, int to_space, uint16_t *dst)
{
    if (src == NULL || dst == NULL || w <= 0 || h <= 0) return X3FRENDER_ERR_PARAM;
    if (from_space < X3F_OUT_SRGB || from_space > X3F_OUT_DCI_P3) from_space = X3F_OUT_SRGB;
    if (to_space   < X3F_OUT_SRGB || to_space   > X3F_OUT_DCI_P3) to_space   = X3F_OUT_SRGB;

    const size_t n = (size_t)w * (size_t)h;
    if (from_space == to_space) {
        memcpy(dst, src, n * 3 * sizeof(uint16_t));
        return X3FRENDER_OK;
    }

    const double *mf = x3f__out_matrix(from_space);   /* sRGB → from */
    const double *mt = x3f__out_matrix(to_space);     /* sRGB → to   */
    double inv[9];
    if (mf != NULL) x3f__mat3_inv(mf, inv);

    /* 两端各做一张 65536 项表：码值→线性（from）、线性→码值（to）。
     * 中间那步矩阵是逐像素的，表管不了；但把两端的 pow 全部摊掉之后，
     * 58.8M 通道的整幅换域从 ~9s 降到 1s 量级。 */
    double *dec = (double *)malloc(65536 * sizeof(double));
    uint16_t *enc = (uint16_t *)malloc(65536 * sizeof(uint16_t));
    if (dec == NULL || enc == NULL) { free(dec); free(enc); return X3FRENDER_ERR_NOMEM; }
    for (int i = 0; i < 65536; i++) dec[i] = out_decode(from_space, (double)i / 65535.0);
    for (int i = 0; i < 65536; i++) {
        double d = out_encode(to_space, (double)i / 65535.0);
        if (!(d > 0.0)) d = 0.0;
        if (d > 1.0) d = 1.0;
        enc[i] = (uint16_t)(d * 65535.0 + 0.5);
    }

    for (size_t k = 0; k < n; k++) {
        const uint16_t *s = src + k * 3;
        double r = dec[s[0]], g = dec[s[1]], b = dec[s[2]];
        if (mf != NULL) {                       /* from 线性 → sRGB 线性 */
            const double a = r, e = g, c = b;
            r = inv[0] * a + inv[1] * e + inv[2] * c;
            g = inv[3] * a + inv[4] * e + inv[5] * c;
            b = inv[6] * a + inv[7] * e + inv[8] * c;
        }
        if (mt != NULL) {                       /* sRGB 线性 → to 线性 */
            const double a = r, e = g, c = b;
            r = mt[0] * a + mt[1] * e + mt[2] * c;
            g = mt[3] * a + mt[4] * e + mt[5] * c;
            b = mt[6] * a + mt[7] * e + mt[8] * c;
        }
        int i0 = (int)(r * 65535.0 + 0.5);
        int i1 = (int)(g * 65535.0 + 0.5);
        int i2 = (int)(b * 65535.0 + 0.5);
        if (i0 < 0) i0 = 0; else if (i0 > 65535) i0 = 65535;
        if (i1 < 0) i1 = 0; else if (i1 > 65535) i1 = 65535;
        if (i2 < 0) i2 = 0; else if (i2 > 65535) i2 = 65535;
        dst[k * 3]     = enc[i0];
        dst[k * 3 + 1] = enc[i1];
        dst[k * 3 + 2] = enc[i2];
    }
    free(dec);
    free(enc);
    return X3FRENDER_OK;
}

int x3f_render_reencode8(const uint8_t *src, int w, int h,
                         int from_space, int to_space, uint8_t *dst)
{
    if (src == NULL || dst == NULL || w <= 0 || h <= 0) return X3FRENDER_ERR_PARAM;
    if (from_space < X3F_OUT_SRGB || from_space > X3F_OUT_DCI_P3) from_space = X3F_OUT_SRGB;
    if (to_space   < X3F_OUT_SRGB || to_space   > X3F_OUT_DCI_P3) to_space   = X3F_OUT_SRGB;

    const size_t n = (size_t)w * (size_t)h;
    if (from_space == to_space) {
        memcpy(dst, src, n * 3);
        return X3FRENDER_OK;
    }

    const double *mf = x3f__out_matrix(from_space);
    const double *mt = x3f__out_matrix(to_space);
    double inv[9];
    if (mf != NULL) x3f__mat3_inv(mf, inv);

    double dec[256];
    uint8_t enc[256];
    for (int i = 0; i < 256; i++) dec[i] = out_decode(from_space, (double)i / 255.0);
    for (int i = 0; i < 256; i++) {
        double d = out_encode(to_space, (double)i / 255.0);
        if (!(d > 0.0)) d = 0.0;
        if (d > 1.0) d = 1.0;
        enc[i] = (uint8_t)(d * 255.0 + 0.5);
    }

    for (size_t k = 0; k < n; k++) {
        const uint8_t *s = src + k * 3;
        double r = dec[s[0]], g = dec[s[1]], b = dec[s[2]];
        if (mf != NULL) {
            const double a = r, e = g, c = b;
            r = inv[0] * a + inv[1] * e + inv[2] * c;
            g = inv[3] * a + inv[4] * e + inv[5] * c;
            b = inv[6] * a + inv[7] * e + inv[8] * c;
        }
        if (mt != NULL) {
            const double a = r, e = g, c = b;
            r = mt[0] * a + mt[1] * e + mt[2] * c;
            g = mt[3] * a + mt[4] * e + mt[5] * c;
            b = mt[6] * a + mt[7] * e + mt[8] * c;
        }
        int i0 = (int)(r * 255.0 + 0.5);
        int i1 = (int)(g * 255.0 + 0.5);
        int i2 = (int)(b * 255.0 + 0.5);
        if (i0 < 0) i0 = 0; else if (i0 > 255) i0 = 255;
        if (i1 < 0) i1 = 0; else if (i1 > 255) i1 = 255;
        if (i2 < 0) i2 = 0; else if (i2 > 255) i2 = 255;
        dst[k * 3]     = enc[i0];
        dst[k * 3 + 1] = enc[i1];
        dst[k * 3 + 2] = enc[i2];
    }
    return X3FRENDER_OK;
}

/* 只读 4 字节判断是不是 X3I(SFD) 容器 —— 下面两个"只看头"的探针不需要把
 * 55MB 的帧物化出来（容器帧是 v4 Quattro 头，色彩模式字段本来就不在里面）。 */
static int is_x3i_container(const char *path)
{
    FILE *f;
    char magic[4];
    if (path == NULL) return 0;
    f = fopen(path, "rb");
    if (f == NULL) return 0;
    int hit = (fread(magic, 1, 4, f) == 4 && memcmp(magic, "FOVi", 4) == 0);
    fclose(f);
    return hit;
}

/* 只读文件头里的色彩模式（Merrill / SD1 这类 2.3+ 头有；Quattro 的 v4 头没有，
 * 那批要去内嵌 JPEG 的 EXIF MakerNote 里找）。同为"不解码只看头"，和下面
 * probe_is_merrill 一个套路。 */
int x3f_render_probe_header_color_mode(const char *path, char *out, int cap)
{
    FILE *f;
    x3f_t *x3f;
    int have = 0;
    char tmp[1024];
    const char *use;

    if (out == NULL || cap <= 0) return 0;
    out[0] = '\0';
    if (path == NULL) return 0;
    /* X3I 容器：帧是 v4 的 Quattro 头，里面没有 color_mode 字符串（那批要走内嵌
     * JPEG 的 EXIF MakerNote），所以这里直接给 0，不用把帧物化出来。 */
    if (is_x3i_container(path)) return 0;
    use = x3f_standalone_path(path, -1, tmp, sizeof(tmp));
    f = fopen(use, "rb");
    if (f == NULL) { x3f_temp_release(tmp); return 0; }
    x3f = x3f_new_from_file(f);
    if (x3f == NULL) { fclose(f); x3f_temp_release(tmp); return 0; }
    if (x3f->header.color_mode[0] != '\0') {
        int i;
        for (i = 0; i < cap - 1 && x3f->header.color_mode[i] != '\0'; i++)
            out[i] = x3f->header.color_mode[i];
        out[i] = '\0';
        have = 1;
    }
    x3f_delete(x3f);
    fclose(f);
    x3f_temp_release(tmp);
    return have;
}

/* 只看文件头：这个 X3F 是不是 Merrill（DP1M/DP2M/DP3M）。
 * 渲染层现在不需要它了（Merrill 那几层修正已随色彩链一起删掉），留着是因为
 * 它本身还有用：A/B、诊断、将来真要做 Merrill 专属处理时得有个判据。 */
int x3f_render_probe_is_merrill(const char *path)
{
    FILE *f;
    x3f_t *x3f;
    x3f_directory_entry_t *DE;
    int is_merrill = 0;
    char tmp[1024];
    const char *use;

    if (path == NULL) return 0;
    /* X3I(SFD) 容器只出现在 Quattro 世代（SFD 是 Quattro 才有的连拍模式），
     * 不是 Merrill —— 直接给 0，省掉一次 55MB 的帧物化。 */
    if (is_x3i_container(path)) return 0;
    use = x3f_standalone_path(path, -1, tmp, sizeof(tmp));
    f = fopen(use, "rb");
    if (f == NULL) { x3f_temp_release(tmp); return 0; }
    x3f = x3f_new_from_file(f);
    if (x3f == NULL) { fclose(f); x3f_temp_release(tmp); return 0; }
    DE = x3f_get_raw(x3f);
    if (DE != NULL)
        is_merrill = (DE->header.data_subsection.image_data.type_format
                      == X3F_IMAGE_RAW_MERRILL);
    x3f_delete(x3f);
    fclose(f);
    x3f_temp_release(tmp);
    return is_merrill;
}

/* 探测这张文件是否 Quattro 世代。判据：
 *   · X3I(SFD) 容器 —— SFD 连拍只在 Quattro 机上有，直接 1（省一次全量解析）；
 *   · RAW 段 type_format == X3F_IMAGE_RAW_QUATTRO（DP0/DP1/DP2/DP3 Quattro、SD Quattro 系）。
 * 其余（Merrill / 经典 Foveon）返回 0。与 probe_is_merrill 同结构，独立开文件探测，
 * 不读全局状态（多线程安全）。 */
int x3f_render_probe_is_quattro(const char *path)
{
    FILE *f;
    x3f_t *x3f;
    x3f_directory_entry_t *DE;
    int is_quattro = 0;
    char tmp[1024];
    const char *use;

    if (path == NULL) return 0;
    if (is_x3i_container(path)) return 1;
    use = x3f_standalone_path(path, -1, tmp, sizeof(tmp));
    f = fopen(use, "rb");
    if (f == NULL) { x3f_temp_release(tmp); return 0; }
    x3f = x3f_new_from_file(f);
    if (x3f == NULL) { fclose(f); x3f_temp_release(tmp); return 0; }
    DE = x3f_get_raw(x3f);
    if (DE != NULL)
        is_quattro = (DE->header.data_subsection.image_data.type_format
                      == X3F_IMAGE_RAW_QUATTRO);
    x3f_delete(x3f);
    fclose(f);
    x3f_temp_release(tmp);
    return is_quattro;
}

/* 探测这张文件是否 Quattro **世代**——过曝矫正（oe mode8 / hole-fix）门控口径。
 * 与 x3f_render_probe_is_quattro 的区别：SD Quattro / SD Quattro H 在 X3F 里是
 * **独立的 type_format**（X3F_IMAGE_RAW_SDQ=0x25 / X3F_IMAGE_RAW_SDQH=0x27），
 * 旧探针只认 QUATTRO(0x23)，把 SDQ/SDQH 判成非 Quattro —— 2026-10-31 用户报
 * 过曝矫正面板在 SDQ/SDQH 上被误置灰。判据：
 *   · X3I(SFD) 容器 —— 同旧探针，直接 1；
 *   · RAW 段 type_format ∈ { QUATTRO, SDQ, SDQH }。
 * 绿增强（g_cm_quattro）**不**换用本探针：×1.06 只在 DP-Q 系上标定过，
 * SDQ 的绿行为保持现状（继续走窄口径旧探针）。 */
int x3f_render_probe_is_quattro_gen(const char *path)
{
    FILE *f;
    x3f_t *x3f;
    x3f_directory_entry_t *DE;
    int is_quattro = 0;
    char tmp[1024];
    const char *use;

    if (path == NULL) return 0;
    if (is_x3i_container(path)) return 1;
    use = x3f_standalone_path(path, -1, tmp, sizeof(tmp));
    f = fopen(use, "rb");
    if (f == NULL) { x3f_temp_release(tmp); return 0; }
    x3f = x3f_new_from_file(f);
    if (x3f == NULL) { fclose(f); x3f_temp_release(tmp); return 0; }
    DE = x3f_get_raw(x3f);
    if (DE != NULL) {
        uint32_t tf = DE->header.data_subsection.image_data.type_format;
        is_quattro = (tf == X3F_IMAGE_RAW_QUATTRO ||
                      tf == X3F_IMAGE_RAW_SDQ ||
                      tf == X3F_IMAGE_RAW_SDQH);
    }
    x3f_delete(x3f);
    fclose(f);
    x3f_temp_release(tmp);
    return is_quattro;
}

/* 探测是否 SD Quattro / SD Quattro H 机身（RAW type_format 0x25 / 0x27）。
 *
 * 「SDQ 高光」面板门控用这个，而不是 x3f_render_probe_sdq_hl：
 *   · 归一强度在 C 链里只对 DigitalISOGain≠1 的机身有像素效果
 *     （SDQH 是 4,4,2；SDQ 是 1,1,1，乘 1.0 精确无变化）；
 *   · 但 X3Lumina 在两种 SDQ 机身上都摆滑杆，按增益门控会把 SDQ 的滑杆
 *     误置灰（2026-11-16 用户实测报障「SDQ 高光拉杆无法拖动」）。
 * 门控按机身、效果按增益，两块口径分开。 */
int x3f_render_probe_is_sdq_body(const char *path)
{
    FILE *f;
    x3f_t *x3f;
    x3f_directory_entry_t *DE;
    int is_sdq = 0;
    char tmp[1024];
    const char *use;

    if (path == NULL) return 0;
    if (is_x3i_container(path)) return 0;
    use = x3f_standalone_path(path, -1, tmp, sizeof(tmp));
    if (use == NULL) { x3f_temp_release(tmp); return 0; }
    f = fopen(use, "rb");
    if (f == NULL) { x3f_temp_release(tmp); return 0; }
    x3f = x3f_new_from_file(f);
    if (x3f == NULL) { fclose(f); x3f_temp_release(tmp); return 0; }
    DE = x3f_get_raw(x3f);
    if (DE != NULL) {
        uint32_t tf = DE->header.data_subsection.image_data.type_format;
        is_sdq = (tf == X3F_IMAGE_RAW_SDQ || tf == X3F_IMAGE_RAW_SDQH);
    }
    x3f_delete(x3f);
    fclose(f);
    x3f_temp_release(tmp);
    return is_sdq;
}

/* 探测这张文件是否属于 Classic-Foveon auto-tone 代（SD14 / SD1 等）。
 * 与 load_common 里设置 g_classic_active 的判据**逐位一致**：
 *   · CAMMODEL == "SIGMA SD1"（无 DBG_Auto* 块，走 SD1 专属曲线）
 *   · 或 CAMF 含 DBG_AutoContrast 键（SD9/SD10/SD14/SD15/DP1/DP2/DP1x/DP2x …）
 * 命中返回 1。
 *
 * 给 Swift 侧 `previewNLMStrength` 用：classic 曲线把暗部提亮、把传感器原生暗部
 * 颗粒显形，低 ISO 预览若不降噪会满屏颗粒（1:1 放大恒带 NLM 故清晰）。所以 classic
 * 文件要**绕过低 ISO 预览零降噪门槛**，与 1:1 同口径。自选择，不写死机型列表。
 *
 * 独立开文件探测，不读全局 g_classic_active（那份是 per-当前文件状态，多线程不安全）。
 * SFD(X3I) 容器只出现在 Quattro 代，非 classic，直接 0。 */
int x3f_render_probe_is_classic_tone(const char *path)
{
    char tmp[1024];
    const char *use;
    int is_classic = 0;
    unsigned cd[3] = {1, 1, 1};
    double v = 0.0;

    if (path == NULL) return 0;
    if (is_x3i_container(path)) return 0;
    use = x3f_standalone_path(path, -1, tmp, sizeof(tmp));
    if (use == NULL) { x3f_temp_release(tmp); return 0; }
    {
        char model[64] = "";
        if (probe_cammodel(use, model, (int)sizeof model) &&
            strcmp(model, "SIGMA SD1") == 0) {
            is_classic = 1;
        }
        if (!is_classic &&
            x3f_camf_matrix_f64(use, "DBG_AutoContrast", &v, 1, cd) == 1) {
            is_classic = 1;
        }
    }
    x3f_temp_release(tmp);
    return is_classic;
}

/* 把这张照片的噪声系数（ISO 缩放后的逐通道 a/b/c，共 9 个）取出来给 x3f_sfd.c
 * 用 —— SFD 合成的 1/(k²σ²) 要和降噪那一路**同源**，不能各写一份标定。
 * 返回 1 = 拿到了（Quattro 会走 那组常量）。*/
int x3f_render_noise_coef_for(const char *path, double out[9])
{
    int i;
    if (path == NULL || out == NULL) return 0;
    nr_load_model(path);
    if (!g_nr.valid) return 0;
    for (i = 0; i < 9; i++) out[i] = g_nr.cn_iso[i];
    return 1;
}

/* ── Merrill「SPP 一致模式」旁路 ────────────────────────────────────────
 *
 * 背景：产品架构是「取 scene-linear → 自己渲染」，滑杆 / WB / 色调曲线全在
 * 我方链上。 Merrill 原生链（float 前端 → SPP NR → tone → 色彩矩阵）
 * 在 x3fuse 里只以**成品**形式存在：spp::render 出 16 位 sRGB，不产
 * scene-linear f32。所以它不能直接接管渲染链 —— 那会让自家滑杆全废。
 *
 * 这里走**旁路**：单独导出 SPP 那张 16 位 sRGB 成品，导出档可以选它。
 * 门控 = 环境变量 X3F_MERRILL_SPP=1（默认关 → 既有路径逐字节不变）。
 *
 * 能不能走由 SPP 侧自己的门控定（x3f_spp_color_export_alloc 内部
 * validate_mode）：必须 Merrill + HI 分辨率档 + 非 Cineon + denoise==10 +
 * CPU 后端 + SkipStepFlags bit23=0 + FPNSuppression=0。不满足返回 -3，
 * 调用方回退既有路径。
 *
 * 注意这一段**不碰 x3f_t**：SPP 入口自己开文件、自己读，所以宿主的
 * legacy/x3fuse 双实现不会把类型搞混（legacy 的 x3f_t 布局不一样）。
 */
/* SPP 一致模式的门控。
 *
 * 两个来源，任一为真即开：
 *   ① 环境变量 X3F_MERRILL_SPP=1 —— 给脚本 / 命令行 / 复现用（老行为）。
 *   ② 程序化开关 x3f_render_spp_set_enabled(1) —— 给产品用：用户在前端
 *      选了「Merrill · SPP 原生链」，Swift 侧就调它打开，不必去 setenv
 *      （setenv 是进程级的全局副作用，且在多线程里改环境不安全）。
 *
 * ② 是**显式**开关：一旦被调用就以它的值为准（-1 = 回到「只听环境变量」）。
 * 默认值 -1 → 既有路径逐字节不变。
 */
static int merrill_spp_switch = -1;

void x3f_render_spp_set_enabled(int on)
{
    /* -1 means "follow X3F_MERRILL_SPP"; it must not collapse to 0. The
     * callers use set_enabled(1) ... set_enabled(-1) around a probe/render. */
    merrill_spp_switch = (on < 0) ? -1 : (on ? 1 : 0);
}

static int merrill_spp_enabled(void)
{
    if (merrill_spp_switch >= 0) return merrill_spp_switch;
    {
        const char *e = getenv("X3F_MERRILL_SPP");
        return e != NULL && *e != '\0' && strcmp(e, "0") != 0;
    }
}

/* 打开 X3F（容器先落成独立 X3F）。成功返回 FILE*，失败 NULL。
 * 临时容器落盘后句柄已开，可以立刻删掉临时文件（POSIX unlink 语义：
 * 已打开的 fd 继续有效）。 */
static FILE *merrill_spp_open(const char *path, char *tmp, size_t tmpcap)
{
    const char *use;
    FILE *f;

    use = x3f_standalone_path(path, -1, tmp, tmpcap);
    f = fopen(use, "rb");
    if (f == NULL) {
        if (use != path) x3f_temp_release(tmp);
        return NULL;
    }
    if (use != path) x3f_temp_release(tmp);
    return f;
}

/* 这个文件能不能走 SPP 一致模式？能 → 返回 1 并把尺寸写进 w / h；
 * 不能 → 返回 0（调用方走既有路径）。门控没开时永远返回 0。 */
int x3f_render_spp_capable(const char *path, int *w, int *h)
{
    FILE *f;
    char tmp[1024];
    int rc;

    if (w != NULL) *w = 0;
    if (h != NULL) *h = 0;
    if (path == NULL || w == NULL || h == NULL) return 0;
    if (!merrill_spp_enabled()) return 0;
    /* X3I(SFD) 容器是 Quattro 世代的连拍模式，不是 Merrill —— 直接否。 */
    if (is_x3i_container(path)) return 0;

    /* 只做门控判定，不渲染 —— 一次完整 SPP 渲染在 24MP 上要好几秒，
     * 而"这个文件能不能走 SPP 路"是个便宜得多的问题。门控顺带给出几何。 */
    f = merrill_spp_open(path, tmp, sizeof(tmp));
    if (f == NULL) return 0;
    {
        uint32_t sw = 0, sh = 0;
        rc = x3f_spp_export_check(f, &sw, &sh);
        fclose(f);
        if (rc != 0) return 0;
        *w = (int)sw;
        *h = (int)sh;
    }
    return 1;
}

/* 用 SPP 原生链导出一张 16 位 sRGB 成品（每通道 u16，0..65535）。
 *
 * out 的 stride 单位是**字节**（= w*3*2），与 x3f_render_apply_ex16 一致；
 * stride 更大时逐行拷。返回 0 成功；非 0 失败（调用方应回退既有路径）。
 *
 * 成品是**传感器方向、未旋转**的，和 x3f_render_load_* 返回的 lin 同一朝向，
 * 上层那套 orientation 处理照常生效。
 *
 * tone 参数直接落在 SPP 自己的 tone 阶段（机身标定曲线就在那一层），
 * 所以曝光/对比/暗部/高光/饱和**全部原生生效**，不需要跨域换算：
 *   exposure   曝光 EV（产品语义：正=变亮）
 *   contrast   对比，叠加在 CAMF 标定值之上
 *   shadows    阴影，正=提亮（SPP 同号）
 *   highlights 高光，**正=提亮**（SPP 语义，与产品「正=压暗」反号，调用方取负）
 *   saturation 饱和度，**SPP 滑杆值**（-2..+2，0 = 原生）。SPP 定义 u = Sat/2 + 1，
 *             在 tone 之后用 Rec.601 饱和度矩阵 —— 与产品 fuse 链同一套定义，
 *             所以这个值可以原样直传，两条解码链的手感一致
 *   nr_level   SPP 降噪档 0..4（CAMF DenoisingSlidersStep 的行），-1 = 原生中档
 * 传 NaN = 该项不覆盖，走原生值。
 */

/* ------------------------------------------------------------------ *
 * ABI 护栏（2026-10-28）
 * ------------------------------------------------------------------ *
 *
 * x3f_spp_tone 是**按指针**传给 Rust 的（repr(C)），所以它和 Rust 的 SppTone
 * 必须逐字节一致。布局对不上的后果不是报错，而是**静默出错**：
 *   - Rust 按新布局读 C 侧结构体尾部的内存 → 越界读；
 *   - 读到的是栈上的垃圾值而**不是 NaN** → "保持原生值"的哨兵失效 →
 *     真的套用一个随机 creative override，而且每次调用都不一样。
 *
 * 三道护栏（详见 crates/x3f-sys/csrc/x3f_spp.h 的注释）：
 *   1) C 头的 _Static_assert 抓本文件自身的漂移；
 *   2) Rust 侧的 const assert 抓 Rust 侧的漂移；
 *   3) 下面这个跨文件比对 —— libx3f.a 若是旧的（没有 x3f_spp_tone_abi_size
 *      这个符号），**链接期就失败**；若 Rust 改了而 C 头没改，运行期这里
 *      直接拒绝，不会带着错位的结构体出图。
 */
static int spp_tone_abi_ok(void)
{
    uint32_t rust = x3f_spp_tone_abi_size();
    uint32_t c = (uint32_t)sizeof(x3f_spp_tone);
    if (rust != c) {
        fprintf(stderr,
                "[x3f_render] SPP tone ABI mismatch: Rust SppTone = %u B, "
                "C x3f_spp_tone = %u B — 拒绝执行（否则会越界读并套用随机 override）\n",
                rust, c);
        return 0;
    }
    return 1;
}

/* 填一套"全用原生值"的 tone。NaN = 该项不覆盖。
 * 新增字段必须**显式**置 NaN：栈上的垃圾值不是 NaN，哨兵会失效。 */
static void spp_tone_reset(x3f_spp_tone *tone)
{
    tone->exposure = NAN;
    tone->contrast = NAN;
    tone->shadows = NAN;
    tone->highlights = NAN;
    tone->saturation = NAN;
    tone->nr_level = -1;
    tone->reserved = 0;
    tone->sharpness = NAN;
    tone->fill_light = NAN;
    tone->highlight_control = NAN;
    tone->coladj[0] = NAN;
    tone->coladj[1] = NAN;
    tone->coladj[2] = NAN;
}

int x3f_render_spp_export_tone(const char *path, uint16_t *out, int stride,
                               int *w, int *h,
                               double exposure, double contrast,
                               double shadows, double highlights,
                               double saturation, int nr_level)
{
    FILE *f;
    char tmp[1024];
    uint16_t *buf = NULL;
    uint32_t sw = 0, sh = 0;
    int rc;

    if (path == NULL || out == NULL || w == NULL || h == NULL)
        return X3FRENDER_ERR_PARAM;
    if (!merrill_spp_enabled()) return X3FRENDER_ERR_PARAM;
    if (is_x3i_container(path)) return X3FRENDER_ERR_PARAM;
    if (!spp_tone_abi_ok()) return X3FRENDER_ERR_PARAM;

    f = merrill_spp_open(path, tmp, sizeof(tmp));
    if (f == NULL) return X3FRENDER_ERR_OPEN;

    {
        x3f_spp_tone tone;
        spp_tone_reset(&tone);
        tone.exposure = exposure;
        tone.contrast = contrast;
        tone.shadows = shadows;
        tone.highlights = highlights;
        tone.saturation = saturation;
        tone.nr_level = nr_level;
        rc = x3f_spp_color_export_alloc(f, &buf, &sw, &sh, &tone);
    }
    fclose(f);
    if (rc != 0) {
        if (buf != NULL) x3f_spp_color_export_free(buf);
        return X3FRENDER_ERR_DECODE;
    }
    {
        int need = (int)(sw * 3 * 2);
        long y;
        if (stride < need) {
            x3f_spp_color_export_free(buf);
            return X3FRENDER_ERR_PARAM;
        }
        for (y = 0; y < (long)sh; y++) {
            memcpy(out + (size_t)y * (size_t)(stride / 2),
                   buf + (size_t)y * (size_t)sw * 3,
                   (size_t)need);
        }
    }
    x3f_spp_color_export_free(buf);
    *w = (int)sw;
    *h = (int)sh;
    return 0;
}

/* 不带 tone 覆盖的 SPP 成品导出：等价于宿主自己的原生档，逐字节不变。 */
int x3f_render_spp_export(const char *path, uint16_t *out, int stride,
                           int *w, int *h)
{
    const double keep = NAN;
    return x3f_render_spp_export_tone(path, out, stride, w, h,
                                      keep, keep, keep, keep, keep, -1);
}

/* ------------------------------------------------------------------ *
 * 两段式 SPP：prepare 一次，tone 段反复跑。
 * ------------------------------------------------------------------ *
 *
 * x3f_render_spp_export_tone 每调一次都把整条 SPP 链重跑一遍（24MP 上
 * ~4.1 s）。这里把链拆成两半：
 *
 *   prepare      解码 + 浮点前端 + 降噪，~3.9 s，做一次；
 *   apply_tone   只重算 tone 曲线 + 输出变换 + 逐像素，~0.03 s。
 *
 * 于是拖曝光/对比/阴影/高光/饱和度这五根滑杆时，每次只付 30 ms 而不是
 * 4 s。降噪档（nr_level）例外：降噪本身就跑在 prepare 段里，换档要重新
 * prepare，此时 apply_tone 返回 X3FRENDER_ERR_STATE。
 */

int x3f_render_spp_prepare(const char *path, int nr_level,
                            x3f_spp_prepared **out, int *w, int *h)
{
    FILE *f;
    char tmp[1024];
    x3f_spp_tone tone;
    x3f_spp_prepared *handle = NULL;
    uint32_t sw = 0, sh = 0;
    int rc;

    if (path == NULL || out == NULL || w == NULL || h == NULL)
        return X3FRENDER_ERR_PARAM;
    *out = NULL;
    *w = 0;
    *h = 0;
    if (!merrill_spp_enabled()) return X3FRENDER_ERR_PARAM;
    /* X3I(SFD) 容器是 Quattro 世代的连拍模式，不是 Merrill —— 直接否。 */
    if (is_x3i_container(path)) return X3FRENDER_ERR_PARAM;
    if (!spp_tone_abi_ok()) return X3FRENDER_ERR_PARAM;

    f = merrill_spp_open(path, tmp, sizeof(tmp));
    if (f == NULL) return X3FRENDER_ERR_OPEN;

    /* 只有 nr_level 在 prepare 段生效；其余 tone 字段是 apply 段的事。 */
    spp_tone_reset(&tone);
    tone.nr_level = nr_level;

    rc = x3f_spp_prepare(f, &tone, &handle, &sw, &sh);
    fclose(f);
    if (rc != 0) {
        if (handle != NULL) x3f_spp_prepared_free(handle);
        if (rc == -2) return X3FRENDER_ERR_OPEN;
        if (rc == -6) return X3FRENDER_ERR_NOMEM;
        return X3FRENDER_ERR_DECODE;
    }
    *out = handle;
    *w = (int)sw;
    *h = (int)sh;
    return X3FRENDER_OK;
}

int x3f_render_spp_apply_tone(x3f_spp_prepared *handle, uint16_t *out, int stride,
                              int *w, int *h,
                              double exposure, double contrast, double shadows,
                              double highlights, double saturation, int nr_level)
{
    x3f_spp_tone tone;
    uint16_t *buf = NULL;
    uint32_t sw = 0, sh = 0;
    int rc;

    if (handle == NULL || out == NULL || w == NULL || h == NULL)
        return X3FRENDER_ERR_PARAM;
    if (!spp_tone_abi_ok()) return X3FRENDER_ERR_PARAM;
    *w = 0;
    *h = 0;

    spp_tone_reset(&tone);
    tone.exposure = exposure;
    tone.contrast = contrast;
    tone.shadows = shadows;
    tone.highlights = highlights;
    tone.saturation = saturation;
    tone.nr_level = nr_level;

    rc = x3f_spp_apply_tone(handle, &tone, &buf, &sw, &sh);
    if (rc != 0) {
        if (buf != NULL) x3f_spp_color_export_free(buf);
        /* -5 = 降噪档与 prepare 时不符，要重新 prepare。 */
        if (rc == -5) return X3FRENDER_ERR_STATE;
        if (rc == -6) return X3FRENDER_ERR_NOMEM;
        return X3FRENDER_ERR_DECODE;
    }
    {
        int need = (int)(sw * 3 * 2);
        long y;
        if (stride < need) {
            x3f_spp_color_export_free(buf);
            return X3FRENDER_ERR_PARAM;
        }
        for (y = 0; y < (long)sh; y++) {
            memcpy(out + (size_t)y * (size_t)(stride / 2),
                   buf + (size_t)y * (size_t)sw * 3,
                   (size_t)need);
        }
    }
    x3f_spp_color_export_free(buf);
    *w = (int)sw;
    *h = (int)sh;
    return X3FRENDER_OK;
}

void x3f_render_spp_prepared_free(x3f_spp_prepared *handle)
{
    if (handle != NULL) x3f_spp_prepared_free(handle);
}
