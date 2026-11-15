/* X3F_SFD.C — X3I（SFD 七帧包围曝光）合成成一张
 *
 * 口径照 `SFDComposeWFA`（见
 * §2/§5，落地口径写在 include/x3f_render.h 的 x3f_render_load_sfd 注释里）：
 *
 *     LUT_A[i] = 1/(1+exp(0.005·(i−9000)))             高光软门
 *     LUT_B[i] = max(1/(1+exp(0.015·(300−i))), 1e-10)  只压很暗的一小段
 *     w        = LUT_A[V]·LUT_B[V] / (k²·σ²)
 *     acc_val += w·k·V ;  acc_w += w                   → 出图 = acc_val/acc_w
 *     k        = 本帧快门 ÷ 基准帧快门（CAMF 的 CaptureShutter，µs；实测是精确的 2 的幂）
 *
 * 三条与 SPP 对齐、但用我们自己的量实现的细节：
 *   · V 用 DN 当量：V = lin · 16383 / LIN_SAT。LIN_SAT（饱和点的线性值）从第 0 帧
 *     解码随手带出来的饱和分（clip 通道，1.0 = 顶到传感器上限）**实测反解**，
 *     不写死常数 —— 换机身/ISO 都不用改。
 * · σ² = a + b·V + c·V²，系数取相机自己的 ColorNoiseModel（Quattro 走
 *     那组常量），与降噪那一路同源（x3f_render_noise_coef_for）。
 *   · 七帧同机同标定、且**像素级对齐**（实测 NCC 峰值都在 0 位移），所以不做配准。
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(_WIN32)
#include <pthread.h>
#define SFD_HAVE_THREADS 1
#else
/* ── Windows 最小 pthread 垫片（只覆盖本文件用到的 create/join）────────
 * 与 own_decoder.c 同一份做法：_beginthreadex 的线程函数是
 * __stdcall unsigned(*)(void*)，pthread 是 void*(*)(void*)——中间垫一层
 * trampoline 转换；join 等结束后回收句柄。双槽预取的线程全部 create 后
 * 必 join（sfd_pipe_drain），不做 detach。 */
#include <windows.h>
#include <process.h>
/* windef.h 会把 near/far 定义成空宏（16 位时代的兼容残留），本文件的
 * 局部变量 `near` 会被它展开坏（near[p] → [p]）——这里立即拆掉。 */
#undef near
#undef far
typedef HANDLE pthread_t;
typedef struct {
    void *(*fn)(void *);
    void *arg;
} win_thunk_t;

static unsigned __stdcall win_thunk(void *p)
{
    win_thunk_t t = *(win_thunk_t *)p;
    free(p);
    t.fn(t.arg);
    return 0;
}

static int pthread_create(pthread_t *th, const void *attr,
                          void *(*fn)(void *), void *arg)
{
    (void)attr;
    win_thunk_t *t = (win_thunk_t *)malloc(sizeof(win_thunk_t));
    if (t == NULL) return -1;
    t->fn = fn; t->arg = arg;
    uintptr_t h = _beginthreadex(NULL, 0, win_thunk, t, 0, NULL);
    if (h == 0) { free(t); return -1; }
    *th = (pthread_t)h;
    return 0;
}

static int pthread_join(pthread_t th, void **value_ptr)
{
    (void)value_ptr;
    WaitForSingleObject(th, INFINITE);
    CloseHandle(th);
    return 0;
}
/* 双槽预取流水线 Windows 也走（与 Mac 同一套行为）；X3F_SFD_PAIR=0 仍可
 * 回退串行做对照。 */
#define SFD_HAVE_THREADS 1
#endif

#include "x3fdec_alias.h"
#include "x3fcore.h"
#include "x3f_io.h"
#include "x3f_process.h"
#include "x3f_render.h"
#include "x3f_parallel.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define X3F_SFD_MAX_FRAMES 8

/* ── 分阶段计时（X3F_SFD_TIMING=1 打开，stderr 打一行汇总）──────────────
 * 累计量只由调用 x3f_render_load_sfd_ex 的那个线程更新：
 *   · 临时帧提取在 sfd_job_start 里（主线程）；
 *   · 解码耗时在 pthread_join 两端量 —— 双槽流水线下 join 等到的是后台
 *     线程的解码墙钟；线程内再拆「解码分发 / 渲染后处理」两半（分发耗时
 *     读 x3f_render_last_decode_ms()，x3f_render.c 在 load_common 里埋的
 *     线程局部桩），随 job 结构带回主线程，不共享写。
 * 解码墙钟合计 + 各处理段加起来 ≈ 总墙钟（流水线让两者部分重叠，正好用
 * 差值看「预取有没有把解码藏住」）。 */
static int sfd_timing(void)
{
    const char *e = getenv("X3F_SFD_TIMING");
    return e != NULL && *e != '\0';
}

#if defined(_WIN32)
static double sfd_now_ms(void)
{
    LARGE_INTEGER f, c;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart * 1000.0 / (double)f.QuadPart;
}
#else
#include <time.h>
static double sfd_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}
#endif

static double g_t_camf;      /* CaptureShutter 读取（逐帧临时文件 + CAMF 解码） */
static double g_t_extract;   /* 临时单帧 X3F 落盘（读容器 + 写 %TEMP%） */
static double g_t_decode;    /* 帧循环里的 join 等待（主线程视角的解码墙钟） */
static double g_t_decode_sum;/* 各解码线程自报的耗时合计 */
static double g_t_dec_disp;  /* 其中「解码分发」（own/x3fuse 那一下） */
static double g_t_passa;     /* Pass A：权重 / 可用帧 / 下界 */
static double g_t_fuse;      /* 逐帧融合回调（App 路径恒空） */
static double g_t_passb;     /* Pass B：加权和累加 */
static double g_t_norm;      /* 归一 + 下界 5×5 中位 */
static double g_t_stage1;    /* Stage1 多尺度降噪 */
static double g_t_edge;      /* 高光边缘轻平滑 */
static double g_t_tail;      /* 软膝收峰 + 收色 + 峰值扫描 */
static double g_t_total;     /* 整个函数的墙钟 */

/* CAMF 取数的 FFI 尺寸参数：**写满 3 个 u32 才对**（见 x3f_render.c 里的同名宏） */
#define X3F_CAMF_DIMS 3

/*"这一帧在这个通道上还算数吗" 的门限（作用在 LUT_A 上）。
 *
 * 为什么需要它：LUT_A 是一条**软**门（1/(1+exp(0.005·(V−9000)))），它只把接近
 * 饱和的帧"调小"，并不真的排除。当七帧在某个通道上**同时**被压到 1e-8 量级时，
 * 剩下的权重差别就只有 1/k²（最多 4096 倍）—— 于是合成值由"曝光最长、最贴饱和
 * 的那一帧"决定，而那正是最不可信的一帧。实测 SDQ_0665 的水桶高光：
 * R 通道七帧的 k·lin（= 各帧对场景亮度的估计）是 1.05 / 7.83 / 6.21 / 1.88 /
 * 0.53 / 0.26 / 0.13，权重却是被切平的那几帧最大，输出 1.35 —— 比唯一还落在
 * 线性段上的 −3 EV 帧给出的 7.83 暗了 2.5 档，而且逐通道偏得不一样（画面里
 * 那一片就是"橙色发闷 + 方块状色斑"的来源）。
 *
 * 门限的取法：A ≥ 1e-3 等价于 V ≤ 10381（= 满量程 63%，而 V 是**乘过白平衡
 * 增益**的，对 R 这种增益 >1 的通道折回 RAW 域约 55% —— 正是 SPP 那条门的中
 * 心 9000/16383）。也就是"只把已经踩上饱和肩的帧排除掉"，线性段内的帧照旧
 * 按 权重加权。*/
#define X3F_SFD_GATE_USABLE 1e-3

/* 极端高光的**中性化**区间（线性域，1.0 = SDR 白）。见下面第 5 步的说明。
 * X3F_SFD_NEUTRAL="lo,hi" 可覆盖；填 "0,0" 等于关掉这条做 A/B。 */
#define X3F_SFD_NEUTRAL_LO 2.0
#define X3F_SFD_NEUTRAL_HI 8.0

/* 收色强度：只轻轻往中性靠（默认 0.3）。拉满会把核心拉成大白块 ——
 * 用户实测"爆的区域比 SPP 大得多"就是这个拉满造成的。 */
#define X3F_SFD_NEUTRAL_STR 0.3

static inline double sfd_sstep(double a, double b, double x)
{
    double t;
    if (b <= a) return x >= b ? 1.0 : 0.0;
    t = (x - a) / (b - a);
    if (t < 0.0) return 0.0;
    if (t > 1.0) return 1.0;
    return t * t * (3.0 - 2.0 * t);
}

/* 降噪模型的系数（实现在 x3f_render.c：和降噪那条链共用同一份标定） */
extern int x3f_render_noise_coef_for(const char *path, double out[9]);

/* 上一次「解码分发」（x3f_own_dispatch_scene_linear）的耗时（ms）。
 * x3f_render.c 里埋的桩，**线程局部**——各解码线程读自己那份。 */
extern double x3f_render_last_decode_ms(void);

/* own 解码后端的逐帧降噪开关（own_decoder.c）。SFD 的口径：
 * 逐帧不做解码内降噪（fuse 后端在 nlm=0 时本来就这样），七帧合完由
 * Stage1 多尺度降噪一次收掉 —— 逐帧跑 own 的 XFCHAIN NLM 是七遍钱，
 * own 之前忽略 nlm 参数导致每帧都付，这是它比 fuse 合成慢的大头。 */
extern void own_set_frame_nr(int on);

/* GPU 加速层的**线程局部**开关（own_gpu_win.c / own_gpu.m）。SFD 的几个
 * 解码线程同时派发会把 GPU 打爆（实测比纯 CPU 慢 3~5 倍），所以它们各自
 * 关掉自己那份；主线程不关，单帧解码/预览/导出渲染照旧吃 GPU。
 * 说明见 own_gpu.h 的同名条目。 */
extern void own_gpu_thread_disable(int on);

/* CAMF 矩阵取值（Rust FFI，定义在 libx3f 里）；dims 是入参也是出参 */
extern int x3f_camf_matrix_f64(const char *path, const char *name,
                               double *out, int cap, unsigned *dims);

/* SPP 那两张表各 65536 项，建一次就够（SPP 也是"只在第一帧建"） */
static double g_lut_a[65536];
static double g_lut_b[65536];
static int    g_lut_built = 0;

/* 上一次合成用的"缩放到饱和值"系数（只给诊断串用；见 §4 那段注释） */
static double g_sfd_norm = 1.0;

/* 上一次合成的**物理**峰值（1.0 = SDR 白，可以 >1）：归一撤掉之后它就是诊断量 */
static double g_sfd_peak = 0.0;

/* 上一次合成里被中性化（收色）的像素数，只给诊断用 */
static long g_sfd_neutral = 0;

/* 诊断计数：这一张里有多少个通道/像素走了"全被门挡住 → 下界"那条路 */
static long g_sfd_fb_ch = 0, g_sfd_fb_px = 0;

/* 核心重建的统计（只给诊断串用） */
static long g_sfd_core_px = 0, g_sfd_core_chg = 0;

/* ── SFD 帧对齐（借鉴 x3fuse-core 0.4.0 相位相关 + 双三次 warp，2026-10-29）──
 * 默认关。X3F_SFD_ALIGN=1 → 全局模式（每帧单位移）；=local/2 → 局部平滑位移场
 * （借鉴 0.4.0 X3F_X3I_ALIGN=local：逐瓦片质量门 + 邻域正则补洞 + 有界梯度 +
 *   双线性场插值 + 逐像素 Catmull-Rom warp）。参考帧 = order[0]（累加器网格）。
 * 零位移素材逐位不变（warp 在 |shift|<1e-4 时原样透传）。 */
static int    g_sfd_align = 0;        /* 本次是否启用了对齐 */
static int    g_sfd_align_mode = 0;   /* 1=全局单位移 2=局部平滑场（随本次 load 生效） */
static int    g_sfd_align_abandon = 0; /* 因运动/位移过大放弃对齐 */
static double g_sfd_align_max = 0.0;   /* 应用的最大位移（全分辨率 px） */
/* 局部场存储：每帧 NT_L×NT_L 瓦片的位移（全分辨率 px，相位输出约定 r=-s）。
 * fld_has[i]=0 表示该帧无场（回退单位移 sfd_sx/sy）。 */
#define SFD_FLD_NT 8
static double g_sfd_fldx[X3F_SFD_MAX_FRAMES][SFD_FLD_NT * SFD_FLD_NT];
static double g_sfd_fldy[X3F_SFD_MAX_FRAMES][SFD_FLD_NT * SFD_FLD_NT];
static int    g_sfd_fld_has[X3F_SFD_MAX_FRAMES];
/* 程序化门控覆盖：见 x3f_render_set_sfd_align。-1 = 未设置，回退环境变量；
   >=0 = 显式 0/1，优先于环境变量（产品用它把开关绑到前端选项）。
   显式开（1）选全局模式；local 目前仅供研究，经环境变量启用。 */
static int    g_sfd_align_override = -1;

static int sfd_debug(void);

static double sfd_lum3(const float *p)
{
    return 0.2126 * p[0] + 0.7152 * p[1] + 0.0722 * p[2];
}

/* 核心重建：**局部中位去亮丝 + 径向参考补回退**，只在极端高光里动手。
 *
 * 触发条件（两层都要满足）：
 *   · 亮度 > 0.5（1.0 = SDR 白）——普通画面不动；
 *   · 参与帧数 ≤ 2——这一格要么只剩最欠曝的一帧，要么两帧拼贴。
 *
 * 做法分两条：
 *   · used≥1：5×5 逐通道中位（保边），混 75%。2~3 像素的亮丝/颗粒被投掉，
 *     ≥5 像素宽的环纹保留；输出是真实邻域值，不会引入椒盐。
 *   · used=0（全帧回退）：用"同半径 ± 3σ"的加权均值（椭圆拟合 + 剔除离群），
 *     因为这一格没有任何可信亮度。
 *
 * 走过的三条死路（避免下一轮重复）：
 *   · 整片等半径均值 → 环纹被抹平（用户："越改越差"）；
 *   · 只替换 >50% 的离群值 → 二值掩膜造成椒盐（边界换、邻格不换）；
 *   · R/Y、B/Y 比值归一 → B 为负的核心除法放大到 1e35。
 *
 * 默认关；X3F_SFD_CORE=1 打开，X3F_SFD_CORE_STR 缩放强度。 */
static void sfd_core_rebuild(float *img, const unsigned char *used, int w, int h)
{
    const char *env = getenv("X3F_SFD_CORE");
    const char *senv = getenv("X3F_SFD_CORE_STR");
    double strength = 1.0;
    size_t np = (size_t)w * (size_t)h, i;
    unsigned char *mask = NULL, *seen = NULL;
    int *stk = NULL;
    double *med = NULL, *med7 = NULL;
    double *samp = NULL;

    g_sfd_core_px = g_sfd_core_chg = 0;
    /* 默认关：三条重建路线里只有"5×5 中位"是安全的，但它是**温和**改善
     * （亮丝没了、角向噪声 −37%、环纹略软），还不足以替掉默认出图。
     * 要用时 X3F_SFD_CORE=1。 */
    if (env == NULL || *env == '\0' || atof(env) == 0.0) return;
    if (senv != NULL && *senv != '\0') strength = atof(senv);
    if (!(strength > 0.0)) return;

    mask = (unsigned char *)malloc(np);
    seen = (unsigned char *)calloc(np, 1);
    if (mask == NULL || seen == NULL) { free(mask); free(seen); return; }
    for (i = 0; i < np; i++)
        mask[i] = (used[i] <= 2 && sfd_lum3(&img[i * 3]) > 0.5) ? 1 : 0;

    for (i = 0; i < np; i++) {
        size_t head = 0, tail = 0, cap = 1 << 12;
        double cx, cy, vx, vy, vxy, tr, det, l1, l2, ang, ca, sa, aa, bb;
        int x0 = w, y0 = h, x1 = -1, y1 = -1, k;
        if (!mask[i] || seen[i]) continue;
        stk = (int *)malloc(cap * sizeof(int));
        if (stk == NULL) break;
        seen[i] = 1;
        stk[tail++] = (int)i;
        while (head < tail) {
            int p = stk[head++], px = p % w, py = p / w, dy, dx;
            if (px < x0) x0 = px;
            if (py < y0) y0 = py;
            if (px > x1) x1 = px;
            if (py > y1) y1 = py;
            for (dy = -1; dy <= 1; dy++)
                for (dx = -1; dx <= 1; dx++) {
                    int nx = px + dx, ny = py + dy;
                    size_t nb;
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    nb = (size_t)ny * (size_t)w + (size_t)nx;
                    if (seen[nb] || !mask[nb]) continue;
                    seen[nb] = 1;
                    if (tail == cap) {
                        int *grown = (int *)realloc(stk, (cap * 2) * sizeof(int));
                        if (grown == NULL) { head = tail; break; }
                        stk = grown;
                        cap *= 2;
                    }
                    stk[tail++] = (int)nb;
                }
        }
        if (tail < 64 || tail == cap) { free(stk); stk = NULL; continue; }

        /* 质心 + 二阶矩 → 椭圆（a=2σ1、b=2σ2）。环是等 rn 线。 */
        {
            double sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, n = (double)tail;
            for (k = 0; k < (int)tail; k++) {
                int p = stk[k], px = p % w, py = p / w;
                sx += px; sy += py;
                sxx += (double)px * px; syy += (double)py * py; sxy += (double)px * py;
            }
            cx = sx / n; cy = sy / n;
            vx = sxx / n - cx * cx; vy = syy / n - cy * cy; vxy = sxy / n - cx * cy;
        }
        tr = vx + vy; det = vx * vy - vxy * vxy;
        l1 = 0.5 * (tr + sqrt(fmax(0.0, tr * tr - 4.0 * det)));
        l2 = 0.5 * (tr - sqrt(fmax(0.0, tr * tr - 4.0 * det)));
        aa = 2.0 * sqrt(fmax(l1, 1.0));
        bb = 2.0 * sqrt(fmax(l2, 1.0));
        if (aa / bb > 4.0 || bb / aa > 4.0) { free(stk); stk = NULL; continue; }
        ang = 0.5 * atan2(2.0 * vxy, vx - vy);
        ca = cos(ang); sa = sin(ang);
        med = (double *)malloc((size_t)tail * 3 * sizeof(double));
        med7 = (double *)malloc((size_t)tail * 3 * sizeof(double));
        samp = (double *)malloc((size_t)(2 * 24 + 1) * (2 * 24 + 1) * 3 * sizeof(double));
        if (med == NULL || med7 == NULL || samp == NULL) {
            free(med); free(med7); free(samp); free(stk); break;
        }
        {
            size_t j;
            int R = 20;
            for (j = 0; j < tail; j++) {
                int p = stk[j], px = p % w, py = p / w;
                double dx = px - cx, dy = py - cy;
                double rp = sqrt(pow((dx * ca + dy * sa) / aa, 2.0)
                                 + pow((-dx * sa + dy * ca) / bb, 2.0));
                int n = 0;
                /* 亮度与色度分开做：
                 *   Y = Rec709 亮度，5×5 中位 —— 投掉 2~3 px 亮丝，保 ≥5 px 环纹；
                 *   U = R−G、V = B−G，9×9 中位 —— 色度允许大核，专门抹掉
                 *       "逐像素挑帧"在同一圈上撕出来的黄/橙色块。
                 * med7[0..2] = Y/U/V 的参考值。 */
                {
                    double ys[25], us[81], vs[81];
                    int ny = 0, nc = 0, m, ky, kx;
                    for (ky = -4; ky <= 4; ky++) {
                        int qy = py + ky;
                        if (qy < 0 || qy >= h) continue;
                        for (kx = -4; kx <= 4; kx++) {
                            int qx = px + kx;
                            size_t q;
                            double r, g, b;
                            if (qx < 0 || qx >= w) continue;
                            q = (size_t)qy * (size_t)w + (size_t)qx;
                            if (sfd_lum3(&img[q * 3]) < 0.3) continue;
                            r = img[q * 3]; g = img[q * 3 + 1]; b = img[q * 3 + 2];
                            us[nc] = r - g;
                            vs[nc] = b - g;
                            nc++;
                            if (ky >= -2 && ky <= 2 && kx >= -2 && kx <= 2)
                                ys[ny++] = 0.2126 * r + 0.7152 * g + 0.0722 * b;
                        }
                    }
                    for (m = 1; m < ny; m++) {          /* 插入排序，ny≤25 */
                        double v = ys[m];
                        int b2 = m - 1;
                        while (b2 >= 0 && ys[b2] > v) { ys[b2 + 1] = ys[b2]; b2--; }
                        ys[b2 + 1] = v;
                    }
                    for (m = 1; m < nc; m++) {          /* nc≤81 */
                        double v = us[m];
                        int b2 = m - 1;
                        while (b2 >= 0 && us[b2] > v) { us[b2 + 1] = us[b2]; b2--; }
                        us[b2 + 1] = v;
                    }
                    for (m = 1; m < nc; m++) {
                        double v = vs[m];
                        int b2 = m - 1;
                        while (b2 >= 0 && vs[b2] > v) { vs[b2 + 1] = vs[b2]; b2--; }
                        vs[b2 + 1] = v;
                    }
                    med7[j * 3]     = ny ? ys[ny / 2] : sfd_lum3(&img[p * 3]);
                    med7[j * 3 + 1] = nc ? us[nc / 2] : 0.0;
                    med7[j * 3 + 2] = nc ? vs[nc / 2] : 0.0;
                }
                {
                    /* 软权重 + 一次离群剔除：硬边界的"等半径中位"会在亮核上留下
                     * 一圈圈方块（采样集合随 rn 跳变），软权重把这件事抹平。
                     * dr 的单位是像素：|dr|>3σ 的样本直接不收。 */
                    double wsum = 0.0, mean[3] = { 0, 0, 0 }, var[3] = { 0, 0, 0 };
                    double wt[1681];
                    int m, c;
                    const double sig_px = 1.6;
                    for (k = -R; k <= R && n < 1681; k++) {
                        int qy = py + k, kx;
                        if (qy < 0 || qy >= h) continue;
                        for (kx = -R; kx <= R && n < 1681; kx++) {
                            int qx = px + kx;
                            size_t q;
                            double dqx, dqy, rq, dr;
                            if (qx < 0 || qx >= w) continue;
                            q = (size_t)qy * (size_t)w + (size_t)qx;
                            if (sfd_lum3(&img[q * 3]) < 0.3) continue;
                            dqx = qx - cx; dqy = qy - cy;
                            rq = sqrt(pow((dqx * ca + dqy * sa) / aa, 2.0)
                                      + pow((-dqx * sa + dqy * ca) / bb, 2.0));
                            dr = (rq - rp) * 0.5 * (aa + bb) / sig_px;
                            if (dr > 3.0 || dr < -3.0) continue;
                            samp[n * 3] = img[q * 3];
                            samp[n * 3 + 1] = img[q * 3 + 1];
                            samp[n * 3 + 2] = img[q * 3 + 2];
                            wt[n] = exp(-0.5 * dr * dr);
                            wsum += wt[n];
                            n++;
                        }
                    }
                    for (m = 0; m < n; m++)
                        for (c = 0; c < 3; c++) mean[c] += wt[m] * samp[m * 3 + c];
                    if (wsum > 0.0)
                        for (c = 0; c < 3; c++) mean[c] /= wsum;
                    else
                        for (c = 0; c < 3; c++) mean[c] = img[p * 3 + c];
                    for (m = 0; m < n; m++)
                        for (c = 0; c < 3; c++) {
                            double d = samp[m * 3 + c] - mean[c];
                            var[c] += wt[m] * d * d;
                        }
                    if (wsum > 0.0)
                        for (c = 0; c < 3; c++) var[c] = sqrt(var[c] / wsum);
                    /* 剔除 >2.2σ 的离群样本（亮丝就是这里的 3~5σ），再算一次 */
                    {
                        double s2 = 0.0, mu[3] = { 0, 0, 0 };
                        for (m = 0; m < n; m++) {
                            int in = 1;
                            for (c = 0; c < 3; c++) {
                                double lim = fmax(2.2 * var[c], 0.02 * fmax(1.0, mean[c]));
                                if (fabs(samp[m * 3 + c] - mean[c]) > lim) in = 0;
                            }
                            if (!in) continue;
                            s2 += wt[m];
                            for (c = 0; c < 3; c++) mu[c] += wt[m] * samp[m * 3 + c];
                        }
                        if (s2 > 0.2) {
                            for (c = 0; c < 3; c++)
                                med[j * 3 + c] = fmax(0.0, mu[c] / s2);
                        } else {
                            for (c = 0; c < 3; c++)
                                med[j * 3 + c] = fmax(0.0, mean[c]);
                        }
                    }
                }
            }
        }
        {
            size_t j;
            for (j = 0; j < tail; j++) {
                int p = stk[j], c;
                /* 亮度走 Y 参考（5×5 中位），色度走 U/V 参考（9×9 中位）。
                 * 一步步踩过的坑都在这里，避免下一轮重复：
                 *   · 整片等半径均值 → 环纹被抹平（"越改越差"）；
                 *   · 只替换 >50% 的离群值 → 二值掩膜造成椒盐（"v4 效果很差"）；
                 *   · R/Y、B/Y 比值归一 → B 为负的核心除法放大成 1e35；
                 *   · RGB 逐通道中位 → 大核才能压色块，但这会把环纹一起糊掉。
                 * 分开做之后：Y 只动 5×5，U/V 才用 9×9；亮度细节原样保留，
                 * 色度允许更重的手。used==0 仍走径向参考（那一格没有可信亮度）。 */
                double wgt = 0.0;
                wgt = strength;
                if (wgt > 1.0) wgt = 1.0;
                g_sfd_core_px++;
                if (wgt > 0.0) g_sfd_core_chg++;
                if (used[p] == 0) {
                    for (c = 0; c < 3; c++) {
                        double v = img[p * 3 + c];
                        v += (med[j * 3 + c] - v) * (0.90 * wgt);
                        img[p * 3 + c] = (float)v;
                    }
                } else {
                    double r = img[p * 3], g = img[p * 3 + 1], b = img[p * 3 + 2];
                    double y = 0.2126 * r + 0.7152 * g + 0.0722 * b;
                    double u = r - g, v = b - g;
                    double y2 = y + (med7[j * 3] - y) * (0.75 * wgt);
                    double u2 = u + (med7[j * 3 + 1] - u) * (0.60 * wgt);
                    double v2 = v + (med7[j * 3 + 2] - v) * (0.60 * wgt);
                    double g2 = y2 - 0.2126 * u2 - 0.0722 * v2;
                    img[p * 3]     = (float)(g2 + u2);
                    img[p * 3 + 1] = (float)g2;
                    img[p * 3 + 2] = (float)(g2 + v2);
                }
            }
        }
        if (sfd_debug())
            fprintf(stderr, "[core] 重建组件 %zu 像素 bbox=(%d,%d)-(%d,%d) 椭圆 %.1f×%.1f\n",
                    tail, x0, y0, x1, y1, aa, bb);
        free(med); med = NULL;
        free(med7); med7 = NULL;
        free(samp); samp = NULL;
        free(stk); stk = NULL;
    }
    free(mask);
    free(seen);
    free(samp);
    free(med);
    free(med7);
}

/* 高光边缘轻平滑（默认开）：只在 used≤2 的亮部**边界 4 像素带**内做一次
 * 3×3 高斯混 50%，把核心与周围之间那圈台阶/碎边稍微抹顺。
 *
 * 为什么只做边缘：核心内部的环纹和亮丝都归 sfd_core_rebuild()（默认关），
 * 这里不碰内部；带外（正常画面）一个像素都不动。
 * X3F_SFD_EDGE=0 关掉，X3F_SFD_EDGE_STR 调强度（默认 0.5）。 */
static void sfd_edge_smooth(float *img, const unsigned char *used, int w, int h)
{
    const char *env = getenv("X3F_SFD_EDGE");
    const char *senv = getenv("X3F_SFD_EDGE_STR");
    double str = 0.5;
    size_t np = (size_t)w * (size_t)h, i;
    unsigned char *mask, *near, *seed;
    float *src;
    int x, y;

    if (env != NULL && *env != '\0' && atof(env) == 0.0) return;
    if (senv != NULL && *senv != '\0') str = atof(senv);
    if (!(str > 0.0)) return;
    mask = (unsigned char *)malloc(np);
    near = (unsigned char *)calloc(np, 1);
    seed = (unsigned char *)calloc(np, 1);
    src = (float *)malloc(np * 3 * sizeof(float));
    if (mask == NULL || near == NULL || seed == NULL || src == NULL) {
        free(mask); free(near); free(seed); free(src);
        return;
    }
    /* 只在"真正爆掉的核心"旁边动：以 used==0 的像素为种子，向外 24 px 内
     * 才算候选。否则建筑窗格这类"只被切平几帧"的亮块也会被当成高光边缘，
     * 把窗框一起抹软（实测第一版就是这样）。 */
    for (i = 0; i < np; i++) {
        if (used[i] != 0) continue;
        {
            int sx = (int)(i % (size_t)w), sy = (int)(i / (size_t)w);
            int y2, x2;
            for (y2 = sy - 24; y2 <= sy + 24; y2++) {
                if (y2 < 0 || y2 >= h) continue;
                for (x2 = sx - 24; x2 <= sx + 24; x2++) {
                    if (x2 < 0 || x2 >= w) continue;
                    seed[(size_t)y2 * (size_t)w + (size_t)x2] = 1;
                }
            }
        }
    }
    for (i = 0; i < np; i++)
        mask[i] = (used[i] <= 2 && sfd_lum3(&img[i * 3]) > 0.5 && seed[i]) ? 1 : 0;
    /* 可分离的 5×5 膨胀/腐蚀：横一趟、竖一趟，比逐像素 25 邻域快 2.5 倍 */
    {
        unsigned char *tmp = (unsigned char *)calloc(np, 1);
        if (tmp == NULL) { free(mask); free(near); free(seed); free(src); return; }
        for (y = 0; y < h; y++)
            for (x = 0; x < w; x++) {
                int dx, any = 0;
                size_t p = (size_t)y * (size_t)w + (size_t)x;
                for (dx = -2; dx <= 2 && !any; dx++) {
                    int nx = x + dx;
                    if (nx >= 0 && nx < w) any = mask[(size_t)y * (size_t)w + (size_t)nx];
                }
                tmp[p] = (unsigned char)any;
            }
        for (y = 0; y < h; y++)
            for (x = 0; x < w; x++) {
                int dy, any = 0;
                size_t p = (size_t)y * (size_t)w + (size_t)x;
                for (dy = -2; dy <= 2 && !any; dy++) {
                    int ny = y + dy;
                    if (ny >= 0 && ny < h) any = tmp[(size_t)ny * (size_t)w + (size_t)x];
                }
                near[p] = (unsigned char)any;
            }
        free(tmp);
    }
    memcpy(src, img, np * 3 * sizeof(float));
    for (y = 1; y < h - 1; y++)
        for (x = 1; x < w - 1; x++) {
            size_t p = (size_t)y * (size_t)w + (size_t)x;
            int c, dy, dx;
            if (!near[p]) continue;
            {
                int edge = 0;
                for (dy = -2; dy <= 2 && !edge; dy++)
                    for (dx = -2; dx <= 2 && !edge; dx++) {
                        int nx = x + dx, ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= w || ny >= h) edge = 1;
                        else if (!mask[(size_t)ny * (size_t)w + (size_t)nx]) edge = 1;
                    }
                if (!edge) continue;                 /* 内部像素不动 */
            }
            for (c = 0; c < 3; c++) {
                double s = 0.0;
                for (dy = -1; dy <= 1; dy++)
                    for (dx = -1; dx <= 1; dx++) {
                        int wgt = (dx == 0 ? 2 : 1) * (dy == 0 ? 2 : 1);
                        s += wgt * src[(p + (size_t)dy * (size_t)w + (size_t)dx) * 3 + c];
                    }
                img[p * 3 + c] = (float)(src[p * 3 + c] + (s / 16.0 - src[p * 3 + c]) * str);
            }
        }
    free(mask); free(near); free(seed); free(src);
}

/* HLP 色比插值（照 `HLPsRestoCorr` 0x169c9c 的形状，见
 * §11）：
 *
 *     f = (C.R/C.G > A.R/A.G) ? W : 0
 *     t = max(A.R, (A.R + A.G·C.R/(C.G+1)) · 0.5)
 *     R' = A.R·(1−f) + f·t                       （B 同理）
 *     G' = A.G·(1−w) + w·max(A.G, (R'+B')/2)
 *
 * 这里 A 是核心像素，C 是**可信邻域**（used≥3）在局部窗口里的平均色。
 * 核心的 G 还敞着、R/B 往往被压扁（−3EV 帧的 B 接近 0），所以保留 G、
 * 用 C 的色比把 R/B 往上拉；只升不降，亮度不会塌。
 *
 * X3F_SFD_HLPCOL=1 打开（默认关，先看量），X3F_SFD_HLPCOL_STR 调权重（0.65）。 */
static void sfd_hlp_color(float *img, const unsigned char *used, int w, int h)
{
    const char *env = getenv("X3F_SFD_HLPCOL");
    const char *senv = getenv("X3F_SFD_HLPCOL_STR");
    double str = 0.65;
    size_t np = (size_t)w * (size_t)h, i;
    unsigned char *seed;
    long changed = 0;
    int x, y;

    if (env == NULL || *env == '\0' || atof(env) == 0.0) return;
    if (senv != NULL && *senv != '\0') str = atof(senv);
    if (!(str > 0.0)) return;
    seed = (unsigned char *)calloc(np, 1);
    if (seed == NULL) return;
    for (i = 0; i < np; i++) {
        if (used[i] != 0) continue;          /* 只围真正爆掉的核心 */
        {
            int sx = (int)(i % (size_t)w), sy = (int)(i / (size_t)w);
            int y2, x2;
            for (y2 = sy - 24; y2 <= sy + 24; y2++) {
                if (y2 < 0 || y2 >= h) continue;
                for (x2 = sx - 24; x2 <= sx + 24; x2++) {
                    if (x2 < 0 || x2 >= w) continue;
                    seed[(size_t)y2 * (size_t)w + (size_t)x2] = 1;
                }
            }
        }
    }
    for (y = 20; y < h - 20; y++)
        for (x = 20; x < w - 20; x++) {
            size_t p = (size_t)y * (size_t)w + (size_t)x;
            double ar, ag, ab, cr = 0.0, cg = 0.0, cb = 0.0, ws = 0.0;
            int dy, dx, n = 0;
            if (!seed[p] || used[p] > 2) continue;
            ar = img[p * 3]; ag = img[p * 3 + 1]; ab = img[p * 3 + 2];
            if (ar + ag + ab < 1.5 || ag <= 1e-6) continue;
            for (dy = -20; dy <= 20; dy++)
                for (dx = -20; dx <= 20; dx++) {
                    size_t q = (size_t)(y + dy) * (size_t)w + (size_t)(x + dx);
                    double r, g, b, wgt;
                    if (used[q] < 3) continue;           /* 可信邻域 */
                    r = img[q * 3]; g = img[q * 3 + 1]; b = img[q * 3 + 2];
                    if (r + g + b < 0.9) continue;
                    wgt = 1.0 / (1.0 + dx * dx + dy * dy);
                    cr += wgt * r; cg += wgt * g; cb += wgt * b; ws += wgt;
                    n++;
                }
            if (n < 40 || ws <= 0.0) continue;
            cr /= ws; cg /= ws; cb /= ws;
            if (cg <= 1e-6) continue;
            {
                double fR = (cr / cg > ar / ag) ? str : 0.0;
                double fB = (cb / cg > ab / ag) ? str : 0.0;
                double tR = fmax(ar, (ar + ag * cr / (cg + 1.0)) * 0.5);
                double tB = fmax(ab, (ab + ag * cb / (cg + 1.0)) * 0.5);
                double r2 = ar * (1.0 - fR) + fR * tR;
                double b2 = ab * (1.0 - fB) + fB * tB;
                double g2 = ag;      /* G 是唯一还敞着的通道，原样保留 */
                img[p * 3]     = (float)fmax(0.0, r2);
                img[p * 3 + 1] = (float)fmax(0.0, g2);
                img[p * 3 + 2] = (float)fmax(0.0, b2);
                if (fR > 0.0 || fB > 0.0) changed++;
            }
        }
    free(seed);
    if (sfd_debug())
        fprintf(stderr, "[hlp] 色比插值：%ld 个像素改了 R/B\n", changed);
}

/* 核心诊断：X3F_SFD_CORE_DEBUG=1 时，把"只剩一帧可用"的那片掩膜与径向剖面料出来。
 *
 * "只剩一帧可用"不是普通的暗部/亮部，而是**所有长曝光帧都被饱和门挡住**、只剩最
 * 欠曝的那一帧还落在传感器线性段上的地方。那一帧对 R 还有效，对 B 往往已经在噪声
 * 地板附近（实测 −3 EV 帧在同一格的 B 是 0.00 / −0.04），于是逐像素挑帧的结果就是
 * "R 来自这一帧、B 来自那一帧"的拼贴 —— 屏幕上那片碎。 */
static void sfd_core_probe(const float *img, const unsigned char *used, int w, int h)
{
    long hist[9] = { 0 };
    size_t np = (size_t)w * (size_t)h, i;
    unsigned char *seen;
    int *q;
    long best_area = 0;
    size_t best_start = 0;
    size_t j;

    for (i = 0; i < np; i++) hist[used[i] > 8 ? 8 : used[i]]++;
    fprintf(stderr, "[core] 参与帧数直方图：");
    for (i = 0; i <= 8; i++) fprintf(stderr, " %zu:%ld", i, hist[i]);
    fprintf(stderr, "\n");

    seen = (unsigned char *)calloc(np, 1);
    q = (int *)malloc(np * sizeof(int));
    if (seen == NULL || q == NULL) { free(seen); free(q); return; }
    for (i = 0; i < np; i++) {
        long area = 0;
        size_t head = 0, tail = 0;
        int x0 = w, y0 = h, x1 = -1, y1 = -1;
        if (used[i] > 1 || seen[i]) continue;
        seen[i] = 1;
        q[tail++] = (int)i;
        while (head < tail) {
            int p = q[head++], px = p % w, py = p / w, dy, dx;
            area++;
            if (px < x0) x0 = px;
            if (py < y0) y0 = py;
            if (px > x1) x1 = px;
            if (py > y1) y1 = py;
            for (dy = -1; dy <= 1; dy++)
                for (dx = -1; dx <= 1; dx++) {
                    int nx = px + dx, ny = py + dy;
                    size_t nb;
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    nb = (size_t)ny * (size_t)w + (size_t)nx;
                    if (seen[nb] || used[nb] > 1) continue;
                    seen[nb] = 1;
                    q[tail++] = (int)nb;
                }
        }
        if (area >= 8)
            fprintf(stderr, "[core] 组件：%ld 像素，bbox=(%d,%d)-(%d,%d)\n",
                    area, x0, y0, x1, y1);
        if (area > best_area) { best_area = area; best_start = i; }
    }
    if (best_area >= 8) {
        double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0, n = 0.0;
        double cx, cy, vx, vy, vxy, tr, det, l1, l2, ax, ay;
        size_t head = 0, tail = 0;
        memset(seen, 0, np);
        seen[best_start] = 1;
        q[tail++] = (int)best_start;
        while (head < tail) {
            int p = q[head++], px = p % w, py = p / w, dy, dx;
            sx += px; sy += py; sxx += (double)px * px; syy += (double)py * py;
            sxy += (double)px * py; n += 1.0;
            for (dy = -1; dy <= 1; dy++)
                for (dx = -1; dx <= 1; dx++) {
                    int nx = px + dx, ny = py + dy;
                    size_t nb;
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    nb = (size_t)ny * (size_t)w + (size_t)nx;
                    if (seen[nb] || used[nb] > 1) continue;
                    seen[nb] = 1;
                    q[tail++] = (int)nb;
                }
        }
        cx = sx / n; cy = sy / n;
        vx = sxx / n - cx * cx; vy = syy / n - cy * cy; vxy = sxy / n - cx * cy;
        tr = vx + vy; det = vx * vy - vxy * vxy;
        l1 = 0.5 * (tr + sqrt(fmax(0.0, tr * tr - 4.0 * det)));
        l2 = 0.5 * (tr - sqrt(fmax(0.0, tr * tr - 4.0 * det)));
        ax = 2.0 * sqrt(fmax(l1, 1e-6));
        ay = 2.0 * sqrt(fmax(l2, 1e-6));
        fprintf(stderr, "[core] 最大组件 %ld 像素：中心 (%.1f,%.1f) 轴 %.1f×%.1f\n",
                best_area, cx, cy, ax, ay);
        /* 径向剖面：rn = 0…1.6，0.1 一格。rel = used>=2（多帧同意）的那批。 */
        {
            double sum[16][3] = {{0}}, srel[16][3] = {{0}};
            long na[16] = {0}, nr[16] = {0};
            int bx0 = (int)(cx - ax * 1.7), bx1 = (int)(cx + ax * 1.7);
            int by0 = (int)(cy - ay * 1.7), by1 = (int)(cy + ay * 1.7);
            int y, x, b, c;
            if (bx0 < 0) bx0 = 0;
            if (by0 < 0) by0 = 0;
            if (bx1 > w - 1) bx1 = w - 1;
            if (by1 > h - 1) by1 = h - 1;
            for (y = by0; y <= by1; y++)
                for (x = bx0; x <= bx1; x++) {
                    double dx = (x - cx) / ax, dy = (y - cy) / ay;
                    double rn = sqrt(dx * dx + dy * dy);
                    size_t p = (size_t)y * (size_t)w + (size_t)x;
                    if (rn >= 1.6) continue;
                    b = (int)(rn * 10.0);
                    if (b > 15) b = 15;
                    for (c = 0; c < 3; c++) sum[b][c] += img[p * 3 + c];
                    na[b]++;
                    if (used[p] >= 2) {
                        for (c = 0; c < 3; c++) srel[b][c] += img[p * 3 + c];
                        nr[b]++;
                    }
                }
            fprintf(stderr, "[core] rn    n    均值 R/G/B（全部）        n_rel 均值 R/G/B（多帧）\n");
            for (b = 0; b < 16; b++) {
                fprintf(stderr, "[core] %.2f %5ld  %6.2f %6.2f %6.2f   %5ld  %6.2f %6.2f %6.2f\n",
                        (b + 0.5) * 0.1, na[b],
                        na[b] ? sum[b][0] / na[b] : 0.0,
                        na[b] ? sum[b][1] / na[b] : 0.0,
                        na[b] ? sum[b][2] / na[b] : 0.0,
                        nr[b],
                        nr[b] ? srel[b][0] / nr[b] : 0.0,
                        nr[b] ? srel[b][1] / nr[b] : 0.0,
                        nr[b] ? srel[b][2] / nr[b] : 0.0);
            }
        }
    }
    free(seen);
    free(q);

    /* 指定像素附近的"参与帧数"地图：看清核心到底是"只剩一帧"还是"两帧拼贴"。 */
    {
        const char *e = getenv("X3F_SFD_CORE_PX");
        int px, py;
        if (e != NULL && sscanf(e, "%d,%d", &px, &py) == 2) {
            int y, x;
            fprintf(stderr, "[core] 参与帧数地图 @(%d,%d)：0=回退 1/2=单/双帧 .=≥3 帧\n",
                    px, py);
            for (y = py - 24; y <= py + 24; y++) {
                fprintf(stderr, "  ");
                for (x = px - 32; x <= px + 32; x++) {
                    if (x < 0 || y < 0 || x >= w || y >= h) { fputc(' ', stderr); continue; }
                    {
                        unsigned u = used[(size_t)y * (size_t)w + (size_t)x];
                        fputc(u == 0 ? '0' : (u == 1 ? '1' : (u == 2 ? '2' : '.')), stderr);
                    }
                }
                fputc('\n', stderr);
            }
        }
    }
}

static void sfd_build_luts(void)
{
    int i;
    if (g_lut_built) return;
    for (i = 0; i < 65536; i++) {
        double b;
        g_lut_a[i] = 1.0 / (1.0 + exp(0.005 * ((double)i - 9000.0)));
        b = 1.0 / (1.0 + exp(0.015 * (300.0 - (double)i)));
        g_lut_b[i] = b > 1e-10 ? b : 1e-10;
    }
    g_lut_built = 1;
}

/* 反解"饱和点的线性值"（SFD 的 V = lin·16383/LIN_SAT 全靠它）。
 *
 * clip 是**相机域**的饱和分：1.0 = 顶到传感器上限，可以 >1（实测最大 3.0）。
 * 所以别拿线性域的通道去除它（那会串通道）。正确的读法很直接：**已经饱和的那批
 * 像素，它们的 maxchan(lin) 就是饱和点** —— 取中位。DP0Q0189 实测 ≈1.38（该图
 * 20% 的像素 clip≥0.9，正是切平的天空）。
 * 整幅都没饱和时用最亮值当饱和点：软门只会在最顶上关，宁可晚关一点。 */
static double sfd_lin_sat(const float *lin, const float *clip, size_t np)
{
    double vals[8192];
    int cnt = 0;
    double bright = 0.0;
    size_t i;
    int j;
    for (i = 0; i < np; i++) {
        double m = lin[i * 3];
        if (lin[i * 3 + 1] > m) m = lin[i * 3 + 1];
        if (lin[i * 3 + 2] > m) m = lin[i * 3 + 2];
        if (m > bright) bright = m;
        if (clip != NULL && clip[i] >= 0.9 && cnt < 8192) vals[cnt++] = m;
    }
    if (cnt >= 16) {
        for (j = 1; j < cnt; j++) {                /* 插入排序：量小，够用 */
            double v = vals[j];
            int k = j - 1;
            while (k >= 0 && vals[k] > v) { vals[k + 1] = vals[k]; k--; }
            vals[k + 1] = v;
        }
        return vals[cnt / 2];
    }
    return bright > 1e-6 ? bright : 1.0;
}

/* 读一个 double 型环境变量（没有就用默认值） */
static double s1_env_num_x(const char *name, double dflt)
{
    const char *e = getenv(name);
    if (e == NULL || *e == '\0') return dflt;
    return atof(e);
}

/* 诊断：X3F_SFD_DEBUG=1 时把权重链路摊开（只打少数像素，别刷屏） */
static int sfd_debug(void)
{
    const char *e = getenv("X3F_SFD_DEBUG");
    return e != NULL && *e != '\0';
}

/* 诊断：X3F_SFD_DEBUG_PX="x,y" —— 只把这个像素的权重链路摊开（查高光色偏时用）。
 * 返回该像素的线性下标，没有设置就返回 (size_t)-1。 */
static size_t sfd_debug_px(int w)
{
    const char *e = getenv("X3F_SFD_DEBUG_PX");
    int x, y;
    if (e == NULL || *e == '\0') return (size_t)-1;
    if (sscanf(e, "%d,%d", &x, &y) != 2) return (size_t)-1;
    if (x < 0 || y < 0 || w <= 0) return (size_t)-1;
    return (size_t)y * (size_t)w + (size_t)x;
}

/* 本线程的解码要不要走 GPU（默认**不走**，见下面流水线的说明）。
 * 每次读环境变量：一帧一次，7 帧 7 次，比 static 缓存的初始化竞态便宜。 */
static int sfd_thread_gpu(void)
{
    const char *e = getenv("X3F_SFD_GPU");
    return (e != NULL && *e != '\0' && atoi(e) != 0);
}

/* ── 帧解码作业（N 帧并发流水线）─────────────────────────────
 * 一个槽 = 一份解码上下文（临时文件路径 + 解码输出 + 线程句柄）。
 * 线程函数内设置 own_set_frame_nr / own_gpu_thread_disable：它们都是
 * **线程局部**的，在主线程设置对后台线程无效。 */
typedef struct {
    char  tmp[1024];        /* x3f_standalone_path 的临时单帧 X3F 路径 */
    const char *fp;         /* 实际解码路径（= tmp，或非容器时 = 原 path） */
    const char *wb;
    const x3f_denoise_t *dn;
    int   down, nlm;
    int   nr_on;            /* own_set_frame_nr（解码线程内设置） */
    float *lin, *clip;
    int   fw, fh, rc;
    char  wbo[64];
    double dec_ms, disp_ms; /* 解码线程自报：load_wb 墙钟 / 其中解码分发 */
} sfd_dec_job;

static void *sfd_dec_thread(void *p)
{
    sfd_dec_job *j = (sfd_dec_job *)p;
    own_set_frame_nr(j->nr_on);
    /* 本线程绕开 GPU（默认）：N 个解码线程同时派发大缓冲会把 GPU 打爆，
     * 实测比纯 CPU 慢 3~5 倍。X3F_SFD_GPU=1 开回来做 A/B。 */
    own_gpu_thread_disable(!sfd_thread_gpu());
    {
        double d0 = sfd_now_ms();
        j->rc = x3f_render_load_wb(j->fp, j->down, j->wb, j->nlm, j->dn,
                                   &j->lin, &j->clip, NULL, NULL, NULL,
                                   &j->fw, &j->fh, j->wbo);
        j->dec_ms = sfd_now_ms() - d0;
        j->disp_ms = x3f_render_last_decode_ms();
    }
    own_set_frame_nr(1);
    return NULL;
}

/* 启动一帧后台解码。pre_tmp 非空 = shutter 循环已提取好的临时文件（所有权
 * 转移，不再重复提取）；空 = 照旧自己提取。返回 1 = 线程已启动；0 = 失败
 * （调用方回退同步解码）。 */
static int sfd_job_start(sfd_dec_job *j, pthread_t *th, const char *path,
                         int frame_idx, int down, const char *wb, int nlm,
                         const x3f_denoise_t *dn, int nr_on, char *pre_tmp)
{
    if (pre_tmp != NULL && pre_tmp[0] != '\0') {
        memcpy(j->tmp, pre_tmp, sizeof(j->tmp));
        j->fp = j->tmp;                 /* 复用分支也要给 fp 赋值（解码线程读它） */
        pre_tmp[0] = '\0';              /* 所有权转移 */
    } else {
        double t0 = sfd_now_ms();
        j->fp = x3f_standalone_path(path, frame_idx, j->tmp, sizeof(j->tmp));
        g_t_extract += sfd_now_ms() - t0;
    }
    j->down = down; j->wb = wb;
    j->nlm = (frame_idx == 0) ? nlm : 0;   /* 基准帧才带 nlm（同串行版口径） */
    j->dn = dn;
    j->nr_on = nr_on;
    j->lin = j->clip = NULL;
    j->fw = j->fh = 0;
    j->rc = X3FRENDER_ERR_DECODE;
    memset(j->wbo, 0, sizeof(j->wbo));
    if (pthread_create(th, NULL, sfd_dec_thread, j) != 0)
        return 0;
    return 1;
}

/* 排空流水线：join 在飞线程、释放滞留的帧与临时文件（fail 路径与收尾共用） */
static void sfd_pipe_drain(sfd_dec_job *jb, pthread_t *th, int *th_on, int nslot)
{
    int q;
    for (q = 0; q < nslot; q++) {
        if (th_on[q]) {
            pthread_join(th[q], NULL);
            th_on[q] = 0;
        }
        free(jb[q].lin);  jb[q].lin = NULL;
        free(jb[q].clip); jb[q].clip = NULL;
        x3f_temp_release(jb[q].tmp);
        jb[q].tmp[0] = '\0';
    }
}

/* ===== 逐像素段的分块并行（2026-10-23）=====
 *
 * 逐像素循环（Pass A/B、归一、下界中位、软膝、收色、峰值）都是"每格只写
 * 自己那格"的独立迭代，用 x3f_parallel_for 按块摊到多核。**逐位一致**的
 * 三个保证：
 *   · 每块内部仍按 j 升序、与串行版同样的算式 —— 浮点次序不变；
 *   · 跨帧的 first-write 语义（下界"先写的 = 最欠曝帧"）由**帧循环**的
 *     顺序保证，帧内并行不动它；
 *   · 计数器（fb_ch/fb_px/neutralized）与峰值 max 用块局部累计、块后串行
 *     汇总 —— 整数精确，max 可交换无舍入。
 * X3F_SERIAL 编译时 x3f_parallel_for 自动退回串行，用来逐位对拍。 */
#define SFD_NCHUNK 64

typedef struct {
    size_t np;
    const float *lin, *clip;          /* 本帧解码值 / 逐通道裁剪标记（可 NULL） */
    float *wmap, *accv, *accw;
    unsigned char *used;
    const double *coef;
    double k, inv_k2, scale_v, lin_sat;
    int frame_idx;
    size_t px_dbg;
} sfd_passa_ctx;

static void sfd_passa_chunk(void *p, size_t ci)
{
    sfd_passa_ctx *c = (sfd_passa_ctx *)p;
    size_t per = (c->np + SFD_NCHUNK - 1) / SFD_NCHUNK;
    size_t j0 = ci * per, j1 = j0 + per, j;
    if (j0 >= c->np) return;
    if (j1 > c->np) j1 = c->np;
    for (j = j0; j < j1; j++) {
        int cch, idx, cmax = 0;
        double vmax = 0.0, a_gate, s2, wgt;
        int px_clipped = (c->clip != NULL && c->clip[j] >= 0.9);
        /* **逐像素一次决定**：用最亮通道的 V / σ² 算这一帧的权重，三通道共用。
         *
         * 为什么不再逐通道各算各的：同一个暗像素上，帧4 的 R 已经踩到饱和肩
         * （V=9879 → 门 0.012）而它的 G/B 还敞着，于是 R 由 0EV 帧、G/B 由
         * +1EV 帧决定 —— 两帧对同一格给出的色比是 1:0.81:0.71 与 1:0.54:0.37，
         * 混出来就是那片不自然的绿色。权重共用之后，每个通道都是"同一组帧的
         * 加权平均"，色度不会再被不同帧撕开。 */
        for (cch = 0; cch < 3; cch++) {
            double v = (double)c->lin[j * 3 + cch] * c->scale_v;
            if (v > vmax) { vmax = v; cmax = cch; }
        }
        if (vmax <= 0.0) continue;
        idx = vmax > 65535.0 ? 65535 : (int)vmax;
        a_gate = g_lut_a[idx];
        /* 这一帧在这一格上不算数：像素被切平，或者**最亮通道**已经踩到饱和肩。
         * 只把它对场景亮度的估计（k·lin）并进下界（先写的 = 最欠曝的那帧——
         * 该顺序由外层帧循环保证，帧内并行不动它）。 */
        if (px_clipped || !(a_gate >= X3F_SFD_GATE_USABLE)) {
            if (c->used[j] == 0) {
                for (cch = 0; cch < 3; cch++) {
                    float *slot_p = px_clipped ? &c->accw[j * 3 + cch]
                                               : &c->accv[j * 3 + cch];
                    if (*slot_p == 0.0f)
                        *slot_p = (float)(-(px_clipped ? c->lin_sat * c->k
                                                       : c->k * (double)c->lin[j * 3 + cch]));
                }
            }
            if (j == c->px_dbg)
                fprintf(stderr, "[sfd px] 帧%d: 不算数（clip=%.2f Vmax=%.0f A=%.3g）\n",
                        c->frame_idx, c->clip != NULL ? (double)c->clip[j] : -1.0,
                        vmax, a_gate);
            continue;
        }
        if (c->used[j] == 0) {                 /* 第一个可用帧：清掉两个下界槽 */
            for (cch = 0; cch < 3; cch++) {
                c->accv[j * 3 + cch] = 0.0f;
                c->accw[j * 3 + cch] = 0.0f;
            }
        }
        s2 = c->coef[cmax] + c->coef[3 + cmax] * vmax + c->coef[6 + cmax] * vmax * vmax;
        if (!(s2 > 1e-9)) s2 = 1e-9;
        wgt = a_gate * g_lut_b[idx] * c->inv_k2 / s2;
        c->wmap[j] = (float)wgt;
        c->used[j]++;
        if (j == c->px_dbg)
            fprintf(stderr, "[sfd px] 帧%d: 参与 Vmax=%.0f s2=%.4g A=%.3g k=%.4g wgt=%.6g 原始lin=%.4f/%.4f/%.4f\n",
                    c->frame_idx, vmax, s2, a_gate, c->k, wgt,
                    (double)c->lin[j * 3], (double)c->lin[j * 3 + 1],
                    (double)c->lin[j * 3 + 2]);
    }
}

typedef struct {
    size_t np;
    const float *lin, *wmap;
    float *accv, *accw;
    double k, scale_v;
} sfd_passb_ctx;

static void sfd_passb_chunk(void *p, size_t ci)
{
    sfd_passb_ctx *c = (sfd_passb_ctx *)p;
    size_t per = (c->np + SFD_NCHUNK - 1) / SFD_NCHUNK;
    size_t j0 = ci * per, j1 = j0 + per, j;
    if (j0 >= c->np) return;
    if (j1 > c->np) j1 = c->np;
    for (j = j0; j < j1; j++) {
        int cch;
        float wgt = c->wmap[j];
        if (!(wgt > 0.0f)) continue;
        for (cch = 0; cch < 3; cch++) {
            double v = (double)c->lin[j * 3 + cch];
            c->accv[j * 3 + cch] += (float)((double)wgt * c->k * v * c->scale_v);
            c->accw[j * 3 + cch] += wgt;
        }
    }
}

typedef struct {
    size_t np;
    float *accv, *accw;
    double scale_v;
    long fb_ch[SFD_NCHUNK], fb_px[SFD_NCHUNK];   /* 每块自己的计数，块后串行汇总：
                                                  * 并发 += 同一字段是丢更新竞争 */
} sfd_norm_ctx;

static void sfd_norm_chunk(void *p, size_t ci)
{
    sfd_norm_ctx *c = (sfd_norm_ctx *)p;
    size_t per = (c->np + SFD_NCHUNK - 1) / SFD_NCHUNK;
    size_t j0 = ci * per, j1 = j0 + per, j;
    long fb_ch = 0, fb_px = 0;
    if (j0 >= c->np) return;
    if (j1 > c->np) j1 = c->np;
    for (j = j0; j < j1; j++) {
        int cch;
        for (cch = 0; cch < 3; cch++) {
            double aw = c->accw[j * 3 + cch];
            double av = (double)c->accv[j * 3 + cch];
            if (aw > 1e-12) {
                c->accv[j * 3 + cch] = (float)(av / aw / c->scale_v);
            } else if (av < 0.0) {
                c->accv[j * 3 + cch] = (float)(-av);
                fb_ch++;
            } else if (aw < 0.0) {
                c->accv[j * 3 + cch] = (float)(-aw);
                fb_ch++;
            } else {
                c->accv[j * 3 + cch] = 0.0f;
            }
        }
        if (c->accw[j * 3] <= 0.0f && c->accw[j * 3 + 1] <= 0.0f &&
            c->accw[j * 3 + 2] <= 0.0f)
            fb_px++;
    }
    c->fb_ch[ci] = fb_ch;
    c->fb_px[ci] = fb_px;
}

typedef struct {
    size_t np;
    int w, h;
    const float *src;                 /* accv 的快照，读旧值防串味 */
    float *accv;
    const float *accw;
} sfd_median_ctx;

static void sfd_median_chunk(void *p, size_t ci)
{
    sfd_median_ctx *c = (sfd_median_ctx *)p;
    size_t per = (c->np + SFD_NCHUNK - 1) / SFD_NCHUNK;
    size_t j0 = ci * per, j1 = j0 + per, j2;
    if (j0 >= c->np) return;
    if (j1 > c->np) j1 = c->np;
    for (j2 = j0; j2 < j1; j2++) {
        int x2, y2, cch, dy, dx;
        if (c->accw[j2 * 3] > 0.0f) continue;      /* 正常像素不动 */
        x2 = (int)(j2 % (size_t)c->w); y2 = (int)(j2 / (size_t)c->w);
        for (cch = 0; cch < 3; cch++) {
            double buf[25];
            int n = 0, i2, k2;
            for (dy = -2; dy <= 2; dy++)
                for (dx = -2; dx <= 2; dx++) {
                    int nx = x2 + dx, ny = y2 + dy;
                    size_t nb;
                    if (nx < 0 || ny < 0 || nx >= c->w || ny >= c->h) continue;
                    nb = (size_t)ny * c->w + nx;
                    if (c->accw[nb * 3] > 0.0f) continue;   /* 只跟同为下界的邻居 */
                    buf[n++] = c->src[nb * 3 + cch];
                }
            for (i2 = 1; i2 < n; i2++) {           /* 插入排序，n≤25 */
                double v = buf[i2];
                k2 = i2 - 1;
                while (k2 >= 0 && buf[k2] > v) { buf[k2 + 1] = buf[k2]; k2--; }
                buf[k2 + 1] = v;
            }
            if (n > 0) c->accv[j2 * 3 + cch] = (float)buf[n / 2];
        }
    }
}

typedef struct {
    size_t n3;                        /* np*3 */
    float *accv;
    double kn, span;
} sfd_knee_ctx;

static void sfd_knee_chunk(void *p, size_t ci)
{
    sfd_knee_ctx *c = (sfd_knee_ctx *)p;
    size_t per = (c->n3 + SFD_NCHUNK - 1) / SFD_NCHUNK;
    size_t k0 = ci * per, k1 = k0 + per, k2;
    if (k0 >= c->n3) return;
    if (k1 > c->n3) k1 = c->n3;
    for (k2 = k0; k2 < k1; k2++) {
        double v = c->accv[k2];
        if (v > c->kn)
            c->accv[k2] = (float)(c->kn + c->span * (1.0 - exp(-(v - c->kn) / c->span)));
    }
}

typedef struct {
    size_t np;
    float *accv;
    double lo, hi;
    long neutralized[SFD_NCHUNK];     /* 每块自己的计数，块后串行汇总 */
} sfd_neutral_ctx;

static void sfd_neutral_chunk(void *p, size_t ci)
{
    sfd_neutral_ctx *c = (sfd_neutral_ctx *)p;
    size_t per = (c->np + SFD_NCHUNK - 1) / SFD_NCHUNK;
    size_t j0 = ci * per, j1 = j0 + per, j;
    long neutralized = 0;
    if (j0 >= c->np) return;
    if (j1 > c->np) j1 = c->np;
    for (j = j0; j < j1; j++) {
        double r = c->accv[j * 3], g = c->accv[j * 3 + 1], b = c->accv[j * 3 + 2];
        double m = r > g ? r : g;
        double t;
        if (b > m) m = b;
        t = sfd_sstep(c->lo, c->hi, m) * X3F_SFD_NEUTRAL_STR;
        if (t <= 0.0) continue;
        if (t >= 1.0) {
            c->accv[j * 3] = c->accv[j * 3 + 1] = c->accv[j * 3 + 2] = (float)m;
        } else {
            c->accv[j * 3]     = (float)(r + (m - r) * t);
            c->accv[j * 3 + 1] = (float)(g + (m - g) * t);
            c->accv[j * 3 + 2] = (float)(b + (m - b) * t);
        }
        neutralized++;
    }
    c->neutralized[ci] = neutralized;
}

typedef struct {
    size_t np;
    const float *accv;
    double peak[SFD_NCHUNK];          /* 每块自己的 max，块后串行汇总：
                                       * max 可交换，串行归约无舍入分歧 */
} sfd_peak_ctx;

static void sfd_peak_chunk(void *p, size_t ci)
{
    sfd_peak_ctx *c = (sfd_peak_ctx *)p;
    size_t per = (c->np + SFD_NCHUNK - 1) / SFD_NCHUNK;
    size_t j0 = ci * per, j1 = j0 + per, j;
    double mx = 0.0;
    if (j0 >= c->np) return;
    if (j1 > c->np) j1 = c->np;
    for (j = j0; j < j1; j++) {
        double m = c->accv[j * 3];
        if (c->accv[j * 3 + 1] > m) m = c->accv[j * 3 + 1];
        if (c->accv[j * 3 + 2] > m) m = c->accv[j * 3 + 2];
        if (m > mx) mx = m;
    }
    c->peak[ci] = mx;
}

/* 分块并行的统一入口：nchunk 固定 64，x3f_parallel_for 自己按核数切线程。 */
static void sfd_par(size_t n, x3f_for_fn fn, void *ctx)
{
    if (n == 0) return;
    x3f_parallel_for(SFD_NCHUNK, fn, ctx);
}

/* ===========================================================================
 * SFD 帧对齐（借鉴 x3fuse-core 0.4.0 的相位相关 + 重采样 warp，2026-10-29）
 * ---------------------------------------------------------------------------
 * 默认关：X3F_SFD_ALIGN=1 开启。参考帧 = order[0]（即累加器网格）。
 * 零位移素材逐位不变（warp 在 |shift|<1e-4 时原样透传）。
 *
 * 流程：
 *   1) 测量：各帧以 1/8 线性域「对数亮度梯度图」做 4×4 瓦片相位相关
 *      （借用 0.4.0 的 tile + 中位数 + 离散度门），得相对参考帧的亚像素位移；
 *   2) 融合前对每帧 lin 做可分离双三次 warp（Catmull-Rom：DC 精确、Σc²≤1
 *      不放大噪声），clip 掩码最近邻跟随，使所有帧对齐到 order[0] 网格。
 * 不做仿射/光流：分块实测仿射项全场 <1px，局部运动场景应检测后放弃而非 warp。
 * =========================================================================== */
typedef struct { float re, im; } sfd_cpx;

/* 原地 radix-2 1D FFT（按 stride 跨步，供行列复用）；inv=0 正变换，inv=1 逆变换 */
static void sfd_fft1d_stride(sfd_cpx *a, int n, int stride, int inv)
{
    /* 标准增量式位反转置换 */
    {
        int j = 0;
        for (int i = 1; i < n; i++) {
            int m = n >> 1;
            while (m >= 1 && j >= m) { j -= m; m >>= 1; }
            j += m;
            if (i < j) { sfd_cpx t = a[i * stride]; a[i * stride] = a[j * stride]; a[j * stride] = t; }
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        double ang = (inv ? 2.0 : -2.0) * M_PI / (double)len;
        sfd_cpx wlen = { (float)cosf((float)ang), (float)sinf((float)ang) };
        for (int i = 0; i < n; i += len) {
            sfd_cpx w = { 1.0f, 0.0f };
            for (int k = 0; k < len / 2; k++) {
                sfd_cpx u = a[(i + k) * stride];
                float tr = w.re * a[(i + k + len / 2) * stride].re - w.im * a[(i + k + len / 2) * stride].im;
                float ti = w.re * a[(i + k + len / 2) * stride].im + w.im * a[(i + k + len / 2) * stride].re;
                a[(i + k) * stride].re = u.re + tr;
                a[(i + k) * stride].im = u.im + ti;
                a[(i + k + len / 2) * stride].re = u.re - tr;
                a[(i + k + len / 2) * stride].im = u.im - ti;
                float nwr = w.re * wlen.re - w.im * wlen.im;
                float nwi = w.re * wlen.im + w.im * wlen.re;
                w.re = nwr; w.im = nwi;
            }
        }
    }
    if (inv) {
        float ninv = 1.0f / (float)n;
        for (int i = 0; i < n; i++) { a[i * stride].re *= ninv; a[i * stride].im *= ninv; }
    }
}

static void sfd_fft2d(sfd_cpx *a, int w, int h, int inv)
{
    for (int y = 0; y < h; y++) sfd_fft1d_stride(a + (size_t)y * w, w, 1, inv);
    for (int x = 0; x < w; x++) sfd_fft1d_stride(a + x, h, w, inv);
}

/* a = 参考帧，b = 移动帧。成功返回 1。
 * 符号约定（已用噪声/方块图实测锁死）：返回的 (dx,dy)=r 满足
 *     b(x,y) = a(x + dx, y + dy)        （移动帧 = 参考帧平移了 +r）
 * 即移动帧相对参考帧右移 dx、下移 dy。要把 b 对齐回 a 的网格，须按
 *     out(p) = b(p - r)
 * 采样（见 sfd_align_warp，约定为 out(p)=lin(p-(sx,sy))，调用时传入 r 即可）。
 * 注意：r = -s（s 为 b=a(x-s) 中的真实位移），并非直接等于位移，这是相位相关的固有符号。 */
static int sfd_phase_shift(const float *a, const float *b, int w, int h,
                           double *dx, double *dy, double *peak)
{
    size_t N = (size_t)w * h;
    sfd_cpx *ca = (sfd_cpx *)malloc(N * sizeof(sfd_cpx));
    sfd_cpx *cb = (sfd_cpx *)malloc(N * sizeof(sfd_cpx));
    if (ca == NULL || cb == NULL) { free(ca); free(cb); return 0; }
    /* radix-2 FFT 要求 w、h 均为 2 的幂；非 2 幂维度 FFT 未定义（会静默产出垃圾值），
       这里安全返回失败（调用方按「放弃该帧」降级），避免错误位移污染融合结果。 */
    if (w < 2 || h < 2 || (w & (w - 1)) != 0 || (h & (h - 1)) != 0) {
        free(ca); free(cb); return 0;
    }
    for (size_t i = 0; i < N; i++) {
        ca[i].re = a[i]; ca[i].im = 0.0f;
        cb[i].re = b[i]; cb[i].im = 0.0f;
    }
    sfd_fft2d(ca, w, h, 0);
    sfd_fft2d(cb, w, h, 0);
    for (size_t i = 0; i < N; i++) {            /* 互功率谱归一 */
        float ar = ca[i].re, ai = ca[i].im, br = cb[i].re, bi = cb[i].im;
        float rr = ar * br + ai * bi;           /* Re(A·conj(B)) */
        float ri = ai * br - ar * bi;           /* Im(A·conj(B)) */
        float mag = sqrtf(rr * rr + ri * ri);
        if (mag > 1e-12f) { ca[i].re = rr / mag; ca[i].im = ri / mag; }
        else { ca[i].re = 0.0f; ca[i].im = 0.0f; }
    }
    sfd_fft2d(ca, w, h, 1);                     /* 逆变换 → 相关面存于 .re */
    double mx = -1.0; int pxx = 0, pyy = 0;     /* 峰值搜索：取全局最大，不剔除中心像素。
                                                   零/极小位移本就在 (0,0)（亚像素插值给出 ~0），
                                                   若剔除中心反会被迫选中邻域伪峰（±1px 真实位移会被丢掉）。 */
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double v = (double)ca[(size_t)y * w + x].re;
            if (v > mx) { mx = v; pxx = x; pyy = y; }
        }
    }
    if (mx < 1e-4) { free(ca); free(cb); return 0; }
    #define SFD_SURF(xx, yy) ca[(((yy) + h) % h) * w + (((xx) + w) % w)].re   /* 环绕取点 */
    /* 3×3 二维二次拟合亚像素（比单轴抛物线偏差更小，对齐精度更高） */
    double zc  = SFD_SURF(pxx, pyy);
    double zl  = SFD_SURF(pxx - 1, pyy),     zr  = SFD_SURF(pxx + 1, pyy);
    double zd  = SFD_SURF(pxx, pyy - 1),     zu  = SFD_SURF(pxx, pyy + 1);
    double zul = SFD_SURF(pxx - 1, pyy - 1), zur = SFD_SURF(pxx + 1, pyy - 1);
    double zdl = SFD_SURF(pxx - 1, pyy + 1), zdr = SFD_SURF(pxx + 1, pyy + 1);
    double A1 = (zr - zl) * 0.5;
    double A2 = (zu - zd) * 0.5;
    double A3 = (zl + zr) * 0.5 - zc;
    double A5 = (zd + zu) * 0.5 - zc;
    double A4 = (zdr + zul - zur - zdl) * 0.25;
    double dxs = 0.0, dys = 0.0;
    double det = 4.0 * A3 * A5 - A4 * A4;
    if (fabs(det) > 1e-9) {
        dxs = (-2.0 * A5 * A1 + A4 * A2) / det;
        dys = (-2.0 * A3 * A2 + A4 * A1) / det;
        if (dxs < -1.0) dxs = -1.0; if (dxs > 1.0) dxs = 1.0;
        if (dys < -1.0) dys = -1.0; if (dys > 1.0) dys = 1.0;
    }
    #undef SFD_SURF
    int ix = pxx > w / 2 ? pxx - w : pxx;
    int iy = pyy > h / 2 ? pyy - h : pyy;
    *dx = (double)ix + dxs;
    *dy = (double)iy + dys;
    *peak = mx;
    free(ca); free(cb);
    return 1;
}

/* 线性域对数亮度梯度幅值（Sobel），输入 lin 为交织 RGB（w*h*3） */
static void sfd_log_grad(const float *lin, int w, int h, float *g)
{
    size_t N = (size_t)w * h;
    float *lg = (float *)malloc(N * sizeof(float));
    if (lg == NULL) { memset(g, 0, N * sizeof(float)); return; }
    for (size_t i = 0; i < N; i++) {
        float lum = (lin[3*i] + lin[3*i+1] + lin[3*i+2]) * (1.0f/3.0f);
        if (lum < 1e-3f) lum = 1e-3f;
        lg[i] = logf(lum);
    }
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (x < 1 || x > w-2 || y < 1 || y > h-2) { g[(size_t)y*w+x] = 0.0f; continue; }
            size_t i = (size_t)y*w + x;
            float gx = (lg[i-w-1] + 2.0f*lg[i-1] + lg[i+w-1])
                     - (lg[i-w+1] + 2.0f*lg[i+1] + lg[i+w+1]);
            float gy = (lg[i-w-1] + 2.0f*lg[i-w] + lg[i-w+1])
                     - (lg[i+w-1] + 2.0f*lg[i+w] + lg[i+w+1]);
            g[i] = sqrtf(gx*gx + gy*gy);
        }
    }
    free(lg);
}

static double sfd_dmedian(double *a, int n)
{
    double *b = (double *)malloc(n * sizeof(double));
    if (b == NULL) return 0.0;
    for (int i = 0; i < n; i++) b[i] = a[i];
    for (int i = 1; i < n; i++) {
        double key = b[i]; int j = i - 1;
        while (j >= 0 && b[j] > key) { b[j+1] = b[j]; j--; }
        b[j+1] = key;
    }
    double m = b[n/2];
    free(b);
    return m;
}

static double sfd_drange(double *a, int n)
{
    double lo = a[0], hi = a[0];
    for (int i = 1; i < n; i++) { if (a[i] < lo) lo = a[i]; if (a[i] > hi) hi = a[i]; }
    return hi - lo;
}

/* 可分离双三次（Catmull-Rom）warp：把 lin 按 (sx,sy) 平移对齐到参考网格。
 * 约定：out(p) = lin(p - (sx,sy))。零位移逐位透传；OOM 时跳过（保留原图，安全）。
 * 符号（已由噪声图实测锁死）：sfd_phase_shift(gref, gi) 返回的 (sx,sy)=r 满足
 *   gi(x) = gref(x + r)        （移动帧 = 参考帧平移了 +r）
 * 故把 gi 映射到 gref 网格须按 p - r 采样：out(x) = gi(x - r) = gref(x)。
 * clip 掩码以同样 p - r 跟随，保证与 lin 严格一致。 */
static void sfd_align_warp(float *lin, float *clip, int w, int h, double sx, double sy)
{
    if (fabs(sx) < 1e-4 && fabs(sy) < 1e-4) return;
    size_t N = (size_t)w * h;
    float *H = (float *)malloc(N * 3 * sizeof(float));
    if (H == NULL) return;
    /* 水平 pass：H(x,y) = 对 lin 在 (x-sx, y) 做 bicubic-x */
    for (int y = 0; y < h; y++) {
        size_t row = (size_t)y * w;
        for (int x = 0; x < w; x++) {
            double fx = (double)x - sx;
            int x0 = (int)floor(fx); double t = fx - x0;
            if (fx < 1.0 || fx > (double)(w - 2) - 1e-6) {
                int xx = x - (int)lround(sx);
                if (xx < 0) xx = 0; if (xx >= w) xx = w - 1;
                for (int c = 0; c < 3; c++) H[(row + x)*3 + c] = lin[(row + xx)*3 + c];
                continue;
            }
            double w0 = -0.5*t*t*t + t*t - 0.5*t;
            double w1 =  1.5*t*t*t - 2.5*t*t + 1.0;
            double w2 = -1.5*t*t*t + 2.0*t*t + 0.5*t;
            double w3 =  0.5*t*t*t - 0.5*t*t;
            int xm = x0 - 1, x1 = x0, x2 = x0 + 1, x3 = x0 + 2;
            if (xm < 0) xm = 0; if (xm >= w) xm = w-1;
            if (x1 < 0) x1 = 0; if (x1 >= w) x1 = w-1;
            if (x2 < 0) x2 = 0; if (x2 >= w) x2 = w-1;
            if (x3 < 0) x3 = 0; if (x3 >= w) x3 = w-1;
            for (int c = 0; c < 3; c++) {
                H[(row + x)*3 + c] = lin[(row + xm)*3 + c]*w0
                                  + lin[(row + x1)*3 + c]*w1
                                  + lin[(row + x2)*3 + c]*w2
                                  + lin[(row + x3)*3 + c]*w3;
            }
        }
    }
    /* 垂直 pass：lin(x,y) = 对 H 在 (x, y-sy) 做 bicubic-y */
    for (int y = 0; y < h; y++) {
        double fy = (double)y - sy;
        int y0 = (int)floor(fy); double t = fy - y0;
        int use_near = (fy < 1.0 || fy > (double)(h - 2) - 1e-6);
        double w0, w1, w2, w3;
        if (!use_near) {
            w0 = -0.5*t*t*t + t*t - 0.5*t;
            w1 =  1.5*t*t*t - 2.5*t*t + 1.0;
            w2 = -1.5*t*t*t + 2.0*t*t + 0.5*t;
            w3 =  0.5*t*t*t - 0.5*t*t;
        }
        int ym = y0 - 1, y1 = y0, y2 = y0 + 1, y3 = y0 + 2;
        if (ym < 0) ym = 0; if (ym >= h) ym = h-1;
        if (y1 < 0) y1 = 0; if (y1 >= h) y1 = h-1;
        if (y2 < 0) y2 = 0; if (y2 >= h) y2 = h-1;
        if (y3 < 0) y3 = 0; if (y3 >= h) y3 = h-1;
        for (int x = 0; x < w; x++) {
            for (int c = 0; c < 3; c++) {
                float v;
                if (use_near) {
                    int yy = y - (int)lround(sy);
                    if (yy < 0) yy = 0; if (yy >= h) yy = h - 1;
                    v = H[((size_t)yy * w + x)*3 + c];
                } else {
                    v = H[((size_t)ym * w + x)*3 + c]*w0
                      + H[((size_t)y1 * w + x)*3 + c]*w1
                      + H[((size_t)y2 * w + x)*3 + c]*w2
                      + H[((size_t)y3 * w + x)*3 + c]*w3;
                }
                lin[((size_t)y * w + x)*3 + c] = v;
            }
        }
    }
    free(H);
    /* clip 掩码：随 lin 同样按 p - (sx,sy) 最近邻跟随，保证与 lin 严格一致 */
    if (clip != NULL) {
        float *c2 = (float *)malloc(N * sizeof(float));
        if (c2 != NULL) {
            for (size_t i = 0; i < N; i++) {
                int x = (int)(i % w), y = (int)(i / w);
                int xx = x - (int)lround(sx), yy = y - (int)lround(sy);
                if (xx < 0) xx = 0; if (xx >= w) xx = w - 1;
                if (yy < 0) yy = 0; if (yy >= h) yy = h - 1;
                c2[i] = clip[(size_t)yy * w + xx];
            }
            memcpy(clip, c2, N * sizeof(float));
            free(c2);
        }
    }
}

/* ── 局部平滑位移场 warp（借鉴 0.4.0 X3F_X3I_ALIGN=local）──
 * 场 = nt×nt 瓦片位移（全分辨率 px，存于瓦片中心）。warp 时逐像素双线性评估
 * 场值，再以 2D Catmull-Rom 直接采样 out(p) = lin(p - s(p))；clip 掩码按
 * p - s(p) 最近邻跟随。场整体近零逐位透传；OOM 回退单位移全局 warp（安全）。 */
static void sfd_fld_eval(const double *fx, const double *fy, int nt,
                         double X, double Y, double FW, double FH,
                         double *sx, double *sy)
{
    double stepx = FW / nt, stepy = FH / nt;
    double u = X / stepx - 0.5, v = Y / stepy - 0.5;
    if (u < 0) u = 0;
    if (u > nt - 1) u = nt - 1;
    if (v < 0) v = 0;
    if (v > nt - 1) v = nt - 1;
    int i0 = (int)u, j0 = (int)v;
    double tx = u - i0, ty = v - j0;
    if (i0 >= nt - 1) { i0 = nt - 2; tx = 1.0; }
    if (j0 >= nt - 1) { j0 = nt - 2; ty = 1.0; }
    double a = (1.0 - tx) * (1.0 - ty), b = tx * (1.0 - ty);
    double c = (1.0 - tx) * ty,        d = tx * ty;
    *sx = a * fx[(size_t)j0 * nt + i0]     + b * fx[(size_t)j0 * nt + i0 + 1]
        + c * fx[(size_t)(j0+1) * nt + i0] + d * fx[(size_t)(j0+1) * nt + i0 + 1];
    *sy = a * fy[(size_t)j0 * nt + i0]     + b * fy[(size_t)j0 * nt + i0 + 1]
        + c * fy[(size_t)(j0+1) * nt + i0] + d * fy[(size_t)(j0+1) * nt + i0 + 1];
}

static void sfd_align_warp_field(float *lin, float *clip, int w, int h,
                                 int nt, const double *fx, const double *fy,
                                 double down, double fmx, double fmy)
{
    /* 场整体近零 → 透传（门控：任一瓦片 |位移| ≥ 1e-4 全分辨率 px 才动） */
    double fmax = 0.0;
    for (int t = 0; t < nt * nt; t++) {
        double m = sqrt(fx[t]*fx[t] + fy[t]*fy[t]);
        if (m > fmax) fmax = m;
    }
    if (fmax < 1e-4) return;
    size_t N = (size_t)w * h;
    float *src = (float *)malloc(N * 3 * sizeof(float));
    float *csrc = (clip != NULL) ? (float *)malloc(N * sizeof(float)) : NULL;
    if (src == NULL || (clip != NULL && csrc == NULL)) {
        free(src); free(csrc);
        sfd_align_warp(lin, clip, w, h, fmx, fmy);   /* OOM 回退：单位移 */
        return;
    }
    memcpy(src, lin, N * 3 * sizeof(float));
    if (csrc != NULL) memcpy(csrc, clip, N * sizeof(float));
    double FW = (double)w * down, FH = (double)h * down;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double sx, sy;
            sfd_fld_eval(fx, fy, nt, (double)x * down, (double)y * down, FW, FH, &sx, &sy);
            sx /= down; sy /= down;
            double fxx = (double)x - sx, fyy = (double)y - sy;
            int x0 = (int)floor(fxx), y0 = (int)floor(fyy);
            double tx = fxx - x0, ty = fyy - y0;
            double wx[4], wy[4];
            wx[0] = -0.5*tx*tx*tx + tx*tx - 0.5*tx;
            wx[1] =  1.5*tx*tx*tx - 2.5*tx*tx + 1.0;
            wx[2] = -1.5*tx*tx*tx + 2.0*tx*tx + 0.5*tx;
            wx[3] =  0.5*tx*tx*tx - 0.5*tx*tx;
            wy[0] = -0.5*ty*ty*ty + ty*ty - 0.5*ty;
            wy[1] =  1.5*ty*ty*ty - 2.5*ty*ty + 1.0;
            wy[2] = -1.5*ty*ty*ty + 2.0*ty*ty + 0.5*ty;
            wy[3] =  0.5*ty*ty*ty - 0.5*ty*ty;
            for (int c = 0; c < 3; c++) {
                double acc = 0.0;
                for (int jj = 0; jj < 4; jj++) {
                    int yy = y0 - 1 + jj;
                    if (yy < 0) yy = 0; if (yy >= h) yy = h - 1;
                    for (int ii = 0; ii < 4; ii++) {
                        int xx = x0 - 1 + ii;
                        if (xx < 0) xx = 0; if (xx >= w) xx = w - 1;
                        acc += wy[jj] * wx[ii] * src[((size_t)yy * w + xx) * 3 + c];
                    }
                }
                lin[((size_t)y * w + x) * 3 + c] = (float)acc;
            }
            if (csrc != NULL) {
                int xx = x - (int)lround(sx), yy = y - (int)lround(sy);
                if (xx < 0) xx = 0; if (xx >= w) xx = w - 1;
                if (yy < 0) yy = 0; if (yy >= h) yy = h - 1;
                clip[(size_t)y * w + x] = csrc[(size_t)yy * w + xx];
            }
        }
    }
    free(src); free(csrc);
}

/* 对齐模式：0=关 1=全局单位移 2=局部平滑场。
 * 程序化门控优先（显式开=全局模式）；环境变量 X3F_SFD_ALIGN：
 *   未设/0 → 关；"local"/"2" → 局部场；其余真值 → 全局。 */
static int sfd_align_mode(void)
{
    if (g_sfd_align_override >= 0) return g_sfd_align_override ? 1 : 0;
    const char *e = getenv("X3F_SFD_ALIGN");
    if (e == NULL || *e == '\0' || *e == '0') return 0;
    if (strcmp(e, "local") == 0 || strcmp(e, "2") == 0) return 2;
    return 1;
}

static int sfd_align_on(void)
{
    return sfd_align_mode() > 0;
}

/* 程序化门控（产品前端用）：见 x3f_render.h 中 x3f_render_set_sfd_align 说明。
 * on < 0 → 回到只听环境变量 X3F_SFD_ALIGN；否则显式 0/1，优先于环境变量。 */
void x3f_render_set_sfd_align(int on)
{
    g_sfd_align_override = (on < 0) ? -1 : (on ? 1 : 0);
}

/* 测量每帧相对参考帧（order[0]）的亚像素位移，写入 sx/sy（全分辨率像素）。
 * 借鉴 0.4.0：1/8 线性域对数亮度梯度图 + 瓦片相位相关 + 中位数。
 * 全局模式（mode 1）：4×4 瓦片 → 中位数 → 离散度门（瓦片不一致放弃该帧）。
 * 局部模式（mode 2，X3F_SFD_ALIGN=local）：8×8 瓦片 → 质量门 + 邻域正则 +
 * 有界梯度 → 逐瓦片平滑位移场存 g_sfd_fldx/y（warp 走 sfd_align_warp_field）。
 * 任何失败都安全降级：全部清零、标记放弃，主流程继续（不对齐）。 */
static void sfd_align_measure(const char *path, const char *wb,
                              const x3f_denoise_t *dn,
                              const int *order, int n,
                              double *sx, double *sy)
{
    int i, ti, tj;
    int ref = order[0];
    int DM = 8;                 /* 测量降采样 */
    int w8 = 0, h8 = 0;
    float *lref = NULL, *cref = NULL, *gref = NULL;
    char tmp[1024];
    char wbo[64];
    g_sfd_align_abandon = 0;
    for (i = 0; i < n; i++) { sx[i] = 0.0; sy[i] = 0.0; }

    /* 解码参考帧（down=DM） */
    {
        const char *fp = x3f_standalone_path(path, ref, tmp, sizeof(tmp));
        if (fp == NULL) goto abandon;
        own_set_frame_nr(0);
        own_gpu_thread_disable(1);
        int fr = x3f_render_load_wb(fp, DM, wb, 0, dn, &lref, &cref,
                                   NULL, NULL, NULL, &w8, &h8, wbo);
        own_gpu_thread_disable(0);
        x3f_temp_release(tmp); tmp[0] = '\0';
        if (fr != X3FRENDER_OK || lref == NULL || w8 < 8 || h8 < 8) goto abandon;
    }
    gref = (float *)malloc((size_t)w8 * h8 * sizeof(float));
    if (gref == NULL) goto abandon;
    sfd_log_grad(lref, w8, h8, gref);
    free(lref); free(cref); lref = cref = NULL;

    /* 自测（X3F_SFD_ALIGN_SELFTEST=1）：对参考图的 2 幂子窗施加已知整数位移，
       验证相位相关的符号/尺度。sfd_phase_shift 要求 w,h 为 2 的幂（radix-2 FFT），
       故取与逐帧瓦片同尺寸的 256×256 窗，匹配生产契约；整图尺寸（1/8 后）非 2 幂不可用。 */
    if (getenv("X3F_SFD_ALIGN_SELFTEST") != NULL) {
        int PW = 256;
        if (w8 >= PW && h8 >= PW) {
            float *aref = (float *)malloc((size_t)PW * PW * sizeof(float));
            float *amov = (float *)malloc((size_t)PW * PW * sizeof(float));
            if (aref && amov) {
                for (int y = 0; y < PW; y++)
                    for (int x = 0; x < PW; x++)
                        aref[(size_t)y * PW + x] = gref[(size_t)y * w8 + x];
                const double tx = 3.0, ty = 2.0;   /* 施加整数位移：amov(x,y) = aref(x-tx, y-ty) */
                for (int y = 0; y < PW; y++)
                    for (int x = 0; x < PW; x++) {
                        int sxx = x - (int)lround(tx), syy = y - (int)lround(ty);
                        if (sxx < 0) sxx = 0; if (sxx >= PW) sxx = PW - 1;
                        if (syy < 0) syy = 0; if (syy >= PW) syy = PW - 1;
                        amov[(size_t)y * PW + x] = aref[(size_t)syy * PW + sxx];
                    }
                double rdx, rdy, pk;
                int ok = sfd_phase_shift(aref, amov, PW, PW, &rdx, &rdy, &pk);
                /* amov = aref(x - tx, y - ty)  =>  phase 应返回 (-tx, -ty)（符号约定见 sfd_phase_shift） */
                double eerr = sqrt((rdx + tx)*(rdx + tx) + (rdy + ty)*(rdy + ty));
                fprintf(stderr, "[sfd align selftest] applied(%.1f,%.1f) recovered(%.3f,%.3f) expected(%.1f,%.1f) |err|=%.3f peak=%.4f -> %s\n",
                        tx, ty, rdx, rdy, -tx, -ty, eerr, pk, (ok && eerr < 0.3) ? "PASS" : "FAIL");
                free(aref); free(amov);
            }
        } else {
            fprintf(stderr, "[sfd align selftest] 跳过：w8=%d h8=%d < 256（窗不足）\n", w8, h8);
        }
    }

    /* 自测 2：局部场插值（sfd_fld_eval）。常量场逐点复原；瓦片中心精确取值；
       相邻瓦片正中 = 两瓦片均值（双线性权重正确性）。 */
    if (getenv("X3F_SFD_ALIGN_SELFTEST") != NULL) {
        double fxc[SFD_FLD_NT * SFD_FLD_NT], fyc[SFD_FLD_NT * SFD_FLD_NT];
        for (int t = 0; t < SFD_FLD_NT * SFD_FLD_NT; t++) { fxc[t] = 1.5; fyc[t] = -2.25; }
        double ex, ey;
        sfd_fld_eval(fxc, fyc, SFD_FLD_NT, 123.4, 777.7, 5424.0, 3616.0, &ex, &ey);
        int ok1 = (fabs(ex - 1.5) < 1e-12 && fabs(ey + 2.25) < 1e-12);
        double fxr[SFD_FLD_NT * SFD_FLD_NT], fyr[SFD_FLD_NT * SFD_FLD_NT];
        for (int y = 0; y < SFD_FLD_NT; y++)
            for (int x = 0; x < SFD_FLD_NT; x++) {
                fxr[y * SFD_FLD_NT + x] = 0.5 + 0.01 * x;
                fyr[y * SFD_FLD_NT + x] = -1.0 + 0.02 * y;
            }
        double stepx = 5424.0 / SFD_FLD_NT, stepy = 3616.0 / SFD_FLD_NT;
        int cx = 3, cy = 5;
        sfd_fld_eval(fxr, fyr, SFD_FLD_NT, (cx + 0.5) * stepx, (cy + 0.5) * stepy,
                     5424.0, 3616.0, &ex, &ey);
        int ok2 = (fabs(ex - fxr[cy * SFD_FLD_NT + cx]) < 1e-12 &&
                   fabs(ey - fyr[cy * SFD_FLD_NT + cx]) < 1e-12);
        sfd_fld_eval(fxr, fyr, SFD_FLD_NT, (cx + 1.0) * stepx, (cy + 0.5) * stepy,
                     5424.0, 3616.0, &ex, &ey);
        double midx = 0.5 * (fxr[cy * SFD_FLD_NT + cx] + fxr[cy * SFD_FLD_NT + cx + 1]);
        int ok3 = (fabs(ex - midx) < 1e-12);
        fprintf(stderr, "[sfd align selftest2] field eval const=%d centre=%d midpoint=%d -> %s\n",
                ok1, ok2, ok3, (ok1 && ok2 && ok3) ? "PASS" : "FAIL");
    }

    /* 逐帧（参考帧自身位移为 0） */
    for (i = 0; i < n; i++) {
        if (order[i] == ref) { sx[i] = 0.0; sy[i] = 0.0; continue; }
        float *li = NULL, *ci = NULL, *gi = NULL;
        char tmp2[1024];
        int ww = 0, hh = 0;
        const char *fp = x3f_standalone_path(path, order[i], tmp2, sizeof(tmp2));
        if (fp == NULL) continue;
        own_set_frame_nr(0);
        own_gpu_thread_disable(1);
        int fr = x3f_render_load_wb(fp, DM, wb, 0, dn, &li, &ci,
                                   NULL, NULL, NULL, &ww, &hh, wbo);
        own_gpu_thread_disable(0);
        x3f_temp_release(tmp2); tmp2[0] = '\0';
        if (fr != X3FRENDER_OK || li == NULL || ww != w8 || hh != h8) {
            free(li); free(ci);
            continue;   /* 该帧无法测 → 仅此帧不偏移（保守） */
        }
        gi = (float *)malloc((size_t)w8 * h8 * sizeof(float));
        if (gi == NULL) { free(li); free(ci); continue; }
        sfd_log_grad(li, w8, h8, gi);
        free(li); free(ci); li = ci = NULL;

        const int loc = (g_sfd_align_mode == 2);
        const int NT = loc ? SFD_FLD_NT : 4;
        int tw = w8 / NT, th = h8 / NT;
        int P = 256, Q = 256;
        sfd_cpx *pa = (sfd_cpx *)malloc((size_t)P * Q * sizeof(sfd_cpx));
        sfd_cpx *pb = (sfd_cpx *)malloc((size_t)P * Q * sizeof(sfd_cpx));
        float *bufa = (float *)malloc((size_t)P * Q * sizeof(float));
        float *bufb = (float *)malloc((size_t)P * Q * sizeof(float));
        double tdx[64], tdy[64]; int tok[64];   /* 逐瓦片（local 用）：值+有效标志 */
        double ldx[64], ldy[64]; int nl = 0;
        for (int t = 0; t < 64; t++) tok[t] = 0;
        if (pa && pb && bufa && bufb) {
            for (tj = 0; tj < NT; tj++) {
                for (ti = 0; ti < NT; ti++) {
                    int x0 = ti * tw, y0 = tj * th;
                    memset(bufa, 0, (size_t)P * Q * sizeof(float));
                    memset(bufb, 0, (size_t)P * Q * sizeof(float));
                    for (int yy = 0; yy < th && y0 + yy < h8; yy++)
                        for (int xx = 0; xx < tw && x0 + xx < w8; xx++) {
                            bufa[(size_t)yy * Q + xx] = gref[(size_t)(y0 + yy) * w8 + (x0 + xx)];
                            bufb[(size_t)yy * Q + xx] = gi[(size_t)(y0 + yy) * w8 + (x0 + xx)];
                        }
                    double dx, dy, pk;
                    /* local 瓦片更小 → 峰值门提高（0.10 vs 0.02），弱峰交给邻域正则补 */
                    if (sfd_phase_shift(bufa, bufb, P, Q, &dx, &dy, &pk) &&
                        pk > (loc ? 0.10 : 0.02)) {
                        int t = tj * NT + ti;
                        tdx[t] = dx; tdy[t] = dy; tok[t] = 1;
                        if (nl < 64) { ldx[nl] = dx; ldy[nl] = dy; nl++; }
                    }
                }
            }
            if (nl >= 6) {
                double mx = sfd_dmedian(ldx, nl);
                double my = sfd_dmedian(ldy, nl);
                if (loc) {
                    /* ── 局部场（借鉴 0.4.0 local）── */
                    /* ① 质量门：偏离全局中位 >1.0（@1/8 = 8 全分辨率 px，即 0.4.0
                       的局部搜索半径 global±8px）的瓦片弃测 → 邻域正则补 */
                    for (int t = 0; t < NT * NT; t++)
                        if (tok[t] && (fabs(tdx[t] - mx) > 1.0 || fabs(tdy[t] - my) > 1.0))
                            tok[t] = 0;
                    /* ①b 邻域一致门（0.4.0："disagree with nearby tiles or the
                       global transform"）：与有效 4 邻中位偏差 >0.5（@1/8 = 4 全
                       分辨率 px）的孤立野值弃测。成片连贯区域邻居中位≈自身值，
                       不受影响；只杀孤立伪峰。 */
                    for (int tj2 = 0; tj2 < NT; tj2++)
                        for (int ti2 = 0; ti2 < NT; ti2++) {
                            int t = tj2 * NT + ti2;
                            if (!tok[t]) continue;
                            double nbx[4], nby[4]; int nn = 0;
                            const int nb[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
                            for (int k = 0; k < 4; k++) {
                                int ax = ti2 + nb[k][0], ay = tj2 + nb[k][1];
                                if (ax < 0 || ax >= NT || ay < 0 || ay >= NT) continue;
                                int ts = ay * NT + ax;
                                if (tok[ts]) { nbx[nn] = tdx[ts]; nby[nn] = tdy[ts]; nn++; }
                            }
                            if (nn >= 2) {
                                double nmx = sfd_dmedian(nbx, nn);
                                double nmy = sfd_dmedian(nby, nn);
                                if (fabs(tdx[t] - nmx) > 0.5 || fabs(tdy[t] - nmy) > 0.5)
                                    tok[t] = 0;
                            }
                        }
                    /* ② 邻域正则：缺格用 4 邻已知值中位补（≥2 个已知才补），迭代至稳定 */
                    for (int iter = 0; iter < NT * NT; iter++) {
                        int changed = 0;
                        for (int tj2 = 0; tj2 < NT; tj2++)
                            for (int ti2 = 0; ti2 < NT; ti2++) {
                                int t = tj2 * NT + ti2;
                                if (tok[t]) continue;
                                double nbx[4], nby[4]; int nn = 0;
                                const int nb[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
                                for (int k = 0; k < 4; k++) {
                                    int ax = ti2 + nb[k][0], ay = tj2 + nb[k][1];
                                    if (ax < 0 || ax >= NT || ay < 0 || ay >= NT) continue;
                                    int ts = ay * NT + ax;
                                    if (tok[ts]) { nbx[nn] = tdx[ts]; nby[nn] = tdy[ts]; nn++; }
                                }
                                if (nn >= 2) {
                                    tdx[t] = sfd_dmedian(nbx, nn);
                                    tdy[t] = sfd_dmedian(nby, nn);
                                    tok[t] = 1; changed = 1;
                                }
                            }
                        if (!changed) break;
                    }
                    /* ③ 剩余缺格 → 全局中位（绝不留洞） */
                    for (int t = 0; t < NT * NT; t++)
                        if (!tok[t]) { tdx[t] = mx; tdy[t] = my; tok[t] = 1; }
                    /* ④ 有界梯度（防折叠）：相邻瓦片位移差钳到 ≤2 全分辨率 px/步。
                       本网格步长数百全分辨率 px 而位移 ≤8px，正常永不触发，纯防御。 */
                    for (int iter = 0; iter < 3; iter++) {
                        for (int tj2 = 0; tj2 < NT; tj2++)
                            for (int ti2 = 0; ti2 < NT; ti2++) {
                                int t = tj2 * NT + ti2;
                                for (int k = 0; k < 2; k++) {   /* k=0 右邻 k=1 下邻 */
                                    if ((k == 0 && ti2 + 1 >= NT) ||
                                        (k == 1 && tj2 + 1 >= NT)) continue;
                                    int t2 = t + (k == 0 ? 1 : NT);
                                    double ddx = tdx[t2] - tdx[t], ddy = tdy[t2] - tdy[t];
                                    double dd = sqrt(ddx*ddx + ddy*ddy) * 8.0;
                                    if (dd > 2.0) {
                                        double pull = (dd - 2.0) / dd * 0.5;
                                        tdx[t] += pull * ddx;  tdy[t] += pull * ddy;
                                        tdx[t2] -= pull * ddx; tdy[t2] -= pull * ddy;
                                    }
                                }
                            }
                    }
                    /* ⑤ 存场（全分辨率 px）；中位留作 note/回退。local 不做离散度放弃
                       ——局部运动正是场要处理的，只有中位位移过大才整张放弃。 */
                    double fm = 0.0;
                    for (int t = 0; t < NT * NT; t++) {
                        g_sfd_fldx[i][t] = tdx[t] * 8.0;
                        g_sfd_fldy[i][t] = tdy[t] * 8.0;
                        double m2 = sqrt(g_sfd_fldx[i][t]*g_sfd_fldx[i][t] +
                                         g_sfd_fldy[i][t]*g_sfd_fldy[i][t]);
                        if (m2 > fm) fm = m2;
                    }
                    g_sfd_fld_has[i] = 1;
                    sx[i] = mx * 8.0;
                    sy[i] = my * 8.0;
                    if (fm > g_sfd_align_max) g_sfd_align_max = fm;
                    /* 放弃判据用全局中位（局部相对运动正是场要处理的，不算失败） */
                    if (sqrt(mx*mx + my*my) * 8.0 > 8.0) {
                        free(pa); free(pb); free(bufa); free(bufb); free(gi);
                        goto abandon_all;
                    }
                } else {
                    double rgx = sfd_drange(ldx, nl);
                    double rgy = sfd_drange(ldy, nl);
                    double spread_full = 8.0 * sqrt(rgx * rgx + rgy * rgy);  /* 1/8→全分辨率 */
                    if (spread_full <= 6.0) {           /* 离散度门：瓦片不一致→局部运动，放弃该帧 */
                        sx[i] = mx * 8.0;
                        sy[i] = my * 8.0;
                        double mag = sqrt(sx[i]*sx[i] + sy[i]*sy[i]);
                        if (mag > g_sfd_align_max) g_sfd_align_max = mag;
                        if (mag > 8.0) {                /* 位移过大 → 拍摄失败，放弃整张对齐 */
                            free(pa); free(pb); free(bufa); free(bufb); free(gi);
                            goto abandon_all;
                        }
                    }
                }
            }
        }
        free(pa); free(pb); free(bufa); free(bufb); free(gi);
    }
    free(gref);
    return;

abandon_all:
    free(gref);
    for (i = 0; i < n; i++) { sx[i] = 0.0; sy[i] = 0.0; g_sfd_fld_has[i] = 0; }
    g_sfd_align_abandon = 1;
    return;

abandon:
    free(lref); free(cref); free(gref);
    for (i = 0; i < n; i++) { sx[i] = 0.0; sy[i] = 0.0; g_sfd_fld_has[i] = 0; }
    g_sfd_align_abandon = 1;
    return;
}

int x3f_render_load_sfd_ex(const char *path, int down, const char *wb, int nlm,
                           const x3f_denoise_t *dn, int max_frames,
                           float **rgb_out, int *w_out, int *h_out,
                           char *note, int note_cap, double *norm_out,
                           x3f_sfd_frame_fuse_fn frame_fuse, void *frame_fuse_ctx,
                           x3f_sfd_progress_fn progress, void *progress_ctx)
{
    int n, i, w = 0, h = 0, rc = X3FRENDER_OK;
    double shutter[X3F_SFD_MAX_FRAMES];
    double coef[9];
    double k0 = 0.0, lin_sat = 1.0, scale_v = 1.0;
    float *accv = NULL, *accw = NULL, *wmap = NULL;
    unsigned char *used = NULL;      /* 每像素实际参与加权的帧数（0…n），核心重建靠它 */
    size_t np = 0, j;
    int order[X3F_SFD_MAX_FRAMES];
    double sfd_sx[X3F_SFD_MAX_FRAMES], sfd_sy[X3F_SFD_MAX_FRAMES];  /* 对齐位移（全分辨率 px） */
    int t;
    char tmps[X3F_SFD_MAX_FRAMES][1024];   /* 各帧已提取的临时文件（shutter 循环产出，解码消费） */
    size_t px_dbg = (size_t)-1;      /* X3F_SFD_DEBUG_PX 指定的那个像素 */

    memset(tmps, 0, sizeof(tmps));

    if (rgb_out == NULL || w_out == NULL || h_out == NULL || path == NULL)
        return X3FRENDER_ERR_PARAM;
    *rgb_out = NULL;
    *w_out = *h_out = 0;
    if (note != NULL && note_cap > 0) note[0] = '\0';
    if (norm_out != NULL) *norm_out = 1.0;
    g_sfd_fb_ch = g_sfd_fb_px = 0;
    g_sfd_neutral = 0;
    g_t_camf = g_t_extract = g_t_decode = g_t_decode_sum = g_t_dec_disp = 0.0;
    g_t_passa = g_t_fuse = g_t_passb = g_t_norm = 0.0;
    g_t_stage1 = g_t_edge = g_t_tail = 0.0;
    g_t_total = sfd_now_ms();

    n = x3f_frame_count(path);
    if (n <= 1) return X3FRENDER_ERR_PARAM;       /* 普通 X3F 没有多帧可合 */
    if (max_frames > 0 && max_frames < n) n = max_frames;
    if (n > X3F_SFD_MAX_FRAMES) n = X3F_SFD_MAX_FRAMES;
    if (down < 1) down = 1;
    for (i = 0; i < X3F_SFD_MAX_FRAMES; i++) shutter[i] = 0.0;
    sfd_build_luts();                             /* 两张 65536 项的表，只建一次 */

    /* 1) 每帧的快门（µs）——曝光归一 k 就是它的比值。
     * 这里提取的每帧临时文件**留着**（tmps[]，函数级声明），给后面的解码
     * 流水线直接用——临时提取 ~120ms/帧，7 帧两次（CAMF 一次 + 解码一次）
     * 变一次，省 ~0.8s。所有权规则：谁消费（sfd_job_start / 串行分支）
     * 谁清 tmps[i]；fail 路径兜底把没消费的一并删掉。 */
    {
        double tc0 = sfd_now_ms();
        for (i = 0; i < n; i++) {
            const char *fp = x3f_standalone_path(path, i, tmps[i], sizeof(tmps[i]));
            unsigned d[X3F_CAMF_DIMS] = { 1, 1, 1 };
            double s = 0.0;
            if (fp != NULL)
                x3f_camf_matrix_f64(fp, "CaptureShutter", &s, 1, d);
            if (s > 0.0)
                shutter[i] = s;
        }
        g_t_camf = sfd_now_ms() - tc0;
    }
    k0 = shutter[0];

    /* 处理顺序：按 k 降序（最欠曝的帧排最前）。k 读不到时按原顺序。 */
    for (i = 0; i < n; i++) order[i] = i;
    for (i = 1; i < n; i++) {
        int cur = order[i], j2 = i - 1;
        double kc = (k0 > 0.0 && shutter[cur] > 0.0) ? k0 / shutter[cur] : 1.0;
        while (j2 >= 0) {
            double kj = (k0 > 0.0 && shutter[order[j2]] > 0.0) ? k0 / shutter[order[j2]] : 1.0;
            if (kj >= kc) break;
            order[j2 + 1] = order[j2];
            j2--;
        }
        order[j2 + 1] = cur;
    }
    if (sfd_debug()) {
        fprintf(stderr, "[sfd] 处理顺序（k 降序）：");
        for (i = 0; i < n; i++) {
            double k = (k0 > 0.0 && shutter[order[i]] > 0.0) ? k0 / shutter[order[i]] : 1.0;
            fprintf(stderr, " 帧%d(k=%.3g)", order[i], k);
        }
        fprintf(stderr, "\n");
    }

    /* 噪声系数（ISO 缩放后的 a/b/c）取基准帧的：七帧同机同标定 */
    {
        char tmp[1024];
        const char *fp = x3f_standalone_path(path, 0, tmp, sizeof(tmp));
        if (!x3f_render_noise_coef_for(fp, coef)) {
            for (i = 0; i < 9; i++) coef[i] = (i % 3 == 0) ? 1e-4 : 0.0;
        }
        x3f_temp_release(tmp);
    }

    /* ── SFD 帧对齐（默认关）：以 1/8 线性域测各帧相对
     * order[0]（累加器网格）的亚像素位移；融合前对每帧 lin 做双三次 warp。
     * X3F_SFD_ALIGN=1 全局单位移；=local 局部平滑位移场（借鉴 0.4.0）。
     * 零位移图逐位不变。 */
    g_sfd_align = 0; g_sfd_align_abandon = 0; g_sfd_align_max = 0.0;
    g_sfd_align_mode = sfd_align_mode();
    for (i = 0; i < X3F_SFD_MAX_FRAMES; i++) {
        sfd_sx[i] = 0.0; sfd_sy[i] = 0.0; g_sfd_fld_has[i] = 0;
    }
    if (g_sfd_align_mode > 0) {
        g_sfd_align = 1;
        sfd_align_measure(path, wb, dn, order, n, sfd_sx, sfd_sy);
    }

    /* 2) 逐帧解码 → 累加（一轮只驻留一帧，别把 7 帧都摆内存里）
     *
     * 处理顺序按 **k 从大到小**（= 曝光从最短到最长）。加权和本身与顺序无关，
     * 这么做是为了"下界"那一格：极端高光里只有最欠曝的帧还落在传感器线性段上，
     * 其余帧的读数是被饱和肩压过的（同一个像素上 R 通道的 k·lin 实测 7.83 vs 1.88）。
     * 下界取 max 或谁大用谁，会让相邻像素挑中不同的帧 → 屏幕上就是方块状色斑；
     * 让**最欠曝的帧先写、后面的不覆盖**（下界槽只写一次），拿到的是一帧自己
     * 的数据，空间上连续。 */
    /* 进度口径（2026-10-23）：total = n + 4 —— 前 n 格是帧解码+融合累加，
     * 后 4 格是 7 帧之后用户仍要等的部分：合成归一（含下界中位/核心重建）、
     * Stage1 多尺度降噪、显示渲染（Swift 侧 apply16/apply 后上报）、写出。
     * 之前只报帧解码，GUI 进度条到 7/7（93%）后钉死，后面 ~30s 像是卡住。 */
    if (progress != NULL) progress(progress_ctx, 0, n + 4);
    /* ── 帧解码作业（N 槽预取流水线）──
     * 处理 order[t] 时 order[t+1..t+nslot-1] 已在后台解码。
     * 处理（权重/累加）仍在主线程按 order 顺序串行 —— 下界"最欠曝帧先写"
     * 的语义不变。
     *
     * **默认 2 槽（2026-10-25 从 3 槽收回）**。3 槽当年是有收益的
     * （DP2Q1514 全分辨率 join 墙钟 18.7s → ~11s），但那是在"每帧解码纯
     * CPU"的年代。加上 GPU 之后实测是反的：每槽 lin+clip ≈ 315MB，3 槽
     * 峰值近 1GB，而 GPU 后端每次调用还要新建/销毁 ~300MB 缓冲、上传
     * ~130MB、回读 86.5MB，几路一起灌就把集显的共享内存带宽打满 ——
     * 单次 guided_up 从独占 0.6~1.3s 涨到 13~33s，整张从 ~35s 退到 ~105s，
     * 而且**不稳定**（同配置重复跑在 15s 与 101s 之间横跳）。
     * 2 槽 + 解码线程绕开 GPU（见 sfd_dec_thread）实测最稳：解码 ~11~16s。
     * X3F_SFD_SLOTS=3..4 仍可调回去，X3F_SFD_GPU=1 把 GPU 开回来做 A/B。 */
    {
        int pipe_on = 0;
        int nslot = 2;
#ifdef SFD_HAVE_THREADS
        const char *pe = getenv("X3F_SFD_PAIR");
        const char *ps = getenv("X3F_SFD_SLOTS");
        pipe_on = 1;
        if (pe != NULL && *pe == '0') pipe_on = 0;
        if (ps != NULL && *ps != '\0') {
            nslot = atoi(ps);
            if (nslot < 2) nslot = 2;
            if (nslot > 4) nslot = 4;
        }
#endif
        {
            sfd_dec_job jb[4];
            pthread_t th[4];
            int th_on[4] = {0, 0, 0, 0};
            int q;

            memset(jb, 0, sizeof(jb));
            for (q = 0; pipe_on && q < nslot && q < n; q++) {
                th_on[q] = sfd_job_start(&jb[q], &th[q], path, order[q],
                                         down, wb, nlm, dn, q == 0, tmps[order[q]]);
            }
            for (t = 0; t < n; t++) {
                i = order[t];
                {
                char tmp[1024];
                const char *fp;
                float *lin = NULL, *clip = NULL;
                int fw = 0, fh = 0;
                char wbo[64];
                double k = 1.0, inv_k2 = 1.0;
                int slot = t % nslot;
                sfd_dec_job *jd = &jb[slot];

                if (th_on[slot]) {
                    double tj0 = sfd_now_ms();
                    pthread_join(th[slot], NULL);
                    g_t_decode += sfd_now_ms() - tj0;
                    th_on[slot] = 0;
                    rc = jd->rc;
                    lin = jd->lin; clip = jd->clip;
                    fw = jd->fw; fh = jd->fh;
                    g_t_decode_sum += jd->dec_ms;
                    g_t_dec_disp += jd->disp_ms;
                    memcpy(wbo, jd->wbo, sizeof(wbo));
                    memcpy(tmp, jd->tmp, sizeof(tmp));   /* 路径拷出，槽位可复用 */
                    jd->lin = jd->clip = NULL;
                    jd->tmp[0] = '\0';
                    if (pipe_on && t + nslot < n)
                        th_on[slot] = sfd_job_start(&jb[slot], &th[slot], path,
                                                    order[t + nslot], down, wb,
                                                    nlm, dn, 0, tmps[order[t + nslot]]);
                } else {
                    /* 槽未起（pthread_create 失败兜底）：同步解码 */
                    double td0 = 0.0;
                    if (tmps[i][0] != '\0') {
                        memcpy(tmp, tmps[i], sizeof(tmp));  /* 复用 shutter 提取的文件 */
                        tmps[i][0] = '\0';
                    } else {
                        td0 = sfd_now_ms();
                        fp = x3f_standalone_path(path, i, tmp, sizeof(tmp));
                        g_t_extract += sfd_now_ms() - td0;
                    }
                    own_set_frame_nr(t == 0 ? 1 : 0);
                    own_gpu_thread_disable(!sfd_thread_gpu());
                    td0 = sfd_now_ms();
                    rc = x3f_render_load_wb(fp, down, wb, i == 0 ? nlm : 0, dn,
                                            &lin, &clip, NULL, NULL, NULL,
                                            &fw, &fh, wbo);
                    own_gpu_thread_disable(0);   /* 主线程恢复，别连坐后续 */
                    g_t_decode += sfd_now_ms() - td0;
                    g_t_decode_sum += g_t_decode;
                    g_t_dec_disp += x3f_render_last_decode_ms();
                }
                fp = tmp;
                if (rc != X3FRENDER_OK || lin == NULL) {
                    free(lin);
                    free(clip);
                    x3f_temp_release(tmp);
                    sfd_pipe_drain(jb, th, th_on, nslot);
                    rc = (rc == X3FRENDER_OK) ? X3FRENDER_ERR_DECODE : rc;
                    goto fail;
                }
                /* SFD 帧对齐 warp：把本帧 lin 对齐到 order[0] 网格（参考帧位移为 0）。
                 * local 模式有场 → 逐像素场 warp；否则常量单位移。近零位移逐位透传。 */
                if (g_sfd_align && g_sfd_fld_has[i]) {
                    sfd_align_warp_field(lin, clip, fw, fh, SFD_FLD_NT,
                                         g_sfd_fldx[i], g_sfd_fldy[i],
                                         (double)down,
                                         sfd_sx[i] / (double)down,
                                         sfd_sy[i] / (double)down);
                } else if (g_sfd_align &&
                           (fabs(sfd_sx[i]) > 1e-4 || fabs(sfd_sy[i]) > 1e-4)) {
                    sfd_align_warp(lin, clip, fw, fh,
                                   sfd_sx[i] / (double)down, sfd_sy[i] / (double)down);
                }
                /* ↓↓ 帧处理体：与串行版逐行相同（下界/权重/累加语义）↓↓ */
        /* 累加器与饱和点在**第一个被处理的帧**上定（顺序按 k 降序）：每帧解码出来的
         * 线性值都是"同一个物理亮度同一个刻度"（各帧自己的饱和点都落在 lin = LIN_SAT），
         * 所以用哪一帧量饱和点都得到同一个数（实测 SDQ_0665：1.34~1.38）。 */
                if (t == 0) {
                    w = fw;
                    h = fh;
                    np = (size_t)w * (size_t)h;
                    px_dbg = sfd_debug_px(w);
                    lin_sat = sfd_lin_sat(lin, clip, np);
                    scale_v = 16383.0 / (lin_sat > 1e-6 ? lin_sat : 1.0);
                    accv = (float *)calloc(np * 3, sizeof(float));
                    accw = (float *)calloc(np * 3, sizeof(float));
                    wmap = (float *)calloc(np, sizeof(float));
                    used = (unsigned char *)calloc(np, 1);
                    if (accv == NULL || accw == NULL || wmap == NULL || used == NULL) {
                        free(lin);
                        free(clip);
                        x3f_temp_release(tmp);
                        sfd_pipe_drain(jb, th, th_on, nslot);
                        rc = X3FRENDER_ERR_NOMEM;
                        goto fail;
                    }
                } else if (fw != w || fh != h) {          /* 同一台相机，尺寸应当一致 */
                    free(lin);
                    free(clip);
                    x3f_temp_release(tmp);
                    sfd_pipe_drain(jb, th, th_on, nslot);
                    rc = X3FRENDER_ERR_DECODE;
                    goto fail;
                }

                if (k0 > 0.0 && shutter[i] > 0.0) {
                    k = k0 / shutter[i];
                    inv_k2 = 1.0 / (k * k);
                }

                memset(wmap, 0, np * sizeof(float));
        /* Pass A：**先按没融合的解码值算权重 / 可用帧 / 下界**。
         *
         * 融合只该改像素值，不该改"这一帧在这一格还算不算数"。原来逐帧回调
         * 先改 `lin`、再算 vmax/门限：高光核心里 V 正好跨过可用门时，融合把
         * 它抬一下，相邻像素就会在"这一帧算数 / 不算数"之间跳，合成出来是
         * 一圈圈环纹和星状接缝（用户 2026-10-20 截图）。
         *
         * 所以权重在这里一次算好存进 `wmap`，回调改完 `lin` 再用同一份权重
         * 累加融合后的值。物理上"这帧曝到哪儿"本来就该看原始解码值。 */
                g_t_passa -= sfd_now_ms();
                {
                    sfd_passa_ctx pa;
                    pa.np = np; pa.lin = lin; pa.clip = clip;
                    pa.wmap = wmap; pa.accv = accv; pa.accw = accw;
                    pa.used = used; pa.coef = coef;
                    pa.k = k; pa.inv_k2 = inv_k2; pa.scale_v = scale_v;
                    pa.lin_sat = lin_sat; pa.frame_idx = i; pa.px_dbg = px_dbg;
                    sfd_par(np, sfd_passa_chunk, &pa);
                }
                g_t_passa += sfd_now_ms();

        /* 逐帧融合：每帧先拿**自己的**内嵌 JPEG 做高光融合，再进累加器。
         * 放在权重算完之后：融合改的是值，不改"哪几帧算数"。 */
                if (frame_fuse != NULL) {
                    g_t_fuse -= sfd_now_ms();
                    int frc = frame_fuse(frame_fuse_ctx, i, lin, fw, fh, k);
                    g_t_fuse += sfd_now_ms();
                    if (frc != 0) {
                        free(lin);
                        free(clip);
                        x3f_temp_release(tmp);
                        sfd_pipe_drain(jb, th, th_on, nslot);
                        rc = frc;
                        goto fail;
                    }
                }

        /* Pass B：用融合后的线性值累加，权重仍是 Pass A 按原始值算的那一份。 */
                g_t_passb -= sfd_now_ms();
                {
                    sfd_passb_ctx pb;
                    pb.np = np; pb.lin = lin; pb.wmap = wmap;
                    pb.accv = accv; pb.accw = accw;
                    pb.k = k; pb.scale_v = scale_v;
                    sfd_par(np, sfd_passb_chunk, &pb);
                }
                g_t_passb += sfd_now_ms();
                if (progress != NULL) progress(progress_ctx, t + 1, n + 4);
                free(lin);
                free(clip);
                x3f_temp_release(tmp);
                }
            }
            sfd_pipe_drain(jb, th, th_on, nslot);   /* 理论上已排空，兜底 */
        }
        /* 无线程平台（pipe_on=0）：上面循环每帧走"槽未起"同步解码分支，
         * 行为与旧串行版逐行相同 */
    }
    own_set_frame_nr(1);          /* 恢复默认：后续单帧解码照常降噪 */

    if (sfd_debug()) {
        double mx = 0.0, mw = 0.0;
        for (j = 0; j < np; j++) {
            if (accv[j * 3] > mx) mx = accv[j * 3];
            if (accw[j * 3] > mw) mw = accw[j * 3];
        }
        fprintf(stderr, "[sfd] 累加完：lin_sat=%.3f scale_v=%.1f 像素0 accv=(%.4g,%.4g,%.4g) accw=%.4g；"
                        "全图 max accv=%.4g max accw=%.4g\n",
                lin_sat, scale_v, accv[0], accv[1], accv[2], accw[0], mx, mw);
    }

    /* 3) 归一 → 换回显示线性域（1.0 = SDR 白，高光可以 >1，最多到 #帧 的曝光范围） */
    g_t_norm -= sfd_now_ms();
    {
        sfd_norm_ctx nc;
        int q;
        nc.np = np; nc.accv = accv; nc.accw = accw; nc.scale_v = scale_v;
        memset(nc.fb_ch, 0, sizeof(nc.fb_ch));
        memset(nc.fb_px, 0, sizeof(nc.fb_px));
        sfd_par(np, sfd_norm_chunk, &nc);
        for (q = 0; q < SFD_NCHUNK; q++) {
            g_sfd_fb_ch += nc.fb_ch[q];
            g_sfd_fb_px += nc.fb_px[q];
        }
    }
    /* 3c) 下界区做一次 **5×5 中位**（只对"全帧不可用、走最欠曝帧下界"的像素）：
     *
     * 那里只有 −3EV 一帧的读数可用，而它靠近饱和肩，会带上像素级亮丝
     * （实测 x=384 那一列 R 9969→14314→13695，画面里就是核心右边的亮点）。
     * 取局部中位：孤立亮点被抹掉、环纹这类成片的低频结构保留；
     * 只跟"同为下界"的邻居取中位，不跟正常区混，所以不会在边界拉出新边。 */
    {
        float *src = (float *)malloc((size_t)np * 3 * sizeof(float));
        if (src != NULL) {
            sfd_median_ctx mc;
            memcpy(src, accv, (size_t)np * 3 * sizeof(float));
            mc.np = np; mc.w = w; mc.h = h;
            mc.src = src; mc.accv = accv; mc.accw = accw;
            sfd_par(np, sfd_median_chunk, &mc);
            free(src);
        }
    }
    /* 3d) 核心重建：只剩 1~2 帧可用的那片亮核，沿等半径方向做鲁棒中位。
     *     放在下界中位之后、Stage1 之前 —— 后面的降噪/曲线都看到的是修好的值。 */
    sfd_core_rebuild(accv, used, w, h);
    /* 3d') HLP 色比插值（默认关，X3F_SFD_HLPCOL=1 试）：把可信邻域的 R/G、B/G
     *      插进核心，治的是"逐像素挑帧"撕出来的黄/橙色块。 */
    sfd_hlp_color(accv, used, w, h);
    {
        const char *e = getenv("X3F_SFD_CORE_DEBUG");
        if (e != NULL && *e != '\0') sfd_core_probe(accv, used, w, h);
    }
    free(accw);
    if (progress != NULL) progress(progress_ctx, n + 1, n + 4);   /* 合成归一完成 */

    /*3b) Stage1「多尺度降噪」（三级金字塔：大核双边 + 边缘图 + 加权混合，
     * 见 x3f_stage1.c 与 §3/§6）。
     *     放在这里而不是逐帧：合成的七帧里只有基准帧跑过 NLM，别的帧的噪声是原样
     *     进图的 —— 合成完再收一遍，代价只有一遍滤波（逐帧要七遍）。 */
    g_t_norm += sfd_now_ms();
    g_t_stage1 -= sfd_now_ms();
    if (x3f_stage1_denoise(accv, w, h, coef, lin_sat) > 0 && sfd_debug())
        fprintf(stderr, "[sfd] Stage1 多尺度降噪：已跑\n");
    g_t_stage1 += sfd_now_ms();

    /* 3e) 高光边缘轻平滑：只抹边界那 4 px 带，核心内部不动。放在 Stage1 之后，
     *     免得降噪又把台阶拉回来。 */
    g_t_edge -= sfd_now_ms();
    sfd_edge_smooth(accv, used, w, h);
    g_t_edge += sfd_now_ms();
    free(used);
    if (progress != NULL) progress(progress_ctx, n + 2, n + 4);   /* Stage1 降噪完成 */

    /* 4) **不再**"缩放到饱和值"（2026-10-17 撤掉，理由见
     *    COMMIT_MSG_SFD高光权重与归一.txt）。
     *
     * 原来这里按 `ApplyScaleToSatVal` 把整幅乘上 LIN_SAT / max，让合成的最亮处
     * 落在饱和点上。撤掉的理由是实测：**SFD 的显示曲线 kX3FSFDTone 本来就是拿"没有归一"
     * 的合成数据标定的**（标定时 lin 就是物理亮度），所以归一等于是给显示曲线多乘了
     * 一次 1/max —— SDQ_0665（归一 ×0.105）整幅比 导出暗 3.2 档，DP0Q0189
     * （×0.320）暗 1.65 档（当年记的那句"偏灰、暗"就是这么来的）。
     * 用 16 位 TIFF 逐像素反解标定系数（`hlprobe` 出物理亮度、Python 读 TIFF
     * 过 kX3FSFDTone 反查）实测：显示值 ≈ kX3FSFDTone(物理亮度 × 1.05)，中位 1.09 ——
     * 也就是这一步本来就该是 1.0。
     *
     * 顺带：1/max 这个口径会让**映射随画面内容变**（同一份物理亮度，高光更炸的片子被压得
     * 更暗），标定是固定的，所以这条口径本身也不成立。
     *
     * 输出口径因此回到"1.0 = SDR 白，高光可以 >1"：显示曲线自己把 2~3 档高光收进 254
     * 以内（表到 lin=16 才 0.990）；HDR 那条链要的本来就是物理亮度，于是 `Exporter`
     * 里"把归一除回来"那一步变成恒等。 */
    g_t_tail -= sfd_now_ms();
    {
        double mx = 0.0;

        /* 4b) 极端高光**软膝收峰**：合成的最亮处实测能到 10 倍 SDR 白，显示曲线在
         * 那一段几乎是平的 —— 顶白范围比 SPP 大、核心里的环被压成一片。
         * 把 1.5 以上的值按指数软膝收向 PEAK（默认 5.0），曲线在中高光段就重新有斜率，
         * 环分得开、白区变小。X3F_SFD_PEAK / X3F_SFD_KNEE 可覆盖（PEAK<=KNEE 关掉）。 */
        {
            double peak = s1_env_num_x("X3F_SFD_PEAK", 5.0);
            double kn = s1_env_num_x("X3F_SFD_KNEE", 1.5);
            if (peak > kn && peak > 0.0) {
                sfd_knee_ctx kc;
                kc.n3 = np * 3; kc.accv = accv;
                kc.kn = kn; kc.span = peak - kn;
                sfd_par(np * 3, sfd_knee_chunk, &kc);
            }
        }

        /* 5) 极端高光**中性化**（2026-10-17 加）。越过高光肩之后，色度在这里本来
         * 就不可信：相机域的读数经色彩矩阵常常出域（实测 −3 EV 帧在同一格上 B 直接
         * 是负的），而亮度（R 通道）仍然平滑。逐像素挑帧的结果就是"相邻两格的 B
         * 差 11 倍"——屏幕上那一片不均匀的橙色块。
         *
         * 口径照：超出量程的高光走向中性白（它的 SFD 导出里那块核心是三通道
         * 一起顶到白的干净色块）。做法是最亮通道超过 SDR 白若干档之后，把色度朝
         * **最亮通道**收（只掉饱和度，亮度单调不减）；环纹那种"更暗的线"因为量级
         * 更低，只被收掉一部分，仍然留得住。 */
        {
            double lo = X3F_SFD_NEUTRAL_LO, hi = X3F_SFD_NEUTRAL_HI;
            const char *env = getenv("X3F_SFD_NEUTRAL");
            long neutralized = 0;
            if (env != NULL && *env != '\0' && sscanf(env, "%lf,%lf", &lo, &hi) != 2) {
                lo = X3F_SFD_NEUTRAL_LO;
                hi = X3F_SFD_NEUTRAL_HI;
            }
            if (hi > lo) {
                sfd_neutral_ctx nc;
                int q;
                nc.np = np; nc.accv = accv; nc.lo = lo; nc.hi = hi;
                memset(nc.neutralized, 0, sizeof(nc.neutralized));
                sfd_par(np, sfd_neutral_chunk, &nc);
                neutralized = 0;
                for (q = 0; q < SFD_NCHUNK; q++) neutralized += nc.neutralized[q];
            }
            g_sfd_neutral = neutralized;
        }

        {
            sfd_peak_ctx pc;
            int q;
            pc.np = np; pc.accv = accv;
            memset(pc.peak, 0, sizeof(pc.peak));
            sfd_par(np, sfd_peak_chunk, &pc);
            mx = 0.0;
            for (q = 0; q < SFD_NCHUNK; q++) if (pc.peak[q] > mx) mx = pc.peak[q];
        }
        g_sfd_peak = mx;                /* 只给 note / 诊断用 */
        g_sfd_norm = 1.0;
        if (norm_out != NULL) *norm_out = 1.0;
    }
    g_t_tail += sfd_now_ms();
    g_t_total = sfd_now_ms() - g_t_total;
    if (sfd_timing())
        fprintf(stderr,
                "[sfd timing] 总 %.0fms | CAMF %.0f | 抽帧 %.0f | 解码join %.0f"
                "（线程合计 %.0f，其中分发 %.0f）| PassA %.0f | fuse %.0f | PassB %.0f"
                " | 归一+下界中位 %.0f | Stage1 %.0f | 边缘 %.0f | 软膝+收色 %.0f\n",
                g_t_total, g_t_camf, g_t_extract, g_t_decode, g_t_decode_sum,
                g_t_dec_disp, g_t_passa, g_t_fuse, g_t_passb, g_t_norm,
                g_t_stage1, g_t_edge, g_t_tail);

    if (note != NULL && note_cap > 0) {
        int off = snprintf(note, (size_t)note_cap, "SFD·%d 帧·饱和点 %.2f",
                           n, lin_sat);
        if (k0 > 0.0 && off > 0 && off < note_cap) {
            off += snprintf(note + off, (size_t)(note_cap - off), "·k=");
            for (i = 0; i < n && off < note_cap - 8; i++) {
                double k = shutter[i] > 0.0 ? k0 / shutter[i] : 1.0;
                off += snprintf(note + off, (size_t)(note_cap - off), "%s%.3g",
                                i ? "," : "", k);
            }
        }
        if (off > 0 && off < note_cap)
            snprintf(note + off, (size_t)(note_cap - off),
                     "·物理峰值 ×%.2f（%+.2f EV）", g_sfd_peak, log2(g_sfd_peak > 0 ? g_sfd_peak : 1.0));
    }
    if (note != NULL && note_cap > 0 && g_sfd_align) {
        size_t nlen = strlen(note);
        if (nlen < (size_t)note_cap)
            snprintf(note + nlen, (size_t)(note_cap - nlen),
                     "·对齐%s%s(最大位移 %.2fpx)",
                     g_sfd_align_mode == 2 ? "[local]" : "",
                     g_sfd_align_abandon ? "放弃(运动/失败)" : "已应用",
                     g_sfd_align_max);
    }
    if (sfd_debug())
        fprintf(stderr, "[sfd] 全部帧都被软门挡住的：%ld 个通道（%.4f%%）、%ld 个像素（%.4f%%）"
                        " → 用 k·lin 下界\n",
                g_sfd_fb_ch, 100.0 * (double)g_sfd_fb_ch / (double)(np * 3),
                g_sfd_fb_px, 100.0 * (double)g_sfd_fb_px / (double)np);
    if (sfd_debug())
        fprintf(stderr, "[sfd] 极端高光收色：%ld 个像素（%.4f%%），峰值 ×%.2f\n",
                g_sfd_neutral, 100.0 * (double)g_sfd_neutral / (double)np, g_sfd_peak);
    if (sfd_debug())
        fprintf(stderr, "[core] 核心重建：%ld 像素，%ld 个改动（%.2f%%）\n",
                g_sfd_core_px, g_sfd_core_chg,
                g_sfd_core_px ? 100.0 * (double)g_sfd_core_chg / (double)g_sfd_core_px : 0.0);
    *rgb_out = accv;
    *w_out = w;
    *h_out = h;
    free(wmap);
    return X3FRENDER_OK;

fail:
    own_set_frame_nr(1);
    /* 兜底：删掉 shutter 循环提取、还没被任何解码消费的临时文件
     * （已消费的路径早已清空，remove 不存在的文件无害） */
    for (i = 0; i < X3F_SFD_MAX_FRAMES; i++)
        x3f_temp_release(tmps[i]);
    free(accv);
    free(accw);
    free(wmap);
    free(used);
    return rc;
}

/* 老入口：不做逐帧回调，行为与加回调之前完全一样。
 * 实验室工具（coreq / sfddump / sfdfuse / hlprobe / sfdorder）都还走它。 */
int x3f_render_load_sfd(const char *path, int down, const char *wb, int nlm,
                        const x3f_denoise_t *dn, int max_frames,
                        float **rgb_out, int *w_out, int *h_out,
                        char *note, int note_cap, double *norm_out)
{
    return x3f_render_load_sfd_ex(path, down, wb, nlm, dn, max_frames,
                                  rgb_out, w_out, h_out, note, note_cap, norm_out,
                                  NULL, NULL, NULL, NULL);
}
