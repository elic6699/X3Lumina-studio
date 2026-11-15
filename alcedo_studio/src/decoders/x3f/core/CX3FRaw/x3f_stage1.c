/*X3F_STAGE1.C — Stage1「多尺度降噪」落地
 *
 * 结构照 §3 / §6（那些地址是 里逐条读出来的）：
 *
 *   尺度表 {1, 1/2, 1/4}，从粗到细，每级：
 *     降采样 → 主双边（权重 = 噪声归一 × 自适应 LUT，核长 11~31）
 *            → 边缘图（逐通道二次式阈值，数"超过阈值的邻居"，edge = clamp(1 − cnt/(r²−1)))
 *            → 局部均值（边缘加权 FIR）×2 → 权重滤波 ×2
 *            → 逐通道线性混合 out = w·src + (1−w)·dst
 *   然后升采样喂给下一级。
 *
 * 我们这三处**按我们的口径实现**（每条都在下面代码里注明）：
 * 1. 噪声模型用我们已有的（CAMF/ 那套 σ² = a + b·V + c·V²，逐通道）——
 *      与 SFD 融合、降噪滑块同源，V 也是 lin·16383/LIN_SAT 的 DN 当量。
 *   2. 自适应 LUT 那条 tanh 影响函数用**噪声归一后的高斯拉普拉斯**代替：
 *      双边权重 w = exp(−(Δ²)/(2·σ²))，Δ 是通道差、σ 是该处噪声标准差。
 *      形状上与 tanh 那一族同构（都是"差值/噪声 → 权重"，单调下降、远处归零）。
 *   3. 两级权重滤波（XDoWeightfilter1/2）合并成一次"边缘图 × 噪声归一"的权重，
 *      混合权重 w = edge（平坦 → 走滤波后；有结构 → 走原值）。
 *
 * 为什么要有它： SFD 出图**干净但细节在**，靠的就是这一层 —— 大核双边只在
 * 小图上跑（1/4 上核长 31 等于原图 7.75 px），而"哪里有结构"由边缘图逐像素决定，
 * 所以平坦区的噪声被压掉、边缘一点不动。我们原来只有"基准帧跑一次色度 NLM"，
 * 合成结果里那六帧的噪声原样留在图上。
 *
 * 开关：环境变量 X3F_STAGE1=0 关掉、X3F_STAGE1=2 只跑 1/4 那级（调试用），
 *       X3F_STAGE1_STRENGTH=0..2 调总的混合强度（默认 1.0）。
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "x3f_parallel.h"

/* Metal GPU 层（Stage1 主双边提速，与 own_decoder.c/x3f_render.c 同一套）。
 * GPU 是 float 域 + 硬件 exp，输出与 CPU double 版差 ±1 LSB 级
 * （与 guided/bilat/chroma_nr/chrmed_sel 同口径）；X3FDEC_GPU=0 或失败
 * 自动回 CPU 行并行版。 */
#if defined(__has_include)
#  if __has_include("own_gpu.h")
#    include "own_gpu.h"
#    define HAVE_OWN_GPU 1
#  endif
#endif

/* 归一化线性值 → DN 当量（和 x3f_sfd.c 里的 scale_v 同一个口径） */
static double s1_v(double lin, double lin_sat)
{
    return lin * 16383.0 / (lin_sat > 1e-6 ? lin_sat : 1.0);
}

/* 该处的噪声标准差（逐通道二次式，与降噪/SFD 融合同源） */
/* σ 的整体倍率：传感器噪声模型给的是**单帧**的 σ，而合成结果里还叠着
 * "逐通道挑帧"带来的低频色度块（远大于单帧的随机噪声）。实测按模型 σ 做
 * 值域项时双边几乎不作用（hf 只掉 1.4%），所以给一个整体倍率；
 * X3F_STAGE1_NSIGMA 可覆盖（默认 4.0）。 */
static double g_s1_nscale = 4.0;

static double s1_sigma(const double coef[9], int c, double v)
{
    double s2;
    double floor;
    if (v < 0.0) v = 0.0;
    s2 = coef[c] + coef[3 + c] * v + coef[6 + c] * v * v;
    /*观测值本身也该计一份（光子噪声）： CalculateNoiseVar 就是这个形状，
     * 我们的二次式已经含 V² 项，这里只兜底一个最小方差 */
    if (!(s2 > 1e-9)) s2 = 1e-9;
    /* 底噪地板：某些通道的二次式在低信号处会算到接近 0（实测 B 通道），
     * 那会把**所有**微分都判成"超过噪声"→ 边缘图整幅为 0 → 一点都收不掉。
     * 至少按 0.4% 的信号 + 2 DN 当量。 */
    floor = 0.004 * v + 2.0;
    if (s2 < floor * floor) s2 = floor * floor;
    return g_s1_nscale * sqrt(s2);
}

/* 2× 盒式降采样（ComputeResampleParams 那条：奇数宽高取 floor，余下一列/行丢掉）
 * 行并行（2026-10-22 提速）：每个输出格只读 src、写本格，格间独立 → 逐位一致。 */
typedef struct {
    const float *src; int w, h; float *dst; int dw, dh;
} s1_down2_ctx;

static void s1_down2_row(void *vctx, size_t yy)
{
    const s1_down2_ctx *d = (const s1_down2_ctx *)vctx;
    int x, c, dy, dx;
    int y = (int)yy;
    for (x = 0; x < d->dw; x++) {
        int sx = x * 2, sy = y * 2;
        for (c = 0; c < 3; c++) {
            double s = 0.0;
            int n = 0;
            for (dy = 0; dy < 2; dy++)
                for (dx = 0; dx < 2; dx++) {
                    int px = sx + dx, py = sy + dy;
                    if (px >= d->w || py >= d->h) continue;
                    s += d->src[((size_t)py * d->w + px) * 3 + c];
                    n++;
                }
            d->dst[((size_t)y * d->dw + x) * 3 + c] = (float)(n > 0 ? s / n : 0.0);
        }
    }
}

static void s1_down2(const float *src, int w, int h, float *dst, int dw, int dh)
{
    s1_down2_ctx d;
    d.src = src; d.w = w; d.h = h; d.dst = dst; d.dw = dw; d.dh = dh;
    x3f_parallel_for((size_t)dh, s1_down2_row, &d);
}

/* 双线性升采样到 (w,h)；边缘按夹取值（SPP 走 ResampleImage，边界处理同族）
 * 行并行（2026-10-22 提速）：每行只读 src，行间独立 → 逐位一致 */
typedef struct {
    const float *src; int sw, sh; float *dst; int w, h;
} s1_up_ctx;

static void s1_up_row(void *vctx, size_t yy)
{
    const s1_up_ctx *u = (const s1_up_ctx *)vctx;
    const int sw = u->sw, sh = u->sh, w = u->w;
    int y = (int)yy, x, c;
    double fy = (y + 0.5) * sh / u->h - 0.5;
    int y0 = (int)floor(fy);
    double wy = fy - y0;
    int y1 = y0 + 1;
    if (y0 < 0) { y0 = 0; y1 = sh > 1 ? 1 : 0; }
    if (y1 > sh - 1) { y1 = sh - 1; y0 = sh > 1 ? sh - 2 : 0; }
    for (x = 0; x < w; x++) {
        double fx = (x + 0.5) * sw / w - 0.5;
        int x0 = (int)floor(fx);
        double wx = fx - x0;
        int x1 = x0 + 1;
        if (x0 < 0) { x0 = 0; x1 = sw > 1 ? 1 : 0; }
        if (x1 > sw - 1) { x1 = sw - 1; x0 = sw > 1 ? sw - 2 : 0; }
        for (c = 0; c < 3; c++) {
            double a = u->src[((size_t)y0 * sw + x0) * 3 + c];
            double b = u->src[((size_t)y0 * sw + x1) * 3 + c];
            double d = u->src[((size_t)y1 * sw + x0) * 3 + c];
            double e = u->src[((size_t)y1 * sw + x1) * 3 + c];
            double top = a + (b - a) * wx, bot = d + (e - d) * wx;
            u->dst[((size_t)y * w + x) * 3 + c] = (float)(top + (bot - top) * wy);
        }
    }
}

static void s1_up(const float *src, int sw, int sh, float *dst, int w, int h)
{
    s1_up_ctx u;
    u.src = src; u.sw = sw; u.sh = sh; u.dst = dst; u.w = w; u.h = h;
    x3f_parallel_for((size_t)h, s1_up_row, &u);
}

/* 升采样并**累加**进 dst（2026-10-25）。三级金字塔要的是
 *     acc += 升采样(本级修正)
 * 原本写成"升采样到一块全幅 up（235MB）→ 再逐元素 cur += up"，等于每级
 * 多写一趟全幅再读回来。合并成一趟之后输出**逐位一致**：(float) 转换仍
 * 发生在累加之前，与"先落 float 再相加"同一个舍入点；
 * 行并行不变（每行只写本行的 3 通道，行间独立）。 */
static void s1_up_add_row(void *vctx, size_t yy)
{
    const s1_up_ctx *u = (const s1_up_ctx *)vctx;
    const int sw = u->sw, sh = u->sh, w = u->w;
    int y = (int)yy, x, c;
    double fy = (y + 0.5) * sh / u->h - 0.5;
    int y0 = (int)floor(fy);
    double wy = fy - y0;
    int y1 = y0 + 1;
    if (y0 < 0) { y0 = 0; y1 = sh > 1 ? 1 : 0; }
    if (y1 > sh - 1) { y1 = sh - 1; y0 = sh > 1 ? sh - 2 : 0; }
    for (x = 0; x < w; x++) {
        double fx = (x + 0.5) * sw / w - 0.5;
        int x0 = (int)floor(fx);
        double wx = fx - x0;
        int x1 = x0 + 1;
        if (x0 < 0) { x0 = 0; x1 = sw > 1 ? 1 : 0; }
        if (x1 > sw - 1) { x1 = sw - 1; x0 = sw > 1 ? sw - 2 : 0; }
        for (c = 0; c < 3; c++) {
            double a = u->src[((size_t)y0 * sw + x0) * 3 + c];
            double b = u->src[((size_t)y0 * sw + x1) * 3 + c];
            double d = u->src[((size_t)y1 * sw + x0) * 3 + c];
            double e = u->src[((size_t)y1 * sw + x1) * 3 + c];
            double top = a + (b - a) * wx, bot = d + (e - d) * wx;
            u->dst[((size_t)y * w + x) * 3 + c] +=
                (float)(top + (bot - top) * wy);
        }
    }
}

static void s1_up_add(const float *src, int sw, int sh, float *dst, int w, int h)
{
    s1_up_ctx u;
    u.src = src; u.sw = sw; u.sh = sh; u.dst = dst; u.w = w; u.h = h;
    x3f_parallel_for((size_t)h, s1_up_add_row, &u);
}

/* 主双边：核半径 r、噪声归一权重。σ 按级缩放（2× 降采样把噪声标准差压 2 倍）
 *
 * 提速（2026-10-22，逐位一致）：
 *   1. spatial 项 exp(-d2/(2σs²)) 只依赖 (dy,dx) 共 (2r+1)² 种组合——函数入口
 *      算一次存表，替代原来每像素每邻域的重算（同一表达式同一输入，位模式相同）；
 *   2. 值域项的 σ 只依赖**中心像素**（s1_sigma(coef,c,s1_v(pc[c]))·level_scale），
 *      每像素预算 3 个 double，不再每邻域重算（输入相同 → 结果逐位相同）；
 *   3. 行并行（x3f_parallel_for）：每行只读 src、写 dst 本行，行间独立。
 * 值域项 exp(-(d·d)/(2σ²)) 的 d 逐邻域变化，保留逐次 exp() 调用不变。 */
typedef struct {
    const float *src;
    float       *dst;
    int          w, h, r;
    const double *coef;
    double       lin_sat, level_scale;
    const double *spt;              /* spatial 表 [(2r+1)*(2r+1)]，索引 (dy+r)*(2r+1)+dx+r */
} s1_bl_ctx;

static void s1_bl_row(void *vctx, size_t yy)
{
    const s1_bl_ctx *b = (const s1_bl_ctx *)vctx;
    const int w = b->w, h = b->h, r = b->r;
    const double *spt = b->spt;
    int y = (int)yy, x, c, dy, dx;

    for (x = 0; x < w; x++) {
        double sum[3] = {0, 0, 0}, wsum = 0.0;
        const float *pc = b->src + ((size_t)y * w + x) * 3;
        double sig[3];
        for (c = 0; c < 3; c++) {
            double s = s1_sigma(b->coef, c, s1_v((double)pc[c], b->lin_sat))
                       * b->level_scale;
            if (!(s > 1e-9)) s = 1e-9;
            sig[c] = s;
        }
        for (dy = -r; dy <= r; dy++) {
            int py = y + dy;
            const double *sptrow;
            if (py < 0 || py >= h) continue;
            sptrow = spt + (size_t)(dy + r) * (2 * r + 1);
            for (dx = -r; dx <= r; dx++) {
                int px = x + dx;
                const float *pq;
                double spatial, wgt;
                if (px < 0 || px >= w) continue;
                pq = b->src + ((size_t)py * w + px) * 3;
                /* 空间项：高斯，σ_s = r/2（查表） */
                spatial = sptrow[dx + r];
                wgt = spatial;
                for (c = 0; c < 3; c++) {
                    double d = s1_v((double)pq[c] - (double)pc[c], b->lin_sat);
                    wgt *= exp(-(d * d) / (2.0 * sig[c] * sig[c]));
                }
                if (wgt < 1e-12) continue;
                for (c = 0; c < 3; c++) sum[c] += wgt * pq[c];
                wsum += wgt;
            }
        }
        if (wsum > 1e-12)
            for (c = 0; c < 3; c++)
                b->dst[((size_t)y * w + x) * 3 + c] = (float)(sum[c] / wsum);
        else
            for (c = 0; c < 3; c++)
                b->dst[((size_t)y * w + x) * 3 + c] = pc[c];
    }
}

static void s1_bilateral(const float *src, float *dst, int w, int h, int r,
                         const double coef[9], double lin_sat, double level_scale)
{
    s1_bl_ctx b;
    double sptbuf[169];             /* r ≤ 6 → (2r+1)² ≤ 169 */
    int dy, dx;

#ifdef HAVE_OWN_GPU
    /* GPU 提速（2026-10-23）：值域 exp 在 GPU 是硬件指令，三级金字塔
     * ~33 亿次 CPU exp 全部消失。float 域 → 输出与 CPU 差 ±1 LSB 级；
     * 失败/关闭（X3FDEC_GPU=0）落回下面的 CPU 行并行版。 */
    if (own_gpu_stage1_bilat(src, dst, w, h, r, coef, lin_sat,
                             level_scale, g_s1_nscale) == 0)
        return;
#endif

    for (dy = -r; dy <= r; dy++)
        for (dx = -r; dx <= r; dx++) {
            double d2 = (double)dx * dx + (double)dy * dy;
            sptbuf[(size_t)(dy + r) * (2 * r + 1) + (dx + r)] =
                exp(-d2 / (2.0 * (r / 2.0) * (r / 2.0)));
        }
    b.src = src; b.dst = dst; b.w = w; b.h = h; b.r = r;
    b.coef = coef; b.lin_sat = lin_sat; b.level_scale = level_scale; b.spt = sptbuf;
    x3f_parallel_for((size_t)h, s1_bl_row, &b);
}

/* 结构闸门（代替 F23CreateEdgeMap 那张二值边缘图）。
 *
 * 为什么换：SPP 那张图是"数窗口里超过噪声阈值的邻居"，窗口半径 5~15 ——
 * 在 1/4 尺度上等于原图 20~60 px，一片瓦墙里每个窗口都会碰到瓦缝，于是
 * **整幅都被判成"有结构"**（实测平均 edge = 0.000）、一点也收不掉。
 * SPP 后面还有两级权重滤波把这张图变软，我们没那一层，就用维纳式软闸门代替：
 *
 *     gate = 1 / (1 + (hp / (2σ))²)      hp = 局部高通幅度
 *
 * 平坦区 hp ≪ σ → gate→1（走滤波后），纹理/边缘 hp ≫ σ → gate→0（保留原值）。
 * 连续、单调、不需要额外参数，形状上就是"信噪比选通"。 */
/* 提速（2026-10-22）：行并行——每像素 3×3 窗口只读 src、写 edge 本行，行间独立 → 逐位一致 */
typedef struct {
    const float *src; int w, h; const double *coef;
    double lin_sat, level_scale; float *edge;
} s1_gate_ctx;

static void s1_gate_row(void *vctx, size_t yy)
{
    const s1_gate_ctx *g = (const s1_gate_ctx *)vctx;
    const int w = g->w, h = g->h;
    int y = (int)yy, x, c;

    for (x = 0; x < w; x++) {
        const float *pc = g->src + ((size_t)y * w + x) * 3;
        double ratio2 = 0.0;
        for (c = 0; c < 3; c++) {
            /* 3×3 高通（中心减一圈均值） */
            double ring = 0.0, sig, hp;
            int n = 0, dy, dx;
            for (dy = -1; dy <= 1; dy++) {
                int py = y + dy;
                if (py < 0 || py >= h) continue;
                for (dx = -1; dx <= 1; dx++) {
                    int px = x + dx;
                    if (px < 0 || px >= w || (dx == 0 && dy == 0)) continue;
                    ring += g->src[((size_t)py * w + px) * 3 + c];
                    n++;
                }
            }
            if (n == 0) continue;
            hp = s1_v((double)pc[c] - ring / n, g->lin_sat);
            sig = s1_sigma(g->coef, c, s1_v((double)pc[c], g->lin_sat)) * g->level_scale;
            if (!(sig > 1e-9)) sig = 1e-9;
            {
                double q = hp / (0.5 * sig);
                if (q * q > ratio2) ratio2 = q * q;      /* 取最"有结构"的那个通道 */
            }
        }
        g->edge[(size_t)y * w + x] = (float)(1.0 / (1.0 + ratio2));
    }
}

static void s1_gate(const float *src, int w, int h, int r, const double coef[9],
                    double lin_sat, double level_scale, float *edge)
{
    s1_gate_ctx g;
    (void)r;
    g.src = src; g.w = w; g.h = h; g.coef = coef;
    g.lin_sat = lin_sat; g.level_scale = level_scale; g.edge = edge;
    x3f_parallel_for((size_t)h, s1_gate_row, &g);
}

static double s1_env_num(const char *name, double dflt)
{
    const char *e = getenv(name);
    if (e == NULL || *e == '\0') return dflt;
    return atof(e);
}

/* 诊断：X3F_STAGE1_DEBUG=1 时每级打一行（平均边缘值、被收掉的像素比例、
 * 滤波前后在本级上的高通能量）—— 标定闸门时要看的就是这三个数。 */
static int s1_debug(void)
{
    const char *e = getenv("X3F_STAGE1_DEBUG");
    return e != NULL && *e != '\0';
}

/* 单通道高通能量（|拉普拉斯| 均值），单位同输入（线性） */
static double s1_hf(const float *img, int w, int h)
{
    double s = 0.0;
    long n = 0;
    int x, y, c;
    for (y = 1; y + 1 < h; y++)
        for (x = 1; x + 1 < w; x++)
            for (c = 0; c < 3; c++) {
                const float *p = img + ((size_t)y * w + x) * 3 + c;
                double l = 4.0 * p[0]
                         - (double)img[((size_t)y * w + x - 1) * 3 + c]
                         - (double)img[((size_t)y * w + x + 1) * 3 + c]
                         - (double)img[((size_t)(y - 1) * w + x) * 3 + c]
                         - (double)img[((size_t)(y + 1) * w + x) * 3 + c];
                s += fabs(l);
                n++;
            }
    return n > 0 ? s / n : 0.0;
}

/* 对外入口：就地把 rgb（线性，1.0 = SDR 白）做一遍三级金字塔降噪。
 * coef = 9 个噪声系数（逐通道 a/b/c，与降噪/SFD 融合同一份标定）。
 * 返回实际跑的级数（0 = 关掉）。 */
int x3f_stage1_denoise(float *rgb, int w, int h, const double coef[9], double lin_sat)
{
    int levels_env, strength_on;
    float *cur = NULL, *tmp = NULL, *fil = NULL, *edge = NULL;
    float *small = NULL;
    /* 逐级 2× 盒式降采样链的缓存：ds[k] = 原图降 k+1 次的结果。
     * 原来每一级都从**原图**重新走一遍链，1/4 那级会把 1/2 那级已经算过的
     * 第一遍再算一遍（58MB 的一趟盒式降采样），三级下来白算两趟。
     * 缓存后各级共用，且算式一字未改 —— 输出逐位一致。 */
    float *ds[3] = { NULL, NULL, NULL };
    int dsw[3] = { 0, 0, 0 }, dsh[3] = { 0, 0, 0 };
    int cw = w, ch = h, level, k;
    /* 核半径：SPP 在降采样图上用 11~31 的核长（半径 5~15）。这里按级给：
     * 1/4 用 6、1/2 用 4、原图用 2 —— 等效到原图的半径分别是 24/8/2 px，
     * 与原文档"大核跑小图"的用意一致，代价是我们要跑得动。 */
    static const int radii[3] = {6, 4, 2};
    static const int keep_scale[3] = {4, 2, 1};   /* 每级 = 原图的 1/keep */
    double strength = s1_env_num("X3F_STAGE1_STRENGTH", 0.6);
    /* Lab 旋钮：只放大"零亮度"的那部分修正，默认 1.0 = 原行为。
     * 2~3 是给「融合后色度降噪」做 A/B 用的；正式 UI 要不要暴露另说。 */
    double chroma = s1_env_num("X3F_STAGE1_CHROMA", 1.0);
    g_s1_nscale = s1_env_num("X3F_STAGE1_NSIGMA", 4.0);
    if (!(g_s1_nscale > 0.05)) g_s1_nscale = 0.05;
    if (!(chroma >= 0.0)) chroma = 1.0;

    levels_env = (int)s1_env_num("X3F_STAGE1", 3.0);
    if (levels_env <= 0 || strength <= 0.0 || w < 16 || h < 16) return 0;
    if (levels_env > 3) levels_env = 3;

    /* acc 一开始就是原图：每一级只把**平坦区**换成那一级滤波后的版本，
     * 有结构的地方一直保留更细一级的数据 —— 这是"干净但细节在"的关键。 */
    cur = (float *)malloc((size_t)w * h * 3 * sizeof(float));
    if (cur == NULL) return 0;
    memcpy(cur, rgb, (size_t)w * h * 3 * sizeof(float));
    /* 2026-10-25：原来每级都要一块**全幅**的 up（w*h*3，19.6M 像素下
     * 235MB）做"升采样 → cur += up"，三级就是 3 次大块 malloc/free 加
     * 3 趟全幅搬运；现在升采样直接累加进 cur（s1_up_add），这一块连同
     * 那 3 趟搬运一起没了 —— 也是 Stage1 偶发 6 倍暴涨（实测 16s → 99s）
     * 的头号嫌疑：大块分配撞上系统内存压缩会有一片页错误。
     * 算式一字未改，输出逐位一致（(float) 转换仍在累加之前）。 */

    for (level = levels_env - 1; level >= 0; level--) {
        int dw = (w + keep_scale[level] - 1) / keep_scale[level];
        int dh = (h + keep_scale[level] - 1) / keep_scale[level];
        int r = radii[level];
        double level_scale = 1.0 / (double)keep_scale[level];
        int i;

        /* small / fil / edge 三块按**最细那一级**开一次，后面几级尺寸更小、
         * 直接用它的前 dw*dh 段（level 从细往粗走，dw/dh 单调不增）。
         * 同样是为了消掉每级一次的大块 malloc/free。 */
        if (small == NULL) {
            small = (float *)malloc((size_t)dw * dh * 3 * sizeof(float));
            fil = (float *)malloc((size_t)dw * dh * 3 * sizeof(float));
            edge = (float *)malloc((size_t)dw * dh * sizeof(float));
            if (small == NULL || fil == NULL || edge == NULL) break;
        }
        /* 降到这一级的尺寸（逐级 2× 盒式，和 ResampleImage 那条一致） */
        {
            float *src = rgb;          /* 这一级的输入永远从**原图**降采样 */
            int sw = w, sh = h, nd = 0;
            while (sw > dw && sh > dh && nd < 3) {
                int nw = (sw + 1) / 2, nh = (sh + 1) / 2;
                if (ds[nd] == NULL) {
                    ds[nd] = (float *)malloc((size_t)nw * nh * 3 * sizeof(float));
                    if (ds[nd] == NULL) break;
                    s1_down2(src, sw, sh, ds[nd], nw, nh);
                    dsw[nd] = nw; dsh[nd] = nh;
                }
                src = ds[nd]; sw = dsw[nd]; sh = dsh[nd];
                nd++;
            }
            memcpy(small, src, (size_t)dw * dh * 3 * sizeof(float));
        }

        s1_bilateral(small, fil, dw, dh, r, coef, lin_sat, level_scale);
        s1_gate(small, dw, dh, r, coef, lin_sat, level_scale, edge);

        if (s1_debug()) {
            double emean = 0.0, edone = 0.0;
            int i2;
            for (i2 = 0; i2 < dw * dh; i2++) {
                emean += edge[i2];
                if (edge[i2] > 0.5) edone += 1.0;
            }
            fprintf(stderr, "[stage1] 级%d(%d/%d) r=%d 平均edge=%.3f 平坦像素=%.1f%% "
                            "hf 前 %.5f 后 %.5f\n",
                    level, keep_scale[level], 1, r, emean / (dw * dh),
                    100.0 * edone / (dw * dh), s1_hf(small, dw, dh), s1_hf(fil, dw, dh));
        }

        /* 只回填**低频修正**（这一处是关键）：
         *
         *     corr = e · (滤波后 − 本级原值)      （e = 闸门，平坦区大）
         *     acc   = 原图 + 升采样(corr)
         *
         * 早先是"平坦区直接换成升采样后的滤波结果"，那等于把 1/4 / 1/2 尺度的
         * 块结构搬回原图 —— 平滑面上就出现一块一块的方块（用户看到的就是这个，
         * 不是压缩：16 位 TIFF 无压缩）。改成只叠加修正量之后，原图的细节一点
         * 不动，去掉的只是各尺度的低频噪声。 */
        for (i = 0; i < dw * dh; i++) {
            double e = pow((double)edge[i], 2.5) * strength;
            double cr[3], cy;
            int c;
            if (e > 1.0) e = 1.0;
            for (c = 0; c < 3; c++)
                cr[c] = e * (fil[i * 3 + c] - small[i * 3 + c]);
            /* 把修正拆成"亮度"与"零亮度色度"两部分；chroma 只乘后者，
             * 所以色度加强不会把整幅亮度/结构一起带走。 */
            cy = 0.2126 * cr[0] + 0.7152 * cr[1] + 0.0722 * cr[2];
            for (c = 0; c < 3; c++)
                small[i * 3 + c] = (float)(cy + chroma * (cr[c] - cy));
        }

        s1_up_add(small, dw, dh, cur, w, h);   /* 直接累加进 cur，不再过 up */
    }
    free(small); free(fil); free(edge);
    small = NULL; fil = NULL; edge = NULL;
    for (k = 0; k < 3; k++) { free(ds[k]); ds[k] = NULL; }

    if (strength < 1.0) {
        /* 总强度（A/B 用）：在原始与降噪结果之间插值 */
        size_t i, n = (size_t)w * h * 3;
        for (i = 0; i < n; i++)
            rgb[i] = (float)(strength * cur[i] + (1.0 - strength) * rgb[i]);
    } else {
        memcpy(rgb, cur, (size_t)w * h * 3 * sizeof(float));
    }
    free(cur);
    return levels_env;
}
