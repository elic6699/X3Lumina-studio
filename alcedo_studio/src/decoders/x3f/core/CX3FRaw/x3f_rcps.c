/* x3f_rcps.c -- SPP RowColumnPatternSuppression 复刻 v2（Merrill 行/列 FPN）。
 *
 * 出处：_
 * §1 + §27 + §28.5 + §29 + §30）。相机算法（DenoiseF20 内，单通道
 * Flt32Mono 残差域，值围绕 1.0）三层结构：
 *
 *   ① memcpy(dst←src)
 *   ② 列 pass + 行 pass：GenResidualMinMaxMap（窗口 2r+1 的残差 min/max，阈
 *      ResidualRCPS_Threshold=0.075）产出门控图；map!=0 的像素做窗口内随机
 *      交换（2r+1 次随机 transposition，值守恒），行 pass 偏移 × stride
 *   ③ 最终保险丝：仅当原始残差 |v−1| < ResidualRCPS_ThresholdMap=0.1 时保留
 *      置换结果，否则还原；blend（ResidualRCPS_Blending=1.0）作用在保险丝内
 *   边界：四周 2r+1 像素不动。
 *
 * 参数默认值 = SPP SetUpNoiseParams_F20camfile 键名直读）：
 *   ResidualRCPS_Threshold=0.075、ResidualRCPS_ThresholdMap=0.1、
 *   ResidualRCPS_Blending=1.0、PermSizeHalf=4。
 *
 * **接入域（§30 定案，2026-10-27 深夜）**：显示域列纹与 lin 列纹基本无关
 * （corr 0.058-0.21，§27.1 早已实测「±4 竖纹主体是 apply_decode_unsharp
 * 制造的色度列纹混入」）——lin 域接入（v1/v2 初版）原理上压不住显示域条纹，
 * 显示域仿真验证有效（主纹 hp9range 压掉 84%）。故提供两个入口：
 *
 *   x3f_rcps_apply(lin, w, h)        lin 域（保留：SFD/诊断用，价值有限）
 *   x3f_rcps_apply_disp(img,bits,..) 显示域（产品正道：tone 后、unsharp 前，
 *                                    由 x3f_render.c apply_ex_common 调用）
 *
 * 两版共用同一个 float 核心 rcps_core（门控/交换/保险丝三层）；显示域版
 * 解包 → 核心 → 钳位写回。置换偏移对 R/G/B 三通道同步（整像素交换）——
 * 保住色彩相关性（v1 色斑问题的修复点）。
 *
 * 环境变量（全部可省，默认 = SPP 参数）：
 *   X3F_RCPS=1           开关（缺省关，零回归）
 *   X3F_RCPS_R=4         半窗口 r（PermSizeHalf）
 *   X3F_RCPS_MTHR=0.075  门控图阈（ResidualRCPS_Threshold）
 *   X3F_RCPS_THR=0.1     保险丝阈（ResidualRCPS_ThresholdMap）
 *   X3F_RCPS_BLEND=1.0   混合强度（ResidualRCPS_Blending）
 *   X3F_RCPS_PASS=hv     作用方向：h=只水平（列纹） v=只垂直（行纹） hv=两轮
 *   X3F_RCPS_DEBUG=1     打印门控覆盖率/交换次数/保险丝通过率/耗时
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "include/x3f_rcps.h"

/* ---- XorShift128（SPP 同款标准种子；表内容固定 → 输出确定可回归） ---- */
static unsigned int xs128(unsigned int s[4]) {
    unsigned int t = s[0] ^ (s[0] << 11);
    s[0] = s[1]; s[1] = s[2]; s[2] = s[3];
    s[3] = (s[3] ^ (s[3] >> 19)) ^ (t ^ (t >> 8));
    return s[3];
}

static int rcps_env_int(const char *name, int dflt, int lo, int hi) {
    const char *e = getenv(name);
    if (e == NULL || *e == '\0') return dflt;
    int v = atoi(e);
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return v;
}

static double rcps_env_dbl(const char *name, double dflt) {
    const char *e = getenv(name);
    if (e == NULL || *e == '\0') return dflt;
    return atof(e);
}

/* ---- 可分离滑动窗口统计（半径 r，边缘钳制；与 v1 盒均值同语义） ---- */

static void box_mean_plane(const float *src, int w, int h, int r,
                           float *tmp, float *dst) {
    const int wn = 2 * r + 1;
    const double inv = 1.0 / (double)wn;
    int x, y, i;
    for (y = 0; y < h; y++) {
        const float *row = src + (size_t)y * (size_t)w;
        double s = 0.0;
        for (i = -r; i <= r; i++) {
            int xx = i; if (xx < 0) xx = 0; if (xx >= w) xx = w - 1;
            s += row[xx];
        }
        for (x = 0; x < w; x++) {
            tmp[(size_t)y * w + x] = (float)(s * inv);
            int xa = x + r + 1; if (xa >= w) xa = w - 1;
            int xr = x - r;     if (xr < 0)  xr = 0;
            s += (double)row[xa] - (double)row[xr];
        }
    }
    for (x = 0; x < w; x++) {
        double s = 0.0;
        for (i = -r; i <= r; i++) {
            int yy = i; if (yy < 0) yy = 0; if (yy >= h) yy = h - 1;
            s += tmp[(size_t)yy * w + x];
        }
        for (y = 0; y < h; y++) {
            dst[(size_t)y * w + x] = (float)(s * inv);
            int ya = y + r + 1; if (ya >= h) ya = h - 1;
            int yr = y - r;     if (yr < 0)  yr = 0;
            s += (double)tmp[(size_t)ya * w + x] - (double)tmp[(size_t)yr * w + x];
        }
    }
}

/* 一维滑动 min/max（单调 deque；居中窗 [c−r, c+r] 用「虚拟下标延迟 r 输出」
 * 实现，右缘越界重复末值 = 钳制窗）。idx 队列长 n+r。 */
static void slide_minmax1d(const float *src, int n, int r,
                           float *out_min, float *out_max,
                           int *idx_min, int *idx_max) {
    const int last = n - 1;
    int hmin = 0, tmin = 0, hmax = 0, tmax = 0;
    int j, c;
    for (j = 0, c = 0; j < n + r; j++) {
        {
            const float v = src[(j <= last) ? j : last];
            while (tmin > hmin &&
                   src[idx_min[tmin - 1] <= last ? idx_min[tmin - 1] : last] >= v)
                tmin--;
            idx_min[tmin++] = j;
            while (tmax > hmax &&
                   src[idx_max[tmax - 1] <= last ? idx_max[tmax - 1] : last] <= v)
                tmax--;
            idx_max[tmax++] = j;
        }
        if (j >= r) {
            while (idx_min[hmin] < c - r) hmin++;
            while (idx_max[hmax] < c - r) hmax++;
            out_min[c] = src[idx_min[hmin] <= last ? idx_min[hmin] : last];
            out_max[c] = src[idx_max[hmax] <= last ? idx_max[hmax] : last];
            c++;
        }
    }
}

/* 二维可分离 min/max：out_min/out_max = (2r+1)² 窗口最小/最大。 */
static void win_minmax2d(const float *src, int w, int h, int r,
                         float *tmp, float *tmp2,
                         float *out_min, float *out_max,
                         int *idxq, int *idxq2) {
    int y, x;
    for (y = 0; y < h; y++)
        slide_minmax1d(src + (size_t)y * w, w, r,
                       tmp + (size_t)y * w, tmp2 + (size_t)y * w,
                       idxq, idxq2);
    {
        float *col = (float *)malloc((size_t)h * sizeof(float));
        float *cmn = (float *)malloc((size_t)h * sizeof(float));
        float *cmx = (float *)malloc((size_t)h * sizeof(float));
        if (!col || !cmn || !cmx) { free(col); free(cmn); free(cmx); return; }
        for (x = 0; x < w; x++) {
            int yy;
            for (yy = 0; yy < h; yy++) col[yy] = tmp[(size_t)yy * w + x];
            slide_minmax1d(col, h, r, cmn, cmx, idxq, idxq2);
            for (yy = 0; yy < h; yy++)
                out_min[(size_t)yy * w + x] = cmn[yy];
            for (yy = 0; yy < h; yy++) col[yy] = tmp2[(size_t)yy * w + x];
            slide_minmax1d(col, h, r, cmn, cmx, idxq, idxq2);
            for (yy = 0; yy < h; yy++)
                out_max[(size_t)yy * w + x] = cmx[yy];
        }
        free(col); free(cmn); free(cmx);
    }
}

/* AUTO 门：load 段按文件头 type_format==X3F_IMAGE_RAW_MERRILL 置位。 */
static int g_rcps_auto_active = 0;

void x3f_rcps_set_auto(int active) { g_rcps_auto_active = active; }

static int rcps_gate_on(void) {
    const char *e = getenv("X3F_RCPS");
    if (e != NULL && *e != '\0') return (*e != '0');
    return g_rcps_auto_active;
}

/* ---- float 核心：src 只读（保险丝/混合基准），dst 进出（预拷贝 src）。
 * 三层 = 门控图 → 窗口随机交换 → 逐像素保险丝。返回交换总次数。 ---- */
static size_t rcps_core(const float *src, float *dst, int w, int h) {
    const int r       = rcps_env_int("X3F_RCPS_R", 4, 1, 64);
    const double mthr = rcps_env_dbl("X3F_RCPS_MTHR", 0.075);
    const double thr  = rcps_env_dbl("X3F_RCPS_THR", 0.1);
    const double blnd = rcps_env_dbl("X3F_RCPS_BLEND", 1.0);
    const char *ps    = getenv("X3F_RCPS_PASS");
    int do_h = 1, do_v = 1;
    if (ps != NULL && *ps != '\0') {
        do_h = (strchr(ps, 'h') != NULL);
        do_v = (strchr(ps, 'v') != NULL);
    }
    const int dbg = rcps_env_int("X3F_RCPS_DEBUG", 0, 0, 1);

    const int wn = 2 * r + 1;
    const int nperm = wn << 9;
    const size_t npix = (size_t)w * (size_t)h;
    const int bx0 = wn, bx1 = w - wn, by0 = wn, by1 = h - wn;

    float *Y    = (float *)malloc(npix * sizeof(float));
    float *box  = (float *)malloc(npix * sizeof(float));
    float *res  = (float *)malloc(npix * sizeof(float));
    float *tmp  = (float *)malloc(npix * sizeof(float));
    float *tmp2 = (float *)malloc(npix * sizeof(float));
    float *rmin = (float *)malloc(npix * sizeof(float));
    float *rmax = (float *)malloc(npix * sizeof(float));
    unsigned char *map = (unsigned char *)malloc(npix);
    int *qa = (int *)malloc(((size_t)(w > h ? w : h) + wn + 1) * sizeof(int));
    int *qb = (int *)malloc(((size_t)(w > h ? w : h) + wn + 1) * sizeof(int));
    int *perm = (int *)malloc((size_t)nperm * sizeof(int));
    size_t nswap = 0;

    if (!Y || !box || !res || !tmp || !tmp2 || !rmin || !rmax || !map ||
        !qa || !qb || !perm) {
        free(Y); free(box); free(res); free(tmp); free(tmp2); free(rmin);
        free(rmax); free(map); free(qa); free(qb); free(perm);
        return 0;
    }

    /* ① 残差模拟：r = Y / box(Y)（Y=(R+G+B)/3，平坦区 ≈1.0） */
    {
        size_t i;
        for (i = 0; i < npix; i++)
            Y[i] = (src[i * 3] + src[i * 3 + 1] + src[i * 3 + 2]) * (1.0f / 3.0f);
    }
    box_mean_plane(Y, w, h, r, tmp, box);
    {
        size_t i;
        for (i = 0; i < npix; i++)
            res[i] = (box[i] > 1e-9f && isfinite(box[i])) ? Y[i] / box[i] : 1.0f;
    }

    /* ② 门控图（GenResidualMinMaxMap 同构）：窗内 min/max 都贴 1.0 才放行 */
    win_minmax2d(res, w, h, r, box, tmp2, rmin, rmax, qa, qb);
    {
        size_t i, cov = 0;
        for (i = 0; i < npix; i++) {
            const double mn = (double)rmin[i], mx = (double)rmax[i];
            const int ok = isfinite((float)mn) && isfinite((float)mx) &&
                           (mx - 1.0) < mthr && (1.0 - mn) < mthr;
            map[i] = (unsigned char)(ok ? 1 : 0);
            cov += (size_t)(ok != 0);
        }
        if (dbg)
            fprintf(stderr, "[rcps] 门控图覆盖 %.2f%%（MTHR=%.3f）\n",
                    100.0 * (double)cov / (double)npix, mthr);
    }

    /* 置换表：SPP 同构 */
    {
        unsigned int s[4] = { 123456789u, 362436069u, 521288629u, 88675123u };
        int i;
        for (i = 0; i < nperm; i++) perm[i] = (int)(xs128(s) % (unsigned)wn);
    }

    /* ③ 交换：门控像素窗口内 (2r+1) 次随机 transposition（整像素 RGB） */
    {
        int pi;
        struct { int dir, on; const char *name; } passes[2] = {
            { 0, do_h, "H(列纹)" }, { 1, do_v, "V(行纹)" }
        };
        for (pi = 0; pi < 2; pi++) {
            const size_t before = nswap;
            int k = 0;
            int xx, yy;
            if (!passes[pi].on) continue;
            for (yy = by0; yy < by1; yy++) {
                for (xx = bx0; xx < bx1; xx++) {
                    if (!map[(size_t)yy * w + xx]) continue;
                    const int base = (passes[pi].dir == 0) ? xx - r : yy - r;
                    int j;
                    for (j = 0; j < wn; j++) {
                        const int b = base + perm[k];
                        const int a = base + j;
                        k = (k + 1) % nperm;
                        if (a == b) continue;
                        {
                            size_t pa, pb;
                            int cch;
                            if (passes[pi].dir == 0) {
                                pa = ((size_t)yy * w + a) * 3;
                                pb = ((size_t)yy * w + b) * 3;
                            } else {
                                pa = ((size_t)a * w + xx) * 3;
                                pb = ((size_t)b * w + xx) * 3;
                            }
                            for (cch = 0; cch < 3; cch++) {
                                const float t = dst[pa + cch];
                                dst[pa + cch] = dst[pb + cch];
                                dst[pb + cch] = t;
                            }
                            nswap++;
                        }
                    }
                }
            }
            if (dbg)
                fprintf(stderr, "[rcps] pass=%s 交换 %zu 次\n",
                        passes[pi].name, nswap - before);
        }
    }

    /* ④ 保险丝：|r−1| < THR 才收置换结果，否则还原 src；blend 在保险丝内。
     * 四周 2r+1 边界一律原样（dst 预拷贝 = src，未触碰即原值）。 */
    {
        size_t kept = 0;
        int xx, yy;
        for (yy = by0; yy < by1; yy++) {
            for (xx = bx0; xx < bx1; xx++) {
                const size_t ip = (size_t)yy * w + xx;
                int cch;
                if (fabs((double)res[ip] - 1.0) >= thr) {
                    const size_t i3 = ip * 3;   /* 非平坦：还原（SPP : src） */
                    for (cch = 0; cch < 3; cch++) dst[i3 + cch] = src[i3 + cch];
                    continue;
                }
                {
                    const size_t i3 = ip * 3;   /* 平坦：blend·dst + (1−blend)·src */
                    for (cch = 0; cch < 3; cch++)
                        dst[i3 + cch] = (float)((double)dst[i3 + cch] * blnd +
                                                (double)src[i3 + cch] * (1.0 - blnd));
                }
                kept++;
            }
        }
        if (dbg)
            fprintf(stderr, "[rcps] 保险丝通过 %.2f%%（THR=%.3f blend=%.2f）\n",
                    100.0 * (double)kept / (double)npix, thr, blnd);
    }

    free(Y); free(box); free(res); free(tmp); free(tmp2); free(rmin);
    free(rmax); free(map); free(qa); free(qb); free(perm);
    return nswap;
}

/* ---- lin 域入口（SFD/诊断用；显示域条纹与 lin 基本无关，§30） ---- */
float *x3f_rcps_apply(const float *lin, int w, int h) {
    if (!rcps_gate_on()) return NULL;
    if (lin == NULL || w <= 0 || h <= 0) return NULL;
    const size_t n3 = (size_t)w * (size_t)h * 3;
    float *bufA = (float *)malloc(n3 * sizeof(float));
    if (!bufA) return NULL;
    memcpy(bufA, lin, n3 * sizeof(float));
    clock_t t0 = clock();
    rcps_core(lin, bufA, w, h);
    if (getenv("X3F_RCPS_DEBUG") != NULL)
        fprintf(stderr, "[rcps] lin 域 done %dx%d %.2fs\n", w, h,
                (double)(clock() - t0) / CLOCKS_PER_SEC);
    return bufA;
}

/* ---- 显示域入口（产品正道）：tone 后、unsharp 前的 sRGB 缓冲 ---- */
void x3f_rcps_apply_disp(void *imgv, int bits, int w, int h) {
    if (!rcps_gate_on()) return;
    if (imgv == NULL || w <= 0 || h <= 0) return;
    const size_t npix = (size_t)w * (size_t)h;
    const size_t n3 = npix * 3;
    const double scale = (bits >= 16) ? 65535.0 : 255.0;
    const double inv = 1.0 / scale;

    float *fsrc = (float *)malloc(n3 * sizeof(float));
    float *fdst = (float *)malloc(n3 * sizeof(float));
    if (!fsrc || !fdst) { free(fsrc); free(fdst); return; }

    /* 解包（sRGB 值归一到 0..1——门控全是比值，尺度无关） */
    {
        size_t i;
        if (bits >= 16) {
            const unsigned short *p = (const unsigned short *)imgv;
            for (i = 0; i < n3; i++) fsrc[i] = (float)((double)p[i] * inv);
        } else {
            const unsigned char *p = (const unsigned char *)imgv;
            for (i = 0; i < n3; i++) fsrc[i] = (float)((double)p[i] * inv);
        }
    }
    memcpy(fdst, fsrc, n3 * sizeof(float));

    clock_t t0 = clock();
    rcps_core(fsrc, fdst, w, h);

    /* 对质落盘（DEBUG>=2）：显示域进出缓冲 */
    if (rcps_env_int("X3F_RCPS_DEBUG", 0, 0, 2) >= 2) {
        FILE *f = fopen("/tmp/rcps_disp_in.f32", "wb");
        if (f) { fwrite(fsrc, sizeof(float), n3, f); fclose(f); }
        f = fopen("/tmp/rcps_disp_out.f32", "wb");
        if (f) { fwrite(fdst, sizeof(float), n3, f); fclose(f); }
        fprintf(stderr, "[rcps] 显示域进出已落盘 /tmp/rcps_disp_{in,out}.f32\n");
    }

    /* 写回：有变化才写，round + 钳位 */
    {
        size_t changed = 0, i;
        if (bits >= 16) {
            unsigned short *p = (unsigned short *)imgv;
            for (i = 0; i < n3; i++) {
                double v = (double)fdst[i] * scale;
                if (v < 0.0) v = 0.0;
                if (v > scale) v = scale;
                const unsigned short q = (unsigned short)(v + 0.5);
                if (q != p[i]) { p[i] = q; changed++; }
            }
        } else {
            unsigned char *p = (unsigned char *)imgv;
            for (i = 0; i < n3; i++) {
                double v = (double)fdst[i] * scale;
                if (v < 0.0) v = 0.0;
                if (v > scale) v = scale;
                const unsigned char q = (unsigned char)(v + 0.5);
                if (q != p[i]) { p[i] = q; changed++; }
            }
        }
        if (getenv("X3F_RCPS_DEBUG") != NULL)
            fprintf(stderr, "[rcps] 显示域 done %dx%d bits=%d 改动 %.2f%% %.2fs\n",
                    w, h, bits, 100.0 * (double)changed / (double)n3,
                    (double)(clock() - t0) / CLOCKS_PER_SEC);
    }
    free(fsrc); free(fdst);
}
