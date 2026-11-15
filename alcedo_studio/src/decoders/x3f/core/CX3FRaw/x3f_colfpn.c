/* Merrill 列 FPN（竖条纹杂讯）标定式校正 —— 显示域版实现。
 * 设计与"为什么在显示域做"见 include/x3f_colfpn.h；原型与验证见
 * §24。*/
#include "x3f_colfpn.h"

#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

static double colfpn_thr(void)
{
    const char *e = getenv("X3F_COLFPN_THR");
    /* 原型实测：0.35/255 以下视为噪声（649 列校正、峰值 ±5.4/255）。 */
    return (e != NULL && atof(e) > 0.0) ? atof(e) : 0.35;
}

/* ---------------------------------------------------------------- */
/* 一维盒滤波（k=9），reflect 边界。 */

static void box9_f(const float *x, float *out, int n)
{
    int i, k;
    for (i = 0; i < n; i++) {
        double s = 0.0;
        for (k = -4; k <= 4; k++) {
            int j = i + k;
            if (j < 0) j = -j;
            if (j >= n) j = 2 * n - 2 - j;
            if (j < 0) j = 0;
            if (j >= n) j = n - 1;
            s += x[j];
        }
        out[i] = (float)(s / 9.0);
    }
}

/* 解卷积：给定 hp9 的输出 h，恢复 hp9 的输入 o（不动点迭代 8 轮）。
 * hp9(v) = v − box9(v) → v = h + box9(v)；收缩因子 ≤ 8/9。 */
static void deconv_hp9(const float *h, float *o, float *tmp, int n)
{
    int it, i;
    memcpy(o, h, (size_t)n * sizeof(float));
    for (it = 0; it < 8; it++) {
        box9_f(o, tmp, n);
        for (i = 0; i < n; i++)
            o[i] = (float)(h[i] + tmp[i]);
    }
}

/* ---------------------------------------------------------------- */
/* 缓存：最近成功标定的 offset 表（键 = w×h，单位 0..255 float）。 */

#define COLFPN_SLOTS 4

static struct {
    int used, w, h;
    float *off;          /* w*3，0..255 口径，已过幅度门限 */
} colfpn_cache[COLFPN_SLOTS];

static pthread_mutex_t colfpn_lock = PTHREAD_MUTEX_INITIALIZER;

static const float *colfpn_cache_get(int w, int h)
{
    const float *r = NULL;
    int i;
    pthread_mutex_lock(&colfpn_lock);
    for (i = 0; i < COLFPN_SLOTS; i++) {
        if (colfpn_cache[i].used && colfpn_cache[i].w == w &&
            colfpn_cache[i].h == h) {
            r = colfpn_cache[i].off;
            break;
        }
    }
    pthread_mutex_unlock(&colfpn_lock);
    return r;
}

static void colfpn_cache_put(int w, int h, const float *off)
{
    int i, victim = -1;
    pthread_mutex_lock(&colfpn_lock);
    for (i = 0; i < COLFPN_SLOTS; i++) {
        if (colfpn_cache[i].used && colfpn_cache[i].w == w &&
            colfpn_cache[i].h == h) {
            victim = i;
            break;
        }
        if (!colfpn_cache[i].used && victim < 0) victim = i;
    }
    if (victim < 0) victim = 0;
    if (!colfpn_cache[victim].used) {
        colfpn_cache[victim].off =
            (float *)malloc((size_t)w * 3 * sizeof(float));
        colfpn_cache[victim].used = colfpn_cache[victim].off != NULL;
    }
    if (colfpn_cache[victim].used) {
        colfpn_cache[victim].w = w;
        colfpn_cache[victim].h = h;
        memcpy(colfpn_cache[victim].off, off, (size_t)w * 3 * sizeof(float));
    }
    pthread_mutex_unlock(&colfpn_lock);
}

/* ---------------------------------------------------------------- */

static int cmp_f(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static inline double px_at(const void *vout, int stride, int bits,
                           int y, int x, int ch, double scale)
{
    if (bits == 16)
        return (double)((const uint16_t *)vout)[(size_t)y * (stride / 2)
                                               + (size_t)x * 3 + ch] * scale;
    return (double)((const uint8_t *)vout)[(size_t)y * stride
                                           + (size_t)x * 3 + ch] * scale;
}

void x3f_colfpn_apply_disp(void *vout, int w, int h, int stride, int bits)
{
    const char *e = getenv("X3F_COLFPN");
    double thr;
    const float *use_off = NULL;
    double *rowgrad = NULL, *rowmean = NULL;
    int *rows = NULL;
    float *samp = NULL, *prof = NULL, *bx = NULL, *hb = NULL;
    float *ob = NULL, *tmp1 = NULL, *off = NULL, *pc = NULL;
    const double scale = (bits == 16) ? 255.0 / 65535.0 : 1.0;
    int max_rows, n_rows = 0, got, ch, x, i, y;
    int nfix = 0;
    double peak = 0.0;
    int dbg;
    const char *srcname = "fresh";

    if (e == NULL || e[0] != '1' || vout == NULL || w < 64 || h < 64)
        return;
    thr = colfpn_thr();
    dbg = getenv("X3F_COLFPN_DEBUG") != NULL;
    if (dbg) {
        /* hook 时刻的原始像素抽查：x1594 列对（G 通道） */
        for (y = 100; y < 103; y++)
            fprintf(stderr, "[colfpn] hook y=%d: G1590=%.1f G1594=%.1f G1598=%.1f\n",
                    y, px_at(vout, stride, bits, y, 1590, 1, scale),
                       px_at(vout, stride, bits, y, 1594, 1, scale),
                       px_at(vout, stride, bits, y, 1598, 1, scale));
    }

    /* ---- 1. 平滑亮行候选（0..255 口径：亮度窗 80..240，原型同款） ---- */
    rowgrad = (double *)malloc((size_t)h * sizeof(double));
    rowmean = (double *)malloc((size_t)h * sizeof(double));
    rows    = (int *)malloc((size_t)h * sizeof(int));
    if (!rowgrad || !rowmean || !rows) goto fail;

    for (y = 0; y < h; y++) {
        double gs = 0.0, ms = 0.0, lp, l;
        lp = (px_at(vout, stride, bits, y, 0, 0, scale)
              + px_at(vout, stride, bits, y, 0, 1, scale)
              + px_at(vout, stride, bits, y, 0, 2, scale)) / 3.0;
        ms += lp;
        for (x = 1; x < w; x++) {
            l = (px_at(vout, stride, bits, y, x, 0, scale)
                 + px_at(vout, stride, bits, y, x, 1, scale)
                 + px_at(vout, stride, bits, y, x, 2, scale)) / 3.0;
            gs += fabs(l - lp);
            ms += l;
            lp = l;
        }
        rowgrad[y] = gs / (w - 1);
        rowmean[y] = ms / w;
    }
    /* 标定噪声 ∝ 1/√rows：上限 512 行（原型 269 行已够；行多更稳）。 */
    max_rows = h / 4;
    if (max_rows > 512) max_rows = 512;
    if (max_rows < 16) max_rows = 16;

    n_rows = 0;
    /* 标定行限制在**上半幅**（原型 y100..560 同款）：0877 实测列纹是
     * 区域性的——天空区 x1594 列纹 ±10/255 极干净，而全图行集（含
     * 中下部暗平滑区）剖面被稀释到无纹。天空/上部亮区是列纹最纯的
     * 标定源。亮度窗上限 205：避开 tone 平顶区（rowmean≈235 的近白行
     * 列纹被 tone 导数压平，0877 首测在那里标出零纹）。 */
    for (y = 0; y < h / 2; y++)
        if (rowmean[y] >= 100.0 && rowmean[y] <= 205.0) rows[n_rows++] = y;
    if (n_rows > max_rows) {
        for (i = 0; i < max_rows; i++) {
            int best = i;
            for (x = i + 1; x < n_rows; x++)
                if (rowgrad[rows[x]] < rowgrad[rows[best]]) best = x;
            { int t = rows[i]; rows[i] = rows[best]; rows[best] = t; }
        }
        n_rows = max_rows;
    }
    got = n_rows;

    if (got < 16) {
        /* 本图无可标定平坦区 → 回落缓存表 */
        use_off = colfpn_cache_get(w, h);
        srcname = "cached";
        if (use_off == NULL) {
            if (dbg)
                fprintf(stderr, "[colfpn] %dx%d disp: no flat rows, no cache\n",
                        w, h);
            goto done;
        }
        goto apply;
    }

    /* ---- 2. 平滑行逐列中位数剖面（抗离群） ---- */
    samp = (float *)malloc((size_t)got * sizeof(float));
    prof = (float *)malloc((size_t)w * 3 * sizeof(float));
    bx   = (float *)malloc((size_t)w * sizeof(float));
    hb   = (float *)malloc((size_t)w * sizeof(float));
    ob   = (float *)malloc((size_t)w * sizeof(float));
    tmp1 = (float *)malloc((size_t)w * sizeof(float));
    off  = (float *)calloc((size_t)w * 3, sizeof(float));
    pc   = (float *)malloc((size_t)w * sizeof(float));
    if (!samp || !prof || !bx || !hb || !ob || !tmp1 || !off || !pc) goto fail;

    for (ch = 0; ch < 3; ch++) {
        for (x = 0; x < w; x++) {
            for (i = 0; i < got; i++)
                samp[i] = (float)px_at(vout, stride, bits, rows[i], x, ch, scale);
            qsort(samp, (size_t)got, sizeof(float), cmp_f);
            prof[(size_t)x * 3 + ch] = samp[got / 2];
        }
        /* ---- 3. 窄尺度高通 + 精确解卷积 ---- */
        if (dbg && ch == 1)
            fprintf(stderr, "[colfpn] dbg prof[1592..1596] = %.2f %.2f %.2f %.2f %.2f  "
                    "samp[0..2]=%.2f %.2f %.2f rows[0..2]=%d %d %d stride=%d bits=%d\n",
                    prof[1592*3+1], prof[1593*3+1], prof[1594*3+1],
                    prof[1595*3+1], prof[1596*3+1],
                    samp[0], samp[1], samp[2], rows[0], rows[1], rows[2], stride, bits);
        for (x = 0; x < w; x++) pc[x] = prof[(size_t)x * 3 + ch];
        box9_f(pc, bx, w);
        for (x = 0; x < w; x++) hb[x] = pc[x] - bx[x];
        deconv_hp9(hb, ob, tmp1, w);
        /* ---- 4. 幅度门限 ---- */
        for (x = 0; x < w; x++) {
            if (fabs(ob[x]) < thr) off[(size_t)x * 3 + ch] = 0.0f;
            else {
                off[(size_t)x * 3 + ch] = ob[x];
                nfix++;
                if (fabs(ob[x]) > peak) peak = fabs(ob[x]);
            }
        }
    }
    free(samp); samp = NULL;
    free(prof); prof = NULL;
    free(bx); bx = NULL;
    free(hb); hb = NULL;
    free(ob); ob = NULL;
    free(tmp1); tmp1 = NULL;
    free(pc); pc = NULL;

    if (nfix == 0) {
        free(off); off = NULL;
        if (dbg) {
            /* 诊断：剖面本身的高通幅度（thr 滤之前） */
            double hbmax = 0.0;
            for (x = 0; x < w; x++)
                if (fabs(hb[x]) > hbmax) hbmax = fabs(hb[x]);
            fprintf(stderr, "[colfpn] %dx%d disp fresh: rows=%d, no FPN above thr "
                    "(pre-thr hb max=%.3f ob max=%.3f)\n", w, h, got, hbmax, peak);
        }
        goto done;
    }
    colfpn_cache_put(w, h, off);
    use_off = colfpn_cache_get(w, h);
    free(off); off = NULL;
    if (use_off == NULL) goto fail;

apply:
    /* ---- 5. 全图逐列全局减（四舍五入到整码） ---- */
    if (bits == 16) {
        uint16_t *p16 = (uint16_t *)vout;
        size_t rw = (size_t)stride / 2;
        for (y = 0; y < h; y++) {
            uint16_t *p = p16 + (size_t)y * rw;
            for (x = 0; x < w; x++)
                for (ch = 0; ch < 3; ch++) {
                    int v = (int)p[(size_t)x * 3 + ch]
                            - (int)lround(use_off[(size_t)x * 3 + ch] / scale);
                    p[(size_t)x * 3 + ch] = (uint16_t)(v < 0 ? 0 : v);
                }
        }
    } else {
        uint8_t *p8 = (uint8_t *)vout;
        for (y = 0; y < h; y++) {
            uint8_t *p = p8 + (size_t)y * stride;
            for (x = 0; x < w; x++)
                for (ch = 0; ch < 3; ch++) {
                    int v = (int)p[(size_t)x * 3 + ch]
                            - (int)lround(use_off[(size_t)x * 3 + ch] / scale);
                    p[(size_t)x * 3 + ch] = (uint8_t)(v < 0 ? 0 : v);
                }
        }
    }
    if (dbg)
        fprintf(stderr, "[colfpn] %dx%d disp src=%s rows=%d fixcols=%d peak=%.2f/255 thr=%.2f\n",
                w, h, srcname, got, nfix, peak, thr);

done:
    free(rowgrad); free(rowmean); free(rows);
    return;

fail:
    free(samp); free(prof); free(bx); free(hb); free(ob); free(tmp1); free(off);
    free(pc);
    free(rowgrad); free(rowmean); free(rows);
}
