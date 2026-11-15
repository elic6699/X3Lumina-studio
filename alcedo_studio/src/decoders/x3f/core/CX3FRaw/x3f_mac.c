/* x3f_mac.c -- SPP Stage2 MAC（MultiAxisCC 像素通路）产品接入（2026-10-26）
 *
 * 「MAC 表溯源 IV（终章）」——构造循环
 * 纯 C ，与 SPP 880/880
 * （5 个色彩模式全过）；像素应用（ApplyMatrx rotate 变体）复现与
 * sppref/apply_mac.c（此前已对 SPP out.bin 100% ）双实现对拍
 * 0/449820 位差。
 *
 * SPP 里这一段在 Stage2 管线 NR 之后、CCM 之前，作用域 = 相机域(raw·WB) 的
 * 16bit 码值：逐像素算 YCoCg 型 Co/Cg → 4 象限 × 5 比率区间选记录 → 命中
 * 记录的 3×3 累加替换。表不是查来的，是 MultiAxisCC 现场构造的：
 *   输入 = CAMF 的 MultiAxisTable_<色彩模式>（dims=[2,5,21]，组A=角度修正、
 * 组B=强度倍率，各 5×21，构造只用行 0 的 21 个数）+ 内置常量。
 *
 * 产品接入口径（与 SPP 行为对齐的三条）：
 *   1. gate：CAMF 里必须同时有 gausMACC_sRGB_MCK（矩阵池）与 MultiAxisTable_。
 *      0337 这类无池稿件 SPP 整段 no-op（out == NR 输出已实测 100%），我们
 *      同样跳过 —— 这也是零滑杆 md5 铁律不被本段扰动的结构保证。
 *      注意 MCK 是 u16 定点条目（CAMF mtype=6），FFI 的 f64 取数口只按
 *      M_FLOAT 查（实测永远失败），必须走 legacy CAMF 直查（M_UINT）。
 * 2. 域：本链 src 是显示域（cam_to_rgb 已乘）， MAC 在相机域(raw·WB)。
 * 用 rgb_to_cam 共轭 + as-shot 增益抬回 域，做完再变回来。
 *      共轭只在 gate 放行时发生，无表稿件逐位 no-op。
 *   3. 刻度：SPP MAC 的判定/替换发生在 16bit 码值域（gray 直通有 (int) 截断；
 *      象限比率 s16/s17 虽尺度无关，但整个 pass 必须在码值域内自洽）。
 *      本链相机域是"白点=1.0"的比例域，×X3F_MAC_SCALE（默认 16383，= 14bit
 *      raw 满量程）抬到码值域，做完除回。
 *   模式名：X3F_MAC_MODE 覆盖；缺省用文件头记录的色彩模式（Quattro v4 头
 *   没有的先按 Standard 试，SPP 打开照片时 UI 初值也是文件模式）。
 *
 * 位级三坑（都踩过，见 README 溯源 IV）：
 * ①9p0 是数值不是位模式 —— 必须按位 memcpy 解包；
 *   ② co1/co2 是 fnmul+fmadd 一次舍入；
 *   ③ c*A+1.0f 单表达式会被 clang -O2 熔合成 fma —— 拆两条语句。
 *   另：不要用 float *M = &M00 的指针技巧，独立局部变量不保证栈上连续。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "x3f_mac.h"
#include "x3f_io.h"
#include "x3f_meta.h"
#include "x3f_process.h"   /* x3f_get_gain */

/* 渲染层/解码层的既有接口（避免为这段改渲染层） */
extern int x3f_camf_matrix_f64(const char *path, const char *name,
                               double *out, int cap, unsigned *dims);
extern int x3fdec_get_camf_matrix_var(x3f_t *x3f, const char *name,
                                      int *dim0, int *dim1, int *dim2,
                                      matrix_type_t type, double *out, int cap);
extern const char *x3f_standalone_path(const char *path, int idx,
                                       char *buf, int buflen);
int x3f_render_probe_header_color_mode(const char *path, char *out, int cap);

typedef struct { float m[9]; int32_t key, idx; } mac_rec_t;

/*---- W 常量 = fmov 立即数的 IEEE 754 位模式（必须按位解包） ----*/
static float mac_bitsf(unsigned u)
{
    float f;
    memcpy(&f, &u, 4);
    return f;
}

/*内置角度表（20 floats，18..360 步长 18）；跨读位 = 0.0*/
static const float mac_ang[20] = {
    18, 36, 54, 72, 90, 108, 126, 144, 162, 180,
    198, 216, 234, 252, 270, 288, 306, 324, 342, 360
};

/* FColorFloat32（sp+0x470）：f0-f2 luma、f3-f5 Co、f6-f8 Cg（位模式照抄） */
static const float mac_fcolor[9] = {
    0.30078125f, 0.5859375f, 0.11328125f,
    -0.16796875f, -0.33203125f, 0.5f,
    0.5f, -0.41796875f, -0.08203125f
};

/* 范围表 sp+0xe0：idx=k 的判定带 = (RANGE[k], RANGE[k-1]] */
static const float mac_range[8] = {
    2048.0f, 1546.0f, 1186.0f, 862.0f,
    502.0f, 0.0f, 0.00549153471f, 0.00540244579f
};

static float mac_fmaddf(float a, float b, float c) { return fmaf(a, b, c); }

/* as-shot 白平衡增益（与 x3f_render.c 的 read_as_shot_gain 同源同实现；
 * 独立一份是因为那边是 static，而这边的 gate 一次开文件时顺手就要用） */
static int mac_as_shot_gain(const char *path, double g[3])
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

/* gate ① + gate ②：一次打开文件，确认 MCK 矩阵池存在并取回
 * MultiAxisTable_<模式> 的 210 个数。返回 0 = 任一 gate 不过（逐位 no-op）。 */
static int mac_gates_and_table(const char *path, const char *mode,
                               double tbl[210])
{
    char tmp[1024];
    const char *use = x3f_standalone_path(path, -1, tmp, sizeof(tmp));
    FILE *f = fopen(use, "rb");
    x3f_t *x3f;
    x3f_directory_entry_t *de;
    int ok = 0;

    if (f == NULL) return 0;
    x3f = x3f_new_from_file(f);
    if (x3f == NULL) { fclose(f); return 0; }
    de = x3f_get_camf(x3f);
    if (de != NULL && x3f_load_data(x3f, de) == X3F_OK) {
        int d0, d1, d2;
        double probe[8];
        /* gate ①：u16 定点池条目（CAMF mtype=6），类型精确匹配 */
        if (x3fdec_get_camf_matrix_var(x3f, "gausMACC_sRGB_MCK",
                                       &d0, &d1, &d2, M_UINT, probe, 8) ||
            x3fdec_get_camf_matrix_var(x3f, "gausMACC_sRGB_MCK",
                                       &d0, &d1, &d2, M_INT, probe, 8)) {
            /* gate ②：MultiAxisTable_<模式>，dims=[2,5,21] float */
            char name[96];
            snprintf(name, sizeof(name), "MultiAxisTable_%s", mode);
            memset(tbl, 0, 210 * sizeof(double));
            if (x3fdec_get_camf_matrix_var(x3f, name,
                                           &d0, &d1, &d2, M_FLOAT, tbl, 210)) {
                if (d0 == 2 && d1 == 5)
                    ok = 1;
                else if (getenv("X3F_MAC_DEBUG"))
                    fprintf(stderr, "[mac] %s dims=%d,%d,%d 形状不符\n",
                            name, d0, d1, d2);
            } else if (getenv("X3F_MAC_DEBUG")) {
                fprintf(stderr, "[mac] %s 未命中\n", name);
            }
        } else if (getenv("X3F_MAC_DEBUG")) {
            fprintf(stderr, "[mac] 无 gausMACC_sRGB_MCK 池 -> 整段 no-op\n");
        }
    }
    x3f_delete(x3f);
    fclose(f);
    return ok;
}

/*MultiAxisCC 构造循环。*/
static void mac_build_recs(const double *tbl, mac_rec_t recs[20])
{
    const float A0 = mac_bitsf(0x3b46f749u), A1 = mac_bitsf(0xbeb03eb5u),
                A2 = mac_bitsf(0x3fe2df03u);
    const float B0 = mac_bitsf(0x3fb3216cu), B1 = mac_bitsf(0xbf379a88u),
                B2 = mac_bitsf(0xbbc91526u);
    const float PI = mac_bitsf(0x40490fdbu);
    float knA[21], knB[21];
    int r;

    for (r = 0; r < 21; r++) {
        knA[r] = (float)tbl[r];           /* 组A 行 0 @ [0][0][*] */
        knB[r] = (float)tbl[105 + r];     /* 组B 行 0 @ [1][0][*] */
    }

    for (r = 0; r < 20; r++) {
        int key = r / 5 + 1, i = r % 5;
        int pos = (key - 1) * 5 + i + 1;  /* 滑窗 pos=1..20，跨读 pos-1 */
        float aA = knA[pos], aB = knA[pos - 1];
        float kB = knB[pos], kB2 = knB[pos - 1];
        float ang0 = mac_ang[key * 5 - 5 + i];
        float ang1 = (key * 5 - 6 + i >= 0) ? mac_ang[key * 5 - 6 + i] : 0.0f;
        float M00, M10, M20, M01, M11, M21, M02, M12, M22, M03, M13, M23;
        float v2, v4, v16, v17, det, inv;
        float al, be1, be2, ga1, ga2, ga3, co0, co1, co2;
        float *m = recs[r].m;
        int col;

        recs[r].key = key;
        recs[r].idx = i + 1;

        /* 四列方向：col0/col1 = 内置角度（无强度），col2/col3 = 角度+组A、
         * 强度乘在 cos/sin 上；L1 归一化用未乘 k 的分量。
         * ★不要用 float *M = &M00 的指针技巧：独立局部变量不保证栈上连续，
         * M[1]/M[2] 会写穿别的变量（实测挂死）。逐列显式赋值。 */
        for (col = 0; col < 4; col++) {
            float ang = (col == 0) ? ang0 : (col == 1) ? ang1
                      : (col == 2) ? (ang0 + aA) : (ang1 + aB);
            float k = (col == 2) ? kB : (col == 3) ? kB2 : 1.0f;
            float rad = (ang / 180.0f) * PI;
            float sn = sinf(rad), cs = cosf(rad);
            float L1 = fabsf(cs) + fabsf(sn);
            float c = (k * cs) / L1, s = (k * sn) / L1;
            /* base = c·A + 1（fmul+fadd 两次舍入，0x173440/0x17344c）。
             * 拆成两条语句写：单表达式 c*A+1.0f 会被 clang -O2 熔合成 fma
             * （位级三坑之三）；跨语句赋值 contract 不生效。 */
            float ca0 = c * A0, ca1 = c * A1, ca2 = c * A2;
            float b0 = ca0 + 1.0f, b1 = ca1 + 1.0f, b2 = ca2 + 1.0f;
            float r0 = mac_fmaddf(s, B0, b0);
            float r1 = mac_fmaddf(s, B1, b1);
            float r2 = mac_fmaddf(s, B2, b2);
            if (col == 0)      { M00 = r0; M10 = r1; M20 = r2; }
            else if (col == 1) { M01 = r0; M11 = r1; M21 = r2; }
            else if (col == 2) { M02 = r0; M12 = r1; M22 = r2; }
            else               { M03 = r0; M13 = r1; M23 = r2; }
        }

        /*2×2 求逆（ 寄存器流照抄；co1/co2 是 fnmul+fmadd
         * 一次舍入 —— 位级三坑之二）。 */
        v2 = M11 - M21;
        v4 = M10 - M20;
        v16 = -(M01 * v4);
        v16 = mac_fmaddf(M00, v2, v16);
        v17 = -(M20 * M11);
        v17 = mac_fmaddf(M10, M21, v17);
        det = v17 + v16;
        inv = 1.0f / det;
        al = v2 * inv;
        be1 = -((M01 - M21) * inv);
        be2 = (M01 - M11) * inv;
        ga1 = -(v4 * inv);
        ga2 = (M00 - M20) * inv;
        ga3 = -((M00 - M10) * inv);
        co0 = v17 * inv;
        co1 = -mac_fmaddf(M00, M21, -(M20 * M01)) * inv;
        co2 = mac_fmaddf(M00, M11, -(M01 * M10)) * inv;

        /*组装行主序，三段舍入照抄）*/
        {
            const float M2[3] = { M02, M12, M22 };
            const float M3[3] = { M03, M13, M23 };
            const float x[3] = { al, be1, be2 };
            const float y[3] = { ga1, ga2, ga3 };
            const float co[3] = { co0, co1, co2 };
            int a, b;
            for (a = 0; a < 3; a++)
                for (b = 0; b < 3; b++)
                    m[a * 3 + b] = co[b] + mac_fmaddf(M2[a], x[b], M3[a] * y[b]);
        }
    }
}

/*ApplyMatrx（rotate 变体）像素通路，float32 语义逐行照抄。
 * cam = 相机域码值 float RGB 交错，就地修改。 */
static void mac_apply_cam(float *cam, size_t npix, const mac_rec_t *recs)
{
    const float cden = mac_bitsf(0x2edbe6ffu);   /* fmov w15，~1e-10 */
    const float K = 2048.0f;                      /* 0x45000000 */
    size_t o;
    for (o = 0; o < npix; o++) {
        float *px = cam + o * 3;
        float R = px[0], G = px[1], B = px[2];
        float Co, Cg, aC, aG, den, s16, s17, accR, accG, accB;
        unsigned char flags[4];
        int r;
        if (R == G && R == B) {                       /* gray 直通：(int) 截断 */
            px[0] = (float)(int)R;
            px[1] = (float)(int)G;
            px[2] = (float)(int)B;
            continue;
        }
        /*舍入顺序照抄：G·f4 先 fmul 舍入，R·f3 走 fmadd，B·f5 再 fmadd*/
        {
            float t1 = G * mac_fcolor[4];
            Co = mac_fmaddf(R, mac_fcolor[3], t1);
            Co = mac_fmaddf(B, mac_fcolor[5], Co);
        }
        {
            float t2 = G * mac_fcolor[7];
            Cg = mac_fmaddf(R, mac_fcolor[6], t2);
            Cg = mac_fmaddf(B, mac_fcolor[8], Cg);
        }
        if (Co == 0.0f && Cg == 0.0f) {
            px[0] = (float)(int)R;
            px[1] = (float)(int)G;
            px[2] = (float)(int)B;
            continue;
        }
        flags[0] = (unsigned char)((Co > 0.0f) & (Cg > 0.0f));
        flags[1] = (unsigned char)((Co <= 0.0f) & (Cg > 0.0f));
        flags[2] = (unsigned char)((Co <= 0.0f) & (Cg <= 0.0f));
        flags[3] = (unsigned char)((Co > 0.0f) & (Cg <= 0.0f));
        aC = fabsf(Co);
        aG = fabsf(Cg);
        den = (aC + aG) + cden;
        s16 = mac_fmaddf(aC, K, cden) / den;
        s17 = mac_fmaddf(aG, K, cden) / den;
        accR = 0.0f; accG = 0.0f; accB = 0.0f;
        for (r = 0; r < 20; r++) {
            const mac_rec_t *q = &recs[r];
            float coord = (((uint32_t)q->key & 0x80000001u) == 1u) ? s16 : s17;
            float p1, ar, p2, ag, p3, ab;
            if (!flags[q->key - 1]) continue;
            if (coord > mac_range[q->idx - 1]) continue;  /* > 上界 -> 跳过 */
            if (!(coord > mac_range[q->idx])) continue;   /* <= 下界 -> 跳过 */
            p1 = G * q->m[1];
            ar = mac_fmaddf(B, q->m[2], mac_fmaddf(R, q->m[0], p1));
            accR = accR + ar;
            p2 = G * q->m[4];
            ag = mac_fmaddf(B, q->m[5], mac_fmaddf(R, q->m[3], p2));
            accG = accG + ag;
            p3 = G * q->m[7];
            ab = mac_fmaddf(B, q->m[8], mac_fmaddf(R, q->m[6], p3));
            accB = accB + ab;
        }
        /*三路皆非零才替换（ 的 cset/csel/tst 链）*/
        if (accR != 0.0f && accG != 0.0f && accB != 0.0f) {
            px[0] = accR; px[1] = accG; px[2] = accB;
        }
    }
}

void x3f_mac_stage(const char *path, float *src, unsigned w, unsigned h,
                   const double rgb_to_cam[9], const double cam_to_rgb[9])
{
    double tbl[210];
    mac_rec_t recs[20];
    const char *mode;
    char modebuf[64];
    double gain[3] = { 1.0, 1.0, 1.0 };
    double scale = 16383.0;
    size_t npix = (size_t)w * h, o;
    float *cam;

    if (src == NULL || npix == 0) return;
    /* 默认关闭（2026-10-26 6563 复盘）：构造 gate 区分不了「SPP 会跑」与
     * 「SPP 不会跑」的稿件——6563 的 Standard 旋钮与 2501 运行时抓包逐位
     * 相同（0/880），构造记录一样，但 6563 上 MAC-on 把色度推离相机 JPEG
     * （R/G +6%，no-MAC 已在 JPEG ±0.6% 内）；2501 上 MAC-on 也推远 SPP
     * 抓包输出。根因嫌疑 = MAC 输入域的 WB 混合与 SPP mac_in 不同（实测
     * 逐通道斜率 0.74/1.18/0.64 + 截距 ~800），同一组矩阵在不同色度混合
     * 上修正强度不同。等 Stage2 域对齐（§13.13 / Task#11）闭合后再默认开。
     * X3F_MAC=1 显式启用。 */
    {
        const char *en = getenv("X3F_MAC");
        if (en == NULL || en[0] == '\0' || en[0] == '0') return;
    }

    /* 模式名：环境变量优先，其次文件头记录（Quattro v4 头没有 -> Standard） */
    mode = getenv("X3F_MAC_MODE");
    if (mode == NULL || mode[0] == '\0') {
        if (x3f_render_probe_header_color_mode(path, modebuf, sizeof(modebuf)))
            mode = modebuf;
        else
            mode = "Standard";
    }

    if (!mac_gates_and_table(path, mode, tbl)) return;

    mac_build_recs(tbl, recs);

    if (getenv("X3F_MAC_SCALE") != NULL)
        scale = atof(getenv("X3F_MAC_SCALE"));
    if (!mac_as_shot_gain(path, gain))
        gain[0] = gain[1] = gain[2] = 1.0;
    {
        const char *eg = getenv("X3F_MAC_GAIN");
        if (eg != NULL && *eg != '\0')
            sscanf(eg, "%lf,%lf,%lf", &gain[0], &gain[1], &gain[2]);
    }

    /* 相机域码值缓冲：rgb_to_cam · src（WB 前）→ ×as-shot gain（WB 后）
     * → ×scale（16bit 码值域） */
    cam = (float *)malloc(npix * 3 * sizeof(float));
    if (cam == NULL) return;
    for (o = 0; o < npix; o++) {
        const float *p = src + o * 3;
        double c0 = rgb_to_cam[0] * p[0] + rgb_to_cam[1] * p[1] + rgb_to_cam[2] * p[2];
        double c1 = rgb_to_cam[3] * p[0] + rgb_to_cam[4] * p[1] + rgb_to_cam[5] * p[2];
        double c2 = rgb_to_cam[6] * p[0] + rgb_to_cam[7] * p[1] + rgb_to_cam[8] * p[2];
        cam[o * 3]     = (float)(c0 * gain[0] * scale);
        cam[o * 3 + 1] = (float)(c1 * gain[1] * scale);
        cam[o * 3 + 2] = (float)(c2 * gain[2] * scale);
    }

    /* X3F_MAC_CAMDUMP=<pre.bin>: dump apply 前码值域相机缓冲（定标用） */
    {
        const char *ed = getenv("X3F_MAC_CAMDUMP");
        if (ed != NULL && ed[0] != '\0') {
            FILE *f = fopen(ed, "wb");
            if (f) { fwrite(cam, sizeof(float), npix * 3, f); fclose(f); }
        }
    }

    mac_apply_cam(cam, npix, recs);

    {
        const char *ed = getenv("X3F_MAC_CAMDUMP");
        if (ed != NULL && ed[0] != '\0') {
            char postpath[512];
            FILE *f;
            snprintf(postpath, sizeof(postpath), "%s.post", ed);
            f = fopen(postpath, "wb");
            if (f) { fwrite(cam, sizeof(float), npix * 3, f); fclose(f); }
        }
    }

    /* 变回显示域：/scale → /gain → cam_to_rgb（不夹 0 —— MAC 是线性段，
     * 夹 0 是 CMCM/tone 那一层的事； MAC 输出也可以是负值）*/
    for (o = 0; o < npix; o++) {
        float *p = src + o * 3;
        double c0 = cam[o * 3]     / (scale * gain[0]);
        double c1 = cam[o * 3 + 1] / (scale * gain[1]);
        double c2 = cam[o * 3 + 2] / (scale * gain[2]);
        p[0] = (float)(cam_to_rgb[0] * c0 + cam_to_rgb[1] * c1 + cam_to_rgb[2] * c2);
        p[1] = (float)(cam_to_rgb[3] * c0 + cam_to_rgb[4] * c1 + cam_to_rgb[5] * c2);
        p[2] = (float)(cam_to_rgb[6] * c0 + cam_to_rgb[7] * c1 + cam_to_rgb[8] * c2);
    }
    free(cam);
    if (getenv("X3F_MAC_DEBUG"))
        fprintf(stderr, "[mac] %s: applied %ux%u\n", mode, w, h);
}
