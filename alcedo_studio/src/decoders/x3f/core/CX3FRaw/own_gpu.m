/* own_gpu.m — Metal GPU 加速层实现（见 own_gpu.h）
 *
 * 设计:
 *  - MSL 源码内嵌字符串，首次调用时运行时编译并缓存 pipeline state
 *  - 主机内存 <-> MTLBuffer 拷贝（Apple Silicon 统一内存，~50GB/s，可忽略）
 *  - 失败任一环节即 own_gpu_available() 永久返回 0，调用方回退 CPU
 *  - 编译: clang -fobjc-arc -c own_gpu.m（Metal/QuartzCore framework 已在链接清单）
 */
#import <Metal/Metal.h>
#include <simd/simd.h>   /* simd_int2 等主机侧 simd 向量（chrmed dims buffer 用） */
#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include "own_gpu.h"

/* ------------------------------------------------------------------ */
/* MSL kernels                                                          */
/* ------------------------------------------------------------------ */

static const char *MSL_SRC =
"#include <metal_stdlib>\n"
"using namespace metal;\n"
"\n"
"/* 引导双边 2x 上采样。dtab = [2][2][5][5] 距离核, 索引 ((pxa*2+pya)*5+j)*5+i */\n"
"kernel void guided_up(const device float *src   [[buffer(0)]],\n"
"                      const device float *t2    [[buffer(1)]],\n"
"                      const device float *guide [[buffer(2)]],\n"
"                      device float *dst         [[buffer(3)]],\n"
"                      constant uint4 &dims      [[buffer(4)]],   /* ew,eh,hw,hh */\n"
"                      constant float &kk        [[buffer(5)]],\n"
"                      const device float *dtab  [[buffer(6)]],\n"
"                      uint2 g [[thread_position_in_grid]])\n"
"{\n"
"    uint ew = dims.x, eh = dims.y, hw = dims.z, hh = dims.w;\n"
"    if (g.x >= ew || g.y >= eh) return;\n"
"    int ix = g.x >> 1, pxa = g.x & 1;\n"
"    int iy = g.y >> 1, pya = g.y & 1;\n"
"    float gv = guide[(size_t)g.y * ew + g.x];\n"
"    float acc = 0.0f, wsum = 0.0f;\n"
"    for (int j = 0; j < 5; j++) {\n"
"        int sx = (int)ix + j - 2;\n"
"        sx = sx < 0 ? -sx : sx;\n"
"        if (sx >= (int)hw) sx = 2 * ((int)hw - 1) - sx;\n"
"        for (int i = 0; i < 5; i++) {\n"
"            int sy = (int)iy + i - 2;\n"
"            sy = sy < 0 ? -sy : sy;\n"
"            if (sy >= (int)hh) sy = 2 * ((int)hh - 1) - sy;\n"
"            size_t s = (size_t)sy * hw + sx;\n"
"            float d = (src[s] + t2[s]) - gv;\n"
"            float ad = fabs(d);\n"
"            float wv = 1.0f / (kk * ad + 1.0f);\n"
"            float w = dtab[((pxa * 2 + pya) * 5 + j) * 5 + i] * wv;\n"
"            acc += src[s] * w;\n"
"            wsum += w;\n"
"        }\n"
"    }\n"
"    dst[(size_t)g.y * ew + g.x] = acc / wsum;\n"
"}\n"
"\n"
"/* 5x5 双边滤波水平 pass（值域核 exp(d^2*inv2r2), 空间核 sw[5]） */\n"
"kernel void bilat_h(const device float *img [[buffer(0)]],\n"
"                    device float *out       [[buffer(1)]],\n"
"                    constant uint2 &dims    [[buffer(2)]],   /* w,h */\n"
"                    constant float &inv2r2  [[buffer(3)]],\n"
"                    constant float *sw      [[buffer(4)]],\n"
"                    uint2 g [[thread_position_in_grid]])\n"
"{\n"
"    uint w = dims.x, h = dims.y;\n"
"    if (g.x >= w || g.y >= h) return;\n"
"    int x = (int)g.x, y = (int)g.y;\n"
"    float c = img[(size_t)y * w + x];\n"
"    float acc = 0.0f, wsum = 0.0f;\n"
"    for (int k = -2; k <= 2; k++) {\n"
"        int xx = x + k;\n"
"        if (xx < 0) xx = 0; else if (xx >= (int)w) xx = (int)w - 1;\n"
"        float v = img[(size_t)y * w + xx];\n"
"        float d = v - c;\n"
"        float wgt = sw[k + 2] * exp(d * d * inv2r2);\n"
"        acc += v * wgt;\n"
"        wsum += wgt;\n"
"    }\n"
"    out[(size_t)y * w + x] = acc / wsum;\n"
"}\n"
"\n"
"/* 垂直 pass（同上） */\n"
"kernel void bilat_v(const device float *img [[buffer(0)]],\n"
"                    device float *out       [[buffer(1)]],\n"
"                    constant uint2 &dims    [[buffer(2)]],\n"
"                    constant float &inv2r2  [[buffer(3)]],\n"
"                    constant float *sw      [[buffer(4)]],\n"
"                    uint2 g [[thread_position_in_grid]])\n"
"{\n"
"    uint w = dims.x, h = dims.y;\n"
"    if (g.x >= w || g.y >= h) return;\n"
"    int x = (int)g.x, y = (int)g.y;\n"
"    float c = img[(size_t)y * w + x];\n"
"    float acc = 0.0f, wsum = 0.0f;\n"
"    for (int k = -2; k <= 2; k++) {\n"
"        int yy = y + k;\n"
"        if (yy < 0) yy = 0; else if (yy >= (int)h) yy = (int)h - 1;\n"
"        float v = img[(size_t)yy * w + x];\n"
"        float d = v - c;\n"
"        float wgt = sw[k + 2] * exp(d * d * inv2r2);\n"
"        acc += v * wgt;\n"
"        wsum += wgt;\n"
"    }\n"
"    out[(size_t)y * w + x] = acc / wsum;\n"
"}\n"
"\n"
"/* Cr/Cb 3x3 中值一次 pass（A -> B, ping-pong 由调用方编排） */\n"
"kernel void chroma_med(const device float *crA [[buffer(0)]],\n"
"                       const device float *cbA [[buffer(1)]],\n"
"                       device float *crB       [[buffer(2)]],\n"
"                       device float *cbB       [[buffer(3)]],\n"
"                       constant uint2 &dims    [[buffer(4)]],   /* w,h */\n"
"                       uint2 g [[thread_position_in_grid]])\n"
"{\n"
"    uint w = dims.x, h = dims.y;\n"
"    if (g.x >= w || g.y >= h) return;\n"
"    int x = (int)g.x, y = (int)g.y;\n"
"    size_t li = (size_t)y * w + x;\n"
"    float rs[9], bs[9];\n"
"    int m = 0;\n"
"    for (int dy = -1; dy <= 1; dy++) {\n"
"        int yy = y + dy; if (yy < 0) yy = 0; if (yy >= (int)h) yy = (int)h - 1;\n"
"        for (int dx = -1; dx <= 1; dx++) {\n"
"            int xx = x + dx; if (xx < 0) xx = 0; if (xx >= (int)w) xx = (int)w - 1;\n"
"            size_t k = (size_t)yy * w + xx;\n"
"            rs[m] = crA[k]; bs[m] = cbA[k]; m++;\n"
"        }\n"
"    }\n"
"    for (int a = 1; a < 9; a++) {\n"
"        float kv = rs[a]; int b = a - 1;\n"
"        while (b >= 0 && rs[b] > kv) { rs[b + 1] = rs[b]; b--; }\n"
"        rs[b + 1] = kv;\n"
"        kv = bs[a]; b = a - 1;\n"
"        while (b >= 0 && bs[b] > kv) { bs[b + 1] = bs[b]; b--; }\n"
"        bs[b + 1] = kv;\n"
"    }\n"
"    crB[li] = rs[4];\n"
"    cbB[li] = bs[4];\n"
"}\n"
"\n"
"/* 渲染层显示域色度中值的单通道窗口中值（x3f_render.c chrmed_row 提速）。\n"
" * 输入是 CPU 预计算的色度平面（U=B−Y / V=R−Y，逐位一致），本核只做\n"
" * 窗口收集 + 快速选择。选中值 = 同一多重集合的第 n/2 小值，与算法无关，\n"
" * 所以与 CPU chrmed_select 结果值精确一致（重建由 CPU 用 double y0 完成）。\n"
" * 半径 ≤ CHRMED_MAX_R(7) → 窗口 ≤ 15×15 = 225 元素。\n"
" * 2026-10-23：改成 nlm3 同款 threadgroup 瓦片 —— 之前的 float a[225] 是\n"
" * 动态索引局部数组，Metal 落到 device 堆，quickselect 全程打慢内存，\n"
" * 2712 口径一通道要几百 ms；瓦片协作加载后选择全在片上。 */\n"
"/* float → 保序 uint（IEEE754 负数位型反转技巧）；keyf 为其逆 */\n"
"inline uint fkey(float v) {\n"
"    uint u = as_type<uint>(v);\n"
"    return (u & 0x80000000u) ? ~u : (u | 0x80000000u);\n"
"}\n"
"inline float keyf(uint k) {\n"
"    uint u = (k & 0x80000000u) ? (k & 0x7FFFFFFFu) : ~k;\n"
"    return as_type<float>(u);\n"
"}\n"
"\n"
"/* Stage1 主双边（x3f_stage1.c s1_bl_row 的 GPU 版，三级金字塔共用）。\n"
" * CPU 基准全程 double，这里 float —— 值域 exp 是硬件指令，\n"
" * 输出与 CPU 差 ±1 LSB 级（与 guided/bilat/chroma_nr 同口径）。\n"
" * σ_c 只依赖中心像素（每像素预计算 3 个）；空间项由 host 预计算成表。\n"
" * 半径 r ≤ 6 → 窗口 ≤ 13×13。 */\n"
"kernel void s1bilat(const device float *src  [[buffer(0)]],\n"
"                    device float *dst        [[buffer(1)]],\n"
"                    const device float *spt  [[buffer(2)]],   /* (2r+1)^2 空间表 */\n"
"                    const device float *coef [[buffer(3)]],   /* 9 个噪声系数 */\n"
"                    constant int4 &pu        [[buffer(4)]],   /* w, h, r, - */\n"
"                    constant float4 &pf      [[buffer(5)]],   /* lin_sat, level_scale, nscale, - */\n"
"                    uint2 g [[thread_position_in_grid]])\n"
"{\n"
"    int x = (int)g.x, y = (int)g.y;\n"
"    int w = pu.x, h = pu.y, r = pu.z;\n"
"    if (x >= w || y >= h) return;\n"
"    float lin_sat = pf.x, lvl = pf.y, ns = pf.z;\n"
"    /* s1_v 的斜率：DN 当量 = lin * 16383 / lin_sat */\n"
"    float k = 16383.0f / (lin_sat > 1e-6f ? lin_sat : 1.0f);\n"
"    size_t li = (size_t)y * (size_t)w + (size_t)x;\n"
"    const device float *pc = src + li * 3;\n"
"    /* σ_c：s1_sigma(coef,c,s1_v(pc[c]))*level_scale 的 float 重算 */\n"
"    float sig[3];\n"
"    for (int c = 0; c < 3; c++) {\n"
"        float v = pc[c] * k; if (v < 0.0f) v = 0.0f;\n"
"        float s2 = coef[c] + coef[3+c]*v + coef[6+c]*v*v;\n"
"        if (!(s2 > 1e-9f)) s2 = 1e-9f;\n"
"        float fl = 0.004f*v + 2.0f;\n"
"        if (s2 < fl*fl) s2 = fl*fl;\n"
"        float s = ns * sqrt(s2) * lvl;\n"
"        sig[c] = s > 1e-9f ? s : 1e-9f;\n"
"    }\n"
"    int D = 2*r + 1;\n"
"    float sum[3] = {0.0f, 0.0f, 0.0f}, wsum = 0.0f;\n"
"    for (int dy = -r; dy <= r; dy++) {\n"
"        int py = y + dy;\n"
"        if (py < 0 || py >= h) continue;\n"
"        const device float *srow = spt + (size_t)(dy + r) * (size_t)D;\n"
"        const device float *row = src + (size_t)py * (size_t)w * 3;\n"
"        for (int dx = -r; dx <= r; dx++) {\n"
"            int px = x + dx;\n"
"            if (px < 0 || px >= w) continue;\n"
"            const device float *pq = row + (size_t)px * 3;\n"
"            float wgt = srow[dx + r];\n"
"            for (int c = 0; c < 3; c++) {\n"
"                float d = (pq[c] - pc[c]) * k;\n"
"                wgt *= exp(-(d*d) / (2.0f*sig[c]*sig[c]));\n"
"            }\n"
"            if (wgt < 1e-12f) continue;\n"
"            sum[0] += wgt*pq[0]; sum[1] += wgt*pq[1]; sum[2] += wgt*pq[2];\n"
"            wsum += wgt;\n"
"        }\n"
"    }\n"
"    device float *o = dst + li * 3;\n"
"    if (wsum > 1e-12f) {\n"
"        o[0] = sum[0]/wsum; o[1] = sum[1]/wsum; o[2] = sum[2]/wsum;\n"
"    } else {\n"
"        o[0] = pc[0]; o[1] = pc[1]; o[2] = pc[2];\n"
"    }\n"
"}\n"
"\n"
"/* 快速非局部均值（x3fuse FastNlMeansDenoisingInvoker 语义, u16 3ch, L1）。\n"
" * 16x16 线程组协作加载 (16+2*ext)^2 瓦片到 threadgroup memory，\n"
" * patch 读取全部命中片上内存；权重 LUT 在 device（L2 缓存）。\n"
" * 每线程: 121 个搜索偏移 × 3x3 patch 的 3ch 联合 SAD → 定点量化权重\n"
" * → U/V 加权平均（Y 输出回写原值，调用方只用 U/V）。\n"
" * 整型运算与 CPU 版逐位一致。 */\n"
"inline int nlm_refl(int i, int n) {\n"
"    if (n == 1) return 0;\n"
"    int period = 2 * (n - 1);\n"
"    int r = i % period; if (r < 0) r += period;\n"
"    if (r >= n) r = period - r;\n"
"    return r;\n"
"}\n"
"kernel void nlm3(const device ushort *src [[buffer(0)]],\n"
"                 device ushort *dst       [[buffer(1)]],\n"
"                 const device int *lut    [[buffer(2)]],\n"
"                 constant uint4 &pu       [[buffer(3)]],  /* w, h, sr, bin_shift */\n"
"                 constant uint2 &pv       [[buffer(4)]],  /* lut_n, tr */\n"
"                 uint2 g   [[thread_position_in_grid]],\n"
"                 uint2 lid [[thread_position_in_threadgroup]],\n"
"                 uint2 tid [[threadgroup_position_in_grid]])\n"
"{\n"
"    threadgroup ushort tile[40][40][3];   /* 最大 16+2*12=40（search 21） */\n"
"    threadgroup int sry[40], srx[40];\n"
"    int w = (int)pu.x, h = (int)pu.y;\n"
"    int sr = (int)pu.z, bin_shift = (int)pu.w;\n"
"    int lut_n = (int)pv.x, tr = (int)pv.y;\n"
"    int sw = 2 * sr + 1, tw = 2 * tr + 1;\n"
"    int ext = sr + tr;\n"
"    int TS = 16 + 2 * ext;\n"
"    int bx = (int)tid.x * 16, by = (int)tid.y * 16;\n"
"    int li = (int)(lid.y * 16 + lid.x);\n"
"    for (int k = li; k < TS; k += 256) {\n"
"        sry[k] = nlm_refl(by + k - ext, h);\n"
"        srx[k] = nlm_refl(bx + k - ext, w);\n"
"    }\n"
"    threadgroup_barrier(mem_flags::mem_threadgroup);\n"
"    for (int k = li; k < TS * TS; k += 256) {\n"
"        int tyy = k / TS, txx = k - tyy * TS;\n"
"        size_t s = ((size_t)sry[tyy] * w + srx[txx]) * 3;\n"
"        tile[tyy][txx][0] = src[s];\n"
"        tile[tyy][txx][1] = src[s + 1];\n"
"        tile[tyy][txx][2] = src[s + 2];\n"
"    }\n"
"    threadgroup_barrier(mem_flags::mem_threadgroup);\n"
"    int x = bx + (int)lid.x, y = by + (int)lid.y;\n"
"    if (x >= w || y >= h) return;\n"
"    int cy = (int)lid.y + ext, cx = (int)lid.x + ext;\n"
"    long est1 = 0, est2 = 0, wsum1 = 0, wsum2 = 0;\n"
"    for (int dy = 0; dy < sw; dy++) {\n"
"        int qy = cy + dy - sr;\n"
"        for (int dx = 0; dx < sw; dx++) {\n"
"            int qx = cx + dx - sr;\n"
"            int sad = 0;\n"
"            for (int ty = 0; ty < tw; ty++) {\n"
"                int pry = cy + ty - tr, qry = qy + ty - tr;\n"
"                for (int tx = 0; tx < tw; tx++) {\n"
"                    int prx = cx + tx - tr, qxx = qx + tx - tr;\n"
"                    sad += abs((int)tile[pry][prx][0] - (int)tile[qry][qxx][0])\n"
"                         + abs((int)tile[pry][prx][1] - (int)tile[qry][qxx][1])\n"
"                         + abs((int)tile[pry][prx][2] - (int)tile[qry][qxx][2]);\n"
"                }\n"
"            }\n"
"            int almost = sad >> bin_shift;\n"
"            if (almost >= lut_n) almost = lut_n - 1;\n"
"            const device int *wgt = lut + (size_t)almost * 3;\n"
"            est1 += (long)wgt[1] * tile[qy][qx][1];\n"
"            est2 += (long)wgt[2] * tile[qy][qx][2];\n"
"            wsum1 += wgt[1]; wsum2 += wgt[2];\n"
"        }\n"
"    }\n"
"    size_t o = ((size_t)y * w + x) * 3;\n"
"    dst[o] = src[o];   /* Y 通道不降噪（h=0），调用方丢弃 */\n"
"    dst[o + 1] = (wsum1 > 0) ? (ushort)clamp((int)((est1 + wsum1 / 2) / wsum1), 0, 65535) : (ushort)0;\n"
"    dst[o + 2] = (wsum2 > 0) ? (ushort)clamp((int)((est2 + wsum2 / 2) / wsum2), 0, 65535) : (ushort)0;\n"
"}\n"
"\n"
"/* ── 预览 apply 链（x3f_render.c apply_ex_common 提速，2026-10-23）──── */\n"
"inline float sstepf(float a, float b, float x) {\n"
"    if (b <= a) return x >= b ? 1.0f : 0.0f;\n"
"    float t = (x - a) / (b - a);\n"
"    t = clamp(t, 0.0f, 1.0f);\n"
"    return t * t * (3.0f - 2.0f * t);\n"
"}\n"
"inline float cm65_bilerp(const device float *t, int ix, int iy, int jx, int jy,\n"
"                         float tx, float ty) {\n"
"    float v00 = t[iy * 65 + ix], v10 = t[iy * 65 + jx];\n"
"    float v01 = t[jy * 65 + ix], v11 = t[jy * 65 + jx];\n"
"    return (v00 + (v10 - v00) * tx) * (1.0f - ty)\n"
"         + (v01 + (v11 - v01) * tx) * ty;\n"
"}\n"
"/* LUT 管线：apply_row 的常规支（无 fill/mono/hl_y_only/warm/color_fit）。\n"
" * 顺序与 CPU 逐行对齐：增益 → 色彩模式 3×3（负夹 0）→ 逐通道 sqrt 轴 LUT\n"
" * （LIN_MAX=8，SIZE=4096）→ 601 饱和度 → RAW 过曝驱动去饱和\n"
" * → 相机色彩匹配（65×65 Cb/Cr 双线性，亮度不动）。float 精度。 */\n"
"kernel void apply_lut(const device float *lin  [[buffer(0)]],\n"
"                      const device float *clip [[buffer(1)]],\n"
"                      const device ushort *lut [[buffer(2)]],\n"
"                      const device ushort *lutg [[buffer(3)]],\n"
"                      const device ushort *lutb [[buffer(4)]],\n"
"                      device ushort *out       [[buffer(5)]],\n"
"                      constant int4 &pu   [[buffer(6)]],  /* w,h,use_gain,use_sat */\n"
"                      constant int4 &pv   [[buffer(7)]],  /* has_cm3,has_cm65,has_clip,- */\n"
"                      constant float4 &pf [[buffer(8)]],  /* sat,hl_desat,lo,hi */\n"
"                      const device float *gain [[buffer(9)]],\n"
"                      const device float *cm3  [[buffer(10)]],\n"
"                      const device float *cmcb [[buffer(11)]],\n"
"                      const device float *cmcr [[buffer(12)]],\n"
"                      uint2 g [[thread_position_in_grid]])\n"
"{\n"
"    int w = pu.x, h = pu.y;\n"
"    int x = (int)g.x, y = (int)g.y;\n"
"    if (x >= w || y >= h) return;\n"
"    size_t i = (size_t)y * (size_t)w + (size_t)x;\n"
"    const device float *src = lin + i * 3;\n"
"    float lv[3];\n"
"    for (int c = 0; c < 3; c++) {\n"
"        float v = src[c];\n"
"        if (pu.z) v *= gain[c];\n"
"        lv[c] = v;\n"
"    }\n"
"    if (pv.x) {\n"
"        float a = lv[0], b = lv[1], e = lv[2];\n"
"        lv[0] = a * cm3[0] + b * cm3[1] + e * cm3[2];\n"
"        lv[1] = a * cm3[3] + b * cm3[4] + e * cm3[5];\n"
"        lv[2] = a * cm3[6] + b * cm3[7] + e * cm3[8];\n"
"        for (int c = 0; c < 3; c++) if (lv[c] < 0.0f) lv[c] = 0.0f;\n"
"    }\n"
"    float d[3];\n"
"    for (int c = 0; c < 3; c++) {\n"
"        const device ushort *tab = (c == 0) ? lut : ((c == 1) ? lutg : lutb);\n"
"        float v = lv[c] <= 0.0f ? 0.0f : lv[c];\n"
"        float t = sqrt(v / 8.0f);\n"
"        float idx = (t >= 1.0f) ? 4096.0f : t * 4096.0f;\n"
"        int i0 = (int)idx;\n"
"        float f = idx - (float)i0;\n"
"        float a, b2;\n"
"        if (i0 >= 4096) { a = (float)tab[4096]; b2 = a; }\n"
"        else { a = (float)tab[i0]; b2 = (float)tab[i0 + 1]; }\n"
"        d[c] = (a + (b2 - a) * f) / 65535.0f;\n"
"    }\n"
"    if (pu.w) {\n"
"        float u = clamp(pf.x, 0.0f, 2.0f);\n"
"        float y = 0.299f * d[0] + 0.587f * d[1] + 0.114f * d[2];\n"
"        for (int c = 0; c < 3; c++) d[c] = y + (d[c] - y) * u;\n"
"    }\n"
"    if (pf.y > 0.0f) {\n"
"        float wgt;\n"
"        if (pv.z) {\n"
"            float cs = clip[i];\n"
"            wgt = sstepf(pf.z, pf.w, cs) * pf.y;\n"
"        } else {\n"
"            float m = min(min(d[0], d[1]), d[2]);\n"
"            wgt = sstepf(0.80f, 1.0f, m) * pf.y;\n"
"        }\n"
"        if (wgt > 0.001f) {\n"
"            float dr = max(d[0], 0.0f), dg = max(d[1], 0.0f), db = max(d[2], 0.0f);\n"
"            float y = 0.2126f * dr + 0.7152f * dg + 0.0722f * db;\n"
"            if (pv.z) {\n"
"                float cs = clip[i];\n"
"                float t2 = sstepf(1.6f, 2.4f, cs);\n"
"                if (t2 > 0.0f) {\n"
"                    float mx = max(max(d[0], d[1]), d[2]);\n"
"                    if (mx > y) y += (mx - y) * t2;\n"
"                }\n"
"            }\n"
"            for (int c = 0; c < 3; c++) d[c] += (y - d[c]) * wgt;\n"
"        }\n"
"    }\n"
"    if (pv.y) {\n"
"        float yy = 0.299f * d[0] + 0.587f * d[1] + 0.114f * d[2];\n"
"        float cb = d[2] - yy, cr = d[0] - yy;\n"
"        float fx = (cb + 0.6f) * 64.0f / 1.2f;\n"
"        float fy = (cr + 0.6f) * 64.0f / 1.2f;\n"
"        if (fx >= 0.0f && fy >= 0.0f && fx <= 64.0f && fy <= 64.0f) {\n"
"            int ix = (int)fx, iy = (int)fy;\n"
"            int jx = (ix + 1 < 65) ? ix + 1 : ix;\n"
"            int jy = (iy + 1 < 65) ? iy + 1 : iy;\n"
"            float tx = fx - (float)ix, ty = fy - (float)iy;\n"
"            float ncb = cm65_bilerp(cmcb, ix, iy, jx, jy, tx, ty);\n"
"            float ncr = cm65_bilerp(cmcr, ix, iy, jx, jy, tx, ty);\n"
"            float rr = yy + ncr, bb = yy + ncb;\n"
"            float gg = (yy - 0.299f * rr - 0.114f * bb) / 0.587f;\n"
"            d[0] = rr; d[1] = gg; d[2] = bb;\n"
"        }\n"
"    }\n"
"    size_t o = i * 3;\n"
"    for (int c = 0; c < 3; c++) {\n"
"        float v = clamp(d[c], 0.0f, 1.0f);\n"
"        out[o + c] = (ushort)(v * 65535.0f + 0.5f);\n"
"    }\n"
"}\n"
"\n"
"/* 色度中值 prep：u16 显示域 RGB → U/V 平面（U=B−Y / V=R−Y，709 亮度）。\n"
" * CPU 用 double 算 yq，这里 float —— 中值取的是样本值，差 ±1e-7 量级。 */\n"
"kernel void chrmed_prep(const device ushort *img [[buffer(0)]],\n"
"                        device float *upl [[buffer(1)]],\n"
"                        device float *vpl [[buffer(2)]],\n"
"                        constant int2 &dims [[buffer(3)]],   /* w,h */\n"
"                        uint2 g [[thread_position_in_grid]])\n"
"{\n"
"    int w = dims.x, h = dims.y;\n"
"    int x = (int)g.x, y = (int)g.y;\n"
"    if (x >= w || y >= h) return;\n"
"    size_t i = (size_t)y * (size_t)w + (size_t)x, o = i * 3;\n"
"    float r = (float)img[o] / 65535.0f;\n"
"    float gg = (float)img[o + 1] / 65535.0f;\n"
"    float b = (float)img[o + 2] / 65535.0f;\n"
"    float yq = 0.2126f * r + 0.7152f * gg + 0.0722f * b;\n"
"    upl[i] = b - yq;\n"
"    vpl[i] = r - yq;\n"
"}\n"
"\n"
"/* 色度中值 emit：src(u16) + omu/omv → 重建 RGB 写回（可原地，逐像素读写）。 */\n"
"kernel void chrmed_emit(const device ushort *img [[buffer(0)]],\n"
"                        const device float *omu [[buffer(1)]],\n"
"                        const device float *omv [[buffer(2)]],\n"
"                        device ushort *out [[buffer(3)]],\n"
"                        constant int2 &dims [[buffer(4)]],   /* w,h */\n"
"                        uint2 g [[thread_position_in_grid]])\n"
"{\n"
"    int w = dims.x, h = dims.y;\n"
"    int x = (int)g.x, y = (int)g.y;\n"
"    if (x >= w || y >= h) return;\n"
"    size_t i = (size_t)y * (size_t)w + (size_t)x, o = i * 3;\n"
"    float r = (float)img[o] / 65535.0f;\n"
"    float gg = (float)img[o + 1] / 65535.0f;\n"
"    float b = (float)img[o + 2] / 65535.0f;\n"
"    float y0 = 0.2126f * r + 0.7152f * gg + 0.0722f * b;\n"
"    float cb = omu[i], cr = omv[i];\n"
"    float v;\n"
"    v = clamp(y0 + cr, 0.0f, 1.0f);\n"
"    out[o] = (ushort)(v * 65535.0f + 0.5f);\n"
"    v = clamp(y0 - (0.2126f * cr + 0.0722f * cb) / 0.7152f, 0.0f, 1.0f);\n"
"    out[o + 1] = (ushort)(v * 65535.0f + 0.5f);\n"
"    v = clamp(y0 + cb, 0.0f, 1.0f);\n"
"    out[o + 2] = (ushort)(v * 65535.0f + 0.5f);\n"
"}\n"
"\n"
"/* 16→8 降位（apply_ex_common down8 循环的 GPU 版，含 Bayer4 抖动）。 */\n"
"kernel void down8k(const device ushort *tmp [[buffer(0)]],\n"
"                   device uchar *out [[buffer(1)]],\n"
"                   constant int4 &p [[buffer(2)]],  /* w,h,stride,dither */\n"
"                   uint2 g [[thread_position_in_grid]])\n"
"{\n"
"    const int bayerT[16] = {0,8,2,10,12,4,14,6,3,11,1,9,15,7,13,5};\n"
"    int w = p.x, h = p.y, stride = p.z, dth_on = p.w;\n"
"    int x = (int)g.x, y = (int)g.y;\n"
"    if (x >= w || y >= h) return;\n"
"    size_t o = ((size_t)y * (size_t)w + (size_t)x) * 3;\n"
"    int dth = dth_on ? (bayerT[((y & 3) << 2) | (x & 3)] - 8) * 16 : 0;\n"
"    size_t dofs = (size_t)y * (size_t)stride + (size_t)x * 3;\n"
"    for (int c = 0; c < 3; c++) {\n"
"        unsigned v = ((unsigned)tmp[o + c] + 128u + (unsigned)dth) >> 8;\n"
"        out[dofs + c] = (uchar)(v > 255u ? 255u : v);\n"
"    }\n"
"}\n";

/* ------------------------------------------------------------------ */
/* chrmed_sel 半径特化变体（追加到 MSL_SRC 之后一起编译）                  */
/* ------------------------------------------------------------------ */

/* 通用版（r 走 buffer）两个硬伤：窗口循环趟数运行期才知道 → 无法展开，
 * 且每趟循环都重复算索引。按 r=1..7 生成编译期特化：循环常数趟全展开，
 * tile 尺寸编译期确定。
 * 选择算法（第四版，2026-10-23）：键域二分基数选择（与 CPU chrmed_select
 * 值精确一致：选 fkey 序下第 k2+1 小，fkey 是 float 全序映射，±0 例外
 * 见下）。历史教训：
 *  ① quickselect 落 float a[225] 数组 → 动态索引被 Metal 放进 device 堆，
 *     全程打慢内存（改 threadgroup 瓦片后修掉）；
 *  ② 32-bit 键域二分本身保证 ≤32 趟、精确 —— 慢在每趟都重新扫 tile
 *     （31 趟 × 49 次 TG 读 ≈ 每平面 90ms，占 85%+）；
 *  ③ 数据枢轴快速选择 + 值域中点 —— 中位数在 ±0 附近时值域中点二分
 *     要穿整段非规格化区（~150 趟），96 上限打穿直接答错，废弃。
 * 第四版：r≤4 时把整个窗口的 fkey 一次性预取进**寄存器数组**（常数趟
 * 展开后索引全是编译期常量 → 不落内存），二分全程寄存器比较；r≥5
 * 窗口 >121 个 key 会撑爆寄存器，保底走 tile 直读。fkey 全序与 CPU
 * 浮点比较的唯一分歧是 −0.0/+0.0（fkey 视为不同、CPU 视为相等），
 * 两者参与后续重建运算结果相同，可接受。 */
#define SELK_HEAD(R) \
"kernel void chrmed_sel" #R "(const device float *pl [[buffer(0)]],\n" \
"                       device float *omed     [[buffer(1)]],\n" \
"                       constant int &w        [[buffer(2)]],\n" \
"                       constant int &h        [[buffer(3)]],\n" \
"                       uint2 g   [[thread_position_in_grid]],\n" \
"                       uint2 lid [[thread_position_in_threadgroup]],\n" \
"                       uint2 tid [[threadgroup_position_in_grid]])\n" \
"{\n" \
"    const int r = " #R ";\n" \
"    threadgroup float tile[16 + 2 * " #R "][16 + 2 * " #R "];\n" \
"    int TS = 16 + 2 * r;\n" \
"    int ext = r;\n" \
"    int bx = (int)tid.x * 16, by = (int)tid.y * 16;\n" \
"    int li = (int)(lid.y * 16 + lid.x);\n" \
"    for (int k = li; k < TS * TS; k += 256) {\n" \
"        int tyy = k / TS, txx = k - tyy * TS;\n" \
"        int sy = by + tyy - ext; if (sy < 0) sy = 0; if (sy >= h) sy = h - 1;\n" \
"        int sx = bx + txx - ext; if (sx < 0) sx = 0; if (sx >= w) sx = w - 1;\n" \
"        tile[tyy][txx] = pl[(size_t)sy * (size_t)w + (size_t)sx];\n" \
"    }\n" \
"    threadgroup_barrier(mem_flags::mem_threadgroup);\n" \
"    int x = bx + (int)lid.x, y = by + (int)lid.y;\n" \
"    if (x >= w || y >= h) return;\n" \
"    int ly = (int)lid.y + ext, lx = (int)lid.x + ext;\n" \
"    int y0 = y - r; if (y0 < 0) y0 = 0;\n" \
"    int y1 = y + r; if (y1 >= h) y1 = h - 1;\n" \
"    int x0 = x - r; if (x0 < 0) x0 = 0;\n" \
"    int x1 = x + r; if (x1 >= w) x1 = w - 1;\n" \
"    /* 窗口按 CPU chrmed_row 同款夹边：k2 = 夹边后窗口数的一半\n" \
"     * （曾固定数 (2r+1)² 把 clamp 重复行列算进去 → 边界全错，勿改回） */\n" \
"    uint k2 = (uint)((y1 - y0 + 1) * (x1 - x0 + 1)) >> 1;\n" \
"    float wmin = tile[ly + y0 - y][lx + x0 - x];\n" \
"    float wmax = wmin;\n" \
"    for (int dy = -r; dy <= r; dy++) {\n" \
"        if (y + dy < 0 || y + dy >= h) continue;\n" \
"        for (int dx = -r; dx <= r; dx++) {\n" \
"            if (x + dx < 0 || x + dx >= w) continue;\n" \
"            float v = tile[ly + dy][lx + dx];\n" \
"            wmin = min(wmin, v);\n" \
"            wmax = max(wmax, v);\n" \
"        }\n" \
"    }\n" \
"    uint lo = fkey(wmin), hi = fkey(wmax);\n" \
"    if (lo < hi) {\n" \

/* r≤4：窗口 key 预取进寄存器数组（常数趟展开后索引全为编译期常量
 * → 不落内存），二分全程寄存器比较。越界槽填，
 * 比任何合法 mid 都大，永不计数。 */
#define SELK_KS_BODY(R) \
"        uint KS[(2 * " #R " + 1) * (2 * " #R " + 1)];\n" \
"        int ki = 0;\n" \
"        for (int dy = -r; dy <= r; dy++) {\n" \
"            bool vy = (y + dy >= 0 && y + dy < h);\n" \
"            for (int dx = -r; dx <= r; dx++) {\n" \
"                bool ok = vy && (x + dx >= 0 && x + dx < w);\n" \
"                KS[ki++] = ok ? fkey(tile[ly + dy][lx + dx]) : 0xFFFFFFFFu;\n" \
"            }\n" \
"        }\n" \
"        while (lo < hi) {\n" \
"            uint mid = lo + (hi - lo) / 2u;\n" \
"            uint cnt = 0;\n" \
"            for (int i = 0; i < (2 * " #R " + 1) * (2 * " #R " + 1); i++)\n" \
"                cnt += (KS[i] <= mid) ? 1u : 0u;\n" \
"            if (cnt >= k2 + 1u) hi = mid; else lo = mid + 1u;\n" \
"        }\n" \

#define SELK_TAIL \
"    }\n" \
"    omed[(size_t)y * (size_t)w + (size_t)x] = keyf(lo);\n" \
"}\n"

#define SELK_SMALL(R) SELK_HEAD(R) SELK_KS_BODY(R) SELK_TAIL

/* r≥5 曾有 tile 直读变体，实测 2712 平面 0.8~3.9s（编译器在这条路径上
 * 完全没优化），只在高 ISO 出现 —— 直接回 CPU 更快，不再生成。 */
static const char *MSL_SRC_EXTRA =
    SELK_SMALL(1) SELK_SMALL(2) SELK_SMALL(3) SELK_SMALL(4);

/* ------------------------------------------------------------------ */
/* 初始化与状态                                                          */
/* ------------------------------------------------------------------ */

static id<MTLDevice>         g_dev;
static id<MTLCommandQueue>   g_queue;
static id<MTLLibrary>        g_lib;    /* 供 csel_ps 惰性建 r 特化 pipeline */
static id<MTLComputePipelineState> g_ps_guided, g_ps_bh, g_ps_bv, g_ps_med, g_ps_nlm, g_ps_s1b;
static id<MTLComputePipelineState> g_ps_alut, g_ps_cprep, g_ps_cemit, g_ps_d8;
/* chrmed_sel 的 r=1..7 特化 pipeline，按需创建（调用方单线程） */
static id<MTLComputePipelineState> g_ps_cselv[8];
static int g_state = -1;   /* -1 未初始化, 0 不可用, 1 可用 */

static id<MTLComputePipelineState> csel_ps(int r)
{
    if (r < 1 || r > 4) return nil;   /* r≥5 无变体 → -1 回 CPU */
    if (g_ps_cselv[r] != nil) return g_ps_cselv[r];
    @autoreleasepool {
        NSString *nm = [NSString stringWithFormat:@"chrmed_sel%d", r];
        g_ps_cselv[r] = [g_dev newComputePipelineStateWithFunction:
                         [g_lib newFunctionWithName:nm] error:nil];
    }
    return g_ps_cselv[r];
}

static int gpu_enabled(void)
{
    const char *e = getenv("X3FDEC_GPU");
    return !(e && e[0] == '0');
}

static id<MTLBuffer> mkbuf(const void *bytes, size_t nfloats)
{
    if (bytes)
        return [g_dev newBufferWithBytes:bytes length:nfloats * sizeof(float)
                                 options:MTLResourceStorageModeShared];
    return [g_dev newBufferWithLength:nfloats * sizeof(float)
                              options:MTLResourceStorageModeShared];
}

static void init_once_impl(void)
{
    @autoreleasepool {
        g_state = 0;
        g_dev = MTLCreateSystemDefaultDevice();
        if (!g_dev) return;
        g_queue = [g_dev newCommandQueue];
        if (!g_queue) return;
        /* MSL_SRC_EXTRA 里是 chrmed_sel1..7 半径特化变体（宏展开），一起编译。
         * 拼接用 stringWithUTF8String —— stringWithFormat:@"%s" 实测整段失效
         * （连 ASCII 都输出空串），别用。 */
        NSMutableString *src = [NSMutableString stringWithUTF8String:MSL_SRC];
        [src appendString:[NSString stringWithUTF8String:MSL_SRC_EXTRA]];
        NSError *err = nil;
        id<MTLLibrary> lib = [g_dev newLibraryWithSource:src options:nil error:&err];
        if (!lib) {
            fprintf(stderr, "own_gpu: MSL compile failed: %s\n",
                    err.localizedDescription.UTF8String);
            return;
        }
        g_lib = lib;   /* csel_ps 惰性建 r 特化 pipeline 用 */
        g_ps_guided = [g_dev newComputePipelineStateWithFunction:
                       [lib newFunctionWithName:@"guided_up"] error:&err];
        g_ps_bh = [g_dev newComputePipelineStateWithFunction:
                   [lib newFunctionWithName:@"bilat_h"] error:&err];
        g_ps_bv = [g_dev newComputePipelineStateWithFunction:
                   [lib newFunctionWithName:@"bilat_v"] error:&err];
        g_ps_med = [g_dev newComputePipelineStateWithFunction:
                    [lib newFunctionWithName:@"chroma_med"] error:&err];
        g_ps_nlm = [g_dev newComputePipelineStateWithFunction:
                    [lib newFunctionWithName:@"nlm3"] error:&err];
        g_ps_s1b = [g_dev newComputePipelineStateWithFunction:
                    [lib newFunctionWithName:@"s1bilat"] error:&err];
        g_ps_alut  = [g_dev newComputePipelineStateWithFunction:
                      [lib newFunctionWithName:@"apply_lut"] error:&err];
        g_ps_cprep = [g_dev newComputePipelineStateWithFunction:
                      [lib newFunctionWithName:@"chrmed_prep"] error:&err];
        g_ps_cemit = [g_dev newComputePipelineStateWithFunction:
                      [lib newFunctionWithName:@"chrmed_emit"] error:&err];
        g_ps_d8    = [g_dev newComputePipelineStateWithFunction:
                      [lib newFunctionWithName:@"down8k"] error:&err];
        if (!g_ps_guided || !g_ps_bh || !g_ps_bv || !g_ps_med || !g_ps_nlm ||
            !g_ps_s1b || !g_ps_alut || !g_ps_cprep ||
            !g_ps_cemit || !g_ps_d8) return;
        g_state = 1;
    }
}

static int ensure_init(void)
{
    /* 双线程（B/G 通道）可能同时首调, pthread_once 防竞态 */
    if (!gpu_enabled()) { g_state = 0; return 0; }
    static pthread_once_t once = PTHREAD_ONCE_INIT;
    pthread_once(&once, init_once_impl);
    return g_state;
}

/* 线程局部的 GPU 关闭（SFD 七帧解码线程用，见 own_gpu.h 的说明）。
 * Mac 侧同样提供：Metal 的 device/queue 本身线程安全，但 SFD 的 N 槽预取
 * 流水线让多帧同时派发大缓冲，独显/集显上的带宽争抢与 win 同族，
 * X3F_SFD_GPU=1 可逐张开回来。 */
static _Thread_local int g_gpu_thread_off = 0;

void own_gpu_thread_disable(int on)
{
    g_gpu_thread_off = on ? 1 : 0;
}

int own_gpu_available(void)
{
    if (g_gpu_thread_off) return 0;
    return ensure_init();
}

/* 单 kernel 派发并等待完成 */
static int run_kernel(id<MTLComputePipelineState> ps,
                      __unsafe_unretained id<MTLBuffer> *bufs, int nbuf,
                      MTLSize grid, MTLSize tpg)
{
    @autoreleasepool {
        id<MTLCommandBuffer> cb = [g_queue commandBuffer];
        if (!cb) return -1;
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        if (!enc) return -1;
        [enc setComputePipelineState:ps];
        for (int i = 0; i < nbuf; i++) [enc setBuffer:bufs[i] offset:0 atIndex:i];
        [enc dispatchThreads:grid threadsPerThreadgroup:tpg];
        [enc endEncoding];
        [cb commit];
        [cb waitUntilCompleted];
        if (cb.status == MTLCommandBufferStatusError) return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* 导出接口                                                              */
/* ------------------------------------------------------------------ */

/* 签名对齐 own_gpu.h / own_decoder.c（GPU 化后 caller 额外传 wv_lut）。
 * Mac 的 guided kernel 还是 exp 旧路径，wv 尚未接入 Metal——先维持原 exp
 * 行为（与已发布版本一致）。TODO(GPU): kernel 换 wv LUT 采样后再上传。 */
int own_gpu_guided_up_2x(const float *src, const float *t2, const float *guide,
                         float *dst, int hw, int hh, float k, float fd,
                         const float *dtab, const float *wv, int wv_n)
{
    (void)wv; (void)wv_n;
    if (!own_gpu_available()) return -1;
    @autoreleasepool {
        int ew = hw * 2, eh = hh * 2;
        /* CPU 版 dtab 以 fd*1000 为失效键, 此处直接使用调用方传来的已初始化表 */
        (void)fd;
        uint32_t dims[4] = { (uint32_t)ew, (uint32_t)eh, (uint32_t)hw, (uint32_t)hh };
        id<MTLBuffer> bsrc = mkbuf(src, (size_t)hw * hh);
        id<MTLBuffer> bt2  = mkbuf(t2,  (size_t)hw * hh);
        id<MTLBuffer> bgd  = mkbuf(guide, (size_t)ew * eh);
        id<MTLBuffer> bdst = mkbuf(NULL,  (size_t)ew * eh);
        id<MTLBuffer> bdims = [g_dev newBufferWithBytes:&dims length:sizeof(dims)
                                                options:MTLResourceStorageModeShared];
        id<MTLBuffer> bk = [g_dev newBufferWithBytes:&k length:sizeof(float)
                                             options:MTLResourceStorageModeShared];
        id<MTLBuffer> bdt = mkbuf(dtab, 100);
        if (!bsrc || !bt2 || !bgd || !bdst || !bdims || !bk || !bdt) return -1;
        __unsafe_unretained id<MTLBuffer> bufs[7] = { bsrc, bt2, bgd, bdst, bdims, bk, bdt };
        MTLSize grid = MTLSizeMake(ew, eh, 1);
        MTLSize tpg = MTLSizeMake(16, 16, 1);
        if (run_kernel(g_ps_guided, bufs, 7, grid, tpg) != 0) return -1;
        memcpy(dst, bdst.contents, (size_t)ew * eh * sizeof(float));
    }
    return 0;
}

/* 签名对齐 own_gpu.h / own_decoder.c（GPU 化后 caller 传 C 侧预计算的
 * rlut/sw）。Mac 这边的 bilat_h/bilat_v kernel 还是 exp(d²·inv2r2) 旧路径，
 * rlut 尚未接入 Metal——先维持原 exp 行为（与已发布版本一致）：
 * inv2r2 仍由 sigma_r 现算，sw 直接吃 caller 传的（同式 expf 产物，与旧
 * 本地计算逐位一致）。TODO(GPU): kernel 换 LUT 采样后再真正上传 rlut。 */
int own_gpu_bilateral5(float *img, int w, int h, float sigma_r,
                       const float *rlut, int rlut_n, const float *sw)
{
    (void)rlut; (void)rlut_n;
    if (!own_gpu_available()) return -1;
    @autoreleasepool {
        size_t n = (size_t)w * h;
        float inv2r2 = -0.5f / (sigma_r * sigma_r);
        uint32_t dims[2] = { (uint32_t)w, (uint32_t)h };
        id<MTLBuffer> bA = mkbuf(img, n);
        id<MTLBuffer> bB = mkbuf(NULL, n);
        id<MTLBuffer> bdims = [g_dev newBufferWithBytes:&dims length:sizeof(dims)
                                                options:MTLResourceStorageModeShared];
        id<MTLBuffer> binv = [g_dev newBufferWithBytes:&inv2r2 length:sizeof(float)
                                               options:MTLResourceStorageModeShared];
        id<MTLBuffer> bsw = mkbuf(sw, 5);
        if (!bA || !bB || !bdims || !binv || !bsw) return -1;
        MTLSize grid = MTLSizeMake(w, h, 1);
        MTLSize tpg = MTLSizeMake(16, 16, 1);
        __unsafe_unretained id<MTLBuffer> hb[5] = { bA, bB, bdims, binv, bsw };
        __unsafe_unretained id<MTLBuffer> vb[5] = { bB, bA, bdims, binv, bsw };
        if (run_kernel(g_ps_bh, hb, 5, grid, tpg) != 0) return -1;
        if (run_kernel(g_ps_bv, vb, 5, grid, tpg) != 0) return -1;
        memcpy(img, bA.contents, n * sizeof(float));
    }
    return 0;
}

int own_gpu_chroma_nr(float *crA, float *cbA, float *crB, float *cbB,
                      int w, int h, int iters)
{
    if (!own_gpu_available() || iters < 1) return -1;
    @autoreleasepool {
        size_t n = (size_t)w * h;
        uint32_t dims[2] = { (uint32_t)w, (uint32_t)h };
        id<MTLBuffer> bcrA = mkbuf(crA, n);
        id<MTLBuffer> bcbA = mkbuf(cbA, n);
        id<MTLBuffer> bcrB = mkbuf(NULL, n);
        id<MTLBuffer> bcbB = mkbuf(NULL, n);
        id<MTLBuffer> bdims = [g_dev newBufferWithBytes:&dims length:sizeof(dims)
                                                options:MTLResourceStorageModeShared];
        if (!bcrA || !bcbA || !bcrB || !bcbB || !bdims) return -1;
        MTLSize grid = MTLSizeMake(w, h, 1);
        MTLSize tpg = MTLSizeMake(16, 16, 1);
        id<MTLComputePipelineState> ps = g_ps_med;
        @autoreleasepool {
            id<MTLCommandBuffer> cb = [g_queue commandBuffer];
            if (!cb) return -1;
            id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
            if (!enc) return -1;
            [enc setComputePipelineState:ps];
            for (int it = 0; it < iters; it++) {
                __unsafe_unretained id<MTLBuffer> b[5] = { bcrA, bcbA, bcrB, bcbB, bdims };
                for (int i = 0; i < 5; i++) [enc setBuffer:b[i] offset:0 atIndex:i];
                [enc dispatchThreads:grid threadsPerThreadgroup:tpg];
                /* ping-pong */
                id<MTLBuffer> t;
                t = bcrA; bcrA = bcrB; bcrB = t;
                t = bcbA; bcbA = bcbB; bcbB = t;
            }
            [enc endEncoding];
            [cb commit];
            [cb waitUntilCompleted];
            if (cb.status == MTLCommandBufferStatusError) return -1;
        }
        /* 每次迭代后 A/B 互换, 循环结束时最新结果恒在 bcrA/bcbA 名下 */
        memcpy(crA, bcrA.contents, n * sizeof(float));
        memcpy(cbA, bcbA.contents, n * sizeof(float));
    }
    return 0;
}

/* NLM: src3/dst3 交错 3ch u16 (w*h*3)。lut = int32[lut_n*3]（调用方按
 * own_decoder.c nlm3_den 同式构造）。twin/swin 奇数窗宽。
 * 与 CPU 版逐位一致（整型运算 + 同一 LUT）。 */
int own_gpu_nlm3(const uint16_t *src3, uint16_t *dst3, int w, int h,
                 const int32_t *lut, int lut_n, int twin, int swin,
                 int bin_shift)
{
    if (!own_gpu_available()) return -1;
    @autoreleasepool {
        size_t npix = (size_t)w * h;
        uint32_t pu[4] = { (uint32_t)w, (uint32_t)h,
                           (uint32_t)(swin / 2), (uint32_t)bin_shift };
        uint32_t pv[2] = { (uint32_t)lut_n, (uint32_t)(twin / 2) };
        id<MTLBuffer> bsrc = [g_dev newBufferWithBytes:src3
                                                length:npix * 3 * sizeof(uint16_t)
                                               options:MTLResourceStorageModeShared];
        id<MTLBuffer> bdst = [g_dev newBufferWithLength:npix * 3 * sizeof(uint16_t)
                                                options:MTLResourceStorageModeShared];
        id<MTLBuffer> blut = [g_dev newBufferWithBytes:lut
                                                length:(size_t)lut_n * 3 * sizeof(int32_t)
                                               options:MTLResourceStorageModeShared];
        id<MTLBuffer> bpu = [g_dev newBufferWithBytes:pu length:sizeof(pu)
                                              options:MTLResourceStorageModeShared];
        id<MTLBuffer> bpv = [g_dev newBufferWithBytes:pv length:sizeof(pv)
                                              options:MTLResourceStorageModeShared];
        if (!bsrc || !bdst || !blut || !bpu || !bpv) return -1;
        __unsafe_unretained id<MTLBuffer> bufs[5] = { bsrc, bdst, blut, bpu, bpv };
        /* 必须用 dispatchThreadgroups 补满整组：nlm3 内有 threadgroup_barrier
         * 和协作 tile 加载，残缺线程组（dispatchThreads 在 w/h 非 16 倍数时）
         * 会导致 tile 未完全加载 → 边缘结果错误（实测）。越界线程由
         * kernel 内 x>=w||y>=h 裁剪（在 barrier 之后）。 */
        @autoreleasepool {
            id<MTLCommandBuffer> cb = [g_queue commandBuffer];
            if (!cb) return -1;
            id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
            if (!enc) return -1;
            [enc setComputePipelineState:g_ps_nlm];
            for (int i = 0; i < 5; i++) [enc setBuffer:bufs[i] offset:0 atIndex:i];
            MTLSize groups = MTLSizeMake((w + 15) / 16, (h + 15) / 16, 1);
            MTLSize tpg = MTLSizeMake(16, 16, 1);
            [enc dispatchThreadgroups:groups threadsPerThreadgroup:tpg];
            [enc endEncoding];
            [cb commit];
            [cb waitUntilCompleted];
            if (cb.status == MTLCommandBufferStatusError) return -1;
        }
        memcpy(dst3, bdst.contents, npix * 3 * sizeof(uint16_t));
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* 渲染层显示域色度中值（x3f_render.c apply_chroma_median 提速）          */
/* ------------------------------------------------------------------ */

/* 单通道窗口中值平面：pl 输入色度平面（CPU 预计算，逐位一致），
 * omed 输出每像素 (2r+1)^2 窗口中值。与 CPU chrmed_select 值精确一致。 */
int own_gpu_chrmed_plane(const float *pl, float *omed, int w, int h, int r)
{
    if (!own_gpu_available()) return -1;
    id<MTLComputePipelineState> ps = csel_ps(r);   /* r∉1..7 → -1 回 CPU */
    if (ps == nil) return -1;
    @autoreleasepool {
        size_t n = (size_t)w * h;
        id<MTLBuffer> bpl  = mkbuf(pl, n);
        id<MTLBuffer> bomd = mkbuf(NULL, n);
        /* r 已编译进 kernel 变体，只需 w/h 两个小 buffer */
        id<MTLBuffer> bw = [g_dev newBufferWithBytes:&w length:sizeof(int)
                                             options:MTLResourceStorageModeShared];
        id<MTLBuffer> bh = [g_dev newBufferWithBytes:&h length:sizeof(int)
                                             options:MTLResourceStorageModeShared];
        if (!bpl || !bomd || !bw || !bh) return -1;
        __unsafe_unretained id<MTLBuffer> bufs[4] = { bpl, bomd, bw, bh };
        MTLSize grid = MTLSizeMake(w, h, 1);
        MTLSize tpg  = MTLSizeMake(16, 16, 1);
        /* chrmed_sel 有 threadgroup_barrier + 协作瓦片加载：必须整组派发 */
        @autoreleasepool {
            id<MTLCommandBuffer> cb = [g_queue commandBuffer];
            if (!cb) return -1;
            id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
            if (!enc) return -1;
            [enc setComputePipelineState:ps];
            for (int i = 0; i < 4; i++) [enc setBuffer:bufs[i] offset:0 atIndex:i];
            MTLSize groups = MTLSizeMake((w + 15) / 16, (h + 15) / 16, 1);
            [enc dispatchThreadgroups:groups threadsPerThreadgroup:tpg];
            [enc endEncoding];
            [cb commit];
            [cb waitUntilCompleted];
            if (cb.status == MTLCommandBufferStatusError) return -1;
        }
        memcpy(omed, bomd.contents, n * sizeof(float));
    }
    return 0;
}

/* 分带版（2026-11-08）：只算带内 [ry0,ry1) 行的中值，GPU 缓冲只占一带来回。
 *
 * 100MP 的整幅版一次要 bpl+bomd 两块 402MB 的 GPU 缓冲，再加上 CPU 侧
 * upl/vpl/omu/omv 四块整幅平面，峰值顶到 5.4GB。分带后每带几十 MB。
 *
 * 与整幅版逐位相同，理由见 own_gpu.h 的注释：kernel 的 clamp 以传入的 bh
 * 为界，中间带的窗口在带内完整，不会触发 clamp。 */
int own_gpu_chrmed_plane_rows(const float *pl, float *omed,
                              int w, int bh, int ry0, int ry1, int r)
{
    if (!own_gpu_available()) return -1;
    id<MTLComputePipelineState> ps = csel_ps(r);
    if (ps == nil) return -1;
    if (pl == NULL || omed == NULL || w <= 0 || bh <= 0) return -1;
    if (ry0 < 0 || ry1 > bh || ry1 <= ry0) return -1;
    @autoreleasepool {
        size_t n = (size_t)w * bh;
        id<MTLBuffer> bpl  = mkbuf(pl, n);
        id<MTLBuffer> bomd = mkbuf(NULL, n);
        int ww = w, hh = bh;
        id<MTLBuffer> bw = [g_dev newBufferWithBytes:&ww length:sizeof(int)
                                             options:MTLResourceStorageModeShared];
        id<MTLBuffer> bhh = [g_dev newBufferWithBytes:&hh length:sizeof(int)
                                              options:MTLResourceStorageModeShared];
        if (!bpl || !bomd || !bw || !bhh) return -1;
        __unsafe_unretained id<MTLBuffer> bufs[4] = { bpl, bomd, bw, bhh };
        MTLSize tpg  = MTLSizeMake(16, 16, 1);
        MTLSize groups = MTLSizeMake((w + 15) / 16, (bh + 15) / 16, 1);
        @autoreleasepool {
            id<MTLCommandBuffer> cb = [g_queue commandBuffer];
            if (!cb) return -1;
            id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
            if (!enc) return -1;
            [enc setComputePipelineState:ps];
            for (int i = 0; i < 4; i++) [enc setBuffer:bufs[i] offset:0 atIndex:i];
            [enc dispatchThreadgroups:groups threadsPerThreadgroup:tpg];
            [enc endEncoding];
            [cb commit];
            [cb waitUntilCompleted];
            if (cb.status == MTLCommandBufferStatusError) return -1;
        }
        memcpy(omed, (const float *)bomd.contents + (size_t)ry0 * w,
               (size_t)(ry1 - ry0) * w * sizeof(float));
    }
    return 0;
}

/* Stage1 主双边（x3f_stage1.c s1_bilateral 的 GPU 提速，三级金字塔共用）。
 * 与 CPU 的差异：float 域 + GPU 硬件 exp → 输出 ±1 LSB 级（非逐位）；
 * 失败（含 X3FDEC_GPU=0）返回 -1，调用方回落 CPU 行并行版。
 * r ≤ 6（SPP 三级半径表 {6,4,2}），nscale = g_s1_nscale 由调用方传入。 */
int own_gpu_stage1_bilat(const float *src, float *dst, int w, int h, int r,
                         const double coef[9], double lin_sat,
                         double level_scale, double nscale)
{
    if (!own_gpu_available()) return -1;
    if (r < 1 || r > 6) return -1;
    @autoreleasepool {
        size_t n = (size_t)w * h;
        int D = 2 * r + 1;
        float spt[169], cf[9];
        int pu[4] = { w, h, r, 0 };
        float pf[4] = { (float)lin_sat, (float)level_scale, (float)nscale, 0.0f };
        int dy, dx, i;

        /* 空间项表：与 CPU 相同表达式（exp(-d²/(2σs²))，σs = r/2） */
        for (dy = -r; dy <= r; dy++)
            for (dx = -r; dx <= r; dx++) {
                double d2 = (double)dx * dx + (double)dy * dy;
                spt[(size_t)(dy + r) * D + (dx + r)] =
                    (float)exp(-d2 / (2.0 * (r / 2.0) * (r / 2.0)));
            }
        for (i = 0; i < 9; i++) cf[i] = (float)coef[i];

        id<MTLBuffer> bsrc = mkbuf(src, n * 3);
        id<MTLBuffer> bdst = mkbuf(NULL, n * 3);
        id<MTLBuffer> bspt = mkbuf(spt, (size_t)D * D);
        id<MTLBuffer> bcf  = mkbuf(cf, 9);
        id<MTLBuffer> bpu  = [g_dev newBufferWithBytes:pu length:sizeof(pu)
                                               options:MTLResourceStorageModeShared];
        id<MTLBuffer> bpf  = [g_dev newBufferWithBytes:pf length:sizeof(pf)
                                               options:MTLResourceStorageModeShared];
        if (!bsrc || !bdst || !bspt || !bcf || !bpu || !bpf) return -1;
        __unsafe_unretained id<MTLBuffer> bufs[6] = { bsrc, bdst, bspt, bcf, bpu, bpf };
        MTLSize grid = MTLSizeMake(w, h, 1);
        MTLSize tpg  = MTLSizeMake(16, 16, 1);
        if (run_kernel(g_ps_s1b, bufs, 6, grid, tpg) != 0) return -1;
        memcpy(dst, bdst.contents, n * 3 * sizeof(float));
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* 预览 apply 链（x3f_render.c apply_ex_common 提速，见 own_gpu.h）       */
/* ------------------------------------------------------------------ */

static id<MTLBuffer> mkbuf_bytes(const void *bytes, size_t nbytes)
{
    if (bytes)
        return [g_dev newBufferWithBytes:bytes length:nbytes
                                 options:MTLResourceStorageModeShared];
    return [g_dev newBufferWithLength:1 options:MTLResourceStorageModeShared];
}

/* 预览 apply 每帧都要 ~110MB 的临时 buffer（u16 图 + U/V/中值平面），
 * 每次新分配在 2712 口径实测要几百 ms —— 按容量缓住，不够再扩。
 * 超过阈值的（1:1 原生 5424 口径 ~400MB）不缓存：那条路是后台一次性，
 * 常驻几百 MB 统一内存不值。调用方（x3f_render apply）已串行；
 * 互斥锁只护缓存槽本身。 */
#define CBUF_CACHE_MAX (48u * 1024u * 1024u)
static pthread_mutex_t g_cbuf_lock = PTHREAD_MUTEX_INITIALIZER;
static id<MTLBuffer> g_cbuf_u16;  static size_t g_cbuf_u16_b;
static id<MTLBuffer> g_cbuf_f[4]; static size_t g_cbuf_f_b[4];

static id<MTLBuffer> cbuf_u16(size_t bytes)
{
    id<MTLBuffer> b = nil;
    if (bytes > CBUF_CACHE_MAX)
        return [g_dev newBufferWithLength:bytes
                                  options:MTLResourceStorageModeShared];
    pthread_mutex_lock(&g_cbuf_lock);
    if (g_cbuf_u16 == nil || g_cbuf_u16_b < bytes) {
        g_cbuf_u16 = [g_dev newBufferWithLength:bytes
                                        options:MTLResourceStorageModeShared];
        g_cbuf_u16_b = bytes;
    }
    b = g_cbuf_u16;
    pthread_mutex_unlock(&g_cbuf_lock);
    return b;
}
static id<MTLBuffer> cbuf_f(int slot, size_t bytes)
{
    id<MTLBuffer> b = nil;
    if (bytes > CBUF_CACHE_MAX)
        return [g_dev newBufferWithLength:bytes
                                  options:MTLResourceStorageModeShared];
    pthread_mutex_lock(&g_cbuf_lock);
    if (g_cbuf_f[slot] == nil || g_cbuf_f_b[slot] < bytes) {
        g_cbuf_f[slot] = [g_dev newBufferWithLength:bytes
                                            options:MTLResourceStorageModeShared];
        g_cbuf_f_b[slot] = bytes;
    }
    b = g_cbuf_f[slot];
    pthread_mutex_unlock(&g_cbuf_lock);
    return b;
}

int own_gpu_apply_lut16(const float *lin, const float *clip,
                        const uint16_t *lut, const uint16_t *lutg,
                        const uint16_t *lutb,
                        const float *gain, const float *cm3,
                        const float *cmcb, const float *cmcr,
                        float sat, int use_sat, float hl_desat,
                        float desat_lo, float desat_hi,
                        uint16_t *out, int w, int h)
{
    if (!own_gpu_available()) return -1;
    @autoreleasepool {
        size_t np = (size_t)w * h;
        /* kernel 内硬编码 SIZE=4096 / LIN_MAX=8 / CM_N=65 / CM_R=0.6，
         * x3f_render.c 调用点有同数编译期断言把关。 */
        int pu[4] = { w, h, (gain != NULL) ? 1 : 0, use_sat ? 1 : 0 };
        int pv[4] = { (cm3 != NULL) ? 1 : 0,
                      (cmcb != NULL && cmcr != NULL) ? 1 : 0,
                      (clip != NULL) ? 1 : 0, 0 };
        float pf[4] = { sat, hl_desat, desat_lo, desat_hi };
        float gainz[3] = { 1.0f, 1.0f, 1.0f };
        float cm3z[9]  = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        float z = 0.0f;
        id<MTLBuffer> blin  = [g_dev newBufferWithBytes:lin
                                                length:np * 3 * sizeof(float)
                                               options:MTLResourceStorageModeShared];
        id<MTLBuffer> bclip = mkbuf_bytes(clip, np * sizeof(float));
        id<MTLBuffer> blut  = mkbuf_bytes(lut,  4097 * sizeof(uint16_t));
        id<MTLBuffer> blutg = mkbuf_bytes(lutg ? lutg : lut, 4097 * sizeof(uint16_t));
        id<MTLBuffer> blutb = mkbuf_bytes(lutb ? lutb : lut, 4097 * sizeof(uint16_t));
        id<MTLBuffer> bout  = [g_dev newBufferWithLength:np * 3 * sizeof(uint16_t)
                                                 options:MTLResourceStorageModeShared];
        id<MTLBuffer> bpu  = [g_dev newBufferWithBytes:pu length:sizeof(pu)
                                               options:MTLResourceStorageModeShared];
        id<MTLBuffer> bpv  = [g_dev newBufferWithBytes:pv length:sizeof(pv)
                                               options:MTLResourceStorageModeShared];
        id<MTLBuffer> bpf  = [g_dev newBufferWithBytes:pf length:sizeof(pf)
                                               options:MTLResourceStorageModeShared];
        id<MTLBuffer> bgain = mkbuf_bytes(gain ? gain : gainz, 3 * sizeof(float));
        id<MTLBuffer> bcm3  = mkbuf_bytes(cm3 ? cm3 : cm3z, 9 * sizeof(float));
        /* 65×65 色彩匹配表：**长度必须是整张表**（4225 floats）——
         * 曾经写成 sizeof(float)，GPU 越界读到全 0，输出退化成纯 601 亮度。 */
        id<MTLBuffer> bcmcb = mkbuf_bytes(cmcb ? cmcb : &z,
                                          (cmcb ? 65 * 65 : 1) * sizeof(float));
        id<MTLBuffer> bcmcr = mkbuf_bytes(cmcr ? cmcr : &z,
                                          (cmcr ? 65 * 65 : 1) * sizeof(float));
        if (!blin || !bclip || !blut || !blutg || !blutb || !bout || !bpu ||
            !bpv || !bpf || !bgain || !bcm3 || !bcmcb || !bcmcr) return -1;
        __unsafe_unretained id<MTLBuffer> bufs[13] = {
            blin, bclip, blut, blutg, blutb, bout, bpu, bpv, bpf,
            bgain, bcm3, bcmcb, bcmcr
        };
        MTLSize grid = MTLSizeMake(w, h, 1);
        MTLSize tpg  = MTLSizeMake(16, 16, 1);
        if (run_kernel(g_ps_alut, bufs, 13, grid, tpg) != 0) return -1;
        memcpy(out, bout.contents, np * 3 * sizeof(uint16_t));
    }
    return 0;
}

int own_gpu_chrmed_u16(uint16_t *buf, int w, int h, int stride, int r)
{
    if (!own_gpu_available() || r < 1 || r > 4) return -1;   /* r≥5 回 CPU */
    @autoreleasepool {
        size_t np = (size_t)w * h;
        (void)stride;   /* 原地重建按 w 行进；stride 只在 CPU 侧按行寻址 */
        id<MTLBuffer> bimg = cbuf_u16(np * 3 * sizeof(uint16_t));
        id<MTLBuffer> bupl = cbuf_f(0, np * sizeof(float));
        id<MTLBuffer> bvpl = cbuf_f(1, np * sizeof(float));
        id<MTLBuffer> bomu = cbuf_f(2, np * sizeof(float));
        id<MTLBuffer> bomv = cbuf_f(3, np * sizeof(float));
        if (bimg == nil || bupl == nil || bvpl == nil || bomu == nil || bomv == nil)
            return -1;
        memcpy(bimg.contents, buf, np * 3 * sizeof(uint16_t));
        /* MSL 侧 constant int2 &：主机侧用 simd_int2（ObjC 下 simd 短名 int2
         * 仅 C++ 可用），布局同 2×int32，字节一一对应 */
        const simd_int2 chrmed_dims = { w, h };
        id<MTLBuffer> bdims = [g_dev newBufferWithBytes:&chrmed_dims
                                                length:sizeof(chrmed_dims)
                                               options:MTLResourceStorageModeShared];
        id<MTLBuffer> bw = [g_dev newBufferWithBytes:&w length:sizeof(int)
                                             options:MTLResourceStorageModeShared];
        id<MTLBuffer> bh = [g_dev newBufferWithBytes:&h length:sizeof(int)
                                             options:MTLResourceStorageModeShared];
        if (!bimg || !bupl || !bvpl || !bomu || !bomv || !bdims || !bw || !bh)
            return -1;
        @autoreleasepool {
            id<MTLCommandBuffer> cb = [g_queue commandBuffer];
            if (!cb) return -1;
            id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
            if (!enc) return -1;
            MTLSize tpg  = MTLSizeMake(16, 16, 1);
            MTLSize groups = MTLSizeMake((w + 15) / 16, (h + 15) / 16, 1);
            /* ① U/V 平面 ②③ 两通道窗口中值 ④ 重建（原地写回 bimg）。
             * sel 有 barrier + 瓦片协作加载 → 整组派发；prep/emit 无 barrier，
             * 但同组整派发也无害（越界线程在 kernel 内裁掉）。
             * sel 用 r 特化变体（r 已编译进 kernel，不绑 r buffer）。 */
            id<MTLComputePipelineState> ps_sel = csel_ps(r);
            if (ps_sel == nil) return -1;
            __unsafe_unretained id<MTLBuffer> prep[4]  = { bimg, bupl, bvpl, bdims };
            __unsafe_unretained id<MTLBuffer> selu[4]  = { bupl, bomu, bw, bh };
            __unsafe_unretained id<MTLBuffer> selv[4]  = { bvpl, bomv, bw, bh };
            __unsafe_unretained id<MTLBuffer> emit[5]  = { bimg, bomu, bomv, bimg, bdims };
            [enc setComputePipelineState:g_ps_cprep];
            for (int i = 0; i < 4; i++) [enc setBuffer:prep[i] offset:0 atIndex:i];
            [enc dispatchThreadgroups:groups threadsPerThreadgroup:tpg];
            [enc setComputePipelineState:ps_sel];
            for (int i = 0; i < 4; i++) [enc setBuffer:selu[i] offset:0 atIndex:i];
            [enc dispatchThreadgroups:groups threadsPerThreadgroup:tpg];
            for (int i = 0; i < 4; i++) [enc setBuffer:selv[i] offset:0 atIndex:i];
            [enc dispatchThreadgroups:groups threadsPerThreadgroup:tpg];
            [enc setComputePipelineState:g_ps_cemit];
            for (int i = 0; i < 5; i++) [enc setBuffer:emit[i] offset:0 atIndex:i];
            [enc dispatchThreadgroups:groups threadsPerThreadgroup:tpg];
            [enc endEncoding];
            [cb commit];
            [cb waitUntilCompleted];
            if (cb.status == MTLCommandBufferStatusError) return -1;
        }
        memcpy(buf, bimg.contents, np * 3 * sizeof(uint16_t));
    }
    return 0;
}

int own_gpu_down8(const uint16_t *tmp, uint8_t *out, int w, int h,
                  int stride, int dither)
{
    if (!own_gpu_available()) return -1;
    @autoreleasepool {
        size_t np = (size_t)w * h;
        int p4[4] = { w, h, stride, dither ? 1 : 0 };
        id<MTLBuffer> btmp = [g_dev newBufferWithBytes:tmp
                                                length:np * 3 * sizeof(uint16_t)
                                               options:MTLResourceStorageModeShared];
        id<MTLBuffer> bout = [g_dev newBufferWithLength:(size_t)stride * (size_t)h
                                                options:MTLResourceStorageModeShared];
        id<MTLBuffer> bp   = [g_dev newBufferWithBytes:p4 length:sizeof(p4)
                                               options:MTLResourceStorageModeShared];
        if (!btmp || !bout || !bp) return -1;
        __unsafe_unretained id<MTLBuffer> bufs[3] = { btmp, bout, bp };
        MTLSize grid = MTLSizeMake(w, h, 1);
        MTLSize tpg  = MTLSizeMake(16, 16, 1);
        if (run_kernel(g_ps_d8, bufs, 3, grid, tpg) != 0) return -1;
        memcpy(out, bout.contents, (size_t)stride * (size_t)h);
    }
    return 0;
}
