// own_guided_up.wgsl — own 后端引导双边上采样 2x（guided_up_2x）的 GPU 版。
//
// 对应 own_decoder.c 的 guided_up_2x：半分辨率 delta 图 src 上采样到 2x，
// 值域引导 guide（全分辨率绝对色彩估计）、顶层对齐图 t2。25 tap（5×5），
// 距离核 dtab（[pxa][pya][j][i] 50 项，C 侧 guided_up_preinit 预计算）、
// 值域核 wv（C 侧 wv_lut，1/(k·|Δ|+1)，按 0.25 步长量化索引）。
//
// 逐位一致性：全部 f32 运算，且**循环序与 CPU 相同**（j 外 i 内，
// acc/wsum 逐 tap 累加）；dtab/wv 原样上传不在 GPU 重算（expf/除法
// 的 ulp 差进不来）。索引与反射式与 CPU 同式（两次 if，非循环）。
// 宿主 A/B：X3FDEC_GPU=0 对比 rgb dump 应逐位相同。

struct GUParams {
    hw: u32,
    hh: u32,
    wv_n: u32,
    p0: u32,
}

@group(0) @binding(0) var<storage, read> src: array<f32>;
@group(0) @binding(1) var<storage, read> t2: array<f32>;
@group(0) @binding(2) var<storage, read> guide: array<f32>;
@group(0) @binding(3) var<storage, read_write> dst: array<f32>;
@group(0) @binding(4) var<storage, read> dtab: array<f32>;   // 50 项
@group(0) @binding(5) var<storage, read> wv: array<f32>;     // WVLUT_N 项
@group(0) @binding(6) var<uniform> p: GUParams;

@compute @workgroup_size(16, 16)
fn guided_up(@builtin(global_invocation_id) gid: vec3<u32>) {
    let px = gid.x;
    let py = gid.y;
    let ew = p.hw * 2u;
    let eh = p.hh * 2u;
    if (px >= ew || py >= eh) { return; }

    let iy = py >> 1u;
    let pya = py & 1u;
    let ix = px >> 1u;
    let pxa = px & 1u;
    let g = guide[py * ew + px];
    let dtbase = (pxa * 2u + pya) * 25u;

    var acc = 0.0;
    var wsum = 0.0;
    for (var j = 0u; j < 5u; j++) {
        var sx = i32(ix) + i32(j) - 2;
        if (sx < 0) { sx = -sx; }
        if (sx >= i32(p.hw)) { sx = 2 * (i32(p.hw) - 1) - sx; }
        for (var i = 0u; i < 5u; i++) {
            var sy = i32(iy) + i32(i) - 2;
            if (sy < 0) { sy = -sy; }
            if (sy >= i32(p.hh)) { sy = 2 * (i32(p.hh) - 1) - sy; }
            let k = u32(sy) * p.hw + u32(sx);
            let s = src[k];
            let d = (s + t2[k]) - g;
            var ad = d;
            if (d < 0.0) { ad = -d; }
            var wi = i32(ad * 4.0 + 0.5);
            if (wi >= i32(p.wv_n)) { wi = i32(p.wv_n) - 1; }
            let w = dtab[dtbase + j * 5u + i] * wv[u32(wi)];
            acc = acc + s * w;
            wsum = wsum + w;
        }
    }
    dst[py * ew + px] = acc / wsum;
}
