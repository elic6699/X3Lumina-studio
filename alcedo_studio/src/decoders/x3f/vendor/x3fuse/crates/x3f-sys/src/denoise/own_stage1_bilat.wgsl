// own_stage1_bilat.wgsl — x3f_stage1.c s1_bilateral 的 GPU 版
// （own_gpu.m 的 s1bilat，MSL 168-222）。3 通道值域双边：σ 按中心像素的
// 噪声模型（二次式 + 0.4%·v+2 DN 地板）计算，空间项用 σs=r/2 的高斯查表。
// float 域 → 与 CPU 差 ±1 LSB 级（由调用方 X3F_STAGE1_* 控制）。失败/关闭
// （X3FDEC_GPU=0）落回 CPU 行并行版。
//
// 数据布局：
//   src : array<f32>   w*h*3（平面交错 RGB 三通道）
//   dst : array<f32>   w*h*3（逐像素写）
//   spt : array<f32>   (2r+1)^2 空间高斯表，索引 (dy+r)*(2r+1)+(dx+r)
//   coef: array<f32>   9（噪声系数，宿主已 double→float）
//   pu  : uniform {w, h, r, _}
//   pf  : uniform {lin_sat, level_scale, nscale, _}

struct S1U { w: u32, h: u32, r: u32, _p: u32 };
struct S1F { lin_sat: f32, level_scale: f32, nscale: f32, _p: f32 };

@group(0) @binding(0) var<storage, read> src: array<f32>;
@group(0) @binding(1) var<storage, read_write> dst: array<f32>;
@group(0) @binding(2) var<storage, read> spt: array<f32>;
@group(0) @binding(3) var<storage, read> coef: array<f32>;
@group(0) @binding(4) var<uniform> pu: S1U;
@group(0) @binding(5) var<uniform> pf: S1F;

@compute @workgroup_size(16, 16)
fn s1bilat(@builtin(global_invocation_id) gid: vec3<u32>) {
    let w = pu.w;
    let h = pu.h;
    let r = pu.r;
    let x = gid.x;
    let y = gid.y;
    if (x >= w || y >= h) { return; }

    let lin_sat = pf.lin_sat;
    let lvl = pf.level_scale;
    let ns = pf.nscale;
    let k = 16383.0 / select(lin_sat, 1.0, lin_sat <= 1e-6);

    let li = y * w + x;
    let pc = vec3<f32>(src[li * 3u], src[li * 3u + 1u], src[li * 3u + 2u]);

    // 每通道 σ：与 CPU s1_sigma 同式（含 0.4%·v+2 DN 地板与 g_s1_nscale 倍率）
    var sig = vec3<f32>(0.0);
    for (var c: u32 = 0u; c < 3u; c = c + 1u) {
        var v = pc[c] * k;
        if (v < 0.0) { v = 0.0; }
        var s2 = coef[c] + coef[3u + c] * v + coef[6u + c] * v * v;
        if (!(s2 > 1e-9)) { s2 = 1e-9; }
        let fl = 0.004 * v + 2.0;
        if (s2 < fl * fl) { s2 = fl * fl; }
        var s = ns * sqrt(s2) * lvl;
        if (s < 1e-9) { s = 1e-9; }
        sig[c] = s;
    }

    let d = 2u * r + 1u;
    var sum = vec3<f32>(0.0);
    var wsum = 0.0;
    for (var dy: i32 = -i32(r); dy <= i32(r); dy = dy + 1) {
        let py = i32(y) + dy;
        if (py < 0 || py >= i32(h)) { continue; }
        let srow = (dy + i32(r)) * i32(d);
        let rowbase = u32(py) * w * 3u;
        for (var dx: i32 = -i32(r); dx <= i32(r); dx = dx + 1) {
            let px = i32(x) + dx;
            if (px < 0 || px >= i32(w)) { continue; }
            let pqi = rowbase + u32(px) * 3u;
            var wgt = spt[u32(srow + (dx + i32(r)))];
            for (var c: u32 = 0u; c < 3u; c = c + 1u) {
                let dlt = (src[pqi + c] - pc[c]) * k;
                wgt = wgt * exp(-(dlt * dlt) / (2.0 * sig[c] * sig[c]));
            }
            if (wgt < 1e-12) { continue; }
            sum = sum + wgt * vec3<f32>(src[pqi], src[pqi + 1u], src[pqi + 2u]);
            wsum = wsum + wgt;
        }
    }

    let o = li * 3u;
    if (wsum > 1e-12) {
        dst[o] = sum.x / wsum;
        dst[o + 1u] = sum.y / wsum;
        dst[o + 2u] = sum.z / wsum;
    } else {
        dst[o] = pc.x;
        dst[o + 1u] = pc.y;
        dst[o + 2u] = pc.z;
    }
}
