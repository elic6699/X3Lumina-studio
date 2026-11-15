// own_chrmed_u16.wgsl — x3f_render.c apply_chroma_median 显示域色度中值的 GPU 版
// （own_gpu.m 的 chrmed_prep / chrmed_emit，对应 MSL 419-461）。
//
// 整条链（宿主 own_apply.rs gpu_chrmed_u16 编排）：
//   prep  img(u16) → U/V 平面（U=B−Y / V=R−Y，709 亮度）
//   sel   对 U、V 平面各做一次窗口中值（r≤3 走单趟排序网络，r≥4 走
//         own_chrmed_sel.wgsl 的键域二分；见 own_nlm3.rs chrmed_med_ok）
//   emit  img(u16) + omu/omv → 重建 RGB 写回 img（原地）
//
// f32 精度，与 CPU 差 ±1 LSB 级。stride 在宿主侧按 w 行进（与 Mac 同），
// 此处忽略 stride。img 以 vec2<u32> 打包（同 own_apply_lut.wgsl）。

struct CDims { w: u32, h: u32, _p0: u32, _p1: u32 };

@group(0) @binding(0) var<storage, read_write> img: array<vec2<u32>>; // w*h
@group(0) @binding(1) var<storage, read_write> upl: array<u32>;      // w*h（f32 位模式）
@group(0) @binding(2) var<storage, read_write> vpl: array<u32>;      // w*h（f32 位模式）
@group(0) @binding(3) var<storage, read> omu: array<u32>;           // w*h（emit 读，f32 位模式）
@group(0) @binding(4) var<storage, read> omv: array<u32>;           // w*h（emit 读，f32 位模式）
@group(0) @binding(5) var<uniform> dims: CDims;

@compute @workgroup_size(16, 16)
fn chrmed_prep(@builtin(global_invocation_id) gid: vec3<u32>) {
    let w = dims.w;
    let h = dims.h;
    let x = gid.x;
    let y = gid.y;
    if (x >= w || y >= h) { return; }
    let i = y * w + x;
    let p = img[i];
    let r = f32(p.x & 0xffffu) / 65535.0;
    let gg = f32(p.x >> 16u) / 65535.0;
    let b = f32(p.y) / 65535.0;
    let yq = 0.2126 * r + 0.7152 * gg + 0.0722 * b;
    upl[i] = bitcast<u32>(b - yq);
    vpl[i] = bitcast<u32>(r - yq);
}

@compute @workgroup_size(16, 16)
fn chrmed_emit(@builtin(global_invocation_id) gid: vec3<u32>) {
    let w = dims.w;
    let h = dims.h;
    let x = gid.x;
    let y = gid.y;
    if (x >= w || y >= h) { return; }
    let i = y * w + x;
    let p = img[i];
    let r = f32(p.x & 0xffffu) / 65535.0;
    let gg = f32(p.x >> 16u) / 65535.0;
    let b = f32(p.y) / 65535.0;
    let y0 = 0.2126 * r + 0.7152 * gg + 0.0722 * b;
    let cb = bitcast<f32>(omu[i]);
    let cr = bitcast<f32>(omv[i]);
    let nr = clamp(y0 + cr, 0.0, 1.0);
    let ng = clamp(y0 - (0.2126 * cr + 0.0722 * cb) / 0.7152, 0.0, 1.0);
    let nb = clamp(y0 + cb, 0.0, 1.0);
    let ur = u32(nr * 65535.0 + 0.5);
    let ug = u32(ng * 65535.0 + 0.5);
    let ub = u32(nb * 65535.0 + 0.5);
    img[i] = vec2<u32>(ur | (ug << 16u), ub);
}
