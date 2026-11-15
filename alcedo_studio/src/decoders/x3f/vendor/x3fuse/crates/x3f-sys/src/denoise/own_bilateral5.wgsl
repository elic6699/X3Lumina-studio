// own_bilateral5.wgsl — own 后端 5×5 可分离双边滤波（bilateral5）的 GPU 版。
//
// 对应 own_decoder.c 的 bilateral5：水平 pass → tmp，垂直 pass → 原图。
// 值域核 rlut（C 侧预计算，16384 项，0.25 步长线性插值）、空间核 sw[5]
// （C 侧 expf 预计算，原样上传 —— GPU 的 exp 与 C 的 expf ulp 不同，
// 不能在 GPU 重算）。边界钳制（clamp 到 0/w−1）与 CPU 同式。
//
// 逐位一致性：f32 运算、k 循环序（−2..2）与累加序与 CPU 相同；
// rlut/sw 原样上传。两个 pass 由宿主分两次派发（中间隐式同步）。
// 宿主 A/B：X3FDEC_GPU=0 对比 rgb dump 应逐位相同。

struct BParams {
    w: u32,
    h: u32,
    rlut_n: u32,
    p0: u32,
}

@group(0) @binding(0) var<storage, read> input: array<f32>;
@group(0) @binding(1) var<storage, read_write> output: array<f32>;
@group(0) @binding(2) var<storage, read> rlut: array<f32>;
@group(0) @binding(3) var<storage, read> sw: array<f32>;     // 5 项
@group(0) @binding(4) var<uniform> p: BParams;

@compute @workgroup_size(256)
fn bilat_h(@builtin(global_invocation_id) gid: vec3<u32>) {
    let idx = gid.x;
    if (idx >= p.w * p.h) { return; }
    let y = idx / p.w;
    let x = idx % p.w;
    let row = y * p.w;
    let c = input[row + x];
    var acc = 0.0;
    var wsum = 0.0;
    for (var k = 0u; k < 5u; k++) {
        var xx = i32(x) + i32(k) - 2;
        if (xx < 0) { xx = 0; } else if (xx >= i32(p.w)) { xx = i32(p.w) - 1; }
        let v = input[row + u32(xx)];
        var ad = v - c;
        if (ad < 0.0) { ad = -ad; }
        let af = ad * 4.0;
        let wi = i32(af);
        var wv = 0.0;
        if (wi + 1 < i32(p.rlut_n)) {
            wv = rlut[u32(wi)] + (rlut[u32(wi) + 1u] - rlut[u32(wi)]) * (af - f32(wi));
        }
        let wgt = sw[k] * wv;
        acc = acc + v * wgt;
        wsum = wsum + wgt;
    }
    output[idx] = acc / wsum;
}

@compute @workgroup_size(256)
fn bilat_v(@builtin(global_invocation_id) gid: vec3<u32>) {
    let idx = gid.x;
    if (idx >= p.w * p.h) { return; }
    let y = idx / p.w;
    let x = idx % p.w;
    let c = input[idx];
    var acc = 0.0;
    var wsum = 0.0;
    for (var k = 0u; k < 5u; k++) {
        var yy = i32(y) + i32(k) - 2;
        if (yy < 0) { yy = 0; } else if (yy >= i32(p.h)) { yy = i32(p.h) - 1; }
        let v = input[u32(yy) * p.w + x];
        var ad = v - c;
        if (ad < 0.0) { ad = -ad; }
        let af = ad * 4.0;
        let wi = i32(af);
        var wv = 0.0;
        if (wi + 1 < i32(p.rlut_n)) {
            wv = rlut[u32(wi)] + (rlut[u32(wi) + 1u] - rlut[u32(wi)]) * (af - f32(wi));
        }
        let wgt = sw[k] * wv;
        acc = acc + v * wgt;
        wsum = wsum + wgt;
    }
    output[idx] = acc / wsum;
}
