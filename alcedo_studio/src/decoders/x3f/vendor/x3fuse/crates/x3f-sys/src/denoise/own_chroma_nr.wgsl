// own_chroma_nr.wgsl — own 后端解码后色度降噪（Cr/Cb 3×3 中值）的 GPU 版。
//
// 对应 own_decoder.c 的 nr_job：对 cr/cb 两块 float 平面做 3×3 中值滤波
// （边界钳制 + 双插入排序取 [4]）。纯比较选择 + 原样搬运的 f32 → 与 CPU
// **逐位一致**（IEEE 比较语义相同，中值取的是输入样本之一，不存在算术）。
// 迭代 ping-pong 由宿主（own_nlm3.rs）逐次派发保证顺序，shader 只做一次。
//
// 平面按 vec2<f32>（cr, cb）交错存放；边界钳制与 nr_job 的
// `yy<0 → 0 / yy>=h → h-1` 同式（clamp）。
//
// chroma extract（double 系数）与 rec_job（double 重组）留在 CPU ——
// WGSL 没有 f64，那两步的逐位一致 GPU 做不到，也没必要（占比 <0.2s）。

struct CNParams {
    width: u32,
    height: u32,
    p0: u32,
    p1: u32,
}

@group(0) @binding(0) var<storage, read> src: array<vec2<f32>>;
@group(0) @binding(1) var<storage, read_write> dst: array<vec2<f32>>;
@group(0) @binding(2) var<uniform> p: CNParams;

@compute @workgroup_size(16, 16)
fn chroma_nr(@builtin(global_invocation_id) gid: vec3<u32>) {
    let x = gid.x;
    let y = gid.y;
    if (x >= p.width || y >= p.height) { return; }

    var rs: array<f32, 9>;
    var bs: array<f32, 9>;
    var m = 0u;
    for (var dy: i32 = -1; dy <= 1; dy++) {
        var yy: i32 = i32(y) + dy;
        yy = clamp(yy, 0, i32(p.height) - 1);
        for (var dx: i32 = -1; dx <= 1; dx++) {
            var xx: i32 = i32(x) + dx;
            xx = clamp(xx, 0, i32(p.width) - 1);
            let v = src[u32(yy) * p.width + u32(xx)];
            rs[m] = v.x;
            bs[m] = v.y;
            m = m + 1u;
        }
    }
    // 插入排序（与 nr_job 逐行同式；比较器 `>` 语义一致）
    for (var a = 1u; a < 9u; a++) {
        var kv: f32 = rs[a];
        var b2: i32 = i32(a) - 1;
        while (b2 >= 0 && rs[b2] > kv) {
            rs[b2 + 1] = rs[b2];
            b2 = b2 - 1;
        }
        rs[b2 + 1] = kv;
        kv = bs[a];
        b2 = i32(a) - 1;
        while (b2 >= 0 && bs[b2] > kv) {
            bs[b2 + 1] = bs[b2];
            b2 = b2 - 1;
        }
        bs[b2 + 1] = kv;
    }
    dst[y * p.width + x] = vec2<f32>(rs[4], bs[4]);
}
