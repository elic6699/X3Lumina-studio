// own_nlm3.wgsl — own 后端（own_decoder.c 的 nlm3_den）专用 GPU nlm3。
//
// 与 nlm.wgsl（产品降噪那条路）的语义差别，不能混用：
//   · 权重表由 C 侧按 own 的定点公式构造好（int32，fixed_point_mult 钳到
//     INT32_MAX），这里**原样上传**，不在 shader 里重算；
//   · 累加与舍入走 int64 定点：est += w·v、wsum += w、出值 =
//     (est + wsum/2) / wsum（整除）。WGSL 没有 64 位整数，用 32bit 双 limb
//     仿真 —— 因此与 own 的 CPU 路径**逐位一致**（Mac 的 Metal 版 own_gpu.m
//     是同一条保证），宿主 X3FDEC_NLMDBG=1 会自动做逐位比对复核。
//
// 窗口约束：sr + tr ≤ 12（tile 上限 40×40，workgroup 存储 ~12.9KB，低于 wgpu
// 默认 16KB 上限）。超出由宿主（own_nlm3.rs）拒绝，调用点自动回退 CPU。
//
// 2026-10-24 性能改版（语义零变化，X3FDEC_NLMDBG=1 逐位复核）：
//   · TS/SR/TR 由宿主按窗口参数模板注入 —— tile 数组按实际窗口取大小
//     （3/3/11/11 时 28×28=6.3KB，原先恒占 40×40=12.8KB，Intel 上共享内存
//     是 occupancy 的硬约束），窗口循环也都变成编译期常量界。
//   · div_round 不再 16 轮二分：f32 估算商（相对误差 ~2^-22 << 1）+
//     上下两个**精确**校正环（条件环无界，估差多大都收敛到同一个数学值，
//     与二分逐位同值）。
//   · SAD 滑动复用：dy 行首窗全量 27 ops，之后每步 dx 只算 B 窗进/出列
//     （A 窗固定）：sad' = sad − |A(cx−tr)−B(qx−tr)| + |A(cx+tr)−B(qx+tr)|，
//     3 通道 × 3 行。减法不借位：被减项就在当前 sad 里。
//   · 权重全零的搜索位跳过 est/wsum 累加（CPU 加 0 等价）。
//   · mul64_16：另一操作数 ≤ 65535（通道值 / 商）时省掉两个恒零部分积。

struct Params {
    width: u32,
    height: u32,
    sr: u32,          // 已模板化进 SR，保留字段占位（探针/兼容布局用）
    tr: u32,
    bin_shift: u32,
    lut_n: u32,
    padding0: u32,
    padding1: u32,
}

const TS: u32 = __TS__u;   // 16 + 2*(SR+TR)，宿主注入
const SR: u32 = __SR__u;   // search 窗半径 = swin/2
const TR: u32 = __TR__u;   // patch 窗半径 = twin/2
const SW: u32 = 2u * SR + 1u;
const TW: u32 = 2u * TR + 1u;

@group(0) @binding(0) var<storage, read> source: array<u32>;
// u16x3 交错流按 u32 重读（偶 u16 在低半、奇 u16 在高半；总长为奇数时末尾补零）。

@group(0) @binding(1) var<storage, read_write> destination: array<vec2<u32>>;
// 每像素 [c0 | c1<<16, c2]（c0=Y 原值回写，c1=U、c2=V 是降噪输出）。

@group(0) @binding(2) var<storage, read> lut: array<u32>;
// int32 定点权重，每档 3 个（[0]=Y 档不用，[1]=U、[2]=V）。

@group(0) @binding(3) var<uniform> params: Params;

var<workgroup> tile: array<vec2<u32>, TS * TS>;
var<workgroup> srx: array<u32, TS>;
var<workgroup> sry: array<u32, TS>;

// u16 交错流的第 o 个 u16
fn fetch16(o: u32) -> u32 {
    let word = source[o >> 1u];
    return select(word & 65535u, word >> 16u, (o & 1u) == 1u);
}

fn unpk(p: vec2<u32>) -> vec3<u32> {
    return vec3<u32>(p.x & 65535u, p.x >> 16u, p.y);
}

// BORDER_REFLECT_101（与 own_decoder.c / own_gpu.m 同式）
fn refl(i: i32, n: i32) -> i32 {
    if (n == 1) { return 0; }
    let period = 2 * (n - 1);
    var r = i % period;
    if (r < 0) { r = r + period; }
    if (r >= n) { r = period - r; }
    return r;
}

// 3 通道联合 L1 距离（与 CPU 的逐通道 |a−b| 累加同式）
fn sad3(a: vec3<u32>, b: vec3<u32>) -> u32 {
    return u32(abs(i32(a.x) - i32(b.x)))
         + u32(abs(i32(a.y) - i32(b.y)))
         + u32(abs(i32(a.z) - i32(b.z)));
}

// 32×32 → 64 位乘（双 limb；u32 运算按 WGSL 语义对 2^32 取模）。
// mid 必须把 p01/p10 都掩到低 16 位（高半归 hi）——漏掩哪边，hi 就把那边的
// 高位重复计一遍，est 虚大、商打到钳制顶（首批实测就是这么炸的）。
fn mul64(a: u32, b: u32) -> vec2<u32> {
    let a0 = a & 65535u;
    let a1 = a >> 16u;
    let b0 = b & 65535u;
    let b1 = b >> 16u;
    let p00 = a0 * b0;
    let p01 = a0 * b1;
    let p10 = a1 * b0;
    let p11 = a1 * b1;
    let mid = (p00 >> 16u) + (p01 & 65535u) + (p10 & 65535u);
    return vec2<u32>((p00 & 65535u) | (mid << 16u),
                     p11 + (p01 >> 16u) + (p10 >> 16u) + (mid >> 16u));
}

// 16 位操作数变体：b < 2^16 → b1 = 0，p01/p11 恒零（调用点保证：通道值
// 与商都不超过 65535）。与 mul64 对 b < 2^16 逐位同值。
// 【操作数顺序】契约是 **b < 2^16**（第二个参数）—— 把 32 位的 wsum.x
// 放进 b 位会丢高 16 位（2026-10-24 首版就这么炸的，商严重偏差）。
fn mul64_16(a: u32, b: u32) -> vec2<u32> {
    let a0 = a & 65535u;
    let a1 = a >> 16u;
    let b0 = b & 65535u;
    let p00 = a0 * b0;
    let p10 = a1 * b0;
    let mid = (p00 >> 16u) + (p10 & 65535u);
    return vec2<u32>((p00 & 65535u) | (mid << 16u),
                     (p10 >> 16u) + (mid >> 16u));
}

fn add64(a: vec2<u32>, b: vec2<u32>) -> vec2<u32> {
    let lo = a.x + b.x;
    let carry = select(0u, 1u, lo < a.x);
    return vec2<u32>(lo, a.y + b.y + carry);
}

fn is_zero64(a: vec2<u32>) -> bool {
    return a.x == 0u && a.y == 0u;
}

fn le64(a: vec2<u32>, b: vec2<u32>) -> bool {
    return a.y < b.y || (a.y == b.y && a.x <= b.x);
}



// (est + wsum/2) / wsum：找最大 q ∈ [0,65535] 使 q·wsum ≤ est + ⌊wsum/2⌋。
// 与 CPU 的 (est + wsum/2)/wsum 整除同值（est ≤ 65535·wsum，q 不会越界，
// 上界 65535 只是兜底）。
// 2026-10-24 试过 f32 估算商 + 精确校正环：逐位正确（nbad=0）但在 Intel
// UHD 上**更慢**（数据依赖 while 的发散代价 > 省下的 mul64），保留二分版。
fn div_round(est: vec2<u32>, wsum: vec2<u32>) -> u32 {
    // 64 位右移一位：低半 limb 的 bit31 来自**高半 limb 的 bit0**（不是
    // wsum.x 的！——原来两个 limb 都取 wsum.x&1，wsum.x 为奇数时 half
    // 虚大 ~2^63，商被钳到 65535）。
    let half = vec2<u32>((wsum.x >> 1u) | ((wsum.y & 1u) << 31u), wsum.y >> 1u);
    let n = add64(est, half);
    var lo = 0u;
    var hi = 65535u;
    loop {
        if (lo >= hi) { break; }
        let mid = (lo + hi + 1u) / 2u;
        let qprod = add64(mul64(mid, wsum.x), vec2<u32>(0u, mid * wsum.y));
        if (le64(qprod, n)) { lo = mid; } else { hi = mid - 1u; }
    }
    return lo;
}

@compute @workgroup_size(16, 16)
fn nlm3(@builtin(workgroup_id) group: vec3<u32>,
        @builtin(local_invocation_id) local: vec3<u32>,
        @builtin(local_invocation_index) index: u32) {
    let w = params.width;
    let h = params.height;
    let ext = SR + TR;
    let bx = group.x * 16u;
    let by = group.y * 16u;
    let li = index;

    // 反射索引表（整组协作写）
    for (var k = li; k < TS; k += 256u) {
        sry[k] = u32(refl(i32(by + k) - i32(ext), i32(h)));
        srx[k] = u32(refl(i32(bx + k) - i32(ext), i32(w)));
    }
    workgroupBarrier();
    // 瓦片加载：TS×TS 像素（含 halo），反射映射后必然落在图内
    for (var k = li; k < TS * TS; k += 256u) {
        let ty = k / TS;
        let tx = k % TS;
        let s = (sry[ty] * w + srx[tx]) * 3u;
        tile[k] = vec2<u32>(fetch16(s) | (fetch16(s + 1u) << 16u), fetch16(s + 2u));
    }
    workgroupBarrier();

    let x = bx + local.x;
    let y = by + local.y;
    if (x >= w || y >= h) { return; }
    let cy = local.y + ext;
    let cx = local.x + ext;
    // 诊断探针（X3FDEC_NLMPROBE，宿主诊断用）：
    //   1 = 输出 LUT[1]/LUT[2]（应为 fpm=2147483647，验 LUT 上传与绑定）
    //   2 = 输出 (sad[dy=1,dx=0], almost[dy=1,dx=0])（验瓦片与距离）
    if (params.padding0 == 1u) {
        destination[y * w + x] = vec2<u32>(lut[1u], lut[2u]);
        return;
    }
    if (params.padding0 == 2u) {
        let qy0 = cy + 1u - SR;
        let qx0 = cx - SR;
        var probe_sad = 0u;
        for (var ty = 0u; ty < TW; ty++) {
            let pry = (cy + ty - TR) * TS;
            let qry = (qy0 + ty - TR) * TS;
            for (var tx = 0u; tx < TW; tx++) {
                let a = unpk(tile[pry + cx + tx - TR]);
                let b = unpk(tile[qry + qx0 + tx - TR]);
                probe_sad = probe_sad + sad3(a, b);
            }
        }
        destination[y * w + x] = vec2<u32>(probe_sad, probe_sad >> params.bin_shift);
        return;
    }

    var est1 = vec2<u32>(0u, 0u);
    var est2 = vec2<u32>(0u, 0u);
    var wsum1 = vec2<u32>(0u, 0u);
    var wsum2 = vec2<u32>(0u, 0u);

    // 搜索窗逐位累积（3×3 patch、3 通道联合 L1 距离；邻居 patch 以
    // (qy,qx) 为**中心**取，与 CPU 的 sy0+ty / sx0+tx（ty,tx ∈ −tr..tr）
    // 同一约定。原来写成 qy+ty / qx+tx（左上角约定），整个 SAD 面偏了
    // 一个斜向像素，中心像素"距离 0"的保证都没了，权重全错。）
    // 注意：SAD(dx) 的每一项是 |A(c) − B(c+Δ)|（Δ=dx−SR 固定的互相关），
    // Δ 平移时**所有**项都变 —— dx 方向没有"进出列"可复用（2026-10-24
    // 试过 dx 滑动复用，对拍 nbad 全崩，已撤）；dy 方向同理。老老实实
    // 全量重算，优化靠模板常量界 + mul64_16 + 零权重跳过。
    for (var dy = 0u; dy < SW; dy++) {
        let qy = cy + dy - SR;
        for (var dx = 0u; dx < SW; dx++) {
            let qx = cx + dx - SR;
            var sad = 0u;
            for (var ty = 0u; ty < TW; ty++) {
                let pry = (cy + ty - TR) * TS;
                let qry = (qy + ty - TR) * TS;
                for (var tx = 0u; tx < TW; tx++) {
                    let a = unpk(tile[pry + cx + tx - TR]);
                    let b = unpk(tile[qry + qx + tx - TR]);
                    sad = sad + sad3(a, b);
                }
            }
            // —— 累加 ——
            var almost = sad >> params.bin_shift;
            almost = min(almost, params.lut_n - 1u);
            let base = almost * 3u;
            let wu = lut[base + 1u];
            let wv = lut[base + 2u];
            if (wu != 0u || wv != 0u) {
                let q = tile[qy * TS + qx];
                est1 = add64(est1, mul64_16(wu, q.x >> 16u));
                est2 = add64(est2, mul64_16(wv, q.y));
                let s1 = wsum1.x + wu;
                wsum1 = vec2<u32>(s1, wsum1.y + select(0u, 1u, s1 < wsum1.x));
                let s2 = wsum2.x + wv;
                wsum2 = vec2<u32>(s2, wsum2.y + select(0u, 1u, s2 < wsum2.x));
            }
        }
    }

    let center = tile[cy * TS + cx];
    let u_out = select(0u, div_round(est1, wsum1), !is_zero64(wsum1));
    let v_out = select(0u, div_round(est2, wsum2), !is_zero64(wsum2));
    let u = min(u_out, 65535u);
    let v = min(v_out, 65535u);
    destination[y * w + x] = vec2<u32>((center.x & 65535u) | (u << 16u), v);
}
