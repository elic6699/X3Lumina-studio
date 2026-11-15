// own_apply_lut.wgsl — x3f_render.c apply_ex_common 显示域 LUT 管线的 GPU 版
// （own_gpu.m 的 apply_lut kernel，对应 MSL 319-415）。
//
// 顺序与 CPU apply_common 逐行对齐：增益 → 色彩模式 3×3（负夹 0）→ 逐通道
// sqrt 轴 LUT（LIN_MAX=8，SIZE=4096）→ 601 饱和度 → RAW 过曝驱动去饱和
// → 相机色彩匹配（65×65 Cb/Cr 双线性，亮度不动）。全部 f32 运算。
//
// u16 三通道用打包 vec2<u32> 表示（低 16=ch0，高 16=ch1，第二 u32=ch2），
// 与 own_nlm3.wgsl / own_chrmed_u16.wgsl 及宿主解包（p[0]&0xffff / p[0]>>16 /
// p[1]）一致。LUT 在宿主侧已归一到 [0,1]（u16/65535），这里直接索引 4097 项。
// 与 CPU 的 ±1 LSB 级差异仅来自 float（非逐位）；X3F_GPU_APPLY_CMP 做逐字节对拍。

struct APU { w: u32, h: u32, use_gain: u32, use_sat: u32 };
struct APV { has_cm3: u32, has_cm65: u32, has_clip: u32, _p: u32 };
// pf 也走 u32 + bitcast（对齐 nlm3 的全 u32 做法）：Vulkan 下 uniform 里的
// f32 字段同样有与 storage array<f32> 类似的读取隐患，统一规避。
struct APF { sat: u32, hl_desat: u32, desat_lo: u32, desat_hi: u32 };

// 注意：所有 host 上传的数值缓冲一律用 array<u32>，shader 内 bitcast<f32>
// 还原。原因：Vulkan 后端对 storage buffer 的 array<f32> 读取在某些驱动/版本
// 下会整体返回 0（而 array<u32> 正常）—— own_nlm3.wgsl 全程用 u32 在 Vulkan
// 下字节级通过，apply_lut 原先全用 f32 在 Vulkan 下整图崩（max=65535）。
// storage buffer 是裸字节，host 上传的 f32 位模式原样保留，u32 读 + bitcast 是
// 无损的；Metal 下 bitcast 等价，不破坏已验证的 ±1LSB 一致性。
@group(0) @binding(0) var<storage, read> lin: array<u32>;     // w*h*3（f32 位模式）
@group(0) @binding(1) var<storage, read> clip: array<u32>;    // w*h（has_clip 时，f32 位模式）
@group(0) @binding(2) var<storage, read> lut: array<u32>;     // 4097，已归一到 [0,1]（f32 位模式）
@group(0) @binding(3) var<storage, read> lutg: array<u32>;    // 4097
@group(0) @binding(4) var<storage, read> lutb: array<u32>;    // 4097
@group(0) @binding(5) var<storage, read_write> outb: array<vec2<u32>>; // w*h
@group(0) @binding(6) var<storage, read> gain: array<u32>;    // 3
@group(0) @binding(7) var<storage, read> cm3: array<u32>;     // 9
@group(0) @binding(8) var<storage, read> cmcb: array<u32>;    // 65*65
@group(0) @binding(9) var<storage, read> cmcr: array<u32>;    // 65*65
@group(0) @binding(10) var<uniform> pu: APU;
@group(0) @binding(11) var<uniform> pv: APV;
@group(0) @binding(12) var<uniform> pf: APF;

fn sstepf(a: f32, b: f32, x: f32) -> f32 {
    if (b <= a) { return select(0.0, 1.0, x >= b); }
    let t = clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// 65×65 表双线性（fx/fy 已夹在 [0,64]）。read-only storage 指针不能作函数
// 参数（naga 校验拒绝），改成传四个角点值的纯值函数。
fn bl(v00: f32, v10: f32, v01: f32, v11: f32, tx: f32, ty: f32) -> f32 {
    return (v00 + (v10 - v00) * tx) * (1.0 - ty) + (v01 + (v11 - v01) * tx) * ty;
}

@compute @workgroup_size(16, 16)
fn apply_lut(@builtin(global_invocation_id) gid: vec3<u32>) {
    let w = pu.w;
    let h = pu.h;
    let x = gid.x;
    let y = gid.y;
    if (x >= w || y >= h) { return; }
    let i = y * w + x;
    let si = i * 3u;

    var lv = vec3<f32>(bitcast<f32>(lin[si]), bitcast<f32>(lin[si + 1u]), bitcast<f32>(lin[si + 2u]));
    if (pu.use_gain != 0u) {
        lv = lv * vec3<f32>(bitcast<f32>(gain[0]), bitcast<f32>(gain[1]), bitcast<f32>(gain[2]));
    }
    if (pv.has_cm3 != 0u) {
        let a = lv.x; let b = lv.y; let e = lv.z;
        lv = vec3<f32>(
            a * bitcast<f32>(cm3[0]) + b * bitcast<f32>(cm3[1]) + e * bitcast<f32>(cm3[2]),
            a * bitcast<f32>(cm3[3]) + b * bitcast<f32>(cm3[4]) + e * bitcast<f32>(cm3[5]),
            a * bitcast<f32>(cm3[6]) + b * bitcast<f32>(cm3[7]) + e * bitcast<f32>(cm3[8]));
        lv = max(lv, vec3<f32>(0.0));
    }

    var d = vec3<f32>(0.0);
    for (var c = 0u; c < 3u; c = c + 1u) {
        var v = lv[c];
        if (v < 0.0) { v = 0.0; }
        let t = sqrt(v / 8.0);
        let idx = select(t * 4096.0, 4096.0, t >= 1.0);
        let i0 = u32(idx);
        let f = idx - f32(i0);
        var a0: f32;
        var b2: f32;
        if (i0 >= 4096u) {
            if (c == 0u) { a0 = bitcast<f32>(lut[4096u]); b2 = bitcast<f32>(lut[4096u]); }
            else if (c == 1u) { a0 = bitcast<f32>(lutg[4096u]); b2 = bitcast<f32>(lutg[4096u]); }
            else { a0 = bitcast<f32>(lutb[4096u]); b2 = bitcast<f32>(lutb[4096u]); }
        } else {
            if (c == 0u) { a0 = bitcast<f32>(lut[i0]); b2 = bitcast<f32>(lut[i0 + 1u]); }
            else if (c == 1u) { a0 = bitcast<f32>(lutg[i0]); b2 = bitcast<f32>(lutg[i0 + 1u]); }
            else { a0 = bitcast<f32>(lutb[i0]); b2 = bitcast<f32>(lutb[i0 + 1u]); }
        }
        d[c] = a0 + (b2 - a0) * f;
    }

    if (pu.use_sat != 0u) {
        let u = clamp(bitcast<f32>(pf.sat), 0.0, 2.0);
        let yv = 0.299 * d.x + 0.587 * d.y + 0.114 * d.z;
        d = vec3<f32>(yv + (d.x - yv) * u, yv + (d.y - yv) * u, yv + (d.z - yv) * u);
    }

    if (bitcast<f32>(pf.hl_desat) > 0.0) {
        var wgt: f32;
        if (pv.has_clip != 0u) {
            let cs = bitcast<f32>(clip[i]);
            wgt = sstepf(bitcast<f32>(pf.desat_lo), bitcast<f32>(pf.desat_hi), cs) * bitcast<f32>(pf.hl_desat);
        } else {
            let m = min(min(d.x, d.y), d.z);
            wgt = sstepf(0.80, 1.0, m) * bitcast<f32>(pf.hl_desat);
        }
        if (wgt > 0.001) {
            let dr = max(d.x, 0.0);
            let dg = max(d.y, 0.0);
            let db = max(d.z, 0.0);
            var yv = 0.2126 * dr + 0.7152 * dg + 0.0722 * db;
            if (pv.has_clip != 0u) {
                let cs = bitcast<f32>(clip[i]);
                let t2 = sstepf(1.6, 2.4, cs);
                if (t2 > 0.0) {
                    let mx = max(max(d.x, d.y), d.z);
                    if (mx > yv) { yv = yv + (mx - yv) * t2; }
                }
            }
            d = d + vec3<f32>((yv - d.x) * wgt, (yv - d.y) * wgt, (yv - d.z) * wgt);
        }
    }

    if (pv.has_cm65 != 0u) {
        let yy = 0.299 * d.x + 0.587 * d.y + 0.114 * d.z;
        let cb = d.z - yy;
        let cr = d.x - yy;
        let fx = (cb + 0.6) * 64.0 / 1.2;
        let fy = (cr + 0.6) * 64.0 / 1.2;
        if (fx >= 0.0 && fy >= 0.0 && fx <= 64.0 && fy <= 64.0) {
            let ix = u32(fx);
            let iy = u32(fy);
            let jx = select(ix + 1u, ix, ix + 1u >= 65u);
            let jy = select(iy + 1u, iy, iy + 1u >= 65u);
            let tx = fx - f32(ix);
            let ty = fy - f32(iy);
            let ncb = bl(bitcast<f32>(cmcb[iy * 65u + ix]), bitcast<f32>(cmcb[iy * 65u + jx]),
                         bitcast<f32>(cmcb[jy * 65u + ix]), bitcast<f32>(cmcb[jy * 65u + jx]), tx, ty);
            let ncr = bl(bitcast<f32>(cmcr[iy * 65u + ix]), bitcast<f32>(cmcr[iy * 65u + jx]),
                         bitcast<f32>(cmcr[jy * 65u + ix]), bitcast<f32>(cmcr[jy * 65u + jx]), tx, ty);
            let rr = yy + ncr;
            let bb = yy + ncb;
            let gg = (yy - 0.299 * rr - 0.114 * bb) / 0.587;
            d = vec3<f32>(rr, gg, bb);
        }
    }

    let u0 = u32(clamp(d.x, 0.0, 1.0) * 65535.0 + 0.5);
    let u1 = u32(clamp(d.y, 0.0, 1.0) * 65535.0 + 0.5);
    let u2 = u32(clamp(d.z, 0.0, 1.0) * 65535.0 + 0.5);
    outb[i] = vec2<u32>(u0 | (u1 << 16u), u2);
}
