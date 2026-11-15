// own_down8.wgsl — x3f_render.c apply_ex_common 8 位降位的 GPU 版
// （own_gpu.m 的 down8k，MSL 464-480）。显示域线性 u16 → 8 位：+128 半档
// 偏置，过曝矫正关闭时（hl_soften）加 4×4 Bayer4 抖动。整数移位，与 CPU
// down8 循环逐字节一致。X3F_GPU_APPLY_CMP 不比对本段（只比 lut+chrmed 的
// tmp），但写成一致保证预览外观 == 导出。
//
// 数据布局（与 own_apply_lut / own_chrmed_u16 一致，u16 三通道用打包
// vec2<u32>；宿主负责打包/解包）：
//   tmp : array<vec2<u32>>  每像素一个 vec2（低16=ch0，高16=ch1，第二=ch2）
//   out : array<u32>        每像素一个 u32，打包 (r | g<<8 | b<<16)；宿主按
//                           stride 解包回 flat u8。

const BAYER: array<u32, 16> = array<u32, 16>(
    0u, 8u, 2u, 10u, 12u, 4u, 14u, 6u, 3u, 11u, 1u, 9u, 15u, 7u, 13u, 5u
);

struct D8P { w: u32, h: u32, stride: u32, dither: u32 };

@group(0) @binding(0) var<storage, read> tmp: array<vec2<u32>>;
@group(0) @binding(1) var<storage, read_write> out: array<u32>;
@group(0) @binding(2) var<uniform> p: D8P;

@compute @workgroup_size(16, 16)
fn down8k(@builtin(global_invocation_id) gid: vec3<u32>) {
    let w = p.w;
    let h = p.h;
    let x = gid.x;
    let y = gid.y;
    if (x >= w || y >= h) { return; }
    let i = y * w + x;
    let px = tmp[i];
    let c0 = f32(px.x & 0xffffu);
    let c1 = f32(px.x >> 16u);
    let c2 = f32(px.y);

    var dth: i32 = 0;
    if (p.dither != 0u) {
        // 抖动索引与 CPU 同：像素 x 的 (x&3)（C 循环里 x/3&3 等价于此）。
        // 复刻 C/ML 的 (unsigned)dth 语义：负 dither 走同款 u32 回绕、末端钳 255。
        let bi = ((y & 3u) << 2u) | (x & 3u);
        dth = i32(BAYER[bi]) - 8;
        dth = dth * 16;
    }
    let dthu = u32(dth);

    let r = (u32(c0) + 128u + dthu) >> 8u;
    let g = (u32(c1) + 128u + dthu) >> 8u;
    let b = (u32(c2) + 128u + dthu) >> 8u;
    let rb = min(r, 255u);
    let gb = min(g, 255u);
    let bb = min(b, 255u);
    out[i] = (bb << 16u) | (gb << 8u) | rb;
}
