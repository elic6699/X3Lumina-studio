/* own_chrmed_sel.wgsl — 色度中值单平面窗口中值（GPU 版，r 半径模板注入）
 *
 * 对应 x3f_render.c apply_chroma_median 的 GPU 平面步骤（own_gpu_chrmed_plane）：
 * 输入 pl（w×h f32，U 或 V 平面），输出 omed = 每像素 (2r+1)² 窗口（边界
 * clamp 收缩，同 CPU chrmed_row 的 y0r/y1r/x0r/x1r）的中值。
 *
 * 选中值与 CPU chrmed_select 值精确一致：都是「同一浮点多重集合的第 n>>1
 * 小值（0-based）」，与选择算法无关（Mac own_gpu.m 同口径，见其注释）。
 * 算法 = Mac 第四版「键域二分基数选择」：float 经 fkey 映射成 u32 全序
 * （正数置符号位、负数取反），二分 32 趟找第 k 小。fkey 与 CPU 浮点比较的
 * 唯一分歧是 −0.0/+0.0（映射后不同、CPU 比较相等），两者进重建运算结果
 * 相同，可接受（Mac 注释原话）。
 *
 * 架构照 Mac：WG×WG workgroup 协作加载 (WG+2r)² tile 到 workgroup 内存
 * （fkey 预算好，二分趟内纯 u32 比较），边界坐标 clamp 到图像（重复边缘，
 * 收缩窗口用不到重复区、固定窗口语义才用；两种语义样本都真实）。
 *
 * 版本史（Intel UHD Vulkan 实测，5424×3616 r=3，每平面）：
 *  ① 窗口收进 per-thread 数组（固定 225 槽）——scratch 900B/线程，3.1s；
 *  ② 现版 tile 协作：2.2s（WG 8/16 无差，非 occupancy 问题，是 31 趟 ×
 *     49 次 TG 读的带宽极限）；
 *  ③ per-thread 精确槽位 key 数组（196B scratch）+ min/max 初始界——
 *     2.7~3.1s，scratch 访问确实比 TG 读慢，弃。
 *
 * __R__ / __WG__ / __TSN__ 由宿主按半径与 workgroup 边长注入，
 * 每个组合只编一次 pipeline。
 *
 * ★ 2026-10-31：**r ≤ 3 已改走单趟排序网络**（own_nlm3.rs 的
 * build_chrmed_med_wgsl，shared 读 32×N → N，5424×3616 单平面 2.67s →
 * 0.26s）。本文件现在只服务两种情形：
 *   ① r ≥ 4 的大窗口（169/225 元素网络过大，寄存器放不下）；
 *   ② 新 shader 里边界像素的收缩窗口（n 可变 → k 不固定，网络没法定位置）。
 * 二分本身没动，对拍结论（与 CPU chrmed_select 值精确一致）继续有效。
 */

struct Dims {
    w: u32,
    h: u32,
    pad0: u32,
    pad1: u32,
};

// pl/omed 改 array<u32>（对齐 nlm3 的全 u32 做法，规避 Vulkan 下 f32 storage
// 读取隐患）。pl 是 prep 写出的 U/V 平面（原 f32 位模式），omed 是选中值（unkey
// 已还原 f32，这里再 bitcast 成 u32 存回）。tile 本身是 workgroup array<u32>。
@group(0) @binding(0) var<storage, read> pl: array<u32>;
@group(0) @binding(1) var<storage, read_write> omed: array<u32>;
@group(0) @binding(2) var<uniform> dims: Dims;

// float → u32 全序映射（IEEE754 位序修正符号翻转）
fn fkey(v: f32) -> u32 {
    let b = bitcast<u32>(v);
    return select(b | 0x80000000u, ~b, (b & 0x80000000u) != 0u);
}

// fkey 逆映射
fn unkey(k: u32) -> f32 {
    let pos = (k & 0x80000000u) != 0u;
    return bitcast<f32>(select(~k, k & 0x7fffffffu, pos));
}

const R: u32 = __R__u;
const WG: u32 = __WG__u;               // workgroup 边长（宿主注入）
const TS: u32 = WG + 2u * R;           // tile 边长
const TSN: u32 = TS * TS;              // tile 元素数（workgroup 数组必须是编译期常量）

var<workgroup> tile: array<u32, __TSN__>;

@compute @workgroup_size(WG, WG)
fn chrmed_sel(
    @builtin(global_invocation_id) g: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
    @builtin(workgroup_id) tid: vec3<u32>,
) {
    let w = dims.w;
    let h = dims.h;
    let x = g.x;
    let y = g.y;
    let bx = tid.x * WG;
    let by = tid.y * WG;
    let li = lid.y * WG + lid.x;

    // 协作加载 tile：坐标 clamp 到图像（重复边缘），fkey 预算好存 u32
    for (var kk = li; kk < TSN; kk = kk + WG * WG) {
        let tyy = kk / TS;
        let txx = kk - tyy * TS;
        var sy = by + tyy;
        sy = select(sy - R, 0u, sy < R);
        sy = min(sy, h - 1u);
        var sx = bx + txx;
        sx = select(sx - R, 0u, sx < R);
        sx = min(sx, w - 1u);
        tile[kk] = fkey(bitcast<f32>(pl[sy * w + sx]));
    }
    workgroupBarrier();

    if (x >= w || y >= h) {
        return;
    }

    // 窗口收缩（与 CPU chrmed_row 同式：越界半径收缩，不镜像）
    let y0 = select(y - R, 0u, y < R);
    let y1 = min(y + R, h - 1u);
    let x0 = select(x - R, 0u, x < R);
    let x1 = min(x + R, w - 1u);
    let n = (y1 - y0 + 1u) * (x1 - x0 + 1u);
    let k = n >> 1u;

    // tile 内窗口区域（tile 原点 = (bx-R, by-R)）
    let ty0 = y0 + R - by;
    let ty1 = y1 + R - by;
    let tx0 = x0 + R - bx;
    let tx1 = x1 + R - bx;

    // 键域二分：第 k 小（0-based）—— 与 chrmed_select(a,n,n>>1) 值精确一致
    var lo: u32 = 0u;
    var hi: u32 = 0xffffffffu;
    loop {
        if (lo >= hi) {
            break;
        }
        let mid = lo + (hi - lo) / 2u;
        var cnt: u32 = 0u;
        for (var tyy = ty0; tyy <= ty1; tyy = tyy + 1u) {
            let rowbase = tyy * TS;
            for (var txx = tx0; txx <= tx1; txx = txx + 1u) {
                if (tile[rowbase + txx] <= mid) {
                    cnt = cnt + 1u;
                }
            }
        }
        if (cnt > k) {
            hi = mid;
        } else {
            lo = mid + 1u;
        }
    }
    omed[y * w + x] = bitcast<u32>(unkey(lo));
}
