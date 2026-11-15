//! DNG 高光通道比重映射（oe-replay）。
//!
//! mode7/8 渲染链高光修复研究成果在 raw 域的实装（2026-10-29 原型闭环，
//! workspace `flame_cmp/oe_id/`）。核心结论：
//!
//! - mode7/8 的高光修复**不是**保色相的 gamut mapping（解析式 gamut 压缩
//!   实测失败：火焰 MAE 63.9 vs 比例 LUT 30.8），而是把出域高光按
//!   「源通道比 → 目标通道比」LUT 往暖白方向重映射；
//! - 逐帧自适应（chroma LUT 式）学不出裁切区的目标比例 ⇒ 必须用离线
//!   标定的固定表（同 oe_spec 哲学）；
//! - 作用域 = equalize 之后、strip 编码之前的 LinearRaw（与原型验证域一致）；
//! - tone/亮度差**不修**——那是外部渲染器的领域，烘进去必打架。
//!
//! 标定方法：mode7 渲染当老师 → 反演渲染链（gamma_inv → inv XYZ2sRGB →
//! inv CM1 → ·ASN）得 raw 域目标 → 按源通道比分 48 bin 取截尾中位数。
//! 当前表由 DP2Q2501 单帧标定（Quattro）；扩多帧标定前只作实验档。
//!
//! 默认关：`ProcessOptions::dng_oe_replay == false` 时本模块完全不参与，
//! 输出与旧版逐位一致。

use crate::image::Image;

pub(crate) const RATIO_BINS: usize = 48;

/// R/G 修正 LUT：key = 源 R/G ∈ [R_LO, R_HI)，value = 目标 R/G。
pub(crate) const LUT_R: [f32; RATIO_BINS] = [
    1.0122, 1.0122, 1.0122, 1.0122, 1.0122, 1.0143,
    1.0166, 1.0165, 1.0149, 1.0105, 1.0021, 0.9913,
    0.9805, 0.9741, 0.9677, 0.9611, 0.9538, 0.9494,
    0.9420, 0.9391, 0.9354, 0.9364, 0.9397, 0.9400,
    0.9355, 0.9359, 0.9371, 0.9375, 0.9432, 0.9559,
    0.9647, 0.9672, 0.9701, 0.9712, 0.9682, 0.9673,
    0.9700, 0.9705, 0.9705, 0.9705, 0.9705, 0.9705,
    0.9705, 0.9705, 0.9705, 0.9705, 0.9705, 0.9705,
];

/// B/G 修正 LUT：key = 源 B/G ∈ [B_LO, R_HI)，value = 目标 B/G。
pub(crate) const LUT_B: [f32; RATIO_BINS] = [
    0.7140, 0.7140, 0.7140, 0.5917, 0.4774, 0.3782,
    0.2969, 0.3616, 0.4797, 0.6181, 0.7601, 0.8940,
    0.9968, 1.0677, 1.1228, 1.1710, 1.2202, 1.3023,
    1.3856, 1.4566, 1.5156, 1.5639, 1.5719, 1.5719,
    1.5719, 1.5719, 1.5719, 1.5719, 1.5719, 1.5719,
    1.5719, 1.5719, 1.5719, 1.5719, 1.5719, 1.5719,
    1.5719, 1.5719, 1.5719, 1.5719, 1.5719, 1.5719,
    1.5719, 1.5719, 1.5719, 1.5719, 1.5719, 1.5719,
];

/// R/G key 区间下限。
const R_LO: f32 = 0.45;
/// B/G key 区间下限。
const B_LO: f32 = 0.35;
/// 两个 LUT 共用的 key 区间上限。
const R_HI: f32 = 2.0;

/// 亮度混合带（u16 域）：G 低于 BLEND0 不作用，BLEND1 起全量。
const BLEND0: f32 = 2500.0;
const BLEND1: f32 = 8000.0;

/// 只压不升：新比例相对原值的下限（防止把中性区推离白平衡）。
const MIN_KEEP: f32 = 0.4;

#[inline]
fn lut_lookup(lut: &[f32; RATIO_BINS], key: f32, lo: f32) -> f32 {
    let span = R_HI - lo;
    let idx = (((key - lo) / span) * RATIO_BINS as f32).floor() as isize;
    let idx = idx.clamp(0, RATIO_BINS as isize - 1) as usize;
    lut[idx]
}

/// 对 equalize 之后的 LinearRaw 施加通道比重映射。
///
/// 每像素：`w` 为亮度混合权重（暗部不作用）；R、B 各查自己的比例 LUT 得
/// 目标值，`clamp(target, MIN_KEEP·原值, 原值)` 保证**只压不升**，再按
/// `w` 与原值混合。G 通道作为锚不动（mode8 的结构基准思想）。
pub fn apply_oe_ratio_remap(image: &mut Image) {
    let channels = image.channels as usize;
    if channels < 3 {
        return;
    }
    let cols = image.columns as usize;
    let stride = image.row_stride as usize;
    let blend_span = BLEND1 - BLEND0;

    for row in image.data.chunks_mut(stride) {
        for col in 0..cols {
            let off = col * channels;
            let g = row[off + 1] as f32;
            if g <= BLEND0 {
                continue;
            }
            let w = if g >= BLEND1 {
                1.0
            } else {
                (g - BLEND0) / blend_span
            };

            let r = row[off] as f32;
            let b = row[off + 2] as f32;

            // R：按源 R/G 查表
            let r_key = r / g.max(1.0);
            let r_target = g * lut_lookup(&LUT_R, r_key, R_LO);
            let r_target = r_target.clamp(r * MIN_KEEP, r); // 只压不升
            row[off] = (r * (1.0 - w) + r_target * w).round().clamp(0.0, 65535.0) as u16;

            // B：按源 B/G 查表
            let b_key = b / g.max(1.0);
            let b_target = g * lut_lookup(&LUT_B, b_key, B_LO);
            let b_target = b_target.clamp(b * MIN_KEEP, b); // 只压不升
            row[off + 2] = (b * (1.0 - w) + b_target * w).round().clamp(0.0, 65535.0) as u16;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn img(pixels: &[[u16; 3]]) -> Image {
        let data: Vec<u16> = pixels.iter().flat_map(|p| p.iter().copied()).collect();
        Image {
            data,
            rows: pixels.len() as u32,
            columns: 1,
            channels: 3,
            row_stride: 3,
            levels: Default::default(),
            dng_highlight_scale: 1.0,
            dng_shoulder_ceiling: 1.0,
        }
    }

    #[test]
    fn dark_pixels_untouched() {
        let mut im = img(&[[3000, 2000, 1000], [100, 50, 25]]);
        apply_oe_ratio_remap(&mut im);
        assert_eq!(&im.data, &[3000, 2000, 1000, 100, 50, 25]);
    }

    #[test]
    fn neutral_highlight_compresses_like_teacher() {
        // 学到的表在 r=1.0 处目标 ≈0.949：老师（mode7）确实把热区中性像素
        // 的 R 轻压 ~5%（高光往暖白重映射的一部分，原型全图 MAE 因此改善）。
        let mut im = img(&[[30000, 30000, 30000]]);
        apply_oe_ratio_remap(&mut im);
        let r = im.data[0];
        assert!(r > 28200 && r < 28800, "neutral hot R should compress ~5%, got {r}");
        assert_eq!(im.data[1], 30000); // G 锚不动
        assert!(im.data[2] <= 30000); // 不允许放大

        // 高 R/G 热像素必须被压得更狠（向 G 收敛）
        let mut im = img(&[[40000, 30000, 20000]]);
        apply_oe_ratio_remap(&mut im);
        assert!(im.data[0] < 34000, "magenta R should be strongly compressed, got {}", im.data[0]);
        assert!(im.data[2] <= 20000);
    }

    #[test]
    fn blend_is_continuous_at_blend0() {
        // G=BLEND0 处权重 0，输出等于原值
        let mut im = img(&[[5000, 2500, 2500]]);
        apply_oe_ratio_remap(&mut im);
        assert_eq!(im.data[0], 5000);
        assert_eq!(im.data[2], 2500);
    }
}
