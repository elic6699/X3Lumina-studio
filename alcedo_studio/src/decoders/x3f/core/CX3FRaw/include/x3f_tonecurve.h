/* x3f_tonecurve.h —— 「色调曲线」四段调节 LUT（显示域 u16→u16）。
 *
 * 模型：0..255 显示域按标尺 r2/r3/r4 分成阴影/暗部/亮度/高光四段
 * （默认 64/128/192 = 四等份），每段滑杆 ±2 做幂重映射，段端点定点；
 * 标尺可拖。详见 x3f_tonecurve.c 头注释。
 */
#ifndef X3F_TONECURVE_H
#define X3F_TONECURVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define X3F_TONECURVE_LUT_SIZE 65536

/* 生成「色调曲线」显示域 LUT。
 * 滑杆：sh=阴影 dk=暗部 li=亮度 hi=高光（0=中性，±2 满格，
 * val>0 提亮 / val<0 压暗所在段）；
 * 标尺：r2/r3/r4 ∈ [0,255]（默认 64/128/192，UI 可拖动调整段边界）。
 * 返回 0 = 四滑杆全 0，out 为恒等表（调用方可据此跳过施加）；
 *       1 = out 为有效映射。 */
int x3f_tone_curve_build(double sh, double dk, double li, double hi,
                         double r2, double r3, double r4,
                         uint16_t out[X3F_TONECURVE_LUT_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* X3F_TONECURVE_H */
