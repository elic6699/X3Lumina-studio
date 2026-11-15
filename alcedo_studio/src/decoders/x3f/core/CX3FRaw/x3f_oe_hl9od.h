/* x3f_oe_hl9od.h — mode9 **输出域** Δ 表（s 域架构改造，2026-10-29）。
 *
 * 与 hl9c_delta_v8（s 域注入）的区别：本表加在 **combined 之后** 的输出域
 * （16bit，表值单位 = 8bit 级），标定目标直接对 SPP 8bit 真值，无反查、
 * 无 s=16399 天花板、无肩部斜率病态。隔离实验（exp_sarch_isolation.py）：
 * 同 bin 网格输出域 oracle MAE 2.46 vs s 域 3.01。
 *
 * 标定：fit_delta_od.py（单场景 DP2Q2501 焰核，跨图泛化未验证——与全部
 * mode9 表同等限定）。启用：X3F_HL9_ODOMAIN=1（默认关，逐位零回归）。
 * X3F_HL9_NO_DELTA=1 同时跳过两种 Δ 注入。
 */
#ifndef X3F_OE_HL9OD_H
#define X3F_OE_HL9OD_H

#define HL9OD_BN_N 17
#define HL9OD_DN_N 10
#define HL9OD_GN_N 8
extern float hl9od_delta[HL9OD_BN_N][HL9OD_DN_N][HL9OD_GN_N][3];
#endif /* X3F_OE_HL9OD_H */
