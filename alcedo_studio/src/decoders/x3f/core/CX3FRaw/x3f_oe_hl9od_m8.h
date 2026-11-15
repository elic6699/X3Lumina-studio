/* x3f_oe_hl9od_m8.h — **mode8 基线**输出域 Δ 表（2026-10-29）。
 *
 * 与 x3f_oe_hl9od.h（mode9 基线表）的区别：
 *  · 基线 = mode8 渲染输出（SPP Stage2 直通重放），非 m9 重建链 combined 输出；
 *  · 标定集 = 施加域全体（mask>0，141390 px @::2），修「标定域≠施加域」
 *    的 +43 推亮 bug（mode9 表的教训，见 2026-10-29 日志）。
 * 模拟（fit_od_m8.py，2501 焰核）：亮区子集 4.653→2.431、全窗 5.791→4.768、
 * 全图 3.143→3.104；180-220 过渡 bias −0.31；留出 2.457/2.445。
 * 启用：X3F_HL9_ODM8=1（mode8 链 + 本表；默认关零回归）。
 * 限定：单场景 DP2Q2501，跨图泛化未验证——与全部 mode9 表同等。
 */
#ifndef X3F_OE_HL9OD_M8_H
#define X3F_OE_HL9OD_M8_H
#define HL9OD_M8_BN_N 17
extern float hl9od_m8_delta[17][10][8][3];
#endif /* X3F_OE_HL9OD_M8_H */
