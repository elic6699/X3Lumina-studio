/* x3f_rcps.h -- SPP RowColumnPatternSuppression 复刻接口。
 *
 * 出处与完整口径见 x3f_rcps.c 头部大注释；行为规范来自
 * §1与
 * §27.3（产品化接入点拍板）。
 * 独立成文件的缘由：与渲染层其他改动（解码质量会话）并发开发，
 * 集中在一个文件里降低同文件并发写入的覆盖风险。 */
#ifndef X3F_RCPS_H
#define X3F_RCPS_H

/* Merrill 行/列 FPN 随机置换抑制（SPP RCPS 同构）。
 *
 *   lin  输入解码 lin 域（f32 interleaved W×H×3，apply_ex 上游），只读
 *   w/h  图像尺寸
 *   返回 X3F_RCPS=1 时为新分配的 W×H×3 f32 缓冲（调用方 free）；
 *        未启用 / 参数错误时返回 NULL（调用方回退用原指针）。
 *
 * 门控语义（2026-10-27 机型研究后定案）：
 *   X3F_RCPS=1        强制开（任意机型，实验用）
 *   X3F_RCPS=0        强制关
 *   未设              AUTO = 仅当 x3f_rcps_set_auto(1) 已置（load 段按
 *                     文件头 type_format==MERRILL 探测）才开 —— 与 SPP
 * 对齐： DenoiseF20 的 RCPS 块门控是三个 RCPS
 *                     参数 >0（默认 0.075/0.1/1.0 全过），即 SPP 默认
 *                     就跑，机型限定由「只有 Merrill 走 F20 引擎」天然
 *                     达成。 */
/* ---- lin 域入口（SFD/诊断用；显示域条纹与 lin 基本无关，见 §30） ----
 *
 *   lin  输入解码 lin 域（f32 interleaved W×H×3），只读
 *   返回 X3F_RCPS=1 时为新分配的 W×H×3 f32 缓冲（调用方 free）；
 *        未启用 / 参数错误时返回 NULL（调用方回退用原指针）。
 */
float *x3f_rcps_apply(const float *lin, int w, int h);

/* ---- 显示域入口（产品正道，§30 定案）----
 *
 *   img   tone 后、unsharp 前的 sRGB 显示缓冲（interleaved RGB，bits=8/16）
 *   bits  每通道位深（8 → unsigned char，16 → unsigned short）
 *   就地修改（门控/交换/保险丝三层，见 x3f_rcps.c 头注释）。
 *   显示域列纹与 lin 列纹基本无关（corr 0.06-0.21，§27.1 实测「±4 竖纹
 *   主体是 apply_decode_unsharp 制造的」），故接入点必须在 unsharp 之前、
 *   tone 之后的显示缓冲上，打破色度列纹让 unsharp 无纹可放。
 */
void x3f_rcps_apply_disp(void *img, int bits, int w, int h);

/* load 段按文件设置的 AUTO 门（x3f_render.c load_common 调用；
 * Merrill=1 / 其它=0）。未调用时 AUTO 关 = 行为与旧版默认关一致。 */
void x3f_rcps_set_auto(int active);

#endif /* X3F_RCPS_H */
