/* x3f_mac.h -- SPP Stage2 MAC（MultiAxisCC）产品接入接口。
 *
 * 出处与完整口径见 x3f_mac.c 头部大注释；
 * 「MAC 表溯源 IV（终章）」。
 * 独立成文件的缘由：与渲染层其他改动（解码质量会话）并发开发，
 * 集中在一个文件里降低同文件并发写入的覆盖风险。 */
#ifndef X3F_MAC_H
#define X3F_MAC_H

/* load 段入口（NR 之后、CCM 之前的域语义；gate 不过 = 逐位 no-op）：
 *   path          源 X3F（容器帧已在调用方落成独立 X3F）
 *   src           显示域线性 RGB（cam_to_rgb 已乘），就地修改
 *   rgb_to_cam/cam_to_rgb  调用方已有的域变换矩阵
 * 环境变量：X3F_MAC=0 整体关；X3F_MAC_MODE 强制模式名；
 *           X3F_MAC_SCALE / X3F_MAC_GAIN / X3F_MAC_DEBUG 调试。 */
void x3f_mac_stage(const char *path, float *src, unsigned w, unsigned h,
                   const double rgb_to_cam[9], const double cam_to_rgb[9]);

#endif /* X3F_MAC_H */
