/* x3f_oe_spec_dark2.h -- calibration tables; values materialized at load (see x3f_packed_assets.c). */

#ifndef X3F_OE_SPEC_DARK2_H
#define X3F_OE_SPEC_DARK2_H

#define DARK2_NN 33

/* w 域节点（三通道共用；覆盖悬崖区 w∈[10,40]，向下延伸到负 bias 侧） */
extern float kOeSpecDark2Wx[DARK2_NN];

/* Δs per channel（R, G, B），v0 全 0 = 逐位基线 */
extern float kOeSpecDark2Dy[3][DARK2_NN];

#endif /* X3F_OE_SPEC_DARK2_H */
