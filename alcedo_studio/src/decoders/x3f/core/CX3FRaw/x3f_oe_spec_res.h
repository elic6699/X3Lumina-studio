/* x3f_oe_spec_res.h -- calibration tables; values materialized at load (see x3f_packed_assets.c). */


#ifndef X3F_OE_SPEC_RES_H
#define X3F_OE_SPEC_RES_H

#define OE_SPEC_RES_G 24
#define OE_SPEC_RES_N 13824

/* index = (ix*24 + iy)*24 + iz，ix/iy/iz = u 分量（u=R 轴 stride 最大）。 */
extern int16_t kOeSpecRes[OE_SPEC_RES_N][3];

#endif /* X3F_OE_SPEC_RES_H */