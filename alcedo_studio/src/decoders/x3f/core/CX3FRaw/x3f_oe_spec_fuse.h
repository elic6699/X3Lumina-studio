/* x3f_oe_spec_fuse.h -- calibration tables; values materialized at load (see x3f_packed_assets.c). */


#ifndef X3F_OE_SPEC_FUSE_H
#define X3F_OE_SPEC_FUSE_H

#include "x3f_oe_spec.h"   /* combined(kOeSpecCombX/Y)：SPP 全局固定，两路共用 */

#define OE_SPEC_FUSE_RES_SCALE 16384.0f
#define OE_SPEC_FUSE_RES_G 24
#define OE_SPEC_FUSE_RES_N 13824
extern double kOeSpecFuseM[4][3];
#define OE_SPEC_FUSE_TC0_N 128
extern float kOeSpecFuseTc0X[128];
extern float kOeSpecFuseTc0Y[128];
#define OE_SPEC_FUSE_TC1_N 128
extern float kOeSpecFuseTc1X[128];
extern float kOeSpecFuseTc1Y[128];
#define OE_SPEC_FUSE_TC2_N 128
extern float kOeSpecFuseTc2X[128];
extern float kOeSpecFuseTc2Y[128];
#endif /* X3F_OE_SPEC_FUSE_H */
