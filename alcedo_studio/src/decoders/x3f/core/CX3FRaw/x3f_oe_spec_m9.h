/* x3f_oe_spec_m9.h -- calibration tables; values materialized at load (see x3f_packed_assets.c). */


#ifndef X3F_OE_SPEC_M9_H
#define X3F_OE_SPEC_M9_H

#include "x3f_oe_spec.h"   /* combined(kOeSpecCombX/Y)：SPP 全局固定，两路共用 */

#define OE_SPEC_M9_RES_SCALE 31744.0f
#define OE_SPEC_M9_RES_G 24
#define OE_SPEC_M9_RES_N 13824
extern double kOeSpecM9M[4][3];
#define OE_SPEC_M9_TC0_N 142
extern float kOeSpecM9Tc0X[142];
extern float kOeSpecM9Tc0Y[142];
#define OE_SPEC_M9_TC1_N 142
extern float kOeSpecM9Tc1X[142];
extern float kOeSpecM9Tc1Y[142];
#define OE_SPEC_M9_TC2_N 142
extern float kOeSpecM9Tc2X[142];
extern float kOeSpecM9Tc2Y[142];
#endif /* X3F_OE_SPEC_M9_H */
