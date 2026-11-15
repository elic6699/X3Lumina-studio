/* x3f_oe_spec.h -- calibration tables; values materialized at load (see x3f_packed_assets.c). */


#ifndef X3F_OE_SPEC_H
#define X3F_OE_SPEC_H

#define OE_SPEC_COMB_N 1025
#define OE_SPEC_COMB_MAX 31744.0

extern double kOeSpecM[4][3];

#define OE_SPEC_TC0_N 130
extern float kOeSpecTc0X[];
extern float kOeSpecTc0Y[];
#define OE_SPEC_TC1_N 130
extern float kOeSpecTc1X[];
extern float kOeSpecTc1Y[];
#define OE_SPEC_TC2_N 130
extern float kOeSpecTc2X[];
extern float kOeSpecTc2Y[];

extern float kOeSpecCombX[OE_SPEC_COMB_N];
extern float kOeSpecCombY[OE_SPEC_COMB_N];
#endif /* X3F_OE_SPEC_H */