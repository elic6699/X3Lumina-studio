/* x3f_oe_hl9corr2.h -- calibration tables; values materialized at load (see x3f_packed_assets.c). */

#define HL9C2_BN_N   34     /* bn 轴：0..1.70 步 0.05（查表值 = bn/0.05） */
#define HL9C2_DN_N   10     /* density 轴：0..1.0 步 0.1 */
#define HL9C2_GN_N    8     /* mG 轴：0.25..1.25 步 0.125 */
#define HL9C2_BN_STEP 0.05
#define HL9C2_DN_STEP 0.1
#define HL9C2_G_LO    0.25
#define HL9C2_G_STEP  0.125
extern float hl9c2_delta[34][10][8][3];
