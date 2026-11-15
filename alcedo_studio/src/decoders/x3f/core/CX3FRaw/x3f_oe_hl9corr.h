/* x3f_oe_hl9corr.h -- calibration tables; values materialized at load (see x3f_packed_assets.c). */

#define HL9C_BN_N   17      /* bn 轴：0..1.75 步 0.1（查表值 = bn/0.1） */
#define HL9C_DN_N   10      /* density 轴：0..1.0 步 0.1 */
#define HL9C_GN_N    8      /* mG 轴：0.25..1.25 步 0.125 */
#define HL9C_BN_STEP 0.1
#define HL9C_DN_STEP 0.1
#define HL9C_G_LO    0.25
#define HL9C_G_STEP  0.125
extern float hl9c_delta[17][10][8][3];
