/* x3f_mono_grain_tables.h — mono grain calibration table declarations.
 * Table values are materialized at load from packed form (see x3f_packed_assets.c). */
#ifndef X3F_MONO_GRAIN_TABLES_H
#define X3F_MONO_GRAIN_TABLES_H

extern int mono_grain_counts[10];
extern unsigned int mono_grain_cdf[10][43];
extern int mono_grain_amp[10][43];
extern unsigned int mono_grain_t10[10];
extern float mono_grain_ga[10];
extern float mono_grain_gb[10];

#endif /* X3F_MONO_GRAIN_TABLES_H */
