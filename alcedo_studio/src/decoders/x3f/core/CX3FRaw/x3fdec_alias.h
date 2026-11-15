/* X3FDEC_ALIAS.H — 把渲染层的 x3f_* 调用改指到分发层
 *
 * 必须在 x3f_io.h / x3f_process.h … 之前 include：这样头文件里的原型
 * 也一起改名，调用点一个字都不用动（x3f_render.c / x3fraw.c 只多这一行）。
 *
 * 只映射有状态的入口；纯数学的三个（3x3/标量乘法、XYZ→sRGB）两边一样，
 * 留给 Rust 那份，不经过分发。
 */

#ifndef X3FDEC_ALIAS_H
#define X3FDEC_ALIAS_H

#define x3f_new_from_file       x3fdec_new_from_file
#define x3f_delete              x3fdec_delete
#define x3f_load_data           x3fdec_load_data
#define x3f_get_raw             x3fdec_get_raw
#define x3f_get_camf            x3fdec_get_camf
#define x3f_get_wb              x3fdec_get_wb
#define x3f_get_camf_float      x3fdec_get_camf_float
#define x3f_get_image           x3fdec_get_image
#define x3f_get_raw_to_xyz      x3fdec_get_raw_to_xyz
#define x3f_get_spatial_gain    x3fdec_get_spatial_gain
#define x3f_cleanup_spatial_gain x3fdec_cleanup_spatial_gain
#define x3f_calc_spatial_gain   x3fdec_calc_spatial_gain

#endif /* X3FDEC_ALIAS_H */
