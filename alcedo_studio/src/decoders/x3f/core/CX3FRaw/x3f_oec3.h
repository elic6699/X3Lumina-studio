/* x3f_oec3.h — OECVer3 逐位复刻高光回注（mode9 专用，X3F_OEC3=1 门控）。
 *
 * 
 * 逐位底稿：同目录 oec_rep.c（vs dylib 全图 16,215,552 floats 0 diff，
 * commit e39e2a5）。本文件 = oec_rep.c 核心 + main16/thumb 构造 + half 场折回。
 */
#ifndef X3F_OEC3_H
#define X3F_OEC3_H

/* lin  : 全分辨率线性域（own 解码，post-WB），w×h×3 float
 * est  : 高光展开世界（hl9buf），作 thumb 增益源
 * 返回 : 新分配 w×h×3（失败返回 NULL，调用方回落 v8b 行为） */
float *x3f_oec3_envelope(const float *lin, const float *est, int w, int h);

#endif /* X3F_OEC3_H */
