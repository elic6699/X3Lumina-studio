/* X3FRAW.H — 进程内 X3F RAW 解码的简单接口
 *
 * 基于 x3f_tools（BSD-3-Clause，见 COPYRIGHT_x3f_tools.txt）的解析与处理代码，
 * 去掉 OpenCV / libtiff 依赖，可直接用于 macOS 与 iOS。
 */

#ifndef X3FRAW_H
#define X3FRAW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    X3FRAW_OK            =  0,
    X3FRAW_ERR_OPEN      = -1,   /* 打不开文件 */
    X3FRAW_ERR_PARSE     = -2,   /* 不是 X3F / 解析失败 */
    X3FRAW_ERR_NORAW     = -3,   /* 没有 RAW 段 */
    X3FRAW_ERR_DECODE    = -4,   /* RAW 解码失败 */
    X3FRAW_ERR_PROCESS   = -5,   /* 处理失败 */
    X3FRAW_ERR_NOMEM     = -6
};

typedef struct {
    int       width;        /* 处理后的宽（crop 后为有效区） */
    int       height;
    int       channels;     /* 通常为 3 */
    int       row_stride;   /* 以 uint16 计 */
    int       is_quattro;
    uint16_t *data;         /* 线性数据，未做色彩变换（等价 -color none） */
    char      wb[64];       /* 实际使用的白平衡预设名（便于排查） */
} x3fraw_image_t;

/* 解码一张 X3F。成功返回 X3FRAW_OK，data 由 x3fraw_free 释放。 */
int x3fraw_decode(const char *path, x3fraw_image_t *out);

/* 解码但不做 Quattro 合并，只取顶层（-qtop 等价）。 */
int x3fraw_decode_top(const char *path, x3fraw_image_t *out);

/* 同上，但可以指定 X3I(SFD) 容器的帧：frame < 0 = 用默认帧（0 EV 基准帧，
 * 环境变量 X3F_SFD_FRAME 可覆盖）。普通 X3F 忽略该参数。 */
int x3fraw_decode_frame(const char *path, int frame, x3fraw_image_t *out);
int x3fraw_decode_top_frame(const char *path, int frame, x3fraw_image_t *out);

void x3fraw_free(x3fraw_image_t *img);

/* 返回静态字符串，无需释放 */
const char *x3fraw_strerror(int code);

#ifdef __cplusplus
}
#endif

#endif /* X3FRAW_H */

