/* X3FRAW.C — 进程内解码接口实现 */

#include "x3fraw.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "x3fdec_alias.h"
#include "x3fcore.h"
#include "x3f_io.h"
#include "x3f_process.h"

static int copy_area(const x3f_area16_t *src, x3fraw_image_t *out)
{
    size_t row_bytes = (size_t)out->width * (size_t)out->channels * sizeof(uint16_t);
    size_t total = (size_t)out->row_stride * (size_t)out->height;
    uint32_t row;

    out->data = (uint16_t *)malloc(total * sizeof(uint16_t));
    if (out->data == NULL) return X3FRAW_ERR_NOMEM;

    for (row = 0; row < src->rows; row++)
        memcpy(out->data + (size_t)row * out->row_stride,
               src->data + (size_t)row * src->row_stride, row_bytes);

    return X3FRAW_OK;
}

/* path 必须是"解码器能直接打开的独立 X3F"（X3I 容器已经在外面物化过了） */
static int decode_inner(const char *path, x3fraw_image_t *out, int top_only)
{
    FILE *f;
    x3f_t *x3f;
    x3f_directory_entry_t *DE;
    x3f_return_t ret;
    x3f_area16_t image;
    /* NONE = 已预处理但未做色彩变换（等价于 x3f_extract -color none）
     * 注意不能用 UNPROCESSED：那个会在 Quattro 合并之前提前返回，只得到 1/4 尺寸的底层数据 */
    x3f_color_encoding_t enc = top_only ? QTOP : NONE;
    int rc;

    if (out == NULL) return X3FRAW_ERR_PROCESS;
    memset(out, 0, sizeof(*out));

    f = fopen(path, "rb");
    if (f == NULL) return X3FRAW_ERR_OPEN;

    x3f = x3f_new_from_file(f);
    if (x3f == NULL) { fclose(f); return X3FRAW_ERR_PARSE; }

    /* CAMF 里是标定数据（阴影/坏点/…），处理时要用 */
    x3f_load_data(x3f, x3f_get_camf(x3f));

    DE = x3f_get_raw(x3f);
    if (DE == NULL) { x3f_delete(x3f); fclose(f); return X3FRAW_ERR_NORAW; }

    ret = x3f_load_data(x3f, DE);
    if (ret != X3F_OK) { x3f_delete(x3f); fclose(f); return X3FRAW_ERR_DECODE; }

    out->is_quattro = (DE->header.data_subsection.image_data.quattro != NULL) ? 1 : 0;

    /* crop=1 有效区、fix_bad=1 修坏点、denoise=0 不降噪、
       apply_sgain=-1 自动（Quattro 默认不施加）、wb=NULL 自动 */
    if (!x3f_get_image(x3f, &image, NULL, enc, 1, 1, 0, -1, NULL)) {
        x3f_delete(x3f); fclose(f); return X3FRAW_ERR_PROCESS;
    }

    out->width = (int)image.columns;
    out->height = (int)image.rows;
    out->channels = (int)image.channels;
    out->row_stride = out->width * out->channels;
    snprintf(out->wb, sizeof(out->wb), "%s", top_only ? "top" : "auto");

    rc = copy_area(&image, out);

    free(image.buf);
    x3f_delete(x3f);
    fclose(f);
    return rc;
}

int x3fraw_decode(const char *path, x3fraw_image_t *out)
{
    return x3fraw_decode_frame(path, -1, out);
}

int x3fraw_decode_top(const char *path, x3fraw_image_t *out)
{
    return x3fraw_decode_top_frame(path, -1, out);
}

/* X3I(SFD) 容器：先把选定帧落成一份临时独立 X3F（末尾 4 字节的自指针补好），
 * 再走原来的解码路径。普通 X3F 时 x3f_standalone_path 原样返回 path，零开销。 */
int x3fraw_decode_frame(const char *path, int frame, x3fraw_image_t *out)
{
    char tmp[1024];
    const char *use = x3f_standalone_path(path, frame, tmp, sizeof(tmp));
    int rc = decode_inner(use, out, 0);
    x3f_temp_release(tmp);
    return rc;
}

int x3fraw_decode_top_frame(const char *path, int frame, x3fraw_image_t *out)
{
    char tmp[1024];
    const char *use = x3f_standalone_path(path, frame, tmp, sizeof(tmp));
    int rc = decode_inner(use, out, 1);
    x3f_temp_release(tmp);
    return rc;
}

void x3fraw_free(x3fraw_image_t *img)
{
    if (img == NULL) return;
    free(img->data);
    img->data = NULL;
    img->width = img->height = 0;
}

const char *x3fraw_strerror(int code)
{
    switch (code) {
    case X3FRAW_OK:          return "成功";
    case X3FRAW_ERR_OPEN:    return "打不开文件";
    case X3FRAW_ERR_PARSE:   return "不是 X3F 或解析失败";
    case X3FRAW_ERR_NORAW:   return "没有 RAW 段";
    case X3FRAW_ERR_DECODE:  return "RAW 解码失败";
    case X3FRAW_ERR_PROCESS: return "图像处理失败";
    case X3FRAW_ERR_NOMEM:   return "内存不足";
    default:                 return "未知错误";
    }
}
