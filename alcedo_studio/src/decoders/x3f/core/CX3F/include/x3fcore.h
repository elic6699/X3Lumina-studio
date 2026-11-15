/* x3fcore.h — Sigma X3F 解析核心（C，可被 Swift/ObjC 调用）
 *
 * 提供的核心能力：定位并取出 X3F 内嵌的全分辨率 JPEG。
 * 该 JPEG 由相机自身的处理管线生成并用硬件编码器压缩，
 * 因此按字节取出即等于「相机自己那张图」，无需复刻 ISP。
 */

#ifndef X3FCORE_H
#define X3FCORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    X3FJPEG_OK          = 0,
    X3FJPEG_ERR_IO      = -1,
    X3FJPEG_ERR_MAGIC   = -2,
    X3FJPEG_ERR_NOJPEG  = -3,
    X3FJPEG_ERR_FORMAT  = -4,
    X3FJPEG_ERR_NOMEM   = -5,
    X3FJPEG_ERR_BOUNDS  = -6
};

#define X3F_MAX_SECTIONS 8
#define X3F_MAX_FRAMES   8   /* X3I(SFD) 容器固定 7 帧，留一点富余 */

typedef struct {
    char  tag[8];      /* 目录条目名，如 "IMA2" / "CAMF" */
    char  sectag[8];   /* 该偏移处的段头 tag，如 "SECi" / "SECc" */
    long  offset;
    long  length;
    int   is_jpeg;     /* 1 表示这一段就是内嵌 JPEG */
} x3fjpeg_section_t;

/* X3I(SFD) 容器里的一帧。普通 X3F 没有这一层（frame_count = 1）。 */
typedef struct {
    long  offset;       /* 帧在容器里的绝对起点（该处是 FOVb 头） */
    long  length;       /* 帧区间长度：到下一段起点为止 */
    long  secd_rel;     /* 帧内 SECd 目录的相对偏移（补末 4 字节要用） */
    long  jpeg_offset;  /* 内嵌 JPEG 的绝对偏移（SOI） */
    long  jpeg_length;  /* 含 SOI..EOI 的长度 */
    int   ev_ladder;    /* 外层头给的曝光台阶号 1..7（1 = 最欠曝，4 = 0 EV）；0 = 未知 */
} x3fframe_t;

typedef struct {
    char  ident[8];            /* "FOVb" 或 "FOVi" */
    char  serial[16];          /* 机身序列号（8 位 ASCII） */
    int   nominal_width;       /* 主头记录的尺寸 */
    int   nominal_height;
    long  file_size;

    int   has_jpeg;            /* 1 = 成功定位内嵌 JPEG */
    long  jpeg_offset;
    long  jpeg_length;         /* 含 SOI..EOI */
    int   width;               /* JPEG 实际尺寸 */
    int   height;
    int   components;
    int   has_exif;
    int   has_icc;
    int   subsampling;         /* 0=4:4:4 1=4:2:2 2=4:2:0 9=其他 */
    int   quant_luma[64];      /* 量化表，用于判断压缩质量 */
    int   quant_chroma[64];

    int   section_count;
    x3fjpeg_section_t sections[X3F_MAX_SECTIONS];

    /* ---- X3I(SFD) 容器 ---- */
    int   is_container;       /* 1 = 这是 .X3I 容器（FOVi） */
    int   frame_count;        /* 容器 = 7；普通 X3F = 1 */
    int   frame_index;        /* 本次解析取的是第几帧（0 = 0 EV 基准帧） */
    x3fframe_t frames[X3F_MAX_FRAMES];
} x3fjpeg_info_t;

/* 只读少量数据即可完成：文件头 + 末尾目录 + JPEG 头部 */
int x3f_probe(const char *path, x3fjpeg_info_t *out);

/* ---- X3I(SFD) 容器（七帧 SFD）--------------------------------------
 *
 * `.X3I` 是 SFD 容器：一个 FOVi 外层头 + 7 个完整的 X3F 帧 + SPPA 元数据块
 * + SECf 帧表。结构见 outputs/X3I_SFD_容器与支持方案.md。
 *
 * `frame` 的语义：0 = 0 EV 基准帧（相机总把它写在最前面），1..6 = −3/−2/−1/+1/+2/+3 EV，
 * **负数 = 用默认帧**（环境变量 X3F_SFD_FRAME，缺省 0）。普通 X3F 忽略该参数。
 */

/* 容器帧数（普通 X3F 返回 1，读不了返回 0） */
int x3f_frame_count(const char *path);

/* 默认帧索引：环境变量 X3F_SFD_FRAME 优先，缺省 0 */
int x3f_default_frame(const char *path);

/* 带帧索引的探测 / 取图 / 落盘；frame < 0 = 用默认帧 */
int x3f_probe_frame(const char *path, int frame, x3fjpeg_info_t *out);
int x3f_get_jpeg_frame(const char *path, int frame, uint8_t **out_bytes,
                       size_t *out_size, x3fjpeg_info_t *out_info /* 可为 NULL */);
int x3f_extract_to_file_frame(const char *src_path, const char *dst_path, int frame,
                             x3fjpeg_info_t *out_info /* 可为 NULL */);

/* 把容器里的第 frame 帧读成一份**独立的合法 X3F** 字节（末尾 4 字节的目录自指针
 * 已补好）。普通 X3F 返回 X3FJPEG_ERR_FORMAT —— 直接用原路径即可。 */
int x3f_frame_read(const char *path, int frame, uint8_t **out_bytes, size_t *out_size);

/* 「解码器能直接打开的那个路径」：普通 X3F 原样返回 path；容器则把该帧落成临时
 * 文件（临时文件名写进 tmp_out，容量至少 1024）并返回 tmp_out。
 * 用完调 x3f_temp_release(tmp_out) 回收；tmp_out 为空表示没有临时文件。 */
const char *x3f_standalone_path(const char *path, int frame, char *tmp_out, size_t cap);
void x3f_temp_release(const char *tmp_path);

/* 取出内嵌 JPEG（malloc 分配，调用方用 x3f_free 释放） */
int x3f_get_jpeg(const char *path, uint8_t **out_bytes, size_t *out_size,
                 x3fjpeg_info_t *out_info /* 可为 NULL */);

/* 直接把内嵌 JPEG 按字节写到文件（不做任何重新编码） */
int x3f_extract_to_file(const char *src_path, const char *dst_path,
                        x3fjpeg_info_t *out_info /* 可为 NULL */);

/* 读取目录中第 index 段的数据（用于将来处理 RAW/CAMF） */
int x3f_read_section(const char *path, int index, uint8_t **out_bytes,
                     size_t *out_size, x3fjpeg_info_t *out_info /* 可为 NULL */);

/* ============================================================================
 * SPP「保存设置」写进 X3F 的那块参数（`SECs` 段）—— 与 SIGMA Photo Pro 互通
 * ============================================================================
 *
 * 用户在 SPP 里调完参数、点「保存设置」（SPP 会警告"这是写进文件"）之后，文件里会
 * 多出一条 **`SECs` 段**（IMA2 类型，固定 4856 字节）—— 调整参数就存在里面
 * （另有 `SPPA` 段：SPP 顺手存的一张 1440×960 预览 JPEG，我们不解析）。
 *
 * 块的**参数槽位是固定步长**：`u32`，从块内偏移 1016 起、每 8 字节一个；另有几个
 * 散落字段（白平衡档位/细节档位/高光控制/白平衡 R·B 增益）。已实测钉死的槽位见
 * 下面的结构体注释（样本与推导见 ）。
 *
 * **两份重要约定**：
 *   1. 读不到 SECs（SPP 没保存过）时 `x3f_spp_read_settings` 返回 `X3F_SPP_NONE`，
 *      调用方回落到自己的 sidecar；
 *   2. 写入是**整块继承 + 打补丁**：只在已有 SECs 段里改认识的槽位，其余字节一个
 *      都不动（因此除了那几个 4 字节，输出与原文件逐字节相同）。目前**只支持已有
 *      SECs 段的文件**；给原本没有的文件"新增一段"那条路要重排目录，单独做。
 */
typedef struct {
    int has_settings;   /* 1 = 读到了 SECs 段 */
    /* 七根滑杆：面板值 ×10 的整数（+0.3 → 3） */
    int exposure;       /* 槽位 0 @1016 */
    int contrast;       /* 槽位 1 @1024 */
    int shadow;         /* 槽位 2 @1032 */
    int highlight;      /* 槽位 3 @1040 */
    int saturation;     /* 槽位 4 @1048 */
    int sharpness;      /* 槽位 5 @1056 */
    int fill_light;     /* 槽位 6 @1064 */
    int slot7;          /* 槽位 7 @1072 —— 未见动过，原样保留 */
    int chroma_noise;   /* 槽位 8 @1080 —— 面板档 − 1 */
    int luma_noise;     /* 槽位 9 @1088 —— 面板档 − 1 */
    int slot10;         /* 槽位 10 @1096 —— 待定（样本里恒为 2） */
    int slot11;         /* 槽位 11 @1104 —— 待定 */
    int wb_preset;      /* @36  —— 1 自动 / 4 覆盖 / 5 白炽灯（其余待补） */
    int detail;         /* @48  —— 细节档位（样本实测 3…6） */
    int hl_control;     /* @696 —— 高光控制（0/1） */
    int wb_gain_r;      /* @168 —— Q15 定点，32768 = 1.0 */
    int wb_gain_b;      /* @180 —— Q15 定点 */
} x3f_spp_settings_t;

#define X3F_SPP_OK     0
#define X3F_SPP_NONE   1   /* 文件里没有 SECs 段（SPP 没保存过设置） */
#define X3F_SPP_ERR   -1

/* 读：没有 SECs 段返回 X3F_SPP_NONE（out 清零） */
int x3f_spp_read_settings(const char *path, x3f_spp_settings_t *out);

/* 写：把 src 原样复制到 dst、只在 SECs 段里打补丁（认识的槽位）。
 * 没有 SECs 段返回 X3F_SPP_NONE（这种情况不写）。调用方负责备份/临时文件/rename。 */
int x3f_spp_write_settings(const char *src_path, const char *dst_path,
                           const x3f_spp_settings_t *in);

/* `SECs` 段在文件里的位置（字节偏移 / 长度）。给验收用：写完一份新文件之后，
 * 拿去和原文件逐字节比，好证明"变的那些字节都落在这段里、图像数据一个没碰"。
 * 没有 SECs 段返回 X3F_SPP_NONE。 */
int x3f_spp_section(const char *path, uint32_t *out_off, uint32_t *out_len);

void x3f_free(void *p);

/* 返回静态字符串，无需释放 */
const char *x3f_strerror(int code);

#ifdef __cplusplus
}
#endif

#endif /* X3FCORE_H */

