/* x3fprobe.c —— 核心自检命令行（CMake 目标 x3fprobe）
 *
 * 它只做一件事：把三层都真跑一遍，并把结果落成磁盘上的文件，好让"这套东西在
 * 这台机器上到底能不能用"当场有答案，而不是靠肉眼看界面推断。三层各验一件事：
 *
 *   CX3F     探文件头 / 目录 → 打印机型、序列号、内嵌 JPEG 的位置与尺寸
 *   CX3F+    把内嵌 JPEG **按字节**抽出来（不重编码，所以逐字节等于相机那张图）
 *   CX3FRaw  从 RAW 解码 + 渲染 → 走 SPP 那套默认参数，写出 PPM（8 位或 16 位）
 *
 * 图像写成 PPM（P6）而不是 PNG/JPEG：PPM 不需要任何第三方库，Windows 上
 * 不引入 zlib/libpng 这些依赖就能先确认管线通不通。要看图用别的工具转一下即可。
 *
 * 用法：
 *   x3fprobe [选项] 文件.X3F [更多文件...]
 *
 *   -o DIR    输出目录（默认当前目录）
 *   -d N      渲染时按 N×N 盒式降采样（默认 4；1 = 全分辨率，很慢、很占内存）
 *   -j        只抽内嵌 JPEG，不渲染
 *   -r        只渲染，不抽内嵌 JPEG
 *   -16       渲染输出 16 位 PPM（走 x3f_render_apply_ex16，与 App 的 16 位 TIFF 同一条链）
 *   --no-base 关掉"相机观感"基础曲线（等于 CLI 的 --no-base，用来做对照）
 *   -h        这份用法
 *
 * 退出码：0 全部成功；1 参数错误；2 至少一个文件失败。
 */

#include "x3fcore.h"
#include "x3f_render.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#endif

enum { EXIT_OK = 0, EXIT_USAGE = 1, EXIT_FAIL = 2 };

typedef struct {
    const char *outdir;
    int         down;
    int         want_jpeg;
    int         want_render;
    int         sixteen;
    int         base_curve;
} probe_opts_t;

static void usage(void)
{
    puts("用法: x3fprobe [选项] 文件.X3F [更多文件...]\n"
         "  -o DIR    输出目录（默认当前目录）\n"
         "  -d N      渲染降采样倍率，1 = 全分辨率（默认 4）\n"
         "  -j        只抽内嵌 JPEG\n"
         "  -r        只渲染（不抽内嵌 JPEG）\n"
         "  -16       渲染输出 16 位 PPM\n"
         "  --no-base 关掉基础曲线（做对照用）\n"
         "  -h        这份用法");
}

/* 取路径的最后一段（两种分隔符都认：Windows 上用户会拖进来 \ 的路径）。 */
static const char *base_name(const char *path)
{
    const char *slash = strrchr(path, '/');
    const char *bslash = strrchr(path, '\\');
    const char *p = path;

    if (slash) p = slash + 1;
    if (bslash && bslash + 1 > p) p = bslash + 1;
    return p;
}

/* 去目录、去扩展名。用来拼输出文件名。 */
static void stem_of(const char *path, char *out, size_t cap)
{
    const char *name = base_name(path);
    size_t n = 0;

    while (name[n] && name[n] != '.' && n + 1 < cap) {
        out[n] = name[n];
        n++;
    }
    out[n] = 0;
    if (n == 0) snprintf(out, cap, "out");
}

static void join_path(char *out, size_t cap, const char *dir, const char *name)
{
    size_t len = strlen(dir);

    if (len > 0 && (dir[len - 1] == '/' || dir[len - 1] == '\\'))
        snprintf(out, cap, "%s%s", dir, name);
    else
        snprintf(out, cap, "%s/%s", dir, name);   /* Windows 也认 '/' */
}

static int write_ppm8(const char *path, const uint8_t *px, int w, int h)
{
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    fwrite(px, 1, (size_t)w * (size_t)h * 3u, f);
    fclose(f);
    return 0;
}

/* 16 位 PPM：每通道两个字节、**大端**（PPM 规定）。 */
static int write_ppm16(const char *path, const uint16_t *px, int w, int h)
{
    size_t n = (size_t)w * (size_t)h * 3u;
    uint8_t *buf = (uint8_t *)malloc(n * 2);
    FILE *f;
    size_t i;

    if (!buf) return -1;
    for (i = 0; i < n; i++) {
        buf[i * 2]     = (uint8_t)(px[i] >> 8);
        buf[i * 2 + 1] = (uint8_t)(px[i] & 0xff);
    }
    f = fopen(path, "wb");
    if (!f) { free(buf); return -1; }
    fprintf(f, "P6\n%d %d\n65535\n", w, h);
    fwrite(buf, 1, n * 2, f);
    fclose(f);
    free(buf);
    return 0;
}

static int probe_one(const char *file, const probe_opts_t *o)
{
    x3fjpeg_info_t info;
    char stem[256], name[600], dst[1024];
    float *rgb = NULL;
    int w = 0, h = 0, rc, bad = 0;
    char wb[64];

    memset(&info, 0, sizeof info);
    memset(wb, 0, sizeof wb);
    stem_of(file, stem, sizeof stem);

    printf("文件      : %s\n", file);

    /* ---- 1) 容器解析 ---- */
    rc = x3f_probe(file, &info);
    if (rc != X3FJPEG_OK) {
        printf("  探针失败: %s (%d)\n\n", x3f_strerror(rc), rc);
        return 1;
    }
    printf("  标识 %s · 序列号 %s · 主头 %dx%d · 文件 %ld 字节\n",
           info.ident, info.serial, info.nominal_width, info.nominal_height,
           (long)info.file_size);
    printf("  机型 %s · 拍摄 ISO %.0f · 解码后端 %s\n",
           x3f_render_probe_is_merrill(file) ? "Merrill" : "Quattro(或经典 Foveon)",
           x3f_render_shooting_iso(file), x3f_render_decoder_name());

    if (info.has_jpeg) {
        printf("  内嵌 JPEG %ld 字节 @ %ld · %dx%d · %d 分量 · 子采样 %d "
               "· EXIF %d · ICC %d\n",
               (long)info.jpeg_length, (long)info.jpeg_offset,
               info.width, info.height, info.components, info.subsampling,
               info.has_exif, info.has_icc);
    } else {
        printf("  内嵌 JPEG：没找到\n");
    }
    if (info.section_count > 0) {
        int i;
        printf("  目录段 %d:", info.section_count);
        for (i = 0; i < info.section_count; i++)
            printf(" %s(%ld)", info.sections[i].tag, (long)info.sections[i].length);
        printf("\n");
    }

    /* ---- 2) 抽内嵌 JPEG（字节级，不重编码）---- */
    if (o->want_jpeg) {
        if (!info.has_jpeg) {
            printf("  抽 JPEG  : 跳过（文件里没有）\n");
        } else {
            snprintf(name, sizeof name, "%s.jpg", stem);
            join_path(dst, sizeof dst, o->outdir, name);
            rc = x3f_extract_to_file(file, dst, NULL);
            if (rc != X3FJPEG_OK) {
                printf("  抽 JPEG  : 失败 %s (%d)\n", x3f_strerror(rc), rc);
                bad = 1;
            } else {
                printf("  抽 JPEG  : %s\n", dst);
            }
        }
    }

    /* ---- 3) RAW 解码 + 渲染 ---- */
    if (o->want_render) {
        x3f_edit_params_t p;

        x3f_render_set_base_curve(o->base_curve);
        rc = x3f_render_load_ex(file, o->down, &rgb, &w, &h, wb);
        if (rc != X3FRENDER_OK) {
            printf("  渲染     : 解码失败 %d（见 x3f_render.h 的错误码）\n", rc);
            return 1;
        }
        printf("  渲染     : %dx%d（降采样 1/%d）· 白平衡 %s\n",
               w, h, o->down, wb[0] ? wb : "(文件里记录的)");

        x3f_edit_defaults(&p);
        if (o->sixteen) {
            size_t stride = (size_t)w * 3u;
            uint16_t *out = (uint16_t *)calloc((size_t)h * stride, sizeof(uint16_t));
            if (!out) { printf("  渲染     : 内存不够\n"); x3f_render_free(rgb); return 1; }
            rc = x3f_render_apply_ex16(&p, rgb, NULL, w, h, out, (int)(stride * 2));
            if (rc != X3FRENDER_OK) {
                printf("  渲染     : apply 失败 %d\n", rc);
                bad = 1;
            } else {
                snprintf(name, sizeof name, "%s.16.ppm", stem);
                join_path(dst, sizeof dst, o->outdir, name);
                bad |= write_ppm16(dst, out, w, h) != 0;
                printf("  写出     : %s（16 位 PPM）\n", dst);
            }
            free(out);
        } else {
            size_t stride = (size_t)w * 3u;
            uint8_t *out = (uint8_t *)calloc((size_t)h * stride, 1);
            if (!out) { printf("  渲染     : 内存不够\n"); x3f_render_free(rgb); return 1; }
            rc = x3f_render_apply(&p, rgb, w, h, out, (int)stride);
            if (rc != X3FRENDER_OK) {
                printf("  渲染     : apply 失败 %d\n", rc);
                bad = 1;
            } else {
                snprintf(name, sizeof name, "%s.ppm", stem);
                join_path(dst, sizeof dst, o->outdir, name);
                bad |= write_ppm8(dst, out, w, h) != 0;
                printf("  写出     : %s（8 位 PPM）\n", dst);
            }
            free(out);
        }
        x3f_render_free(rgb);
    }

    printf("\n");
    return bad;
}

int main(int argc, char **argv)
{
    probe_opts_t o;
    int i, files = 0, failed = 0;
    char gpu[128];

#if defined(_WIN32)
    /* 源文件是 UTF-8（编译时靠 /utf-8），所以字符串常量在运行时也是 UTF-8
     * 字节；而中文控制台的默认输出代码页是 936，直接打印会变成乱码。把控制台
     * 输出代码页也切到 UTF-8，自检信息里的中文才可读。重定向到文件/管道时
     * 字节不变，仍是 UTF-8。 */
    SetConsoleOutputCP(CP_UTF8);
#endif

    o.outdir = ".";
    o.down = 4;
    o.want_jpeg = 1;
    o.want_render = 1;
    o.sixteen = 0;
    o.base_curve = 1;

    for (i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
            usage();
            return EXIT_OK;
        } else if (strcmp(a, "-o") == 0 && i + 1 < argc) {
            o.outdir = argv[++i];
        } else if (strcmp(a, "-d") == 0 && i + 1 < argc) {
            o.down = atoi(argv[++i]);
            if (o.down < 1) o.down = 1;
        } else if (strcmp(a, "-j") == 0) {
            o.want_render = 0;
        } else if (strcmp(a, "-r") == 0) {
            o.want_jpeg = 0;
        } else if (strcmp(a, "-16") == 0) {
            o.sixteen = 1;
        } else if (strcmp(a, "--no-base") == 0) {
            o.base_curve = 0;
        } else if (a[0] == '-' && a[1] != 0) {
            fprintf(stderr, "不认识的选项：%s\n", a);
            usage();
            return EXIT_USAGE;
        } else {
            files++;
            failed += probe_one(a, &o) != 0;
        }
    }

    if (files == 0) {
        usage();
        return EXIT_USAGE;
    }

    gpu[0] = 0;
    printf("GPU 降噪 : %s\n",
           x3f_render_gpu_available(gpu, (int)sizeof gpu)
               ? (gpu[0] ? gpu : "可用")
               : "拿不到适配器（会自动退回 CPU）");
    printf("结论     : %d 个文件，%d 个失败\n", files, failed);

    return failed ? EXIT_FAIL : EXIT_OK;
}
