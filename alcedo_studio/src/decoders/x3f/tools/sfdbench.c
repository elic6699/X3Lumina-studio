/* sfdbench.c —— SFD（X3I 七帧）合成基准（CMake 目标 sfdbench）
 *
 * 只做一件事：对给定容器跑一遍 x3f_render_load_sfd_ex（与 App 预览/导出同一条
 * 合成链），把合成产物（线性 RGB，w*h*3 个 float）落盘，供优化前后**逐位对拍**。
 * 计时细目在 x3f_sfd.c 里：设 X3F_SFD_TIMING=1 会在 stderr 打各阶段耗时拆账。
 *
 * 用法：
 *   sfdbench [选项] 文件.X3I [out.f32]
 *
 *   -d N      合成降采样倍率（默认 1 = 全分辨率，与 App 预览/导出口径一致）
 *   -f N      最多合成前 N 帧（默认 7，与 App 一致）
 *   -q        不落盘（只看耗时与 note）
 * 退出码：0 成功；1 参数错误；2 合成失败。
 */

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

static void usage(void)
{
    puts("用法: sfdbench [选项] 文件.X3I [out.f32]\n"
         "  -d N    合成降采样倍率（默认 1 = 全分辨率）\n"
         "  -f N    最多合成前 N 帧（默认 7）\n"
         "  -q      不落盘（只计时报 note）\n"
         "计时细目：设 X3F_SFD_TIMING=1（stderr 打各阶段拆账）");
}

int main(int argc, char **argv)
{
    const char *src = NULL, *out = NULL;
    int down = 1, max_frames = 7;
    int i, rc, w = 0, h = 0;
    float *rgb = NULL;
    double norm = 1.0;
    char note[256] = { 0 };

#if defined(_WIN32)
    /* 控制台输出按 UTF-8（源码/诊断串里有中文） */
    SetConsoleOutputCP(CP_UTF8);
#endif

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            down = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            max_frames = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-q") == 0) {
            out = "";
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage();
            return EXIT_OK;
        } else if (argv[i][0] != '-' && src == NULL) {
            src = argv[i];
        } else if (argv[i][0] != '-' && out == NULL) {
            out = argv[i];
        } else {
            usage();
            return EXIT_USAGE;
        }
    }
    if (src == NULL) { usage(); return EXIT_USAGE; }

    rc = x3f_render_load_sfd_ex(src, down, NULL, 0, NULL, max_frames,
                                &rgb, &w, &h, note, (int)sizeof(note), &norm,
                                NULL, NULL, NULL, NULL);
    if (rc != 0 || rgb == NULL) {
        fprintf(stderr, "sfdbench: 合成失败 rc=%d\n", rc);
        return EXIT_FAIL;
    }
    printf("尺寸 %dx%d  norm=%.4f\nnote: %s\n", w, h, norm, note);

    if (out != NULL && out[0] != '\0') {
        FILE *o = fopen(out, "wb");
        if (o == NULL) {
            fprintf(stderr, "sfdbench: 写不出 %s\n", out);
            x3f_render_free(rgb);
            return EXIT_FAIL;
        }
        fwrite(rgb, sizeof(float), (size_t)w * (size_t)h * 3, o);
        fclose(o);
        printf("已写 %s（%.1f MB，float 线性 RGB，供逐位对拍）\n", out,
               (double)w * (double)h * 3.0 * 4.0 / 1048576.0);
    }
    x3f_render_free(rgb);
    return EXIT_OK;
}
