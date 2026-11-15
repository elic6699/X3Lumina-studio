/* x3fcore.c — X3F 解析核心实现 */

#include "x3fcore.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <io.h>       /* _close、_mktemp（MSVC 没有 unistd.h / mkstemp） */
#else
#include <unistd.h>   /* close（x3f_standalone_path 里 fdopen 失败的兜底） */
#endif

/* ---------- 小工具 ---------- */

static uint32_t rd32(const uint8_t *b, size_t o) {
    return (uint32_t)b[o] | ((uint32_t)b[o + 1] << 8) |
           ((uint32_t)b[o + 2] << 16) | ((uint32_t)b[o + 3] << 24);
}

static void copy_tag(char *dst, const uint8_t *src, size_t n) {
    size_t i = 0;
    for (; i < n && i < 7; i++) {
        uint8_t c = src[i];
        dst[i] = (c >= 32 && c < 127) ? (char)c : 0;
        if (!dst[i]) break;
    }
    dst[i] = 0;
}

static long find_bytes(const uint8_t *b, size_t n, const char *s,
                       size_t from, size_t to) {
    size_t m = strlen(s);
    if (to > n) to = n;
    if (m == 0 || from >= to || to - from < m) return -1;
    for (size_t i = from; i + m <= to; i++)
        if (b[i] == (uint8_t)s[0] && memcmp(b + i, s, m) == 0) return (long)i;
    return -1;
}

/* 从 SOI 走 JPEG 段结构直到 EOI，返回结束位置（不含），失败返回 -1 */
static long jpeg_end(const uint8_t *b, size_t n, size_t start) {
    size_t i = start + 2;
    while (i + 1 < n) {
        if (b[i] != 0xFF) { i++; continue; }
        uint8_t m = b[i + 1];
        if (m == 0xFF || m == 0x00 || m == 0x01 || m == 0xD8) { i += 2; continue; }
        if (m >= 0xD0 && m <= 0xD7) { i += 2; continue; }
        if (m == 0xD9) return (long)(i + 2);
        /* 合法 marker 才允许继续走段结构（C0-CF SOF/DHT、DA-DF SOS/DQT/DRI、
         * E0-EF APPn、FE COM；F0-FD 是保留值，真编码器不会写）。
         * RAW 熵编码数据里凑巧出现的假 SOI（FF D8 FF xx）靠这一步拦下——
         * 不拦的话会把 RAW 字节当 JPEG 段"走"几 MB，直到某个偶然的 FFD9。 */
        if (!((m >= 0xC0 && m <= 0xCF) || (m >= 0xDA && m <= 0xEF) || m == 0xFE))
            return -1;
        if (i + 4 > n) return -1;
        size_t len = ((size_t)b[i + 2] << 8) | b[i + 3];
        if (m == 0xDA) {
            size_t j = i + 2 + len;
            while (j + 1 < n) {
                if (b[j] == 0xFF) {
                    uint8_t nx = b[j + 1];
                    if (nx == 0xD9) return (long)(j + 2);
                    if (nx == 0x00 || (nx >= 0xD0 && nx <= 0xD7)) { j += 2; continue; }
                }
                j++;
            }
            return -1;
        }
        i += 2 + len;
    }
    return -1;
}

/* 解析 JPEG 头部：尺寸、分量、子采样、量化表 */
static void parse_jpeg_head(const uint8_t *b, size_t n, size_t start,
                            size_t limit, x3fjpeg_info_t *o) {
    o->subsampling = 9;
    size_t i = start + 2;
    while (i + 3 < n && i < limit) {
        if (b[i] != 0xFF) { i++; continue; }
        uint8_t m = b[i + 1];
        if (m == 0xFF || m == 0x00) { i++; continue; }
        if (m == 0xD8 || m == 0x01 || (m >= 0xD0 && m <= 0xD7)) { i += 2; continue; }
        if (m == 0xD9) return;
        size_t len = ((size_t)b[i + 2] << 8) | b[i + 3];
        size_t s = i + 4, e = i + 2 + len;
        if (e > n) return;

        if (m == 0xE1) {
            if (e - s >= 6 && memcmp(b + s, "Exif\0\0", 6) == 0) o->has_exif = 1;
        } else if (m == 0xE2) {
            if (e - s >= 11 && memcmp(b + s, "ICC_PROFILE", 11) == 0) o->has_icc = 1;
        } else if (m == 0xDB) {
            size_t k = s;
            while (k < e) {
                int pq = b[k] >> 4, tq = b[k] & 0x0F;
                k++;
                int cnt = 64 * (pq == 0 ? 1 : 2);
                if (k + (size_t)cnt > e) break;
                for (int x = 0; x < 64; x++)
                    (tq == 0 ? o->quant_luma : o->quant_chroma)[x] =
                        pq == 0 ? b[k + x] : ((int)b[k + x * 2] << 8 | b[k + x * 2 + 1]);
                k += (size_t)cnt;
            }
        } else if (m == 0xC0 || m == 0xC1 || m == 0xC2) {
            if (e - s >= 6) {
                o->height = (b[s + 1] << 8) | b[s + 2];
                o->width  = (b[s + 3] << 8) | b[s + 4];
                o->components = b[s + 5];
                if (o->components >= 3 && e - s >= 11) {
                    int h1 = b[s + 7] >> 4, v1 = b[s + 7] & 15;
                    int h2 = b[s + 10] >> 4, v2 = b[s + 10] & 15;
                    if (h1 == 1 && v1 == 1) o->subsampling = 0;
                    else if (h1 == 2 && v1 == 1 && h2 == 1 && v2 == 1) o->subsampling = 1;
                    else if (h1 == 2 && v1 == 2 && h2 == 1 && v2 == 1) o->subsampling = 2;
                    else o->subsampling = 9;
                }
            }
        } else if (m == 0xDA) {
            return;
        }
        i += 2 + len;
    }
}

/* ---------- 文件读取 ---------- */

static int read_range(FILE *f, long off, size_t len, uint8_t *buf) {
    if (off < 0) return X3FJPEG_ERR_BOUNDS;
    if (fseek(f, off, SEEK_SET) != 0) return X3FJPEG_ERR_IO;
    if (len == 0) return X3FJPEG_OK;
    if (fread(buf, 1, len, f) != len) return X3FJPEG_ERR_IO;
    return X3FJPEG_OK;
}

/* X3I(SFD) 容器：实现在下面，probe_structure 见到 FOVi 就转过去 */
static int probe_container(FILE *f, long fsize, x3fjpeg_info_t *o);
static int fill_frame(FILE *f, x3fjpeg_info_t *o, int frame);

/* 解析主头 + 末尾目录，填充除 JPEG 细节外的信息 */
static int probe_structure(FILE *f, long fsize, x3fjpeg_info_t *o) {
    uint8_t head[64];
    if (read_range(f, 0, sizeof(head), head) != X3FJPEG_OK) return X3FJPEG_ERR_IO;
    if (memcmp(head, "FOVb", 4) != 0 && memcmp(head, "FOVi", 4) != 0)
        return X3FJPEG_ERR_MAGIC;
    memcpy(o->ident, head, 4);
    o->ident[4] = 0;
    memcpy(o->serial, head + 0x14, 8);
    o->serial[8] = 0;
    /* 主头记录的标称尺寸，位置跟头版式走（版式分界与 Rust x3f-core 的
     * resolve_sgain 同一口径：X3F_VERSION_4_0 = 0x00040000，<4.0 = pre-Quattro）：
     *   Quattro（4.x，如 DP0Q/DP1Q/DP2Q）：0x28/0x2C；
     *   pre-Quattro（3.x Merrill，如 SD1M/DP1M/DP2M）：0x1C/0x20 —— 0x28 起是
     *   白平衡描述名（"Sunlight"），按 0x28 读会拿到 ASCII 垃圾（实测）。
     * X3I 容器的外层 FOVi 头 version 字段是 10000（不遵循 4.x 编号）但版式
     * 跟 Quattro 一致（实测 0x28/0x2C = 标称尺寸），所以 FOVi 恒走 0x28。 */
    uint32_t ver = rd32(head, 0x04);
    int dim_off = (memcmp(head, "FOVi", 4) == 0 || ver >= 0x00040000u) ? 0x28 : 0x1c;
    o->nominal_width  = (int)rd32(head, dim_off);
    o->nominal_height = (int)rd32(head, dim_off + 4);
    o->file_size = fsize;

    /* .X3I：SFD 七帧容器。帧表在尾部（SECf），与普通 X3F 的 SECd 是另一套 */
    if (memcmp(head, "FOVi", 4) == 0) return probe_container(f, fsize, o);
    o->frame_count = 1;

    long tail_len = fsize < 4096 ? fsize : 4096;
    uint8_t *tail = malloc((size_t)tail_len);
    if (!tail) return X3FJPEG_ERR_NOMEM;
    if (read_range(f, fsize - tail_len, (size_t)tail_len, tail) != X3FJPEG_OK) {
        free(tail);
        return X3FJPEG_ERR_IO;
    }
    long secd = find_bytes(tail, (size_t)tail_len, "SECd", 0, (size_t)tail_len);
    if (secd >= 0 && secd + 20 <= tail_len) {
        uint32_t count    = rd32(tail, secd + 8);
        uint32_t firstoff = rd32(tail, secd + 12);
        uint32_t firstlen = rd32(tail, secd + 16);
        if ((long)count > X3F_MAX_SECTIONS) count = X3F_MAX_SECTIONS;

        /* 第 0 段：内嵌 JPEG（由 SECd 头的首段偏移/长度给出） */
        uint8_t tag[4];
        if (firstoff + 4 <= (uint32_t)fsize &&
            read_range(f, firstoff, 4, tag) == X3FJPEG_OK) {
            x3fjpeg_section_t *s = &o->sections[o->section_count++];
            memset(s, 0, sizeof(*s));
            strcpy(s->tag, "JPEG");
            copy_tag(s->sectag, tag, 4);
            s->offset = firstoff;
            s->length = firstlen;
            s->is_jpeg = 1;
            o->jpeg_offset = (long)firstoff + 28;   /* 段头 28 字节 */
            o->jpeg_length = (long)firstlen - 28;
        }

        /* 其余段：目录条目 (tag, offset, length) */
        for (uint32_t k = 0; k < count; k++) {
            size_t p = (size_t)secd + 20 + k * 12;
            if (p + 12 > (size_t)tail_len) break;
            if (o->section_count >= X3F_MAX_SECTIONS) break;
            uint32_t off = rd32(tail, p + 4);
            uint32_t len = rd32(tail, p + 8);
            x3fjpeg_section_t *s = &o->sections[o->section_count++];
            memset(s, 0, sizeof(*s));
            copy_tag(s->tag, tail + p, 4);
            s->offset = off;
            s->length = len;
            if (off + 4 <= (uint32_t)fsize) {
                uint8_t st[4];
                if (read_range(f, off, 4, st) == X3FJPEG_OK) copy_tag(s->sectag, st, 4);
            }
        }
    }
    free(tail);
    return X3FJPEG_OK;
}

/* 段尾可能有对齐填充，用段尾窗口内的最后一个 FFD9 精确收敛 JPEG 长度 */
static int refine_jpeg_end(FILE *f, long fsize, x3fjpeg_info_t *o) {
    long sec_start = 0, sec_len = 0;
    for (int i = 0; i < o->section_count; i++)
        if (o->sections[i].is_jpeg) { sec_start = o->sections[i].offset;
                                      sec_len = o->sections[i].length; break; }
    if (sec_len <= 0 || sec_start + sec_len > fsize) return X3FJPEG_ERR_BOUNDS;
    long win = sec_len < 4096 ? sec_len : 4096;
    uint8_t *tb = malloc((size_t)win);
    if (!tb) return X3FJPEG_ERR_NOMEM;
    if (read_range(f, sec_start + sec_len - win, (size_t)win, tb) != X3FJPEG_OK) {
        free(tb);
        return X3FJPEG_ERR_IO;
    }
    long pos = -1;
    for (long i = win - 2; i >= 0; i--)
        if (tb[i] == 0xFF && tb[i + 1] == 0xD9) { pos = i; break; }
    free(tb);
    if (pos < 0) return X3FJPEG_ERR_FORMAT;
    long end = sec_start + sec_len - win + pos + 2;
    long jstart = o->jpeg_offset;
    if (end <= jstart) return X3FJPEG_ERR_FORMAT;
    o->jpeg_length = end - jstart;
    return X3FJPEG_OK;
}

/* 在文件里定位 JPEG：优先用目录给出的段，失败则扫描头部区域 */
static int locate_jpeg(FILE *f, long fsize, x3fjpeg_info_t *o) {
    /* 1) 目录路径：只需读段头附近 1MB 解析尺寸/量化表 */
    if (o->jpeg_offset > 0 && o->jpeg_length > 0 &&
        o->jpeg_offset + 32 < fsize) {
        size_t probe_len = (size_t)(o->jpeg_length < (1L << 20)
                                    ? o->jpeg_length : (1L << 20));
        uint8_t *buf = malloc(probe_len);
        if (buf) {
            int rc = read_range(f, o->jpeg_offset, probe_len, buf);
            if (rc == X3FJPEG_OK && buf[0] == 0xFF && buf[1] == 0xD8) {
                parse_jpeg_head(buf, probe_len, 0, probe_len, o);
                free(buf);
                refine_jpeg_end(f, fsize, o);
                o->has_jpeg = 1;
                return X3FJPEG_OK;
            }
            free(buf);
        }
    }

    /* 2) 回退：扫描前 64MB 找 SOI（.X3I 等缺少目录的文件） */
    size_t scan = (size_t)(fsize < (64L << 20) ? fsize : (64L << 20));
    uint8_t *buf = malloc(scan);
    if (!buf) return X3FJPEG_ERR_NOMEM;
    if (read_range(f, 0, scan, buf) != X3FJPEG_OK) { free(buf); return X3FJPEG_ERR_IO; }
    size_t from = scan > 0x100 ? 0x100 : 0;
    for (size_t i = from; i + 3 < scan; i++) {
        if (buf[i] == 0xFF && buf[i + 1] == 0xD8 && buf[i + 2] == 0xFF) {
            /* SOI 后第一个 marker 必须合法（APPn/SOF/DQT/COM 等）。
             * SDIM7843 这类"JPEG 排在 RAW 之后"的老机身，RAW 熵编码流里
             * 会凑巧出现 FF D8 FF FA/AA/1B —— 假 SOI 先于真 JPEG 被扫到，
             * 缩略图就从 RAW 中段抽、ImageIO 解不出来，列表整格空白。 */
            uint8_t m0 = buf[i + 3];
            if (!((m0 >= 0xC0 && m0 <= 0xCF) || (m0 >= 0xDA && m0 <= 0xEF) ||
                  m0 == 0xFE))
                continue;
            long e = jpeg_end(buf, scan, i);
            if (e > 0 && e - (long)i > 4096) {
                o->jpeg_offset = (long)i;
                o->jpeg_length = e - (long)i;
                parse_jpeg_head(buf, scan, i, scan, o);
                o->has_jpeg = 1;
                free(buf);
                return X3FJPEG_OK;
            }
        }
    }
    free(buf);
    return X3FJPEG_ERR_NOJPEG;
}

/* ---------- X3I(SFD) 容器 ----------
 *
 * `.X3I` = FOVi 外层头(1440B) + 7 个完整 X3F 帧 + SPPA 元数据块 + SECf 帧表(112B)。
 * 帧表自带每帧的 (offset, length)，不用扫描；每帧内部与独立 X3F 同构，
 * 唯一差别是**文件末 4 字节的目录自指针**（容器里帧 1..6 是 0，解码器正是靠
 * 它找 SECd，所以要补）。详见 outputs/X3I_SFD_容器与支持方案.md。
 */

/* 帧内 SECd 目录的校验。
 *
 * SPP / Kalpanika 读这个目录的口径是
 *     "SECd" + version + count  +  count × (offset u32, size u32, type u32)
 * 条目给的是**逻辑顺序**（缩略图 JPEG / CAMF / RAW），跟各段在文件里的
 * **物理顺序无关**：实测 SDQ 的 X3I 帧是 CAMF → RAW → JPEG，而独立 X3F 里
 * 两种排法都有（DP2Q6457 是 JPEG → CAMF → RAW，SDQH0980 是 CAMF → RAW → JPEG）。
 * 所以不能要求"条目首尾相接、第 0 段由头给出"—— 那条链式校验正是 SDQ_0665.X3I
 * 七帧全被判成"格式错误"的原因（见 outputs/X3I_SFD_容器与支持方案.md）。
 *
 * 改成逐条 + 覆盖性校验，仍然拦得住"RAW 数据里偶然出现的 SECd"：
 *   · 每条 (off, len) 落在帧内，且整段都在目录之前；
 *   · off 处真的站着一个认得出的段头（SECi / SECc / SECp / SECs）；
 *   · 按物理偏移排序后两两不重叠，且前一段的尾就是后一段的头（段间不留缝）；
 *   · 末段与目录之间允许留白（对齐填充，DP2Q6457 留了 0x5400）。
 */
static int secd_parse(FILE *f, long frame_off, long frame_len, long secd_rel,
                      int *out_count) {
    uint8_t hd[12], ent[X3F_MAX_SECTIONS * 12];
    long offs[X3F_MAX_SECTIONS], lens[X3F_MAX_SECTIONS];

    if (secd_rel < 0x40 || secd_rel + 12 > frame_len) return 0;
    if (read_range(f, frame_off + secd_rel, 12, hd) != X3FJPEG_OK) return 0;
    if (memcmp(hd, "SECd", 4) != 0) return 0;
    uint32_t count = rd32(hd, 8);
    if (count < 2 || count > X3F_MAX_SECTIONS) return 0;
    if (secd_rel + 12 + 12 * (long)count > frame_len) return 0;
    if (read_range(f, frame_off + secd_rel + 12, (size_t)(12 * count), ent)
        != X3FJPEG_OK) return 0;

    for (uint32_t k = 0; k < count; k++) {
        long off = (long)rd32(ent, 12 * k);
        long len = (long)rd32(ent, 12 * k + 4);
        uint8_t id[4];
        if (off < 0x40 || len <= 0 || off + len > secd_rel) return 0;
        if (read_range(f, frame_off + off, 4, id) != X3FJPEG_OK) return 0;
        if (memcmp(id, "SECi", 4) != 0 && memcmp(id, "SECc", 4) != 0 &&
            memcmp(id, "SECp", 4) != 0 && memcmp(id, "SECs", 4) != 0) return 0;
        offs[k] = off;
        lens[k] = len;
    }

    /* 按物理偏移排序（count ≤ 8，插入排序够用），再查重叠与缝隙 */
    for (uint32_t a = 1; a < count; a++) {
        long o = offs[a], l = lens[a];
        int b = (int)a - 1;
        while (b >= 0 && offs[b] > o) { offs[b + 1] = offs[b]; lens[b + 1] = lens[b]; b--; }
        offs[b + 1] = o;
        lens[b + 1] = l;
    }
    for (uint32_t k = 0; k + 1 < count; k++)
        if (offs[k] + lens[k] != offs[k + 1]) return 0;

    if (out_count) *out_count = (int)count;
    return 1;
}

/* 解析容器外层头 + 尾部 SECf 帧表，填 frames[]（不含各帧内嵌 JPEG 的细节） */
static int probe_container(FILE *f, long fsize, x3fjpeg_info_t *o) {
    uint8_t h[64], tail[4], d[256];
    if (read_range(f, 0, sizeof(h), h) != X3FJPEG_OK) return X3FJPEG_ERR_IO;
    if (read_range(f, fsize - 4, 4, tail) != X3FJPEG_OK) return X3FJPEG_ERR_IO;

    uint32_t secf_off = rd32(tail, 0);       /* 文件末 4 字节 = 目录自指针 */
    if (secf_off < 0x100 || (long)secf_off + 32 > fsize) return X3FJPEG_ERR_FORMAT;
    /* 目录就在文件尾部，读"能读到的"那一段（112 字节左右），别越过 EOF */
    long avail = fsize - (long)secf_off;
    size_t dlen = (avail < (long)sizeof(d)) ? (size_t)avail : sizeof(d);
    if (read_range(f, (long)secf_off, dlen, d) != X3FJPEG_OK) return X3FJPEG_ERR_IO;
    if (memcmp(d, "SECf", 4) != 0) return X3FJPEG_ERR_FORMAT;

    uint32_t count = rd32(d, 8);
    if (count < 2 || count > X3F_MAX_FRAMES + 1) return X3FJPEG_ERR_FORMAT;
    long off[X3F_MAX_FRAMES + 1], len[X3F_MAX_FRAMES + 1];
    int n = 0;
    off[n] = (long)rd32(d, 12);
    len[n] = (long)rd32(d, 16);
    n++;
    for (uint32_t k = 0; k + 1 < count; k++) {      /* 最后一个条目是截断的自指针 */
        size_t p = 20 + (size_t)k * 12;
        if (p + 12 > dlen) return X3FJPEG_ERR_FORMAT;
        off[n] = (long)rd32(d, p + 4);
        len[n] = (long)rd32(d, p + 8);
        n++;
    }

    int nf = 0;
    for (int i = 0; i < n && nf < X3F_MAX_FRAMES; i++) {
        uint8_t tag[4];
        long end = (i + 1 < n) ? off[i + 1] : fsize;     /* 区间取到下一段起点 */
        if (off[i] <= 0 || end <= off[i] || end > fsize) continue;
        if (read_range(f, off[i], 4, tag) != X3FJPEG_OK) continue;
        if (memcmp(tag, "FOVb", 4) != 0) continue;       /* 非帧段（SPPA 元数据块）跳过 */
        x3fframe_t *fr = &o->frames[nf];
        memset(fr, 0, sizeof(*fr));
        fr->offset = off[i];
        fr->length = end - off[i];
        /* 外层头 0x35 起是"文件顺序 → 曝光台阶"表：1 = 最欠曝，4 = 0 EV */
        if (0x35 + nf < (int)sizeof(h)) fr->ev_ladder = h[0x35 + nf];
        nf++;
    }
    if (nf == 0) return X3FJPEG_ERR_FORMAT;
    o->is_container = 1;
    o->frame_count = nf;
    return X3FJPEG_OK;
}

/* 定位某一帧的 SECd，并把它的内嵌 JPEG / 段表填进 info（段偏移都换算成绝对） */
static int fill_frame(FILE *f, x3fjpeg_info_t *o, int frame) {
    x3fframe_t *fr = &o->frames[frame];
    long win = fr->length < 1024 ? fr->length : 1024;
    uint8_t buf[1024], sd[X3F_MAX_SECTIONS * 12], id[4];
    long hit = -1;
    int count = 0;

    if (win < 64) return X3FJPEG_ERR_FORMAT;
    if (read_range(f, fr->offset + fr->length - win, (size_t)win, buf) != X3FJPEG_OK)
        return X3FJPEG_ERR_IO;
    /* 目录就在帧尾（后面最多跟十几字节对齐填充），从后往前取第一个站得住的 */
    for (long i = win - 4; i >= 0; i--) {
        if (buf[i] != 'S' || memcmp(buf + i, "SECd", 4) != 0) continue;
        long rel = (fr->length - win) + i;
        if (secd_parse(f, fr->offset, fr->length, rel, &count)) {
            hit = rel;
            break;
        }
    }
    if (hit < 0) return X3FJPEG_ERR_FORMAT;
    fr->secd_rel = hit;

    if (read_range(f, fr->offset + hit + 12, (size_t)(12 * count), sd) != X3FJPEG_OK)
        return X3FJPEG_ERR_IO;

    /* 目录里的条目是逻辑顺序（JPEG / CAMF / RAW），物理顺序可能完全不同，
     * 所以按 off 处那个段头来认段：SECc 是 CAMF，SECi 再看数据头 ——
     * 从 FF D8 起头的是内嵌 JPEG，另一个就是 RAW。
     * JPEG 仍旧落在 sections[0]（下游 locate_jpeg / fr->jpeg_* 都按这个约定）。 */
    int jpeg_k = -1;
    for (int k = 0; k < count; k++) {
        long off = (long)rd32(sd, 12 * k), len = (long)rd32(sd, 12 * k + 4);
        uint8_t so[2];
        if (len < 32) continue;
        if (read_range(f, fr->offset + off, 4, id) != X3FJPEG_OK) continue;
        if (memcmp(id, "SECi", 4) != 0) continue;
        if (read_range(f, fr->offset + off + 28, 2, so) != X3FJPEG_OK) continue;
        if (so[0] == 0xFF && so[1] == 0xD8) { jpeg_k = k; break; }
    }

    o->section_count = 0;
    for (int pass = 0; pass < 2; pass++) {
        for (int k = 0; k < count && k < X3F_MAX_SECTIONS; k++) {
            if (pass == 0 ? k != jpeg_k : k == jpeg_k) continue;
            x3fjpeg_section_t *s = &o->sections[o->section_count++];
            memset(s, 0, sizeof(*s));
            copy_tag(s->tag, sd + 12 * (size_t)k + 8, 4);   /* 目录里声明的类型 */
            s->offset = fr->offset + (long)rd32(sd, 12 * k);
            s->length = (long)rd32(sd, 12 * k + 4);
            if (read_range(f, s->offset, 4, id) == X3FJPEG_OK)
                copy_tag(s->sectag, id, 4);
            if (k == jpeg_k) {
                strcpy(s->tag, "JPEG");                      /* 和独立 X3F 那条路同一个叫法 */
                s->is_jpeg = 1;
            }
        }
    }
    if (o->section_count == 0) return X3FJPEG_ERR_FORMAT;

    fr->jpeg_offset = o->sections[0].offset + 28;   /* SECi 段头 28 字节 */
    fr->jpeg_length = o->sections[0].length - 28;
    if (fr->jpeg_offset <= 0 || fr->jpeg_length <= 0 ||
        fr->jpeg_offset + fr->jpeg_length > o->file_size)
        return X3FJPEG_ERR_FORMAT;
    return X3FJPEG_OK;
}

/* ---------- 对外接口 ---------- */

/* 默认帧：环境变量 X3F_SFD_FRAME 优先（和仓库里其它诊断开关一个套路），缺省 0
 * —— 0 就是 0 EV 基准帧，相机总把它写在容器最前面。 */
int x3f_default_frame(const char *path) {
    const char *e = getenv("X3F_SFD_FRAME");
    (void)path;
    if (e != NULL && *e != '\0') {
        int v = atoi(e);
        if (v >= 0 && v < X3F_MAX_FRAMES) return v;
    }
    return 0;
}

int x3f_probe_frame(const char *path, int frame, x3fjpeg_info_t *out) {
    if (!path || !out) return X3FJPEG_ERR_FORMAT;
    memset(out, 0, sizeof(*out));
    out->subsampling = 9;
    FILE *f = fopen(path, "rb");
    if (!f) return X3FJPEG_ERR_IO;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return X3FJPEG_ERR_IO; }
    long fsize = ftell(f);
    if (fsize < 0x40) { fclose(f); return X3FJPEG_ERR_MAGIC; }

    int rc = probe_structure(f, fsize, out);
    if (rc != X3FJPEG_OK) { fclose(f); return rc; }

    if (out->is_container) {
        int fi = (frame < 0) ? x3f_default_frame(path) : frame;
        if (fi < 0 || fi >= out->frame_count) { fclose(f); return X3FJPEG_ERR_BOUNDS; }
        out->frame_index = fi;
        rc = fill_frame(f, out, fi);
        if (rc != X3FJPEG_OK) { fclose(f); return rc; }
        out->jpeg_offset = out->frames[fi].jpeg_offset;
        out->jpeg_length = out->frames[fi].jpeg_length;
    }

    rc = locate_jpeg(f, fsize, out);
    fclose(f);
    return out->has_jpeg ? X3FJPEG_OK : X3FJPEG_ERR_NOJPEG;
}

int x3f_probe(const char *path, x3fjpeg_info_t *out) {
    return x3f_probe_frame(path, -1, out);
}

int x3f_frame_count(const char *path) {
    x3fjpeg_info_t info;
    if (path == NULL) return 0;
    /* 只读外层头 + 尾部帧表就能数出来；JPEG 找不找得到不影响帧数 */
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return 0; }
    long fsize = ftell(f);
    if (fsize < 0x40) { fclose(f); return 0; }
    memset(&info, 0, sizeof(info));
    int rc = probe_structure(f, fsize, &info);
    fclose(f);
    if (rc != X3FJPEG_OK) return 0;
    return info.is_container ? info.frame_count : 1;
}

int x3f_get_jpeg_frame(const char *path, int frame, uint8_t **out_bytes,
                       size_t *out_size, x3fjpeg_info_t *out_info) {
    if (!out_bytes || !out_size) return X3FJPEG_ERR_FORMAT;
    *out_bytes = NULL;
    *out_size = 0;
    x3fjpeg_info_t info;
    int rc = x3f_probe_frame(path, frame, &info);
    if (out_info) *out_info = info;
    if (rc != X3FJPEG_OK) return rc;

    FILE *f = fopen(path, "rb");
    if (!f) return X3FJPEG_ERR_IO;
    uint8_t *buf = malloc((size_t)info.jpeg_length);
    if (!buf) { fclose(f); return X3FJPEG_ERR_NOMEM; }
    if (read_range(f, info.jpeg_offset, (size_t)info.jpeg_length, buf) != X3FJPEG_OK) {
        free(buf); fclose(f); return X3FJPEG_ERR_IO;
    }
    fclose(f);
    *out_bytes = buf;
    *out_size = (size_t)info.jpeg_length;
    return X3FJPEG_OK;
}

int x3f_get_jpeg(const char *path, uint8_t **out_bytes, size_t *out_size,
                 x3fjpeg_info_t *out_info) {
    return x3f_get_jpeg_frame(path, -1, out_bytes, out_size, out_info);
}

int x3f_extract_to_file_frame(const char *src_path, const char *dst_path, int frame,
                              x3fjpeg_info_t *out_info) {
    uint8_t *bytes = NULL;
    size_t n = 0;
    int rc = x3f_get_jpeg_frame(src_path, frame, &bytes, &n, out_info);
    if (rc != X3FJPEG_OK) return rc;
    FILE *o = fopen(dst_path, "wb");
    if (!o) { free(bytes); return X3FJPEG_ERR_IO; }
    size_t written = fwrite(bytes, 1, n, o);
    fclose(o);
    free(bytes);
    return written == n ? X3FJPEG_OK : X3FJPEG_ERR_IO;
}

int x3f_extract_to_file(const char *src_path, const char *dst_path,
                        x3fjpeg_info_t *out_info) {
    return x3f_extract_to_file_frame(src_path, dst_path, -1, out_info);
}

int x3f_frame_read(const char *path, int frame, uint8_t **out_bytes, size_t *out_size) {
    if (!out_bytes || !out_size) return X3FJPEG_ERR_FORMAT;
    *out_bytes = NULL;
    *out_size = 0;

    x3fjpeg_info_t info;
    int rc = x3f_probe_frame(path, frame, &info);
    if (rc != X3FJPEG_OK) return rc;
    if (!info.is_container) return X3FJPEG_ERR_FORMAT;   /* 普通 X3F 本身就是独立文件 */

    const x3fframe_t *fr = &info.frames[info.frame_index];
    if (fr->secd_rel <= 0 || fr->secd_rel + 28 > fr->length) return X3FJPEG_ERR_FORMAT;

    FILE *f = fopen(path, "rb");
    if (!f) return X3FJPEG_ERR_IO;
    uint8_t *buf = malloc((size_t)fr->length);
    if (!buf) { fclose(f); return X3FJPEG_ERR_NOMEM; }
    if (read_range(f, fr->offset, (size_t)fr->length, buf) != X3FJPEG_OK) {
        free(buf); fclose(f); return X3FJPEG_ERR_IO;
    }
    fclose(f);

    /* 末 4 字节 = SECd 目录的自指针。容器里帧 1..6 那 4 字节是 0（只有最后一帧
     * 恰好写对），而解码器正是从文件末 4 字节找目录 —— 必须补成该帧的相对偏移。 */
    long l = fr->length;
    buf[l - 4] = (uint8_t)((uint32_t)fr->secd_rel & 0xff);
    buf[l - 3] = (uint8_t)(((uint32_t)fr->secd_rel >> 8) & 0xff);
    buf[l - 2] = (uint8_t)(((uint32_t)fr->secd_rel >> 16) & 0xff);
    buf[l - 1] = (uint8_t)(((uint32_t)fr->secd_rel >> 24) & 0xff);

    *out_bytes = buf;
    *out_size = (size_t)l;
    return X3FJPEG_OK;
}

const char *x3f_standalone_path(const char *path, int frame, char *tmp_out, size_t cap) {
    if (tmp_out != NULL && cap > 0) tmp_out[0] = '\0';
    if (path == NULL) return path;
    if (tmp_out == NULL || cap < 256) return path;   /* 没地方放临时路径就别落盘了 */

    x3fjpeg_info_t info;
    if (x3f_probe_frame(path, -1, &info) != X3FJPEG_OK || !info.is_container) return path;

    uint8_t *bytes = NULL;
    size_t n = 0;
    if (x3f_frame_read(path, frame, &bytes, &n) != X3FJPEG_OK) return path;

    const char *dir = getenv("TMPDIR");
#ifdef _WIN32
    if (dir == NULL || *dir == '\0') dir = getenv("TEMP");
#endif
    char tmpl[1024];
    if (dir == NULL || *dir == '\0') dir = "/tmp";
    snprintf(tmpl, sizeof(tmpl), "%s/x3i-XXXXXX", dir);

    int ok = 0;
#ifdef _WIN32
    /* MSVC 没有 mkstemp/fdopen：_mktemp 就地把 XXXXXX 换成唯一后缀，再普通 fopen */
    FILE *o = (_mktemp(tmpl) != NULL) ? fopen(tmpl, "wb") : NULL;
    if (o != NULL) {
        ok = (fwrite(bytes, 1, n, o) == n);
        fclose(o);
    }
#else
    int fd = mkstemp(tmpl);
    if (fd < 0) { free(bytes); return path; }

    FILE *o = fdopen(fd, "wb");
    if (o != NULL) {
        ok = (fwrite(bytes, 1, n, o) == n);
        fclose(o);
    } else {
        close(fd);
    }
#endif
    free(bytes);
    if (!ok) {
        remove(tmpl);
        return path;
    }
    snprintf(tmp_out, cap, "%s", tmpl);
    return tmp_out;
}

void x3f_temp_release(const char *tmp_path) {
    if (tmp_path != NULL && *tmp_path != '\0') remove(tmp_path);
}

int x3f_read_section(const char *path, int index, uint8_t **out_bytes,
                     size_t *out_size, x3fjpeg_info_t *out_info) {
    if (!out_bytes || !out_size) return X3FJPEG_ERR_FORMAT;
    *out_bytes = NULL;
    *out_size = 0;
    x3fjpeg_info_t info;
    int rc = x3f_probe(path, &info);
    if (out_info) *out_info = info;
    if (rc != X3FJPEG_OK) return rc;
    if (index < 0 || index >= info.section_count) return X3FJPEG_ERR_BOUNDS;
    x3fjpeg_section_t *s = &info.sections[index];
    if (s->length <= 0 || s->offset + s->length > info.file_size) return X3FJPEG_ERR_BOUNDS;
    FILE *f = fopen(path, "rb");
    if (!f) return X3FJPEG_ERR_IO;
    uint8_t *buf = malloc((size_t)s->length);
    if (!buf) { fclose(f); return X3FJPEG_ERR_NOMEM; }
    if (read_range(f, s->offset, (size_t)s->length, buf) != X3FJPEG_OK) {
        free(buf); fclose(f); return X3FJPEG_ERR_IO;
    }
    fclose(f);
    *out_bytes = buf;
    *out_size = (size_t)s->length;
    return X3FJPEG_OK;
}

void x3f_free(void *p) { free(p); }

const char *x3f_strerror(int code) {
    switch (code) {
        case X3FJPEG_OK:         return "成功";
        case X3FJPEG_ERR_IO:     return "文件读写失败";
        case X3FJPEG_ERR_MAGIC:  return "不是 X3F/X3I 文件";
        case X3FJPEG_ERR_NOJPEG: return "未找到内嵌 JPEG";
        case X3FJPEG_ERR_FORMAT: return "参数或格式错误";
        case X3FJPEG_ERR_NOMEM:  return "内存不足";
        case X3FJPEG_ERR_BOUNDS: return "段越界";
        default:             return "未知错误";
    }
}

/* ============================================================================
 * SPP「保存设置」那块参数（SECs 段）—— 见 x3fcore.h 里那一大段说明
 * ==========================================================================*/

/* 槽位：块内偏移（u32）。前七个是七根滑杆，8/9 是色度/亮度噪讯。 */
static const int SPP_SLOT_OFF[12] = {
    1016, 1024, 1032, 1040, 1048, 1056, 1064, 1072, 1080, 1088, 1096, 1104
};
#define SPP_WB_PRESET_OFF   36
#define SPP_DETAIL_OFF      48
#define SPP_HL_CONTROL_OFF 696
#define SPP_WB_GAIN_R_OFF  168
#define SPP_WB_GAIN_B_OFF  180
/* SPP 写的那块设置固定 4856 字节；相机自己写给 X3I 的那块是 3956 字节，
 * 槽位位置上是别的数据（按设置去读会读出一堆假参数），所以按长度认。 */
#define SPP_SECS_BYTES     4856

static int spp_find_secs(FILE *f, long fsize, long *out_off, long *out_len)
{
    uint8_t tail[4];
    if (read_range(f, fsize - 4, 4, tail) != X3FJPEG_OK) return 0;
    long dir = (long)rd32(tail, 0);
    if (dir <= 0 || dir + 32 > fsize) return 0;
    uint8_t d[256];
    /* 目录就贴在文件尾，别越过 EOF 读（read_range 要读满才算成功） */
    long avail = fsize - dir;
    size_t dn = avail < (long)sizeof(d) ? (size_t)avail : sizeof(d);
    if (dn < 32) return 0;
    memset(d, 0, sizeof(d));
    if (read_range(f, dir, dn, d) != X3FJPEG_OK) return 0;
    /* 独立 X3F 的目录 tag 是 SECd，X3I(SFD) 容器的是 SECf —— 条目格式一模一样
     * （off / len / type 三条一组的 12 字节），所以两种都认。以前只认 SECd，
     * 于是**所有 .X3I 里那块 SECs 都读不到**（SPP 存过参数的 SFD 也一样）。 */
    if (memcmp(d, "SECd", 4) != 0 && memcmp(d, "SECf", 4) != 0) return 0;
    uint32_t cnt = rd32(d, 8);
    long e_off = (long)rd32(d, 12), e_len = (long)rd32(d, 16);
    /* 目录里那条 SECs 得是 SPP 写的那块才算设置（见 SPP_SECS_BYTES 处的说明） */
    for (uint32_t k = 0; k + 1 < cnt && k < 64; k++) {
        uint8_t tag[4];
        if (e_off > 0 && e_off + 4 <= fsize &&
            read_range(f, e_off, 4, tag) == X3FJPEG_OK && memcmp(tag, "SECs", 4) == 0 &&
            e_len == SPP_SECS_BYTES) {
            *out_off = e_off; *out_len = e_len; return 1;
        }
        long p = dir + 20 + (long)k * 12;
        if (p + 12 > fsize) break;
        {
            uint8_t tmp[12];
            if (read_range(f, p, 12, tmp) != X3FJPEG_OK) return 0;
            e_off = (long)rd32(tmp, 4);
            e_len = (long)rd32(tmp, 8);
        }
    }
    if (e_off > 0 && e_off + 4 <= fsize && e_len == SPP_SECS_BYTES) {
        uint8_t tag[4];
        if (read_range(f, e_off, 4, tag) == X3FJPEG_OK && memcmp(tag, "SECs", 4) == 0) {
            *out_off = e_off; *out_len = e_len; return 1;
        }
    }
    return 0;
}

static int spp_get32(FILE *f, long off, int *out)
{
    uint8_t b[4];
    if (read_range(f, off, 4, b) != X3FJPEG_OK) return 0;
    *out = (int)(int32_t)rd32(b, 0);
    return 1;
}

int x3f_spp_read_settings(const char *path, x3f_spp_settings_t *out)
{
    FILE *f;
    long fsize, off = 0, len = 0;
    int v;
    if (path == NULL || out == NULL) return X3F_SPP_ERR;
    memset(out, 0, sizeof(*out));
    f = fopen(path, "rb");
    if (f == NULL) return X3F_SPP_ERR;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return X3F_SPP_ERR; }
    fsize = ftell(f);
    if (fsize < 0x40 || !spp_find_secs(f, fsize, &off, &len)) {
        fclose(f);
        return X3F_SPP_NONE;
    }
    out->has_settings = 1;
    /* 七根滑杆（槽位 0..6），逐个读 */
    spp_get32(f, off + SPP_SLOT_OFF[0], &out->exposure);
    spp_get32(f, off + SPP_SLOT_OFF[1], &out->contrast);
    spp_get32(f, off + SPP_SLOT_OFF[2], &out->shadow);
    spp_get32(f, off + SPP_SLOT_OFF[3], &out->highlight);
    spp_get32(f, off + SPP_SLOT_OFF[4], &out->saturation);
    spp_get32(f, off + SPP_SLOT_OFF[5], &out->sharpness);
    spp_get32(f, off + SPP_SLOT_OFF[6], &out->fill_light);
    spp_get32(f, off + SPP_SLOT_OFF[7], &out->slot7);
    spp_get32(f, off + SPP_SLOT_OFF[8], &out->chroma_noise);
    spp_get32(f, off + SPP_SLOT_OFF[9], &out->luma_noise);
    spp_get32(f, off + SPP_SLOT_OFF[10], &out->slot10);
    spp_get32(f, off + SPP_SLOT_OFF[11], &out->slot11);
    spp_get32(f, off + SPP_WB_PRESET_OFF, &out->wb_preset);
    spp_get32(f, off + SPP_DETAIL_OFF, &out->detail);
    spp_get32(f, off + SPP_HL_CONTROL_OFF, &out->hl_control);
    spp_get32(f, off + SPP_WB_GAIN_R_OFF, &out->wb_gain_r);
    spp_get32(f, off + SPP_WB_GAIN_B_OFF, &out->wb_gain_b);
    fclose(f);
    return X3F_SPP_OK;
}

int x3f_spp_write_settings(const char *src_path, const char *dst_path,
                           const x3f_spp_settings_t *in)
{
    FILE *fin, *fout;
    unsigned char *buf = NULL;
    long fsize, off = 0, len = 0, w;
    int rc;
    uint8_t v[4];
    if (src_path == NULL || dst_path == NULL || in == NULL) return X3F_SPP_ERR;
    fin = fopen(src_path, "rb");
    if (fin == NULL) return X3F_SPP_ERR;
    if (fseek(fin, 0, SEEK_END) != 0) { fclose(fin); return X3F_SPP_ERR; }
    fsize = ftell(fin);
    if (fsize <= 0 || fseek(fin, 0, SEEK_SET) != 0) { fclose(fin); return X3F_SPP_ERR; }
    buf = (unsigned char *)malloc((size_t)fsize);
    if (buf == NULL || fread(buf, 1, (size_t)fsize, fin) != (size_t)fsize) {
        free(buf); fclose(fin); return X3F_SPP_ERR;
    }
    fclose(fin);
    /* 用内存里的副本找 SECs：借同一个目录走法 */
    {
        fout = fopen(src_path, "rb");
        if (fout == NULL || !spp_find_secs(fout, fsize, &off, &len)) {
            if (fout) fclose(fout);
            free(buf);
            return X3F_SPP_NONE;
        }
        fclose(fout);
    }
    #define PATCH(off_, val_) do { \
        uint32_t u = (uint32_t)(int32_t)(val_); \
        v[0] = (uint8_t)(u & 0xff); v[1] = (uint8_t)((u >> 8) & 0xff); \
        v[2] = (uint8_t)((u >> 16) & 0xff); v[3] = (uint8_t)((u >> 24) & 0xff); \
        memcpy(buf + (off) + (off_), v, 4); \
    } while (0)
    PATCH(SPP_SLOT_OFF[0], in->exposure);
    PATCH(SPP_SLOT_OFF[1], in->contrast);
    PATCH(SPP_SLOT_OFF[2], in->shadow);
    PATCH(SPP_SLOT_OFF[3], in->highlight);
    PATCH(SPP_SLOT_OFF[4], in->saturation);
    PATCH(SPP_SLOT_OFF[5], in->sharpness);
    PATCH(SPP_SLOT_OFF[6], in->fill_light);
    PATCH(SPP_SLOT_OFF[7], in->slot7);
    PATCH(SPP_SLOT_OFF[8], in->chroma_noise);
    PATCH(SPP_SLOT_OFF[9], in->luma_noise);
    PATCH(SPP_SLOT_OFF[10], in->slot10);
    PATCH(SPP_SLOT_OFF[11], in->slot11);
    PATCH(SPP_WB_PRESET_OFF, in->wb_preset);
    PATCH(SPP_DETAIL_OFF, in->detail);
    PATCH(SPP_HL_CONTROL_OFF, in->hl_control);
    PATCH(SPP_WB_GAIN_R_OFF, in->wb_gain_r);
    PATCH(SPP_WB_GAIN_B_OFF, in->wb_gain_b);
    #undef PATCH
    fout = fopen(dst_path, "wb");
    if (fout == NULL) { free(buf); return X3F_SPP_ERR; }
    w = (long)fwrite(buf, 1, (size_t)fsize, fout);
    fclose(fout);
    free(buf);
    rc = (w == fsize) ? X3F_SPP_OK : X3F_SPP_ERR;
    return rc;
}

/* SECs 段在文件里的位置 —— 给验收用（写完一份新文件后跟原文件逐字节比，
 * 好证明"变的字节全落在这段里"）。没有 SECs 段返回 X3F_SPP_NONE。 */
int x3f_spp_section(const char *path, uint32_t *out_off, uint32_t *out_len)
{
    FILE *f;
    long fsize, off = 0, len = 0;
    if (path == NULL) return X3F_SPP_ERR;
    f = fopen(path, "rb");
    if (f == NULL) return X3F_SPP_ERR;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return X3F_SPP_ERR; }
    fsize = ftell(f);
    if (fsize < 0x40 || !spp_find_secs(f, fsize, &off, &len)) {
        fclose(f);
        return X3F_SPP_NONE;
    }
    fclose(f);
    if (out_off) *out_off = (uint32_t)off;
    if (out_len) *out_len = (uint32_t)len;
    return X3F_SPP_OK;
}
