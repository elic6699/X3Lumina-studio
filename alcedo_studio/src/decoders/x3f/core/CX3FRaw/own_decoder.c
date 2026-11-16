/*
 * own_decoder.c — 自研 Sigma X3F (Quattro 世代) 解码核心
 *
 * 从 x3fdec-app/x3fdec.c 改造：去掉 CLI/缓存/渲染段，补齐与 x3fuse
 * x3f_scene_linear 同构的输出契约（场景线性 float RGB + camera_to_rgb）。
 * 详见 own_decoder.h 头注释。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <setjmp.h>
#include <math.h>
#ifdef _WIN32
/* ── Windows 最小 pthread 垫片（只覆盖本文件用到的 create/join）────────
 * _beginthreadex 的线程函数是 __stdcall unsigned(*)(void*)，pthread 是
 * void*(*)(void*)——中间垫一层 trampoline 转换；join 等结束后回收句柄。
 * 不做 detach（本文件的线程全部 create 后必 join）。 */
#include <windows.h>
#include <process.h>
/* windef.h 会把 near/far 定义成空宏（16 位时代的兼容残留），任何以 near
 * 命名的局部变量都会被它展开坏 —— 立即拆掉（与 x3f_sfd.c 同一份做法）。 */
#undef near
#undef far
typedef HANDLE pthread_t;
typedef struct {
    void *(*fn)(void *);
    void *arg;
} win_thunk_t;

static unsigned __stdcall win_thunk(void *p)
{
    win_thunk_t t = *(win_thunk_t *)p;
    free(p);
    t.fn(t.arg);
    return 0;
}

static int pthread_create(pthread_t *th, const void *attr,
                          void *(*fn)(void *), void *arg)
{
    (void)attr;
    win_thunk_t *t = (win_thunk_t *)malloc(sizeof(win_thunk_t));
    if (t == NULL) return -1;
    t->fn = fn; t->arg = arg;
    uintptr_t h = _beginthreadex(NULL, 0, win_thunk, t, 0, NULL);
    if (h == 0) { free(t); return -1; }
    *th = (pthread_t)h;
    return 0;
}

static int pthread_join(pthread_t th, void **value_ptr)
{
    (void)value_ptr;
    WaitForSingleObject(th, INFINITE);
    CloseHandle(th);
    return 0;
}

/* 调试打印（X3F_OE_SEL_DEBUG）用的线程 id：POSIX pthread_t 在这同样只作
 * %p 显示，垫成 GetCurrentThreadId 语义等价。 */
static DWORD pthread_self(void) { return GetCurrentThreadId(); }
#else
#include <pthread.h>
#include <unistd.h>
#endif
#include "own_gpu.h"

#include "own_decoder.h"
#include "x3f_render.h"     /* x3f_denoise_t / X3F_DN_*：解码步降噪参数，2026-11-14 接入 */

/* ------------------------------------------------------------------ */
/* 基础工具                                                            */
/* ------------------------------------------------------------------ */

/* 作为库运行时，致命错误走 longjmp 回到 own_scene_linear 的恢复点，
 * 由调用方回落 x3fuse，而不是把宿主进程带走。
 * **线程局部**（2026-10-23）：SFD 成对并发解码时两个线程各自 setjmp，
 * 全局 jmp_buf 会被后到的线程覆盖，先到的线程出错时 longjmp 会跳进
 * 别人的栈帧 —— 必须每线程一份。 */
static _Thread_local jmp_buf own_jmp;
static _Thread_local int     own_jmp_armed = 0;

/* lazy 表（guided_up dtab/wv_lut、bilateral rlut）初始化锁。
 * 这些表是"check-then-fill"模式，单线程下安全；成对并发解码时两个线程
 * 可能同时首调 —— wv_lut 那段 free/malloc 会把对方正在填的表释放掉。
 * 锁只盖初始化段（参数不变时立即返回，稳态无竞争）。 */
#ifdef _WIN32
static SRWLOCK g_lazy_lock = SRWLOCK_INIT;
#define LAZY_LOCK()   AcquireSRWLockExclusive(&g_lazy_lock)
#define LAZY_UNLOCK() ReleaseSRWLockExclusive(&g_lazy_lock)
#else
static pthread_mutex_t g_lazy_lock = PTHREAD_MUTEX_INITIALIZER;
#define LAZY_LOCK()   pthread_mutex_lock(&g_lazy_lock)
#define LAZY_UNLOCK() pthread_mutex_unlock(&g_lazy_lock)
#endif

/* SFD 逐帧解码的降噪开关（**线程局部**）。
 *
 * 背景：fuse 后端尊重 nlm=0（解码步整个跳过降噪），而 own 解码的 XFCHAIN
 * NLM 链是无条件跑的 —— SFD 七帧合成的每一帧都在付两趟 NLM 的钱（半层
 * YUV NLM σ=300 + 全分辨率 NLM{0,σ,2σ}），这正是 own 合成比 fuse 慢的
 * 大头。口径本来就是"合成前不逐帧降噪，合成完跑一次 Stage1"。
 *
 * 语义：0 = 跳过解码内**降噪**（XFCHAIN 两趟 NLM / 回退分支的 FPN 双边 /
 * scene_linear 的色度中值），上采样路径完全不动；1 = 恢复默认全开。
 * 线程局部是因为批量导出允许两条 SFD 合成并发（fusionGate 限 2），
 * 进程级全局会在两条合成之间互相踩。 */
static _Thread_local int g_own_frame_nr = 1;

/* 逐通道裁剪标记（见 own_decoder.h）。单槽：请求 → 解码时生成 →
 * take 转移所有权；不取则在下次解码开头释放（最多滞留一份，~26MB）。
 * **线程局部**（2026-10-23）：request → 解码 → take 全程在同一调用线程，
 * 线程局部化后并发解码各用各的槽，零竞态、单线程行为不变。 */
static _Thread_local int g_clip_requested = 0;
static _Thread_local uint8_t *g_clip_mask = NULL;
static _Thread_local int g_clip_w = 0, g_clip_h = 0;

void own_request_clip_mask(void) { g_clip_requested = 1; }

uint8_t *own_take_clip_mask(int *w, int *h)
{
    uint8_t *m = g_clip_mask;
    if (w) *w = g_clip_w;
    if (h) *h = g_clip_h;
    g_clip_mask = NULL; g_clip_w = g_clip_h = 0;
    return m;
}

void own_set_frame_nr(int on)
{
    g_own_frame_nr = on ? 1 : 0;
}

/* ── 宿主下发的「解码步降噪」参数（2026-11-14）────────────────────────
 * 背景：x3f_own_dispatch_scene_linear 一直收到 nlm(0..10) 和 x3f_denoise_t，
 * 但转调 own_scene_linear 时把两者丢了；own 的 XFCHAIN 链里 σ 写死 200。
 * 于是 Quattro 默认「自研解码」档下，界面「降噪强度」滑杆与 CLI --nlm 对
 * 导出 / 预览都不起作用（实测 nlm 0/10 输出 md5 逐位相同）。
 *
 * 接法：own 的 XFCHAIN 两趟 NLM 正是 x3fuse 里 denoise_intensity 控制的那一遍
 * （quattro.rs 的 denoise_active），所以把 nlm/dn 映射到 σ 与各通道权重，
 * 而不是另叠一层降噪。把既有工作点 σ=200 定义为**默认档 nlm=5**（GUI/CLI
 * 默认），σ = 200·(nlm/5)·strength —— 默认参数下逐位零回归；nlm=0 关链、
 * nlm=10 翻倍，与 fuse 侧 scale=nlm/10 同为线性。
 *
 * AUTO 的亮度判据已在 x3f_render.c:load_common_inner 解成 0/1 才传进来，
 * 这里不再读 ISO。线程局部：批量导出允许两条 SFD 合成并发，与
 * g_own_frame_nr 同规格。 */
typedef struct {
    int    nlm;          /* 0..10，产品默认 5 */
    int    has_dn;       /* 宿主有没有给 x3f_denoise_t */
    int    luminance;    /* 0/1（AUTO 已解） */
    int    chroma;
    int    backend;      /* X3F_DN_BACKEND_* */
    int    algorithm;    /* X3F_DN_ALGO_*；own 只有 NLM 实现 */
    double strength;
    double luma_scale;
    int    cpatch, csearch;
} OwnDnCtx;

static _Thread_local OwnDnCtx g_dn = {
    5, 0, 0, 1, X3F_DN_BACKEND_AUTO, X3F_DN_ALGO_NLM, 1.0, 1.0, 3, 11
};

static void own_dn_setup(int nlm, const x3f_denoise_t *dn)
{
    OwnDnCtx c;
    c.nlm       = nlm < 0 ? 0 : (nlm > 10 ? 10 : nlm);
    c.has_dn    = dn != NULL;
    c.luminance = (dn && dn->luminance != 0) ? 1 : 0;
    c.chroma    = dn ? (dn->chroma != 0) : 1;
    c.backend   = dn ? dn->backend   : X3F_DN_BACKEND_AUTO;
    c.algorithm = dn ? dn->algorithm : X3F_DN_ALGO_NLM;
    c.strength  = dn ? dn->strength   : 1.0;
    c.luma_scale= dn ? dn->luma_scale : 1.0;
    c.cpatch    = dn ? dn->chroma_patch  : 3;
    c.csearch   = dn ? dn->chroma_search : 11;
    /* 夹到头文件声明的合法域，别让非法值把 NLM 窗口算崩 */
    if (!(c.strength > 0.0) || c.strength < 0.1) c.strength = 1.0;
    if (c.strength > 3.0) c.strength = 3.0;
    if (!(c.luma_scale > 0.0) || c.luma_scale < 0.1) c.luma_scale = 1.0;
    if (c.luma_scale > 3.0) c.luma_scale = 3.0;
    if (c.cpatch < 1) c.cpatch = 1;
    if (c.cpatch > 9) c.cpatch = 9;
    if (!(c.cpatch & 1)) c.cpatch++;
    if (c.csearch < 1) c.csearch = 1;
    if (c.csearch > 31) c.csearch = 31;
    if (!(c.csearch & 1)) c.csearch++;
    /* own 链目前只有 NLM 复刻，没有 BM3D；显式选 BM3D 时诚实报一声（诊断开关）。 */
    if (c.algorithm == X3F_DN_ALGO_BM3D && getenv("X3FDEC_DN_DEBUG"))
        fprintf(stderr, "[owndn] BM3D 未移植到自研解码链，本次按 NLM 执行\n");
    g_dn = c;
}

/* σ 基准：nlm=5 / strength=1 → 200.0（既有工作点，逐位零回归）。 */
static double own_dn_sigma_base(void)
{
    if (g_dn.nlm <= 0) return 0.0;
    return 200.0 * ((double)g_dn.nlm / 5.0) * g_dn.strength;
}
static double own_dn_h_chroma(double base) { return g_dn.chroma ? base : 0.0; }
static double own_dn_h_luma(double base)
{
    return g_dn.luminance ? base * g_dn.luma_scale : 0.0;
}
static int    own_dn_patch(void)  { return g_dn.cpatch; }
static int    own_dn_search(void) { return g_dn.csearch; }
/* 显式 CPU 档：连 own 的 GPU NLM 也一起关掉（否则 backend 选择名存实亡）。 */
static int    own_dn_force_cpu(void)
{
    return g_dn.has_dn && g_dn.backend == X3F_DN_BACKEND_CPU;
}

static void die(const char *msg)
{
    fprintf(stderr, "x3fdec: %s\n", msg);
    if (own_jmp_armed) longjmp(own_jmp, 1);
    exit(1);
}

/* 「这个文件不是 own 能解的形态」标记（2026-10-28）。
 *
 * own 解码器只覆盖 Quattro 三层形态。别的机型（Merrill 等）连 Quattro RAW
 * 段都找不到，die() 会 longjmp 回到 own_scene_linear 的 setjmp 点。原来那里
 * 一律当"解码失败"（rc=-1）报出去，日志写的是 `own 回落 x3fuse (decode-failed)`
 * —— 听着像文件坏了，其实只是机型不适用。Merrill 现在是正常路径（用户 2026-10-28
 * 定案：Quattro 强制 own + 回落，Merrill 固定 x3fuse），这个分类必须准，
 * 否则每次出图都打一条误导性的错误日志。
 *
 * 置位后 rc=1（unsupported），dispatch 的日志就会写 `unsupported`。
 */
static _Thread_local int own_die_unsupported = 0;

static void die_unsupported(const char *msg)
{
    own_die_unsupported = 1;
    die(msg);
}

static uint32_t rd_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static uint16_t rd_u16(const uint8_t *p) { uint16_t v; memcpy(&v, p, 2); return v; }
static float    rd_f32(const uint8_t *p) { float v;    memcpy(&v, p, 4); return v; }

static uint8_t *read_file(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "x3fdec: cannot open %s\n", path); exit(1); }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc(n ? n : 1);
    if (!buf) die("out of memory");
    if (n > 0 && fread(buf, 1, n, f) != (size_t)n) die("read error");
    fclose(f);
    *len = n;
    return buf;
}

static void write_file(const char *path, const void *buf, size_t len)
{
    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "x3fdec: cannot write %s\n", path); exit(1); }
    if (len && fwrite(buf, 1, len, f) != len) die("write error");
    fclose(f);
}

/* ------------------------------------------------------------------ */
/* 位读取器（MSB-first，与 x3fuse BitReader 一致）                      */
/* ------------------------------------------------------------------ */

typedef struct {
    const uint8_t *bytes;
    size_t len, pos;
    uint64_t buf;
    uint32_t nbits;
} BR;

static void br_fill(BR *br)
{
    while (br->nbits <= 56 && br->pos < br->len) {
        br->buf |= (uint64_t)br->bytes[br->pos++] << (56 - br->nbits);
        br->nbits += 8;
    }
}

static void br_init(BR *br, const uint8_t *bytes, size_t len)
{
    br->bytes = bytes; br->len = len; br->pos = 0;
    br->buf = 0; br->nbits = 0;
    br_fill(br);
}

/* 窥视 n 位（n<=32, 高位对齐）。
 * 不变量: 有效位之下的位恒为 0（skip 左移补零）, 故流尾 nbits<n 时
 * 低位的 0 即天然补零, 无需特判。 */
static inline uint32_t br_peek(BR *br, int n)
{
    if (br->nbits < n) br_fill(br);
    return (uint32_t)(br->buf >> (64 - n));
}

static inline void br_skip(BR *br, int n)
{
    br->buf <<= n;
    br->nbits -= n;
}

/* 慢路径/流尾用: 单 bit 读取 */
static inline int br_get_bit(BR *br)
{
    if (br->nbits == 0) {
        br_fill(br);
        if (br->nbits == 0) return -1;
    }
    int bit = (int)(br->buf >> 63);
    br->buf <<= 1;
    br->nbits--;
    return bit;
}

/* ------------------------------------------------------------------ */
/* Huffman 树（数组实现）                                               */
/* 码字: element.code 的高 code_size 位（左对齐到字节高位）              */
/* 叶值 = 表索引 = 后续幅度位数                                          */
/* ------------------------------------------------------------------ */

typedef struct { int32_t branch[2]; int32_t leaf; } HNode;

#define HNODE_LEAF_UNDEF 0x7fffffff
#define HLOOK 15
#define HLUT_SIZE (1 << HLOOK)

typedef struct {
    HNode *nodes; int nalloc, nused;
    uint8_t *hlen; int8_t *hval;   /* HLOOK 位前缀 -> (码长, 叶值), 0=未命中 */
} HTree;

static void htree_init(HTree *t, int max_codes)
{
    t->nalloc = max_codes * 9 + 4;
    t->nodes = calloc(t->nalloc, sizeof(HNode));
    t->hlen = calloc(HLUT_SIZE, 1);
    t->hval = malloc(HLUT_SIZE);
    if (!t->nodes || !t->hlen || !t->hval) die("out of memory");
    t->nused = 1;
    t->nodes[0].leaf = HNODE_LEAF_UNDEF;
    t->nodes[0].branch[0] = t->nodes[0].branch[1] = 0;
}

static void htree_insert(HTree *t, int length, int code, int value)
{
    if (length <= 0 || length > 27) die("invalid TRUE code length");
    if ((code >> length) != 0) die("TRUE code does not fit length");
    int cur = 0;
    for (int i = length - 1; i >= 0; i--) {
        if (t->nodes[cur].leaf != HNODE_LEAF_UNDEF)
            die("overlapping TRUE codes");
        int bit = (code >> i) & 1;
        if (t->nodes[cur].branch[bit] == 0) {
            if (t->nused >= t->nalloc) die("huffman tree overflow");
            int idx = t->nused++;
            t->nodes[idx].leaf = HNODE_LEAF_UNDEF;
            t->nodes[idx].branch[0] = t->nodes[idx].branch[1] = 0;
            t->nodes[cur].branch[bit] = idx;
        }
        cur = t->nodes[cur].branch[bit];
    }
    if (t->nodes[cur].branch[0] || t->nodes[cur].branch[1] ||
        t->nodes[cur].leaf != HNODE_LEAF_UNDEF)
        die("overlapping TRUE codes");
    t->nodes[cur].leaf = value;
    /* LUT 条目写回（在树校验之后, 避免重叠码污染） */
    if (length <= HLOOK) {
        int base = code << (HLOOK - length);
        int nent = 1 << (HLOOK - length);
        for (int i = 0; i < nent; i++) {
            t->hlen[base + i] = (uint8_t)length;
            t->hval[base + i] = (int8_t)value;
        }
    }
}

/* 逐位走树（长码 >HLOOK 或流尾回退） */
static int htree_read(HTree *t, BR *br)
{
    int cur = 0;
    for (int depth = 0; depth <= 27; depth++) {
        if (t->nodes[cur].leaf != HNODE_LEAF_UNDEF) return t->nodes[cur].leaf;
        int bit = br_get_bit(br);
        if (bit < 0) die("truncated entropy stream");
        cur = t->nodes[cur].branch[bit];
        if (cur == 0) die("invalid huffman code");
    }
    die("huffman tree too deep");
    return -1;
}

static inline int huff_decode(HTree *t, BR *br)
{
    uint32_t p = br_peek(br, HLOOK);
    int len = t->hlen[p];
    if (len) {
        if (len > br->nbits) die("truncated entropy stream");
        br_skip(br, len);
        return t->hval[p];
    }
    return htree_read(t, br);
}

/* TRUE 差分解码（与 x3fuse true_diff 位级一致） */
static int32_t true_diff(HTree *t, BR *br)
{
    int bits = huff_decode(t, br);
    if (bits == 0) return 0;
    if (bits > 31) die("TRUE difference exceeds 31 bits");
    if (br->nbits < bits) {
        br_fill(br);
        if (br->nbits < bits) die("truncated magnitude bits");
    }
    uint32_t m = (uint32_t)(br->buf >> (64 - bits));
    br_skip(br, bits);
    int32_t d = (int32_t)m;
    if (!(m >> (bits - 1))) d -= ((int32_t)1 << bits) - 1;
    return d;
}

/* ------------------------------------------------------------------ */
/* 阶段计时（X3FDEC_TIME=1 开启, 输出到 stderr）                        */
/* ------------------------------------------------------------------ */
#include <time.h>

static _Thread_local double stage_acc;
static _Thread_local struct timespec stage_t0;
static _Thread_local int stage_on = -1;

#ifdef _WIN32
/* MSVC 没有 clock_gettime（POSIX）：用 QueryPerformanceCounter 等价实现，
 * 单调、纳秒分辨率 —— 只是计时打点（X3FDEC_TIME），语义不变。 */
#include <time.h>
static void win_clock_gettime(struct timespec *ts)
{
    static LARGE_INTEGER freq = {0}, t0 = {0};
    LARGE_INTEGER now;
    if (freq.QuadPart == 0) {
        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&t0);
    }
    QueryPerformanceCounter(&now);
    long long el = now.QuadPart - t0.QuadPart;
    ts->tv_sec  = (time_t)(el / freq.QuadPart);
    ts->tv_nsec = (long)((el % freq.QuadPart) * 1000000000LL / freq.QuadPart);
}
#endif

static void stage_t(const char *name)
{
    if (stage_on < 0) stage_on = getenv("X3FDEC_TIME") != NULL;
    if (!stage_on) return;
    struct timespec now;
#ifdef _WIN32
    win_clock_gettime(&now);
#else
    clock_gettime(CLOCK_MONOTONIC, &now);
#endif
    static _Thread_local int first = 1;
    if (first) { stage_t0 = now; first = 0; }
    double dt = (now.tv_sec - stage_t0.tv_sec) + (now.tv_nsec - stage_t0.tv_nsec) / 1e9;
    stage_acc += dt;
    fprintf(stderr, "TIME %-26s %6.3f s (acc %.3f)\n", name, dt, stage_acc);
#ifdef _WIN32
    win_clock_gettime(&stage_t0);
#else
    clock_gettime(CLOCK_MONOTONIC, &stage_t0);
#endif
}

/* ------------------------------------------------------------------ */
/* 线程并行（独立通道/平面/行分块并行）                                  */
/* ------------------------------------------------------------------ */

static void run2(void *(*fn)(void *), void *a, void *b)
{
    pthread_t th;
    if (pthread_create(&th, NULL, fn, b) == 0) {
        fn(a);
        pthread_join(th, NULL);
    } else { fn(a); fn(b); }
}

static void run3(void *(*fn)(void *), void *a, void *b, void *c)
{
    pthread_t t1, t2;
    int ok1 = pthread_create(&t1, NULL, fn, b) == 0;
    int ok2 = ok1 && pthread_create(&t2, NULL, fn, c) == 0;
    fn(a);
    if (ok2) pthread_join(t2, NULL);
    if (ok1) pthread_join(t1, NULL);
}

/* N 个同构作业并行（作业数组 jobs, 每个大小 jsz）。 */
static void run_jobs(void *(*fn)(void *), void *jobs, size_t jsz, int n)
{
    pthread_t th[16];
    if (n > 16) n = 16;
    int started = 0;
    for (int i = 1; i < n; i++) {
        void *jp = (char *)jobs + (size_t)i * jsz;
        if (pthread_create(&th[started], NULL, fn, jp) == 0) started++;
    }
    fn(jobs); /* 第 0 块主线程执行 */
    for (int i = 0; i < started; i++) pthread_join(th[i], NULL);
}

/* 并行度：核数（上限 16）。行分块并行用，X3F_OWN_THREADS 可覆盖（1 = 串行）。 */
static int own_nt(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("X3F_OWN_THREADS");
        if (e != NULL && *e != '\0') v = atoi(e);
        else {
#ifdef _WIN32
            SYSTEM_INFO si;
            GetSystemInfo(&si);
            v = (int)si.dwNumberOfProcessors;
#else
            v = (int)sysconf(_SC_NPROCESSORS_ONLN);
#endif
        }
        if (v < 1) v = 1;
        if (v > 16) v = 16;
    }
    return v;
}

/* 行分块并行。fn(ctx, y0, y1) 只处理 [y0, y1) 这些行；
 * 要求各段的像素互不重叠、互不依赖 —— 满足时结果与串行**逐位一致**
 * （不涉浮点重排，只是把独立像素分给不同线程）。
 * minrows = 每段最少行数，活太小就不值得开线程。
 * 注意别嵌套：run_rows 内部不再调 run_rows（SFD 成对解码是两条独立线程，
 * 各开 own_nt() 份，上限 16 是为了这种情形不把核挤爆）。 */
typedef void (*RowFn)(void *ctx, int y0, int y1);
typedef struct { RowFn fn; void *ctx; int y0, y1; } RowChunk;
static void *row_chunk_thunk(void *p)
{
    RowChunk *c = (RowChunk *)p;
    c->fn(c->ctx, c->y0, c->y1);
    return NULL;
}
static void run_rows(RowFn fn, void *ctx, int nrows, int minrows)
{
    int nt, i, started = 0;
    RowChunk chunk[16];
    pthread_t th[16];
    if (nrows <= 0) return;
    if (minrows < 1) minrows = 1;
    nt = own_nt();
    if (nt > nrows / minrows) nt = nrows / minrows;
    if (nt < 2) { fn(ctx, 0, nrows); return; }
    for (i = 1; i < nt; i++) {
        int y0 = (int)((long long)i * nrows / nt);
        int y1 = (int)((long long)(i + 1) * nrows / nt);
        if (y1 <= y0) continue;
        chunk[started].fn = fn; chunk[started].ctx = ctx;
        chunk[started].y0 = y0; chunk[started].y1 = y1;
        if (pthread_create(&th[started], NULL, row_chunk_thunk, &chunk[started]) == 0)
            started++;
    }
    fn(ctx, 0, (int)((long long)nrows / nt));
    for (i = 0; i < started; i++) pthread_join(th[i], NULL);
}

/* ------------------------------------------------------------------ */
/* CAMF (SECc type5) 解码                                               */
/* ------------------------------------------------------------------ */

#define CMbP 0x50624d43
#define CMbT 0x54624d43
#define CMbM 0x4d624d43

typedef struct {
    char    name[64];
    int32_t type;
    /* CMbM 矩阵 */
    int32_t mtype, mrows, mcols;
    double *mdata;       /* 行优先 mrows*mcols */
    /* CMbT 文本 */
    char   *text;
    uint32_t elen;         /* 整个条目长度（属性表偏移的基准上限） */
    /* 原始条目（CMbP 解析用） */
    uint8_t *entry;      /* 条目起始 */
    const uint8_t *value;/* 值区起始 */
    size_t vlen;
} CamfEntry;

typedef struct {
    uint8_t *decoded; size_t decoded_size;
    CamfEntry *entries; int nentries;
} Camf;

/* type5: 头 28 字节 = Huffman 表对 (code_size,code) 至 code_size==0；
 * offset 28: u32 位流长度；SECc 头 val0=解码后字节数, val1=种子；
 * 位流自 offset 32 开始，逐字节累积差分。 */
static void camf_decode_type5(const uint8_t *data, size_t len,
                              uint32_t val0, uint32_t val1, Camf *camf)
{
    HTree tree;
    htree_init(&tree, 16);
    int ntab = 0;
    for (size_t off = 0; off + 1 < 28; off += 2) {
        if (data[off] == 0) break;
        if (++ntab > 14) die("unterminated CAMF huffman table");
        htree_insert(&tree, data[off], data[off + 1] >> (8 - data[off]), ntab - 1);
    }
    if (len < 32) die("CAMF section too small");
    uint32_t decoding_size = rd_u32(data + 28);
    if (decoding_size > len - 32) die("CAMF bitstream outside section");

    size_t size = val0;
    if (size == 0 || size > ((size_t)1 << 30)) die("empty CAMF");
    uint8_t *out = calloc(size, 1);
    if (!out) die("out of memory");

    BR br; br_init(&br, data + 32, decoding_size);
    int32_t acc = (int32_t)val1;
    for (size_t i = 0; i < size; i++) {
        acc += true_diff(&tree, &br);
        out[i] = (uint8_t)acc;
    }
    camf->decoded = out;
    camf->decoded_size = size;
    free(tree.nodes); free(tree.hlen); free(tree.hval);
}

static double camf_matrix_get(const CamfEntry *e, int r, int c)
{
    return e->mdata[(size_t)r * e->mcols + c];
}

static void camf_parse_entries(Camf *camf)
{
    const uint8_t *d = camf->decoded;
    size_t len = camf->decoded_size;
    int count = 0;
    size_t off = 0;
    while (off < len) {
        int all_zero = 1;
        for (size_t i = off; i < len; i++) if (d[i]) { all_zero = 0; break; }
        if (all_zero) break;
        uint32_t id = rd_u32(d + off);
        if (id != CMbP && id != CMbT && id != CMbM)
            die("unknown CAMF entry signature");
        uint32_t esize = rd_u32(d + off + 8);
        if (esize < 20 || off + esize > len) die("invalid CAMF entry length");
        off += esize;
        count++;
    }
    camf->entries = calloc(count ? count : 1, sizeof(CamfEntry));
    camf->nentries = count;
    off = 0;
    for (int i = 0; i < count; i++) {
        CamfEntry *e = &camf->entries[i];
        uint32_t id      = rd_u32(d + off);
        uint32_t esize   = rd_u32(d + off + 8);
        uint32_t nameoff = rd_u32(d + off + 12);
        uint32_t valoff  = rd_u32(d + off + 16);
        e->type = (int32_t)id;
        if (nameoff < 20 || valoff < nameoff || valoff > esize)
            die("invalid CAMF name/value offsets");
        size_t nlen = valoff - nameoff;
        if (nlen >= sizeof(e->name)) nlen = sizeof(e->name) - 1;
        memcpy(e->name, d + off + nameoff, nlen);
        e->name[nlen] = 0;
        const uint8_t *val = d + off + valoff;
        size_t vlen = esize - valoff;
        e->entry = (uint8_t *)(d + off);
        e->value = val;
        e->vlen = vlen;
        e->elen = esize;
        if (e->type == CMbT) {
            uint32_t tsize = vlen >= 4 ? rd_u32(val) : 0;
            if (tsize > vlen - 4) tsize = (uint32_t)(vlen - 4);
            e->text = malloc((size_t)tsize + 1);
            memcpy(e->text, val + 4, tsize);
            e->text[tsize] = 0;
        } else if (e->type == CMbM) {
            e->mtype = (int32_t)rd_u32(val);
            uint32_t dims = rd_u32(val + 4);
            uint32_t dataoff = rd_u32(val + 8);
            if (dims < 1 || dims > 4) die("bad CAMF matrix dims");
            int64_t elems = 1;
            for (uint32_t k = 0; k < dims; k++)
                elems *= (int32_t)rd_u32(val + 12 + 12 * k);
            e->mrows = (int32_t)rd_u32(val + 12);                /* dim0 */
            e->mcols = dims >= 2 ? (int32_t)rd_u32(val + 24) : 1; /* dim1 */
            if (elems <= 0 || elems > (1 << 24)) die("bad CAMF matrix size");
            e->mdata = calloc((size_t)elems, sizeof(double));
            const uint8_t *raw = d + off + dataoff;   /* dataoff 相对条目起始 */
            int esz = e->mtype == 0 ? 2 : (e->mtype == 5 ? 1 :
                      (e->mtype == 6 ? 2 : 4));
            for (int64_t k = 0; k < elems; k++) {
                const uint8_t *p = raw + k * esz;
                switch (e->mtype) {
                case 0: e->mdata[k] = (int16_t)rd_u16(p); break;
                case 1: case 2: e->mdata[k] = rd_u32(p); break;
                case 3: e->mdata[k] = rd_f32(p); break;
                case 5: e->mdata[k] = p[0]; break;
                case 6: e->mdata[k] = rd_u16(p); break;
                default: die("unknown CAMF matrix type");
                }
            }
        }
        off += esize;
    }
}

static const CamfEntry *camf_find(const Camf *c, const char *name)
{
    for (int i = 0; i < c->nentries; i++)
        if (!strcmp(c->entries[i].name, name)) return &c->entries[i];
    return NULL;
}

/* CMbP 属性表查键：list(如 "WhiteBalanceGains") 里 key(如 "Auto") 对应的
 * 值字符串（通常是矩阵名，如 "AutoWBGain"）。与 x3f_get_camf_property 同构。
 * 返回 1 命中（*out 指向 CAMF 缓冲内的字符串，生命周期随 Camf）。 */
static int camf_prop(const Camf *c, const char *list, const char *key,
                     const char **out)
{
    const CamfEntry *e = camf_find(c, list);
    if (!e || e->type != CMbP || e->vlen < 8) return 0;
    uint32_t n    = rd_u32(e->value);
    uint32_t soff = rd_u32(e->value + 4);
    if (n > 4096 || soff > e->elen) return 0;
    for (uint32_t k = 0; k < n; k++) {
        if (8 + 8 * (k + 1) + 8 > e->vlen) break;
        uint32_t no = soff + rd_u32(e->value + 8 + 8 * k);
        uint32_t vo = soff + rd_u32(e->value + 12 + 8 * k);
        if (no >= e->elen || vo >= e->elen) {
            if (getenv("X3FDEC_CAMF_RAW"))
                fprintf(stderr, "[propdbg] %s: pair %u skipped no=%u vo=%u elen=%u\n",
                        e->name, k, no, vo, e->elen);
            continue;
        }
        const char *nm = (const char *)e->entry + no;
        if (!strncmp(nm, key, strlen(key)) && nm[strlen(key)] == '\0') {
            *out = (const char *)e->entry + vo;
            return 1;
        }
    }
    return 0;
}

/* 白平衡名 → CAMF 矩阵：先查属性表（规范路径，与
 * x3f_get_camf_matrix_for_wb 同构），miss 再退回 "<wb><后缀>" 直接命名的
 * 老式条目（x3fdec-app 原型就是这么找的，两代文件都吃得下）。 */
static const CamfEntry *camf_matrix_for_wb(const Camf *c, const char *list,
                                           const char *wb, const char *suffix)
{
    const char *mat = NULL;
    if (camf_prop(c, list, wb, &mat) && mat[0])
        return camf_find(c, mat);
    char alt[96];
    snprintf(alt, sizeof(alt), "%s%s", wb, suffix);
    return camf_find(c, alt);
}

#define SG_MAXCORR 3         /* 非 HP：R, G, B */
typedef struct SgCorr SgCorr;

/* -------- 镜头空间增益 sgain（按 SPP 口径移植 x3f_spatial_gain.c）--------

 *
 * 来源：x3f_tools (Roland & Erik Karlsson, BSD) 的 x3f_spatial_gain.c ——
 * fuj 的 x3fuse 侧也直接 FFI 调这份 C 实现（process.rs:4550/4341），所以
 * 对齐它就是对齐 x3fuse/SPP。施加位置：黑位扣除 + 满量程归一之后、
 * camera_to_rgb 之前，逐通道乘（process.rs: `sg[c] * sat_ratio[c]`）。
 *
 * 布局：Merrill/Quattro 非 HP 路径（corr_num=3, R/G/B）。
 *   - CAMF "CaptureAperture"（float 1x1）
 *   - CAMF "SpatialGain_Fstop"（float 表），块名 SpatialGainsProps_<idx>_<INF|MOD>
 *   - 每块属性 GainsTable<chan> → uint 矩阵名、MinGains<chan>、Delta<chan>
 *   - 块坐标 x=1/fstop，y=透镜位置（INF→焦距；MOD→lens_position(f, MOD)）
 *   - 目标点 (1/aperture, lens_position(f, ObjectDistance×10))，四象限双线性
 * 已知偏差：FLENGTH 读自 PROP 段（SECp），本实现不解析 SECp，恒用 legacy
 * 的缺省 30mm —— 实测 DP2Q 文件没有 SECp 段（x3fuse 同样走缺省），仅当
 * 未来遇到带 SECp 的 Quattro 文件且合焦不在无穷远时 y 轴权重会有微小差异。
 */

typedef struct {
    double weight;
    const double *gain;      /* uint 表（0..255）→ double */
    double mingain, delta;
} SgMerrill;

struct SgCorr {
    double *gain;            /* 按权重合成后的最终表 */
    int rows, cols;
    int rowoff, coloff, rowpitch, colpitch;
    int chan, channels;
    SgMerrill mgain[4];
    int mgain_num;
};

static double sg_lens_position(double focal_length, double object_distance)
{
    return 1.0 / (1.0 / focal_length - 1.0 / object_distance);
}

static double sg_focal_length(void)
{
    /* legacy：PROP 段 "FLENGTH"，缺省 30mm（DP2Q 无 SECp 段，两路同值） */
    return 30.0;
}

static double sg_object_distance(const Camf *c)
{
    const CamfEntry *e = camf_find(c, "ObjectDistance");
    if (e && e->mrows * e->mcols >= 1)
        return (double)e->mdata[0] * 10.0;      /* cm → mm */
    return INFINITY;
}

static double sg_get_MOD(const Camf *c)
{
    const CamfEntry *e = camf_find(c, "LensInformation");
    int32_t li = e ? (int32_t)e->mdata[0] : -1;
    switch (li) {
    case 1003: return 200.0;    /* DP1 Merrill */
    case 1004: return 280.0;    /* DP2 Merrill */
    case 1005: return 226.0;    /* DP3 Merrill */
    default:   return 280.0;
    }
}

/* 块属性：GainsTable<chan> → uint 矩阵，MinGains<chan>/Delta<chan> → 数值 */
static int sg_gains_table(const Camf *c, const char *block, const char *chan,
                          const double **tab, int *rows, int *cols,
                          double *mingain, double *delta)
{
    char key[32];
    const char *val;
    snprintf(key, sizeof(key), "GainsTable%s", chan);
    if (!camf_prop(c, block, key, &val)) return 0;
    const CamfEntry *m = camf_find(c, val);
    if (!m || m->mrows <= 0 || m->mcols <= 0) return 0;
    if (*rows != -1 && *rows != m->mrows) return 0;
    if (*cols != -1 && *cols != m->mcols) return 0;
    *rows = m->mrows; *cols = m->mcols;
    *tab = m->mdata;

    snprintf(key, sizeof(key), "MinGains%s", chan);
    if (!camf_prop(c, block, key, &val)) return 0;
    *mingain = atof(val);
    snprintf(key, sizeof(key), "Delta%s", chan);
    if (!camf_prop(c, block, key, &val)) return 0;
    *delta = atof(val);
    return 1;
}

typedef struct { const char *name; double x, y; } SgBlock;

/* 返回 corr_num（0 = 该文件没有可用的 merrill 型增益表） */
static int sg_get_merrill_type(const Camf *c, SgCorr *corr)
{
    const CamfEntry *ap_e = camf_find(c, "CaptureAperture");
    if (!ap_e || ap_e->mrows * ap_e->mcols < 1) return 0;
    double capture_aperture = (double)ap_e->mdata[0];

    /* IncludeBlocks = 全量属性表；从里面挑 SpatialGainsProps_* 块名 */
    const CamfEntry *ib = camf_find(c, "IncludeBlocks");
    if (!ib || ib->type != CMbP || ib->vlen < 8) return 0;

    SgBlock blocks[64];
    int nblocks = 0;
    double fstop_tab[64]; int num_fstop = 0;
    const CamfEntry *fs = camf_find(c, "SpatialGain_Fstop");
    if (fs && fs->mrows * fs->mcols >= 1) {
        num_fstop = fs->mrows * fs->mcols;
        if (num_fstop > 64) num_fstop = 64;
        for (int k = 0; k < num_fstop; k++) fstop_tab[k] = fs->mdata[k];
    }

    uint32_t n    = rd_u32(ib->value);
    uint32_t soff = rd_u32(ib->value + 4);
    if (n > 4096) n = 4096;
    for (uint32_t k = 0; k < n && nblocks < 64; k++) {
        if (8 + 8 * (k + 1) + 8 > ib->vlen) break;
        uint32_t no = soff + rd_u32(ib->value + 8 + 8 * k);
        if (no >= ib->elen) continue;
        const char *nm = (const char *)ib->entry + no;
        int idx; char fd[4], dummy;
        if (num_fstop > 0 &&
            sscanf(nm, "SpatialGainsProps_%d_%3s%c", &idx, fd, &dummy) == 2 &&
            idx >= 0 && idx < num_fstop) {
            double lenspos;
            if (!strcmp(fd, "INF"))
                lenspos = sg_lens_position(sg_focal_length(), INFINITY);
            else if (!strcmp(fd, "MOD"))
                lenspos = sg_lens_position(sg_focal_length(), sg_get_MOD(c));
            else continue;
            blocks[nblocks].name = nm;
            blocks[nblocks].x = 1.0 / fstop_tab[idx];
            blocks[nblocks].y = lenspos;
            nblocks++;
        } else {
            double aperture, lenspos;
            if (sscanf(nm, "SpatialGainsProps_%lf_%lf%c",
                       &aperture, &lenspos, &dummy) == 2 && aperture > 0) {
                blocks[nblocks].name = nm;
                blocks[nblocks].x = 1.0 / aperture;
                blocks[nblocks].y = lenspos;
                nblocks++;
            }
        }
    }
    if (!nblocks) return 0;

    double x = 1.0 / capture_aperture;
    double y = sg_lens_position(sg_focal_length(), sg_object_distance(c));

    /* 四象限最近点 + 双线性权重（照抄 legacy） */
    SgBlock *qc[4] = { NULL, NULL, NULL, NULL };
    double qdx[4] = { INFINITY, -INFINITY, -INFINITY, INFINITY };
    double qdy[4] = { INFINITY, INFINITY, -INFINITY, -INFINITY };
    double qd2[4] = { INFINITY, INFINITY, INFINITY, INFINITY };
    for (int i = 0; i < nblocks; i++) {
        double dx = blocks[i].x - x, dy = blocks[i].y - y;
        double d2 = dx * dx + dy * dy;
        int q;
        if      (dx > 0.0 && dy > 0.0) q = 0;
        else if (dx > 0.0)             q = 3;
        else if (dy > 0.0)             q = 1;
        else                           q = 2;
        if (d2 < qd2[q]) { qc[q] = &blocks[i]; qdx[q] = dx; qdy[q] = dy; qd2[q] = d2; }
    }
    double wx[4], wy[4], w[4];
    wx[0] = qdx[1] / (qdx[1] - qdx[0]);
    wx[1] = qdx[0] / (qdx[0] - qdx[1]);
    wx[2] = qdx[3] / (qdx[3] - qdx[2]);
    wx[3] = qdx[2] / (qdx[2] - qdx[3]);
    wy[0] = qdy[3] / (qdy[3] - qdy[0]);
    wy[1] = qdy[2] / (qdy[2] - qdy[1]);
    wy[2] = qdy[1] / (qdy[1] - qdy[2]);
    wy[3] = qdy[0] / (qdy[0] - qdy[3]);
    for (int i = 0; i < 4; i++) {
        if (isnan(wx[i])) wx[i] = 1.0;
        if (isnan(wy[i])) wy[i] = 1.0;
        w[i] = wx[i] * wy[i];
    }

    static const char *channels[3] = { "R", "G", "B" };
    for (int i = 0; i < SG_MAXCORR; i++) {
        SgCorr *cc = &corr[i];
        cc->gain = NULL; cc->rows = cc->cols = -1;
        cc->rowoff = cc->coloff = 0;
        cc->rowpitch = cc->colpitch = 1;
        cc->chan = i; cc->channels = 1;
        cc->mgain_num = 0;
    }
    for (int i = 0; i < 4; i++) {
        if (!qc[i]) continue;
        for (int j = 0; j < SG_MAXCORR; j++) {
            SgCorr *cc = &corr[j];
            SgMerrill *m = &cc->mgain[cc->mgain_num++];
            m->weight = w[i];
            if (!sg_gains_table(c, qc[i]->name, channels[j],
                                &m->gain, &cc->rows, &cc->cols,
                                &m->mingain, &m->delta))
                return 0;
        }
    }
    for (int i = 0; i < SG_MAXCORR; i++)
        if (corr[i].mgain_num == 0) return 0;

    /* 按权重合成最终表：mingain + delta·table 的加权和 */
    for (int i = 0; i < SG_MAXCORR; i++) {
        SgCorr *cc = &corr[i];
        int num = cc->rows * cc->cols * cc->channels;
        cc->gain = malloc((size_t)num * sizeof(double));
        for (int j = 0; j < num; j++) {
            cc->gain[j] = 0.0;
            for (int g = 0; g < cc->mgain_num; g++) {
                const SgMerrill *m = &cc->mgain[g];
                cc->gain[j] += m->weight * (m->mingain + m->delta * m->gain[j]);
            }
        }
    }
    return SG_MAXCORR;
}

static void sg_cleanup(SgCorr *corr, int n)
{
    for (int i = 0; i < n; i++) free(corr[i].gain);
}

/* x3f_calc_spatial_gain 原样移植：表坐标 = 相对行/列 × (表维-1) 双线性 */
static double sg_calc(const SgCorr *corr, int corr_num,
                      int row, int col, int chan, int rows, int cols)
{
    double gain = 1.0;
    double rrel = (double)row / rows;
    double crel = (double)col / cols;
    for (int i = 0; i < corr_num; i++) {
        const SgCorr *c = &corr[i];
        int ch = chan - c->chan;
        if (ch < 0 || ch >= c->channels) continue;
        if (row % c->rowpitch != c->rowoff) continue;
        if (col % c->colpitch != c->coloff) continue;

        double rc = rrel * (c->rows - 1);
        int ri = (int)floor(rc);
        double rf = rc - ri;
        double cc2 = crel * (c->cols - 1);
        int ci = (int)floor(cc2);
        double cf = cc2 - ci;

        const double *r1, *r2;
        int co1, co2;
        if (ri < 0) r1 = r2 = &c->gain[0];
        else if (ri >= c->rows - 1)
            r1 = r2 = &c->gain[(size_t)(c->rows - 1) * c->cols * c->channels];
        else {
            r1 = &c->gain[(size_t)ri * c->cols * c->channels];
            r2 = &c->gain[(size_t)(ri + 1) * c->cols * c->channels];
        }
        if (ci < 0) co1 = co2 = ch;
        else if (ci >= c->cols - 1)
            co1 = co2 = (c->cols - 1) * c->channels + ch;
        else {
            co1 = ci * c->channels + ch;
            co2 = (ci + 1) * c->channels + ch;
        }
        double gr1 = r1[co1] + cf * (r1[co2] - r1[co1]);
        double gr2 = r2[co1] + cf * (r2[co2] - r2[co1]);
        gain *= gr1 + rf * (gr2 - gr1);
    }
    return gain;
}

/* ------------------------------------------------------------------ */
/* 容器解析                                                             */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t offset, size, type;
    /* SECi */
    uint32_t sec_type, sec_format, columns, rows, row_stride;
    /* SECc */
    uint32_t camf_type, val0, val1;
} DirEntry;

typedef struct {
    uint8_t *data; size_t len;
    uint32_t version;
    DirEntry *ent; int nent;
} X3F;

static void x3f_parse(X3F *x, const char *path)
{
    x->data = read_file(path, &x->len);
    if (x->len < 28 || memcmp(x->data, "FOVb", 4)) die("not an X3F file (FOVb)");
    x->version = rd_u32(x->data + 4);
    uint32_t diroff = rd_u32(x->data + x->len - 4);
    if (diroff + 12 > x->len) die("bad directory offset");
    const uint8_t *d = x->data + diroff;
    if (memcmp(d, "SECd", 4)) die("bad directory signature");
    uint32_t count = rd_u32(d + 8);
    if (diroff + 12 + (uint64_t)count * 12 > x->len) die("directory overflow");
    x->nent = (int)count;
    x->ent = calloc(count ? count : 1, sizeof(DirEntry));
    for (uint32_t i = 0; i < count; i++) {
        const uint8_t *e = d + 12 + 12 * i;
        DirEntry *de = &x->ent[i];
        de->offset = rd_u32(e);
        de->size   = rd_u32(e + 4);
        de->type   = rd_u32(e + 8);
        if ((uint64_t)de->offset + de->size > x->len || de->size < 8)
            die("section outside container");
        const uint8_t *sh = x->data + de->offset;
        if (!memcmp(sh, "SECi", 4)) {
            if (de->size < 28) die("truncated image header");
            de->sec_type   = rd_u32(sh + 8);
            de->sec_format = rd_u32(sh + 12);
            de->columns    = rd_u32(sh + 16);
            de->rows       = rd_u32(sh + 20);
            de->row_stride = rd_u32(sh + 24);
        } else if (!memcmp(sh, "SECc", 4)) {
            if (de->size < 28) die("truncated camf header");
            de->camf_type = rd_u32(sh + 8);
            de->val0      = rd_u32(sh + 12);
            de->val1      = rd_u32(sh + 16);
        }
    }
}

/* ------------------------------------------------------------------ */
/* TRUE 平面解码                                                        */
/* ------------------------------------------------------------------ */

static void decode_true_plane(const uint8_t *plane, size_t plen, HTree *tree,
                              uint32_t rows, uint32_t cols, uint16_t seed,
                              uint16_t *out, uint32_t out_rows,
                              uint32_t out_cols, int out_ch, int out_channels)
{
    BR br; br_init(&br, plane, plen);
    int32_t starts[2][2] = { { seed, seed }, { seed, seed } };
    for (uint32_t row = 0; row < rows; row++) {
        int parity = row & 1;
        int32_t acc[2] = { 0, 0 };
        for (uint32_t col = 0; col < cols; col++) {
            int odd = col & 1;
            int32_t prev = (col < 2) ? starts[parity][odd] : acc[odd];
            int32_t v = prev + true_diff(tree, &br);
            acc[odd] = v;
            if (col < 2) starts[parity][odd] = v;
            if (col < out_cols && row < out_rows) {
                size_t idx = (size_t)row * out_cols * out_channels
                           + (size_t)col * out_channels + out_ch;
                out[idx] = (uint16_t)v;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Quattro 三层重建（对应 x3fuse quattro.rs，OpenCV 双三次 a=-0.75）      */
/* ------------------------------------------------------------------ */

/* 单通道双三次 2x 上采样（float 版）: src (sw x sh) -> dst (2sw x 2sh)
 * 与 OpenCV INTER_CUBIC 对齐: sx = (dx+0.5)/2 - 0.5 */
static inline float cubicf(float d, float a)
{
    d = fabsf(d);
    if (d <= 1.0f) return ((a + 2.0f) * d - (a + 3.0f)) * d * d + 1.0f;
    if (d < 2.0f)  return ((a * d - 5.0f * a) * d + 8.0f * a) * d - 4.0f * a;
    return 0.0f;
}

/* 双三次 2x 上采样。两趟（水平 → tmp、垂直 → dst）各自按行分块并行：
 * 每个输出行只依赖固定 4 个 tap 的输入行/列，块与块之间没有任何共享状态，
 * 所以并行结果与串行**逐位一致**（不需要对拍容差，md5 相同）。
 * 汇总记账：5424×3616 全幅两色平面 0.59s（含下面的值域钳制）→ 多核 0.1s 级。 */
typedef struct {
    const float *src; int sw, sh;
    float *tmp; int dw, dh;
    float *dst;
    float tw[2][4]; int toff[2];
    float A;
} Rb2Ctx;

static void rb2_hpass(void *vctx, int y0, int y1)
{
    Rb2Ctx *c = (Rb2Ctx *)vctx;
    int sw = c->sw, dw = c->dw;
    for (int y = y0; y < y1; y++) {
        const float *srow = c->src + (size_t)y * sw;
        float *trow = c->tmp + (size_t)y * dw;
        for (int dx = 0; dx < dw; dx++) {
            int phase = dx & 1;
            int k = dx >> 1;
            float v = 0;
            for (int j = 0; j < 4; j++) {
                int xx = k + c->toff[phase] + j;
                if (xx < 0) xx = -xx;
                if (xx >= sw) xx = 2 * (sw - 1) - xx;
                v += srow[xx] * c->tw[phase][j];
            }
            trow[dx] = v;
        }
    }
}

static void rb2_vpass(void *vctx, int dy0, int dy1)
{
    Rb2Ctx *c = (Rb2Ctx *)vctx;
    int sh = c->sh, dw = c->dw;
    for (int dy = dy0; dy < dy1; dy++) {
        float sy = (dy + 0.5f) * 0.5f - 0.5f;
        int isy = (int)floorf(sy);
        float t = sy - (float)isy;     /* t ∈ {0.25, 0.75} */
        float w[4];
        w[0] = cubicf(1.0f + t, c->A);
        w[1] = cubicf(t, c->A);
        w[2] = cubicf(1.0f - t, c->A);
        w[3] = cubicf(2.0f - t, c->A);
        int taps[4];
        for (int j = 0; j < 4; j++) {
            int yy = isy - 1 + j;
            if (yy < 0) yy = -yy;
            if (yy >= sh) yy = 2 * (sh - 1) - yy;
            taps[j] = yy;
        }
        const float *r0 = c->tmp + (size_t)taps[0] * dw;
        const float *r1 = c->tmp + (size_t)taps[1] * dw;
        const float *r2 = c->tmp + (size_t)taps[2] * dw;
        const float *r3 = c->tmp + (size_t)taps[3] * dw;
        float *drow = c->dst + (size_t)dy * dw;
        for (int dx = 0; dx < dw; dx++)
            drow[dx] = w[0] * r0[dx] + w[1] * r1[dx] + w[2] * r2[dx] + w[3] * r3[dx];
    }
}

static void resize_bicubic_2x(const float *src, int sw, int sh, float *dst)
{
    Rb2Ctx c;
    int dw = 2 * sw, dh = 2 * sh;
    int phase;
    memset(&c, 0, sizeof c);
    c.src = src; c.sw = sw; c.sh = sh;
    c.dst = dst; c.dw = dw; c.dh = dh;
    c.A = -0.75f;
    c.tmp = (float *)malloc((size_t)sh * dw * sizeof(float));
    if (!c.tmp) die("out of memory");

    /* 每两个输出列共享一组权重（偶 dx: t=0.75, 奇 dx: t=0.25），
     * tap 基准随 k=dx/2 平移: 偶 dx taps = k-2+j, 奇 dx taps = k-1+j */
    c.toff[0] = -2; c.toff[1] = -1;
    for (phase = 0; phase < 2; phase++) {
        float t = phase ? 0.25f : 0.75f;   /* dx=2k+phase: sx 相位小数部分 */
        c.tw[phase][0] = cubicf(1.0f + t, c.A);
        c.tw[phase][1] = cubicf(t, c.A);
        c.tw[phase][2] = cubicf(1.0f - t, c.A);
        c.tw[phase][3] = cubicf(2.0f - t, c.A);
    }

    run_rows(rb2_hpass, &c, sh, 8);
    run_rows(rb2_vpass, &c, dh, 8);
    free(c.tmp);
}

/* 值域核 LUT 长度: ad ∈ [0, 16383.75], 步长 0.25 */
#define WVLUT_N (1 << 16)

/* 共享静态表（文件作用域: 多线程分块执行前由 guided_up_preinit 统一初始化） */
static float dtab[2][2][5][5];          /* 距离核表 [相位x][相位y][j0+2][i0+2] */
static int dtab_fd = -999;
static float *wv_lut;                   /* 值域核查表 */
static double wv_k = -1;

static void guided_up_preinit(double k, double fd)
{
    /* 并发解码安全（2026-10-23）：wv_lut 的 free/malloc 段两个线程同时
     * 走会把对方正在填的表释放掉。锁内 check-then-fill；参数不变时
     * 快速返回，稳态无竞争。 */
    LAZY_LOCK();
    if (dtab_fd != (int)(fd * 1000)) {
        for (int pxa = 0; pxa < 2; pxa++)
            for (int pya = 0; pya < 2; pya++)
                for (int j = 0; j < 5; j++)
                    for (int i = 0; i < 5; i++) {
                        double fx = pxa ? 0.25 : -0.25;
                        double fy = pya ? 0.25 : -0.25;
                        double dx = fx - (j - 2), dy = fy - (i - 2);
                        dtab[pxa][pya][j][i] = (float)exp(-fd * sqrt(dx * dx + dy * dy));
                    }
        dtab_fd = (int)(fd * 1000);
    }
    if (wv_k != k) {
        float *nw = (float *)malloc(WVLUT_N * sizeof(float));
        if (!nw) { LAZY_UNLOCK(); die("out of memory"); }
        for (int i = 0; i < WVLUT_N; i++)
            nw[i] = (float)(1.0 / (k * (i * 0.25) + 1.0));
        free(wv_lut);
        wv_lut = nw;
        wv_k = k;
    }
    LAZY_UNLOCK();
}

/* 引导双边上采样 2x（对应 SPP QT2B.cl BilateralUpsampling）。
 * 半分辨率 delta 图 src (hw x hh) 上采样到 (2hw x 2hh)。
 * 值域引导用全分辨率绝对色彩估计 guide（半层同通道绝对值的双三次上采样）：
 *   比较对象 = src[s] + t2[s]（半层样本绝对值）vs guide[px]（全分辨率估计）
 * 同通道绝对值比较天然极性无关——黄花瓣/蓝天空这类反极性色彩边缘
 * （蓝通道与 R/G 反相关）不会把样本权重拉错方向，消除花瓣紫边。
 *   值域核 wv = 1/(k·|Δ| + 1)
 *   距离核 wd = exp(−fd·√(fDx²+fDy²))，fD = (±0.25 − j0)（0.25 亚像素相位），预计算 */
static void guided_up_2x(const float *src, int hw, int hh,
                         const float *guide, const float *t2,
                         float *dst, double k, double fd, int y0, int y1)
{
    int ew = 2 * hw, eh = 2 * hh;
    (void)k; (void)fd;   /* 表已由 guided_up_preinit 建好 */
    for (int py = y0; py < y1 && py < eh; py++) {
        int iy = py >> 1, pya = py & 1;
        const float *grow = guide + (size_t)py * ew;
        /* 本输出行涉及的 5 个源行基指针（反射索引每行算一次） */
        const float *srow[5], *trow[5];
        for (int i = 0; i < 5; i++) {
            int sy = iy + i - 2;
            if (sy < 0) sy = -sy;
            if (sy >= hh) sy = 2 * (hh - 1) - sy;
            srow[i] = src + (size_t)sy * hw;
            trow[i] = t2  + (size_t)sy * hw;
        }
        for (int px = 0; px < ew; px++) {
            int ix = px >> 1, pxa = px & 1;
            float g = grow[px];
            float acc = 0, wsum = 0;
            for (int j = 0; j < 5; j++) {
                int sx = ix + j - 2;
                if (sx < 0) sx = -sx;
                if (sx >= hw) sx = 2 * (hw - 1) - sx;
                const float *dr = &dtab[pxa][pya][j][0];
                for (int i = 0; i < 5; i++) {
                    float s = srow[i][sx];
                    float d = (s + trow[i][sx]) - g;
                    float ad = d < 0 ? -d : d;
                    int wi = (int)(ad * 4.0f + 0.5f);
                    if (wi >= WVLUT_N) wi = WVLUT_N - 1;
                    float w = dr[i] * wv_lut[wi];
                    acc += s * w;
                    wsum += w;
                }
            }
            dst[(size_t)py * ew + px] = acc / wsum;
        }
    }
}

/* =================================================================== */
/* 顶层边感知上采样修正 (X3F_OWN_TOPEDGE, 默认关)                      */
/* ------------------------------------------------------------------- */
/* 背景：Quattro 的几何与亮度来自**顶层**（全分辨率），而色度 delta
 * (dB/dG) 只有半层；双三次 2x 把半层色度摊开 ±2px，于是"全分辨率的锐边"
 * 与"它的色度"不同宽 —— 边缘处出现色度比亮度宽一档的晕，1–2px 的细
 * 弱结构（屋顶竖杆、天线、远处栏杆）上最明显：杆本体（顶层）是 1–2px，
 * 色度晕却有 3–4px。
 *
 * 与旧的 X3FDEC_GUIDE_SRC=1 的本质区别：**引导图不再重新放大**。
 * 旧实现把 t2 双三次放大当 guide —— 那份 guide 本身就是糊的，等于没有
 * 高频可依；而且它改的是"半层 R/G 绝对值 vs 估计值"的跨通道比较，在中
 * 低调会把半层色度拉出低频绿/品红斑块（2026-10-24 已回退，X3FDEC_GUIDE
 * 默认关）。本实现直接用**原始全分辨率顶层像素 top**（真 HR、零损失）
 * 作为值域核的一侧，比较对象是同一信号的下采样版 t2 —— 两者同模态、同
 * 黑电平口径，|top−t2| 只衡量"这个全分辨率像素离那个半层采样点所在平台
 * 有多远"，不引入半层 R/G 电荷扩散滞后的跨通道误差。
 *
 * 安全边界（三条）：
 *   - 只重排双三次的**插值权重**，不改写 delta 值本身 → 结果仍落在参与
 *     插值的邻域样本范围内，不会凭空造色；
 *   - 平坦区由 elo 门控直接跳过（变换率 0%，局部 std 零损失）；
 *   - 结果按 conf·mix 混合回原双三次值，mix=0 等价关闭；
 *   - 在本函数调用顺序上排在 clamp_upsample_range 之前，残留的负 lobe
 *     过冲还会被收一次。
 * =================================================================== */
#define TOPEDGE_LUTN 16384
static float *topedge_lut;
static double topedge_sig = -1.0;

static void topedge_preinit(double sigma)
{
    LAZY_LOCK();
    if (topedge_sig != sigma) {
        float *nl = (float *)malloc(TOPEDGE_LUTN * sizeof(float));
        if (!nl) { LAZY_UNLOCK(); die("out of memory"); }
        double inv = 1.0 / (2.0 * sigma * sigma);
        for (int i = 0; i < TOPEDGE_LUTN; i++) {
            double ad = (double)i;
            nl[i] = (float)exp(-ad * ad * inv);
        }
        free(topedge_lut);
        topedge_lut = nl;
        topedge_sig = sigma;
    }
    LAZY_UNLOCK();
}

typedef struct {
    const float *src;    /* 半层 delta (hw x hh) */
    const float *t2;     /* 半层顶层下采样 (hw x hh)，与 src 同源对齐 */
    const uint16_t *top; /* 全分辨率顶层原始像素（未扣黑） */
    int hw, hh, topw;
    float blk2;
    float *dst;          /* 全分辨率：入 = 双三次结果；出 = 修正后结果 */
    float mix, elo, ehi; /* 混合比 / 置信下门 / 置信上门（raw 灰度单位） */
    float ar;            /* 别名判据: 半层动态范围至少为顶层梯度的 ar 倍 */
    int y0, y1;          /* 负责的输出行区间 [y0, y1) */
    long nact, ntot;     /* 实际重算 / 总像素 */
} TopEdgeJob;

static void *topedge_job(void *arg)
{
    TopEdgeJob *j = (TopEdgeJob *)arg;
    const int hw = j->hw, hh = j->hh, ew = hw * 2, eh = hh * 2;
    const float A = -0.75f;
    float tw[2][4];
    for (int ph = 0; ph < 2; ph++) {
        float t = ph ? 0.25f : 0.75f;
        tw[ph][0] = cubicf(1.0f + t, A);
        tw[ph][1] = cubicf(t, A);
        tw[ph][2] = cubicf(1.0f - t, A);
        tw[ph][3] = cubicf(2.0f - t, A);
    }
    const float *lut = topedge_lut;
    const float blk2 = j->blk2;
    long nact = 0, ntot = 0;

    for (int Y = j->y0; Y < j->y1 && Y < eh; Y++) {
        const int ky = Y >> 1;
        const float *wy = tw[Y & 1];
        const uint16_t *tp0 = j->top + (size_t)Y * j->topw;
        const uint16_t *tpU = j->top + (size_t)(Y > 0 ? Y - 1 : Y) * j->topw;
        const uint16_t *tpD = j->top + (size_t)(Y < eh - 1 ? Y + 1 : Y) * j->topw;
        float *drow = j->dst + (size_t)Y * ew;
        /* 本输出行涉及的 4 个源行（反射索引每行算一次） */
        const float *tr[4], *sr[4];
        for (int i = 0; i < 4; i++) {
            int yy = ky + ((Y & 1) ? -1 : -2) + i;
            if (yy < 0) yy = -yy;
            if (yy >= hh) yy = 2 * (hh - 1) - yy;
            if (yy < 0) yy = 0;
            if (yy >= hh) yy = hh - 1;
            tr[i] = j->t2  + (size_t)yy * hw;
            sr[i] = j->src + (size_t)yy * hw;
        }
        for (int X = 0; X < ew; X++) {
            ntot++;
            /* --- 顶层 HR 局部梯度幅值（本行/上下行单步差的最大值） --- */
            float g0 = (float)tp0[X] - blk2;
            float gx = fabsf(g0 - ((float)tp0[X > 0 ? X - 1 : 0] - blk2));
            if (X + 1 < ew) {
                float v = fabsf(g0 - ((float)tp0[X + 1] - blk2));
                if (v > gx) gx = v;
            }
            float gy = fabsf(g0 - ((float)tpU[X] - blk2));
            {
                float v = fabsf(g0 - ((float)tpD[X] - blk2));
                if (v > gy) gy = v;
            }
            float gmag = gx > gy ? gx : gy;
            if (gmag <= j->elo) continue;          /* 平坦区：零损失 */
            float conf = (gmag - j->elo) / (j->ehi - j->elo);
            if (conf > 1.0f) conf = 1.0f;
            conf = conf * conf * (3.0f - 2.0f * conf);   /* smoothstep */

            /* --- 边感知权重重分配：双三次空间核 × 顶层值域核 --- */
            const int kx = X >> 1, offx = (X & 1) ? -1 : -2;
            const float *wx = tw[X & 1];
            int sxr[4];
            for (int i = 0; i < 4; i++) {
                int xx = kx + offx + i;
                if (xx < 0) xx = -xx;
                if (xx >= hw) xx = 2 * (hw - 1) - xx;
                if (xx < 0) xx = 0;
                if (xx >= hw) xx = hw - 1;
                sxr[i] = xx;
            }
            /* --- 别名判据：只动"半层里真的存在的大尺度边" ---
             * 顶层有强梯度、但 4x4 半层样本几乎同值 (trange 小) ⇒ 这是
             * 亚半层分辨率的细节（1px 天线、细枝）：半层 delta 里本来就
             * 没有它，任何重插值都只会把它的色度拖向某个邻点而变淡。
             * 实测确认：不加这道判据时 TOPEDGE 会把极弱天线抹淡。此处
             * 直接跳过，保留原双三次结果（= 保住细弱结构的现状）。 */
            float tlo = tr[0][sxr[0]], thi = tlo;
            for (int iy = 0; iy < 4; iy++)
                for (int ix = 0; ix < 4; ix++) {
                    float tv = tr[iy][sxr[ix]];
                    if (tv < tlo) tlo = tv; else if (tv > thi) thi = tv;
                }
            if ((thi - tlo) < j->ar * gmag) continue;
            float num = 0.0f, den = 0.0f;
            for (int iy = 0; iy < 4; iy++) {
                if (wy[iy] == 0.0f) continue;
                const float *tri = tr[iy], *sri = sr[iy];
                for (int ix = 0; ix < 4; ix++) {
                    float ad = fabsf(g0 - tri[sxr[ix]]);
                    int ai = (int)(ad + 0.5f);
                    if (ai >= TOPEDGE_LUTN) ai = TOPEDGE_LUTN - 1;
                    float w = wx[ix] * wy[iy] * lut[ai];
                    num += w * sri[sxr[ix]];
                    den += w;
                }
            }
            if (den < 1e-6f && den > -1e-6f) continue;
            nact++;
            float v0 = drow[X];
            drow[X] = v0 + conf * j->mix * ((num / den) - v0);
        }
    }
    j->nact = nact; j->ntot = ntot;
    return NULL;
}

/* 就地修正 dst（内容已是双三次结果）。sigma/elo/ehi 单位 = raw 灰度。 */
static void topedge_up_2x(const float *src, const float *t2, int hw, int hh,
                          const uint16_t *top, int topw, float blk2,
                          float *dst, double sigma, double mix,
                          double elo, double ehi, double ar)
{
    enum { TN = 8 };
    topedge_preinit(sigma);
    const int eh = hh * 2;
    TopEdgeJob jobs[TN];
    for (int i = 0; i < TN; i++)
        jobs[i] = (TopEdgeJob){ .src = src, .t2 = t2, .top = top,
                                .hw = hw, .hh = hh, .topw = topw,
                                .blk2 = blk2, .dst = dst,
                                .mix = (float)mix, .elo = (float)elo,
                                .ehi = (float)ehi, .ar = (float)ar,
                                .y0 = i * eh / TN, .y1 = (i + 1) * eh / TN };
    run_jobs(topedge_job, jobs, sizeof(TopEdgeJob), TN);
    long na = 0, nt = 0;
    for (int i = 0; i < TN; i++) { na += jobs[i].nact; nt += jobs[i].ntot; }
    if (getenv("X3F_OWN_TOPEDGE_STAT"))
        fprintf(stderr, "[topedge] sigma=%.1f mix=%.3f elo=%.0f ehi=%.0f "
                "touched=%ld/%ld (%.3f%%)\n",
                sigma, mix, elo, ehi, na, nt,
                nt ? 100.0 * (double)na / (double)nt : 0.0);
}

/* 开关读取 + 两通道调用（默认链与回退链共用）。未开启时不读任何像素。 */
static void maybe_topedge(float *upB, float *upG,
                          const float *dB, const float *dG, const float *t2,
                          int hw, int hh, const uint16_t *top, int topw,
                          float blk2, double tclip)
{
    const char *te = getenv("X3F_OWN_TOPEDGE");
    if (!te || atoi(te) == 0) return;
    if (!(tclip > 0.0)) tclip = 16383.0;
    /* 2026-10-31 实测（sigma_dp3_quattro_09 楼顶 + DP2Q6563 双素材）:
     *   sigma=0.06·Tclip → 与 SPP 的差距仅 −0.8%（几乎空转）
     *   sigma=0.02·Tclip → −2.6%
     *   sigma=0.01·Tclip → −4.9%，目视最干净（无振铃/无新色边），
     *     平坦区高通 std 前后一致（噪点未增加），低频色斑 +0.2%
     * ⇒ 默认取 0.01。再小会有过度贴边的风险，需要另跑一轮目检。 */
    double sigr = 0.01, mix = 1.0, elor = 0.015, ehir = 0.06, ar = 0.5;
    const char *p;
    if ((p = getenv("X3F_OWN_TOPEDGE_S"))   && atof(p) > 0.0) sigr = atof(p);
    if ((p = getenv("X3F_OWN_TOPEDGE_MIX")))                 mix  = atof(p);
    if ((p = getenv("X3F_OWN_TOPEDGE_LO"))  && atof(p) >= 0.0) elor = atof(p);
    if ((p = getenv("X3F_OWN_TOPEDGE_HI"))  && atof(p) > 0.0) ehir = atof(p);
    if ((p = getenv("X3F_OWN_TOPEDGE_AR"))  && atof(p) >= 0.0) ar   = atof(p);
    topedge_up_2x(dB, t2, hw, hh, top, topw, blk2, upB,
                  sigr * tclip, mix, elor * tclip, ehir * tclip, ar);
    topedge_up_2x(dG, t2, hw, hh, top, topw, blk2, upG,
                  sigr * tclip, mix, elor * tclip, ehir * tclip, ar);
    stage_t("  fuse: top-edge upsample");
}

static inline uint16_t sat_u16(double v)
{
    double r = round(v);
    if (r <= 0.0) return 0;
    if (r >= 65535.0) return 65535;
    return (uint16_t)r;
}

/* 暗列修复：Quattro 传感器每 16 列读出块含 1 根遮蔽列（值≈黑电平-350），
 * 另有若干键合区暗列簇。官方管线将其修复（参考 x3f_extract 输出无周期暗列，
 * 且与 SPP 导出构图逐像素对齐排除跳列重映射）。
 * 检测：逐列中位数 vs 滑动基线（中位数对占用<50%行的场景竖直结构鲁棒）；
 * 修复：暗列各像素由最近非暗列线性插值。 */
static int cmp_f64(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

/* 就地快速选择: 返回排序后第 k 小的元素（打乱数组, 与 qsort+取[n/2]等价） */
static double dselect(double *a, int n, int k)
{
    int lo = 0, hi = n - 1;
    while (lo < hi) {
        double pivot = a[(lo + hi) >> 1];
        int i = lo, j = hi;
        while (i <= j) {
            while (a[i] < pivot) i++;
            while (a[j] > pivot) j--;
            if (i <= j) {
                double t = a[i]; a[i] = a[j]; a[j] = t;
                i++; j--;
            }
        }
        if (k <= j) hi = j;
        else if (k >= i) lo = i;
        else break;
    }
    return a[k];
}

static float fselect(float *a, int n, int k)
{
    int lo = 0, hi = n - 1;
    while (lo < hi) {
        float pivot = a[(lo + hi) >> 1];
        int i = lo, j = hi;
        while (i <= j) {
            while (a[i] < pivot) i++;
            while (a[j] > pivot) j--;
            if (i <= j) {
                float t = a[i]; a[i] = a[j]; a[j] = t;
                i++; j--;
            }
        }
        if (k <= j) hi = j;
        else if (k >= i) lo = i;
        else break;
    }
    return a[k];
}
/* 逐列中位数（并行 4 线程, 列区间独立）。starty/stepy 为行采样方式。 */
typedef struct {
    const uint16_t *plane; int nch, ch, w, h, nsamp;
    int starty, stepy;
    double *col; int x0, x1;
} ColMedJob;

static void *col_med_job(void *arg)
{
    ColMedJob *j = (ColMedJob *)arg;
    double *buf = malloc((size_t)j->nsamp * sizeof(double));
    if (!buf) die("out of memory");
    for (int x = j->x0; x < j->x1; x++) {
        int n = 0;
        for (int y = j->starty; y < j->h - 2; y += j->stepy)
            buf[n++] = j->plane[((size_t)y * j->w + x) * j->nch + j->ch];
        j->col[x] = dselect(buf, n, n / 2);
    }
    free(buf);
    return NULL;
}

static void col_med_parallel(const uint16_t *plane, int nch, int ch,
                             int w, int h, int nsamp,
                             int starty, int stepy, double *col)
{
    enum { NT = 4 };
    ColMedJob jobs[NT];
    pthread_t th[NT];
    for (int t = 0; t < NT; t++) {
        jobs[t].plane = plane; jobs[t].nch = nch; jobs[t].ch = ch;
        jobs[t].w = w; jobs[t].h = h; jobs[t].nsamp = nsamp;
        jobs[t].starty = starty; jobs[t].stepy = stepy;
        jobs[t].col = col;
        jobs[t].x0 = t * w / NT;
        jobs[t].x1 = (t + 1) * w / NT;
        if (t && pthread_create(&th[t], NULL, col_med_job, &jobs[t]) != 0)
            jobs[t].x1 = jobs[t].x0;      /* 主线程接管该区间 */
    }
    col_med_job(&jobs[0]);
    for (int t = 1; t < NT; t++)
        if (jobs[t].x1 > jobs[t].x0) pthread_join(th[t], NULL);
}

/* 逐列「分带中位数」（并行）：把采样行按高度等分 nb 带，每列每带各求
 * 中位数 → bmed[b*w + x]。供死列检测的行带一致性校验用
 * （见 repair_defect_columns_u16 内 a2 段注释）。 */
typedef struct {
    const uint16_t *plane; int nch, ch, w, h, nsamp, nb;
    int starty, stepy;
    double *bmed; int x0, x1;
} ColBandJob;

static void *col_band_job(void *arg)
{
    ColBandJob *j = (ColBandJob *)arg;
    int per = j->nsamp / j->nb + 2;
    double *buf = malloc((size_t)per * sizeof(double));
    if (!buf) die("out of memory");
    for (int x = j->x0; x < j->x1; x++) {
        for (int b = 0; b < j->nb; b++) {
            int y0 = (int)((long)j->h * b / j->nb);
            int y1 = (int)((long)j->h * (b + 1) / j->nb);
            int n = 0;
            for (int y = y0 + j->starty; y < y1 - 2 && y < j->h - 2; y += j->stepy)
                buf[n++] = j->plane[((size_t)y * j->w + x) * j->nch + j->ch];
            j->bmed[(size_t)b * j->w + x] = n ? dselect(buf, n, n / 2) : 0.0;
        }
    }
    free(buf);
    return NULL;
}

static void col_band_med_parallel(const uint16_t *plane, int nch, int ch,
                                  int w, int h, int nsamp,
                                  int starty, int stepy, int nb, double *bmed)
{
    enum { NT = 4 };
    ColBandJob jobs[NT];
    pthread_t th[NT];
    for (int t = 0; t < NT; t++) {
        jobs[t].plane = plane; jobs[t].nch = nch; jobs[t].ch = ch;
        jobs[t].w = w; jobs[t].h = h; jobs[t].nsamp = nsamp; jobs[t].nb = nb;
        jobs[t].starty = starty; jobs[t].stepy = stepy; jobs[t].bmed = bmed;
        jobs[t].x0 = t * w / NT;
        jobs[t].x1 = (t + 1) * w / NT;
        if (t && pthread_create(&th[t], NULL, col_band_job, &jobs[t]) != 0)
            jobs[t].x1 = jobs[t].x0;
    }
    col_band_job(&jobs[0]);
    for (int t = 1; t < NT; t++)
        if (jobs[t].x1 > jobs[t].x0) pthread_join(th[t], NULL);
}

/* 孤立点状坏点修复（2026-10-21）。
 *
 * 实测 DP2Q1884 在 top(B) 平面有孤立暗点：比 4 邻域最小值还低 500~900 raw
 * （半层 R/G 偶发同样形态）。x3fuse 路径有 fix_bad=1 覆盖这类点，自研路径
 * 一直没有 → 渲染后在 bokeh 背景上表现为 1~2px 黑点。
 *
 * 判定刻意保守，绝不触碰真实纹理/边缘：
 *   暗: c < min(4邻域) − dark_thr；亮: c > max(4邻域) + hot_thr；
 *   且 4 邻域自身一致 (max−min < cons_thr)——锐边上邻域跨度大，直接跳过，
 *   边上的坏点宁可留着（bokeh 平滑区才是可见黑点的主要来源）。
 * 修复 = 4 邻域中位数。两遍执行（先标记后填充），无级联。
 * 只处理 (1..w-2, 1..h-2) 内部像素。 */
typedef struct {
    const uint16_t *plane; int nch, ch, w, h;
    double dark_thr, hot_thr, cons_thr;
    unsigned char *mark;
} PointJob;

static void repair_point_defects_u16(uint16_t *plane, int nch, int ch,
                                     int w, int h,
                                     double dark_thr, double hot_thr,
                                     double cons_thr)
{
    unsigned char *mark = calloc((size_t)w * h, 1);
    if (!mark) return;
#define PX(y, x) plane[((size_t)(y) * w + (x)) * nch + ch]
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            double c = PX(y, x);
            /* 8 邻域联合检测：真坏点四面八方都正常；软边缘/纹理总有一侧
             * 邻居与中心过渡 —— 4 邻域版在软边缘误触发率极高（实测 15 万
             * 次/张，每个修复点经 2x 上采样留下 2×2 平滑补丁 → 点纹）。
             * 8 邻域版把触发压到真坏点量级（千级）。 */
            double n[8] = {
                PX(y - 1, x - 1), PX(y - 1, x), PX(y - 1, x + 1),
                PX(y,     x - 1),               PX(y,     x + 1),
                PX(y + 1, x - 1), PX(y + 1, x), PX(y + 1, x + 1)
            };
            double nmin = n[0], nmax = n[0];
            for (int k = 1; k < 8; k++) {
                if (n[k] < nmin) nmin = n[k];
                if (n[k] > nmax) nmax = n[k];
            }
            if (nmax - nmin >= cons_thr) continue;   /* 邻域跨度大 = 边缘/纹理 */
            if (c < nmin - dark_thr || c > nmax + hot_thr)
                mark[(size_t)y * w + x] = 1;
        }
    }
    int nrep = 0;
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            if (!mark[(size_t)y * w + x]) continue;
            /* 填充用 4 正交邻域均值（与检测解耦，正交邻居最可信） */
            double mean = (PX(y - 1, x) + PX(y + 1, x) +
                           PX(y, x - 1) + PX(y, x + 1)) * 0.25;
            PX(y, x) = (uint16_t)(mean + 0.5);
            nrep++;
        }
    }
    if (nrep && getenv("X3FDEC_PLANEDBG"))
        fprintf(stderr, "point defects repaired: %d (w=%d)\n", nrep, w);
#undef PX
    free(mark);
}

/* float 版点修复：用于 fuse_quattro 内 delta 域（列修复之后，
 * 只剩真正的孤立点缺陷）。判定/填充与 u16 版相同。 */
static void repair_point_defects_f(float *plane, int nch, int ch,
                                   int w, int h,
                                   double dark_thr, double hot_thr,
                                   double cons_thr)
{
    unsigned char *mark = calloc((size_t)w * h, 1);
    if (!mark) return;
#define PF(y, x) plane[((size_t)(y) * w + (x)) * nch + ch]
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            float c = PF(y, x);
            float n[8] = {
                PF(y - 1, x - 1), PF(y - 1, x), PF(y - 1, x + 1),
                PF(y,     x - 1),               PF(y,     x + 1),
                PF(y + 1, x - 1), PF(y + 1, x), PF(y + 1, x + 1)
            };
            float nmin = n[0], nmax = n[0];
            for (int k = 1; k < 8; k++) {
                if (n[k] < nmin) nmin = n[k];
                if (n[k] > nmax) nmax = n[k];
            }
            if (nmax - nmin >= (float)cons_thr) continue;
            if (c < nmin - (float)dark_thr || c > nmax + (float)hot_thr)
                mark[(size_t)y * w + x] = 1;
        }
    }
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            if (!mark[(size_t)y * w + x]) continue;
            PF(y, x) = (PF(y - 1, x) + PF(y + 1, x) +
                        PF(y, x - 1) + PF(y, x + 1)) * 0.25f;
        }
    }
#undef PF
    free(mark);
}

static void repair_dark_columns_u16(uint16_t *plane, int nch, int ch,
                                    int w, int h, double thresh)
{
    int nsamp = 0;
    for (int y = 2; y < h - 2; y += 3) nsamp++;
    double *col = malloc((size_t)w * sizeof(double));
    double *buf = malloc((size_t)nsamp * sizeof(double));
    double *base = malloc((size_t)w * sizeof(double));
    unsigned char *mask = calloc(w, 1);
    if (!col || !buf || !base || !mask) die("out of memory");

    for (int x = 0; x < w; x++) {
        int n = 0;
        for (int y = 2; y < h - 2; y += 3)
            buf[n++] = plane[((size_t)y * w + x) * nch + ch];
        qsort(buf, n, sizeof(double), cmp_f64);
        col[x] = buf[n / 2];
    }
    for (int x = 0; x < w; x++) {
        int l = x - 12 < 0 ? 0 : x - 12;
        int r = x + 12 >= w ? w - 1 : x + 12;
        double s = 0;
        for (int i = l; i <= r; i++) s += col[i];
        base[x] = s / (r - l + 1);
        mask[x] = col[x] < base[x] - thresh;
    }
    /* 逐行插值暗列 */
    for (int y = 0; y < h; y++) {
        uint16_t *row = plane + (size_t)y * w * nch + ch;
        int x = 0;
        while (x < w) {
            if (!mask[x]) { x++; continue; }
            int s = x;
            while (x < w && mask[x]) x++;
            int e = x - 1;                  /* 暗列区间 [s,e] */
            int xl = s - 1, xr = e + 1;
            double vl, vr;
            if (xl < 0)  vl = xr < w ? row[xr * nch] : 0;
            else         vl = row[xl * nch];
            if (xr >= w) vr = xl >= 0 ? row[xl * nch] : vl;
            else         vr = row[xr * nch];
            for (int i = s; i <= e; i++) {
                double t = (e == s) ? 0.5 : (double)(i - s) / (e - s);
                double v = vl + (vr - vl) * t;
                row[i * nch] = v < 0 ? 0 : (v > 16383 ? 16383 : (uint16_t)v);
            }
        }
    }
    if (getenv("X3FDEC_PLANEDBG")) {
        int nd = 0;
        for (int x = 0; x < w; x++) nd += mask[x];
        fprintf(stderr, "repair ch%d: %d dark cols\n", ch, nd);
    }
    free(col); free(buf); free(base); free(mask);
}

/* 缺陷列修复（半层键合区阴影列）。
 * 实测（DP2Q1884）: 半层 R/G 存在成簇的点状暗列——精确每 16 列 1 根
 * （x≡0 mod16），仅部分行发暗（点状，半数行亏损>50），行中位数亏损小，
 * "中位数 < 基线-thresh"检测漏掉；经 2x 放大与 CCM 大系数后成为全画面
 * 点状竖纹/绿紫色斑。
 * 检测（两路合并）:
 *   a) 行中位数亏损 > dead_thr（整列死列）；
 *   b) 点状列: 在"局部平滑行"（x±3 对比度 < smooth_thr）上统计
 *      hp = v[x] − (v[x−3]+v[x+3])/2 < −dot_thr 的频率，超 frac 即缺陷列
 *      （只统计平滑行，竖直边缘两侧的列不会被误判）。
 * 修复: 掩膜列（±1 膨胀）逐行由最近正常列线性插值。 */
static void repair_defect_columns_u16(uint16_t *plane, int nch, int ch,
                                      int w, int h,
                                      double dead_thr, double dot_thr,
                                      double smooth_thr, double frac_thr)
{
    int nsamp = 0;
    for (int y = 2; y < h - 2; y += 2) nsamp++;
    double *col = malloc((size_t)w * sizeof(double));
    double *buf = malloc((size_t)nsamp * sizeof(double));
    double *base = malloc((size_t)w * sizeof(double));
    double *frac = calloc((size_t)w, sizeof(double));
    unsigned char *mask = calloc(w, 1);
    if (!col || !buf || !base || !frac || !mask) die("out of memory");

    /* a) 行中位数亏损（死列, 并行 gather） */
    col_med_parallel(plane, nch, ch, w, h, nsamp, 2, 2, col);
    for (int x = 0; x < w; x++) {
        int l = x - 12 < 0 ? 0 : x - 12;
        int r = x + 12 >= w ? w - 1 : x + 12;
        double s = 0;
        for (int i = l; i <= r; i++) s += col[i];
        base[x] = s / (r - l + 1);
        mask[x] = col[x] < base[x] - dead_thr;
    }
    /* a2) 行带一致性校验（2026-10-21 紫边根因修复）。
     *
     * 内容竖直边界（如楼剪影与天空的交界）会让暗侧列的整列中位数被楼占的
     * 行段拖低，而 25 列基线混入亮侧列 → 亏损超阈被误判"死列"；随后逐行
     * 水平插值把真实边缘抹平。该列进 t2（顶层 2×2 均值）后成为 dB/dG 的
     * 共模误差 —— YUV 融合 (U=2·dB, V=dB−2·dG) 抵不掉共模，上采样后就是
     * 沿边缘的紫/绿色条（SDQ_0665_1 楼边缘 9 连列误判，B−G≈+55）。
     *
     * 判据：真缺陷列的亏损与内容无关、全高一致；内容边界的亏损只出现在
     * 含暗结构的行带。把采样行按高度等分 8 带，逐带算「带内列中位数 −
     * 带内 25 列基线」，仅当 ≥6/8 带亏损都超 0.6·dead_thr 才确认死列。
     * 实测 SDQ_0665_1：真死列 col199-203（59% 黑行散布全高）保留，
     * 内容边界 col1461-1469（亏损只在楼的行段）被排除。 */
    {
        enum { NB = 8, NEED = 6 };
        double *bmed = malloc((size_t)NB * w * sizeof(double));
        if (!bmed) die("out of memory");
        col_band_med_parallel(plane, nch, ch, w, h, nsamp, 2, 2, NB, bmed);
        for (int x = 0; x < w; x++) {
            if (!mask[x]) continue;
            int l = x - 12 < 0 ? 0 : x - 12;
            int r = x + 12 >= w ? w - 1 : x + 12;
            int ok = 0;
            for (int b = 0; b < NB; b++) {
                double s = 0;
                for (int i = l; i <= r; i++) s += bmed[(size_t)b * w + i];
                if (s / (r - l + 1) - bmed[(size_t)b * w + x] > dead_thr * 0.6)
                    ok++;
            }
            if (ok < NEED) mask[x] = 0;
        }
        free(bmed);
    }
    /* b) 点状列: 平滑行上的高频亏损频率 */
    for (int y = 2; y < h - 2; y += 2) {
        const uint16_t *row = plane + (size_t)y * w * nch + ch;
        for (int x = 3; x < w - 3; x++) {
            double vl = row[(x - 3) * nch], vr = row[(x + 3) * nch], v = row[x * nch];
            double lo = vl < vr ? vl : vr, hi = vl < vr ? vr : vl;
            if (hi - lo > smooth_thr) continue;          /* 邻域有边缘, 跳过 */
            if (v - 0.5 * (vl + vr) < -dot_thr)
                frac[x] += 1.0;
        }
    }
    for (int x = 3; x < w - 3; x++)
        if (frac[x] > frac_thr * nsamp) mask[x] = 1;
    /* 掩膜膨胀 1 列 */
    {
        unsigned char *m2 = malloc(w);
        if (!m2) die("out of memory");
        for (int x = 0; x < w; x++)
            m2[x] = mask[x] || (x > 0 && mask[x - 1]) || (x + 1 < w && mask[x + 1]);
        memcpy(mask, m2, w);
        free(m2);
    }
    /* 逐行插值掩膜列 */
    for (int y = 0; y < h; y++) {
        uint16_t *row = plane + (size_t)y * w * nch + ch;
        int x = 0;
        while (x < w) {
            if (!mask[x]) { x++; continue; }
            int s = x;
            while (x < w && mask[x]) x++;
            int e = x - 1;                  /* 暗列区间 [s,e] */
            int xl = s - 1, xr = e + 1;
            double vl, vr;
            if (xl < 0)  vl = xr < w ? row[xr * nch] : 0;
            else         vl = row[xl * nch];
            if (xr >= w) vr = xl >= 0 ? row[xl * nch] : vl;
            else         vr = row[xr * nch];
            for (int i = s; i <= e; i++) {
                double t = (e == s) ? 0.5 : (double)(i - s) / (e - s);
                double v = vl + (vr - vl) * t;
                row[i * nch] = v < 0 ? 0 : (v > 16383 ? 16383 : (uint16_t)v);
            }
        }
    }
    if (getenv("X3FDEC_PLANEDBG")) {
        int nd = 0, ndot = 0;
        fprintf(stderr, "defect ch%d masked cols:", ch);
        for (int x = 0; x < w; x++)
            if (mask[x]) { fprintf(stderr, " %d", x); nd++; }
        for (int x = 0; x < w; x++) ndot += frac[x] > frac_thr * nsamp;
        fprintf(stderr, "\n  => %d cols masked (%d dotted) w=%d\n", nd, ndot, w);
        /* 每根掩膜列的检测数值：中位数 / 25列基线 / 亏损 */
        for (int x = 0; x < w; x++)
            if (mask[x])
                fprintf(stderr, "  col %d: med=%.0f base=%.0f deficit=%.0f%s\n",
                        x, col[x], base[x], base[x] - col[x],
                        col[x] < base[x] - dead_thr ? " (dead)" : "");
    }
    free(col); free(buf); free(base); free(frac); free(mask);
}

/* 列平坦场校正（u16 平面，在计算 delta 前对原始层做）。
 * 实测: 半层 R/G 有 16 列读出周期 FPN（幅度 ~27@天空均值5659，随亮度缩放→增益型），
 * 顶层 B 有奇偶列交替（P=2，幅度 ~11-15）。增益型分量无法靠扣全局中位数消除。
 * 方法: level[x] = 行采样中位数; baseline[x] = 9 列滑动均值;
 *       mode=0: v += clamp(baseline-level, ±cap)      （偏移型）
 *       mode=1: v *= clamp(baseline/level, 1±cap/level)（增益型，随亮度缩放）
 * 中位数对竖直结构（桥塔等）鲁棒。 */
static void column_flatfield_u16(uint16_t *plane, int nch, int ch, int w, int h,
                                 double cap, int gainmode)
{
    int nsamp = 0;
    for (int y = 1; y < h - 1; y += 2) nsamp++;
    double *col = malloc((size_t)w * sizeof(double));
    double *buf = malloc((size_t)nsamp * sizeof(double));
    double *base = malloc((size_t)w * sizeof(double));
    if (!col || !buf || !base) die("out of memory");

    col_med_parallel(plane, nch, ch, w, h, nsamp, 1, 2, col);
    for (int x = 0; x < w; x++) {
        /* 基线窗口 33 列: 必须远宽于 FPN 周期(16)，否则基线保留周期图案、校正量为零 */
        int l = x - 16 < 0 ? 0 : x - 16;
        int r = x + 16 >= w ? w - 1 : x + 16;
        double s = 0;
        for (int i = l; i <= r; i++) s += col[i];
        base[x] = s / (r - l + 1);
    }
    /* 全局列电平中位数: 增益型 FPN 归一化基准（v = true·(1+amp) → true = v/(1+amp)） */
    double *csorted = malloc((size_t)w * sizeof(double));
    if (!csorted) die("out of memory");
    memcpy(csorted, col, (size_t)w * sizeof(double));
    qsort(csorted, w, sizeof(double), cmp_f64);
    double glev = csorted[w / 2];
    free(csorted);
    for (int y = 0; y < h; y++) {
        uint16_t *row = plane + (size_t)y * w * nch + ch;
        for (int x = 0; x < w; x++) {
            double v = row[x];
            if (gainmode) {
                double amp = (col[x] - base[x]) / (glev > 1 ? glev : 1);
                double ac = cap / (glev > 1 ? glev : 1);
                if (amp > ac) amp = ac; else if (amp < -ac) amp = -ac;
                double nv = v / (1.0 + amp);
                row[x] = nv < 0 ? 0 : (nv > 65535 ? 65535 : (uint16_t)(nv + 0.5));
            } else {
                double d = base[x] - col[x];
                if (d > cap) d = cap; else if (d < -cap) d = -cap;
                double nv = v + d;
                row[x] = nv < 0 ? 0 : (nv > 65535 ? 65535 : (uint16_t)(nv + 0.5));
            }
        }
    }
    if (getenv("X3FDEC_PLANEDBG")) {
        double mx = 0;
        for (int x = 0; x < w; x++) {
            double d = base[x] - col[x];
            if (fabs(d) > fabs(mx)) mx = d;
        }
        fprintf(stderr, "flatfield ch%d w=%d: max col correction %.1f (cap %.0f, %s)\n",
                ch, w, mx, cap, gainmode ? "gain" : "offset");
    }
    free(col); free(buf); free(base);
}

/* 列固定图案噪声（FPN）校正：半层各列增益/偏移不均，2x 放大后成竖条纹。
 * 逐列取水平高通（v[x]-(v[x-2]+v[x+2])/2）的中位数作为列偏移并扣除。
 * 中位数对场景内容鲁棒（少量竖直边缘不会带偏）。 */
/* 周期锁定列纹校正。实测 Quattro 顶层有 P=2 奇偶交替（幅度随行缓变）。
 * 每行带求列均值 profile; delta[x] = profile[x] − (profile[x−P]+profile[x+P])/2
 * （周期 P 高通，平滑场景内容全消）; 按 x mod P 折叠平均得该带图案;
 * 行带间三点平滑后逐像素扣除。 */
static void column_period_correct_u16(uint16_t *plane, int nch, int ch, int w, int h,
                                      int P, int band, double cap)
{
    int nb = (h + band - 1) / band;
    double *corr = calloc((size_t)nb * P, sizeof(double));
    double *scorr = malloc((size_t)nb * P * sizeof(double));
    double *prof = malloc((size_t)w * sizeof(double));
    if (!corr || !scorr || !prof) die("out of memory");

    for (int b = 0; b < nb; b++) {
        int y0 = b * band, y1 = y0 + band < h ? y0 + band : h;
        memset(prof, 0, (size_t)w * sizeof(double));
        for (int y = y0; y < y1; y++) {
            const uint16_t *row = plane + (size_t)y * w * nch + ch;
            for (int x = 0; x < w; x++) prof[x] += row[x];
        }
        double inv = 1.0 / (y1 - y0);
        for (int x = 0; x < w; x++) prof[x] *= inv;
        double *ph = corr + (size_t)b * P;
        int cnt[64];
        for (int k = 0; k < P; k++) { ph[k] = 0; cnt[k] = 0; }
        for (int x = 0; x < w; x++) {
            int xl = x >= P ? x - P : x + P;
            int xr = x + P < w ? x + P : x - P;
            double d = prof[x] - 0.5 * (prof[xl] + prof[xr]);
            ph[x % P] += d; cnt[x % P]++;
        }
        for (int k = 0; k < P; k++) {
            ph[k] /= cnt[k] > 0 ? cnt[k] : 1;
            if (ph[k] > cap) ph[k] = cap; else if (ph[k] < -cap) ph[k] = -cap;
        }
    }
    for (int b = 0; b < nb; b++)
        for (int k = 0; k < P; k++) {
            double a = corr[(size_t)b * P + k];
            double prev = b > 0 ? corr[(size_t)(b - 1) * P + k] : a;
            double next = b + 1 < nb ? corr[(size_t)(b + 1) * P + k] : a;
            scorr[(size_t)b * P + k] = 0.25 * prev + 0.5 * a + 0.25 * next;
        }
    for (int y = 0; y < h; y++) {
        const double *ph = scorr + (size_t)(y / band) * P;
        uint16_t *row = plane + (size_t)y * w * nch + ch;
        for (int x = 0; x < w; x++) {
            double nv = row[x] - ph[x % P];
            row[x] = nv < 0 ? 0 : (nv > 65535 ? 65535 : (uint16_t)(nv + 0.5));
        }
    }
    if (getenv("X3FDEC_PLANEDBG")) {
        double mx = 0;
        for (size_t i = 0; i < (size_t)nb * P; i++)
            if (fabs(corr[i]) > fabs(mx)) mx = corr[i];
        fprintf(stderr, "periodcorr ch%d P=%d band=%d: max pattern amp %.1f (cap %.0f)\n",
                ch, P, band, mx, cap);
    }
    free(corr); free(scorr); free(prof);
}

/* 分行带列 FPN 校正：列噪声随行缓变，全局中位数低估局部幅度。
 * 每带(128行)估计逐列偏移（±2 高通的中位数），列向 1-2-1 平滑，
 * 带间 0.25/0.5/0.25 平滑后扣除。
 * 双重结构门控（2026-10-25，DP2Q2501 窗台彩条实锤后加入）：
 * 1) 带内显著性：|median| >= T1·1.2533·σ/√n（σ=MAD/0.6745，中位数
 *    标准误）才保留——拦掉带内高通以噪声为主的偶然偏移；
 * 2) 跨带一致性：真 FPN 是传感器列属性，所有带偏移相近；场景竖直边
 *    （窗框）只在其穿过的带里产生一致但巨大的偏移（半层列 2108-2122
 *    的 sill 带实测 ±430，其余带 0）。逐列取带间中位数 gmed 与带间
 *    散布 gs，偏移偏离 gmed 超过 T2·(带内SE + 0.324·gs) 的带回退到
 *    gmed——结构边被丢弃，真 FPN 照常修正。
 * 只用门 1 拦不住竖直结构边（带内 MAD 小、显著性高）。X3FDEC_FPNT /
 * _FPNT2 覆盖门限（默认 6/8），X3FDEC_FPNGATE=0 关闭门控回旧行为。 */
/* 列 FPN 统计的第一段：逐带、逐列求「列中值 - 左右邻均值」的中值。
 * 每条带完全独立（只写自己那行 corr/madA），按带切块并行；块内 buf/dev
 * 必须每线程各一份（fselect 会原地排序）。 */
typedef struct {
    const float *img; int w, h, band, nb;
    float *corr, *madA;
    int use_gate; double gate_t;
} FpnP1Ctx;

static void fpn_p1_bands(void *v, int b0, int b1)
{
    FpnP1Ctx *ctx = (FpnP1Ctx *)v;
    for (int b = b0; b < b1; b++) {
        int y0 = b * ctx->band;
        int y1 = y0 + ctx->band < ctx->h ? y0 + ctx->band : ctx->h;
        int cap = y1 > y0 ? y1 - y0 : 1;
        float *c = ctx->corr + (size_t)b * ctx->w;
        float *buf = (float *)malloc((size_t)cap * sizeof(float));
        float *dev = ctx->use_gate ? (float *)malloc((size_t)cap * sizeof(float))
                                   : NULL;
        if (!buf || (ctx->use_gate && !dev)) { free(buf); free(dev); die("out of memory"); }
        for (int x = 0; x < ctx->w; x++) {
            int n = 0;
            for (int y = y0; y < y1; y++) {
                const float *row = ctx->img + (size_t)y * ctx->w;
                int xl = x >= 2 ? x - 2 : 0;
                int xr = x + 2 < ctx->w ? x + 2 : ctx->w - 1;
                buf[n++] = row[x] - 0.5f * (row[xl] + row[xr]);
            }
            float med = fselect(buf, n, n / 2);
            if (ctx->use_gate) {
                for (int i = 0; i < n; i++) dev[i] = fabsf(buf[i] - med);
                float mad = fselect(dev, n, n / 2);
                ctx->madA[(size_t)b * ctx->w + x] = mad;
                float sig = mad * 1.4826f;                    /* MAD→σ */
                float se = 1.2533f * sig / sqrtf((float)n);   /* 中位数 SE */
                if (fabsf(med) < ctx->gate_t * se) med = 0;   /* 门1: 带内显著性 */
            }
            c[x] = med;
        }
        free(buf); free(dev);
    }
}

static void column_fpn_banded(float *img, int w, int h, int band, const char *tag)
{
    double gate_t = 6.0, gate_t2 = 8.0;
    int use_gate = 1;
    const char *ge = getenv("X3FDEC_FPNT");
    if (ge) gate_t = atof(ge);
    if ((ge = getenv("X3FDEC_FPNT2"))) gate_t2 = atof(ge);
    ge = getenv("X3FDEC_FPNGATE");
    if (ge && !atoi(ge)) use_gate = 0;
    int nb = (h + band - 1) / band;
    float *corr = calloc((size_t)nb * w, sizeof(float));
    float *scorr = malloc((size_t)nb * w * sizeof(float));
    float *madA = use_gate ? calloc((size_t)nb * w, sizeof(float)) : NULL;
    float *buf = malloc((size_t)band * sizeof(float));
    float *dev = use_gate ? malloc((size_t)band * sizeof(float)) : NULL;
    float *bb = use_gate ? malloc((size_t)nb * sizeof(float)) : NULL;
    if (!corr || !scorr || !buf || (use_gate && (!madA || !dev || !bb)))
        die("out of memory");
    {
        FpnP1Ctx p1c;
        p1c.img = img; p1c.w = w; p1c.h = h; p1c.band = band; p1c.nb = nb;
        p1c.corr = corr; p1c.madA = madA;
        p1c.use_gate = use_gate; p1c.gate_t = gate_t;
        run_rows(fpn_p1_bands, &p1c, nb, 1);
    }
    if (use_gate) {
        /* 门2: 跨带一致性。逐列 O(nb²) 中位数，nb≤16、w=2944，可承受 */
        for (int x = 0; x < w; x++) {
            for (int b = 0; b < nb; b++) bb[b] = corr[(size_t)b * w + x];
            float gmed = fselect(bb, nb, nb / 2);
            for (int b = 0; b < nb; b++) bb[b] = fabsf(corr[(size_t)b * w + x] - gmed);
            float gs = fselect(bb, nb, nb / 2) * 1.4826f;     /* 带间散布 */
            float se_g = 1.2533f * gs / sqrtf((float)nb);
            if (fabsf(gmed) < gate_t * se_g) gmed = 0;        /* gmed 本身须显著 */
            for (int b = 0; b < nb; b++) {
                float *c = corr + (size_t)b * w;
                float se_b = 1.2533f * madA[(size_t)b * w + x] * 1.4826f
                           / sqrtf((float)band);
                float thr = (float)gate_t2 * (se_b + se_g);
                if (fabsf(c[x] - gmed) > thr) c[x] = gmed;    /* 结构边→回退 gmed */
            }
        }
    }
    for (int b = 0; b < nb; b++) {
        float *c = corr + (size_t)b * w;
        /* 列向平滑 */
        float prev = c[0];
        for (int x = 0; x < w; x++) {
            float cur = c[x], next = x + 1 < w ? c[x + 1] : c[x];
            c[x] = 0.25f * prev + 0.5f * cur + 0.25f * next;
            prev = cur;
        }
    }
    /* 带间平滑 */
    for (int b = 0; b < nb; b++)
        for (int x = 0; x < w; x++) {
            float a = corr[(size_t)b * w + x];
            float p = b > 0 ? corr[(size_t)(b - 1) * w + x] : a;
            float q = b + 1 < nb ? corr[(size_t)(b + 1) * w + x] : a;
            scorr[(size_t)b * w + x] = 0.25f * p + 0.5f * a + 0.25f * q;
        }
    if (getenv("X3FDEC_FPNDBG")) {
        char path[512];
        snprintf(path, sizeof path, "%s.%s", getenv("X3FDEC_FPNDBG"), tag);
        FILE *df = fopen(path, "wb");
        if (df) {
            fwrite(&w, 4, 1, df); fwrite(&h, 4, 1, df); fwrite(&band, 4, 1, df);
            fwrite(scorr, 4, (size_t)nb * w, df);   /* 实际施加的校正（门控+平滑后） */
            fclose(df);
        }
    }
    for (int y = 0; y < h; y++) {
        const float *c = scorr + (size_t)(y / band) * w;
        float *row = img + (size_t)y * w;
        for (int x = 0; x < w; x++) row[x] -= c[x];
    }
    free(corr); free(scorr); free(buf); free(dev); free(madA); free(bb);
}
static void column_fpn(double *img, int w, int h)
{
    double *buf = malloc((size_t)h * sizeof(double));
    double *col = malloc((size_t)w * sizeof(double));
    if (!buf || !col) die("out of memory");
    for (int x = 0; x < w; x++) {
        int xl = x >= 2 ? x - 2 : 0;
        int xr = x + 2 < w ? x + 2 : w - 1;
        for (int y = 0; y < h; y++) {
            size_t row = (size_t)y * w;
            buf[y] = img[row + x] - 0.5 * (img[row + xl] + img[row + xr]);
        }
        qsort(buf, h, sizeof(double), cmp_f64);
        col[x] = buf[h / 2];    /* 中位数 */
    }
    /* 列偏移做轻度平滑，避免校正量本身引入列噪声 */
    for (int x = 0; x < w; x++) {
        int l = x > 0 ? x - 1 : 0, r = x + 1 < w ? x + 1 : w - 1;
        col[x] = 0.25 * col[l] + 0.5 * col[x] + 0.25 * col[r];
    }
    for (int y = 0; y < h; y++) {
        double *row = img + (size_t)y * w;
        for (int x = 0; x < w; x++)
            row[x] -= col[x];
    }
    free(buf); free(col);
}

/* 5x5 边缘保持双边滤波（去半层条纹噪声，保边缘）。
 * sigma_r 为值域σ；空间权重固定高斯。in/out 可同址。 */
enum { RLUT_N = 4 * 4096 };
static float rlut[RLUT_N];              /* 值域核 LUT（文件作用域, 预初始化防多线程竞态） */
static double rlut_sig = -1;

static void bilateral_preinit(double sigma_r)
{
    if (rlut_sig == sigma_r) return;
    float inv2r2 = (float)(-0.5 / (sigma_r * sigma_r));
    for (int i = 0; i < RLUT_N; i++) {
        float d = i * 0.25f;
        rlut[i] = expf(inv2r2 * d * d);
    }
    rlut_sig = sigma_r;
}

static void bilateral5(float *img, int w, int h, double sigma_r)
{
    const int R = 2;
    /* 值域核 LUT: exp(-d^2/(2σ^2)), |d| ∈ [0,4096), 步长 0.25, 线性插值。
     * 权重可忽略区（>6σ≈48）之外的精度: 相对误差 ≤ |x'|^2 h^2 / 8 ≈ 0.1% */
    /* lazy 填表段上锁（2026-10-23 并发解码）：首轮两线程同填会读到半成品 */
    LAZY_LOCK();
    if (rlut_sig != sigma_r) bilateral_preinit(sigma_r);
    LAZY_UNLOCK();
    float sw[5];
    for (int i = -R; i <= R; i++) {
        float d = i / (float)R;
        sw[i + R] = expf(-0.5f * d * d * 2.0f);     /* 空间 σ≈R/1.41 */
    }
    /* GPU 优先（X3FDEC_GPU=0 强制 CPU; X3FDEC_GPU_BILAT=0 单独关; 失败自动回退）。
     * rlut/sw 都是 C 侧 expf 产物 —— GPU 的 exp ulp 不同，必须原样上传。 */
    {
        const char *e = getenv("X3FDEC_GPU_BILAT");
        if (own_gpu_available() && (!e || e[0] != '0') &&
            own_gpu_bilateral5(img, w, h, (float)sigma_r,
                               rlut, RLUT_N, sw) == 0)
            return;
        if (getenv("X3FDEC_GPUDBG"))
            fprintf(stderr, "[gpu] bilateral fallback (b=%p)\n", (void*)img);
    }
    float *tmp = malloc((size_t)w * h * sizeof(float));
    if (!tmp) die("out of memory");
    /* 水平 pass -> tmp */
    for (int y = 0; y < h; y++) {
        const float *row = img + (size_t)y * w;
        float *trow = tmp + (size_t)y * w;
        for (int x = 0; x < w; x++) {
            float c = row[x], acc = 0, wsum = 0;
            for (int k = -R; k <= R; k++) {
                int xx = x + k;
                if (xx < 0) xx = 0; else if (xx >= w) xx = w - 1;
                float v = row[xx];
                float ad = v - c; if (ad < 0) ad = -ad;
                int wi = (int)(ad * 4.0f);
                float wv = wi + 1 < RLUT_N
                    ? rlut[wi] + (rlut[wi + 1] - rlut[wi]) * (ad * 4.0f - wi)
                    : 0.0f;
                float wgt = sw[k + R] * wv;
                acc += v * wgt; wsum += wgt;
            }
            trow[x] = acc / wsum;
        }
    }
    /* 垂直 pass -> img */
    for (int y = 0; y < h; y++) {
        float *orow = img + (size_t)y * w;
        for (int x = 0; x < w; x++) {
            float c = tmp[(size_t)y * w + x], acc = 0, wsum = 0;
            for (int k = -R; k <= R; k++) {
                int yy = y + k;
                if (yy < 0) yy = 0; else if (yy >= h) yy = h - 1;
                float v = tmp[(size_t)yy * w + x];
                float ad = v - c; if (ad < 0) ad = -ad;
                int wi = (int)(ad * 4.0f);
                float wv = wi + 1 < RLUT_N
                    ? rlut[wi] + (rlut[wi + 1] - rlut[wi]) * (ad * 4.0f - wi)
                    : 0.0f;
                float wgt = sw[k + R] * wv;
                acc += v * wgt; wsum += wgt;
            }
            orow[x] = acc / wsum;
        }
    }
    free(tmp);
}

/* 半层点状缺陷（键合区暗点/坏列, x≡0 mod16）的 delta 域修复。
 * 直接在 half 上插值会破坏半层边缘形状：缺陷列斜穿软边缘时,边缘感知
 * 插值把 3-4 列的渐变阶梯化成单侧跳变,与顶层的平滑边缘错位 → 融合
 * delta 出现 ±1000 虚假摆动 → 全分辨率边界绿/紫斑。
 * delta = half − t2 只含色度,平面平滑,线性插值即可正确重建,不触碰
 * 亮度边缘形状。d 为 2 通道交错平面 (dR,dG)；检测用两通道均值,
 * 修复时两通道联合选边,保证 R/G 一致。 */
/* 半层缺陷（键合区点状阴影列 + 死列）的 delta 域修复。
 *
 * 为什么不在 u16 域修：u16 列插值把斜穿软色彩边缘的 3-4 列渐变整列
 * 替换为单侧值（阶梯化），与顶层平滑边缘错位 → delta 出现 ±860~1000
 * 的虚假摆动 → 2x 上采样+CCM 后成为花瓣/花萼边界的紫/绿色斑。
 * delta = half − t2 只含色度、平面平滑，线性插值即可正确重建，
 * 不触碰亮度边缘形状。
 *
 * d: 两通道交错 (dB,dG) 平面。检测用两通道均值：
 *   a) 行中位数亏损 > dead_thr（整列死列）；
 *   b) 点状列: 平滑行（x±3 对比度 < smooth_thr）上
 *      v − (vl+vr)/2 < −dot_thr 的频率 > frac_thr。
 * 修复: 掩膜±1膨胀，逐行插值（线性为主; 锐色度边缘 step>60 时按
 * 两通道联合梯度选边，防止 R/G 独立选边导致 R−G 爆炸）。 */

/* F23 出厂标定坏点逐像素修复（借鉴 Kalpanika x3f interpolate_bad_pixels；
 * 2026-10-22 DP2Q6341 紫带根因对策）。
 * SPP/Kalpanika 用相机出厂标定表（CAMF BadPixelsChromaF23/BadPixelsLumaF23，
 * 行程编码 row,(col,code)*,0；code 为严重度标志，占位跳过）逐像素多轮邻域
 * 插值，从不做整列推断。键合阴影列的点状缺陷就在表内（ChromaF23 ≈17.4万点
 * ≈ 1836 行 × 184 周期列 × 50%）。统计式列检测在密草场景把贯通全高的暗色
 * 叶隙（half 920-923/941，全高中位亏损 101-150 恰超阈值）误判为死列，全高
 * 插值抹掉真实内容 → 2x 上采样后断续紫/绿竖带；顶层 u16 列修复（本文件
 * 无行带校验的路径）同理误伤（DP2Q6341 实证 173 列）。标定表法零误伤：
 * DP2Q6341 区域紫像素 25151→3927（fuse 参照 1530），DP2Q1884 茎边紫斑消失。
 * X3FDEC_NOCALIB=1 关闭（回落统计式修复，对照实验用） */
static int _Thread_local g_calib_chroma_done = 0;
static int _Thread_local g_calib_luma_done = 0;
/* dot-only 模式：出厂标定表已覆盖随机坏点时，统计式修复只保留点状
 * 列检测（键合阴影列点状缺陷不在标定表内，仍需修）；整列死列检测
 * 跳过（密草场景误伤内容 → 紫带，DP2Q6341 实证）
 * 这三个标志每帧解码开头重置、解码流程内读取 —— 并发解码各线程
 * 互不覆盖，必须线程局部（2026-10-23）。 */
static int _Thread_local g_defect_dot_only = 0;
static void repair_bad_pixels_f23(uint16_t *plane, int nch, int w, int h,
                                  const double *list, int listlen,
                                  const char *tag)
{
    typedef struct BP { int c, r; struct BP *next, *prev; } BP;
    uint32_t *vec = calloc(((size_t)w * h + 31) / 32, sizeof(uint32_t));
    if (!vec) die("out of memory");
    BP *head = NULL;
    int row = -1, nmark = 0, oob = 0;
    for (int i = 0; i < listlen; i++) {
        int v = (int)list[i];
        if (row < 0) { if (v != 0) row = v; continue; }
        if (v == 0) { row = -1; continue; }
        int c = v;
        i++;                              /* 第二个值 = 严重度 code, 跳过 */
        if (row < 0 || row >= h || c < 0 || c >= w) { oob++; continue; }
        size_t idx = (size_t)row * w + c;
        if (!(vec[idx >> 5] & (1u << (idx & 31)))) {
            BP *p = malloc(sizeof(BP));
            if (!p) die("out of memory");
            p->c = c; p->r = row;
            p->prev = NULL; p->next = head;
            if (head) head->prev = p;
            head = p;
            vec[idx >> 5] |= 1u << (idx & 31);
            nmark++;
        }
    }
    int pass = 0, nfixed = 0, nfail = 0, fix_corner = 0;
    while (head) {
        BP *fixed = NULL;
        int left = 0;
        static const int dx[4] = { -1, 1, 0, 0 };
        static const int dy[4] = { 0, 0, -1, 1 };
        for (BP *p = head; p; ) {
            BP *pn = p->next;
            const uint16_t *inp[4] = { NULL, NULL, NULL, NULL };
            int num = 0;
            for (int k = 0; k < 4; k++) {
                int c = p->c + dx[k], r = p->r + dy[k];
                if (c < 0 || c >= w || r < 0 || r >= h) continue;
                size_t idx = (size_t)r * w + c;
                if (!(vec[idx >> 5] & (1u << (idx & 31)))) {
                    inp[k] = plane + ((size_t)r * w + c) * nch;
                    num++;
                }
            }
            if (inp[0] && inp[1] && inp[2] && inp[3]) {
                /* 四邻全好 */
            } else if (inp[0] && inp[1]) {
                inp[2] = inp[3] = NULL; num = 2;        /* 左右线性 */
            } else if (inp[2] && inp[3]) {
                inp[0] = inp[1] = NULL; num = 2;        /* 上下线性 */
            } else if (fix_corner && num == 2) {
                /* 收尾角点 */
            } else {
                left++; p = pn; continue;
            }
            uint16_t *outp = plane + ((size_t)p->r * w + p->c) * nch;
            for (int ch = 0; ch < nch; ch++) {
                uint32_t sum = 0;
                for (int k = 0; k < 4; k++) if (inp[k]) sum += inp[k][ch];
                outp[ch] = (uint16_t)((sum + num / 2) / num);
            }
            if (p->prev) p->prev->next = p->next; else head = p->next;
            if (p->next) p->next->prev = p->prev;
            p->prev = NULL; p->next = fixed;
            fixed = p;
            p = pn;
        }
        if (!fixed) {
            if (!fix_corner) fix_corner = 1;
            else { nfail = left; break; }
        }
        for (BP *q = fixed; q; ) {
            BP *qn = q->next;
            size_t idx = (size_t)q->r * w + q->c;
            vec[idx >> 5] &= ~(1u << (idx & 31));
            free(q);
            nfixed++;
            q = qn;
        }
        pass++;
    }
    fprintf(stderr, "calib badpix %s: %d marked, %d fixed in %d passes, "
            "%d failed, %d oob\n", tag, nmark, nfixed, pass, nfail, oob);
    free(vec);
}

static void repair_defect_deltas(float *d, int w, int h,
                                 double dead_thr, double dot_thr,
                                 double smooth_thr, double frac_thr)
{
    int nsamp = 0;
    for (int y = 2; y < h - 2; y += 2) nsamp++;
    float *col = malloc((size_t)w * sizeof(float));
    float *buf = malloc((size_t)nsamp * sizeof(float));
    float *base = malloc((size_t)w * sizeof(float));
    float *frac = calloc((size_t)w, sizeof(float));
    unsigned char *mask = calloc(w, 1);
    if (!col || !buf || !base || !frac || !mask) die("out of memory");

    /* a) 行中位数亏损（死列）— dot-only 模式跳过 */
    if (!g_defect_dot_only)
    for (int x = 0; x < w; x++) {
        int n = 0;
        for (int y = 2; y < h - 2; y += 2)
            buf[n++] = 0.5f * (d[((size_t)y * w + x) * 2] + d[((size_t)y * w + x) * 2 + 1]);
        col[x] = fselect(buf, n, n / 2);
    }
    if (!g_defect_dot_only)
    for (int x = 0; x < w; x++) {
        int l = x - 12 < 0 ? 0 : x - 12;
        int r = x + 12 >= w ? w - 1 : x + 12;
        float s = 0;
        for (int i = l; i <= r; i++) s += col[i];
        base[x] = s / (r - l + 1);
        mask[x] = col[x] < base[x] - dead_thr;
    }
    /* b) 点状列: 平滑行上的高频亏损频率 */
    for (int y = 2; y < h - 2; y += 2) {
        const float *row = d + (size_t)y * w * 2;
        for (int x = 3; x < w - 3; x++) {
            float vl = 0.5f * (row[(x - 3) * 2] + row[(x - 3) * 2 + 1]);
            float vr = 0.5f * (row[(x + 3) * 2] + row[(x + 3) * 2 + 1]);
            float v  = 0.5f * (row[ x      * 2] + row[ x      * 2 + 1]);
            float lo = vl < vr ? vl : vr, hi = vl < vr ? vr : vl;
            if (hi - lo > smooth_thr) continue;
            if (v - 0.5f * (vl + vr) < -dot_thr)
                frac[x] += 1.0f;
        }
    }
    for (int x = 3; x < w - 3; x++)
        if (frac[x] > frac_thr * nsamp) mask[x] = 1;
    /* dot-only 模式：点状列上做**逐像素**修复（只修满足点缺陷条件的像素，
     * 不动列内其他内容、不膨胀、不整列插值）。键合阴影列点状缺陷不在
     * 出厂标定表内，需此步；而整列插值在密草高对比区会抹掉内容 → 紫/绿
     * 竖带（DP2Q6341 实证: 3 宽点列 span 也触发，region 紫像素 3927→16572） */
    if (g_defect_dot_only) {
        int ndotcol = 0, nfix = 0;
        for (int x = 3; x < w - 3; x++) ndotcol += mask[x];
        for (int y = 0; y < h; y++) {
            float *row = d + (size_t)y * w * 2;
            for (int x = 3; x < w - 3; x++) {
                if (!mask[x]) continue;
                float vl = 0.5f * (row[(x - 3) * 2] + row[(x - 3) * 2 + 1]);
                float vr = 0.5f * (row[(x + 3) * 2] + row[(x + 3) * 2 + 1]);
                float v  = 0.5f * (row[ x      * 2] + row[ x      * 2 + 1]);
                float lo = vl < vr ? vl : vr, hi = vl < vr ? vr : vl;
                if (hi - lo > smooth_thr) continue;
                if (v - 0.5f * (vl + vr) >= -dot_thr) continue;
                for (int c = 0; c < 2; c++)
                    row[x * 2 + c] = 0.5f * (row[(x - 1) * 2 + c] +
                                             row[(x + 1) * 2 + c]);
                nfix++;
            }
        }
        fprintf(stderr, "defect deltas: dot-only %d cols, %d pixels fixed\n",
                ndotcol, nfix);
        free(col); free(buf); free(base); free(frac); free(mask);
        return;
    }
    /* 掩膜膨胀 1 列 */
    {
        unsigned char *m2 = malloc(w);
        if (!m2) die("out of memory");
        for (int x = 0; x < w; x++)
            m2[x] = mask[x] || (x > 0 && mask[x - 1]) || (x + 1 < w && mask[x + 1]);
        memcpy(mask, m2, w);
        free(m2);
    }
    /* 逐行插值（delta 平滑 → 线性为主; 锐色度边缘联合选边保形） */
    for (int y = 0; y < h; y++) {
        float *row = d + (size_t)y * w * 2;
        int x = 0;
        while (x < w) {
            if (!mask[x]) { x++; continue; }
            int s = x;
            while (x < w && mask[x]) x++;
            int e = x - 1;
            int xl = s - 1, xr = e + 1;
            float step = 0, gl = 0, gr = 0;
            for (int c = 0; c < 2; c++) {
                float vl, vr;
                if (xl < 0)  vl = xr < w ? row[xr * 2 + c] : 0;
                else         vl = row[xl * 2 + c];
                if (xr >= w) vr = xl >= 0 ? row[xl * 2 + c] : vl;
                else         vr = row[xr * 2 + c];
                step += fabsf(vr - vl);
                float vl2 = (xl - 1 >= 0) ? row[(xl - 1) * 2 + c] : vl;
                float vr2 = (xr + 1 < w) ? row[(xr + 1) * 2 + c] : vr;
                gl += fabsf(vl - vl2);
                gr += fabsf(vr - vr2);
            }
            int copy_side = 0;
            if (step > 60.0) {
                if (gl < gr * 0.5) copy_side = 1;
                else if (gr < gl * 0.5) copy_side = 2;
            }
            for (int i = s; i <= e; i++) {
                for (int c = 0; c < 2; c++) {
                    float v;
                    if (copy_side) {
                        int xs = copy_side == 1 ? xl : xr;
                        if (xs < 0) xs = xr >= 0 ? xr : 0;
                        if (xs >= w) xs = xl >= 0 ? xl : w - 1;
                        v = row[xs * 2 + c];
                    } else {
                        float vl, vr;
                        if (xl < 0)  vl = xr < w ? row[xr * 2 + c] : 0;
                        else         vl = row[xl * 2 + c];
                        if (xr >= w) vr = xl >= 0 ? row[xl * 2 + c] : vl;
                        else         vr = row[xr * 2 + c];
                        float t = (e == s) ? 0.5f : (float)(i - s) / (e - s);
                        v = vl + (vr - vl) * t;
                    }
                    row[i * 2 + c] = v;
                }
            }
        }
    }
    if (getenv("X3FDEC_PLANEDBG")) {
        int nd = 0, ndot = 0;
        for (int x = 0; x < w; x++) { nd += mask[x]; ndot += frac[x] > frac_thr * nsamp; }
        fprintf(stderr, "defect deltas: %d cols masked (%d dotted) w=%d\n", nd, ndot, w);
    }
    free(col); free(buf); free(base); free(frac); free(mask);
}

/* 强彩边滞后（bloom lag）NLM 降噪（2026-10-21）。
 *
 * 现象：Quattro 半层 R/G 在强亮度边缘有过扩散的滞后过渡（平滑斜坡
 * 2~4 半层像素、500~1300 raw），顶层 B 边缘却很锐。融合 out = top +
 * up(delta) 时错位带里的 delta 误差 → 花尖 1~2px 品红/蓝边（own 481
 * 个品红像素，x3fuse 6 个）。
 *
 * 对策 = 复刻 x3fuse quattro 展开的 pre-upsample NLM（denoise.rs
 * fast_nlm_denoise，F23/Yis4T: h=300，template 3×3，search 11×11，
 * L1 距离，weight = exp(−dist²/(h²·3))，BORDER_REFLECT_101）：
 *   Y = 4·t2, U = 2·dB, V = dB − 2·dG   （与 bmt_to_yuv_yis4t 同构）
 * NLM 后逆变换回 dB/dG。中间试验过「t2 边缘对齐台阶重构 / 肩部中值」
 * 两代启发式，都会在对角/曲边上制造新瑕疵（品红 936/2847），已废弃——
 * patch 相似度权重是唯一既修斜坡又不伤真实结构的方法。
 * X3FDEC_NONLM 关闭；X3FDEC_NLM_H 调强度（默认 300 = x3fuse F23）。 */
typedef struct {
    float *dg;        /* hw*hh*2, [dB,dG] 交错 */
    const float *t2;  /* hw*hh 顶层均值 */
    int w, h, y0, y1;
    float h2;         /* h*h*3 */
} NlmJob;

static void *nlm_job(void *arg)
{
    NlmJob *j = (NlmJob *)arg;
    const int w = j->w, h = j->h;
    const float h2 = j->h2;
    /* 带反射边界的扩展缓冲（半径 5 + patch 半径 1 = 6） */
    const int B = 6;
    const int ew = w + 2 * B, eh = (j->y1 - j->y0) + 2 * B;
    float *ext = malloc((size_t)ew * eh * 3 * sizeof(float));
    if (!ext) return NULL;
#define REF101(v, n) ((v) < 0 ? -(v) : ((v) >= (n) ? 2 * (n) - 2 - (v) : (v)))
    for (int y = j->y0 - B; y < j->y1 + B; y++) {
        int sy = REF101(y, h);
        float *dst = ext + (size_t)(y - (j->y0 - B)) * ew * 3;
        for (int x = -B; x < w + B; x++) {
            int sx = REF101(x, w);
            size_t si = (size_t)sy * w + sx;
            dst[(x + B) * 3]     = 4.0f * j->t2[si];
            dst[(x + B) * 3 + 1] = 2.0f * j->dg[si * 2];
            dst[(x + B) * 3 + 2] = j->dg[si * 2] - 2.0f * j->dg[si * 2 + 1];
        }
    }
    for (int y = j->y0; y < j->y1; y++) {
        float *crow = ext + (size_t)(y - (j->y0 - B)) * ew * 3 + B * 3;
        for (int x = 0; x < w; x++) {
            const float *c = crow + (size_t)x * 3;
            float est[3] = {0, 0, 0}, wsum = 0;
            for (int sy = -5; sy <= 5; sy++) {
                const float *nrow = crow + (size_t)sy * ew * 3;
                for (int sx = -5; sx <= 5; sx++) {
                    if (!sy && !sx) { wsum += 1.0f; est[0]+=c[0]; est[1]+=c[1]; est[2]+=c[2]; continue; }
                    const float *n = nrow + (size_t)sx * 3;
                    /* 3x3 patch L1 距离（9 像素 × 3 通道） */
                    float dist = 0;
                    for (int py = -1; py <= 1; py++) {
                        const float *pa = crow + (size_t)(py * ew + x) * 3;
                        const float *pb = nrow + (size_t)(sx + py * ew) * 3;
                        for (int px = -1; px <= 1; px++) {
                            float d0 = pa[px*3]   - pb[px*3];
                            float d1 = pa[px*3+1] - pb[px*3+1];
                            float d2 = pa[px*3+2] - pb[px*3+2];
                            dist += fabsf(d0) + fabsf(d1) + fabsf(d2);
                        }
                    }
                    float wgt = expf(-dist * dist / h2);
                    est[0] += wgt * n[0]; est[1] += wgt * n[1]; est[2] += wgt * n[2];
                    wsum += wgt;
                }
            }
            float U = est[1] / wsum, V = est[2] / wsum;
            /* 逆变换: dB = U/2, dG = (U/2 − V)/2 */
            j->dg[((size_t)y * w + x) * 2]     = U * 0.5f;
            j->dg[((size_t)y * w + x) * 2 + 1] = (U * 0.5f - V) * 0.5f;
        }
    }
#undef REF101
    free(ext);
    return NULL;
}

static void repair_edge_lag_deltas(float *dg, const float *t2, int w, int h)
{
    double hh = 300.0;
    const char *e = getenv("X3FDEC_NLM_H");
    if (e) hh = atof(e);
    if (hh <= 0) return;
    enum { NT = 4 };
    NlmJob jobs[NT];
    pthread_t th[NT];
    for (int t = 0; t < NT; t++) {
        jobs[t].dg = dg; jobs[t].t2 = t2; jobs[t].w = w; jobs[t].h = h;
        jobs[t].y0 = t * h / NT;
        jobs[t].y1 = (t + 1) * h / NT;
        jobs[t].h2 = (float)(hh * hh * 3.0);
        if (t && pthread_create(&th[t], NULL, nlm_job, &jobs[t]) != 0)
            jobs[t].y1 = jobs[t].y0;
    }
    nlm_job(&jobs[0]);
    for (int t = 1; t < NT; t++)
        if (jobs[t].y1 > jobs[t].y0) pthread_join(th[t], NULL);
}
/* B/G 通道并行作业 */
typedef struct { float *img; int w, h, band; double sig; const char *tag; } ChJob;
static void *ch_fpn_job(void *arg)
{
    ChJob *j = (ChJob *)arg;
    column_fpn_banded(j->img, j->w, j->h, j->band, j->tag);
    bilateral5(j->img, j->w, j->h, j->sig);
    return NULL;
}
/* 只跑列 FPN 校正的作业（XFCHAIN=1 默认链用）：NLM 整链自己管降噪，
 * 不需要回退分支的 bilateral5，只挂 column_fpn_banded。 */
typedef struct { float *img; int w, h, band; const char *tag; } FpnOnlyJob;
static void *fpn_only_job(void *arg)
{
    FpnOnlyJob *j = (FpnOnlyJob *)arg;
    column_fpn_banded(j->img, j->w, j->h, j->band, j->tag);
    return NULL;
}
typedef struct { const float *src; int sw, sh; float *dst; } RsJob;
static void *rs_job(void *arg)
{
    RsJob *j = (RsJob *)arg;
    resize_bicubic_2x(j->src, j->sw, j->sh, j->dst);
    return NULL;
}
typedef struct {
    const float *src; int hw, hh;
    const float *guide; const float *t2; float *dst;
    double k, fd;
    int y0, y1;   /* 本分块负责的输出行区间 [y0, y1) */
} GuJob;
static void *gu_job(void *arg)
{
    GuJob *j = (GuJob *)arg;
    guided_up_2x(j->src, j->hw, j->hh, j->guide, j->t2, j->dst, j->k, j->fd,
                 j->y0, j->y1);
    return NULL;
}
/* ==================================================================== */
/* x3fuse 等价 NLM（FastNlMeansDenoisingInvoker 忠实移植，u16 定点）      */
/* 参考: fuj/outputs/X3FCore/vendor/x3fuse/crates/x3f-sys/src/denoise.rs  */
/* 算法: 3 通道联合 L1 patch 距离 + 定点量化权重 LUT                      */
/*   w[c] = exp(−dist²/(h[c]²·3))，dist = 量化 patch SAD；h[c]=0 的通道   */
/*   仅在 dist=0 时 w=1（亮度通道输出会被上层丢弃，只起引导作用）；        */
/*   OpenCV 增量列和 invoker + 行分块多线程；BORDER_REFLECT_101。         */
/* ==================================================================== */

static int reflect101(int i, int n)
{
    if (n == 1) return 0;
    int period = 2 * (n - 1);
    int r = i % period;
    if (r < 0) r += period;
    if (r >= n) r = period - r;
    return r;
}

static inline int nlm3_dist(const uint16_t *ext, size_t ec,
                            int y1, int x1, int y2, int x2)
{
    size_t i1 = ((size_t)y1 * ec + x1) * 3, i2 = ((size_t)y2 * ec + x2) * 3;
    int a = (int)ext[i1]     - (int)ext[i2];
    int b = (int)ext[i1 + 1] - (int)ext[i2 + 1];
    int c = (int)ext[i1 + 2] - (int)ext[i2 + 2];
    return (a < 0 ? -a : a) + (b < 0 ? -b : b) + (c < 0 ? -c : c);
}

typedef struct {
    const uint16_t *ext;
    size_t ec;
    int cols, border, tr, sr, tw, sw, bin_shift;
    const int32_t *lut;
    int lut_n;
    int y0, y1;        /* 负责的输出行 [y0,y1]（含端点） */
    uint16_t *dst;     /* 全尺寸输出 rows*cols*3（各行不相交，可并行写） */
} Nlm3Job;

static void *nlm3_job(void *arg)
{
    Nlm3Job *j = (Nlm3Job *)arg;
    const uint16_t *ext = j->ext;
    size_t ec = j->ec;
    int cols = j->cols, border = j->border;
    int tr = j->tr, sr = j->sr, tw = j->tw, sw = j->sw;
    int bin_shift = j->bin_shift;
    const int32_t *lut = j->lut;
    int lut_n = j->lut_n;

    int32_t *dist_sums = calloc((size_t)sw * sw, sizeof(int32_t));
    int32_t *col_dist  = calloc((size_t)tw * sw * sw, sizeof(int32_t));
    int32_t *up_col    = malloc((size_t)cols * sw * sw * sizeof(int32_t));
    if (!dist_sums || !col_dist || !up_col) die("out of memory");

#define NDS(y, x)     ((y) * sw + (x))
#define NCD(t, y, x)  (((size_t)(t) * sw + (y)) * sw + (x))
#define NUC(jj, y, x) (((size_t)(jj) * sw + (y)) * sw + (x))

    int first_col = -1;
    for (int i = j->y0; i <= j->y1; i++) {
        for (int jj = 0; jj < cols; jj++) {
            if (jj == 0) {
                /* calcDistSumsForFirstElementInRow: 行首全量重算 */
                for (int y = 0; y < sw; y++)
                    for (int x = 0; x < sw; x++) {
                        dist_sums[NDS(y, x)] = 0;
                        for (int t = 0; t < tw; t++) col_dist[NCD(t, y, x)] = 0;
                        int sy0 = i + y - sr, sx0 = x - sr;
                        for (int ty = -tr; ty <= tr; ty++)
                            for (int tx = -tr; tx <= tr; tx++) {
                                int d = nlm3_dist(ext, ec,
                                                  border + i + ty, border + tx,
                                                  border + sy0 + ty,
                                                  border + sx0 + tx);
                                dist_sums[NDS(y, x)] += d;
                                col_dist[NCD(tx + tr, y, x)] += d;
                            }
                        up_col[NUC(0, y, x)] = col_dist[NCD(tw - 1, y, x)];
                    }
                first_col = 0;
            } else if (i == j->y0) {
                /* calcDistSumsForElementInFirstRow: 带首行重算滑入列 */
                int ay = border + i, ax = border + jj + tr;
                int sby = border + i - sr, sbx = border + jj - sr + tr;
                for (int y = 0; y < sw; y++)
                    for (int x = 0; x < sw; x++) {
                        dist_sums[NDS(y, x)] -= col_dist[NCD(first_col, y, x)];
                        int by = sby + y, bx = sbx + x, s = 0;
                        for (int ty = -tr; ty <= tr; ty++)
                            s += nlm3_dist(ext, ec, ay + ty, ax, by + ty, bx);
                        col_dist[NCD(first_col, y, x)] = s;
                        dist_sums[NDS(y, x)] += s;
                        up_col[NUC(jj, y, x)] = s;
                    }
                first_col = (first_col + 1) % tw;
            } else {
                /* 增量：滑出列 −，滑入列 +（垂直方向复用上一行列和） */
                int ay = border + i, ax = border + jj + tr;
                int sby = border + i - sr, sbx = border + jj - sr + tr;
                for (int y = 0; y < sw; y++)
                    for (int x = 0; x < sw; x++) {
                        int cidx = (int)NCD(first_col, y, x);
                        dist_sums[NDS(y, x)] -= col_dist[cidx];
                        int bx = sbx + x;
                        int upd = nlm3_dist(ext, ec,
                                            ay + tr, ax, sby + tr + y, bx)
                                - nlm3_dist(ext, ec,
                                            ay - tr - 1, ax, sby - tr - 1 + y, bx);
                        int nc = up_col[NUC(jj, y, x)] + upd;
                        col_dist[cidx] = nc;
                        dist_sums[NDS(y, x)] += nc;
                        up_col[NUC(jj, y, x)] = nc;
                    }
                first_col = (first_col + 1) % tw;
            }

            /* 搜索窗加权平均 */
            int64_t est[3] = { 0, 0, 0 }, wsum[3] = { 0, 0, 0 };
            const uint16_t *p0 = ext +
                ((size_t)(border + i - sr) * ec + (border + jj - sr)) * 3;
            for (int y = 0; y < sw; y++) {
                const uint16_t *pr = p0 + (size_t)y * ec * 3;
                for (int x = 0; x < sw; x++) {
                    int almost = dist_sums[NDS(y, x)] >> bin_shift;
                    size_t ai = almost < 0 ? 0 : (size_t)almost;
                    if (ai >= (size_t)lut_n) ai = (size_t)lut_n - 1;
                    const int32_t *w = lut + ai * 3;
                    const uint16_t *p = pr + (size_t)x * 3;
                    est[0] += (int64_t)w[0] * p[0];
                    est[1] += (int64_t)w[1] * p[1];
                    est[2] += (int64_t)w[2] * p[2];
                    wsum[0] += w[0]; wsum[1] += w[1]; wsum[2] += w[2];
                }
            }
            uint16_t *o = j->dst + ((size_t)i * cols + jj) * 3;
            for (int c = 0; c < 3; c++) {
                uint16_t v = 0;
                if (wsum[c] > 0) {
                    int64_t n = (est[c] + wsum[c] / 2) / wsum[c];
                    v = n < 0 ? 0 : n > 65535 ? 65535 : (uint16_t)n;
                }
                o[c] = v;
            }
        }
    }
#undef NDS
#undef NCD
#undef NUC
    free(dist_sums); free(col_dist); free(up_col);
    return NULL;
}

/* src3/dst3: 交错 3 通道 u16 图。hp[3] 为各通道 h（0 = 该通道不降噪仅引导）。
 * twin/swin: template/search 窗全宽（奇数，如 3/11）。 */
static void nlm3_den(const uint16_t *src3, int w, int hgt,
                     const double hp[3], int twin, int swin, uint16_t *dst3)
{
    if (w <= 0 || hgt <= 0) return;
    uint16_t *dbg_ref = NULL;   /* X3FDEC_NLMDBG: GPU 结果备份 */
    int tr = twin / 2, sr = swin / 2;
    int tw = 2 * tr + 1, sw = 2 * sr + 1;
    int border = sr + tr;
    int ext_rows = hgt + 2 * border, ext_cols = w + 2 * border;
    size_t ec = (size_t)ext_cols;

    /* 定点权重 LUT（almost_dist2weight；fixed_point_mult = INT32_MAX） */
    int tw_sq = tw * tw;
    int bin_shift = 0;
    while ((1 << bin_shift) < tw_sq) bin_shift++;
    double mult = (double)(1 << bin_shift) / tw_sq;
    int64_t fpm = INT64_MAX / ((int64_t)sw * sw * 65535);
    if (fpm > INT32_MAX) fpm = INT32_MAX;
    double thr = 0.001 * (double)fpm;
    int lut_n = (int)((double)(65535 * 3) / mult + 1.0);
    int32_t *lut = malloc((size_t)lut_n * 3 * sizeof(int32_t));
    if (!lut) die("out of memory");
    for (int ad = 0; ad < lut_n; ad++) {
        double dist = (double)ad * mult;
        for (int c = 0; c < 3; c++) {
            double wgt;
            if (hp[c] > 0.0)
                wgt = exp(-(dist * dist) / (hp[c] * hp[c] * 3.0));
            else
                wgt = (dist == 0.0) ? 1.0 : 0.0;   /* h=0: 0/0=NaN→1（Rust 同） */
            int64_t weight = (int64_t)rint((double)fpm * wgt);
            if ((double)weight < thr) weight = 0;
            lut[(size_t)ad * 3 + c] = (int32_t)weight;
        }
    }

    /* GPU 优先（X3FDEC_GPUNLM=0 单独关闭；失败回退 CPU 行分块）。
     * GPU kernel 与 CPU 整型运算逐位一致（同 LUT 同定点语义）。
     * X3FDEC_NLMDBG=1: GPU 结果备份后仍跑 CPU 路径，末尾逐位比对。
     * 2026-11-14：dn->backend=CPU 时也强制走 CPU，别让后端选择名存实亡。 */
    {
        const char *ge = getenv("X3FDEC_GPUNLM");
        int gpu_ok = 0;
        if (own_gpu_available() && !own_dn_force_cpu() && (!ge || ge[0] != '0'))
            gpu_ok = own_gpu_nlm3(src3, dst3, w, hgt, lut, lut_n, twin, swin,
                                  bin_shift) == 0;
        if (gpu_ok && !getenv("X3FDEC_NLMDBG")) {
            free(lut);
            return;
        }
        if (gpu_ok) {
            dbg_ref = malloc((size_t)w * hgt * 3 * sizeof(uint16_t));
            if (dbg_ref) {
                memcpy(dbg_ref, dst3, (size_t)w * hgt * 3 * sizeof(uint16_t));
                memset(dst3, 0, (size_t)w * hgt * 3 * sizeof(uint16_t));
            }
            /* 不 return：继续走 CPU 路径，函数末尾比对 */
        }
    }

    /* BORDER_REFLECT_101 扩展源 */
    int *rmap = malloc((size_t)ext_rows * sizeof(int));
    int *cmap = malloc((size_t)ext_cols * sizeof(int));
    uint16_t *ext = malloc((size_t)ext_rows * ext_cols * 3 * sizeof(uint16_t));
    if (!rmap || !cmap || !ext) die("out of memory");
    for (int ey = 0; ey < ext_rows; ey++) rmap[ey] = reflect101(ey - border, hgt);
    for (int ex = 0; ex < ext_cols; ex++) cmap[ex] = reflect101(ex - border, w);
    for (int ey = 0; ey < ext_rows; ey++) {
        const uint16_t *srow = src3 + (size_t)rmap[ey] * w * 3;
        uint16_t *drow = ext + (size_t)ey * ec * 3;
        for (int ex = 0; ex < ext_cols; ex++) {
            const uint16_t *s = srow + (size_t)cmap[ex] * 3;
            uint16_t *d = drow + (size_t)ex * 3;
            d[0] = s[0]; d[1] = s[1]; d[2] = s[2];
        }
    }
    free(rmap); free(cmap);

    /* 行分块并行（每带独立增量状态） */
    enum { NT = 6 };
    Nlm3Job jobs[NT];
    pthread_t th[NT];
    int band = (hgt + NT - 1) / NT;
    for (int t = 0; t < NT; t++) {
        jobs[t].ext = ext; jobs[t].ec = ec; jobs[t].cols = w;
        jobs[t].border = border; jobs[t].tr = tr; jobs[t].sr = sr;
        jobs[t].tw = tw; jobs[t].sw = sw; jobs[t].bin_shift = bin_shift;
        jobs[t].lut = lut; jobs[t].lut_n = lut_n;
        jobs[t].dst = dst3;
        jobs[t].y0 = t * band;
        jobs[t].y1 = jobs[t].y0 + band - 1;
        if (jobs[t].y1 >= hgt) jobs[t].y1 = hgt - 1;
        if (t && jobs[t].y0 < hgt &&
            pthread_create(&th[t], NULL, nlm3_job, &jobs[t]) != 0)
            jobs[t].y1 = jobs[t].y0 - 1;
    }
    if (jobs[0].y1 >= jobs[0].y0) nlm3_job(&jobs[0]);
    for (int t = 1; t < NT; t++)
        if (jobs[t].y1 >= jobs[t].y0) pthread_join(th[t], NULL);

    free(ext);
    if (dbg_ref) {
        long bad = -1, nbad = 0, maxdiff = 0;
        for (size_t i = 0; i < (size_t)w * hgt * 3; i++)
            if (dbg_ref[i] != dst3[i]) {
                nbad++;
                if (bad < 0) bad = (long)i;
                long d = (long)dbg_ref[i] - (long)dst3[i];
                if (d < 0) d = -d;
                if (d > maxdiff) maxdiff = d;
            }
        fprintf(stderr, "[nlmdbg] %dx%d sw=%d nbad=%ld first=%ld "
                "gpu@first=%d cpu@first=%d maxdiff=%ld\n",
                w, hgt, swin, nbad, bad,
                bad >= 0 ? dst3[bad] : -1,
                bad >= 0 ? dbg_ref[bad] : -1, maxdiff);
        if (getenv("X3FDEC_NLMHEX")) {
            fprintf(stderr, "[nlmhex] row0 U/V gpu vs cpu (first 24 pixels):\n");
            for (int px = 0; px < 24 && px < w; px++) {
                size_t o = ((size_t)px) * 3;
                fprintf(stderr, "  px%02d U %5u/%5u  V %5u/%5u\n", px,
                        dst3[o + 1], dbg_ref[o + 1], dst3[o + 2], dbg_ref[o + 2]);
            }
            fprintf(stderr, "[nlmhex] row0 INPUT U/V (first 24 pixels):\n");
            for (int px = 0; px < 24 && px < w; px++) {
                size_t o = ((size_t)px) * 3;
                fprintf(stderr, "  px%02d in U %5u  V %5u\n", px, src3[o + 1], src3[o + 2]);
            }
            {
                long sat_u = 0, sat_v = 0, sat_y = 0;
                for (size_t p = 0; p < (size_t)w * hgt; p++) {
                    if (src3[p * 3] == 65535) sat_y++;
                    if (src3[p * 3 + 1] == 65535) sat_u++;
                    if (src3[p * 3 + 2] == 65535) sat_v++;
                }
                fprintf(stderr, "[nlmhex] input sat count: Y=%ld U=%ld V=%ld of %d\n",
                        sat_y, sat_u, sat_v, w * hgt);
            }
            /* 朴素全量重算参照（每偏移独立算 SAD，不经滑窗状态）——
             * 用来裁决：GPU 与「数学定义」一致而与增量 CPU 不一致，
             * 还是 GPU 自己错了。只算 row0 前 24 像素。 */
            {
                int b2 = border;
                long ec2 = (long)w + 2 * b2;
                uint16_t *ex2 = malloc((size_t)(hgt + 2 * b2) * ec2 * 3 * sizeof(uint16_t));
                if (ex2) {
                    for (int ey = 0; ey < hgt + 2 * b2; ey++) {
                        int ry = reflect101(ey - b2, hgt);
                        for (int exx = 0; exx < w + 2 * b2; exx++) {
                            const uint16_t *s = src3 + ((size_t)ry * w + reflect101(exx - b2, w)) * 3;
                            uint16_t *d = ex2 + ((size_t)ey * ec2 + exx) * 3;
                            d[0] = s[0]; d[1] = s[1]; d[2] = s[2];
                        }
                    }
                    fprintf(stderr, "[nlmnaive] row0-3 gpu vs cpu (px0-15, * = 65535):\n");
                    for (int ry = 0; ry < 4 && ry < hgt; ry++) {
                        fprintf(stderr, "  row%d: ", ry);
                        for (int jj = 0; jj < 16 && jj < w; jj++) {
                            size_t o = ((size_t)ry * w + jj) * 3;
                            unsigned gu = dbg_ref[o + 1];
                            fprintf(stderr, "%c", gu == 65535 ? '*' : '.');
                        }
                        fprintf(stderr, "  |");
                        for (int jj = 0; jj < 16 && jj < w; jj++) {
                            size_t o = ((size_t)ry * w + jj) * 3;
                            fprintf(stderr, " %u/%u", dbg_ref[o + 1], dst3[o + 1]);
                        }
                        fprintf(stderr, "\n");
                    }
                    for (int jj = 0; jj < 24 && jj < w; jj++) {
                        int64_t e[3] = { 0, 0, 0 }, ws[3] = { 0, 0, 0 };
                        for (int y = 0; y < sw; y++)
                            for (int x = 0; x < sw; x++) {
                                int dsum = 0;
                                for (int ty = -tr; ty <= tr; ty++)
                                    for (int tx = -tr; tx <= tr; tx++) {
                                        size_t i1 = ((size_t)(b2 + ty) * ec2 + (b2 + jj + tx)) * 3;
                                        size_t i2 = ((size_t)(b2 + y - sr + ty) * ec2 + (b2 + jj + x - sr + tx)) * 3;
                                        int a = (int)ex2[i1] - (int)ex2[i2];
                                        int b = (int)ex2[i1 + 1] - (int)ex2[i2 + 1];
                                        int c = (int)ex2[i1 + 2] - (int)ex2[i2 + 2];
                                        dsum += (a < 0 ? -a : a) + (b < 0 ? -b : b) + (c < 0 ? -c : c);
                                    }
                                int almost = dsum >> bin_shift;
                                size_t ai = almost < 0 ? 0 : (size_t)almost;
                                if (ai >= (size_t)lut_n) ai = (size_t)lut_n - 1;
                                const int32_t *ww = lut + ai * 3;
                                size_t po = ((size_t)(b2 + y - sr) * ec2 + (b2 + jj + x - sr)) * 3;
                                e[1] += (int64_t)ww[1] * ex2[po + 1];
                                e[2] += (int64_t)ww[2] * ex2[po + 2];
                                ws[1] += ww[1]; ws[2] += ww[2];
                            }
                        unsigned nu = 0, nv = 0;
                        if (ws[1] > 0) { int64_t n = (e[1] + ws[1] / 2) / ws[1];
                            nu = n < 0 ? 0 : n > 65535 ? 65535 : (unsigned)n; }
                        if (ws[2] > 0) { int64_t n = (e[2] + ws[2] / 2) / ws[2];
                            nv = n < 0 ? 0 : n > 65535 ? 65535 : (unsigned)n; }
                        fprintf(stderr, "  px%02d gpu U %5u V %5u | naive U %5u V %5u\n",
                                jj, dbg_ref[(size_t)jj * 3 + 1], dbg_ref[(size_t)jj * 3 + 2], nu, nv);
                    }
                    free(ex2);
                }
            }
        }
        free(dbg_ref);
    }
    free(lut);
}

static inline uint16_t sat16f(float v)
{
    long n = lrintf(v);
    return n < 0 ? 0 : n > 65535 ? 65535 : (uint16_t)n;
}

/* 单通道 3×3 中值，BORDER_REPLICATE（OpenCV medianBlur ksize=3） */
static void median3_ch(uint16_t *img3, int w, int h, int ch)
{
    uint16_t *plane = malloc((size_t)w * h * sizeof(uint16_t));
    if (!plane) die("out of memory");
    for (size_t i = 0; i < (size_t)w * h; i++) plane[i] = img3[i * 3 + ch];
    for (int y = 0; y < h; y++) {
        int ym = y > 0 ? y - 1 : 0, yp = y < h - 1 ? y + 1 : h - 1;
        for (int x = 0; x < w; x++) {
            int xm = x > 0 ? x - 1 : 0, xp = x < w - 1 ? x + 1 : w - 1;
            uint16_t v[9] = {
                plane[ym * w + xm], plane[ym * w + x], plane[ym * w + xp],
                plane[y * w + xm],  plane[y * w + x],  plane[y * w + xp],
                plane[yp * w + xm], plane[yp * w + x], plane[yp * w + xp] };
            for (int a = 0; a < 8; a++)
                for (int b = a + 1; b < 9; b++)
                    if (v[b] < v[a]) { uint16_t t = v[a]; v[a] = v[b]; v[b] = t; }
            img3[((size_t)y * w + x) * 3 + ch] = v[4];
        }
    }
    free(plane);
}

/* INTER_AREA 3 通道 u16 缩放（denoise.rs resize_area: 水平 f64 → 垂直 u16） */
static void resize_area_3ch(const uint16_t *src, int sw, int sh,
                            uint16_t *dst, int dw, int dh)
{
    double sx = (double)sw / dw, sy = (double)sh / dh;
    double *tmp = malloc((size_t)sh * dw * 3 * sizeof(double));
    if (!tmp) die("out of memory");
    for (int y = 0; y < sh; y++) {
        const uint16_t *srow = src + (size_t)y * sw * 3;
        double *trow = tmp + (size_t)y * dw * 3;
        for (int dx = 0; dx < dw; dx++) {
            double f1 = dx * sx, f2 = f1 + sx, acc[3] = { 0, 0, 0 };
            int s0 = (int)floor(f1), s1 = (int)ceil(f2);
            for (int s = s0; s < s1; s++) {
                double lo = s > f1 ? s : f1;
                double hi = (s + 1) < f2 ? (s + 1) : f2;
                double wgt = (hi - lo) / sx;
                if (wgt <= 0.0) continue;
                int si = s < 0 ? 0 : s >= sw ? sw - 1 : s;
                const uint16_t *p = srow + (size_t)si * 3;
                acc[0] += wgt * p[0]; acc[1] += wgt * p[1]; acc[2] += wgt * p[2];
            }
            trow[dx * 3] = acc[0]; trow[dx * 3 + 1] = acc[1];
            trow[dx * 3 + 2] = acc[2];
        }
    }
    for (int dy = 0; dy < dh; dy++) {
        double f1 = dy * sy, f2 = f1 + sy;
        int s0 = (int)floor(f1), s1 = (int)ceil(f2);
        uint16_t *drow = dst + (size_t)dy * dw * 3;
        for (int dx = 0; dx < dw; dx++) {
            double acc[3] = { 0, 0, 0 };
            for (int s = s0; s < s1; s++) {
                double lo = s > f1 ? s : f1;
                double hi = (s + 1) < f2 ? (s + 1) : f2;
                double wgt = (hi - lo) / sy;
                if (wgt <= 0.0) continue;
                int si = s < 0 ? 0 : s >= sh ? sh - 1 : s;
                const double *p = tmp + ((size_t)si * dw + dx) * 3;
                acc[0] += wgt * p[0]; acc[1] += wgt * p[1]; acc[2] += wgt * p[2];
            }
            for (int c = 0; c < 3; c++) {
                long n = lrint(acc[c]);   /* round_ties_even */
                drow[dx * 3 + c] = n < 0 ? 0 : n > 65535 ? 65535 : (uint16_t)n;
            }
        }
    }
    free(tmp);
}

/* i16 3 通道 INTER_CUBIC 放大（denoise.rs resize_cubic_i16: a=−0.75,
 * reflect101, round_ties_even, clamp i16）。用于低频残差 4x 放大。 */
static void cubic_up_i16_3ch(const int16_t *src, int sw, int sh,
                             int16_t *dst, int dw, int dh)
{
    double sx = (double)sw / dw, sy = (double)sh / dh;
    double *tmp = malloc((size_t)sh * dw * 3 * sizeof(double));
    if (!tmp) die("out of memory");
    for (int y = 0; y < sh; y++) {
        const int16_t *srow = src + (size_t)y * sw * 3;
        double *trow = tmp + (size_t)y * dw * 3;
        for (int dx = 0; dx < dw; dx++) {
            double fx = (dx + 0.5) * sx - 0.5;
            int isx = (int)floor(fx);
            double t = fx - isx;
            float w[4];
            w[0] = cubicf((float)(1.0 + t), -0.75f);
            w[1] = cubicf((float)t, -0.75f);
            w[2] = cubicf((float)(1.0 - t), -0.75f);
            w[3] = cubicf((float)(2.0 - t), -0.75f);
            double acc[3] = { 0, 0, 0 };
            for (int k = 0; k < 4; k++) {
                int xx = reflect101(isx - 1 + k, sw);
                const int16_t *p = srow + (size_t)xx * 3;
                acc[0] += w[k] * p[0]; acc[1] += w[k] * p[1]; acc[2] += w[k] * p[2];
            }
            trow[dx * 3] = acc[0]; trow[dx * 3 + 1] = acc[1];
            trow[dx * 3 + 2] = acc[2];
        }
    }
    for (int dy = 0; dy < dh; dy++) {
        double fy = (dy + 0.5) * sy - 0.5;
        int isy = (int)floor(fy);
        double t = fy - isy;
        float w[4];
        w[0] = cubicf((float)(1.0 + t), -0.75f);
        w[1] = cubicf((float)t, -0.75f);
        w[2] = cubicf((float)(1.0 - t), -0.75f);
        w[3] = cubicf((float)(2.0 - t), -0.75f);
        int taps[4];
        for (int k = 0; k < 4; k++) taps[k] = reflect101(isy - 1 + k, sh);
        int16_t *drow = dst + (size_t)dy * dw * 3;
        for (int dx = 0; dx < dw; dx++) {
            for (int c = 0; c < 3; c++) {
                double v = w[0] * tmp[(size_t)taps[0] * dw * 3 + dx * 3 + c]
                         + w[1] * tmp[(size_t)taps[1] * dw * 3 + dx * 3 + c]
                         + w[2] * tmp[(size_t)taps[2] * dw * 3 + dx * 3 + c]
                         + w[3] * tmp[(size_t)taps[3] * dw * 3 + dx * 3 + c];
                long n = lrint(v);   /* round_ties_even */
                drow[dx * 3 + c] =
                    n < -32768 ? -32768 : n > 32767 ? 32767 : (int16_t)n;
            }
        }
    }
    free(tmp);
}

/* x3fuse denoise_nlm 完整链（stage 0，半层 active 子区，就地修改）:
 *   1) NLM h={0,σ,σ}（亮度只引导不降噪）
 *   2) V 通道 3×3 中值
 *   3) 低频: ÷4 area → NLM{0,σ/8,σ/4} search21 → 有符号残差 → 4x cubic → 减回
 * img3: (Y, U+32768, V+32768) 交错 u16 */
static void denoise_nlm_yuv16(uint16_t *img3, int w, int h,
                              double sigma, double hY, int twin, int swin)
{
    uint16_t *out3 = malloc((size_t)w * h * 3 * sizeof(uint16_t));
    if (!out3) die("out of memory");
    /* Y 通道只在 dn->luminance 开时降（luma_scale 已乘进 hY）；U/V 走色度档。
     * 与 x3fuse channel_strengths / denoise_active 同一套三通道权重口径。 */
    double h0[3] = { hY, sigma, sigma };
    nlm3_den(img3, w, h, h0, twin, swin, out3);
    if (sigma > 0.01) median3_ch(out3, w, h, 2);

    int sh2 = (int)lrint(h * 0.25), sw2 = (int)lrint(w * 0.25);
    if (sh2 >= 1 && sw2 >= 1) {
        uint16_t *sub   = malloc((size_t)sw2 * sh2 * 3 * sizeof(uint16_t));
        uint16_t *subdn = malloc((size_t)sw2 * sh2 * 3 * sizeof(uint16_t));
        int16_t  *res   = malloc((size_t)sw2 * sh2 * 3 * sizeof(int16_t));
        int16_t  *resup = malloc((size_t)w * h * 3 * sizeof(int16_t));
        if (!sub || !subdn || !res || !resup) die("out of memory");
        resize_area_3ch(out3, w, h, sub, sw2, sh2);
        double h1[3] = { hY / 8.0, sigma / 8.0, sigma / 4.0 };
        nlm3_den(sub, sw2, sh2, h1, twin, 21, subdn);
        for (size_t i = 0; i < (size_t)sw2 * sh2 * 3; i++) {
            int d = (int)sub[i] - (int)subdn[i];
            res[i] = d < -32768 ? -32768 : d > 32767 ? 32767 : (int16_t)d;
        }
        cubic_up_i16_3ch(res, sw2, sh2, resup, w, h);
        for (size_t i = 0; i < (size_t)w * h * 3; i++) {
            int v = (int)out3[i] - (int)resup[i];
            out3[i] = v < 0 ? 0 : v > 65535 ? 65535 : (uint16_t)v;
        }
        free(sub); free(subdn); free(res); free(resup);
    }
    memcpy(img3, out3, (size_t)w * h * 3 * sizeof(uint16_t));
    free(out3);
}

/* Quattro 三层合并（浮点，黑电平已扣除）。
 *
 * 数学依据：BMT→YUV (Y=4T,U=2B-2T,V=B-2M+T) 与逆变换精确互逆；x3fuse/SPP
 * 在上采样后用全分辨率顶层替换 Y，等价于：
 *   out_T = top - black2
 *   out_B = out_T + up(half_b - black0 - t2x2 + black2)
 *   out_M = out_T + up(half_m - black1 - t2x2 + black2)
 * 其中 t2x2 = 顶层 2x2 块均值（对应 x3fuse "Downsample top16 4-pixel
 * sums into image[2]"，即 AverageBlue 思路）。
 * 上采样核：Catmull-Rom a=-0.75，OpenCV 像素相位约定。
 * act[4] = ActiveImageArea 的半层坐标 (x0,y0,w,h)；NLM 只在 active 子区
 * 跑（与 x3fuse active view 一致），x0<0 或 w<=0 表示全图。 */
/* 诊断 dump：半层/全分辨率色度 crop（NLM 前/后对照）。
 * 受 X3F_OWN_LAGDUMP 控制：非空即开；若含逗号 "cx,cy,w,h"（全分辨率坐标）
 * 指定 crop 中心与尺寸，否则用默认 2776,828,400,400。写文本浮点网格到 cwd。 */
static int g_lag_cx = 2776, g_lag_cy = 828, g_lag_w = 400, g_lag_h = 400;
static int g_lag_init = 0;
static void own_lag_parse(void)
{
    if (g_lag_init) return;
    g_lag_init = 1;
    const char *e = getenv("X3F_OWN_LAGDUMP");
    if (!e) return;
    int cx, cy, w, h;
    if (sscanf(e, "%d,%d,%d,%d", &cx, &cy, &w, &h) == 4) {
        g_lag_cx = cx; g_lag_cy = cy; g_lag_w = w; g_lag_h = h;
    }
}

/* scale2=1: buf 为半层 (hw×hh)；=2: 全分辨率 (ew×eh)。
 * crop 中心/尺寸始终以全分辨率坐标给出：半层缓冲需 ÷2，全分辨率缓冲 ÷1。 */
static void own_dump_lag(const char *path, const float *buf, int w, int h,
                         int scale2)
{
    int div = (scale2 == 1) ? 2 : 1;   /* 半层缓冲像素 = 全分辨率/2 */
    int bx = g_lag_cx / div, by = g_lag_cy / div;
    int bw = g_lag_w / div, bh = g_lag_h / div;
    int x0 = bx - bw / 2, y0 = by - bh / 2;
    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "[lagdump] cannot open %s\n", path); return; }
    fprintf(f, "# %s scale2=%d w=%d h=%d x0=%d y0=%d bw=%d bh=%d\n",
            path, scale2, w, h, x0, y0, bw, bh);
    for (int j = 0; j < bh; j++) {
        int yy = y0 + j;
        for (int i = 0; i < bw; i++) {
            int xx = x0 + i;
            float v = 0.0f;
            if (xx >= 0 && xx < w && yy >= 0 && yy < h)
                v = buf[(size_t)yy * w + xx];
            fprintf(f, "%.3f%c", v, (i + 1 < bw) ? ' ' : '\n');
        }
    }
    fclose(f);
}

/* 全分辨率原始顶层（u16）dump：用于定位暗线是否在传感器原始 top 自带，
 * 还是被 decode_file_wb 的顶层处理函数引入。crop 坐标同 own_dump_lag。 */
static void own_dump_lag_u16(const char *path, const uint16_t *buf, int w, int h)
{
    int bx = g_lag_cx, by = g_lag_cy;
    int bw = g_lag_w, bh = g_lag_h;
    int x0 = bx - bw / 2, y0 = by - bh / 2;
    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "[lagdump] cannot open %s\n", path); return; }
    fprintf(f, "# %s u16 w=%d h=%d x0=%d y0=%d bw=%d bh=%d\n",
            path, w, h, x0, y0, bw, bh);
    for (int j = 0; j < bh; j++) {
        int yy = y0 + j;
        for (int i = 0; i < bw; i++) {
            int xx = x0 + i;
            float v = 0.0f;
            if (xx >= 0 && xx < w && yy >= 0 && yy < h)
                v = (float)buf[(size_t)yy * w + xx];
            fprintf(f, "%.1f%c", v, (i + 1 < bw) ? ' ' : '\n');
        }
    }
    fclose(f);
}

/* ---- 边缘 delta 冲激修复（半层域；NLM 之后、上采样之前）--------------
 *
 * 病灶：暗屋顶/亮天空等硬边处，天空侧贴边 1px 的 R/G 线性值下陷 ~4%
 * （实测 G 0.418→0.401、R 0.269→0.246，而 B=顶层干净不下陷）→
 * 沿屋顶延伸、只比天空深一点点的暗线。
 *
 * 根因（2026-10-31 dump 实证）：半层 delta（dB=R−T、dG=G−T）在**边缘像素**
 * 上，R/G 半层与顶层 T 不同步跨越边缘（半层先跨、顶层后跨），使 delta 在
 * 该像素出现超出两侧的冲激——沿边连续，实测 dG 比天空侧低 20~90 raw
 * （半层 col 240 处 −196 vs 上方 −110）。
 * 半层/全分辨率 NLM 都**不修正**它：Y 硬引导 h=0 使边缘处 patch 权重全部
 * 压到自身，输出=输入（实测 rowh 146 的 dG −170.1→−170.5，改动 0.4）。
 * 于是冲激原样经双三次上采样放大成全分辨率 1~2px 下陷。
 *
 * 修法：沿梯度方向两侧各取 K 个像素的 delta 中值 → 区间 [lo,hi]（即该边
 * 两侧应有的取值范围）；凡**超出**该区间的才是冲激，钳回最近边界。
 * 判据必须用"超出两侧区间"而非"偏离同侧值"：强边的过渡像素（如 dB 在
 * 天空 −1265 与建筑 −224 之间的 −1034）本来就远离同侧值，用同侧替换会把
 * 单调过渡改成跳变 → 双三次 ringing 加剧（实测贴边反变暗 19 灰阶）。
 * 细线排除：任一侧 K 像素内出现反向强梯度 → 判为细线（电线等），跳过。
 *
 * 默认关（X3F_OWN_EDGEFIX 未设），零回归。
 */
static float own_med_small(float *v, int n)
{
    for (int i = 1; i < n; i++) {           /* 插入排序，n<=8 */
        float t = v[i]; int j = i - 1;
        while (j >= 0 && v[j] > t) { v[j + 1] = v[j]; j--; }
        v[j + 1] = t;
    }
    return v[n / 2];
}

static long fix_edge_delta_overshoot(float *dA, float *dBc, const float *T,
                                     int w, int h,
                                     int x0, int y0, int aw, int ah,
                                     float mag_thr, float slack, float mix,
                                     int chmask)
{
    const int K = 5;
    float sa[8], sb[8], sa2[8], sb2[8];
    long nfix = 0, nedge = 0;

    for (int y = y0; y < y0 + ah; y++) {
        if (y < 1 || y + 1 >= h) continue;
        const float *tc = T + (size_t)y * w;
        const float *tu = T + (size_t)(y - 1) * w;
        const float *td = T + (size_t)(y + 1) * w;
        for (int x = x0; x < x0 + aw; x++) {
            if (x < 1 || x + 1 >= w) continue;
            float gx = tc[x + 1] - tc[x - 1];
            float gy = td[x] - tu[x];
            float mag = fabsf(gx) + fabsf(gy);         /* L1 足够 */
            if (mag < mag_thr) continue;
            nedge++;

            int horiz = fabsf(gx) >= fabsf(gy);
            float g = horiz ? gx : gy;
            int d   = (g > 0.0f) ? 1 : -1;             /* 上坡方向（T 增大）*/

            /* 两侧各取 K 个样本 */
            int n1 = 0, n2 = 0;
            for (int k = 1; k <= K; k++) {
                int xx = horiz ? x + d * k : x;
                int yy = horiz ? y : y + d * k;
                if (xx < 0 || xx >= w || yy < 0 || yy >= h) break;
                size_t i = (size_t)yy * w + xx;
                sa[n1] = dA[i]; sb[n1] = dBc[i]; n1++;
            }
            for (int k = 1; k <= K; k++) {
                int xx = horiz ? x - d * k : x;
                int yy = horiz ? y : y - d * k;
                if (xx < 0 || xx >= w || yy < 0 || yy >= h) break;
                size_t i = (size_t)yy * w + xx;
                sa2[n2] = dA[i]; sb2[n2] = dBc[i]; n2++;
            }
            if (n1 < 2 || n2 < 2) continue;

            /* 细线排除：任一侧 K 内出现反向强梯度 → 跳过（电线等） */
            int thin = 0;
            for (int s = 0; s < 2 && !thin; s++) {
                int sgn = s ? -d : d;
                for (int k = 1; k <= K; k++) {
                    int xx = horiz ? x + sgn * k : x;
                    int yy = horiz ? y : y + sgn * k;
                    if (xx < 1 || xx + 1 >= w || yy < 1 || yy + 1 >= h) break;
                    float gg = horiz
                        ? (T[(size_t)yy * w + xx + 1] - T[(size_t)yy * w + xx - 1])
                        : (T[(size_t)(yy + 1) * w + xx] - T[(size_t)(yy - 1) * w + xx]);
                    if (gg * (float)d < 0.0f && fabsf(gg) > 0.4f * mag) { thin = 1; break; }
                }
            }
            if (thin) continue;

            /* 只钳真正超出两侧区间的冲激 */
            size_t ip = (size_t)y * w + x;
            if (chmask & 1) {
                float lo, hi;
                float m1 = own_med_small(sa, n1), m2 = own_med_small(sa2, n2);
                if (m1 < m2) { lo = m1; hi = m2; } else { lo = m2; hi = m1; }
                float v = dA[ip], t = v;
                if (v < lo - slack)      t = lo;
                else if (v > hi + slack) t = hi;
                if (t != v) { dA[ip] = v + (t - v) * mix; nfix++; }
            }
            if (chmask & 2) {
                float lo, hi;
                float m1 = own_med_small(sb, n1), m2 = own_med_small(sb2, n2);
                if (m1 < m2) { lo = m1; hi = m2; } else { lo = m2; hi = m1; }
                float v = dBc[ip], t = v;
                if (v < lo - slack)      t = lo;
                else if (v > hi + slack) t = hi;
                if (t != v) { dBc[ip] = v + (t - v) * mix; nfix++; }
            }
        }
    }
    if (getenv("X3F_OWN_EDGEFIX_STAT"))
        fprintf(stderr, "[edgefix] edges=%ld fixed=%ld (%.2f%%)\n",
                nedge, nfix, nedge ? 100.0 * (double)nfix / (double)nedge : 0.0);
    return nfix;
}

/* ---- 上采样值域钳制：削掉双三次过冲（默认关）--------------------------
 *
 * 双三次（a=−0.75）在硬边处会产生过冲/下冲（ringing）：半层 delta 的台阶
 * 经 2x 上采样后，边缘外侧出现超出两端样本范围的尖峰 → 合成后 R/G 在该处
 * 下陷 → 暗屋顶/亮天空边界"只比天空深一点点"的暗线。
 *
 * 修法（比"判断哪个 delta 是冲激"更正确）：插值结果不应超出它据以插值的
 * 那些样本的范围。对全分辨率像素取其对应的 2×2 半层样本 min/max，把双三次
 * 输出钳回该区间。对 dB/dG 对称处理，不破坏 delta 与顶层 base 的配套关系
 * （2026-10-31 教训：单修 delta 会破坏该配套，实测净效果为负）。
 *
 * 实测（2026-10-31，sigma_dp3_quattro_09 楼顶区，147 列对齐平均）：
 * 贴边 1px 下陷 基线 1.60% → 钳制 1.29%，x3fuse(Rust 移植) 为 1.27%
 * —— 基本追平 x3fuse。改动 83.5% 落在边缘区（平坦区仅 0.85%），
 * 天空细节零损失（局部 std 0.00305 前后完全一致）。
 *
 * 默认**开**（2026-10-31）；显式 X3F_OWN_UPCLAMP=0 关闭。
 */
typedef struct {
    float *up; int ew, eh;
    const float *half; int hw, hh;
} ClampCtx;

static void clamp_rows(void *vctx, int y0, int y1)
{
    ClampCtx *c = (ClampCtx *)vctx;
    for (int y = y0; y < y1; y++) {
        int j0 = y >> 1, j1 = j0 + 1;
        if (j1 >= c->hh) j1 = c->hh - 1;
        const float *r0 = c->half + (size_t)j0 * c->hw;
        const float *r1 = c->half + (size_t)j1 * c->hw;
        float *op = c->up + (size_t)y * c->ew;
        for (int x = 0; x < c->ew; x++) {
            int i0 = x >> 1, i1 = i0 + 1;
            if (i1 >= c->hw) i1 = c->hw - 1;
            float a = r0[i0], b = r0[i1], c2 = r1[i0], d = r1[i1];
            float lo = a, hi = a;
            if (b < lo) lo = b; else if (b > hi) hi = b;
            if (c2 < lo) lo = c2; else if (c2 > hi) hi = c2;
            if (d < lo) lo = d; else if (d > hi) hi = d;
            float v = op[x];
            if (v < lo)      op[x] = lo;
            else if (v > hi) op[x] = hi;
        }
    }
}

/* 逐行独立（每个输出像素只看自己那 2×2 半层样本），按行分块并行，
 * 结果与串行逐位一致。全幅 5424×3616 单色平面本来要 ~0.15s。 */
static void clamp_upsample_range(float *up, int ew, int eh,
                                 const float *half, int hw, int hh)
{
    ClampCtx c;
    c.up = up; c.ew = ew; c.eh = eh;
    c.half = half; c.hw = hw; c.hh = hh;
    run_rows(clamp_rows, &c, eh, 16);
}

/* 半层 YUV 正/反变换的行函数（NLM 的输入输出格式）。
 * 每个像素只读写自己的位置，行间无依赖 → 按行分块并行逐位一致。 */
typedef struct {
    const float *t2, *dB, *dG;
    float *dBw, *dGw;
    uint16_t *yuv;
    int hw, hx, hy, aw2;
} HalfYuvCtx;

static void half_yuv_fwd_rows(void *v, int y0, int y1)
{
    HalfYuvCtx *c = (HalfYuvCtx *)v;
    for (int y = y0; y < y1; y++) {
        const float *tr2 = c->t2 + (size_t)(c->hy + y) * c->hw + c->hx;
        const float *rb  = c->dB + (size_t)(c->hy + y) * c->hw + c->hx;
        const float *rg  = c->dG + (size_t)(c->hy + y) * c->hw + c->hx;
        uint16_t *orow = c->yuv + (size_t)y * c->aw2 * 3;
        for (int x = 0; x < c->aw2; x++) {
            orow[x * 3]     = sat16f(4.0f * tr2[x]);
            orow[x * 3 + 1] = sat16f(2.0f * rb[x] + 32768.0f);
            orow[x * 3 + 2] = sat16f(rb[x] - 2.0f * rg[x] + 32768.0f);
        }
    }
}

static void half_yuv_inv_rows(void *v, int y0, int y1)
{
    HalfYuvCtx *c = (HalfYuvCtx *)v;
    for (int y = y0; y < y1; y++) {
        float *rb = c->dBw + (size_t)(c->hy + y) * c->hw + c->hx;
        float *rg = c->dGw + (size_t)(c->hy + y) * c->hw + c->hx;
        const uint16_t *irow = c->yuv + (size_t)y * c->aw2 * 3;
        for (int x = 0; x < c->aw2; x++) {
            float u = (float)irow[x * 3 + 1] - 32768.0f;
            float vv = (float)irow[x * 3 + 2] - 32768.0f;
            rb[x] = 0.5f * u;
            rg[x] = 0.5f * (rb[x] - vv);
        }
    }
}

/* 色度提取（Y 与 Cr/Cb）：每像素独立，按行分块。
 * 表达式与原串行版逐字相同（double 运算顺序不变）→ 逐位一致。 */
typedef struct { const float *out; float *cr, *cb; int w; } ChromaExCtx;
static void chroma_ex_rows(void *v, int y0, int y1)
{
    ChromaExCtx *c = (ChromaExCtx *)v;
    for (int y = y0; y < y1; y++) {
        for (int x = 0; x < c->w; x++) {
            size_t i = (size_t)y * c->w + x;
            double yy = 0.2126 * c->out[i * 3] + 0.7152 * c->out[i * 3 + 1]
                      + 0.0722 * c->out[i * 3 + 2];
            c->cr[i] = (float)(c->out[i * 3] - yy);
            c->cb[i] = (float)(c->out[i * 3 + 2] - yy);
        }
    }
}

/* fuse_quattro — Quattro 三层融合。
 * clipmask（可 NULL）：ew*eh 每像素 4 位标记，按**修复前 raw**判——
 * bit0/1 = 半层 R/G 裁剪（一个半层像素置位覆盖 2×2 全分辨率块）、
 * bit2 = 顶层 B 裁剪、bit3 = 高光基座已用半层亮度重建（hlbase，
 * 渲染层 mode7 据此跳过去饱和）。阈值 = sat[c]（raw 饱和值，通常 = maxraw）。 */

/* float 2D separable box blur（原地，tmp 工作缓冲；两遍调用 ≈ 三角核）。
 * 高光基座重建的局部 k 标定用。 */
static void blur_f2d(float *buf, float *tmp, int w, int h, int r)
{
    if (r <= 0) return;
    double *pre = malloc((size_t)(w > h ? w : h) * sizeof(double));
    if (!pre) return;
    for (int y = 0; y < h; y++) {
        float *row = buf + (size_t)y * w;
        double acc = 0.0;
        for (int x = 0; x < w; x++) { acc += row[x]; pre[x] = acc; }
        for (int x = 0; x < w; x++) {
            int x0 = x - r; if (x0 < 0) x0 = 0;
            int x1 = x + r; if (x1 >= w) x1 = w - 1;
            tmp[x] = (float)((pre[x1] - (x0 > 0 ? pre[x0 - 1] : 0.0)) / (double)(x1 - x0 + 1));
        }
        memcpy(row, tmp, (size_t)w * sizeof(float));
    }
    for (int x = 0; x < w; x++) {
        double acc = 0.0;
        for (int y = 0; y < h; y++) { acc += buf[(size_t)y * w + x]; pre[y] = acc; }
        for (int y = 0; y < h; y++) {
            int y0 = y - r; if (y0 < 0) y0 = 0;
            int y1 = y + r; if (y1 >= h) y1 = h - 1;
            buf[(size_t)y * w + x] =
                (float)((pre[y1] - (y0 > 0 ? pre[y0 - 1] : 0.0)) / (double)(y1 - y0 + 1));
        }
    }
    free(pre);
}

/* 解码级根因修复：天空侧细节层单边抑制（X3F_OWN_DETAILFIX，默认关）。
 * 半层细节 dB/dG = half−t2 在天空→物体强边上方有单边宽负下陷（半层
 * 电荷扩散滞后），双三次上采样加回全分辨率 → 每边一条暗线。
 * 本函数在「边上方天空侧窄带」把 dB/dG 向 0 衰减；同一因子 w 缩放
 * 保 upB:upG 比例 = 保色相；不动建筑侧、不动平坦区纹理。
 * 作用在半层（双三次前），衰减随上采样自然平滑。 */
static int cmp_float(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

static void detail_fix_skyside(float *dB, float *dG, const float *t2,
                               int hw, int hh)
{
    const char *p;
    float strength = 0.80f;     /* 带内衰减到 (1−strength) */
    int   band     = 22;        /* 掩码半层像素宽（≈44 全分辨率）*/
    int   emargin  = 5;         /* 边处保留带（保真实锐边，半层像素）*/
    float sky_q    = 0.90f;     /* 天空基线在顶区的分位（亮）*/
    if ((p = getenv("X3F_OWN_DETAILFIX_STR")))   strength = (float)atof(p);
    if ((p = getenv("X3F_OWN_DETAILFIX_BAND")))  band = atoi(p);
    if ((p = getenv("X3F_OWN_DETAILFIX_EMARGIN"))) emargin = atoi(p);
    if ((p = getenv("X3F_OWN_DETAILFIX_SKYQ")))  sky_q = (float)atof(p);
    if (strength <= 0.0f) return;
    if (strength > 0.98f) strength = 0.98f;
    if (band < 1) band = 1;
    if (emargin < 0) emargin = 0;
    if (emargin > band) emargin = band;

    const size_t nh = (size_t)hw * hh;
    float *w = malloc(nh * sizeof(float));
    if (!w) return;
    for (long i = 0; i < (long)nh; i++) w[i] = 1.0f;

    /* 逐列局部天空基线（顶区亮值分位），再找天空→物体向下跳变（建筑顶），
     * 在边上方天空侧窄带置 w<1。用局部基线而非全局分位：亮天空会把全局
     * 分位抬太高、近地平线的楼顶反不被识别（前次全局分位实测全图 w=1）。 */
    int toprows = hh / 8; if (toprows < 20) toprows = 20;
    int nedge = 0;
    float sky_min = 1e30f, sky_max = -1e30f;
    for (int x = 0; x < hw; x++) {
        float *col = malloc((size_t)toprows * sizeof(float));
        if (!col) continue;
        for (int y = 0; y < toprows; y++)
            col[y] = t2[(size_t)y * hw + x];
        qsort(col, toprows, sizeof(float), cmp_float);
        float sky = col[(size_t)(sky_q * (toprows - 1))];
        free(col);
        if (sky <= 1e-3f) continue;
        if (sky < sky_min) sky_min = sky;
        if (sky > sky_max) sky_max = sky;
        int y = 0;
        while (y + 1 < hh) {
            float v  = t2[(size_t)y * hw + x];
            float vn = t2[(size_t)(y + 1) * hw + x];
            if (v >= 0.85f * sky && vn <= 0.75f * sky) {
                int k = y + 1;
                while (k < hh && t2[(size_t)k * hw + x] <= 0.80f * sky) k++;
                if (k - (y + 1) >= 4) {            /* 持续暗 → 确认建筑顶 */
                    int ey = y;                      /* 天空侧末行 = 跳变行 */
                    int y0 = ey - band; if (y0 < 0) y0 = 0;
                    for (int yy = y0; yy <= ey; yy++) {
                        int off = ey - yy;           /* 距边（半层像素）*/
                        float wv = (off < emargin)
                            ? 1.0f - strength * ((float)off / (float)emargin)
                            : 1.0f - strength;
                        size_t idx = (size_t)yy * hw + x;
                        if (wv < w[idx]) w[idx] = wv;
                    }
                    nedge++;
                    y = k - 1;                       /* 跳过物体段 */
                    continue;
                }
            }
            y++;
        }
    }
    if (getenv("X3F_OWN_DETAILFIX_STAT"))
        fprintf(stderr, "[detailfix] nedge=%d sky_baseline[%.1f,%.1f] hw=%d hh=%d\n",
                nedge, sky_min, sky_max, hw, hh);

    /* 平滑掩码（水平+垂直 2px）抑制列抖动 */
    float *tmp = malloc(nh * sizeof(float));
    if (tmp) { blur_f2d(w, tmp, hw, hh, 2); free(tmp); }

    if (getenv("X3F_OWN_LAGDUMP")) {
        own_dump_lag("lag_fix_w.txt", w, hw, hh, 1);
        float *dBc = malloc(nh * sizeof(float));
        float *dGc = malloc(nh * sizeof(float));
        if (dBc && dGc) {
            for (long i = 0; i < (long)nh; i++) { dBc[i] = dB[i] * w[i]; dGc[i] = dG[i] * w[i]; }
            own_dump_lag("lag_fix_dB.txt", dBc, hw, hh, 1);
            own_dump_lag("lag_fix_dG.txt", dGc, hw, hh, 1);
            free(dBc); free(dGc);
        }
    }

    /* 应用：同一因子缩放 dB/dG（保 upB:upG 比例 = 保色相）*/
    for (long i = 0; i < (long)nh; i++) { dB[i] *= w[i]; dG[i] *= w[i]; }
    free(w);
}

static float *fuse_quattro(const uint16_t *half, int hw, int hh,
                           const uint16_t *top, int topw,
                           const double black[3], const int act[4],
                           uint8_t *clipmask, const double sat[3])
{
    int ew = hw * 2, eh = hh * 2;
    float *t2  = malloc((size_t)hw * hh * sizeof(float));
    float *dB  = malloc((size_t)hw * hh * sizeof(float));
    float *dG  = malloc((size_t)hw * hh * sizeof(float));
    float *upB = malloc((size_t)ew * eh * sizeof(float));
    float *upG = malloc((size_t)ew * eh * sizeof(float));
    float  *out = malloc((size_t)ew * eh * 3 * sizeof(float));
    if (!t2 || !dB || !dG || !upB || !upG || !out) die("out of memory");

    const float blk0 = (float)black[0], blk1 = (float)black[1], blk2 = (float)black[2];
    for (int y = 0; y < hh; y++) {
        for (int x = 0; x < hw; x++) {
            size_t ti = (size_t)y * hw + x;
            size_t q1 = (size_t)(2 * y) * topw + 2 * x;
            size_t q2 = (size_t)(2 * y + 1) * topw + 2 * x;
            float tsum = (top[q1] + top[q1 + 1] + top[q2] + top[q2 + 1]) * 0.25f - blk2;
            t2[ti] = tsum;
            /* 【2026-10-21 紫边问题】旧公式 dB = halfR − t2 在强彩边电荷扩散
             * 滞后区会把半层共模下降泄漏进 delta（G 掉 ~800、B(top) 不掉 →
             * B−G≈800 品红边；fuse 6 个品红像素 vs own 481）。
             * 尝试过的 BMT 平均基线 (0.75R−0.25G−0.5t2) 会把色度整体减半
             * （花变粉）——错误，已回退。x3fuse 的半层 T 通道同样是下采样
             * 顶层，色度定义相同；它不紫边靠的是 Y 替换后的 NLM 色度平滑。
             * 当前对策：保持旧 delta，滞后区负摆动由下面 X3FDEC_LAGCLAMP
             * 的色度下限钳制处理。 */
            float r = (float)half[ti * 3]     - blk0;
            float g = (float)half[ti * 3 + 1] - blk1;
            dB[ti] = r - tsum;
            dG[ti] = g - tsum;
            if (clipmask) {
                uint8_t bits = (uint8_t)((half[ti * 3]     >= sat[0] ? 1 : 0) |
                                         (half[ti * 3 + 1] >= sat[1] ? 2 : 0));
                if (bits) {
                    clipmask[(size_t)(2 * y)     * ew + 2 * x]     |= bits;
                    clipmask[(size_t)(2 * y)     * ew + 2 * x + 1] |= bits;
                    clipmask[(size_t)(2 * y + 1) * ew + 2 * x]     |= bits;
                    clipmask[(size_t)(2 * y + 1) * ew + 2 * x + 1] |= bits;
                }
            }
        }
    }

    /* ===== 高光基座重建（过曝矫正 mode7 专属；2026-10-22 晚） =====
     * 顶层裁剪 → base 平顶，但半层 R/G 像素更小、饱和点更晚，裁剪区往往
     * 还有完整亮度梯度（SPP HLP/F23 重建用的就是这份数据）。本段在半层
     * 还活着的时候把平顶基座按半层亮度提回去：
     *   - 局部标定 t2/Lh 比例（只在未裁亮区统计，box blur 摊平成 k 图，
     *     自动跟随光源光谱）；
     *   - est = k·Lh，只提不压，w 随顶层饱和度 smoothstep 渐入（部分
     *     裁剪子块过渡连续）；
     *   - dB/dG 不动 —— 恢复的亮度梯度与真实色彩全保留；
     *   - 半层也裁尽的块没信息，保持平顶（渲染层 mode7 会去饱和成白）。
     * 仅当 clipmask 请求（mode7）时启用，默认出图逐位不变；X3F_OE_HLBASE=0 关。 */
    float *baseL = NULL;
    if (clipmask != NULL) {
        int hlbase = 1;
        const char *ehb = getenv("X3F_OE_HLBASE");
        if (ehb != NULL && ehb[0] == '0') hlbase = 0;
        if (hlbase) {
            const double Tclip = (double)sat[2] - blk2;   /* 顶层裁剪电平 */
            const double H0 = (double)sat[0] - blk0, H1 = (double)sat[1] - blk1;
            const size_t nh = (size_t)hw * hh;
            float *kmap = malloc(nh * sizeof(float));
            float *num  = malloc(nh * sizeof(float));
            float *den  = malloc(nh * sizeof(float));
            float *wt   = malloc(nh * sizeof(float));
            float *tbuf = malloc(nh * sizeof(float));
            if (kmap && num && den && wt && tbuf) {
                /* 统计有效未裁亮区 */
                long nvalid = 0;
                for (int y = 0; y < hh; y++)
                    for (int x = 0; x < hw; x++) {
                        size_t ti = (size_t)y * hw + x;
                        double Lh = 0.5 * ((double)half[ti * 3] - blk0 +
                                           (double)half[ti * 3 + 1] - blk1);
                        double T = t2[ti];
                        int ok = (T < 0.85 * Tclip && T > 0.05 * Tclip &&
                                  Lh > 0.08 * Tclip && Lh < 0.9 * Tclip);
                        wt[ti]  = ok ? 1.0f : 0.0f;
                        num[ti] = ok ? (float)T : 0.0f;
                        den[ti] = ok ? (float)Lh : 0.0f;
                        nvalid += ok;
                    }
                if (nvalid > 500) {
                    /* 局部 k 图：两遍 box blur（r=6 半层像素 ≈ 12 全分辨率） */
                    blur_f2d(num, tbuf, hw, hh, 6);
                    blur_f2d(den, tbuf, hw, hh, 6);
                    blur_f2d(wt,  tbuf, hw, hh, 6);
                    for (size_t ti = 0; ti < nh; ti++)
                        kmap[ti] = (wt[ti] > 1e-3f && den[ti] > 1e-3f)
                                   ? num[ti] / den[ti] : 0.0f;

                    baseL = calloc((size_t)ew * eh, sizeof(float));
                    if (baseL) {
                        long nrec = 0, nhot = 0, nhalfok = 0, nkzero = 0;
                        for (int y = 0; y < hh; y++)
                            for (int x = 0; x < hw; x++) {
                                size_t ti = (size_t)y * hw + x;
                                double T = t2[ti];
                                if (T < 0.88 * Tclip) continue;
                                nhot++;
                                double r = (double)half[ti * 3] - blk0;
                                double g = (double)half[ti * 3 + 1] - blk1;
                                if (r >= H0 || g >= H1) continue;   /* 半层也裁：没信息 */
                                nhalfok++;
                                double Lh = 0.5 * (r + g);
                                if (Lh <= 0.0 || kmap[ti] <= 0.0) { nkzero++; continue; }
                                double est = (double)kmap[ti] * Lh;
                                if (est > 2.0 * Tclip) est = 2.0 * Tclip;
                                if (est <= T) continue;             /* 只提不压 */
                                double w = (T - 0.88 * Tclip) / (0.10 * Tclip);
                                if (w > 1.0) w = 1.0;
                                w = w * w * (3.0 - 2.0 * w);
                                float nb = (float)(T + w * (est - T));
                                size_t q = (size_t)(2 * y) * ew + 2 * x;
                                baseL[q] = baseL[q + 1] =
                                baseL[q + ew] = baseL[q + ew + 1] = nb;
                                if (clipmask) {
                                    clipmask[q] |= 8; clipmask[q + 1] |= 8;
                                    clipmask[q + ew] |= 8; clipmask[q + ew + 1] |= 8;
                                }
                                nrec++;
                            }
                        if (getenv("X3FDEC_HLBASEDBG"))
                            fprintf(stderr, "[hlbase] Tclip=%.0f hot=%ld halfok=%ld kzero=%ld rec=%ld /%ld\n",
                                    Tclip, nhot, nhalfok, nkzero, nrec, (long)nh);
                    }
                }
            }
            free(kmap); free(num); free(den); free(wt); free(tbuf);
        }
    }

    /* 半层缺陷修复：出厂标定表已应用时仅点状列检测+逐像素修复（键合阴影列
     * 点状缺陷不在标定表内，需保留；整列死列检测误伤内容，见
     * repair_bad_pixels_f23 注释）。X3FDEC_NODEFECT 关闭 /
     * X3FDEC_STATDEFECT 强制完整统计式修复（对照实验） */
    if (!getenv("X3FDEC_NODEFECT")) {
        g_defect_dot_only = g_calib_chroma_done && !getenv("X3FDEC_STATDEFECT");
        float *dg2 = malloc((size_t)hw * hh * 2 * sizeof(float));
        if (!dg2) die("out of memory");
        for (size_t i = 0; i < (size_t)hw * hh; i++) {
            dg2[2 * i]     = dB[i];
            dg2[2 * i + 1] = dG[i];
        }
        repair_defect_deltas(dg2, hw, hh, 100.0, 50.0, 150.0, 0.25);
        for (size_t i = 0; i < (size_t)hw * hh; i++) {
            dB[i] = dg2[2 * i];
            dG[i] = dg2[2 * i + 1];
        }
        free(dg2);
        /* 列修复后的孤立点缺陷（delta 域，见 repair_point_defects_f）。
         * 必须在列修复之后：键合列未修时点修复会整列误触发（15 万次/张）
         * → 上采样后沿缺陷列的垂直点状竖纹。 */
        if (!getenv("X3FDEC_NOPOINTFIX")) {
            repair_point_defects_f(dB, 1, 0, hw, hh, 450.0, 600.0, 300.0);
            repair_point_defects_f(dG, 1, 0, hw, hh, 450.0, 600.0, 300.0);
        }
    }
    /* ===== 色度降噪 + 上采样 =====
     * XFCHAIN=1（默认）: x3fuse 等价完整链路 —— 半层 YUV NLM（σ=300）→
     * V 3×3 中值 → 低频残差 → 双三次 2x → 全分辨率 YUV NLM（h={0,σ,2σ}）。
     * 根因: 半层 R/G 电荷扩散滞后（强亮度边缘处 2~4 半层像素平滑斜坡
     * 500~1300 raw）vs 顶层锐边 → 融合错位带色度误差 → 紫/绿色块。
     * 教训: 单独复刻半层 NLM 无效（双边后 delta 平滑、patch 权重二值化、
     * 滞后带自相似）—— 必须整链：Y 主导 patch 距离 + 全分辨率 NLM +
     * V 中值 + 低频残差。X3FDEC_NLM_H/_NLM_SCALE 调 σ；XFCHAIN=0 回退
     * 旧链路（双边 + 引导上采样）。
     * 2026-10-24: 该链实测在暗带把 R−G 推高 +3~5.5 灰阶（vs SPP，0337
     * 显示域），是暗部发红主力量级之一；当时默认 σ=0 关闭、改用渲染层
     * 降噪滑杆。2026-10-25: 用户工作点定为 NLM_H=200，故默认 σ=200
     * （整链默认开）；显式 X3FDEC_NLM_H<=0 仍可关闭整链。如需更强/更弱
     * 降噪改 env 即可；暗带发红风险在可接受范围（用户 1884/0337 目检通过）。 */
    {
        int xfchain = 1;
        /* 2026-11-14：σ 改由宿主的 nlm / dn 下发（见 own_dn_setup）。
         * nlm=5（GUI/CLI 默认）+ strength=1 → 200.0，逐位零回归；
         * X3FDEC_NLM_H / _NLM_SCALE 仍是诊断用的显式覆盖，优先级不变。 */
        double sigma = own_dn_sigma_base();
        const char *ev = getenv("X3FDEC_XFCHAIN");
        if (ev) xfchain = atoi(ev);
        if ((ev = getenv("X3FDEC_NLM_H"))) {
            double v = atof(ev);
            sigma = (v <= 0.0) ? 0.0 : v;   /* 显式 <=0 = 关闭整链 */
        }
        if ((ev = getenv("X3FDEC_NLM_SCALE")) && atof(ev) > 0.0)
            sigma *= atof(ev);
        /* 三通道分别按 dn 开：色度关则 U/V 权重 0；亮度开则 Y 也降。
         * 下面把 sigma 收敛成「色度档」，Y 单独走 dnHy（变量 hy 已被 active
         * 区域左上角占用，别重名）。 */
        const double dnHc = own_dn_h_chroma(sigma);
        const double dnHy = own_dn_h_luma(sigma);
        const int    dnTw = own_dn_patch(), dnSw = own_dn_search();
        sigma = dnHc;

        /* --- 诊断 dump：半层色度 NLM 前（X3F_OWN_LAGDUMP） --- */
        own_lag_parse();
        if (getenv("X3F_OWN_LAGDUMP")) {
            fprintf(stderr, "[lagdump] ew=%d eh=%d hw=%d hh=%d topw=%d "
                    "act=[%d %d %d %d] frame_nr=%d sigma=%.1f\n",
                    ew, eh, hw, hh, topw, act[0], act[1], act[2], act[3],
                    g_own_frame_nr, sigma);
            own_dump_lag("lag_pre_dB.txt", dB, hw, hh, 1);
            own_dump_lag("lag_pre_dG.txt", dG, hw, hh, 1);
            own_dump_lag("lag_pre_t2.txt", t2, hw, hh, 1);
        }

        if (xfchain && (sigma > 0.01 || dnHy > 0.01)) {
            int hx = act[0], hy = act[1], aw2 = act[2], ah2 = act[3];
            if (hx < 0 || hy < 0 || aw2 <= 0 || ah2 <= 0 ||
                hx + aw2 > hw || hy + ah2 > hh) {
                hx = 0; hy = 0; aw2 = hw; ah2 = hh;
            }
            if (getenv("X3F_OWN_LAGDUMP"))
                fprintf(stderr, "[lagdump] active_eff=[%d %d %d %d] "
                        "crop_half=[%d %d %d %d]\n",
                        hx, hy, aw2, ah2,
                        g_lag_cx/2 - g_lag_w/4, g_lag_cy/2 - g_lag_h/4,
                        g_lag_w/2, g_lag_h/2);
            /* --- 半层 YUV NLM（active 子区独立图，BORDER 由子区内容反射，
             * 与 x3fuse active view 一致）。Y=4·T（T=下采样顶层，黑电平
             * 已扣）；U=2B−2T=2·dB；V=B−2M+T=dB−2·dG。O_UV=32768。
             * SFD 非基准帧（own_set_frame_nr(0)）整段跳过：dB/dG 原样
             * 保留，下面的双三次 2x 语义不变，降噪由合成后的 Stage1 收。 --- */
            /* --- 列 FPN 校正（双重结构门控，61998cf）：此前只活在 XFCHAIN=0
             * 回退分支；69603f9 把默认工作点切到整链 NLM（σ=200）后，本校正
             * 被埋进 else，默认链路的解码层列纹修正停摆 —— 亮平区竖向条带
             * 回归（Mac 10-25 的验证基线正是回退分支工作点）。按修复说明把
             * 校正挂回默认链：半层 delta 域全幅、逐 128 行带、双门控
             * （X3FDEC_FPNT/_FPNT2/_FPNGATE 同一套），在半层 YUV NLM 之前
             * 施加 —— NLM 的 patch 距离对列噪声敏感（带噪 patch 权重失真、
             * 列噪被扩散），与回退分支"先剥列纹再降噪"同序。SFD 非基准帧
             * 跳过，与回退分支同语义。 --- */
            if (g_own_frame_nr) {
                FpnOnlyJob jb2 = { dB, hw, hh, 128, "B" };
                FpnOnlyJob jg2 = { dG, hw, hh, 128, "G" };
                run2(fpn_only_job, &jb2, &jg2);
                stage_t("  fuse: fpn banded");
            }
            if (g_own_frame_nr) {
            uint16_t *yuv = malloc((size_t)aw2 * ah2 * 3 * sizeof(uint16_t));
            if (!yuv) die("out of memory");
            {
                HalfYuvCtx yc;
                yc.t2 = t2; yc.dB = dB; yc.dG = dG;
                yc.dBw = dB; yc.dGw = dG;
                yc.yuv = yuv; yc.hw = hw; yc.hx = hx; yc.hy = hy; yc.aw2 = aw2;
                run_rows(half_yuv_fwd_rows, &yc, ah2, 16);
                stage_t("  fuse: nlm half begin");
                denoise_nlm_yuv16(yuv, aw2, ah2, sigma, dnHy, dnTw, dnSw);
                run_rows(half_yuv_inv_rows, &yc, ah2, 16);
            }
            free(yuv);
            stage_t("  fuse: nlm half");
            }

            /* --- 诊断 dump：半层色度 NLM 后（X3F_OWN_LAGDUMP） --- */
            if (getenv("X3F_OWN_LAGDUMP")) {
                own_dump_lag("lag_half_dB.txt", dB, hw, hh, 1);
                own_dump_lag("lag_half_dG.txt", dG, hw, hh, 1);
                fprintf(stderr, "[lagdump] wrote lag_half_*.txt\n");
            }

            /* --- 边缘 delta 冲激修复（默认关；X3F_OWN_EDGEFIX=1 开）--- */
            if (getenv("X3F_OWN_EDGEFIX")) {
                float magt = 300.0f, slack = 40.0f, mixv = 1.0f;
                const char *p;
                if ((p = getenv("X3F_OWN_EDGEFIX_MAG")) && atof(p) > 0.0)
                    magt = (float)atof(p);
                if ((p = getenv("X3F_OWN_EDGEFIX_OVER")) && atof(p) > 0.0)
                    slack = (float)atof(p);
                if ((p = getenv("X3F_OWN_EDGEFIX_MIX")))
                    mixv = (float)atof(p);
                int chmask = 3;
                if ((p = getenv("X3F_OWN_EDGEFIX_CH"))) chmask = atoi(p);
                fix_edge_delta_overshoot(dB, dG, t2, hw, hh,
                                         hx, hy, aw2, ah2, magt, slack, mixv,
                                         chmask);
                if (getenv("X3F_OWN_LAGDUMP")) {
                    own_dump_lag("lag_post_dB.txt", dB, hw, hh, 1);
                    own_dump_lag("lag_post_dG.txt", dG, hw, hh, 1);
                    fprintf(stderr, "[lagdump] wrote lag_post_*.txt\n");
                }
            }

            /* --- 解码级根因修复：天空侧细节层单边抑制（默认关 X3F_OWN_DETAILFIX）--- */
            if (getenv("X3F_OWN_DETAILFIX"))
                detail_fix_skyside(dB, dG, t2, hw, hh);

            /* --- 双三次 2x 上采样（全平面；x3fuse 语义无引导）--- */
            {
                RsJob rbj = { dB, hw, hh, upB };
                RsJob rgj = { dG, hw, hh, upG };
                run2(rs_job, &rbj, &rgj);
            }
            /* --- 顶层边感知重建（默认关；X3F_OWN_TOPEDGE=1 开）---
             * 排在值域钳制之前：重排权重后仍可能有双三次负 lobe 残过冲，
             * 交给 clamp_upsample_range 收最后一刀。 */
            maybe_topedge(upB, upG, dB, dG, t2, hw, hh, top, topw, blk2,
                          (double)sat[2] - (double)blk2);
            /* --- 值域钳制：削双三次过冲（默认开；=0 关闭）--- */
            {
                const char *uc = getenv("X3F_OWN_UPCLAMP");
                if (!uc || atoi(uc) != 0) {
                clamp_upsample_range(upB, ew, eh, dB, hw, hh);
                clamp_upsample_range(upG, ew, eh, dG, hw, hh);
                }
            }
            stage_t("  fuse: bicubic upsample");

            /* --- 全分辨率 YUV NLM（active 子区），h={0,σ,2σ}（V 双倍强度）。
             * SFD 非基准帧同样跳过：双三次上采样后的 upB/upG 原样进融合。 --- */
            if (g_own_frame_nr) {
            {
                int fw = aw2 * 2, fh = ah2 * 2;
                int fx0 = hx * 2, fy0 = hy * 2;
                uint16_t *yuvf = malloc((size_t)fw * fh * 3 * sizeof(uint16_t));
                uint16_t *outf = malloc((size_t)fw * fh * 3 * sizeof(uint16_t));
                if (!yuvf || !outf) die("out of memory");
                for (int y = 0; y < fh; y++) {
                    const uint16_t *trow = top + (size_t)(fy0 + y) * topw + fx0;
                    const float *rb = upB + (size_t)(fy0 + y) * ew + fx0;
                    const float *rg = upG + (size_t)(fy0 + y) * ew + fx0;
                    uint16_t *orow = yuvf + (size_t)y * fw * 3;
                    for (int x = 0; x < fw; x++) {
                        orow[x * 3]     = sat16f(4.0f * ((float)trow[x] - blk2));
                        orow[x * 3 + 1] = sat16f(2.0f * rb[x] + 32768.0f);
                        orow[x * 3 + 2] = sat16f(rb[x] - 2.0f * rg[x] + 32768.0f);
                    }
                }
                double hf[3] = { dnHy, sigma, 2.0 * sigma };
                nlm3_den(yuvf, fw, fh, hf, dnTw, dnSw, outf);
                for (int y = 0; y < fh; y++) {
                    float *rb = upB + (size_t)(fy0 + y) * ew + fx0;
                    float *rg = upG + (size_t)(fy0 + y) * ew + fx0;
                    const uint16_t *irow = outf + (size_t)y * fw * 3;
                    for (int x = 0; x < fw; x++) {
                        float u = (float)irow[x * 3 + 1] - 32768.0f;
                        float v = (float)irow[x * 3 + 2] - 32768.0f;
                        rb[x] = 0.5f * u;
                        rg[x] = 0.5f * (rb[x] - v);
                    }
                }
                free(yuvf); free(outf);
                /* --- 诊断 dump：全分辨率 NLM 后（X3F_OWN_LAGDUMP） --- */
                if (getenv("X3F_OWN_LAGDUMP")) {
                    own_dump_lag("lag_full_upB.txt", upB, ew, eh, 2);
                    own_dump_lag("lag_full_upG.txt", upG, ew, eh, 2);
                    fprintf(stderr, "[lagdump] wrote lag_full_*.txt\n");
                }
            }
            stage_t("  fuse: nlm full");
            }
        } else {
    /* 半层条纹噪声抑制：列向噪声经 2x 放大会拉成竖条纹，
     * 双边滤波在上采样前压掉高频噪声并保住边缘。
     * 顺序：先双边后 NLM —— NLM 的 patch 距离对列噪声敏感，
     * 带噪 patch 会让权重失真、把列噪扩散成点纹（实测）。
     * X3FDEC_DENOISE=值域σ（默认 8.0，设 0 关闭） */
    double dnsig = 8.0;
    {
        const char *e = getenv("X3FDEC_DENOISE");
        if (e) dnsig = atof(e);
    }
    /* XFCHAIN=0 的回退分支：双边滤波也是降噪，SFD 非基准帧一并跳过 */
    if (g_own_frame_nr && dnsig > 0.01) {
        LAZY_LOCK();
        bilateral_preinit(dnsig);
        LAZY_UNLOCK();
        ChJob jb = { dB, hw, hh, 128, dnsig, "B" };
        ChJob jg = { dG, hw, hh, 128, dnsig, "G" };
        run2(ch_fpn_job, &jb, &jg);
        stage_t("  fuse: fpn+bilateral");
    }
    /* 强彩边滞后 NLM（实验性，默认关）。半层滞后斜坡的色度误差
     * （花瓣尖品红边，own 665 vs x3fuse 6 个品红像素）——本实现按
     * x3fuse denoise.rs 参数（h=300, 3×3/11×11, L1）复刻后实测近似
     * 空转（仅 34 像素变化）：双边滤波后 delta 已很平滑，patch 权重
     * 近二值化，NLM 退化为"复制近重复 patch"；滞后带自相似，修不动。
     * x3fuse 的干净来自其 spp 原始层降噪 + 两趟 NLM 的完整链路，
     * 需要时以 X3FDEC_LAGNLM=1 开启本 pass 做实验。 */
    if (getenv("X3FDEC_LAGNLM")) {
        float *dg2 = malloc((size_t)hw * hh * 2 * sizeof(float));
        if (!dg2) die("out of memory");
        for (size_t i = 0; i < (size_t)hw * hh; i++) {
            dg2[2 * i]     = dB[i];
            dg2[2 * i + 1] = dG[i];
        }
        repair_edge_lag_deltas(dg2, t2, hw, hh);
        for (size_t i = 0; i < (size_t)hw * hh; i++) {
            dB[i] = dg2[2 * i];
            dG[i] = dg2[2 * i + 1];
        }
        free(dg2);
        stage_t("  fuse: lag NLM");
    }
    /* 上采样: 默认双三次 2x（x3fuse 同款）。2026-10-24 改默认：
     * 引导双边（QT2B 思路，值域引导 = 半层同通道绝对值的双三次 2x）在
     * 中低调把半层色度拉出低频绿/品红斑块（6563 娃手背/大人脚踝绿斑，
     * 0337 暗部偏红同源），X3FDEC_GUIDE=0 实测全部大幅收敛且贴近
     * x3fuse；代价仅 1884 花瓣尖品红 9→30 像素（全图 19.6MP，视觉
     * 可忽略）。需要旧引导行为时 X3FDEC_GUIDE=1；
     * X3FDEC_GUIDE_K / _FD 调节值域/距离核。 */
    {
        int use_guide = 0;
        int guide_src = 0;   /* 0=半层同通道绝对值(默认), 1=t2 顶层对齐(紫边实验) */
        double gk = 0.0015, gfd = 1.1;
        const char *e = getenv("X3FDEC_GUIDE");
        if (e) use_guide = atoi(e);
        if ((e = getenv("X3FDEC_GUIDE_SRC"))) guide_src = atoi(e);
        if ((e = getenv("X3FDEC_GUIDE_K"))) gk = atof(e);
        if ((e = getenv("X3FDEC_GUIDE_FD"))) gfd = atof(e);
        if (use_guide) {
            float *absB = malloc((size_t)hw * hh * sizeof(float));
            float *absG = malloc((size_t)hw * hh * sizeof(float));
            float *guideB = malloc((size_t)ew * eh * sizeof(float));
            float *guideG = malloc((size_t)ew * eh * sizeof(float));
            if (!absB || !absG || !guideB || !guideG) die("out of memory");
            if (guide_src == 1) {
                /* 引导 = t2（下采样顶层）：色度过渡与顶层锐边对齐。
                 * 半层滞后带（R/G 边缘比 t2 晚 2~4 半层像素、平滑斜坡
                 * 500~1300 raw）用 half 引导会把色度边沿滞后带铺开 →
                 * 花尖品红/蓝边。t2 引导让权重跨过滞后带直接对齐锐边。 */
                RsJob rt = { t2, hw, hh, guideB };
                rs_job(&rt);
                memcpy(guideG, guideB, (size_t)ew * eh * sizeof(float));
                (void)absB; (void)absG;
            } else {
                /* 引导值 = 半层同通道绝对值（极性无关）。 */
                for (long i = 0; i < (long)hw * hh; i++) {
                    absB[i] = (float)half[i * 3]     - blk0;
                    absG[i] = (float)half[i * 3 + 1] - blk1;
                }
                RsJob rb = { absB, hw, hh, guideB };
                RsJob rg = { absG, hw, hh, guideG };
                run2(rs_job, &rb, &rg);
            }
            free(absB); free(absG);
            guided_up_preinit(gk, gfd);
            /* GPU 优先: B/G 两通道各整面一次派发 */
            {
                const char *e = getenv("X3FDEC_GPU_GUIDE");
                int gdone = own_gpu_available() && (!e || e[0] != '0') &&
                        own_gpu_guided_up_2x(dB, t2, guideB, upB, hw, hh,
                                             (float)gk, (float)gfd,
                                             (const float *)dtab,
                                             wv_lut, WVLUT_N) == 0 &&
                        own_gpu_guided_up_2x(dG, t2, guideG, upG, hw, hh,
                                             (float)gk, (float)gfd,
                                             (const float *)dtab,
                                             wv_lut, WVLUT_N) == 0;
                if (gdone) {
                    free(guideB); free(guideG);
                    stage_t("  fuse: guided upsample");
                    goto guided_done;
                }
                if (getenv("X3FDEC_GPUDBG"))
                    fprintf(stderr, "[gpu] guided fallback\n");
            }
            /* CPU 回退: B/G 通道各按输出行切成 GNT 块并行（总线程 2×GNT） */
            enum { GNT = 4 };
            GuJob gb[GNT], gg[GNT];
            for (int i = 0; i < GNT; i++) {
                gb[i] = (GuJob){ dB, hw, hh, guideB, t2, upB, gk, gfd,
                                 i * eh / GNT, (i + 1) * eh / GNT };
                gg[i] = (GuJob){ dG, hw, hh, guideG, t2, upG, gk, gfd,
                                 i * eh / GNT, (i + 1) * eh / GNT };
            }
            run_jobs(gu_job, gb, sizeof(GuJob), GNT);
            run_jobs(gu_job, gg, sizeof(GuJob), GNT);
            stage_t("  fuse: guided upsample");
            free(guideB); free(guideG);
        guided_done: ;
        } else {
            RsJob rb = { dB, hw, hh, upB };
            RsJob rg = { dG, hw, hh, upG };
            run2(rs_job, &rb, &rg);
            maybe_topedge(upB, upG, dB, dG, t2, hw, hh, top, topw, blk2,
                          (double)sat[2] - (double)blk2);
        }
        }
    }
    }

    if (getenv("X3FDEC_PLANEDBG")) {
        long n = (long)hw * hh;
        double s0 = 0, s1 = 0, st = 0, m0 = 0, m1 = 0, mt = 0;
        double sd01 = 0, sd0t = 0, sd1t = 0;
        double v0m = 0, v1m = 0;
        for (long i = 0; i < n; i++) {
            double v0 = (double)half[i * 3] - black[0];
            double v1 = (double)half[i * 3 + 1] - black[1];
            double vt = t2[i];
            m0 += v0; m1 += v1; mt += vt;
            s0 += v0 * v0; s1 += v1 * v1; st += vt * vt;
            sd01 += (v0 - v1) * (v0 - v1);
            sd0t += (v0 - vt) * (v0 - vt);
            sd1t += (v1 - vt) * (v1 - vt);
        }
        m0 /= n; m1 /= n; mt /= n;
        for (long i = 0; i < n; i++) {
            double v0 = (double)half[i * 3] - black[0] - m0;
            double v1 = (double)half[i * 3 + 1] - black[1] - m1;
            double vt = t2[i] - mt;
            v0m += v0 * vt; v1m += v1 * vt;
        }
        printf("dbg: rms(half0)=%.1f rms(half1)=%.1f rms(t2)=%.1f\n",
               sqrt(s0 / n), sqrt(s1 / n), sqrt(st / n));
        printf("dbg: rms(h0-h1)=%.1f rms(h0-t2)=%.1f rms(h1-t2)=%.1f\n",
               sqrt(sd01 / n), sqrt(sd0t / n), sqrt(sd1t / n));
        printf("dbg: corr(h0,t2)=%.4f corr(h1,t2)=%.4f  means %.1f/%.1f/%.1f\n",
               v0m / sqrt((s0 - n * m0 * m0) * (st - n * mt * mt) + 1e-9),
               v1m / sqrt((s1 - n * m1 * m1) * (st - n * mt * mt) + 1e-9),
               m0, m1, mt);
    }

    for (int y = 0; y < eh; y++) {
        const uint16_t *trow = top + (size_t)y * topw;
        float *orow = out + (size_t)y * ew * 3;
        float *brow = baseL ? baseL + (size_t)y * ew : NULL;
        uint8_t *mrow = clipmask ? clipmask + (size_t)y * ew : NULL;
        for (int x = 0; x < ew; x++) {
            size_t i = (size_t)y * ew + x;
            /* 高光基座重建：顶层裁剪的块用半层亮度估计替换平顶 */
            float base = (brow && brow[x] > 0.0f) ? brow[x] : (float)trow[x] - blk2;
            orow[x * 3]     = base + upB[i];
            orow[x * 3 + 1] = base + upG[i];
            orow[x * 3 + 2] = base;
            if (mrow && trow[x] >= sat[2]) mrow[x] |= 4;
        }
    }

    free(t2); free(dB); free(dG); free(upB); free(upG); free(baseL);
    if (getenv("X3FDEC_PLANEDBG")) {
        long n = (long)ew * eh;
        double s = 0, sg = 0, sb = 0;
        for (long i = 0; i < n; i++) {
            double dr = (double)out[i*3] - out[i*3+2];
            double dg = (double)out[i*3+1] - out[i*3+2];
            s += dr * dr; sg += dg * dg;
            sb += (double)out[i*3+2] * out[i*3+2];
        }
        printf("dbg: fused rms(c0-c2)=%.1f rms(c1-c2)=%.1f rms(ch2)=%.1f\n",
               sqrt(s/n), sqrt(sg/n), sqrt(sb/n));
    }
    return out;
}

/* ------------------------------------------------------------------ */
/* PNG 写出（8-bit RGB）                                                */
/* ------------------------------------------------------------------ */

#ifndef _WIN32
/* 这套依赖 zlib 的 compress2/compressBound（IDAT 压缩）。Windows 上没编
 * zlib —— own_write_preview_png 是调试预览口、当前无人调用，那边直接
 * 返 -1（见下），这几个 static 助手就不编译了。 */
#include <zlib.h>

static uint32_t crc_table[256];
static void crc_init(void)
{
    for (uint32_t n = 0; n < 256; n++) {
        uint32_t c = n;
        for (int k = 0; k < 8; k++)
            c = (c & 1) ? 0xedb88320u ^ (c >> 1) : c >> 1;
        crc_table[n] = c;
    }
}
static uint32_t crc_calc(const uint8_t *buf, size_t len)
{
    uint32_t crc = 0xffffffffu;
    for (size_t i = 0; i < len; i++)
        crc = crc_table[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
    return crc ^ 0xffffffffu;
}

static void png_chunk(FILE *f, const char *type, const uint8_t *data, size_t len)
{
    uint8_t hdr[8];
    hdr[0] = (uint8_t)(len >> 24); hdr[1] = (uint8_t)(len >> 16);
    hdr[2] = (uint8_t)(len >> 8);  hdr[3] = (uint8_t)len;
    memcpy(hdr + 4, type, 4);
    fwrite(hdr, 1, 8, f);
    if (len) fwrite(data, 1, len, f);
    uint8_t *tmp = malloc(len + 4);
    if (!tmp) die("out of memory");
    memcpy(tmp, type, 4);
    if (len) memcpy(tmp + 4, data, len);
    uint32_t crc = crc_calc(tmp, len + 4);
    free(tmp);
    uint8_t cb[4] = { (uint8_t)(crc >> 24), (uint8_t)(crc >> 16),
                      (uint8_t)(crc >> 8), (uint8_t)crc };
    fwrite(cb, 1, 4, f);
}

static void write_png_rgb8(const char *path, int w, int h, const uint8_t *rgb)
{
    static int crc_ready = 0;
    if (!crc_ready) { crc_init(); crc_ready = 1; }
    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "x3fdec: cannot write %s\n", path); exit(1); }
    static const uint8_t sig[8] = { 137, 80, 78, 71, 13, 10, 26, 10 };
    fwrite(sig, 1, 8, f);

    uint8_t ihdr[13];
    ihdr[0] = (uint8_t)(w >> 24); ihdr[1] = (uint8_t)(w >> 16);
    ihdr[2] = (uint8_t)(w >> 8);  ihdr[3] = (uint8_t)w;
    ihdr[4] = (uint8_t)(h >> 24); ihdr[5] = (uint8_t)(h >> 16);
    ihdr[6] = (uint8_t)(h >> 8);  ihdr[7] = (uint8_t)h;
    ihdr[8] = 8; ihdr[9] = 2; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;
    png_chunk(f, "IHDR", ihdr, 13);

    size_t stride = 1 + (size_t)w * 3;
    size_t rawlen = (size_t)h * stride;
    uint8_t *raw = malloc(rawlen);
    if (!raw) die("out of memory");
    for (int y = 0; y < h; y++) {
        raw[(size_t)y * stride] = 0;
        memcpy(raw + (size_t)y * stride + 1, rgb + (size_t)y * w * 3, (size_t)w * 3);
    }
    uLongf clen = compressBound(rawlen);
    uint8_t *comp = malloc(clen);
    if (!comp) die("out of memory");
    if (compress2(comp, &clen, raw, rawlen, 2) != Z_OK) die("zlib compress failed");
    png_chunk(f, "IDAT", comp, clen);
    png_chunk(f, "IEND", NULL, 0);
    free(raw); free(comp);
    fclose(f);
}
#endif /* !_WIN32（PNG 写出，zlib 依赖） */

/* ------------------------------------------------------------------ */
/* 解码主流程                                                           */
/* ------------------------------------------------------------------ */

typedef struct {
    int w, h;                /* 全分辨率有效区尺寸 */
    float *bmt;              /* 全分辨率 [R,G,B] interleaved，黑电平已扣除 */
    double black[3];
    double maxraw[3];
    double wbgain[3];
    double cc[9];
    int has_cc, has_wb;
    int orientation;         /* 内嵌 JPEG EXIF IFD0 Orientation (1-8) */
    char camera[64];
    /* scene-linear 组矩阵用的附加标定 */
    double iso_scale;        /* CaptureISO / SensorISO，缺省 1 */
    double dig_gain[3];      /* DigitalISOGain，缺省 [1,1,1] */
    char   wb_name[32];      /* as-shot 预设名（"Auto"/"Sunlight"/...） */
} Decoded;

/* 基线 TIFF 写出：RGB 16-bit、未压缩、单 strip、小端（II\42）。
 * IFD 在头部之后：8 + 2 + 11*12 + 4 = 146 → BitsPerSample@146，像素@152。 */
static void write_tiff_rgb16(const char *path, int w, int h, const uint16_t *rgb)
{
    FILE *f = fopen(path, "wb");
    if (!f) die("cannot open output file");
    uint32_t nbytes = (uint32_t)w * h * 6;
    uint8_t hdr[8] = { 'I','I', 42, 0, 8,0,0,0 };
    fwrite(hdr, 1, 8, f);

    struct TiffTag { uint16_t tag, type; uint32_t cnt, val; };  /* 12B, LE 主机 */
    struct TiffTag tags[11] = {
        { 256, 4, 1, (uint32_t)w },          /* ImageWidth */
        { 257, 4, 1, (uint32_t)h },          /* ImageLength */
        { 258, 3, 3, 146 },                  /* BitsPerSample -> 外置 */
        { 259, 3, 1, 1 },                    /* Compression = none */
        { 262, 3, 1, 2 },                    /* Photometric = RGB */
        { 273, 4, 1, 152 },                  /* StripOffsets */
        { 277, 3, 1, 3 },                    /* SamplesPerPixel */
        { 278, 4, 1, (uint32_t)h },          /* RowsPerStrip */
        { 279, 4, 1, nbytes },               /* StripByteCounts */
        { 284, 3, 1, 1 },                    /* PlanarConfig = chunky */
        { 296, 3, 1, 2 },                    /* ResolutionUnit = inch */
    };
    uint16_t ntags = 11;
    fwrite(&ntags, 2, 1, f);
    for (int i = 0; i < ntags; i++) {
        /* SHORT 内联值放在 4 字节值域的低 2 字节（LE） */
        uint8_t e[12];
        uint16_t tag = tags[i].tag, typ = tags[i].type;
        uint32_t cnt = tags[i].cnt, val = tags[i].val;
        memcpy(e, &tag, 2); memcpy(e + 2, &typ, 2);
        memcpy(e + 4, &cnt, 4); memcpy(e + 8, &val, 4);
        fwrite(e, 1, 12, f);
    }
    uint32_t nextifd = 0;
    fwrite(&nextifd, 4, 1, f);
    uint16_t bps[3] = { 16, 16, 16 };
    fwrite(bps, 2, 3, f);                    /* 146..151，像素数据 152 起 */
    fwrite(rgb, 2, (size_t)w * h * 3, f);    /* 小端主机直接写 */
    fclose(f);
}

static int cmp_double(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

/* 解析内嵌 JPEG 的 EXIF IFD0 Orientation（Quattro 旋转依据）。
 * 直接在文件缓冲中扫描 "Exif\0\0" 签名（APP1 段），解析 TIFF IFD0。 */
static int jpeg_exif_orientation(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i + 14 < len; i++) {
        if (memcmp(buf + i, "Exif\0\0", 6)) continue;
        const uint8_t *t = buf + i + 6;
        if (t + 8 > buf + len) break;
        int big;
        if (!memcmp(t, "II", 2)) big = 0;
        else if (!memcmp(t, "MM", 2)) big = 1;
        else continue;
        const uint8_t *rd16 = t + 2;
        unsigned magic = big ? ((unsigned)rd16[0] << 8 | rd16[1])
                             : ((unsigned)rd16[1] << 8 | rd16[0]);
        if (magic != 42) continue;
        uint32_t ifd0 = big ? ((uint32_t)t[4] << 24 | t[5] << 16 | t[6] << 8 | t[7])
                            : ((uint32_t)t[7] << 24 | t[6] << 16 | t[5] << 8 | t[4]);
        const uint8_t *ifd = t + ifd0;
        if (ifd + 2 > buf + len) continue;
        unsigned n = big ? ((unsigned)ifd[0] << 8 | ifd[1])
                         : ((unsigned)ifd[1] << 8 | ifd[0]);
        for (unsigned k = 0; k < n && ifd + 2 + 12 * (k + 1) <= buf + len; k++) {
            const uint8_t *e = ifd + 2 + 12 * k;
            unsigned tag = big ? ((unsigned)e[0] << 8 | e[1])
                               : ((unsigned)e[1] << 8 | e[0]);
            if (tag != 0x0112) continue;         /* Orientation, type SHORT */
            unsigned v = big ? ((unsigned)e[8] << 8 | e[9])
                             : ((unsigned)e[9] << 8 | e[8]);
            return (v >= 1 && v <= 8) ? (int)v : 1;
        }
        return 1;   /* IFD0 存在但无 Orientation */
    }
    return 1;
}

typedef struct {
    const uint8_t *plane; size_t plen; HTree *tree;
    uint32_t rows, cols; uint16_t seed;
    uint16_t *out; uint32_t out_rows, out_cols;
    int out_ch, out_channels;
} PlaneJob;

static void *plane_job_fn(void *arg)
{
    PlaneJob *j = (PlaneJob *)arg;
    decode_true_plane(j->plane, j->plen, j->tree, j->rows, j->cols, j->seed,
                      j->out, j->out_rows, j->out_cols, j->out_ch, j->out_channels);
    return NULL;
}

static int decode_file_wb(const char *inpath, OwnDecoded *dec, const char *wb_override);

/* ==================================================================
 * sd Quattro H 机内 as-shot 渲染链
 *
 * 照上游开源 sagwaco/x3fuse-core v0.4.1 的
 *   color_modes.rs  (sensor / resolve_color_mode / render_matrix)
 *   process.rs      (sd_quattro_camera / sd_quattro_preprocess /
 *                    sd_quattro_gain / color_mode_base_calibration /
 *                    native_count_span / merrill_render_iso_gain)
 *   merrill.rs      (generate_quattro_tone_curve / RenderLuts)
 * 装配到 own 的 scene-linear 出口。
 *
 * 门控（与上游 sd_quattro_camera 的双判据一致）：
 *   CAMERAID == 41 且 RAW type_format == 0x00010027。
 * 其它机型（dp Quattro 0x23、sd Quattro 0x25、Merrill、Classic）
 * 一行都不进入，输出逐位不变。
 *
 * 上游默认渲染不经过这条链；机内 JPEG 走的是「原生计数域 + 机内
 * 色调曲线」的 as-shot 路径，本模块把它补上：
 *
 *   counts   = min(DigitalISOGain * (raw - RawOffset)
 *                  * 100 / SensorISO, 65535)
 *   rgb_cnt  = iso * CC * C * diag(WB_B) * diag(count_span) * s + offset
 *   linear   = tone_lut[rgb_cnt]
 *
 * 其中 s 是逐通道归一化传感器值 (raw-black)/(max-black)，tone_lut 是
 * Quattro 的 logistic + Bezier + inverse-sRGB 曲线（14bit 饱和，
 * 输出线性值）。渲染层随后照旧做 sRGB 编码，等价于上游
 * RenderLuts::render_indices 拆成「线性 LUT + 输出 gamma」两步。
 *
 * 诊断开关：X3FDEC_SDQH_NATIVE=0 关闭、X3FDEC_SDQH_MODE=<名字或id>
 * 指定机内色彩模式、X3FDEC_SDQH_DEBUG=1 打印装配参数。
 * ================================================================== */

#define SDQH_LUT_SIZE 65536

typedef struct {
    int    active;
    double matrix[9];   /* 归一化传感器值 -> 原生 RGB 计数 */
    double offset[3];   /* 计数域黑台阶 */
    float *tone;        /* SDQH_LUT_SIZE 项：原生计数 -> 线性 [0,1] */
    /* 上游 convert_row 在进色彩模式矩阵之前对 sat_ratio 的处理。
     * 见 process.rs:4000 recovery_conv_matrix / highlight.rs:100
     * compute_chroma_prior / process.rs:4591 reconstruct_highlights /
     * process.rs:4610-4652 matrix-pathology gate。 */
    double conv[9];       /* recovery_matrix = physical_matrix / saturation */
    double prior[3];      /* compute_chroma_prior 的中性方向 */
    double blend_low, restore_thresh, sat_factor;  /* get_highlight_params */
    double gate_thr, gate_width;                  /* 上游默认 0.20 / 0.30 */
    int    recovery_ok;
} SdqhChain;

static _Thread_local SdqhChain g_sdqh;

/* 渲染层（x3f_render.c）用它判断本次 load 是否走了 SDQH 机内链；
 * 未注册时那边恒为 0，Mac 侧不需要本文件也能编译。 */
extern void x3f_render_register_sdqh_query(int (*fn)(void));
static int own_sdqh_native_active(void);

/* CAMF 标量 / 向量 / 3x3 读取（矩阵条目的整数与浮点已统一存成 double） */
static double sdqh_num(const Camf *c, const char *name, double def)
{
    const CamfEntry *e = camf_find(c, name);
    if (!e) return def;
    if (e->type == CMbM && e->mdata && e->mrows * e->mcols >= 1)
        return e->mdata[0];
    if (e->type == CMbT && e->text) return strtod(e->text, NULL);
    return def;
}

static int sdqh_vec3(const Camf *c, const char *name, double out[3])
{
    const CamfEntry *e = camf_find(c, name);
    if (!e || e->type != CMbM || !e->mdata || e->mrows * e->mcols < 3) return 0;
    for (int i = 0; i < 3; i++) out[i] = e->mdata[i];
    return 1;
}

static int sdqh_mat9(const Camf *c, const char *name, double out[9])
{
    const CamfEntry *e = camf_find(c, name);
    if (!e || e->type != CMbM || !e->mdata || e->mrows * e->mcols < 9) return 0;
    for (int i = 0; i < 9; i++) out[i] = e->mdata[i];
    return 1;
}

typedef struct {
    double start, end, steep, steep2, breakpoint, scale;
} SdqhToneParams;

/* 上游 quattro_tone：Quattro 的内置机内影调常数，按模式枚举号选行 */
static SdqhToneParams sdqh_tone_params(int id)
{
    SdqhToneParams p;
    p.start = -1.17; p.end = 2.85; p.steep = 3.0;
    p.steep2 = 1.4;  p.breakpoint = 0.1; p.scale = 1.0;
    switch (id) {
    case 3:  p.start = -1.19; p.steep = 3.0;  p.steep2 = 1.55; p.breakpoint = 0.078; break;
    case 4:  p.start = -1.20; p.steep = 3.55; p.steep2 = 1.60; p.breakpoint = 0.070; break;
    case 5:  p.start = -1.19; p.steep = 2.80; p.steep2 = 1.56; p.breakpoint = 0.080; break;
    case 6:  p.start = -1.05; p.end = 2.65;
             p.steep = 3.10;  p.steep2 = 1.97; p.breakpoint = 0.030; break;
    case 12: p.start = -1.20; p.steep = 3.50; p.steep2 = 1.90; p.breakpoint = 0.070; break;
    default: break;
    }
    return p;
}

typedef struct {
    int    valid;
    int    tone_id, depth;
    double contrast;
    float *lut;
} SdqhToneCache;

static _Thread_local SdqhToneCache g_sdqh_tone_cache;

/* 生成 Quattro 机内色调曲线（merrill.rs::generate_quattro_tone_curve
 * + generate_color_mode_linear_tone_lut）。返回 65536 项线性 LUT。
 * 计算量约 1.4e7 次 fmaf，按 (tone_id, contrast, depth) 缓存。 */
static float *sdqh_tone_lut(int tone_id, double contrast, int depth)
{
    if (g_sdqh_tone_cache.valid &&
        g_sdqh_tone_cache.tone_id == tone_id &&
        g_sdqh_tone_cache.depth == depth &&
        g_sdqh_tone_cache.contrast == contrast)
        return g_sdqh_tone_cache.lut;

    SdqhToneParams p = sdqh_tone_params(tone_id);
    double saturation = (double)((1u << depth) - 1u);
    int n = (int)saturation;
    if (n < 4 || n > 16384 || p.start >= p.end || p.steep <= 0.0 ||
        p.steep2 <= 0.0 || p.scale <= 0.0 || !isfinite(contrast))
        return NULL;

    double cf = pow(2.0, 0.5 * (contrast < -2.0 ? -2.0 :
                                (contrast > 2.0 ? 2.0 : contrast)));
    double k1 = p.steep * cf, k2 = p.steep2 * cf;
    double gam = 1.0 / 2.2222222222222223;
    double step = 1.0 / saturation;
    float *curve = (float *)malloc((size_t)(n + 1) * sizeof(float));
    if (!curve) return NULL;

    /* 前段 k1、后段 k2，断点处加常数补偿保持连续 */
    double x = 0.0;
    int i = 0;
    for (;;) {
        double dom = pow(x, gam) * (p.end - p.start) + p.start;
        curve[i] = (float)(1.0 / (1.0 + exp(-k1 * dom)));
        x += step;
        if (x >= p.breakpoint || i >= n) break;
        i++;
    }
    {
        double at_break = pow(x, gam) * (p.end - p.start) + p.start;
        double corr = 1.0 / (1.0 + exp(-k1 * at_break))
                    - 1.0 / (1.0 + exp(-k2 * at_break));
        for (int j = i + 1; j <= n; j++) {
            double dom = pow(x, gam) * (p.end - p.start) + p.start;
            curve[j] = (float)(1.0 / (1.0 + exp(-k2 * dom)) + corr);
            x += step;
        }
    }

    /* CombineToneCurve：每四个计数取一个控制点，Bezier 平滑成 1/4 网格 */
    int quarter_n = n / 4;
    float controls[256], work[256];
    int cc = (int)(saturation / 50.0 * 0.25) + 1;
    if (cc > 250) cc = 250;
    for (int j = 0; j < cc; j++) {
        int ci = j * 200;
        controls[j] = curve[ci <= n ? ci : n];
    }
    controls[cc++] = curve[quarter_n * 4 <= n ? quarter_n * 4 : n];
    float *quarter = (float *)malloc((size_t)(quarter_n + 1) * sizeof(float));
    if (!quarter) { free(curve); return NULL; }
    float reciprocal = (float)(1.0 / (saturation * 0.25));
    for (int q = 0; q <= quarter_n; q++) {
        memcpy(work, controls, (size_t)cc * sizeof(float));
        float t = (float)q * reciprocal;
        float omt = 1.0f - t;
        for (int level = 1; level < cc; level++)
            for (int j = cc - 1; j >= level; j--)
                work[j] = fmaf(omt, work[j - 1], t * work[j]);
        quarter[q] = work[cc - 1];
    }
    curve[0] = quarter[0];
    for (int q = 0; q < quarter_n; q++) {
        float base = curve[q * 4];
        float delta = (quarter[q + 1] - quarter[q]) * 0.25f;
        curve[q * 4 + 1] = base + delta;
        curve[q * 4 + 2] = fmaf(2.0f, delta, base);
        curve[q * 4 + 3] = fmaf(3.0f, delta, base);
        curve[q * 4 + 4] = quarter[q + 1];
    }
    for (int j = quarter_n * 4 + 1; j <= n; j++) curve[j] = quarter[quarter_n];
    free(quarter);

    /* 归一 + inverse-sRGB：把机内显示曲线拆成「线性 LUT + 输出 gamma」 */
    float lo = curve[0];
    double range = (double)curve[n] - (double)lo;
    if (!(range > 0.0)) { free(curve); return NULL; }
    double normalization = saturation;   /* at_sat == 1（curve[n] 即饱和点） */
    for (int j = 0; j <= n; j++) {
        double v = ((double)curve[j] - (double)lo) / range;
        double linear = v < 0.04045 ? v / 12.92
                                    : pow((v + 0.055) / 1.055, 2.4);
        curve[j] = (float)(normalization * linear);
    }
    double scale = p.scale * 65535.0 / (double)curve[n];
    if (!isfinite(scale) || scale <= 0.0) { free(curve); return NULL; }

    float *lut = (float *)malloc((size_t)SDQH_LUT_SIZE * sizeof(float));
    if (!lut) { free(curve); return NULL; }
    for (int j = 0; j < n; j++) {
        double v = (double)curve[j] * scale / 65535.0;
        lut[j] = (float)(v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v));
    }
    float tail = lut[n - 1];
    for (int j = n; j < SDQH_LUT_SIZE; j++) lut[j] = tail;
    free(curve);

    free(g_sdqh_tone_cache.lut);
    g_sdqh_tone_cache.lut = lut;
    g_sdqh_tone_cache.tone_id = tone_id;
    g_sdqh_tone_cache.depth = depth;
    g_sdqh_tone_cache.contrast = contrast;
    g_sdqh_tone_cache.valid = 1;
    return lut;
}

static int sdqh_m3inv(const double a[9], double out[9])
{
    double d = a[0] * (a[4] * a[8] - a[5] * a[7])
             - a[1] * (a[3] * a[8] - a[5] * a[6])
             + a[2] * (a[3] * a[7] - a[4] * a[6]);
    if (!(fabs(d) > 1e-30)) return 0;
    out[0] =  (a[4] * a[8] - a[5] * a[7]) / d;
    out[1] = -(a[1] * a[8] - a[2] * a[7]) / d;
    out[2] =  (a[1] * a[5] - a[2] * a[4]) / d;
    out[3] = -(a[3] * a[8] - a[5] * a[6]) / d;
    out[4] =  (a[0] * a[8] - a[2] * a[6]) / d;
    out[5] = -(a[0] * a[5] - a[2] * a[3]) / d;
    out[6] =  (a[3] * a[7] - a[4] * a[6]) / d;
    out[7] = -(a[0] * a[7] - a[1] * a[6]) / d;
    out[8] =  (a[0] * a[4] - a[1] * a[3]) / d;
    return 1;
}

static void sdqh_m3mul(const double a[9], const double b[9], double out[9])
{
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            out[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c]
                           + a[r * 3 + 2] * b[6 + c];
}

/* 色彩模式枚举号 -> CAMF 属性表键（上游 ColorMode::enum_id / key） */
static const char *sdqh_mode_key(int id)
{
    static const char *keys[] = {
        "", "", "", "Standard", "Vivid", "Neutral", "Portrait",
        "Landscape", "FCBlue", "SunsetRed", "ForestGreen",
        "FCYellow", "Cinema", "TealAndOrange"
    };
    return (id >= 3 && id <= 13) ? keys[id] : "Standard";
}

static int sdqh_mode_id(void)
{
    const char *e = getenv("X3FDEC_SDQH_MODE");
    if (!e || !e[0]) return 3;                 /* as-shot 缺省 Standard */
    static const struct { const char *name; int id; } named[] = {
        { "Standard", 3 }, { "Vivid", 4 }, { "Neutral", 5 },
        { "Portrait", 6 }, { "Landscape", 7 }, { "FCBlue", 8 },
        { "SunsetRed", 9 }, { "ForestGreen", 10 }, { "FCYellow", 11 },
        { "Cinema", 12 }, { "TealAndOrange", 13 }
    };
    for (int i = 0; i < (int)(sizeof(named) / sizeof(named[0])); i++)
        if (!strcmp(e, named[i].name)) return named[i].id;
    int id = atoi(e);
    return (id >= 3 && id <= 13) ? id : 3;
}

/* 装配整条链。返回 1 = SDQH 且标定齐全（可以走机内链）。 */
static int sdqh_prepare(const Camf *camf, const OwnDecoded *dec,
                        const char *wb, uint32_t type_format)
{
    g_sdqh.active = 0;
    x3f_render_register_sdqh_query(own_sdqh_native_active);
    {
        const char *off = getenv("X3FDEC_SDQH_NATIVE");
        if (off && off[0] == '0') return 0;
    }
    if (type_format != 0x00010027u) return 0;
    if ((int)sdqh_num(camf, "CAMERAID", -1.0) != 41) return 0;

    /* sd_quattro_preprocess：只有 SDQH 读 DigitalISOGain */
    double raw_offset = sdqh_num(camf, "RawOffset", 256.0);
    double sensor_iso = sdqh_num(camf, "SensorISO", 0.0);
    if (sensor_iso == 0.0) sensor_iso = 100.0;
    double dig[3] = { 1.0, 1.0, 1.0 };
    (void)sdqh_vec3(camf, "DigitalISOGain", dig);
    double span[3], count_off[3];
    for (int c = 0; c < 3; c++) {
        double gain = dig[c] * (100.0 / sensor_iso);
        double top  = gain * (dec->maxraw[c] - raw_offset);
        /* sd_quattro_gain：超过机内 65535 计数夹断就无法用线性矩阵表达 */
        if (!(gain > 0.0) || !(top > 0.0) || top > 65535.0) return 0;
        span[c]      = gain * (dec->maxraw[c] - dec->black[c]);
        count_off[c] = gain * (dec->black[c] - raw_offset);
    }

    /* color_mode_base_calibration 的 SDQ 分支：WB_B 只吃 TempGainFact、
     * 绿通道归一，不乘 SensorAdjustmentGainFact / FNumberGainFact。
     * Fluorescent 档的 FnumberGainTable 插值量在 1% 量级，且 SDQH 的
     * 内嵌缩略图不带 EXIF FNumber，这里按上游「表缺失即 unity」处理。 */
    double wbraw[3], temp[3] = { 1.0, 1.0, 1.0 };
    const CamfEntry *g = camf_matrix_for_wb(camf, "WhiteBalanceGains", wb, "WBGain");
    if (!g || g->type != CMbM || !g->mdata || g->mrows * g->mcols < 3) return 0;
    for (int i = 0; i < 3; i++) wbraw[i] = g->mdata[i];
    if (sdqh_vec3(camf, "TempGainFact", temp)) {
        int ok = 1;
        for (int i = 0; i < 3; i++)
            if (!isfinite(temp[i]) || temp[i] < 0.5 || temp[i] > 1.5) ok = 0;
        if (!ok) { temp[0] = temp[1] = temp[2] = 1.0; }
    }
    double wbb[3];
    for (int i = 0; i < 3; i++) {
        if (!isfinite(wbraw[i]) || !(wbraw[i] > 0.0)) return 0;
        wbb[i] = temp[i] * wbraw[i];
    }
    if (!(wbb[1] > 0.0)) return 0;
    {
        double green = wbb[1];      /* 先用未改动的绿通道，避免就地除法串改 */
        for (int i = 0; i < 3; i++) wbb[i] /= green;
    }

    /* merrill_render_iso_gain(capture, 100, 0, ISOrange) */
    double capture = sdqh_num(camf, "CaptureISO", 0.0);
    unsigned iso_range = (unsigned)sdqh_num(camf, "ISOrange", 0.0);
    float capf = (float)capture;
    if (!(capf > 0.0f)) return 0;
    if (iso_range != 0) {
        if (capf == 800.0f) capf = 666.0f;
        else if (capf == 1600.0f) capf = 1200.0f;
        else if (capf == 3200.0f) capf = 2133.0f;
        else if (capf == 6400.0f) capf = 4266.0f;
    }
    double iso_gain = (double)(capf / 100.0f);
    if (!isfinite(iso_gain) || !(iso_gain > 0.0)) return 0;

    int mode_id = sdqh_mode_id();
    const char *mkey = sdqh_mode_key(mode_id);

    /* ColorModeCompensations -> 3x3 创意矩阵（Standard 通常是单位阵） */
    double cc[9];
    for (int i = 0; i < 9; i++) cc[i] = dec->cc[i] * iso_gain;
    {
        const char *cmname = NULL;
        double cm[9];
        if (camf_prop(camf, "ColorModeCompensations", mkey, &cmname) &&
            cmname[0] && sdqh_mat9(camf, cmname, cm) && cm[0] != 0.0) {
            double prod[9];
            sdqh_m3mul(cc, cm, prod);
            memcpy(cc, prod, sizeof(prod));
        }
    }

    /* ColorModeContrastCompensations -> 对比度标量 */
    double contrast = 0.0;
    {
        const char *cname = NULL;
        if (camf_prop(camf, "ColorModeContrastCompensations", mkey, &cname) &&
            cname[0])
            contrast = sdqh_num(camf, cname, 0.0);
    }

    /* quattro_tone：TCColorModeSettings 行数不足时退回内置默认行 */
    const CamfEntry *tcs = camf_find(camf, "TCColorModeSettings");
    int rows = (tcs && tcs->type == CMbM) ? tcs->mrows : 0;
    int tone_id = (rows > 0 && rows <= mode_id) ? 0 : mode_id;

    int depth = (int)sdqh_num(camf, "ImageDepth", 14.0);
    if (depth < 1 || depth > 14) depth = 14;
    float *lut = sdqh_tone_lut(tone_id, contrast, depth);
    if (!lut) return 0;

    /* ColorModeCalibration::render_matrix */
    for (int i = 0; i < 9; i++)
        g_sdqh.matrix[i] = cc[i] * wbb[i % 3] * span[i % 3];
    for (int r = 0; r < 3; r++) {
        double v = 0.0;
        for (int c = 0; c < 3; c++) v += cc[r * 3 + c] * wbb[c] * count_off[c];
        g_sdqh.offset[r] = v;
    }
    /* 照抄上游 QuattroColorModeRender::new 的 recovery_matrix：
     * 不含创意矩阵的物理标定矩阵，再除以 params.pixel_saturation。
     * compute_chroma_prior = normalize(inverse(conv) * [1,1,1])。 */
    {
        double sat = (double)((1u << depth) - 1u);
        for (int i = 0; i < 9; i++)
            g_sdqh.conv[i] = dec->cc[i] * iso_gain * wbb[i % 3]
                           * span[i % 3] / sat;
        double inv[9], dir[3];
        g_sdqh.recovery_ok = sdqh_m3inv(g_sdqh.conv, inv);
        if (g_sdqh.recovery_ok) {
            double mx = 0.0;
            for (int r = 0; r < 3; r++) {
                dir[r] = inv[r * 3] + inv[r * 3 + 1] + inv[r * 3 + 2];
                if (dir[r] > mx) mx = dir[r];
            }
            if (mx <= 1e-12) {
                g_sdqh.prior[0] = g_sdqh.prior[1] = g_sdqh.prior[2] = 1.0;
            } else {
                for (int c = 0; c < 3; c++) g_sdqh.prior[c] = dir[c] / mx;
            }
        }
    }
    /* highlight.rs:56 get_highlight_params 的 CAMF 默认值；
     * process.rs:4849 的 matrix-pathology gate 默认常数。 */
    g_sdqh.blend_low      = sdqh_num(camf, "HighlightBlendingLow", 0.75);
    g_sdqh.restore_thresh = sdqh_num(camf, "HighlightRestoreThresh", 1.75);
    g_sdqh.sat_factor     = sdqh_num(camf, "HighlightSatFactor", 1.0);
    g_sdqh.gate_thr       = 0.20;
    g_sdqh.gate_width     = 0.30;

    g_sdqh.tone = lut;
    g_sdqh.active = 1;

    if (getenv("X3FDEC_SDQH_DEBUG"))
        fprintf(stderr,
                "[sdqh] as-shot chain: mode=%s(%d) tone_id=%d depth=%d "
                "contrast=%.3f iso=%.3f wbb=[%.4f %.4f %.4f] "
                "span=[%.1f %.1f %.1f] off=[%.2f %.2f %.2f]\n",
                mkey, mode_id, tone_id, depth, contrast, iso_gain,
                wbb[0], wbb[1], wbb[2],
                span[0], span[1], span[2],
                g_sdqh.offset[0], g_sdqh.offset[1], g_sdqh.offset[2]);
    return 1;
}

/* scene-linear 像素循环：归一化传感器值 -> 计数域 -> 机内 tone LUT。
 * 输出仍是线性 RGB，由渲染层照旧做 sRGB 编码。 */
typedef struct {
    const float *bmt; float *out; int w, y0, y1;
    const double *span; const SdqhChain *chain;
} SdqhJob;

static void *sdqh_pixel_job(void *arg)
{
    SdqhJob *j = (SdqhJob *)arg;
    const SdqhChain *ch = j->chain;
    /* 渲染层对 Quattro 有产品级冷调 R*0.96（x3f_render.c）。机内链已经
     * 自带白平衡，这里预乘倒数抵消，避免再被压一次；X3F_COOL_R 覆盖时
     * 同步跟随，保持「只影响非 SDQH」的既有行为不变。 */
    double cool = 0.96;
    const char *ce = getenv("X3F_COOL_R");
    if (ce && ce[0]) { double v = atof(ce); if (v > 0.5 && v < 1.5) cool = v; }
    double inv_cool = (cool != 1.0 && cool > 0.0) ? 1.0 / cool : 1.0;

    for (int y = j->y0; y < j->y1; y++) {
        size_t i = (size_t)y * j->w;
        for (int x = 0; x < j->w; x++, i++) {
            double s[3];
            s[0] = j->bmt[i * 3]     / j->span[0];
            s[1] = j->bmt[i * 3 + 1] / j->span[1];
            s[2] = j->bmt[i * 3 + 2] / j->span[2];

            /* 照抄上游 convert_row 的 sat_ratio 处理，顺序一致：
             * reconstruct_highlights → matrix-pathology gate → 色彩模式矩阵。
             * 上游在 4669-4671 把这份 input 交给 QuattroColorModeRender::render。 */
            if (ch->recovery_ok) {
                double s_max = 0.0, u_max = 0.0;
                for (int c = 0; c < 3; c++) {
                    double pc = ch->prior[c] > 1e-12 ? ch->prior[c] : 1e-12;
                    double u  = s[c] / pc;
                    if (s[c] > s_max) s_max = s[c];
                    if (u > u_max) u_max = u;
                }
                if (s_max > ch->blend_low) {
                    double big_l = u_max * ch->sat_factor;
                    if (u_max >= ch->restore_thresh) {
                        for (int c = 0; c < 3; c++) s[c] = big_l * ch->prior[c];
                    } else {
                        double tt = (s_max - ch->blend_low)
                                  / (1.0 - ch->blend_low);
                        if (tt < 0.0) tt = 0.0;
                        else if (tt > 1.0) tt = 1.0;
                        for (int c = 0; c < 3; c++)
                            s[c] = (1.0 - tt) * s[c]
                                 + tt * (big_l * ch->prior[c]);
                    }
                }
                double mp[3];
                for (int r = 0; r < 3; r++)
                    mp[r] = ch->conv[r * 3]     * s[0]
                          + ch->conv[r * 3 + 1] * s[1]
                          + ch->conv[r * 3 + 2] * s[2];
                double mg = mp[1] - (mp[0] > mp[2] ? mp[0] : mp[2]);
                double my = (mp[0] < mp[1] ? mp[0] : mp[1]) - mp[2];
                double margin = mg > my ? mg : my;
                double strength = (margin - ch->gate_thr) / ch->gate_width;
                if (strength > 0.0) {
                    if (strength > 1.0) strength = 1.0;
                    double um = 0.0;
                    for (int c = 0; c < 3; c++) {
                        double pc = ch->prior[c] > 1e-12 ? ch->prior[c] : 1e-12;
                        double u  = s[c] / pc;
                        if (u > um) um = u;
                    }
                    for (int c = 0; c < 3; c++)
                        s[c] = (1.0 - strength) * s[c]
                             + strength * (um * ch->prior[c]);
                }
            }
            for (int c = 0; c < 3; c++) {
                double cnt = ch->matrix[c * 3]     * s[0]
                           + ch->matrix[c * 3 + 1] * s[1]
                           + ch->matrix[c * 3 + 2] * s[2]
                           + ch->offset[c];
                float fc = (float)cnt;               /* 上游按 f32 取索引 */
                if (fc < 0.0f) fc = 0.0f;
                else if (fc > 65535.0f) fc = 65535.0f;
                int idx = (int)(fc + 0.5f);
                if (idx > 65535) idx = 65535;
                double v = (double)ch->tone[idx];
                if (c == 0) v *= inv_cool;
                j->out[i * 3 + c] = (float)v;
            }
        }
    }
    return NULL;
}


/* ------------------------------------------------------------------ */
/* CAMF 工厂坏点修复（对齐 x3f_tools / x3fuse 的 interpolate_bad_pixels）  */
/*                                                                    */
/* 黑点根因（2026-10-21 定案）：F23 机型的 CAMF 里带工厂坏点表          */
/*   BadPixelsLumaF23   → 顶层平面（colors=1）                         */
/*   BadPixelsChromaF23 → 半层平面（colors=3）                         */
/* 坐标是精确元数据（实测呈 16 列块同相位的周期结构、垂直成对 —— 读出    */
/* 块硬件缺陷，非随机坏点）。x3fuse 解码后按表做四邻域均值多轮插值，     */
/* 自研核心此前缺这套系统 → 表内位置原样透出 = 均匀分布黑点/色点。       */
/* 另有 HighlightPixelsInfo 周期网格（HLP 高光专用像素）与 sd Quattro   */
/* 系的 CAMERAID AF 网格，一并按 x3fuse 语义标记。X3FDEC_NOBADPIX 关。  */
/* ------------------------------------------------------------------ */

typedef struct { int c, r; } BadPix;

static int bp_test(const uint32_t *vec, int c, int r, int w, int h)
{
    if (c < 0 || c >= w || r < 0 || r >= h) return 1;
    long p = (long)r * w + c;
    return (int)((vec[p >> 5] >> (p & 31)) & 1u);
}

static void bp_mark(uint32_t *vec, BadPix *arr, int *n, long cap,
                    int c, int r, int w, int h)
{
    if (c < 0 || c >= w || r < 0 || r >= h) return;
    long p = (long)r * w + c;
    if ((vec[p >> 5] >> (p & 31)) & 1u) return;
    vec[p >> 5] |= 1u << (p & 31);
    if (*n < cap) { arr[*n].c = c; arr[*n].r = r; (*n)++; }
}

static void camf_fix_bad_pixels(const Camf *camf, uint16_t *data,
                                int w, int h, int ch)
{
    const int colors = ch;   /* 1=顶层 luma，3=半层 chroma（legacy 语义） */

    long cap = 1 << 22;
    uint32_t *vec = calloc((size_t)(((long)w * h + 63) / 64) * 2 + 2, 4);
    BadPix *arr = malloc((size_t)cap * sizeof(BadPix));
    BadPix *still = malloc((size_t)cap * sizeof(BadPix));
    if (!vec || !arr || !still) die("out of memory");
    int n = 0;

    /* --- F23 打包坏点表：[row, col, (跳过), col, (跳过), ..., 0]* --- */
    const CamfEntry *e = camf_find(camf, colors == 1 ? "BadPixelsLumaF23"
                                                     : "BadPixelsChromaF23");
    if (e && e->type == CMbM && e->mdata) {
        long len = (long)e->mrows * e->mcols;
        int row = -1;
        for (long i = 0; i < len; i++) {
            uint32_t v = (uint32_t)e->mdata[i];
            if (row == -1) row = (int)v;
            else if (v == 0) row = -1;
            else {
                bp_mark(vec, arr, &n, cap, (int)v, row, w, h);
                i++;            /* 与 C 版 i++ quirk 一致：跳过下一条目 */
            }
        }
    }

    /* --- HighlightPixelsInfo 周期网格（仅 colors==3，legacy 语义） --- */
    if (colors == 3) {
        const CamfEntry *hp = camf_find(camf, "HighlightPixelsInfo");
        if (hp && hp->type == CMbM && hp->mrows * hp->mcols >= 4) {
            int c0 = (int)hp->mdata[0], r0 = (int)hp->mdata[1];
            int cst = (int)hp->mdata[2], rst = (int)hp->mdata[3];
            if (cst > 0 && rst > 0)
                for (int r = r0; r < h; r += rst)
                    for (int c = c0; c < w; c += cst)
                        bp_mark(vec, arr, &n, cap, c, r, w, h);
        }
    }

    /* --- sd Quattro / sd Quattro H 的 AF 像素网格（位置硬编码，legacy 同） --- */
    {
        const CamfEntry *cid = camf_find(camf, "CAMERAID");
        uint32_t cameraid = 0xffffffffu;
        if (cid && cid->type == CMbM && cid->mrows * cid->mcols >= 1)
            cameraid = (uint32_t)cid->mdata[0];
        else if (cid && cid->type == CMbT && cid->text)
            cameraid = (uint32_t)strtoul(cid->text, NULL, 10);
        /* {ci, cf, cp, cs, ri, rf, rp, rs} —— legacy x3f_process.c 原值 */
        static const int sdq_luma[8]   = { 217, 5641, 16, 1,  464, 3312, 32, 2 };
        static const int sdq_chroma[8] = { 108, 2820,  8, 1,  232, 1656, 16, 1 };
        static const int sdqh_luma[8]  = { 233, 6425, 16, 1,  592, 3888, 32, 2 };
        static const int sdqh_chroma[8]= { 116, 2820,  8, 1,  296, 1944, 16, 1 };
        const int *g = NULL;
        if (cameraid == 40) g = colors == 1 ? sdq_luma : sdq_chroma;       /* SDQ  */
        else if (cameraid == 41) g = colors == 1 ? sdqh_luma : sdqh_chroma;/* SDQH */
        if (g)
            for (int r = g[4]; r <= g[5]; r += g[6])
                for (int c = g[0]; c <= g[1]; c += g[2])
                    for (int rr = 0; rr < g[7]; rr++)
                        for (int cc = 0; cc < g[3]; cc++)
                            bp_mark(vec, arr, &n, cap, c + cc, r + rr, w, h);
    }

    if (getenv("X3FDEC_BADPIXDBG"))
        fprintf(stderr, "[badpix] %s: marked %d (%dx%d ch%d)\n",
                colors == 1 ? "luma" : "chroma", n, w, h, ch);
    if (n == 0) { free(vec); free(arr); free(still); return; }

    /* --- 多轮四邻域均值插值（与 legacy/x3fuse 逐语义一致：
     * 邻域判定看本轮开始时的位图，fixed 的标记在轮末统一清） --- */
    int fix_corner = 0;
    int *fidx = malloc((size_t)cap * sizeof(int));
    if (!fidx) die("out of memory");
    while (n > 0) {
        int ns = 0, nf = 0;
        for (int i = 0; i < n; i++) {
            int c = arr[i].c, r = arr[i].r;
            const uint16_t *inp[4] = { NULL, NULL, NULL, NULL };
            int num = 0;
            if (!bp_test(vec, c - 1, r, w, h))
                inp[num++] = data + ((long)r * w + (c - 1)) * ch;
            if (!bp_test(vec, c + 1, r, w, h))
                inp[num++] = data + ((long)r * w + (c + 1)) * ch;
            if (!bp_test(vec, c, r - 1, w, h))
                inp[num++] = data + ((long)(r - 1) * w + c) * ch;
            if (!bp_test(vec, c, r + 1, w, h))
                inp[num++] = data + ((long)(r + 1) * w + c) * ch;

            const uint16_t *use[4]; int unum = 0;
            if (num == 4) {
                memcpy(use, inp, sizeof use); unum = 4;
            } else if (inp[0] && inp[1]) {
                use[0] = inp[0]; use[1] = inp[1]; unum = 2;
            } else if (inp[2] && inp[3]) {
                use[0] = inp[2]; use[1] = inp[3]; unum = 2;
            } else if (fix_corner && num == 2) {
                memcpy(use, inp, sizeof use); unum = 2;
            } else {
                still[ns++] = arr[i];
                continue;
            }
            uint16_t *outp = data + ((long)r * w + c) * ch;
            for (int k = 0; k < ch; k++) {
                uint32_t sum = 0;
                for (int u = 0; u < unum; u++) sum += use[u][k];
                outp[k] = (uint16_t)((sum + (uint32_t)unum / 2) / (uint32_t)unum);
            }
            fidx[nf++] = i;
        }
        for (int i = 0; i < nf; i++) {
            long p = (long)arr[fidx[i]].r * w + arr[fidx[i]].c;
            vec[p >> 5] &= ~(1u << (p & 31));
        }
        memcpy(arr, still, (size_t)ns * sizeof(BadPix));
        n = ns;
        if (nf == 0) {
            if (!fix_corner) fix_corner = 1;
            else break;         /* 剩余保持原值（legacy 的放弃路径） */
        }
    }

    free(fidx);
    free(vec); free(arr); free(still);
}

int own_decode_file(const char *path, OwnDecoded *dec)
{
    return decode_file_wb(path, dec, NULL);
}

static int decode_file_wb(const char *inpath, OwnDecoded *dec, const char *wb_override)
{
    stage_t("0: decode entry");
    dec->clipmask = NULL;       /* 本函数只在 Quattro 融合分支里分配 */
    X3F x; x3f_parse(&x, inpath);

    DirEntry *raw_de = NULL, *camf_de = NULL;
    for (int i = 0; i < x.nent; i++) {
        const uint8_t *sh = x.data + x.ent[i].offset;
        if (!memcmp(sh, "SECi", 4)) {
            uint32_t tf = (x.ent[i].sec_type << 16) | x.ent[i].sec_format;
            if (tf == 0x00010023 || tf == 0x00010025 || tf == 0x00010027) {
                if (!raw_de || x.ent[i].size > raw_de->size) raw_de = &x.ent[i];
            }
        } else if (!memcmp(sh, "SECc", 4)) {
            if (x.ent[i].camf_type == 5 && (!camf_de || x.ent[i].size > camf_de->size))
                camf_de = &x.ent[i];
        }
    }
    if (!raw_de) die_unsupported("no Quattro RAW section found");
    if (!camf_de) die("no CAMF section found");

    fprintf(stderr, "x3fdec: version %u.%u, %d sections\n",
            x.version >> 16, x.version & 0xffff, x.nent);

    dec->orientation = jpeg_exif_orientation(x.data, x.len);
    fprintf(stderr, "x3fdec: exif orientation = %d\n", dec->orientation);

    /* --- CAMF --- */
    Camf camf; memset(&camf, 0, sizeof(camf));
    {
        const uint8_t *sh = x.data + camf_de->offset;
        /* SECc 头 28 字节（id/ver/type/val0..3），数据从其后开始 */
        camf_decode_type5(sh + 28, camf_de->size - 28,
                          camf_de->val0, camf_de->val1, &camf);
        fprintf(stderr, "x3fdec: CAMF decoded %zu bytes\n", camf.decoded_size);
        camf_parse_entries(&camf);
        fprintf(stderr, "x3fdec: CAMF %d entries\n", camf.nentries);
    }

    /* --- RAW TRUE 头 --- */
    const uint8_t *sec = x.data + raw_de->offset;
    const uint8_t *p = sec + 28;
    size_t left = raw_de->size - 28;

    uint16_t pw[3], ph[3];
    for (int i = 0; i < 3; i++) {
        pw[i] = rd_u16(p + 4 * i);
        ph[i] = rd_u16(p + 4 * i + 2);
    }
    p += 12; left -= 12;

    int layout;
    if (ph[0] * 2 == raw_de->rows) layout = 1;
    else if (ph[0] == raw_de->rows) layout = 0;
    else die("unknown Quattro layer size");

    uint16_t seed[3];
    for (int i = 0; i < 3; i++) seed[i] = rd_u16(p + 2 * i);
    p += 8; left -= 8;      /* 3 seeds + unknown u16 */

    HTree tree; htree_init(&tree, 32);
    int ti = 0;
    for (;;) {
        if (left < 2) die("unterminated TRUE table");
        uint8_t cs = p[0], cc = p[1];
        p += 2; left -= 2;
        if (cs == 0) break;
        if (++ti > 31) die("unterminated TRUE table");
        htree_insert(&tree, cs, cc >> (8 - cs), ti - 1);
    }
    if (left < 4) die("missing plane sizes");
    p += 4; left -= 4;      /* unknown u32 */

    uint32_t psize[3];
    for (int i = 0; i < 3; i++) {
        psize[i] = rd_u32(p + 4 * i);
        if (psize[i] == 0) die("empty plane");
    }
    p += 12; left -= 12;
    if (left < (size_t)psize[0] + psize[1] + psize[2]) die("planes exceed section");

    fprintf(stderr, "x3fdec: planes %ux%u (layout %d), sizes %u/%u/%u\n",
            pw[2], ph[2], layout, psize[0], psize[1], psize[2]);

    /* --- 解码三平面 --- */
    /* 颜色 0,1 -> 半分辨率 R,G（通道 0,1）；颜色 2 -> 全分辨率 B */
    uint16_t *half = calloc((size_t)ph[0] * pw[0] * 3, sizeof(uint16_t));
    uint16_t *top  = calloc((size_t)ph[2] * pw[2], sizeof(uint16_t));
    if (!half || !top) die("out of memory");

    {
        PlaneJob jobs[3];
        size_t off = 0;
        for (int color = 0; color < 3; color++) {
            jobs[color].plane = p + off; jobs[color].plen = psize[color];
            jobs[color].tree = &tree;
            if (color < 2) {
                jobs[color].rows = ph[color]; jobs[color].cols = pw[color];
                jobs[color].seed = seed[color];
                jobs[color].out = half; jobs[color].out_rows = ph[0];
                jobs[color].out_cols = pw[0];
                jobs[color].out_ch = color; jobs[color].out_channels = 3;
            } else {
                jobs[color].rows = ph[2]; jobs[color].cols = pw[2];
                jobs[color].seed = seed[2];
                jobs[color].out = top; jobs[color].out_rows = ph[2];
                jobs[color].out_cols = pw[2];
                jobs[color].out_ch = 0; jobs[color].out_channels = 1;
            }
            off += (psize[color] + 15u) & ~15u;
        }
        /* 三平面独立位流, 并行解码 */
        run3(plane_job_fn, &jobs[0], &jobs[1], &jobs[2]);
        free(tree.nodes); free(tree.hlen); free(tree.hval);
    stage_t("entropy decode 3 planes");
    }

    /* 诊断：原始顶层（任何处理之前）dump（X3F_OWN_LAGDUMP） */
    own_lag_parse();
    if (getenv("X3F_OWN_LAGDUMP"))
        own_dump_lag_u16("lag_top_raw.txt", top, (int)pw[2], (int)ph[2]);

    /* --- CAMF 工厂坏点修复（元数据精确标记，见 camf_fix_bad_pixels 注释）。
     * 顶层用 BadPixelsLumaF23，半层用 BadPixelsChromaF23。
     * 必须在启发式点修复之前：先把表内已知位置修掉，
     * 启发式只兜底表外漏网的。X3FDEC_NOBADPIX 关。 --- */
    if (!getenv("X3FDEC_NOBADPIX")) {
        camf_fix_bad_pixels(&camf, half, (int)pw[0], (int)ph[0], 3);
        camf_fix_bad_pixels(&camf, top, (int)pw[2], (int)ph[2], 1);
    }

    /* --- 孤立点状坏点修复：仅 top 平面在此处做。
     * half 平面【不能】在这里做：半层键合缺陷列（x≡0 mod 16，每列大量
     * 像素亏损）尚未修复，点修复会把整列像素逐个"修"一遍（实测 6.4 万
     * 次/通道），每个修复点经 2x 上采样留下 2×2 平滑印记 → 沿缺陷列的
     * 垂直点状竖纹。half 的点缺陷在 fuse_quattro 内、列修复之后处理
     * （那里只剩真正的孤立点）。X3FDEC_NOPOINTFIX 关闭。 --- */
    if (!getenv("X3FDEC_NOPOINTFIX")) {
        repair_point_defects_u16(top, 1, 0, pw[2], ph[2], 450.0, 600.0, 300.0);
    }

    /* --- 黑电平（需在释放 half 前估计） --- */
    {
        /* DarkShieldTop [x1,y1,x2,y2]（KeepImageArea 坐标，闭区间） */
        int x1 = 0, y1 = 0, x2 = (int)pw[0] - 1, y2 = 3;   /* 回退: 顶部4行 */
        int fx1 = 0, fy1 = 0, fx2 = (int)pw[2] - 1, fy2 = 7;
        const CamfEntry *ds = camf_find(&camf, "DarkShieldTop");
        if (ds && ds->mrows * ds->mcols >= 4) {
            int rx1 = (int)ds->mdata[0], ry1 = (int)ds->mdata[1];
            int rx2 = (int)ds->mdata[2], ry2 = (int)ds->mdata[3];
            if (rx2 >= rx1 && ry2 >= ry1) {
                x1 = rx1 / 2; y1 = ry1 / 2;
                x2 = rx2 / 2; y2 = ry2 / 2;
                fx1 = rx1; fy1 = ry1; fx2 = rx2; fy2 = ry2;
            }
        }
        if (x2 >= pw[0]) x2 = pw[0] - 1;
        if (y2 >= ph[0]) y2 = ph[0] - 1;
        if (fx2 >= pw[2]) fx2 = pw[2] - 1;
        if (fy2 >= ph[2]) fy2 = ph[2] - 1;

        for (int c = 0; c < 3; c++) {
            const uint16_t *plane = (c < 2) ? half : top;
            int pw__ = (c < 2) ? pw[0] : pw[2];
            /* 采样矩形内每通道值，取低于均值的样本均值（抗离群） */
            double sum = 0;
            long n = (long)(x2 - x1 + 1) * (y2 - y1 + 1);
            long fn = (long)(fx2 - fx1 + 1) * (fy2 - fy1 + 1);
            const int ax1 = c < 2 ? x1 : fx1, ay1 = c < 2 ? y1 : fy1;
            const int ax2 = c < 2 ? x2 : fx2, ay2 = c < 2 ? y2 : fy2;
            const long an = c < 2 ? n : fn;
            double *v = malloc((size_t)an * sizeof(double));
            if (!v) die("out of memory");
            long k = 0;
            for (int y = ay1; y <= ay2; y++)
                for (int x = ax1; x <= ax2; x++) {
                    double val = plane[((size_t)y * pw__ + x) * (c < 2 ? 3 : 1) + (c < 2 ? c : 0)];
                    sum += val;
                    v[k++] = val;
                }
            qsort(v, (size_t)an, sizeof(double), cmp_double);
            double mean = sum / an;
            double lsum = 0; long ln = 0;
            for (long i = 0; i < an; i++)
                if (v[i] <= mean) { lsum += v[i]; ln++; }
            /* 【2026-10-24 暗部粉纱修复】"低于均值样本均值" = E[X|X<μ]
             * = μ − 0.798σ，对高斯读噪**天然偏低 0.8σ**（σ≈8-11 → 差 6-9 码）。
             * 黑电平低估 → 三平面带正基座 → 暗部（信号小、WB 增益与 conv
             * 大系数相对放大）出粉/品红纱幕（DP2Q0337 实证；black=2048
             * 强制后目检消除）。改用中位数：同样抗离群、对对称噪声无偏。
             * X3FDEC_BLACK_LEGACY=1 回退旧口径对照。 */
            if (getenv("X3FDEC_BLACK_LEGACY")) {
                dec->black[c] = ln ? lsum / ln : mean;
            } else {
                long mid = an / 2;
                dec->black[c] = (an & 1) ? v[mid]
                                : 0.5 * (v[mid - 1] + v[mid]);
            }
            free(v);
        }
        /* 【2026-10-24 b：默认黑电平口径 = x3fuse get_black_level 复刻】
         * DarkShieldTop + DarkShieldBottom（CAMMODEL=="SIGMA DP2" 时跳过，
         * DP2 固件 bug，x3fuse 同款 workaround）+ DarkShieldColRange 左右
         * 屏蔽列，四区域**合并普通均值**。半层取 ch0/ch1（矩形 ÷2），
         * 顶层取 ch0（原坐标）。
         * 实证（2026-10-24）：2123 基准 ISO 下与 x3fuse 残差 <0.7 显示码；
         * 0337（ISO320 扩展）仍有全局色相差（R/G +2.5%，conv 链溯源待查）。
         * 回退开关：X3FDEC_BLACK_MEDIAN=1 → 中位数；X3FDEC_BLACK_LEGACY=1
         * → 旧"低于均值样本均值"（偏低 0.8σ，暗部粉纱根源，勿用）。 */
        if (!getenv("X3FDEC_BLACK_MEDIAN") && !getenv("X3FDEC_BLACK_LEGACY")) {
            int use_bottom = 0;
            int bx1 = 0, by1 = 0, bx2 = -1, by2 = -1;
            const CamfEntry *dsb = camf_find(&camf, "DarkShieldBottom");
            if (dsb && dsb->mrows * dsb->mcols >= 4) {
                int rx1 = (int)dsb->mdata[0], ry1 = (int)dsb->mdata[1];
                int rx2 = (int)dsb->mdata[2], ry2 = (int)dsb->mdata[3];
                if (rx2 >= rx1 && ry2 >= ry1) {
                    bx1 = rx1; by1 = ry1; bx2 = rx2; by2 = ry2;
                    use_bottom = 1;
                }
            }
            {
                const CamfEntry *cm = camf_find(&camf, "CAMMODEL");
                if (cm && cm->text && strcmp(cm->text, "SIGMA DP2") == 0)
                    use_bottom = 0;
            }
            /* sd Quattro H（CAMERAID=41）固件把 DarkShieldBottom 矩形写错，
             * 该区域压到了有效成像区。把它并进黑电平会把三平面黑电平整体
             * 抬高约 60/60/108 码（SDQH4616 实测 [318.9 315.4 366.0]，
             * x3fuse 为 [256.6 256.2 257.5]），经病态 CC 矩阵放大后就是
             * 默认出图欠曝约 1 EV 且明显偏蓝青。x3fuse get_black_level
             * (process.rs) 与 legacy x3f_process.c 都有同一条 CAMERAID
             * 判据显式跳过 Bottom；own 复刻黑电平时漏了这一段，这里补齐。
             * 只有 CAMERAID==41 生效：其它机型（含 sd Quattro 非 H 的 40）
             * 的取样区域与改动前完全相同，解码结果逐位不变。 */
            {
                const CamfEntry *cid = camf_find(&camf, "CAMERAID");
                uint32_t cameraid = 0xffffffffu;
                if (cid && cid->type == CMbM && cid->mdata &&
                    cid->mrows * cid->mcols >= 1)
                    cameraid = (uint32_t)cid->mdata[0];
                else if (cid && cid->type == CMbT && cid->text)
                    cameraid = (uint32_t)strtoul(cid->text, NULL, 10);
                if (cameraid == 41)      /* X3F_CAMERAID_SDQH */
                    use_bottom = 0;
                if (getenv("X3FDEC_METADEBUG"))
                    fprintf(stderr, "[meta] CAMERAID=%u use_bottom=%d\n",
                            cameraid, use_bottom);
            }
            int cr[4] = { -1, -1, -1, -1 };
            const CamfEntry *crs = camf_find(&camf, "DarkShieldColRange");
            if (crs && crs->mrows * crs->mcols >= 4)
                for (int i = 0; i < 4; i++) cr[i] = (int)crs->mdata[i];
            for (int c = 0; c < 3; c++) {
                const uint16_t *plane = (c < 2) ? half : top;
                int pw__ = (c < 2) ? (int)pw[0] : (int)pw[2];
                int ph__ = (c < 2) ? (int)ph[0] : (int)ph[2];
                int sc   = (c < 2) ? 2 : 1;
                int ch   = (c < 2) ? c : 0;
                int nch  = (c < 2) ? 3 : 1;
                double sum = 0; long n = 0;
#define BLK_ADD(RX1, RY1, RX2, RY2) do { \
                    int ax1 = (RX1) / sc, ay1 = (RY1) / sc; \
                    int ax2 = (RX2) / sc, ay2 = (RY2) / sc; \
                    if (ax1 < 0) ax1 = 0; \
                    if (ay1 < 0) ay1 = 0; \
                    if (ax2 >= pw__) ax2 = pw__ - 1; \
                    if (ay2 >= ph__) ay2 = ph__ - 1; \
                    if (ax1 > ax2 || ay1 > ay2) break; \
                    for (int y = ay1; y <= ay2; y++) { \
                        const uint16_t *rowp = plane + (size_t)y * pw__ * nch; \
                        for (int x = ax1; x <= ax2; x++) sum += rowp[(size_t)x * nch + ch]; \
                        n += ax2 - ax1 + 1; \
                    } \
                } while (0)
                BLK_ADD(fx1, fy1, fx2, fy2);                                   /* Top */
                if (use_bottom) BLK_ADD(bx1, by1, bx2, by2);                   /* Bottom */
                if (cr[0] >= 0 && cr[1] >= cr[0]) BLK_ADD(cr[0], 0, cr[1], 0x7fffffff); /* 左列 */
                if (cr[2] >= 0 && cr[3] >= cr[2]) BLK_ADD(cr[2], 0, cr[3], 0x7fffffff); /* 右列 */
#undef BLK_ADD
                if (n > 0) dec->black[c] = sum / n;
            }
            fprintf(stderr, "x3fdec: black FUSE = [%.1f %.1f %.1f]\n",
                    dec->black[0], dec->black[1], dec->black[2]);
        }
    }
    fprintf(stderr, "x3fdec: black = [%.1f %.1f %.1f]\n",
            dec->black[0], dec->black[1], dec->black[2]);
    /* 【实验 X3FDEC_BLACK_FORCE】"r,g,b" 强制覆盖自估计黑电平。
     * 自估计（低于均值样本均值）天然偏低 ~0.8σ；0337 暗部粉纱需要
     * 三平面各多扣 ~5-8 码，疑似黑电平真值 ~2048。 */
    {
        const char *e = getenv("X3FDEC_BLACK_FORCE");
        if (e && *e) {
            if (sscanf(e, "%lf,%lf,%lf", &dec->black[0], &dec->black[1],
                       &dec->black[2]) == 3)
                fprintf(stderr, "x3fdec: black FORCED = [%.1f %.1f %.1f]\n",
                        dec->black[0], dec->black[1], dec->black[2]);
        }
    }

    /* --- 三层重建 --- */
    if (layout) {
        int hw = pw[0], hh = ph[0];
        int ew = hw * 2, eh = hh * 2;
        /* 缺陷修复: 半层键合区点状阴影列（实测每16列1根, x≡0 mod16）+
         * 死列 → 在 delta 域修复（fuse_quattro 内 repair_defect_deltas），
         * 不在 u16 域做列插值 —— 列插值会把斜穿软边缘的 3-4 列渐变阶梯化，
         * 与顶层平滑边缘错位 → 融合 delta 出现 ±860~1000 虚假摆动 →
         * 全分辨率边界紫/绿斑。顶层保留 u16 列修复（实测仅边界1根）。 */
        stage_t("  pre: black+alloc");
        /* F23 出厂标定坏点表逐像素修复（Kalpanika/SPP 同源方案）。
         * 置于统计式修复之前：标定表覆盖键合阴影列点状缺陷与死点，
         * 覆盖后统计式列修复（半层 delta 域 + 顶层 u16）默认跳过 ——
         * 统计式列检测在密草场景把贯通全高的内容竖直结构误判为死列，
         * 全高插值抹掉内容 → 2x 上采样紫/绿竖带（DP2Q6341 实证）。 */
        g_calib_chroma_done = 0; g_calib_luma_done = 0;
        if (!getenv("X3FDEC_NOCALIB")) {
            const CamfEntry *bpc = camf_find(&camf, "BadPixelsChromaF23");
            if (bpc && bpc->mdata && !getenv("X3FDEC_NOCHROMAFIX")) {
                repair_bad_pixels_f23(half, 3, hw, hh, bpc->mdata,
                                      bpc->mrows * bpc->mcols, "chroma/half");
                g_calib_chroma_done = 1;
            }
            const CamfEntry *bpl = camf_find(&camf, "BadPixelsLumaF23");
            if (bpl && bpl->mdata && !getenv("X3FDEC_NOLUMAFIX")) {
                repair_bad_pixels_f23(top, 1, pw[2], ph[2], bpl->mdata,
                                      bpl->mrows * bpl->mcols, "luma/top");
                g_calib_luma_done = 1;
            }
            stage_t("  calib badpix");
        }
        /* X3FDEC_NOTOPCOL=1 跳过（紫边二分实验：顶层 u16 列插值疑似
         * 在硬边缘处抹平 t2 基线 → dB/dG 共模误差 → 色度紫条）
         * 出厂标定表已覆盖时跳过（X3FDEC_STATDEFECT=1 强制启用对照） */
        if (!getenv("X3FDEC_NOTOPCOL") &&
            (!g_calib_luma_done || getenv("X3FDEC_STATDEFECT")))
            repair_defect_columns_u16(top, 1, 0, pw[2], ph[2], 80.0, 50.0, 150.0, 0.25);
        /* 顶层奇偶列交替 (P=2) 分行带校正。
         *
         * 【2026-10-21 顶层 flatfield 默认关闭】column_flatfield_u16 的列电平
         * 取自**整幅图像**的逐列中位数、基线是 33 列滑动均值 —— 整幅中位数
         * 主要由内容（每列的天空/地面/建筑占比）决定，16~65px 尺度的内容
         * 结构被当成"FPN 增益"乘进每一行，平坦天空里表现为宽竖条纹
         * （DP2Q1884 实测：开着 93.7 → 关掉 8.4，scene×1e-4；x3fuse 为 ~8）。
         * 真实顶层 FPN 仅 3~4 raw（x3fuse 预处理实测 30→3.5 是错觉——它根本
         * 没这一步），无需此校正。需要时 X3FDEC_TOPCOL_CAP=30 重新打开。 */
        {
            double tcap = 0.0;
            const char *e = getenv("X3FDEC_TOPCOL_CAP");
            if (e) tcap = atof(e);
            if (tcap > 0)
                column_flatfield_u16(top, 1, 0, pw[2], ph[2], tcap, 1);
            /* X3FDEC_NOTOPP2=1 跳过（紫边二分实验） */
            if (!getenv("X3FDEC_NOTOPP2"))
                column_period_correct_u16(top, 1, 0, pw[2], ph[2], 2, 128, 15.0);
        }
        if (getenv("X3FDEC_DUMPLANES")) {
            FILE *fh = fopen("half.raw", "wb"), *ft = fopen("top.raw", "wb");
            if (fh) { fwrite(half, 2, (size_t)hw * hh * 3, fh); fclose(fh); }
            if (ft) { fwrite(top, 2, (size_t)pw[2] * ph[2], ft); fclose(ft); }
            fprintf(stderr, "dumped: half %dx%d x3, top %dx%d\n",
                    hw, hh, pw[2], ph[2]);
        }
        stage_t("  pre: top col repairs");
        /* ActiveImageArea（展开图坐标 [x1,y1,x2,y2] 闭区间）；act[] 为其
         * 半层坐标，供 fuse_quattro 内 NLM 的 active 子区使用（与 x3fuse
         * 的 denoise-active 语义一致：NLM 只跑 active，边界由子区内容
         * 反射，不引入 active 外的掩蔽列/垃圾数据）。 */
        int ax = 0, ay = 0, aw = ew, ah = eh;
        int act[4] = { 0, 0, hw, hh };
        const CamfEntry *aa = camf_find(&camf, "ActiveImageArea");
        if (aa && aa->mrows * aa->mcols >= 4) {
            int x1 = (int)aa->mdata[0], y1 = (int)aa->mdata[1];
            int x2 = (int)aa->mdata[2], y2 = (int)aa->mdata[3];
            if (x2 >= x1 && y2 >= y1 && x1 >= 0 && y1 >= 0 &&
                x2 < ew && y2 < eh) {
                ax = x1; ay = y1;
                aw = x2 - x1 + 1; ah = y2 - y1 + 1;
                act[0] = x1 / 2; act[1] = y1 / 2;
                act[2] = (aw + 1) / 2; act[3] = (ah + 1) / 2;
                if (act[0] + act[2] > hw) act[2] = hw - act[0];
                if (act[1] + act[3] > hh) act[3] = hh - act[1];
            }
        }
        /* 逐通道裁剪标记（过曝矫正 mode7）：fuse_quattro 内按修复前 raw
         * 判饱和。饱和值 = CAMF ImageDepth 的 (1<<depth)-1（与下面
         * dec->maxraw 同源，这里提前读一遍；读不到就按 14bit 满码）。 */
        uint8_t *cm = NULL;
        double satraw[3] = { 16383, 16383, 16383 };
        if (g_clip_requested || getenv("X3FDEC_HLPDBG")) {
            const CamfEntry *id0 = camf_find(&camf, "ImageDepth");
            if (id0 && id0->mrows * id0->mcols >= 3)
                for (int i = 0; i < 3; i++) {
                    double d = id0->mdata[i];
                    if (d >= 8 && d <= 16) satraw[i] = (1u << (int)d) - 1;
                }
            if (getenv("X3FDEC_HLPDBG"))
                cm = malloc((size_t)ew * eh);       /* 只为复用下方路径 */
            else
                cm = malloc((size_t)ew * eh);
            if (cm) memset(cm, 0, (size_t)ew * eh);
        }
        /*【HLP 探针】专用高光像素：CAMF HighlightPixelsInfo
         * (type6, info[0..3]=行偏移/列偏移/行距/列距)，读数混在半层
         * Huffman 流里。这里验证：网格位置上的半层 G 在裁剪区是否
         * 真的没裁（SPP 用它们重建高光的数据源）。X3FDEC_HLPDBG=1。 */
        if (getenv("X3FDEC_HLPDBG")) {
            const CamfEntry *hi = camf_find(&camf, "HighlightPixelsInfo");
            if (hi && hi->mrows * hi->mcols >= 4) {
                int ro = (int)hi->mdata[0], co = (int)hi->mdata[1];
                int rp = (int)hi->mdata[2], cp = (int)hi->mdata[3];
                fprintf(stderr, "[hlp] raw info = %d %d %d %d (half %dx%d, satG=%.0f)\n",
                        ro, co, rp, cp, hw, hh, satraw[1]);
                if (rp != 0 && cp != 0) {
                    if (rp < 0) { ro = hh - 1 - ro; rp = -rp; }
                    if (cp < 0) { co = hw - 1 - co; cp = -cp; }
                    /* 变体矩阵：HLP 采样位置/层的解释还没定，全部试一遍。
                     * v0: 半层G@CAMF网格(半层单位)  v1: 半层R@同
                     * v2: 顶层B@全分辨率网格(CAMF单位=全分辨率像素)
                     * v3: 半层G@全分辨率网格/2
                     * 参照：全体半层G 的裁剪率（普通像素基线） */
                    struct { const char *name; int plane; int div; int ch; } v4[] = {
                        { "half-G@half-grid", 0, 1, 1 },
                        { "half-R@half-grid", 0, 1, 0 },
                        { "top-B@full-grid",  1, 1, 0 },
                        { "half-G@full/2",    0, 2, 1 },
                    };
                    long bn = 0, bclip = 0;
                    for (int y = 0; y < hh; y++)
                        for (int x = 0; x < hw; x++)
                            if (half[((size_t)y * hw + x) * 3 + 1] >= satraw[1]) bclip++;
                    fprintf(stderr, "[hlp] baseline normal half-G clipped %ld/%ld (%.2f%%)\n",
                            bclip, (long)hw * hh, 100.0 * bclip / (hw * hh));
                    for (int vi = 0; vi < 4; vi++) {
                        int plane = v4[vi].plane;          /* 0=half 1=top */
                        double sc = (vi == 3) ? 0.5 : 1.0; /* 网格坐标→平面坐标 */
                        int gh = plane ? ph[2] : (int)(hh / sc);
                        int gw = plane ? pw[2] : (int)(hw / sc);
                        long n = 0, nclip = 0, hot = 0, hotclip = 0;
                        double s = 0;
                        /* 按局部亮度（同 2x2 块其他 3 个顶层像素均值）分桶 */
                        double bsum[4] = {0,0,0,0}; double bmax[4] = {0,0,0,0};
                        long bn2[4] = {0,0,0,0};
                        double TclipL = satraw[2] - dec->black[2];
                        double satl = plane ? satraw[2] : satraw[v4[vi].ch];
                        for (int gy = ro; gy < gh; gy += rp)
                            for (int gx = co; gx < gw; gx += cp) {
                                int py = (int)(gy * sc), px = (int)(gx * sc);
                                double v;
                                if (plane == 0)
                                    v = half[((size_t)py * hw + px) * 3 + v4[vi].ch];
                                else
                                    v = top[(size_t)py * pw[2] + px];
                                n++; s += v;
                                if (v >= satl) nclip++;
                                if (v >= 0.9 * satl) {
                                    hot++;
                                    if (v >= satl) hotclip++;
                                }
                                if (plane == 1) {
                                    int hy = py / 2, hx = px / 2;
                                    if (hy < hh && hx < hw) {
                                        double acc = 0; int cnt = 0;
                                        for (int dy = 0; dy < 2; dy++)
                                            for (int dx = 0; dx < 2; dx++) {
                                                int yy = 2 * hy + dy, xx = 2 * hx + dx;
                                                if (yy == py && xx == px) continue;
                                                acc += top[(size_t)yy * pw[2] + xx];
                                                cnt++;
                                            }
                                        double t = (cnt ? acc / cnt : 0) - dec->black[2];
                                        int b = t < 0.5 * TclipL ? 0 :
                                                t < 0.85 * TclipL ? 1 :
                                                t < 0.98 * TclipL ? 2 : 3;
                                        bsum[b] += v; bn2[b]++;
                                        if (v > bmax[b]) bmax[b] = v;
                                    }
                                }
                            }
                        fprintf(stderr, "[hlp] %-18s n=%ld clip=%.2f%% hot>=90%%=%ld hotclip=%.1f%% mean=%.0f\n",
                                v4[vi].name, n, n ? 100.0 * nclip / n : 0.0,
                                hot, hot ? 100.0 * hotclip / hot : 0.0, n ? s / n : 0.0);
                        if (plane == 1)
                            for (int b = 0; b < 4; b++)
                                fprintf(stderr, "[hlp]   t2 bucket %d: n=%ld HLPmean=%.0f HLPmax=%.0f\n",
                                        b, bn2[b], bn2[b] ? bsum[b] / bn2[b] : 0.0, bmax[b]);
                    }
                }
            } else {
                fprintf(stderr, "[hlp] no HighlightPixelsInfo in CAMF\n");
            }
        }
        /* 挂到 dec 上再调：fuse_quattro 内部 die() 走 longjmp 时
         * own_scene_linear 的恢复点会把它一起 free。 */
        dec->clipmask = cm;
        /* 诊断：处理后的顶层 base = top - blk2（与 final B 对照，确认
         * 暗线在处理后是否已存在于 base） */
        own_lag_parse();
        if (getenv("X3F_OWN_LAGDUMP")) {
            int cx = g_lag_cx, cy = g_lag_cy, cw = g_lag_w, ch = g_lag_h;
            int lx0 = cx - cw / 2, ly0 = cy - ch / 2;
            FILE *ft = fopen("lag_top_base.txt", "wb");
            if (ft) {
                fprintf(ft, "# top-base w=%d h=%d x0=%d y0=%d bw=%d bh=%d blk2=%.1f\n",
                        ew, eh, lx0, ly0, cw, ch, dec->black[2]);
                for (int j = 0; j < ch; j++) {
                    int yy = ly0 + j;
                    for (int i = 0; i < cw; i++) {
                        int xx = lx0 + i;
                        float v = 0.0f;
                        if (xx >= 0 && xx < ew && yy >= 0 && yy < eh)
                            v = (float)top[(size_t)yy * pw[2] + xx]
                                - (float)dec->black[2];
                        fprintf(ft, "%.3f%c", v, (i + 1 < cw) ? ' ' : '\n');
                    }
                }
                fclose(ft);
            }
        }
        float *fused = fuse_quattro(half, hw, hh, top, pw[2], dec->black, act,
                                    cm, satraw);
        /* 诊断：最终融合线性 RGB 的 lagdump 裁剪（X3F_OWN_LAGDUMP） */
        own_lag_parse();
        if (getenv("X3F_OWN_LAGDUMP")) {
            int cx = g_lag_cx, cy = g_lag_cy, cw = g_lag_w, ch = g_lag_h;
            int lx0 = cx - cw / 2, ly0 = cy - ch / 2;
            const char *fn[3] = { "lag_final_R.txt", "lag_final_G.txt",
                                  "lag_final_B.txt" };
            for (int cn = 0; cn < 3; cn++) {
                FILE *f = fopen(fn[cn], "wb");
                if (!f) continue;
                fprintf(f, "# final chn=%d w=%d h=%d x0=%d y0=%d bw=%d bh=%d\n",
                        cn, ew, eh, lx0, ly0, cw, ch);
                for (int j = 0; j < ch; j++) {
                    int yy = ly0 + j;
                    for (int i = 0; i < cw; i++) {
                        int xx = lx0 + i;
                        float v = 0.0f;
                        if (xx >= 0 && xx < ew && yy >= 0 && yy < eh)
                            v = fused[((size_t)yy * ew + xx) * 3 + cn];
                        fprintf(f, "%.3f%c", v, (i + 1 < cw) ? ' ' : '\n');
                    }
                }
                fclose(f);
            }
            fprintf(stderr, "[lagdump] wrote lag_final_*.txt\n");
        }
        if (cm) {
            /* 与 bmt 同步裁到 ActiveImageArea */
            uint8_t *crop = malloc((size_t)aw * ah);
            if (crop) {
                for (int y = 0; y < ah; y++)
                    memcpy(crop + (size_t)y * aw,
                           cm + (size_t)(ay + y) * ew + ax, (size_t)aw);
            }
            free(cm);
            dec->clipmask = crop;
        }
        free(half); free(top);
        dec->w = aw; dec->h = ah;
        dec->bmt = malloc((size_t)aw * ah * 3 * sizeof(float));
        if (!dec->bmt) die("out of memory");
        for (int y = 0; y < ah; y++)
            memcpy(dec->bmt + (size_t)y * aw * 3,
                   fused + ((size_t)(ay + y) * ew + ax) * 3,
                   (size_t)aw * 3 * sizeof(float));
        free(fused);
        stage_t("decode_file_wb total");
    } else {
        /* binned: 三平面同尺寸 */
        dec->w = pw[2]; dec->h = ph[2];
        dec->bmt = malloc((size_t)dec->w * dec->h * 3 * sizeof(float));
        if (!dec->bmt) die("out of memory");
        if (pw[0] != pw[2] || ph[0] != ph[2]) die("binned plane size mismatch");
        for (size_t i = 0; i < (size_t)dec->w * dec->h; i++) {
            dec->bmt[i * 3]     = (float)((double)half[i * 3]     - dec->black[0]);
            dec->bmt[i * 3 + 1] = (float)((double)half[i * 3 + 1] - dec->black[1]);
            dec->bmt[i * 3 + 2] = (float)((double)top[i]          - dec->black[2]);
        }
        free(half); free(top);
    }

    /* --- 镜头空间增益 sgain（SPP 口径）：黑位已扣的 BMT 平面逐通道乘。
     * 归一化（÷span）与乘法可交换，所以直接乘在 bmt 上与
     * `sg[c] * (v-black)/span` 等价。
     * 注意：x3fuse 只在 u16/DNG 输出路径（convert_data / get_image_impl /
     * apply_highlight_clip_dng）施加 sgain，x3f_scene_linear 不施加
     * （scene.rs 零引用）。本实现与 scene 契约对齐 → 默认关闭；
     * 将来接管 DNG 路径时用 X3FDEC_SGAIN=1 打开。 --- */
    if (getenv("X3FDEC_SGAIN")) {
        SgCorr sgcorr[SG_MAXCORR];
        int sgnum = sg_get_merrill_type(&camf, sgcorr);
        if (sgnum > 0) {
            for (int y = 0; y < dec->h; y++) {
                float *row = dec->bmt + (size_t)y * dec->w * 3;
                for (int x = 0; x < dec->w; x++) {
                    for (int c = 0; c < 3; c++)
                        row[(size_t)x * 3 + c] *= (float)
                            sg_calc(sgcorr, sgnum, y, x, c, dec->h, dec->w);
                }
            }
            sg_cleanup(sgcorr, sgnum);
            fprintf(stderr, "x3fdec: sgain applied (%d corr)\n", sgnum);
        }
    }

    /* --- 元数据 --- */
    /* max_raw：Quattro 走 CAMF ImageDepth（逐通道 bit 数），与 x3fuse
     * x3f_get_max_raw 的 ImageDepth 分支一致：max = (1<<depth) - 1。 */
    for (int i = 0; i < 3; i++) {
        dec->maxraw[i] = 16383; dec->wbgain[i] = 1.0;
        dec->dig_gain[i] = 1.0;
    }
    dec->iso_scale = 1.0;
    {
        const CamfEntry *id = camf_find(&camf, "ImageDepth");
        if (id && id->mrows * id->mcols >= 3)
            for (int i = 0; i < 3; i++) {
                double d = id->mdata[i];
                if (d >= 8 && d <= 16) dec->maxraw[i] = (1u << (int)d) - 1;
            }
    }

    /* WB 模式码 → 预设名。Quattro 的码表与老机型不同
     * （x3f_get_wb，meta.rs）：1=Auto 2=Sunlight 3=Shade 4=Overcast
     * 5=Incandescent 6=Fluorescent 7=Flash 8=Custom
     * 11=ColorTemp 12=AutoLSP；未知码一律 Auto。
     * ★键名必须与文件内属性表逐字一致（DP2Q2661 实测，2026-10-26）：
     *   WhiteBalanceGains 表键 = Sunlight/Shade/Overcast/Incandescent/
     *   Fluorescent/Flash/Auto/AutoLSP/Custom。x3fuse 上游写的
     *   "Shadow"/"Florescent" 是错名——属性表没有这两个键，Shadow 片
     *   （WB mode 3）增益/CCM 查找双双 miss → wb=[1,1,1] → 渲染全黑
     *   （fuse 路径则直接 RAW 载入失败 -4 回退内嵌 JPEG）。 */
    int wbmode = 1;
    const CamfEntry *wbm = camf_find(&camf, "WhiteBalance");
    if (wbm && wbm->mcols >= 1) wbmode = (int)camf_matrix_get(wbm, 0, 0);
    const char *wbn = "Auto";
    switch (wbmode) {
    case 1: wbn = "Auto"; break;
    case 2: wbn = "Sunlight"; break;
    case 3: wbn = "Shade"; break;
    case 4: wbn = "Overcast"; break;
    case 5: wbn = "Incandescent"; break;
    case 6: wbn = "Fluorescent"; break;
    case 7: wbn = "Flash"; break;
    case 8: wbn = "Custom"; break;
    case 11: wbn = "ColorTemp"; break;
    case 12: wbn = "AutoLSP"; break;
    }
    if (wb_override && wb_override[0]) wbn = wb_override;
    /* ★保险（DP2Q2661 全黑事故，2026-10-26）：所选 WB 在 WhiteBalanceGains
     * 属性表里 miss 时回退 Auto。错名/未知预设会让增益与 CCM 双双落空，
     * wb=[1,1,1] 一路渲染成全黑；回退 Auto 最多色温略偏，不会黑屏。 */
    {
        const CamfEntry *gp = camf_matrix_for_wb(&camf, "WhiteBalanceGains", wbn, "WBGain");
        if (!(gp && gp->mrows * gp->mcols >= 3) && strcmp(wbn, "Auto")) {
            fprintf(stderr, "x3fdec: WB '%s' 增益 miss，回退 Auto\n", wbn);
            wbn = "Auto";
        }
    }
    snprintf(dec->wb_name, sizeof(dec->wb_name), "%s", wbn);
    fprintf(stderr, "x3fdec: WB mode %d (%s)\n", wbmode, wbn);

    /* 白平衡增益：规范路径是 WhiteBalanceGains 属性表 (wb → 矩阵名)；
     * x3f_get_gain 的三个 GainFact 修正逐通道乘上。 */
    if (getenv("X3FDEC_METADEBUG")) {
        static const char *probe[] = { "WhiteBalanceGains",
                                       "WhiteBalanceColorCorrections",
                                       "DP1_WhiteBalanceGains",
                                       "DP1_WhiteBalanceColorCorrections",
                                       "WhiteBalanceIlluminants",
                                       "WhiteBalanceCorrections",
                                       "SunlightCCMatrix", "SunlightWBGain" };
        for (int pi = 0; pi < 8; pi++) {
            const char *pv = NULL;
            if (camf_prop(&camf, probe[pi], wbn, &pv))
                fprintf(stderr, "[meta] %s[\"%s\"] -> \"%s\"\n",
                        probe[pi], wbn, pv);
            else {
                const CamfEntry *pe = camf_find(&camf, probe[pi]);
                fprintf(stderr, "[meta] %-34s %s\n", probe[pi],
                        pe ? (pe->type == CMbM ? "矩阵条目（无属性表或未命中）" : "条目存在")
                           : "无");
            }
        }
    }
    const CamfEntry *g = camf_matrix_for_wb(&camf, "WhiteBalanceGains", wbn, "WBGain");
    if (getenv("X3FDEC_METADEBUG")) {
        const CamfEntry *dm = camf_find(&camf, "SunlightCCMatrix");
        if (dm && dm->mdata)
            fprintf(stderr, "[meta] SunlightCCMatrix %dx%d = "
                    "[%.5f %.5f %.5f / %.5f %.5f %.5f / %.5f %.5f %.5f]\n",
                    dm->mrows, dm->mcols,
                    dm->mdata[0], dm->mdata[1], dm->mdata[2],
                    dm->mdata[3], dm->mdata[4], dm->mdata[5],
                    dm->mdata[6], dm->mdata[7], dm->mdata[8]);
        if (g && g->mdata)
            fprintf(stderr, "[meta] WBGain %dx%d = [%.5f %.5f %.5f]\n",
                    g->mrows, g->mcols,
                    g->mdata[0], g->mdata[1], g->mdata[2]);
    }
    if (g && g->mrows * g->mcols >= 3) {
        for (int i = 0; i < 3; i++) dec->wbgain[i] = g->mdata[i];
        static const char *facts[] = { "SensorAdjustmentGainFact",
                                       "TempGainFact", "FNumberGainFact" };
        for (int fi = 0; fi < 3; fi++) {
            const CamfEntry *f = camf_find(&camf, facts[fi]);
            if (f && f->mrows * f->mcols >= 3)
                for (int i = 0; i < 3; i++) dec->wbgain[i] *= f->mdata[i];
        }
    }
    /* CC 矩阵：WhiteBalanceColorCorrections 属性表 → "AutoCCMatrix" 一类 */
    const CamfEntry *cc = camf_matrix_for_wb(&camf, "WhiteBalanceColorCorrections",
                                             wbn, "CCMatrix");
    if (cc && cc->mrows * cc->mcols >= 9) {
        for (int i = 0; i < 9; i++) dec->cc[i] = cc->mdata[i];
    }
    /* ISO 缩放（get_conv_controlled）：CaptureISO / SensorISO */
    {
        double sensor_iso = 0, capture_iso = 0;
        const CamfEntry *si = camf_find(&camf, "SensorISO");
        const CamfEntry *ci = camf_find(&camf, "CaptureISO");
        if (si && si->mrows * si->mcols >= 1) sensor_iso = si->mdata[0];
        if (ci && ci->mrows * ci->mcols >= 1) capture_iso = ci->mdata[0];
        if (sensor_iso > 0 && capture_iso > 0)
            dec->iso_scale = capture_iso / sensor_iso;
    }
    /* 数字 ISO 增益（列缩放进矩阵；缺省 [1,1,1]） */
    {
        const CamfEntry *dg = camf_find(&camf, "DigitalISOGain");
        if (dg && dg->mrows * dg->mcols >= 3)
            for (int i = 0; i < 3; i++) dec->dig_gain[i] = dg->mdata[i];
    }
    const CamfEntry *cam = camf_find(&camf, "CAMMODEL");
    if (cam && cam->text) snprintf(dec->camera, sizeof(dec->camera), "%s", cam->text);
    if (!dec->camera[0]) snprintf(dec->camera, sizeof(dec->camera), "Sigma Quattro");

    fprintf(stderr, "x3fdec: camera=%s max_raw=[%.0f %.0f %.0f] wb=[%.3f %.3f %.3f]"
            " iso_scale=%.3f dig=[%.3f %.3f %.3f]\n",
            dec->camera, dec->maxraw[0], dec->maxraw[1], dec->maxraw[2],
            dec->wbgain[0], dec->wbgain[1], dec->wbgain[2],
            dec->iso_scale, dec->dig_gain[0], dec->dig_gain[1], dec->dig_gain[2]);

    /* sd Quattro H 机内 as-shot 链：双判据门控，其它机型不进入。 */
    {
        uint32_t tf = (raw_de->sec_type << 16) | raw_de->sec_format;
        sdqh_prepare(&camf, dec, dec->wb_name, tf);
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* 渲染                                                                 */
/* ------------------------------------------------------------------ */

/* ================================================================== */
/* scene-linear 装配（与 x3fuse x3f_scene_linear 同构的对外契约）         */
/* ================================================================== */

/* 3x3 行主序，作用在列向量上 */
static void m3_mul(const double a[9], const double b[9], double out[9])
{
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            out[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c]
                           + a[r * 3 + 2] * b[6 + c];
}

/* x3f_matrix.c 同款常量（D65） */
static void srgb_to_xyz(double m[9])
{
    m[0] = 0.4124; m[1] = 0.3576; m[2] = 0.1805;
    m[3] = 0.2126; m[4] = 0.7152; m[5] = 0.0722;
    m[6] = 0.0193; m[7] = 0.1192; m[8] = 0.9505;
}

static void xyz_to_srgb(double m[9])
{
    m[0] = +3.2406; m[1] = -1.5372; m[2] = -0.4986;
    m[3] = -0.9689; m[4] = +1.8758; m[5] = +0.0415;
    m[6] = +0.0557; m[7] = -0.2040; m[8] = +1.0570;
}

/* camera_to_rgb = iso_scale · XYZ_to_sRGB · (sRGB_to_XYZ · CC) · diag(gain)，
 * 列再乘 DigitalISOGain —— 公式链与 get_conv_controlled /
 * x3f_get_raw_to_xyz 完全一致。失败（缺 CC）返回 0。 */
static int build_camera_to_rgb(const OwnDecoded *dec, const char *wb,
                               double out[9])
{
    double s2x[9], x2s[9], bmt2xyz[9], raw2xyz[9], conv[9];
    srgb_to_xyz(s2x);
    xyz_to_srgb(x2s);
    m3_mul(s2x, dec->cc, bmt2xyz);
    double gm[9] = { dec->wbgain[0], 0, 0,
                     0, dec->wbgain[1], 0,
                     0, 0, dec->wbgain[2] };
    m3_mul(bmt2xyz, gm, raw2xyz);
    m3_mul(x2s, raw2xyz, conv);
    for (int i = 0; i < 9; i++) conv[i] *= dec->iso_scale;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            out[r * 3 + c] = conv[r * 3 + c] * dec->dig_gain[c];
    return 1;
}

/* -------- 对外：探测 -------- */

int own_probe(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    char magic[4] = { 0 };
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "FOVb", 4)) {
        fclose(f);
        return 0;   /* X3I 容器、Merrill 世代之外的都让 x3fuse 上 */
    }
    fclose(f);
    return 1;
}

/* -------- 对外：场景线性解码 -------- */

void own_scene_linear_free(float *data)
{
    free(data);
}

/* scene_linear 像素循环（行分块并行, 每像素 double 数学与串行版逐位一致） */
typedef struct {
    const float *bmt; float *out; int w;
    const double *span, *conv; int y0, y1;
} CcmJob;

static void *ccm_job(void *arg)
{
    CcmJob *j = (CcmJob *)arg;
    for (int y = j->y0; y < j->y1; y++) {
        size_t i = (size_t)y * j->w;
        for (int x = 0; x < j->w; x++, i++) {
            double s0 = j->bmt[i * 3]     / j->span[0];
            double s1 = j->bmt[i * 3 + 1] / j->span[1];
            double s2 = j->bmt[i * 3 + 2] / j->span[2];
            j->out[i * 3]     = (float)(j->conv[0] * s0 + j->conv[1] * s1 + j->conv[2] * s2);
            j->out[i * 3 + 1] = (float)(j->conv[3] * s0 + j->conv[4] * s1 + j->conv[5] * s2);
            j->out[i * 3 + 2] = (float)(j->conv[6] * s0 + j->conv[7] * s1 + j->conv[8] * s2);
        }
    }
    return NULL;
}

/* 色度降噪 3x3 中值（行分块并行, 逐位一致） */
typedef struct {
    const float *crA, *cbA; float *crB, *cbB; int w, h, y0, y1;
} NrJob;

static void *nr_job(void *arg)
{
    NrJob *j = (NrJob *)arg;
    for (int y = j->y0; y < j->y1; y++) {
        for (int x = 0; x < j->w; x++) {
            float rs[9], bs[9];
            int m = 0;
            for (int dy = -1; dy <= 1; dy++) {
                int yy = y + dy; if (yy < 0) yy = 0; if (yy >= j->h) yy = j->h - 1;
                for (int dx = -1; dx <= 1; dx++) {
                    int xx = x + dx; if (xx < 0) xx = 0; if (xx >= j->w) xx = j->w - 1;
                    size_t k = (size_t)yy * j->w + xx;
                    rs[m] = j->crA[k]; bs[m] = j->cbA[k]; m++;
                }
            }
            for (int a = 1; a < 9; a++) {
                float kv = rs[a]; int b2 = a - 1;
                while (b2 >= 0 && rs[b2] > kv) { rs[b2 + 1] = rs[b2]; b2--; }
                rs[b2 + 1] = kv;
                kv = bs[a]; b2 = a - 1;
                while (b2 >= 0 && bs[b2] > kv) { bs[b2 + 1] = bs[b2]; b2--; }
                bs[b2 + 1] = kv;
            }
            size_t i = (size_t)y * j->w + x;
            j->crB[i] = rs[4]; j->cbB[i] = bs[4];
        }
    }
    return NULL;
}

/* 色度回合成 RGB（行分块并行, 逐位一致） */
typedef struct {
    float *out; const float *cr, *cb; int w, y0, y1;
} RecJob;

static void *rec_job(void *arg)
{
    RecJob *j = (RecJob *)arg;
    for (int y = j->y0; y < j->y1; y++) {
        size_t i = (size_t)y * j->w;
        for (int x = 0; x < j->w; x++, i++) {
            double y_ = 0.2126 * j->out[i * 3] + 0.7152 * j->out[i * 3 + 1]
                      + 0.0722 * j->out[i * 3 + 2];
            double r = y_ + j->cr[i], b = y_ + j->cb[i];
            double g = (y_ - 0.2126 * r - 0.0722 * b) / 0.7152;
            j->out[i * 3]     = (float)r;
            j->out[i * 3 + 1] = (float)g;
            j->out[i * 3 + 2] = (float)b;
        }
    }
    return NULL;
}

int own_scene_linear(const char *path,
                     float **data, unsigned *w, unsigned *h,
                     unsigned short *orientation,
                     double *camera_to_rgb,
                     const char *wb_preset,
                     int denoise_intensity,
                     const void *denoise)
{
    OwnDecoded dec;
    memset(&dec, 0, sizeof(dec));
    /* 2026-11-14：接住宿主的 nlm / x3f_denoise_t（原来在这里整个丢掉）。
     * dn 的 AUTO 亮度判据已由 load_common_inner 解成 0/1。 */
    own_dn_setup(denoise_intensity, (const x3f_denoise_t *)denoise);

    own_jmp_armed = 1;
    if (setjmp(own_jmp)) {
        own_jmp_armed = 0;
        free(dec.bmt);
        free(dec.clipmask);
        *data = NULL;
        /* rc=1 = 这不是 own 能解的形态（机型不适用）；rc=-1 = 真的解坏了。
         * 见 own_die_unsupported 的说明。 */
        return own_die_unsupported ? 1 : -1;
    }
    own_die_unsupported = 0;

    if (decode_file_wb(path, &dec, wb_preset) != 0) {
        own_jmp_armed = 0;
        g_sdqh.active = 0;
        free(dec.bmt);
        return -1;
    }

    /* 三层布局才是我们覆盖的 Quattro 形态；binned（同尺寸三平面）退回
     * x3fuse（SD1 Merrill / Quattro binning 都走那边）。 */
    int is_quattro = strstr(dec.camera, "Quattro") != NULL;
    if (!is_quattro) {
        own_jmp_armed = 0;
        g_sdqh.active = 0;
        free(dec.bmt);
        free(dec.clipmask);
        return 1;
    }

    double conv[9];
    if (!build_camera_to_rgb(&dec, dec.wb_name, conv)) {
        own_jmp_armed = 0;
        g_sdqh.active = 0;
        free(dec.bmt);
        free(dec.clipmask);
        return 1;           /* 缺色彩标定：不是我们该接的文件 */
    }
    own_jmp_armed = 0;

    /* 像素循环：norm[c] = (raw-black)/(max-black)（bmt 已扣黑），
     * scene = conv · norm。负值与 >1 原样保留（extended linear）。 */
    size_t n = (size_t)dec.w * dec.h;
    float *out = malloc(n * 3 * sizeof(float));
    if (!out) { free(dec.bmt); return -1; }
    double span[3];
    for (int c = 0; c < 3; c++)
        span[c] = dec.maxraw[c] - dec.black[c] > 1.0
                ? dec.maxraw[c] - dec.black[c] : 1.0;

    {
        int ntj = own_nt();   /* 行分块数按核数走（原来写死 6） */
        if (g_sdqh.active && g_sdqh.tone) {
            SdqhJob jobs[16];
            for (int i = 0; i < ntj; i++) {
                jobs[i].bmt = dec.bmt; jobs[i].out = out; jobs[i].w = dec.w;
                jobs[i].span = span; jobs[i].chain = &g_sdqh;
                jobs[i].y0 = (int)((size_t)i * dec.h / ntj);
                jobs[i].y1 = (int)((size_t)(i + 1) * dec.h / ntj);
            }
            run_jobs(sdqh_pixel_job, jobs, sizeof(SdqhJob), ntj);
        } else {
            CcmJob jobs[16];
            for (int i = 0; i < ntj; i++) {
                jobs[i].bmt = dec.bmt; jobs[i].out = out; jobs[i].w = dec.w;
                jobs[i].span = span; jobs[i].conv = conv;
                jobs[i].y0 = (int)((size_t)i * dec.h / ntj);
                jobs[i].y1 = (int)((size_t)(i + 1) * dec.h / ntj);
            }
            run_jobs(ccm_job, jobs, sizeof(CcmJob), ntj);
        }
    }
    stage_t("    sl: ccm jobs");

    /* 色度降噪: CCM 大系数把 R/G 残余噪声放大成紫/绿噪点。
     * 对 Cr/Cb（亮度不动）做 3x3 中值滤波，对齐 SPP QT1 的 YUV 域降噪。
     * 单次 3x3 中值杀不掉 3px 宽的顶层 HLP 亮斑（斑占窗口多数），
     * 迭代 2 次可侵蚀 3-5px 色度斑块；中值保边，直线色边缘是迭代不动点。
     * X3FDEC_CHROMA_NR=0 关闭, X3FDEC_CHROMA_ITER=迭代次数(默认2)
     * SFD 非基准帧跳过（own_set_frame_nr(0)）：色度噪声由合成后的 Stage1 收。
     * 2026-10-24: 短暂试过默认关闭（配合 NLM 链关闭），用户实测暗部彩噪/
     * 绿色斑块明显 —— 恢复默认开。它对暗带 R−G 的贡献仅 ~+0.3 灰阶
     * （0337 vs SPP），发红主因是 NLM 链（保持关），此开关保留作诊断。 */
    {
        const char *e = getenv("X3FDEC_CHROMA_NR");
        if (g_own_frame_nr && (!e || atoi(e))) {
            int iters = 2;
            if ((e = getenv("X3FDEC_CHROMA_ITER"))) iters = atoi(e);
            if (iters < 1) iters = 1;
            size_t n = (size_t)dec.w * dec.h;
            int iw = dec.w, ih = dec.h;
            float *crA = malloc(n * sizeof(float));
            float *cbA = malloc(n * sizeof(float));
            float *crB = malloc(n * sizeof(float));
            float *cbB = malloc(n * sizeof(float));
            if (!crA || !cbA || !crB || !cbB) { free(crA); free(cbA); free(crB); free(cbB); }
            else {
                {
                    ChromaExCtx xe;
                    xe.out = out; xe.cr = crA; xe.cb = cbA; xe.w = iw;
                    run_rows(chroma_ex_rows, &xe, ih, 16);
                }
                stage_t("    sl: chroma extract");
                const char *ge = getenv("X3FDEC_GPU_NR");
            if (own_gpu_available() && (!ge || ge[0] != '0') &&
                own_gpu_chroma_nr(crA, cbA, crB, cbB, iw, ih, iters) == 0) {
                stage_t("    sl: gpu chroma_nr");
                int ntj = own_nt();   /* 行分块数按核数走（原来写死 6） */
                RecJob jobs[16];
                for (int i = 0; i < ntj; i++) {
                    jobs[i].out = out; jobs[i].cr = crA; jobs[i].cb = cbA;
                    jobs[i].w = iw;
                    jobs[i].y0 = (int)((size_t)i * ih / ntj);
                    jobs[i].y1 = (int)((size_t)(i + 1) * ih / ntj);
                }
                run_jobs(rec_job, jobs, sizeof(RecJob), ntj);
                free(crA); free(cbA); free(crB); free(cbB);
                goto chroma_done;
            }
            if (getenv("X3FDEC_GPUDBG"))
                fprintf(stderr, "[gpu] chroma fallback\n");
            for (int it = 0; it < iters; it++) {
                int ntj = own_nt();   /* 行分块数按核数走（原来写死 6） */
                NrJob jobs[16];
                for (int i = 0; i < ntj; i++) {
                    jobs[i].crA = crA; jobs[i].cbA = cbA;
                    jobs[i].crB = crB; jobs[i].cbB = cbB;
                    jobs[i].w = iw; jobs[i].h = ih;
                    jobs[i].y0 = (int)((size_t)i * ih / ntj);
                    jobs[i].y1 = (int)((size_t)(i + 1) * ih / ntj);
                }
                run_jobs(nr_job, jobs, sizeof(NrJob), ntj);
                float *t;
                t = crA; crA = crB; crB = t;
                t = cbA; cbA = cbB; cbB = t;
            }
            {
                int ntj = own_nt();   /* 行分块数按核数走（原来写死 6） */
                RecJob jobs[16];
                for (int i = 0; i < ntj; i++) {
                    jobs[i].out = out; jobs[i].cr = crA; jobs[i].cb = cbA;
                    jobs[i].w = iw;
                    jobs[i].y0 = (int)((size_t)i * ih / ntj);
                    jobs[i].y1 = (int)((size_t)(i + 1) * ih / ntj);
                }
                run_jobs(rec_job, jobs, sizeof(RecJob), ntj);
            }
            free(crA); free(cbA); free(crB); free(cbB);
            }
            chroma_done: ;
        }
    }

    *data = out;
    *w = (unsigned)dec.w;
    *h = (unsigned)dec.h;
    *orientation = (unsigned short)dec.orientation;
    memcpy(camera_to_rgb, conv, 9 * sizeof(double));
    /* 裁剪标记出槽：先释放调用方没取走的上一份，再把这份挂上
     * （own_take_clip_mask 转移所有权；非 Quattro/未请求时槽保持空）。 */
    free(g_clip_mask); g_clip_mask = NULL; g_clip_w = g_clip_h = 0;
    if (dec.clipmask) {
        g_clip_mask = dec.clipmask;
        g_clip_w = dec.w; g_clip_h = dec.h;
        dec.clipmask = NULL;
    }
    free(dec.bmt);
    stage_t("  sl: ccm+chroma+recompose");
    return 0;
}

/* -------- 实验室：线性 → sRGB PNG 预览 -------- */

static inline int srgb_encode8(double v)
{
    double s = v <= 0.0031308 ? 12.92 * v : 1.055 * pow(v, 1.0 / 2.4) - 0.055;
    if (s < 0) s = 0;
    if (s > 1) s = 1;
    return (int)(s * 255.0 + 0.5);
}

typedef struct {
    const float *linear; uint8_t *rgb8; int w, y0, y1;
} SrgbJob;

static void *srgb_job(void *arg)
{
    SrgbJob *j = (SrgbJob *)arg;
    for (int y = j->y0; y < j->y1; y++) {
        size_t i = (size_t)y * j->w;
        uint8_t *row = j->rgb8 + i * 3;
        for (int x = 0; x < j->w; x++) {
            const float *px = j->linear + (i + x) * 3;
            row[x * 3]     = (uint8_t)srgb_encode8(px[0]);
            row[x * 3 + 1] = (uint8_t)srgb_encode8(px[1]);
            row[x * 3 + 2] = (uint8_t)srgb_encode8(px[2]);
        }
    }
    return NULL;
}

int own_write_preview_png(const char *path, int w, int h, const float *linear)
{
#ifdef _WIN32
    /* Windows：PNG 写出那套依赖 zlib（见上），这个调试预览口暂时不给
     * （当前也没有调用方）。 */
    (void)path; (void)w; (void)h; (void)linear;
    return -1;
#else
    size_t n = (size_t)w * h;
    uint8_t *rgb8 = malloc(n * 3);
    if (!rgb8) return -1;
    int ntj = own_nt();   /* 行分块数按核数走（原来写死 6） */
    SrgbJob jobs[16];
    for (int i = 0; i < ntj; i++) {
        jobs[i].linear = linear; jobs[i].rgb8 = rgb8; jobs[i].w = w;
        jobs[i].y0 = (int)((size_t)i * h / ntj);
        jobs[i].y1 = (int)((size_t)(i + 1) * h / ntj);
    }
    run_jobs(srgb_job, jobs, sizeof(SrgbJob), ntj);
    write_png_rgb8(path, w, h, rgb8);
    free(rgb8);
    return 0;
#endif
}

/* -------- 分发钩子：x3f_scene_linear 同签名 --------
 *
 * fuj 侧唯一需要的改动：x3f_render.c 里那次 x3f_scene_linear(...) 调用换成
 * x3f_own_dispatch_scene_linear(...)，所有 x3f_scene_linear_free 换成
 * x3f_own_dispatch_scene_linear_free（见 hook/x3f_render.patch，共 1 处调用
 * + 5 处 free + 2 行 extern 声明）。渲染层其余零改动；SFD 七帧的每一帧
 * 都经 x3f_render_load_wb → 本钩子（X3I 帧已被 CX3F 展开成独立 X3F），
 * 所以静态照片与七帧合成一并覆盖。
 *
 * 后端选择：
 *   X3F_DECODER=own    → 强制 own（非覆盖文件返回错误，不回落）
 *   X3F_DECODER=auto   → own 优先，不支持/解坏自动回落 x3fuse（GUI「自研解码」档）。
 *                         2026-10-26 起真正生效 —— 此前 auto 与未设同义，GUI 开关
 *                         名存实亡；macOS 版 applyDecoderSwitch 的注释一直写的
 *                         就是「auto = 自研优先、解不动回落」这个意图，补齐实现。
 *   未设（缺省）        → 直接 x3fuse（2026-10-25 起 Quattro 默认改用 x3fuse；
 *                         own 自研核心仅作 A/B 对照保留，需显式 own/auto 才走）
 *   X3F_DECODER=fuse    → 显式 x3fuse（与缺省一致，仅用于对照）
 *
 * 2026-11-14 起 own 路径已接 denoise_intensity / denoise（见 own_dn_setup，
 * 驱动 XFCHAIN 两趟 NLM 的 σ、三通道权重、窗口与后端）；own 仍不使用
 * highlight_recovery（自研展开链用自己的高光重建）。wb_preset 两路原样传。
 */

extern int  x3f_scene_linear(const char *path, float **data, unsigned *w,
                             unsigned *h, unsigned short *orientation,
                             double *camera_to_rgb, int highlight_recovery,
                             unsigned char denoise_intensity,
                             const void *denoise, const char *wb_preset);
extern void x3f_scene_linear_free(float *ptr);

static _Thread_local int g_dispatch_last_fuse = 0;

/* 运行时覆盖（GUI setDecoder 经 DLL 导出的 x3f_decoder_set_override 写入的 DLL 内部
 * 全局）。壳层是 /MT 静态 CRT，_putenv 写的环境副本与这里 /MD 的 getenv 不互通。 */
extern const char *x3f_decoder_override(void);


/* ---- sd Quattro H 专属分派（2026-11-06 定案） ----
 * SDQH 一律交给 x3fuse 解码：机内 as-shot 色彩链属于 x3fuse 域，own 侧的
 * 移植链不再参与。判据用 RAW type_format == 0x00010027
 * （x3f_io.h 的 X3F_IMAGE_RAW_SDQH，SDQH 专属），其它机型一律不命中。
 *
 * 纯元数据探测：只读文件头 + 目录 + 每个 SECi 的前 28 字节，不解码、
 * 不解压 CAMF、不分配大内存、不动全局状态。X3FDEC_SDQH_FUSE=0 可退回
 * 原分派做 A/B。
 *
 * 2026-11-09：Windows 移植分支曾以「删除重复定义」为由删掉本函数，
 * 合并回 mac 主线后只剩下面 6350 行的调用，导致 `swift build` 报
 * implicit-function-declaration。这里原样补回 4.0.7 树里的唯一定义，
 * 行为与 4.0.7 逐位一致。 */
static int own_is_sdqh_file(const char *path)
{
    FILE *f;
    unsigned char b[28];
    unsigned char *dir;
    uint32_t diroff, count, i;
    int found = 0;

    if (!path) return 0;
    f = fopen(path, "rb");
    if (!f) return 0;
    if (fread(b, 1, 28, f) != 28 || memcmp(b, "FOVb", 4)) goto done;
    if (fseek(f, -4L, SEEK_END) != 0) goto done;
    if (fread(b, 1, 4, f) != 4) goto done;
    diroff = rd_u32(b);
    if (fseek(f, (long)diroff, SEEK_SET) != 0) goto done;
    if (fread(b, 1, 12, f) != 12 || memcmp(b, "SECd", 4)) goto done;
    count = rd_u32(b + 8);
    if (count == 0 || count > 4096) goto done;
    dir = (unsigned char *)malloc((size_t)count * 12);
    if (!dir) goto done;
    if (fread(dir, 1, (size_t)count * 12, f) != (size_t)count * 12) {
        free(dir);
        goto done;
    }
    for (i = 0; i < count && !found; i++) {
        unsigned char sh[28];
        uint32_t off  = rd_u32(dir + i * 12);
        uint32_t size = rd_u32(dir + i * 12 + 4);
        if (size < 28) continue;
        if (fseek(f, (long)off, SEEK_SET) != 0) break;
        if (fread(sh, 1, 28, f) != 28) break;
        if (!memcmp(sh, "SECi", 4)) {
            uint32_t tf = (rd_u32(sh + 8) << 16) | rd_u32(sh + 12);
            if (tf == 0x00010027u) found = 1;   /* X3F_IMAGE_RAW_SDQH */
        }
    }
    free(dir);
done:
    fclose(f);
    return found;
}

int x3f_own_dispatch_scene_linear(const char *path, float **data, unsigned *w,
                                  unsigned *h, unsigned short *orientation,
                                  double *camera_to_rgb,
                                  int highlight_recovery,
                                  unsigned char denoise_intensity,
                                  const void *denoise,
                                  const char *wb_preset)
{
    const char *e = x3f_decoder_override();   /* GUI 运行时覆盖，绕过 /MT 壳层 env 副本错配 */
    if (!e) e = getenv("X3F_DECODER");
    int force_own  = e && !strcmp(e, "own");
    int prefer_own = e && !strcmp(e, "auto");   /* GUI「自研解码」档：own 优先，解不动回落 */
    int sdqh_fuse  = 0;   /* 本次是否命中 sd Quattro H 强制 x3fuse 分派 */

    *data = NULL;
    g_dispatch_last_fuse = 0;

    /* 独立解码器（X3F_DECODER=independent）只走 Node 子进程的 surface 命令，
     * 不进这条显影管线；这里显式拒绝（rc==1=unsupported），避免界面误触发时
     * 静默回落 fuse 出一张成品图，与「独立解码只产原始 surface」语义冲突。 */
    if (e && !strcmp(e, "independent")) {
        *data = NULL;
        return 1;
    }

    /* sd Quattro H 默认走 **own**：X3Lumina 的 own 解码在 SDQH 上色彩是准的
     * （0184 中性点与机内 JPEG 差 <1/255），而 x3fuse 那条路有 ~5% 的绿偏，
     * 且要靠 +0.5 EV 硬补亮度。所以这里不再强制 fuse。
     *
     * 历史逃生开关保留为 opt-in：X3FDEC_SDQH_FUSE=1 才回 x3fuse（并启用下面
     * 的 SDQH 曝光补偿）；设 0 或不设都走 own，与其它 Quattro 一致。 */
    {
        const char *sf = getenv("X3FDEC_SDQH_FUSE");
        if (sf && sf[0] == '1' && own_is_sdqh_file(path)) {
            force_own = prefer_own = 0;
            sdqh_fuse = 1;
            if (getenv("X3FDEC_SDQH_DEBUG"))
                fprintf(stderr, "[dispatch] SDQH -> x3fuse (escape switch)\n");
        }
    }

    /* 缺省（未设）直接走 x3fuse；显式 own 强制自研（不回落），显式 auto 自研优先。 */
    if (force_own || prefer_own) {
        int rc = own_scene_linear(path, data, w, h, orientation,
                                  camera_to_rgb, wb_preset,
                                  (int)denoise_intensity, denoise);
        if (rc == 0 && *data)
            return 0;                       /* own 成功 */
        /* rc==1 不支持 / rc==-1 解坏：都回落 x3fuse（除非强制 own） */
        if (force_own) return rc == 0 ? -1 : rc;
        fprintf(stderr, "[dispatch] own 回落 x3fuse (%s): %s\n",
                rc == 1 ? "unsupported" : "decode-failed", path);
    }
    g_dispatch_last_fuse = 1;
    if (getenv("X3F_OE_SEL_DEBUG"))
        fprintf(stderr, "[dispatch] fuse thread=%p\n", (void*)pthread_self());
    {
        int rc = x3f_scene_linear(path, data, w, h, orientation, camera_to_rgb,
                                  highlight_recovery, denoise_intensity,
                                  denoise, wb_preset);
        /* sd Quattro H 专属曝光补偿：x3fuse 默认渲染相对机内直出欠曝约
         * 0.5 EV（4616/4617/0184 实测），在场景线性域补 2^EV 即可对齐亮度。
         * 只作用于命中 type_format==0x00010027 且实际走 fuse 的这一条路径，
         * 其它机型、own 路径、逃生开关关闭时一律不进，逐位零回归。
         * X3FDEC_SDQH_EV 可覆盖（默认 0.5，设 0 关闭、设 1 = 整一档）。 */
        if (rc == 0 && *data && sdqh_fuse) {
            const char *evs = getenv("X3FDEC_SDQH_EV");
            double ev = (evs && *evs) ? atof(evs) : 0.5;
            if (ev != 0.0 && ev > -8.0 && ev < 8.0) {
                float g = (float)pow(2.0, ev);
                size_t n = (size_t)(*w) * (size_t)(*h) * 3u;
                size_t i;
                for (i = 0; i < n; i++) (*data)[i] *= g;
                if (getenv("X3FDEC_SDQH_DEBUG"))
                    fprintf(stderr, "[dispatch] SDQH exposure %+.3f EV (x%.4f)\n",
                            ev, g);
            }
        }
        return rc;
    }
}

void x3f_own_dispatch_scene_linear_free(float *data)
{
    if (g_dispatch_last_fuse) x3f_scene_linear_free(data);
    else free(data);
}

/* 本次解码是否真的走了 SDQH 机内链（own 成功且非 fuse 回落）。 */
static int own_sdqh_native_active(void)
{
    return g_sdqh.active && !g_dispatch_last_fuse;
}

/* 上一次分发实际用的后端（1=fuse / 0=own）。
 * 用途：渲染层里有些东西是**按解码域标定的**，得分路生效 —— 典型就是 mode8
 * 的逐像素直通表（换解码 = 换 lin 域 = 必须换表，见 x3f_oe_spec_fuse.h）。 */
int own_dispatch_used_fuse(void)
{
    return g_dispatch_last_fuse;
}

/* -------- 调试：CAMF 全量 dump -------- */

void own_camf_dump(const char *path)
{
    X3F x; x3f_parse(&x, path);
    DirEntry *camf_de = NULL;
    for (int i = 0; i < x.nent; i++) {
        const uint8_t *sh = x.data + x.ent[i].offset;
        if (!memcmp(sh, "SECc", 4) && x.ent[i].camf_type == 5 &&
            (!camf_de || x.ent[i].size > camf_de->size))
            camf_de = &x.ent[i];
    }
    if (!camf_de) { fprintf(stderr, "no CAMF\n"); return; }
    Camf camf; memset(&camf, 0, sizeof(camf));
    const uint8_t *sh = x.data + camf_de->offset;
    camf_decode_type5(sh + 28, camf_de->size - 28,
                      camf_de->val0, camf_de->val1, &camf);
    camf_parse_entries(&camf);
    for (int i = 0; i < camf.nentries; i++) {
        const CamfEntry *e = &camf.entries[i];
        if (e->type == CMbM) {
            printf("%-48s M%d %dx%d  [", e->name, e->mtype, e->mrows, e->mcols);
            long n = (long)e->mrows * e->mcols;
            for (long k = 0; k < n && k < 12; k++)
                printf("%s%.5g", k ? " " : "", e->mdata[k]);
            printf("%s]\n", n > 12 ? " ..." : "");
        } else if (e->type == CMbT) {
            printf("%-48s T \"%s\"\n", e->name, e->text ? e->text : "");
        } else if (e->type == CMbP && e->vlen >= 8) {
            printf("%-48s P\n", e->name);
            int raw = getenv("X3FDEC_CAMF_RAW") != NULL;
            if (raw) {
                printf("    vlen=%u head:", (unsigned)e->vlen);
                for (uint32_t q = 0; q < 16 && 4 * q + 4 <= e->vlen; q++)
                    printf(" %08x", rd_u32(e->value + 4 * q));
                printf("\n");
            }
            uint32_t n = rd_u32(e->value);
            uint32_t soff = rd_u32(e->value + 4);
            for (uint32_t k = 0; k < n && k < 400; k++) {
                uint32_t no = soff + rd_u32(e->value + 8 + 8 * k);
                uint32_t vo = soff + rd_u32(e->value + 12 + 8 * k);
                printf("    %s = %s\n", e->entry + no, e->entry + vo);
            }
        }
    }
}
