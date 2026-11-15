/* X3F_PARALLEL.H — 内部用的并行小工具
 *
 * 用 GCD 的 dispatch_apply_f 把 n 个互相独立的迭代摊到多核。
 * 关键约束：各迭代之间必须没有共享可写状态、也没有先后依赖，
 * 这样每份结果与串行执行逐位相同。
 *
 * 编译时加 -DX3F_SERIAL 可退回单线程（用来做逐字节对照）。
 */

#ifndef X3F_PARALLEL_H
#define X3F_PARALLEL_H

#include <stddef.h>
#include <stdlib.h>

#if defined(__APPLE__) && defined(__has_include)
#  if __has_include(<dispatch/dispatch.h>)
#    include <dispatch/dispatch.h>
#    define X3F_HAVE_GCD 1
#  endif
#endif

#if defined(_WIN32) && !defined(X3F_HAVE_GCD)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#  define X3F_HAVE_WIN32 1
#endif

#ifdef X3F_SERIAL
#  undef X3F_HAVE_GCD
#  undef X3F_HAVE_WIN32
#endif

typedef void (*x3f_for_fn)(void *ctx, size_t i);

#ifdef X3F_HAVE_WIN32
/* Win32 没有 GCD，自己把迭代切成若干块发给线程。
 * 这些迭代本来就被要求互相独立，所以分块不会改变结果。 */
typedef struct {
  x3f_for_fn fn;
  void *ctx;
  size_t begin, end;
} x3f_win_job_t;

static DWORD WINAPI x3f_win_worker(LPVOID p)
{
  x3f_win_job_t *j = (x3f_win_job_t *)p;
  size_t i;
  for (i = j->begin; i < j->end; i++) j->fn(j->ctx, i);
  return 0;
}
#endif

static inline void x3f_parallel_for(size_t n, x3f_for_fn fn, void *ctx)
{
  if (n == 0) return;
#ifdef X3F_HAVE_GCD
  /* 只有 1 个迭代时没必要过一遍调度 */
  if (n == 1) { fn(ctx, 0); return; }
  dispatch_apply_f(n, DISPATCH_APPLY_AUTO, ctx, fn);
#elif defined(X3F_HAVE_WIN32)
  {
    static int s_nthread = 0;          /* 进程内探测一次（各 TU 各一份，值相同） */
    x3f_win_job_t job[64];
    HANDLE th[64];
    size_t nt, t;

    if (s_nthread == 0) {
      SYSTEM_INFO si;
      DWORD nc;
      const char *ev;
      GetSystemInfo(&si);
      nc = si.dwNumberOfProcessors;
      /* X3F_THREADS 显式覆盖：排查带宽受限的 kernel 时把线程压到物理核数
       * 常常更快，也用来复现「核多反而不快」的现场。 */
      ev = getenv("X3F_THREADS");
      if (ev != NULL && atoi(ev) > 0) nc = (DWORD)atoi(ev);
      if (nc == 0) nc = 1;
      if (nc > 64) nc = 64;
      s_nthread = (int)nc;
    }

    /* 2026-11-01 修正并行门限（原实现 n < nthread*4 就退回串行）。
     *
     * 那个 4 倍门槛在**核多的机器上反向失效**：低于 4×核数的迭代数会被判成
     * 串行，核越多越容易被判掉，正好是「配置越高、提升越不明显」的来源。
     * 典型受害者是 apply_decode_unsharp 的列块段（US_NCHUNK=64 块）：
     * 8 线程机上 64 ≥ 32 走并行，32 线程机上 64 < 128 就退回单线程。
     *
     * 现在改成：线程数按迭代数收敛（nt = min(nthread, n)），只有真剩 1 个
     * 线程时才串行。迭代本来就要求互相独立（本文件头注释的约定），把同一批
     * 迭代分给更多线程不改变任何一份结果。
     *
     * 注意 x3f_io.c 的三色平面解码**不在此列**：那处曾被这个门槛挡成串行，
     * 真正放开后解码会卡死（见那边的注释），现已显式写成串行循环。 */
    nt = (size_t)s_nthread;
    if (nt > n) nt = n;
    if (nt < 2) {
      size_t i;
      for (i = 0; i < n; i++) fn(ctx, i);
      return;
    }
    for (t = 0; t < nt; t++) {
      job[t].fn = fn; job[t].ctx = ctx;
      job[t].begin = n * t / nt;
      job[t].end   = n * (t + 1) / nt;
      th[t] = CreateThread(NULL, 0, x3f_win_worker, &job[t], 0, NULL);
      if (th[t] == NULL) x3f_win_worker(&job[t]);   /* 起不来就本线程做 */
    }
    for (t = 0; t < nt; t++)
      if (th[t] != NULL) { WaitForSingleObject(th[t], INFINITE); CloseHandle(th[t]); }
  }
#else
  { size_t i; for (i = 0; i < n; i++) fn(ctx, i); }
#endif
}

#endif /* X3F_PARALLEL_H */
