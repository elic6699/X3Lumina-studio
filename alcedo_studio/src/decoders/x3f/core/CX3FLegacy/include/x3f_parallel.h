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
    SYSTEM_INFO si;
    x3f_win_job_t job[64];
    HANDLE th[64];
    int nthread, t;

    GetSystemInfo(&si);
    nthread = (int)si.dwNumberOfProcessors;
    if (nthread > 64) nthread = 64;
    /* 迭代数太少时开线程不划算，直接串行 */
    if (nthread < 2 || n < (size_t)nthread * 4) {
      size_t i;
      for (i = 0; i < n; i++) fn(ctx, i);
      return;
    }
    for (t = 0; t < nthread; t++) {
      job[t].fn = fn; job[t].ctx = ctx;
      job[t].begin = n * (size_t)t / (size_t)nthread;
      job[t].end   = n * (size_t)(t + 1) / (size_t)nthread;
      th[t] = CreateThread(NULL, 0, x3f_win_worker, &job[t], 0, NULL);
      if (th[t] == NULL) x3f_win_worker(&job[t]);   /* 起不来就本线程做 */
    }
    for (t = 0; t < nthread; t++)
      if (th[t] != NULL) { WaitForSingleObject(th[t], INFINITE); CloseHandle(th[t]); }
  }
#else
  { size_t i; for (i = 0; i < n; i++) fn(ctx, i); }
#endif
}

#endif /* X3F_PARALLEL_H */
