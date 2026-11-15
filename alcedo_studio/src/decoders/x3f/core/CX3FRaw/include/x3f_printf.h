/* X3F_PRINTF.H
 *
 * Library for printout, depending on seriousity level.
 *
 * Copyright 2015 - Roland and Erik Karlsson
 * BSD-style - see doc/copyright.txt
 *
 */

#ifndef X3F_PRINTF_H
#define X3F_PRINTF_H

#ifdef __cplusplus
extern "C" {
#endif

/* 本地修改：上游把这个枚举常量叫 DEBUG，而构建系统在 Debug 配置下会自己
 * 在命令行定义 DEBUG 宏（SwiftPM 给 C target 的 -DDEBUG、Xcode 的
 * GCC_PREPROCESSOR_DEFINITIONS=DEBUG=1）。宏一旦存在，下面这行就会被替换成
 * "typedef enum {ERR=0, WARN=1, INFO=2, 1=3}"，直接编译失败：
 *     error: expected identifier
 *     error: duplicate case value: 'WARN' and '1' both equal '1'
 * 这个代码库里没有任何地方用 #ifdef DEBUG 做条件编译，所以先撤掉它，
 * 让 DEBUG 老老实实当枚举常量用。
 * 为什么打在 csrc 而不是只在 Sources/CX3FRaw/include：那一份是
 * build_x3fuse.sh 每次从 csrc 拷贝过去的，改那边会被覆盖。 */
#ifdef DEBUG
#undef DEBUG
#endif

typedef enum {ERR=0, WARN=1, INFO=2, DEBUG=3} x3f_verbosity_t;

extern x3f_verbosity_t x3f_printf_level;

/* `__attribute__((format(printf, ...)))` is a GCC/Clang extension that lets
 * the compiler typecheck the variadic args against `fmt`. MSVC's cl.exe
 * doesn't understand it (it's a syntax error there), so expand it away on
 * non-GNU compilers — it's a diagnostic hint, not part of the ABI. */
#if defined(__GNUC__) || defined(__clang__)
#  define X3F_PRINTF_FORMAT(fmt_idx, args_idx) \
     __attribute__((format(printf, fmt_idx, args_idx)))
#else
#  define X3F_PRINTF_FORMAT(fmt_idx, args_idx)
#endif

extern void x3f_printf(x3f_verbosity_t level, const char *fmt, ...)
  X3F_PRINTF_FORMAT(2, 3);

/* Optional log routing hook. If non-NULL, x3f_printf formats the message
 * into a 2 KiB stack buffer and forwards it here instead of writing to
 * stdout/stderr. Used by Rust consumers (mobile/WASM/library embeds) to
 * capture diagnostic output. Default NULL = legacy stdout/stderr behaviour.
 *
 * The callback receives the verbosity level and a NUL-terminated message
 * string with no level prefix and no trailing newline normalisation. */
typedef void (*x3f_printf_callback_t)(x3f_verbosity_t level, const char *msg);
extern x3f_printf_callback_t x3f_printf_callback;

#ifdef __cplusplus
}
#endif

#endif
