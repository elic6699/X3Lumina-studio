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

/* 上游把这个枚举常量叫 DEBUG，但构建系统在 Debug 配置下会自己在命令行
 * 定义 DEBUG 宏（Xcode 的 GCC_PREPROCESSOR_DEFINITIONS=DEBUG=1、
 * SwiftPM 给 C target 的 -DDEBUG）。宏一旦存在，下面的枚举就会被替换成
 * "typedef enum {ERR=0, WARN=1, INFO=2, 1=3}"，直接编译失败：
 *     error: expected identifier
 *     error: duplicate case value: 'WARN' and '1' both equal '1'
 * 这个代码库里没有任何地方用 #ifdef DEBUG 做条件编译，所以这里把它撤掉，
 * 让 DEBUG 老老实实当枚举常量用。 */
#ifdef DEBUG
#undef DEBUG
#endif

typedef enum {ERR=0, WARN=1, INFO=2, DEBUG=3} x3f_verbosity_t;

extern x3f_verbosity_t x3f_printf_level;

/* 同 CX3FRaw/include/x3f_printf.h 里的写法：`__attribute__((format(printf, ...)))`
 * 是 GCC/Clang 扩展，让编译器检查可变参数；MSVC 的 cl.exe 不认它，会直接报
 * "语法错误: 标识符__attribute__"。它只是诊断提示、不属于 ABI，非 GNU 编译器
 * 上展开成空即可。用 #ifndef 保护：两份 x3f_printf.h 万一同时被包含也不会重定义。 */
#ifndef X3F_PRINTF_FORMAT
#  if defined(__GNUC__) || defined(__clang__)
#    define X3F_PRINTF_FORMAT(fmt_idx, args_idx) \
       __attribute__((format(printf, fmt_idx, args_idx)))
#  else
#    define X3F_PRINTF_FORMAT(fmt_idx, args_idx)
#  endif
#endif

extern void x3f_printf(x3f_verbosity_t level, const char *fmt, ...)
  X3F_PRINTF_FORMAT(2, 3);

#ifdef __cplusplus
}
#endif

#endif
