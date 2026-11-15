#!/bin/sh
# 生成 x3f_legacy_prefix.h：把旧 C 解码器的**所有全局符号**加上 legacy_ 前缀。
#
# 为什么需要：新的解码层是 x3fuse（Rust 静态库），它把同一套 x3f_tools C API
# 也以 #[no_mangle] extern "C" 导出了（实测 71 个名字与旧 C 版撞车）。
# 两个实现要同时链进一个二进制，就必须给其中一方改名。
#
# 这里给旧的那一方改名，方式是编译时 -include x3f_legacy_prefix.h，
# 宏展开同时作用于定义与引用，所以旧解码器内部照旧互相调用，只是外部名字变了。
# 头文件里的关键字由「把旧源编成 .o 后 nm -g」机械生成，不手抄，
# 免得以后上游加了函数忘了补。
#
# 用法（在 CX3FLegacy 目录下）：
#   ./gen_prefix.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

cd "$HERE"
for f in *.c; do
    clang -O0 -c -Iinclude -o "$TMP/${f%.c}.o" "$f"
done

nm -g "$TMP"/*.o | awk '$2 ~ /^[TDBSC]$/ {print $3}' | sed 's/^_//' | sort -u > "$TMP/syms"

{
    echo "/* 自动生成，勿手改 —— 见 gen_prefix.sh */"
    echo "#ifndef X3F_LEGACY_PREFIX_H"
    echo "#define X3F_LEGACY_PREFIX_H"
    while read -r s; do
        case "$s" in
            _*|"") continue ;;   # 跳过编译器内部的 __x 之类
        esac
        echo "#define $s legacy_$s"
    done < "$TMP/syms"
    echo "#endif"
} > include/x3f_legacy_prefix.h

echo "写了 $(wc -l < include/x3f_legacy_prefix.h) 行到 include/x3f_legacy_prefix.h"
