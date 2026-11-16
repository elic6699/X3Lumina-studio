# SPDX-License-Identifier: GPL-3.0-only
#
# X3F / X3I (Sigma Foveon) decode core for Alcedo Studio.
#
# Source provenance (see repository NOTICE and THIRD_PARTY_NOTICE.txt):
#   core/CX3F, core/CX3FRaw  - X3Lumina X3FCore tree, contributed under GPL-3.0-only.
#   core/CX3FLegacy          - x3f_tools 0.57, BSD-3-Clause (COPYRIGHT_x3f_tools.txt).
#   vendor/x3fuse            - sagwaco/x3fuse, Apache-2.0 (LICENSE / NOTICE).
#
# This file is self-contained: it can be configured on its own for a quick
# compile/smoke check without Qt, or added with add_subdirectory() by the
# Alcedo build. Keep the target names prefixed with alcedo_x3f_ so they do
# not collide with the LibRaw targets of the host project.

cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(alcedo_x3f LANGUAGES C CXX)
  set(ALCEDO_X3F_TOP_LEVEL TRUE)
else()
  enable_language(C CXX)
  set(ALCEDO_X3F_TOP_LEVEL FALSE)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ALCEDO_X3F_BUILD_TOOLS "Build x3fprobe / sfdbench smoke tools" ${ALCEDO_X3F_TOP_LEVEL})
option(ALCEDO_X3F_GPU "Build x3fuse with the wgpu GPU denoise path" OFF)
option(ALCEDO_X3F_CARGO_OFFLINE "Pass --offline to cargo" OFF)

set(X3F_DIR          "${CMAKE_CURRENT_SOURCE_DIR}")
set(X3F_CORE_INC     "${X3F_DIR}/core/CX3F/include")
set(X3F_LEGACY_INC   "${X3F_DIR}/core/CX3FLegacy/include")
set(X3F_RAW_INC      "${X3F_DIR}/core/CX3FRaw/include")
set(X3F_CARGO_DIR    "${X3F_DIR}/vendor/x3fuse")
set(X3F_CARGO_TARGET "${CMAKE_BINARY_DIR}/x3f-cargo")
set(X3F_RUST_STATIC  "${X3F_CARGO_TARGET}/release/libx3f.a")

set(X3F_CORE_SOURCES
    core/CX3F/x3fcore.c)

# The legacy decoder is compiled with every global symbol renamed to legacy_*
# by x3f_legacy_prefix.h, so it can coexist with the Rust x3fuse C ABI.
set(X3F_LEGACY_SOURCES
    core/CX3FLegacy/x3f_io.c
    core/CX3FLegacy/x3f_meta.c
    core/CX3FLegacy/x3f_process.c
    core/CX3FLegacy/x3f_image.c
    core/CX3FLegacy/x3f_matrix.c
    core/CX3FLegacy/x3f_histogram.c
    core/CX3FLegacy/x3f_spatial_gain.c
    core/CX3FLegacy/x3f_quattro_c.c
    core/CX3FLegacy/x3f_dump.c
    core/CX3FLegacy/x3f_printf.c)

set(X3F_RAW_SOURCES
    core/CX3FRaw/x3fdec.c
    core/CX3FRaw/x3f_render.c
    core/CX3FRaw/x3f_rcps.c
    core/CX3FRaw/x3f_tonecurve.c
    core/CX3FRaw/x3f_mac.c
    core/CX3FRaw/x3f_sfd.c
    core/CX3FRaw/x3fraw.c
    core/CX3FRaw/own_decoder.c
    core/CX3FRaw/x3f_stage1.c
    core/CX3FRaw/x3f_dodrx.c
    core/CX3FRaw/x3f_oec3.c
    core/CX3FRaw/x3f_hole_fix.c
    core/CX3FRaw/x3f_packed_assets.c
    core/CX3FRaw/own_gpu_win.c
    core/CX3FRaw/x3f_alcedo_export.c
    core/CX3FRaw/x3f_alcedo_foveon.c)

if(NOT MSVC)
  add_compile_options(-Wall -Wno-unused-parameter -Wno-unused-variable)
else()
  add_compile_options(/utf-8)
endif()

# ---------------------------------------------------------------------------
# Rust static library (x3fuse). LTO must stay off: thin LTO internalises the
# #[no_mangle] C ABI symbols that the C render layer calls into.
# ---------------------------------------------------------------------------
find_program(ALCEDO_X3F_CARGO NAMES cargo REQUIRED)

if(ALCEDO_X3F_GPU)
  set(X3F_RUST_FEATURES --features gpu)
else()
  set(X3F_RUST_FEATURES --no-default-features)
endif()
if(ALCEDO_X3F_CARGO_OFFLINE)
  set(X3F_RUST_OFFLINE --offline)
endif()

add_custom_command(
  OUTPUT "${X3F_RUST_STATIC}"
  COMMAND "${CMAKE_COMMAND}" -E env "CARGO_TARGET_DIR=${X3F_CARGO_TARGET}"
          "${ALCEDO_X3F_CARGO}" build --release -p x3f-ffi-c
          ${X3F_RUST_FEATURES} ${X3F_RUST_OFFLINE}
          --config "profile.release.lto=false"
  WORKING_DIRECTORY "${X3F_CARGO_DIR}"
  COMMENT "cargo build -p x3f-ffi-c (x3fuse decode layer, LTO off)"
  VERBATIM)
add_custom_target(alcedo_x3f_rust DEPENDS "${X3F_RUST_STATIC}")

# ---------------------------------------------------------------------------
# C compilation units
# ---------------------------------------------------------------------------
add_library(alcedo_x3f_core_obj OBJECT ${X3F_CORE_SOURCES})
target_include_directories(alcedo_x3f_core_obj PRIVATE "${X3F_CORE_INC}")

add_library(alcedo_x3f_legacy_obj OBJECT ${X3F_LEGACY_SOURCES})
target_include_directories(alcedo_x3f_legacy_obj PRIVATE "${X3F_LEGACY_INC}")
if(MSVC)
  target_compile_options(alcedo_x3f_legacy_obj PRIVATE
      "/FI${X3F_LEGACY_INC}/x3f_legacy_prefix.h")
else()
  target_compile_options(alcedo_x3f_legacy_obj PRIVATE
      "-include" "${X3F_LEGACY_INC}/x3f_legacy_prefix.h")
endif()

# Do NOT add X3F_LEGACY_INC here: both trees contain x3f_io.h et al. and the
# include order would decide which ABI the render layer sees.
add_library(alcedo_x3f_raw_obj OBJECT ${X3F_RAW_SOURCES})
target_include_directories(alcedo_x3f_raw_obj PRIVATE "${X3F_RAW_INC}" "${X3F_CORE_INC}")

foreach(t alcedo_x3f_core_obj alcedo_x3f_legacy_obj alcedo_x3f_raw_obj)
  target_compile_definitions(${t} PRIVATE NDEBUG)
endforeach()
target_compile_definitions(alcedo_x3f_legacy_obj PRIVATE VERSION="\"0.57-legacy\"")
target_compile_definitions(alcedo_x3f_raw_obj PRIVATE VERSION="\"0.57-port\"")
if(ALCEDO_X3F_GPU)
  target_compile_definitions(alcedo_x3f_raw_obj PRIVATE X3F_HAVE_RUST_GPU)
endif()
if(MSVC)
  foreach(t alcedo_x3f_core_obj alcedo_x3f_legacy_obj alcedo_x3f_raw_obj)
    target_compile_definitions(${t} PRIVATE
        _CRT_SECURE_NO_WARNINGS _CRT_NONSTDC_NO_DEPRECATE)
  endforeach()
endif()

# ---------------------------------------------------------------------------
# Public static libraries consumed by the C++ decoder bridge
# ---------------------------------------------------------------------------
add_library(alcedo_x3fcore STATIC $<TARGET_OBJECTS:alcedo_x3f_core_obj>)
target_include_directories(alcedo_x3fcore PUBLIC "${X3F_CORE_INC}")

add_library(alcedo_x3fraw STATIC
    $<TARGET_OBJECTS:alcedo_x3f_legacy_obj>
    $<TARGET_OBJECTS:alcedo_x3f_raw_obj>)
target_include_directories(alcedo_x3fraw PUBLIC "${X3F_RAW_INC}" "${X3F_CORE_INC}")
# own_decoder.c writes PNG previews with zlib (compress2 / compressBound).
find_package(ZLIB REQUIRED)
target_link_libraries(alcedo_x3fraw PUBLIC
    alcedo_x3fcore "${X3F_RUST_STATIC}" ZLIB::ZLIB)
add_dependencies(alcedo_x3fraw alcedo_x3f_rust)

if(WIN32)
  target_link_libraries(alcedo_x3fraw PUBLIC
      advapi32 bcrypt ntdll userenv ws2_32 ole32 oleaut32)
elseif(APPLE)
  target_link_libraries(alcedo_x3fraw PUBLIC
      "-framework Metal" "-framework QuartzCore" "-framework CoreGraphics"
      "-framework Foundation" iconv)
else()
  target_link_libraries(alcedo_x3fraw PUBLIC dl m pthread)
endif()

# ---------------------------------------------------------------------------
# C++ support layer shared by import, preview and export paths.
# Header-only contract lives in the host project's public include root.
# ---------------------------------------------------------------------------
# ALCEDO_INCLUDE_ROOT exists only when the Alcedo tree owns this subdirectory.
# A standalone smoke project can also add_subdirectory() this tree, so decide
# by the variable rather than by ALCEDO_X3F_TOP_LEVEL.
if(DEFINED ALCEDO_INCLUDE_ROOT AND NOT "${ALCEDO_INCLUDE_ROOT}" STREQUAL "")
  set(X3F_HOST_INCLUDE_ROOT "${ALCEDO_INCLUDE_ROOT}")
else()
  set(X3F_HOST_INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../include")
endif()

add_library(alcedo_x3f_support STATIC
    x3f_support.cpp
    x3f_foveon.cpp)
target_include_directories(alcedo_x3f_support PUBLIC
    "${X3F_HOST_INCLUDE_ROOT}" "${X3F_CORE_INC}")
# The C++ support layer now also owns the scene-linear decode bridge (colour
# matrix probe + in-memory frame decode), so it needs the full decode core.
target_link_libraries(alcedo_x3f_support PUBLIC alcedo_x3fcore alcedo_x3fraw)
target_compile_features(alcedo_x3f_support PUBLIC cxx_std_20)
set_target_properties(alcedo_x3f_support PROPERTIES
    AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)

# ---------------------------------------------------------------------------
# Smoke tools (only when this directory is the top-level project)
# ---------------------------------------------------------------------------
if(ALCEDO_X3F_BUILD_TOOLS)
  add_executable(x3fprobe tools/x3fprobe.c)
  target_link_libraries(x3fprobe PRIVATE alcedo_x3fraw)
  add_executable(sfdbench tools/sfdbench.c)
  target_link_libraries(sfdbench PRIVATE alcedo_x3fraw)

  # C++ path-B bridge smoke: defaults, two-stage caching, decode-key
  # invalidation and the 16-bit sRGB output contract.
  add_executable(foveon_bridge_smoke tools/foveon_bridge_smoke.cpp)
  target_link_libraries(foveon_bridge_smoke PRIVATE alcedo_x3f_support)
endif()

message(STATUS "alcedo_x3f: GPU=${ALCEDO_X3F_GPU} cargo=${ALCEDO_X3F_CARGO} "
               "rust=${X3F_RUST_STATIC}")
