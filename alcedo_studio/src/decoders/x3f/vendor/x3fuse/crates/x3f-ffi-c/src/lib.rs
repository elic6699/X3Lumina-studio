//! C ABI for the Rust x3f library.
//!
//! Surface is intentionally small in M8a: enough to satisfy the per-target
//! smoke test the PORT-PLAN calls for ("load file → metadata dump →
//! thumbnail extract"). Process-to-DNG/TIFF and other heavier surfaces will
//! follow once the iOS/Android/WASM build wiring is in place.
//!
//! Conventions
//! -----------
//! * **Handles** (`X3FReader *`) are opaque pointers owned by this
//!   library. Allocate with `x3f_reader_open`; free with
//!   `x3f_reader_close`. Mixing this library's allocator with the
//!   caller's malloc/free across handles is undefined.
//! * **Return codes**: 0 = success, non-zero = error. On error, callers
//!   may pull a human-readable message from `x3f_last_error()` (thread-
//!   local; cleared on the next successful call).
//! * **Strings**: every `*const c_char` is expected to be NUL-terminated
//!   UTF-8. Output strings owned by the library (`x3f_last_error`) live
//!   until the next call on the same thread.
//! * **Threading**: each `X3FReader` is single-threaded; callers must not
//!   share a handle across threads without external synchronization.
//!   Distinct handles are independent.

#![deny(unsafe_op_in_unsafe_fn)]

use std::cell::RefCell;
use std::ffi::{c_char, c_int, CStr, CString};
use std::path::Path;
use std::ptr;
use std::sync::Once;

// ---------------------------------------------------------------------------
// 崩溃定位：panic hook
//
// 背景（2026-11-03）：Quattro 文件走 fuse 解码时进程被 `int 29h` 直接掐死
// （0xC0000409 fast-fail）。反汇编确认那是 Rust 在 `extern "C"` 边界里 panic
// 之后走的 non-unwinding 出口 —— 它绕过进程内所有 C/C++ 钩子
// （SetUnhandledExceptionFilter / set_terminate / _set_abort_behavior 全无效），
// 宿主只能看到一个没有任何日志的空崩溃，连调用栈都要靠 WER 才能拿到。
//
// 这里在 fuse 入口装一个 panic hook：abort 之前先把「文件:行号 + 消息」写进
// 日志。宿主拿到的就不再是裸 RVA，而是确切的 panic 点 —— 不需要复现、不需要
// dump、不需要 PDB 配对。
//
// 输出位置：环境变量 `X3F_PANIC_LOG` 指定；否则 <exe目录>\cache\x3f-panic.log。
// ---------------------------------------------------------------------------

static PANIC_HOOK_ONCE: Once = Once::new();

/// panic 日志落到哪。优先环境变量，其次 exe 同级的 cache 目录（与 host.log 同处）。
fn panic_log_path() -> std::path::PathBuf {
    if let Some(p) = std::env::var_os("X3F_PANIC_LOG") {
        return std::path::PathBuf::from(p);
    }
    let dir = std::env::current_exe()
        .ok()
        .and_then(|e| e.parent().map(|p| p.to_path_buf()))
        .unwrap_or_else(|| std::path::PathBuf::from("."));
    dir.join("cache").join("x3f-panic.log")
}

/// 装 panic hook（幂等）。hook 内部只用最基础的 fs/io，避免自身再 panic。
fn install_panic_hook() {
    PANIC_HOOK_ONCE.call_once(|| {
        std::panic::set_hook(Box::new(|info| {
            use std::io::Write;

            let loc = info
                .location()
                .map(|l| format!("{}:{}:{}", l.file(), l.line(), l.column()))
                .unwrap_or_else(|| "<unknown>".to_string());
            let msg = if let Some(s) = info.payload().downcast_ref::<&str>() {
                (*s).to_string()
            } else if let Some(s) = info.payload().downcast_ref::<String>() {
                s.clone()
            } else {
                "<non-string payload>".to_string()
            };
            let tname = std::thread::current()
                .name()
                .unwrap_or("<unnamed>")
                .to_string();
            let text = format!("[x3f panic] thread={tname}\n  at {loc}\n  msg: {msg}\n");

            let path = panic_log_path();
            if let Some(parent) = path.parent() {
                let _ = std::fs::create_dir_all(parent);
            }
            if let Ok(mut f) = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&path)
            {
                let _ = f.write_all(text.as_bytes());
                let _ = f.flush();
            }
            // 同时给 stderr —— 命令行跑 x3f_extract 时能直接看到。
            let _ = std::io::stderr().write_all(text.as_bytes());
        }));
    });
}

use x3f_core::Reader;
use x3f_core::{
    prepare_scene_linear_wb, DenoiseAlgorithm, DenoiseBackend, DenoiseOptions, NlmOptions,
    SceneLinearOptions,
};

/// Opaque handle for an open X3F reader. Created by `x3f_reader_open`,
/// freed by `x3f_reader_close`. The empty-enum idiom guarantees the
/// type cannot be instantiated from C and forces cbindgen to emit a
/// forward declaration rather than a struct body.
pub enum X3FReader {}

thread_local! {
    static LAST_ERROR: RefCell<Option<CString>> = const { RefCell::new(None) };
}

fn set_last_error(msg: impl Into<String>) {
    let cstr = CString::new(msg.into()).unwrap_or_else(|_| {
        // Caller's message contained NULs; replace them with '?'.
        CString::new("error message contained NUL bytes").unwrap()
    });
    LAST_ERROR.with(|cell| *cell.borrow_mut() = Some(cstr));
}

fn clear_last_error() {
    LAST_ERROR.with(|cell| *cell.borrow_mut() = None);
}

/// Returns the last error message produced on this thread, or NULL if no
/// error has occurred since the last successful call. The pointer is owned
/// by the library and remains valid until the next call on the same thread
/// that produces or clears an error.
///
/// # Safety
///
/// The returned pointer must not be freed. Copy the string before making
/// further library calls if you need to retain it.
#[no_mangle]
pub extern "C" fn x3f_last_error() -> *const c_char {
    LAST_ERROR.with(|cell| match &*cell.borrow() {
        Some(s) => s.as_ptr(),
        None => ptr::null(),
    })
}

/// C 字符串 → `Option<String>`：NULL、空串、非 UTF-8 都当作"没给"。
///
/// # Safety
///
/// `p` 要么是 NULL，要么指向一个 NUL 结尾的字符串。
unsafe fn cstr_to_string(p: *const c_char) -> Option<String> {
    if p.is_null() {
        return None;
    }
    // SAFETY: caller guarantees NUL termination.
    let s = unsafe { CStr::from_ptr(p) }.to_str().ok()?;
    (!s.is_empty()).then(|| s.to_string())
}

/// Decode `path` as a UTF-8 NUL-terminated C string and turn it into a
/// `Path`. Sets last-error and returns `None` on failure.
///
/// # Safety
///
/// `path` must be a valid NUL-terminated string or NULL.
unsafe fn cstr_to_path<'a>(path: *const c_char) -> Option<&'a Path> {
    if path.is_null() {
        set_last_error("path argument was NULL");
        return None;
    }
    // SAFETY: caller guarantees path is NUL-terminated.
    let cstr = unsafe { CStr::from_ptr(path) };
    match cstr.to_str() {
        Ok(s) => Some(Path::new(s)),
        Err(_) => {
            set_last_error("path was not valid UTF-8");
            None
        }
    }
}

/// Opens and parses an X3F file's directory. Returns NULL on error; call
/// `x3f_last_error()` to retrieve the failure reason.
///
/// # Safety
///
/// `path` must be a NUL-terminated UTF-8 string or NULL.
#[no_mangle]
pub unsafe extern "C" fn x3f_reader_open(path: *const c_char) -> *mut X3FReader {
    // SAFETY: caller contract.
    let Some(p) = (unsafe { cstr_to_path(path) }) else {
        return ptr::null_mut();
    };
    match Reader::open(p) {
        Ok(r) => {
            clear_last_error();
            Box::into_raw(Box::new(r)).cast()
        }
        Err(e) => {
            set_last_error(e.to_string());
            ptr::null_mut()
        }
    }
}

/// Parses an X3F file from an in-memory byte buffer. Use this from
/// environments without a host filesystem (browser WASM, JNI, Swift
/// `Data`-bridged buffers). The bytes are *copied* internally; the
/// caller may free `data` immediately after this returns.
///
/// Returns NULL on parse failure or on an unsupported host (currently
/// Windows, where libc has no `fmemopen` equivalent — `x3f_reader_open`
/// is the right path there). Call `x3f_last_error()` for details.
///
/// # Safety
///
/// `data` must be valid for `len` bytes (or `len` may be 0 with a
/// NULL `data`, which always returns NULL).
#[no_mangle]
pub unsafe extern "C" fn x3f_reader_open_from_bytes(data: *const u8, len: usize) -> *mut X3FReader {
    #[cfg(any(unix, target_arch = "wasm32"))]
    {
        if data.is_null() || len == 0 {
            set_last_error("x3f_reader_open_from_bytes: data was NULL or len was 0");
            return ptr::null_mut();
        }
        // SAFETY: caller contract — data valid for `len` bytes.
        let bytes = unsafe { core::slice::from_raw_parts(data, len) };
        match Reader::from_bytes(bytes) {
            Ok(r) => {
                clear_last_error();
                Box::into_raw(Box::new(r)).cast()
            }
            Err(e) => {
                set_last_error(e.to_string());
                ptr::null_mut()
            }
        }
    }
    #[cfg(not(any(unix, target_arch = "wasm32")))]
    {
        let _ = (data, len);
        set_last_error(
            "x3f_reader_open_from_bytes: unsupported on this host (no fmemopen); use x3f_reader_open",
        );
        ptr::null_mut()
    }
}

/// Frees an `X3FReader` previously returned by `x3f_reader_open`. NULL is
/// a no-op (matches `free`).
///
/// # Safety
///
/// `handle` must be either NULL or a pointer previously returned by
/// `x3f_reader_open` and not yet closed.
#[no_mangle]
pub unsafe extern "C" fn x3f_reader_close(handle: *mut X3FReader) {
    if handle.is_null() {
        return;
    }
    // SAFETY: caller contract — handle was a `Box::into_raw` from
    // `x3f_reader_open` and has not been freed.
    let _ = unsafe { Box::from_raw(handle.cast::<Reader>()) };
}

/// Reads the X3F header version recorded at file open. Returns 0 if
/// `handle` is NULL.
///
/// # Safety
///
/// `handle` must be NULL or a live pointer from `x3f_reader_open`.
#[no_mangle]
pub unsafe extern "C" fn x3f_reader_header_version(handle: *const X3FReader) -> u32 {
    if handle.is_null() {
        return 0;
    }
    // SAFETY: caller contract.
    let r = unsafe { &*handle.cast::<Reader>() };
    r.header_version()
}

/// Loads CAMF + property list and writes a textual metadata dump to
/// `out_path`. Output format matches the legacy `x3f_extract -meta`.
/// Returns 0 on success, non-zero on error (call `x3f_last_error`).
///
/// # Safety
///
/// `handle` must be a live pointer from `x3f_reader_open`. `out_path`
/// must be a NUL-terminated UTF-8 string.
#[no_mangle]
pub unsafe extern "C" fn x3f_reader_dump_meta(
    handle: *mut X3FReader,
    out_path: *const c_char,
) -> c_int {
    if handle.is_null() {
        set_last_error("reader handle was NULL");
        return -1;
    }
    // SAFETY: caller contract.
    let r = unsafe { &mut *handle.cast::<Reader>() };
    // SAFETY: caller contract.
    let Some(p) = (unsafe { cstr_to_path(out_path) }) else {
        return -1;
    };

    if let Err(e) = r.load_camf() {
        set_last_error(format!("load_camf: {e}"));
        return -1;
    }
    if let Err(e) = r.load_property_list() {
        set_last_error(format!("load_property_list: {e}"));
        return -1;
    }
    if let Err(e) = r.dump_meta(p) {
        set_last_error(format!("dump_meta: {e}"));
        return -1;
    }
    clear_last_error();
    0
}

/// Loads and writes the embedded JPEG thumbnail to `out_path`. Returns 0
/// on success, non-zero on error.
///
/// # Safety
///
/// `handle` must be a live pointer from `x3f_reader_open`. `out_path`
/// must be a NUL-terminated UTF-8 string.
#[no_mangle]
pub unsafe extern "C" fn x3f_reader_dump_jpeg_thumbnail(
    handle: *mut X3FReader,
    out_path: *const c_char,
) -> c_int {
    if handle.is_null() {
        set_last_error("reader handle was NULL");
        return -1;
    }
    // SAFETY: caller contract.
    let r = unsafe { &mut *handle.cast::<Reader>() };
    // SAFETY: caller contract.
    let Some(p) = (unsafe { cstr_to_path(out_path) }) else {
        return -1;
    };

    if let Err(e) = r.load_thumbnail_jpeg() {
        set_last_error(format!("load_thumbnail_jpeg: {e}"));
        return -1;
    }
    if let Err(e) = r.dump_jpeg(p) {
        set_last_error(format!("dump_jpeg: {e}"));
        return -1;
    }
    clear_last_error();
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn last_error_is_null_initially() {
        assert!(x3f_last_error().is_null());
    }

    #[test]
    fn open_with_null_path_returns_null_and_sets_error() {
        let h = unsafe { x3f_reader_open(ptr::null()) };
        assert!(h.is_null());
        assert!(!x3f_last_error().is_null());
    }

    #[test]
    fn open_with_nonexistent_path_returns_null() {
        let cpath = CString::new("/tmp/definitely-does-not-exist.X3F").unwrap();
        let h = unsafe { x3f_reader_open(cpath.as_ptr()) };
        assert!(h.is_null());
        let err = x3f_last_error();
        assert!(!err.is_null());
        let msg = unsafe { CStr::from_ptr(err) }.to_str().unwrap();
        assert!(msg.contains("could not open input file"), "got: {msg}");
    }

    #[test]
    fn close_null_handle_is_safe() {
        // Mirrors the `free(NULL)` contract.
        unsafe { x3f_reader_close(ptr::null_mut()) };
    }

    #[test]
    fn header_version_with_null_handle_is_zero() {
        assert_eq!(unsafe { x3f_reader_header_version(ptr::null()) }, 0);
    }
}

// ---------------------------------------------------------------------------
// 场景线性输出：给宿主渲染层用的"和 DNG 同一套色彩变换"的入口。
//
// 为什么需要它：宿主自己拼矩阵（XYZ→sRGB · bmt_to_xyz · diag(as-shot 增益)）
// 会漏掉相机标定那一步，出来的颜色和同一个文件写出的 DNG 对不上——
// 表现为 DP Merrill 偏色、黄/红类高饱和色发闷。而 DNG 写出的色彩是由
// 相机元数据（ColorMatrix + AsShotNeutral）决定的，任何 RAW 引擎都照它算。
// 这个函数用的就是同一套 ColorCalibration，所以 JPEG/HEIC 走它就和 DNG 一致。
// ---------------------------------------------------------------------------

/// C 侧的降噪参数：x3fuse 0.2.0 的 `DenoiseOptions` + `DenoiseBackend`。
///
/// **布局必须与 `Sources/CX3FRaw/include/x3f_render.h` 里的 `x3f_denoise_t`
/// 逐字段一致**（那张头是手写的，不走 cbindgen）。字段顺序、类型都别改，
/// 要加就加在末尾并同步改 C 侧，否则是静默读错内存。
///
/// 用 `x3f_denoise_default()` 填过再用，别拿 `{0}` 初始化：
/// 0.0 的 strength 会被下面判成非法（正确范围 0.1~3.0）。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct x3f_denoise_t {
    /// 0 = NLM（默认），1 = BM3D。
    pub algorithm: c_int,
    /// 亮度通道要不要一起降（默认 0 = 只压色度）。
    pub luminance: c_int,
    /// 色度通道（默认 1）。
    pub chroma: c_int,
    /// 强度倍率，0.1 ~ 3.0（默认 1.0）。
    pub strength: f64,
    /// **本地新增**：亮度通道单独的倍率（只乘在亮度那一路上，色度不受影响）。
    /// 1.0 = 与不开这一项时完全相同。见 `DenoiseOptions::luma_scale`。
    pub luma_scale: f64,
    /// NLM 的主 pass 窗口：patch 1..=9、search 1..=31，都取奇数（默认 3/3/11/11）。
    /// 对 BM3D 无效（它是固定的 8x8 块）。
    pub luma_patch: c_int,
    pub chroma_patch: c_int,
    pub luma_search: c_int,
    pub chroma_search: c_int,
    /// 0 = auto（默认：能用 GPU 就用，失败退回 CPU），1 = cpu，2 = gpu（要不到就报错）。
    pub backend: c_int,
}

/// `x3f_denoise_t::algorithm` 的取值。
pub const X3F_DN_ALGO_NLM: c_int = 0;
pub const X3F_DN_ALGO_BM3D: c_int = 1;
/// `x3f_denoise_t::backend` 的取值。
pub const X3F_DN_BACKEND_AUTO: c_int = 0;
pub const X3F_DN_BACKEND_CPU: c_int = 1;
pub const X3F_DN_BACKEND_GPU: c_int = 2;

/// 把 C 侧那套结构翻成 Rust 的 [`DenoiseOptions`] + [`DenoiseBackend`]。
/// `dn` 为 NULL 时给默认那套（NLM / 只压色度 / 1.0 / 3,3,11,11 / auto）。
///
/// # Safety
///
/// `dn` 要么是 NULL，要么指向一个合法的 `x3f_denoise_t`。
unsafe fn denoise_from_c(
    dn: *const x3f_denoise_t,
) -> std::result::Result<(DenoiseOptions, DenoiseBackend), String> {
    // SAFETY: caller contract; `as_ref` handles NULL.
    let Some(d) = (unsafe { dn.as_ref() }) else {
        return Ok((DenoiseOptions::default(), DenoiseBackend::Auto));
    };
    let clamp = |v: c_int, lo: c_int, hi: c_int| v.clamp(lo, hi) as u8;
    let options = DenoiseOptions {
        algorithm: match d.algorithm {
            X3F_DN_ALGO_BM3D => DenoiseAlgorithm::Bm3d,
            _ => DenoiseAlgorithm::Nlm,
        },
        luminance: d.luminance != 0,
        chroma: d.chroma != 0,
        strength: d.strength as f32,
        luma_scale: d.luma_scale as f32,
        nlm: NlmOptions {
            luma_patch: clamp(d.luma_patch, 1, 9),
            chroma_patch: clamp(d.chroma_patch, 1, 9),
            luma_search: clamp(d.luma_search, 1, 31),
            chroma_search: clamp(d.chroma_search, 1, 31),
        },
    };
    options.validate().map_err(|e| e.to_string())?;
    let backend = match d.backend {
        X3F_DN_BACKEND_CPU => DenoiseBackend::Cpu,
        X3F_DN_BACKEND_GPU => DenoiseBackend::Gpu,
        _ => DenoiseBackend::Auto,
    };
    Ok((options, backend))
}

/// 解成**扩展线性 sRGB**（f32，通道交错）。相机标定、白平衡（AsShotNeutral）、
/// 相对数字 ISO 增益都已按相机元数据算好，只差显示用的色调曲线；
/// active area 已裁、**未**按 EXIF 转正（`out_orientation` 交给渲染方应用一次）。
///
/// 成功返回 0。`*out_data` 由 libc::malloc 分配，用 [`x3f_scene_linear_free`]
/// 释放（或直接 free，同一个分配器）。失败时返回非 0，可读 [`x3f_last_error`]。
///
/// `denoise_intensity` 取 0..=10（0 = 完全不降噪；不确定就传 0）。
/// `denoise` 给算法/通道/强度倍率/窗口/后端，NULL = 默认那套（NLM、只压色度、
/// 倍率 1.0、3/3/11/11 窗、后端 auto）。它只在 `denoise_intensity != 0` 时有意义。
///
/// # Safety
///
/// `path` 必须是 NUL 结尾的 UTF-8 字符串；`out_data` / `out_width` /
/// `out_height` 必须可写；`out_orientation` 可为 NULL；
/// `denoise` 可为 NULL，否则指向一个合法的 `x3f_denoise_t`。
#[no_mangle]
pub unsafe extern "C" fn x3f_scene_linear(
    path: *const c_char,
    out_data: *mut *mut f32,
    out_width: *mut u32,
    out_height: *mut u32,
    out_orientation: *mut u16,
    out_camera_to_rgb: *mut f64,
    highlight_recovery: c_int,
    denoise_intensity: u8,
    denoise: *const x3f_denoise_t,
    wb_preset: *const c_char,
) -> c_int {
    // 解码崩溃（Quattro/own 回落等）的现场靠这一个 hook 落盘。
    install_panic_hook();
    // SAFETY: caller contract for `path`.
    let Some(p) = (unsafe { cstr_to_path(path) }) else {
        return 1;
    };
    if out_data.is_null() || out_width.is_null() || out_height.is_null() {
        set_last_error("x3f_scene_linear: null output pointer");
        return 1;
    }
    // SAFETY: caller contract for `denoise`.
    let (denoise_options, denoise_backend) = match unsafe { denoise_from_c(denoise) } {
        Ok(pair) => pair,
        Err(e) => {
            set_last_error(format!("x3f_scene_linear: {e}"));
            return 1;
        }
    };
    let options = SceneLinearOptions {
        denoise_intensity: denoise_intensity.min(10),
        denoise_backend,
        denoise_options,
        // x3fuse 自带的高光重建（chroma LUT / RepairPix / 矩阵病态门），
        // DNG 写出用的就是这一套。**在 BMT（传感器）域**重建被压住的层，
        // 正是显示域补不出来的那部分信息。默认关，这里由调用方决定。
        highlight_recovery: highlight_recovery != 0,
        ..SceneLinearOptions::default()
    };
    // 相机自己的白平衡预设名（NULL/空 = 用文件里记录的那个）。它同时决定
    // 用哪一份"每光源色彩校正矩阵"，所以是相机标定而不是我们自己拍的通道
    // 增益。见 `prepare_scene_linear_wb`。
    let wb = unsafe { cstr_to_string(wb_preset) };
    let cancel = std::sync::atomic::AtomicBool::new(false);
    match prepare_scene_linear_wb(p, &options, wb.as_deref(), &cancel, |_| {}) {
        Ok(img) => {
            let count = img.data.len();
            // SAFETY: malloc so the C side can free it with the same allocator.
            let buf = unsafe { libc::malloc(count * std::mem::size_of::<f32>()) }.cast::<f32>();
            if buf.is_null() {
                set_last_error("x3f_scene_linear: out of memory");
                return 1;
            }
            // SAFETY: buf holds `count` f32; img.data has exactly `count` elements.
            unsafe {
                std::ptr::copy_nonoverlapping(img.data.as_ptr(), buf, count);
                *out_data = buf;
                *out_width = img.width;
                *out_height = img.height;
                if !out_orientation.is_null() {
                    *out_orientation = img.orientation;
                }
                // 相机 BMT(逐通道饱和比) → 线性 sRGB 的 3x3。宿主拿它**求逆**就能
                // 把每个像素的逐通道饱和比反解出来 —— 那是判断"这个通道有没有
                // 顶到传感器上限"的原生量，比事后在 sRGB 上猜阈值准得多。
                if !out_camera_to_rgb.is_null() {
                    std::ptr::copy_nonoverlapping(
                        img.white_balance.camera_to_rgb.as_ptr(),
                        out_camera_to_rgb,
                        9,
                    );
                }
            }
            clear_last_error();
            0
        }
        Err(e) => {
            set_last_error(e.to_string());
            1
        }
    }
}

/// 释放 [`x3f_scene_linear`] 返回的缓冲。NULL 安全。
///
/// # Safety
///
/// `ptr` 必须是 `x3f_scene_linear` 返回的指针且只释放一次。
#[no_mangle]
pub unsafe extern "C" fn x3f_scene_linear_free(ptr: *mut f32) {
    if !ptr.is_null() {
        // SAFETY: caller contract; the buffer came from libc::malloc above.
        unsafe { libc::free(ptr.cast()) };
    }
}

/// GPU 降噪能不能用：能用返回 1 并把适配器名（`"Apple M1 (Metal)"`）写进
/// `out_name`（NUL 结尾，最多 `cap` 字节；可传 NULL / 0 只要那个 1/0）。
/// 不能（没编 GPU 后端、或拿不到适配器）返回 0。
///
/// 为什么需要它：后端 `auto` 在拿不到适配器时会**静默退回 CPU**，宿主的
/// 导出照样成功，光看成功/失败分不出来。而全分辨率导出的降噪成本在这台
/// 机器上差 4 倍以上，界面有义务把这件事说清楚。
///
/// # Safety
///
/// `out_name` 要么是 NULL，要么指向至少 `cap` 字节的可写缓冲。
#[no_mangle]
pub unsafe extern "C" fn x3f_denoise_gpu_probe(out_name: *mut c_char, cap: c_int) -> c_int {
    match x3f_core::gpu_adapter_name() {
        Ok(name) => {
            if !out_name.is_null() && cap > 1 {
                let bytes = name.as_bytes();
                let n = bytes.len().min(cap as usize - 1);
                // SAFETY: caller guarantees `cap` writable bytes at `out_name`.
                unsafe {
                    std::ptr::copy_nonoverlapping(bytes.as_ptr().cast::<c_char>(), out_name, n);
                    *out_name.add(n) = 0;
                }
            }
            1
        }
        Err(_) => 0,
    }
}

// ---------------------------------------------------------------------------
// 相机标定（CAMF）取数
//
// 渲染层要复刻 SPP 的降噪，就得拿相机自己的噪声标定：
//   ColorNoiseModel       σ² 的 (a,b,c)，逐通道
//   ISONoiseSettings      逐 ISO 的 12 个噪声参数
//   DenoisingSlidersStep  5 档滑块对应的乘子（见 SPP_降噪三滑块_反编译.md）
// 这些都在 CAMF 段里，而 CAMF 是 Huffman 压缩的 —— 解压只有这一层能做，
// 所以在这里开一个按名字取矩阵的口子，C 侧不用自己解析。
// ---------------------------------------------------------------------------

/// 按名字取 CAMF 里的一个 float 矩阵，拷成 f64 写进 `out`。
///
/// 返回**元素个数**（0 = 文件打开失败 / 没有这个名字 / 类型不是 float）。
/// `dims`（长度 3，可传 NULL）拿到三维尺寸，缺的维按 1 填。
/// `cap` 是 `out` 能放下的元素个数；实际拷贝 `min(count, cap)` 个。
///
/// # Safety
///
/// `path` / `name` 必须是 NUL 结尾的 UTF-8；`out` 至少 `cap` 个 f64；
/// `dims` 可为 NULL，否则要能放下 3 个 u32。
#[no_mangle]
pub unsafe extern "C" fn x3f_camf_matrix_f64(
    path: *const c_char,
    name: *const c_char,
    out: *mut f64,
    cap: c_int,
    dims: *mut u32,
) -> c_int {
    install_panic_hook();
    clear_last_error();
    if path.is_null() || name.is_null() || cap <= 0 {
        set_last_error("x3f_camf_matrix_f64: bad argument");
        return 0;
    }

    // SAFETY: caller contract for `path`.
    let file = unsafe { libc::fopen(path, c"rb".as_ptr()) };
    if file.is_null() {
        set_last_error("x3f_camf_matrix_f64: cannot open file");
        return 0;
    }
    // SAFETY: `file` is a valid open FILE*.
    let x3f = match unsafe { x3f_sys::new_from_file(file.cast(), x3f_sys::Control::none()) } {
        Ok(p) => p,
        Err(e) => {
            unsafe { libc::fclose(file) };
            set_last_error(format!("x3f_camf_matrix_f64: {e}"));
            return 0;
        }
    };

    // CAMF 是独立的一段，得先找目录项再让库把数据读进来（解压在这里发生）。
    let mut count: c_int = 0;
    let stage;
    // SAFETY: `x3f` is valid until x3f_delete below.
    unsafe {
        let de = x3f_sys::x3f_get_camf(x3f);
        if de.is_null() {
            stage = "x3f_get_camf returned null";
        } else if let Err(e) = x3f_sys::load_data(x3f, de, x3f_sys::Control::none()) {
            stage = "load_data failed";
            set_last_error(format!("x3f_camf_matrix_f64: load_data: {e}"));
        } else {
            stage = "matrix not found";
            let mut d0: c_int = 0;
            let mut d1: c_int = 0;
            // 注意：`x3f_get_camf_matrix_var` 对 2 维矩阵要求 **dim2 必须为 NULL**，
            // 传一个非空指针会被判成参数错误直接返回 0（这里踩过一次）。
            // 1 维矩阵更严：dim1 也必须是 NULL。所以对维数不敏感地试两次。
            let mut ptr: *mut std::ffi::c_void = ptr::null_mut();
            let mut n = x3f_sys::x3f_get_camf_matrix_var(
                x3f,
                name as *mut c_char,
                &mut d0,
                &mut d1,
                ptr::null_mut::<c_int>(),
                x3f_sys::matrix_type_t_M_FLOAT,
                &mut ptr,
            );
            if n == 0 {
                // 试 1 维：dim1 传 NULL
                d1 = 1;
                ptr = ptr::null_mut();
                n = x3f_sys::x3f_get_camf_matrix_var(
                    x3f,
                    name as *mut c_char,
                    &mut d0,
                    ptr::null_mut::<c_int>(),
                    ptr::null_mut::<c_int>(),
                    x3f_sys::matrix_type_t_M_FLOAT,
                    &mut ptr,
                );
            }
            // 注意：`x3f_get_camf_matrix_var` 成功时返回的是 **1（布尔）**，不是元素个数；
            // 元素个数得自己用维数乘出来（这里也踩过一次）。
            if n > 0 && !ptr.is_null() {
                let n = (d0.max(1) as usize) * (d1.max(1) as usize);
                if !out.is_null() {
                    let take = n.min(cap as usize);
                    std::ptr::copy_nonoverlapping(ptr.cast::<f64>(), out, take);
                    count = take as c_int;
                } else {
                    count = n as c_int;
                }
                if !dims.is_null() {
                    // 调用方必须给 dims 留 **3** 个 u32 的位置：除了 (行, 列)
                    // 还会写一个恒为 1 的第三项。少给一个就是越界写 —— 2026-10-15
                    // 在 Windows 那条链上实测：C 侧只给 int d[2] 时正好压到栈保护
                    // 金丝雀，函数返回即 abort（x3f_render.c 里的 X3F_CAMF_DIMS）。
                    *dims.add(0) = d0.max(0) as u32;
                    *dims.add(1) = d1.max(0) as u32;
                    *dims.add(2) = 1;
                }
            }
        }
    }

    // SAFETY: `x3f` was created by new_from_file; `file` is still open.
    unsafe {
        x3f_sys::x3f_delete(x3f);
        libc::fclose(file);
    }
    if count == 0 {
        if LAST_ERROR.with(|c| c.borrow().is_none()) {
            set_last_error(format!("x3f_camf_matrix_f64: {stage}"));
        }
    }
    count
}

// ── own 后端（X3FCore own_decoder.c）的 GPU nlm3 ─────────────────────────
// own_decoder.c 的 nlm3_den 有现成的 GPU 优先分支（own_gpu_nlm3），Mac 走
// Metal（own_gpu.m）；Windows 经这两个入口接到 x3f-sys 的 wgpu 实现
// （denoise::own_nlm3，int64 定点语义与 own 的 CPU 路径逐位一致）。
// 权重表由 C 侧构造好后**原样上传**，Rust 侧不重算。

/// 适配器探针：1 = 有非软件适配器，0 = 没有（宿主据此关掉 own 的 GPU 分支）。
/// 本进程已熔断（device lost / wgpu panic）时恒返 0，宿主随即整体回落 CPU。
#[cfg(all(feature = "gpu", not(target_arch = "wasm32")))]
#[no_mangle]
pub extern "C" fn x3f_own_gpu_probe() -> std::ffi::c_int {
    install_panic_hook();
    x3f_sys::own_nlm3::gpu_probe()
}

/// GPU 熔断状态：1 = 本进程内已因 device lost / wgpu panic 停用 GPU。
/// C 侧 `own_gpu_available()` 每次问它，避免探针结果被缓存后还在走 GPU。
#[cfg(all(feature = "gpu", not(target_arch = "wasm32")))]
#[no_mangle]
pub extern "C" fn x3f_own_gpu_poisoned() -> std::ffi::c_int {
    x3f_sys::own_nlm3::gpu_poisoned() as std::ffi::c_int
}

/// own 的 nlm3（u16 交错 3ch，int32 定点权重表）。成功返 0，任何失败返 -1，
/// 调用点自动回退 CPU 行分块。
#[cfg(all(feature = "gpu", not(target_arch = "wasm32")))]
#[no_mangle]
pub extern "C" fn x3f_own_gpu_nlm3(
    src3: *const u16,
    dst3: *mut u16,
    w: std::ffi::c_int,
    h: std::ffi::c_int,
    lut: *const i32,
    lut_n: std::ffi::c_int,
    twin: std::ffi::c_int,
    swin: std::ffi::c_int,
    bin_shift: std::ffi::c_int,
) -> std::ffi::c_int {
    x3f_sys::own_nlm3::gpu_nlm3(src3, dst3, w, h, lut, lut_n, twin, swin, bin_shift)
}

/// own 的解码后色度降噪（own_decoder.c nr_job 的 GPU 版）：cr/cb float 平面
/// 做 iters 次 3×3 中值，结果写回 crA/cbA（与 CPU 路径语义一致）。成功返 0，
/// 任何失败返 -1，调用点自动回退 CPU。
#[cfg(all(feature = "gpu", not(target_arch = "wasm32")))]
#[no_mangle]
pub extern "C" fn x3f_own_gpu_chroma_nr(
    cr_a: *mut f32,
    cb_a: *mut f32,
    cr_b: *mut f32,
    cb_b: *mut f32,
    w: std::ffi::c_int,
    h: std::ffi::c_int,
    iters: std::ffi::c_int,
) -> std::ffi::c_int {
    x3f_sys::own_nlm3::gpu_chroma_nr(cr_a, cb_a, cr_b, cb_b, w, h, iters)
}

/// guided_up_2x 的 GPU 版（own_decoder.c）：半分辨率 src/t2 → 全分辨率
/// dst，dtab/wv 为 C 侧预计算核表。成功返 0，失败返 -1（回退 CPU）。
#[cfg(all(feature = "gpu", not(target_arch = "wasm32")))]
#[no_mangle]
pub extern "C" fn x3f_own_gpu_guided_up_2x(
    src: *const f32,
    t2: *const f32,
    guide: *const f32,
    dst: *mut f32,
    hw: std::ffi::c_int,
    hh: std::ffi::c_int,
    dtab: *const f32,
    wv: *const f32,
    wv_n: std::ffi::c_int,
) -> std::ffi::c_int {
    x3f_sys::own_nlm3::gpu_guided_up_2x(src, t2, guide, dst, hw, hh, dtab, wv, wv_n)
}

/// bilateral5 的 GPU 版（own_decoder.c）：两 pass 可分离双边滤波，
/// 结果写回 img；rlut/sw 为 C 侧预计算表。成功返 0，失败返 -1。
#[cfg(all(feature = "gpu", not(target_arch = "wasm32")))]
#[no_mangle]
pub extern "C" fn x3f_own_gpu_bilateral5(
    img: *mut f32,
    w: std::ffi::c_int,
    h: std::ffi::c_int,
    rlut: *const f32,
    rlut_n: std::ffi::c_int,
    sw: *const f32,
) -> std::ffi::c_int {
    x3f_sys::own_nlm3::gpu_bilateral5(img, w, h, rlut, rlut_n, sw)
}

/// 色度中值单平面窗口中值（x3f_render.c apply_chroma_median 的 GPU 步骤）：
/// pl/omed 各 w*h f32，窗口 (2r+1)²（r 1..7），与 CPU chrmed_select 值精确
/// 一致（第 k 小语义，算法无关）。成功返 0，失败返 -1。
#[cfg(all(feature = "gpu", not(target_arch = "wasm32")))]
#[no_mangle]
pub extern "C" fn x3f_own_gpu_chrmed_plane(
    pl: *const f32,
    omed: *mut f32,
    w: std::ffi::c_int,
    h: std::ffi::c_int,
    r: std::ffi::c_int,
) -> std::ffi::c_int {
    x3f_sys::own_nlm3::gpu_chrmed_plane(pl, omed, w, h, r)
}

// ── 渲染层 apply 链 GPU 化（2026-10-27，对齐 Mac own_gpu.m）────────────────
// 四个入口对应 own_gpu_win.c 里原先恒 -1 的桩：apply_lut16 / chrmed_u16 /
// down8（x3f_render.c 预览 apply 链）+ stage1_bilat（x3f_stage1.c）。全部
// 预览专用（X3F_GPU_APPLY gate），失败返 -1 由 C 侧回退 CPU 对应段。

#[cfg(all(feature = "gpu", not(target_arch = "wasm32")))]
#[no_mangle]
pub extern "C" fn x3f_own_gpu_apply_lut16(
    lin: *const f32,
    clip: *const f32,
    lut: *const u16,
    lutg: *const u16,
    lutb: *const u16,
    gain: *const f32,
    cm3: *const f32,
    cmcb: *const f32,
    cmcr: *const f32,
    sat: f32,
    use_sat: std::ffi::c_int,
    hl_desat: f32,
    desat_lo: f32,
    desat_hi: f32,
    out: *mut u16,
    w: std::ffi::c_int,
    h: std::ffi::c_int,
) -> std::ffi::c_int {
    x3f_sys::own_nlm3::gpu_apply_lut16(
        lin, clip, lut, lutg, lutb, gain, cm3, cmcb, cmcr, sat, use_sat,
        hl_desat, desat_lo, desat_hi, out, w, h,
    )
}

#[cfg(all(feature = "gpu", not(target_arch = "wasm32")))]
#[no_mangle]
pub extern "C" fn x3f_own_gpu_chrmed_u16(
    buf: *mut u16,
    w: std::ffi::c_int,
    h: std::ffi::c_int,
    stride: std::ffi::c_int,
    r: std::ffi::c_int,
) -> std::ffi::c_int {
    x3f_sys::own_nlm3::gpu_chrmed_u16(buf, w, h, stride, r)
}

#[cfg(all(feature = "gpu", not(target_arch = "wasm32")))]
#[no_mangle]
pub extern "C" fn x3f_own_gpu_down8(
    tmp: *const u16,
    out: *mut u8,
    w: std::ffi::c_int,
    h: std::ffi::c_int,
    stride: std::ffi::c_int,
    dither: std::ffi::c_int,
) -> std::ffi::c_int {
    x3f_sys::own_nlm3::gpu_down8(tmp, out, w, h, stride, dither)
}

#[cfg(all(feature = "gpu", not(target_arch = "wasm32")))]
#[no_mangle]
pub extern "C" fn x3f_own_gpu_stage1_bilat(
    src: *const f32,
    dst: *mut f32,
    w: std::ffi::c_int,
    h: std::ffi::c_int,
    r: std::ffi::c_int,
    coef: *const f64,
    lin_sat: f64,
    level_scale: f64,
    nscale: f64,
) -> std::ffi::c_int {
    x3f_sys::own_nlm3::gpu_stage1_bilat(src, dst, w, h, r, coef, lin_sat, level_scale, nscale)
}
