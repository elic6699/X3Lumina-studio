//! GPU 熔断：本进程内一旦发生 device lost / wgpu 内部 panic，就永久停用 GPU。
//!
//! 背景（2026-11-03 实测）：老显卡/老驱动（Quadro K1100M 的 Vulkan）跑到一半
//! device lost，此时 wgpu 的 `Device::poll` 在 **crate 内部**直接 panic：
//!
//! ```text
//! panicked at wgpu-27.0.1/src/backend/wgpu_core.rs:1826:
//! Error in Device::poll: Validation Error
//! Caused by: Parent device is lost
//! ```
//!
//! 这个 panic 跨过 `extern "C"` 边界就是 non-unwinding abort（`int 29h` →
//! 0xC0000409），宿主既拿不到返回值也留不下日志，进程直接死；更糟的是每张
//! 照片都会再死一次（"很多图都崩"）。
//!
//! 现在两处 GPU 上下文（`denoise::gpu` 的产品 NLM、`denoise::own_nlm3` 的
//! own 链）都在 `catch_unwind` 内派发：panic 或 device lost 会被截在 Rust
//! 内部，先置位熔断、记一条日志，再按既有约定返回错误。调用点（own_decoder.c
//! / x3f_render.c）拿到失败码后回落 CPU —— 用户只会看到"这一张慢一点"，
//! 而不是程序消失。

use std::sync::Mutex;

static POISON: Mutex<Option<String>> = Mutex::new(None);

/// 熔断原因；`None` 表示 GPU 仍可用。
pub(crate) fn gpu_poison_reason() -> Option<String> {
    POISON.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// 本进程是否已因 GPU 故障熔断。
pub fn gpu_poisoned() -> bool {
    POISON.lock().unwrap_or_else(|e| e.into_inner()).is_some()
}

/// 置位熔断（幂等：先入者为准），并落一行日志。
pub(crate) fn poison_gpu(reason: &str) {
    {
        let mut slot = POISON.lock().unwrap_or_else(|e| e.into_inner());
        if slot.is_some() {
            return;
        }
        *slot = Some(reason.to_string());
    }
    eprintln!("x3f gpu: GPU 已停用（本进程），后续走 CPU：{reason}");
    log_gpu_event(reason);
}

/// 落一行到 `<exe 目录>\cache\x3f-gpu.log`（与 panic hook 的 x3f-panic.log
/// 同目录）。这里的任何失败都静默 —— 记账路径本身不能再制造错误。
fn log_gpu_event(reason: &str) {
    use std::io::Write;
    let Ok(exe) = std::env::current_exe() else {
        return;
    };
    let Some(dir) = exe.parent() else {
        return;
    };
    let cache = dir.join("cache");
    let _ = std::fs::create_dir_all(&cache);
    let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(cache.join("x3f-gpu.log"))
    else {
        return;
    };
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let _ = writeln!(f, "[{secs}] GPU disabled for this process: {reason}");
}

/// 从 `catch_unwind` 的 payload 里取可读文案。
pub(crate) fn panic_text(payload: &(dyn std::any::Any + Send)) -> String {
    if let Some(s) = payload.downcast_ref::<&str>() {
        return (*s).to_string();
    }
    if let Some(s) = payload.downcast_ref::<String>() {
        return s.clone();
    }
    "non-string panic payload".to_string()
}

/// wgpu/驱动侧"设备已丢失"的文案判定（小写比较，兼容几种写法）。
pub(crate) fn is_device_lost(msg: &str) -> bool {
    let m = msg.to_ascii_lowercase();
    m.contains("device is lost") || m.contains("device lost") || m.contains("devicelost")
}
