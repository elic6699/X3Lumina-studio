//! own 后端（X3FCore own_decoder.c 的 `nlm3_den`）专用的 GPU nlm3。
//!
//! 与本目录 `gpu.rs` 的产品 NLM 是两套语义，不能混用：
//!   · 权重表由 C 侧按 own 的定点公式构造（int32，fixed_point_mult 钳到
//!     INT32_MAX），这里**原样上传**，不在 Rust 侧重算；
//!   · kernel 内 int64 定点累加 + CPU 同式舍入（见 own_nlm3.wgsl），与 own
//!     的 CPU 路径**逐位一致** —— 宿主 X3FDEC_NLMDBG=1 会自动逐位比对。
//!
//! 对外入口（x3f-ffi-c 加 `#[no_mangle]` 包装成 C ABI）：
//!   `gpu_probe()` —— 适配器探针，喂给 own_gpu_available()；
//!   `gpu_nlm3(..)` —— 单次整幅派发（不分块），成功返 0，任何失败返 -1，
//!   调用点（own_decoder.c）自动回退 CPU 行分块。
//!
//! 并发：fuse 的两次 NLM 可能来自并发的解码线程（预览 + 导出），CTX 这把
//! 互斥锁把整幅调用串行化 —— CPU 路径本来就是可重入的，串行化只影响吞吐
//! 不影响正确性；wgpu 的 device/queue 本身线程安全。

use std::collections::HashMap;
use std::sync::{mpsc, Mutex};
use std::time::{Duration, Instant};

type Shared = Mutex<Option<std::result::Result<Ctx, String>>>;
static CTX: Shared = Mutex::new(None);

// GPU 熔断与记账都在 gpu_fault（产品 NLM 与 own 链共用同一份状态）。
use super::gpu_fault::{gpu_poison_reason, is_device_lost, panic_text, poison_gpu};
// 供 x3f-ffi-c 询问"本进程 GPU 是否已熔断"（own_gpu_available 每次都要问）。
pub use super::gpu_fault::gpu_poisoned;

struct Ctx {
    device: wgpu::Device,
    queue: wgpu::Queue,
    // 按窗口参数 (sr, tr) 缓存的 pipeline —— shader 里的 TS/SR/TR 是模板注入
    // 的编译期常量（tile 数组按实际窗口取大小 + 常量界循环），每个窗口组合
    // 只编一次。调用被 CTX 串行化，HashMap 无并发访问。
    pipelines: Mutex<HashMap<(u32, u32), wgpu::ComputePipeline>>,
    // chroma_nr 的 pipeline（窗口固定 3×3，单实例就够）
    chroma_pipeline: Mutex<Option<wgpu::ComputePipeline>>,
    // guided_up / bilateral5 的 pipeline（单实例）
    gu_pipeline: Mutex<Option<wgpu::ComputePipeline>>,
    bh_pipeline: Mutex<Option<wgpu::ComputePipeline>>,
    bv_pipeline: Mutex<Option<wgpu::ComputePipeline>>,
    // 预览 apply 链（own_gpu.h 2026-10-23 新增）的 GPU 管线
    apply_lut_pipeline: Mutex<Option<wgpu::ComputePipeline>>,
    chrmed_prep_pipeline: Mutex<Option<wgpu::ComputePipeline>>,
    chrmed_emit_pipeline: Mutex<Option<wgpu::ComputePipeline>>,
    down8_pipeline: Mutex<Option<wgpu::ComputePipeline>>,
    s1bilat_pipeline: Mutex<Option<wgpu::ComputePipeline>>,
    failure: std::sync::Arc<Mutex<Option<String>>>,
}

impl Ctx {
    fn check(&self) -> std::result::Result<(), String> {
        match self.failure.lock().unwrap_or_else(|e| e.into_inner()).as_ref() {
            Some(e) => Err(e.clone()),
            None => Ok(()),
        }
    }

    fn pipeline_for(
        &self,
        sr: u32,
        tr: u32,
    ) -> std::result::Result<wgpu::ComputePipeline, String> {
        if let Some(p) = self
            .pipelines
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&(sr, tr))
        {
            return Ok(p.clone());
        }
        let ts = 16 + 2 * (sr + tr);
        // own_nlm3.wgsl 的三个占位按实际窗口注入（`__TS__u` → `28u`）。
        let source = include_str!("own_nlm3.wgsl")
            .replace("__TS__", &ts.to_string())
            .replace("__SR__", &sr.to_string())
            .replace("__TR__", &tr.to_string());
        let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("x3f own nlm3"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
        self.check()?;
        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("x3f own nlm3"),
                layout: None,
                module: &shader,
                entry_point: Some("nlm3"),
                compilation_options: Default::default(),
                cache: None,
            });
        self.check()?;
        self.pipelines
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert((sr, tr), pipeline.clone());
        Ok(pipeline)
    }

    fn chroma_pipeline_for(&self) -> std::result::Result<wgpu::ComputePipeline, String> {
        if let Some(p) = self.chroma_pipeline.lock().unwrap_or_else(|e| e.into_inner()).as_ref() {
            return Ok(p.clone());
        }
        let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("x3f own chroma_nr"),
            source: wgpu::ShaderSource::Wgsl(include_str!("own_chroma_nr.wgsl").into()),
        });
        self.check()?;
        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("x3f own chroma_nr"),
                layout: None,
                module: &shader,
                entry_point: Some("chroma_nr"),
                compilation_options: Default::default(),
                cache: None,
            });
        self.check()?;
        *self.chroma_pipeline.lock().unwrap_or_else(|e| e.into_inner()) = Some(pipeline.clone());
        Ok(pipeline)
    }

    fn apply_lut_pipeline_for(&self) -> std::result::Result<wgpu::ComputePipeline, String> {
        if let Some(p) = self
            .apply_lut_pipeline
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
        {
            return Ok(p.clone());
        }
        let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("x3f own apply_lut"),
            source: wgpu::ShaderSource::Wgsl(include_str!("own_apply_lut.wgsl").into()),
        });
        self.check()?;
        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("x3f own apply_lut"),
                layout: None,
                module: &shader,
                entry_point: Some("apply_lut"),
                compilation_options: Default::default(),
                cache: None,
            });
        self.check()?;
        *self
            .apply_lut_pipeline
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(pipeline.clone());
        Ok(pipeline)
    }

    fn chrmed_prep_pipeline_for(&self) -> std::result::Result<wgpu::ComputePipeline, String> {
        if let Some(p) = self
            .chrmed_prep_pipeline
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
        {
            return Ok(p.clone());
        }
        let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("x3f own chrmed_prep"),
            source: wgpu::ShaderSource::Wgsl(include_str!("own_chrmed_u16.wgsl").into()),
        });
        self.check()?;
        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("x3f own chrmed_prep"),
                layout: None,
                module: &shader,
                entry_point: Some("chrmed_prep"),
                compilation_options: Default::default(),
                cache: None,
            });
        self.check()?;
        *self
            .chrmed_prep_pipeline
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(pipeline.clone());
        Ok(pipeline)
    }

    fn chrmed_emit_pipeline_for(&self) -> std::result::Result<wgpu::ComputePipeline, String> {
        if let Some(p) = self
            .chrmed_emit_pipeline
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
        {
            return Ok(p.clone());
        }
        let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("x3f own chrmed_emit"),
            source: wgpu::ShaderSource::Wgsl(include_str!("own_chrmed_u16.wgsl").into()),
        });
        self.check()?;
        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("x3f own chrmed_emit"),
                layout: None,
                module: &shader,
                entry_point: Some("chrmed_emit"),
                compilation_options: Default::default(),
                cache: None,
            });
        self.check()?;
        *self
            .chrmed_emit_pipeline
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(pipeline.clone());
        Ok(pipeline)
    }

    fn down8_pipeline_for(&self) -> std::result::Result<wgpu::ComputePipeline, String> {
        if let Some(p) = self
            .down8_pipeline
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
        {
            return Ok(p.clone());
        }
        let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("x3f own down8"),
            source: wgpu::ShaderSource::Wgsl(include_str!("own_down8.wgsl").into()),
        });
        self.check()?;
        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("x3f own down8"),
                layout: None,
                module: &shader,
                entry_point: Some("down8k"),
                compilation_options: Default::default(),
                cache: None,
            });
        self.check()?;
        *self
            .down8_pipeline
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(pipeline.clone());
        Ok(pipeline)
    }

    fn s1bilat_pipeline_for(&self) -> std::result::Result<wgpu::ComputePipeline, String> {
        if let Some(p) = self
            .s1bilat_pipeline
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
        {
            return Ok(p.clone());
        }
        let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("x3f own s1bilat"),
            source: wgpu::ShaderSource::Wgsl(include_str!("own_stage1_bilat.wgsl").into()),
        });
        self.check()?;
        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("x3f own s1bilat"),
                layout: None,
                module: &shader,
                entry_point: Some("s1bilat"),
                compilation_options: Default::default(),
                cache: None,
            });
        self.check()?;
        *self
            .s1bilat_pipeline
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(pipeline.clone());
        Ok(pipeline)
    }
}

#[cfg(feature = "gpu")]
struct StderrLogger;
#[cfg(feature = "gpu")]
impl log::Log for StderrLogger {
    fn enabled(&self, meta: &log::Metadata) -> bool { meta.level() <= log::Level::Error }
    fn log(&self, record: &log::Record) {
        if self.enabled(record.metadata()) {
            eprintln!("[wgpu-log] {}: {}", record.level(), record.args());
        }
    }
    fn flush(&self) {}
}

fn ctx_init() -> std::result::Result<Ctx, String> {
    // 挂最小 stderr logger：wgpu 验证错误的完整细节走 log crate（uncaptured
    // error 的 Display 在部分路径只有泛化 "Validation Error"），不挂日志
    // 就丢失根因。仅 Error 级，一次安装。
    #[cfg(feature = "gpu")]
    {
        static LOG_INSTALL: std::sync::Once = std::sync::Once::new();
        LOG_INSTALL.call_once(|| {
            let _ = log::set_logger(&StderrLogger);
            log::set_max_level(log::LevelFilter::Error);
        });
    }
    // X3FDEC_GPUBACKEND=vulkan|dx12|metal|all：诊断用后端强制（默认全开）。
    let backends = gpu_backends();
    let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
        backends,
        ..Default::default()
    });
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::HighPerformance,
        force_fallback_adapter: false,
        compatible_surface: None,
    }))
    .map_err(|e| e.to_string())?;
    if adapter.get_info().device_type == wgpu::DeviceType::Cpu {
        return Err("adapter is a software renderer".into());
    }
    // 限制拉到适配器上限：own 的调用是一次整幅派发（不走分块），源缓冲
    // ≈ 像素×6 字节，Quattro 全幅 ~140MB，wgpu 默认 128MiB 绑定上限不够。
    // 适配器上限也不够时由调用处显式检查、回退 CPU。
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("x3f own nlm3"),
        required_limits: adapter.limits(),
        ..Default::default()
    }))
    .map_err(|e| e.to_string())?;
    // 诊断：适配器上限一次性打到 stderr——apply_lut 有 10 个 storage 绑定，
    // 若 Vulkan 设备 max_storage_buffers_per_shader_stage < 10，管线创建会
    // 异步验证失败 → 输出全零（此前 max=65535 的疑似根因，用此行实证）。
    {
        let info = adapter.get_info();
        let lim = device.limits();
        eprintln!(
            "own_gpu: ctx adapter [{}] {:?} max_storage_buffers_per_shader_stage={} max_bindings_per_bind_group={} max_storage_buffer_binding_size={} max_buffer_size={}",
            info.name,
            info.backend,
            lim.max_storage_buffers_per_shader_stage,
            lim.max_bindings_per_bind_group,
            lim.max_storage_buffer_binding_size,
            lim.max_buffer_size
        );
    }
    // 不挂这个，wgpu 的 uncaptured error 会直接 panic（在解码线程里 abort 宿主）。
    // 挂上后错误进 failure 槽，后续调用经 check 失败 → own 回退 CPU。
    let failure = std::sync::Arc::new(Mutex::new(None::<String>));
    let errors = failure.clone();
    device.on_uncaptured_error(std::sync::Arc::new(move |error: wgpu::Error| {
        *errors
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(error.to_string());
    }));
    Ok(Ctx {
        device,
        queue,
        pipelines: Mutex::new(HashMap::new()),
        chroma_pipeline: Mutex::new(None),
        gu_pipeline: Mutex::new(None),
        bh_pipeline: Mutex::new(None),
        bv_pipeline: Mutex::new(None),
        apply_lut_pipeline: Mutex::new(None),
        chrmed_prep_pipeline: Mutex::new(None),
        chrmed_emit_pipeline: Mutex::new(None),
        down8_pipeline: Mutex::new(None),
        s1bilat_pipeline: Mutex::new(None),
        failure,
    })
}

fn with_ctx<T>(
    run: impl FnOnce(&Ctx) -> std::result::Result<T, String>,
) -> std::result::Result<T, String> {
    if let Some(reason) = gpu_poison_reason() {
        return Err(format!("GPU disabled after earlier failure: {reason}"));
    }
    let mut guard = CTX.lock().unwrap_or_else(|e| e.into_inner());
    let ctx = guard
        .get_or_insert_with(ctx_init)
        .as_ref()
        .map_err(|e| e.clone())?;
    // wgpu 的 device lost 会在 crate 内部 panic（Device::poll），跨 FFI 就是
    // non-unwinding abort。截在这里：置熔断 + 返 Err，由调用点回落 CPU。
    let outcome = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        ctx.check()?;
        run(ctx)
    })) {
        Ok(result) => result,
        Err(payload) => {
            let reason = format!("wgpu panic: {}", panic_text(&*payload));
            poison_gpu(&reason);
            Err(reason)
        }
    };
    // 走 Err 返回路径的 device lost（poll 报错而不是 panic）同样熔断：
    // 设备已经丢了，再试多少次都是同一个结果，没必要让每张图都慢一遍。
    if let Err(ref e) = outcome {
        if is_device_lost(e) {
            poison_gpu(e);
        }
    }
    outcome
}

/// wgpu 后端集合。`X3FDEC_GPUBACKEND=vulkan|dx12|metal|all` 诊断用强制；
/// 默认三种全开，由 wgpu 自己挑。探针与建上下文共用同一份判定，避免
/// "探针说有 GPU、真建出来却是另一套后端"。
fn gpu_backends() -> wgpu::Backends {
    match std::env::var("X3FDEC_GPUBACKEND").as_deref() {
        Ok("vulkan") => wgpu::Backends::VULKAN,
        Ok("dx12") => wgpu::Backends::DX12,
        Ok("metal") => wgpu::Backends::METAL,
        _ => wgpu::Backends::METAL | wgpu::Backends::VULKAN | wgpu::Backends::DX12,
    }
}

/// 适配器探针（进程内缓存；探测要建 wgpu Instance 并请求适配器，几十毫秒）。
/// 1 = 有非软件适配器；0 = 没编 GPU 后端 / 拿不到适配器 / 本进程已熔断。
pub fn gpu_probe() -> i32 {
    if gpu_poisoned() {
        return 0;
    }
    static OK: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let ok = *OK.get_or_init(|| {
        let r = (|| -> std::result::Result<(), String> {
            let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
                backends: gpu_backends(),
                ..Default::default()
            });
            let adapter = pollster::block_on(instance.request_adapter(
                &wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::HighPerformance,
                    force_fallback_adapter: false,
                    compatible_surface: None,
                },
            ))
            .map_err(|e| e.to_string())?;
            if adapter.get_info().device_type == wgpu::DeviceType::Cpu {
                return Err("adapter is a software renderer".into());
            }
            Ok(())
        })();
        r.is_ok()
    });
    ok as i32
}

fn dbg_enabled() -> bool {
    std::env::var("X3FDEC_GPUDBG").is_ok()
}

/// own 的 nlm3：src3/dst3 交错 3ch u16（w*h*3 个），lut 为 C 侧构造的
/// int32 定点权重表（lut_n*3 个，非负）。twin/swin 奇数窗全宽。
/// 成功返 0；窗口超限 / 缓冲超设备上限 / 派发或回读失败返 -1。
pub fn gpu_nlm3(
    src3: *const u16,
    dst3: *mut u16,
    w: i32,
    h: i32,
    lut: *const i32,
    lut_n: i32,
    twin: i32,
    swin: i32,
    bin_shift: i32,
) -> i32 {
    let t_start = Instant::now();
    let r = gpu_nlm3_impl(src3, dst3, w, h, lut, lut_n, twin, swin, bin_shift);
    if std::env::var("X3FDEC_TIME").is_ok() {
        let tag = match r {
            Ok(()) => "gpu",
            Err(_) => "GPU-FAIL->cpu",
        };
        eprintln!(
            "TIME   own nlm3 {}x{} {}: {:.3} s",
            w,
            h,
            tag,
            t_start.elapsed().as_secs_f64()
        );
    }
    match r {
        Ok(()) => 0,
        Err(e) => {
            if dbg_enabled() {
                eprintln!("own_gpu: nlm3 GPU 失败：{e}（回退 CPU）");
            }
            -1
        }
    }
}

fn gpu_nlm3_impl(
    src3: *const u16,
    dst3: *mut u16,
    w: i32,
    h: i32,
    lut: *const i32,
    lut_n: i32,
    twin: i32,
    swin: i32,
    bin_shift: i32,
) -> std::result::Result<(), String> {
    if w <= 0 || h <= 0 || lut_n <= 0 {
        return Err("bad dimensions".into());
    }
    let tr = (twin / 2).max(0) as u32;
    let sr = (swin / 2).max(0) as u32;
    let ext = sr + tr;
    if ext > 12 {
        return Err(format!("window too large (sr+tr={ext}, 上限 12)"));
    }
    let w = w as u32;
    let h = h as u32;
    let len_u16 = (w * h * 3) as usize;

    // SAFETY: 调用方（own_decoder.c nlm3_den）保证 src3 可读 len_u16 个 u16、
    // lut 可读 lut_n*3 个 i32、dst3 可写 len_u16 个 u16。
    let src_bytes: &[u8] =
        unsafe { std::slice::from_raw_parts(src3 as *const u8, len_u16 * 2) };
    let lut_slice: &[i32] =
        unsafe { std::slice::from_raw_parts(lut, lut_n as usize * 3) };
    let out_words = (len_u16 + 1) / 2;

    with_ctx(|ctx| {
        let lim = ctx.device.limits();
        let src_size = (out_words * 4) as u64;
        let dst_size = (w * h) as u64 * 8;
        let lut_size = (lut_words_len(lut_n) * 4) as u64;
        let cap = lim
            .max_buffer_size
            .min(u64::from(lim.max_storage_buffer_binding_size));
        if src_size > cap || dst_size > cap {
            return Err(format!(
                "buffer {src_size}/{dst_size} bytes over device limit {cap}"
            ));
        }

        // 源按 u32 重读上传（总 u16 数为奇数时末尾补一个零）
        let mut src_words = vec![0u32; out_words];
        let src_copy: &mut [u8] = bytemuck::cast_slice_mut(&mut src_words);
        src_copy[..len_u16 * 2].copy_from_slice(src_bytes);
        // LUT：int32 → u32（权重非负；负值意味着 C 侧算错了，钳 0 与 CPU 同败法）
        let lut_words: Vec<u32> = lut_slice.iter().map(|&v| v as i64 as u32).collect();
        let probe = std::env::var("X3FDEC_NLMPROBE")
            .ok()
            .and_then(|v| v.parse::<u32>().ok())
            .unwrap_or(0);
        let params: [u32; 8] = [
            w,
            h,
            sr,
            tr,
            bin_shift.max(0) as u32,
            lut_n as u32,
            probe,
            0,
        ];

        let src_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("own nlm3 src"),
            size: src_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let dst_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("own nlm3 dst"),
            size: dst_size,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let readback = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("own nlm3 readback"),
            size: dst_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let lut_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("own nlm3 lut"),
            size: lut_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let params_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("own nlm3 params"),
            size: 32,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let pipeline = ctx.pipeline_for(sr, tr)?;
        let bind = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("own nlm3 bind"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: src_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: dst_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: lut_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: params_buf.as_entire_binding(),
                },
            ],
        });

        ctx.queue
            .write_buffer(&src_buf, 0, bytemuck::cast_slice(&src_words));
        ctx.queue
            .write_buffer(&lut_buf, 0, bytemuck::cast_slice(&lut_words));
        ctx.queue
            .write_buffer(&params_buf, 0, bytemuck::cast_slice(&params));

        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups(w.div_ceil(16), h.div_ceil(16), 1);
        }
        encoder.copy_buffer_to_buffer(&dst_buf, 0, &readback, 0, dst_size);
        ctx.queue.submit([encoder.finish()]);

        let slice = readback.slice(..dst_size);
        let (send, recv) = mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = send.send(result);
        });
        let started = Instant::now();
        let mut map_result: Result<(), String> = Ok(());
        loop {
            match ctx.device.poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: Some(Duration::from_millis(50)),
            }) {
                Ok(_) | Err(wgpu::PollError::Timeout) => {}
                Err(e) => {
                    map_result = Err(e.to_string());
                    break;
                }
            }
            match recv.try_recv() {
                Ok(Ok(())) => break,
                Ok(Err(e)) => {
                    map_result = Err(e.to_string());
                    break;
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    map_result = Err("readback disconnected".into());
                    break;
                }
                Err(mpsc::TryRecvError::Empty) => {}
            }
            if started.elapsed() > Duration::from_secs(120) {
                map_result = Err("GPU readback timed out".into());
                break;
            }
        }
        let result = map_result;
        if result.is_ok() {
            let mapped = slice.get_mapped_range();
            // SAFETY: dst3 可写 len_u16 个 u16（见上）。
            let out: &mut [u16] =
                unsafe { std::slice::from_raw_parts_mut(dst3, len_u16) };
            // 回读解包（19.6M 像素的单线程循环要 ~0.2s）按核数切块并行。
            let pixels: &[[u32; 2]] = bytemuck::cast_slice(&mapped);
            let px_len = pixels.len();
            let threads = std::thread::available_parallelism()
                .map(|n| n.get())
                .unwrap_or(4)
                .min(16);
            let chunk = pixels.len().div_ceil(threads);
            std::thread::scope(|s| {
                for (t, out_chunk) in out.chunks_mut(chunk * 3).enumerate() {
                    let start = (t * chunk).min(px_len);
                    let end = (start + out_chunk.len() / 3).min(px_len);
                    let px = &pixels[start..end];
                    s.spawn(move || {
                        for (i, p) in px.iter().enumerate() {
                            out_chunk[i * 3] = (p[0] & 0xFFFF) as u16;
                            out_chunk[i * 3 + 1] = (p[0] >> 16) as u16;
                            out_chunk[i * 3 + 2] = p[1] as u16;
                        }
                    });
                }
            });
            drop(mapped);
        }
        readback.unmap();
        result
    })
}

fn lut_words_len(lut_n: i32) -> usize {
    lut_n as usize * 3
}

/// own 的解码后色度降噪（nr_job 的 GPU 版）：cr/cb 两块 float 平面做
/// iters 次 3×3 中值（逐位一致，见 own_chroma_nr.wgsl）。
/// crA/cbA 是输入，结果**写回 crA/cbA**（与 CPU 路径的 A/B 交换语义等价，
/// own_decoder.c 的 rec_job 从 crA/cbA 读）；crB/cbB 只当宿主的 CPU 暂存，
/// 这里不碰。成功返 0，任何失败返 -1（调用点回退 CPU）。
pub fn gpu_chroma_nr(
    cr_a: *mut f32,
    cb_a: *mut f32,
    _cr_b: *mut f32,
    _cb_b: *mut f32,
    w: i32,
    h: i32,
    iters: i32,
) -> i32 {
    let t_start = Instant::now();
    let r = gpu_chroma_nr_impl(cr_a, cb_a, w, h, iters);
    if std::env::var("X3FDEC_TIME").is_ok() {
        let tag = match r {
            Ok(()) => "gpu",
            Err(_) => "GPU-FAIL->cpu",
        };
        eprintln!(
            "TIME   own chroma_nr {}x{} iters={} {}: {:.3} s",
            w,
            h,
            iters,
            tag,
            t_start.elapsed().as_secs_f64()
        );
    }
    match r {
        Ok(()) => 0,
        Err(e) => {
            if dbg_enabled() {
                eprintln!("own_gpu: chroma_nr GPU 失败：{e}（回退 CPU）");
            }
            -1
        }
    }
}

fn gpu_chroma_nr_impl(
    cr_a: *mut f32,
    cb_a: *mut f32,
    w: i32,
    h: i32,
    iters: i32,
) -> std::result::Result<(), String> {
    if w <= 0 || h <= 0 || iters < 0 {
        return Err("bad dimensions".into());
    }
    let w = w as u32;
    let h = h as u32;
    let n = (w as usize) * (h as usize);

    // SAFETY: 调用方（own_decoder.c）保证 crA/cbA 各可读可写 n 个 f32。
    let cr: &[f32] = unsafe { std::slice::from_raw_parts(cr_a, n) };
    let cb: &[f32] = unsafe { std::slice::from_raw_parts(cb_a, n) };

    with_ctx(|ctx| {
        let pipeline = ctx.chroma_pipeline_for()?;
        let plane_size = (n * 8) as u64; // vec2<f32> 每像素 8 字节

        // 上传：cr/cb 交错成 vec2<f32>（memcpy 级开销，单线程即可）
        let mut host = vec![0f32; n * 2];
        for i in 0..n {
            host[i * 2] = cr[i];
            host[i * 2 + 1] = cb[i];
        }

        let lim = ctx.device.limits();
        let cap = lim
            .max_buffer_size
            .min(u64::from(lim.max_storage_buffer_binding_size));
        if plane_size > cap {
            return Err(format!("chroma_nr buffer {plane_size} bytes over device limit {cap}"));
        }

        let mk = |label: &str, usage: wgpu::BufferUsages| {
            ctx.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: plane_size,
                usage,
                mapped_at_creation: false,
            })
        };
        let buf1 = mk(
            "own chroma_nr buf1",
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
        );
        let buf2 = mk(
            "own chroma_nr buf2",
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
        );
        let readback = mk("own chroma_nr readback", wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST);
        let params_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("own chroma_nr params"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let params: [u32; 4] = [w, h, 0, 0];
        ctx.queue.write_buffer(&params_buf, 0, bytemuck::cast_slice(&params));

        let layout = pipeline.get_bind_group_layout(0);
        let mk_bind = |src: &wgpu::Buffer, dst: &wgpu::Buffer| {
            ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("own chroma_nr bind"),
                layout: &layout,
                entries: &[
                    wgpu::BindGroupEntry { binding: 0, resource: src.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 1, resource: dst.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 2, resource: params_buf.as_entire_binding() },
                ],
            })
        };
        let bg12 = mk_bind(&buf1, &buf2);
        let bg21 = mk_bind(&buf2, &buf1);

        ctx.queue.write_buffer(&buf1, 0, bytemuck::cast_slice(&host));
        drop(host);

        // iters 次中值，每次单独一个 compute pass（保证上一轮写完再读）
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        for it in 0..iters {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&pipeline);
            if it % 2 == 0 {
                pass.set_bind_group(0, &bg12, &[]);
            } else {
                pass.set_bind_group(0, &bg21, &[]);
            }
            pass.dispatch_workgroups(w.div_ceil(16), h.div_ceil(16), 1);
        }
        // iters 为偶数（含 0）时结果在 buf1，奇数在 buf2
        let final_buf = if iters % 2 == 0 { &buf1 } else { &buf2 };
        encoder.copy_buffer_to_buffer(final_buf, 0, &readback, 0, plane_size);
        ctx.queue.submit([encoder.finish()]);

        let slice = readback.slice(..plane_size);
        let (send, recv) = mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |res| {
            let _ = send.send(res);
        });
        let started = Instant::now();
        let mut map_result: Result<(), String> = Ok(());
        loop {
            match ctx.device.poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: Some(Duration::from_millis(50)),
            }) {
                Ok(_) | Err(wgpu::PollError::Timeout) => {}
                Err(e) => {
                    map_result = Err(e.to_string());
                    break;
                }
            }
            match recv.try_recv() {
                Ok(Ok(())) => break,
                Ok(Err(e)) => {
                    map_result = Err(e.to_string());
                    break;
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    map_result = Err("readback disconnected".into());
                    break;
                }
                Err(mpsc::TryRecvError::Empty) => {}
            }
            if started.elapsed() > Duration::from_secs(120) {
                map_result = Err("GPU readback timed out".into());
                break;
            }
        }
        let result = map_result;
        if result.is_ok() {
            // SAFETY: crA/cbA 各可写 n 个 f32（见上）。
            let out_cr: &mut [f32] = unsafe { std::slice::from_raw_parts_mut(cr_a, n) };
            let out_cb: &mut [f32] = unsafe { std::slice::from_raw_parts_mut(cb_a, n) };
            let mapped = slice.get_mapped_range();
            let vals: &[f32] = bytemuck::cast_slice(&mapped);
            // 解交错按块并行（19.6M 像素单线程 ~0.1s）
            let threads = std::thread::available_parallelism()
                .map(|x| x.get())
                .unwrap_or(4)
                .min(16);
            let chunk = n.div_ceil(threads);
            std::thread::scope(|s| {
                for (t, (oc, ob)) in out_cr
                    .chunks_mut(chunk)
                    .zip(out_cb.chunks_mut(chunk))
                    .enumerate()
                {
                    let i0 = t * chunk;
                    let i1 = i0 + oc.len();
                    let v = &vals[i0 * 2..i1 * 2];
                    s.spawn(move || {
                        for i in 0..oc.len() {
                            oc[i] = v[i * 2];
                            ob[i] = v[i * 2 + 1];
                        }
                    });
                }
            });
            drop(mapped);
        }
        readback.unmap();
        result
    })
}

// ---------- guided_up_2x / bilateral5（2026-10-24 GPU 化） ----------

impl Ctx {
    fn gu_pipeline_for(&self) -> std::result::Result<wgpu::ComputePipeline, String> {
        if let Some(p) = self.gu_pipeline.lock().unwrap_or_else(|e| e.into_inner()).as_ref() {
            return Ok(p.clone());
        }
        let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("x3f own guided_up"),
            source: wgpu::ShaderSource::Wgsl(include_str!("own_guided_up.wgsl").into()),
        });
        self.check()?;
        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("x3f own guided_up"),
                layout: None,
                module: &shader,
                entry_point: Some("guided_up"),
                compilation_options: Default::default(),
                cache: None,
            });
        self.check()?;
        *self.gu_pipeline.lock().unwrap_or_else(|e| e.into_inner()) = Some(pipeline.clone());
        Ok(pipeline)
    }

    fn bilat_pipelines_for(
        &self,
    ) -> std::result::Result<(wgpu::ComputePipeline, wgpu::ComputePipeline), String> {
        {
            let h = self.bh_pipeline.lock().unwrap_or_else(|e| e.into_inner());
            if let (Some(hp), Some(vp)) = (h.as_ref(), self.bv_pipeline.lock().unwrap_or_else(|e| e.into_inner()).as_ref()) {
                return Ok((hp.clone(), vp.clone()));
            }
        }
        let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("x3f own bilateral5"),
            source: wgpu::ShaderSource::Wgsl(include_str!("own_bilateral5.wgsl").into()),
        });
        self.check()?;
        let mk = |entry: &str, label: &str| {
            self.device
                .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                    label: Some(label),
                    layout: None,
                    module: &shader,
                    entry_point: Some(entry),
                    compilation_options: Default::default(),
                    cache: None,
                })
        };
        let hp = mk("bilat_h", "x3f own bilateral5 h");
        self.check()?;
        let vp = mk("bilat_v", "x3f own bilateral5 v");
        self.check()?;
        *self.bh_pipeline.lock().unwrap_or_else(|e| e.into_inner()) = Some(hp.clone());
        *self.bv_pipeline.lock().unwrap_or_else(|e| e.into_inner()) = Some(vp.clone());
        Ok((hp, vp))
    }
}

/// 把 map 好的 readback（f32 流）并行拷回宿主指针（块切分与 nlm3 回读同式）。
///
/// SAFETY（调用方保证）：dst 可写 n 个 f32。
unsafe fn readback_f32_par(
    mapped: &wgpu::BufferView,
    dst: *mut f32,
    n: usize,
) {
    let vals: &[f32] = bytemuck::cast_slice(&**mapped);
    let out: &mut [f32] = std::slice::from_raw_parts_mut(dst, n);
    let threads = std::thread::available_parallelism()
        .map(|x| x.get())
        .unwrap_or(4)
        .min(16);
    let chunk = n.div_ceil(threads);
    std::thread::scope(|s| {
        for (t, oc) in out.chunks_mut(chunk).enumerate() {
            let i0 = t * chunk;
            let v = &vals[i0..i0 + oc.len()];
            s.spawn(move || oc.copy_from_slice(v));
        }
    });
}

/// 等 readback map 完成（与 nlm3 的轮询同式；120s 超时）。
/// map_async 由调用方先发起（借用链干净），这里只管轮询收结果。
fn wait_map(
    ctx: &Ctx,
    recv: mpsc::Receiver<std::result::Result<(), String>>,
) -> std::result::Result<(), String> {
    let started = Instant::now();
    loop {
        match ctx.device.poll(wgpu::PollType::Wait {
            submission_index: None,
            timeout: Some(Duration::from_millis(50)),
        }) {
            Ok(_) | Err(wgpu::PollError::Timeout) => {}
            Err(e) => return Err(e.to_string()),
        }
        match recv.try_recv() {
            Ok(Ok(())) => return Ok(()),
            Ok(Err(e)) => return Err(e.to_string()),
            Err(mpsc::TryRecvError::Disconnected) => {
                return Err("readback disconnected".into());
            }
            Err(mpsc::TryRecvError::Empty) => {}
        }
        if started.elapsed() > Duration::from_secs(120) {
            return Err("GPU readback timed out".into());
        }
    }
}

/// guided_up_2x 的 GPU 版（own_decoder.c）。src/t2 是半分辨率 hw×hh，
/// guide/dst 是全分辨率 2hw×2hh；dtab 100 项、wv 为值域核表（C 侧预计算）。
/// 成功返 0，任何失败返 -1（调用点回退 CPU 行分块）。
pub fn gpu_guided_up_2x(
    src: *const f32,
    t2: *const f32,
    guide: *const f32,
    dst: *mut f32,
    hw: i32,
    hh: i32,
    dtab: *const f32,
    wv: *const f32,
    wv_n: i32,
) -> i32 {
    let t_start = Instant::now();
    let r = gpu_guided_up_2x_impl(src, t2, guide, dst, hw, hh, dtab, wv, wv_n);
    if std::env::var("X3FDEC_TIME").is_ok() {
        let tag = match r {
            Ok(()) => "gpu",
            Err(_) => "GPU-FAIL->cpu",
        };
        eprintln!(
            "TIME   own guided_up {}x{}->{}x{} {}: {:.3} s",
            hw, hh, hw * 2, hh * 2, tag, t_start.elapsed().as_secs_f64()
        );
    }
    match r {
        Ok(()) => 0,
        Err(e) => {
            if dbg_enabled() {
                eprintln!("own_gpu: guided_up GPU 失败：{e}（回退 CPU）");
            }
            -1
        }
    }
}

fn gpu_guided_up_2x_impl(
    src: *const f32,
    t2: *const f32,
    guide: *const f32,
    dst: *mut f32,
    hw: i32,
    hh: i32,
    dtab: *const f32,
    wv: *const f32,
    wv_n: i32,
) -> std::result::Result<(), String> {
    if hw <= 0 || hh <= 0 || wv_n <= 0 {
        return Err("bad dimensions".into());
    }
    let hw = hw as u32;
    let hh = hh as u32;
    let n_src = (hw as usize) * (hh as usize);
    let n_dst = n_src * 4;

    // SAFETY: 调用方保证各缓冲可读 n 项（见 own_gpu.h 注释）。
    let src_s: &[f32] = unsafe { std::slice::from_raw_parts(src, n_src) };
    let t2_s: &[f32] = unsafe { std::slice::from_raw_parts(t2, n_src) };
    let guide_s: &[f32] = unsafe { std::slice::from_raw_parts(guide, n_dst) };
    // dtab = float[2][2][5][5] = 100 项（首版误传 50，GPU 读 75+ 索引越界：
    // 既是错值来源也是设备丢失/段错误来源 —— 2026-10-24 修复）
    let dtab_s: &[f32] = unsafe { std::slice::from_raw_parts(dtab, 100) };
    let wv_s: &[f32] = unsafe { std::slice::from_raw_parts(wv, wv_n as usize) };

    with_ctx(|ctx| {
        let pipeline = ctx.gu_pipeline_for()?;
        let lim = ctx.device.limits();
        let cap = lim
            .max_buffer_size
            .min(u64::from(lim.max_storage_buffer_binding_size));
        let dst_size = (n_dst * 4) as u64;
        if dst_size > cap {
            return Err(format!("guided_up buffer {dst_size} bytes over device limit {cap}"));
        }

        let src_buf = mk_storage_buf(&ctx.device, &ctx.queue, "gu src", src_s)?;
        let t2_buf = mk_storage_buf(&ctx.device, &ctx.queue, "gu t2", t2_s)?;
        let guide_buf = mk_storage_buf(&ctx.device, &ctx.queue, "gu guide", guide_s)?;
        let dtab_buf = mk_storage_buf(&ctx.device, &ctx.queue, "gu dtab", dtab_s)?;
        let wv_buf = mk_storage_buf(&ctx.device, &ctx.queue, "gu wv", wv_s)?;
        let dst_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("gu dst"),
            size: dst_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let params: [u32; 4] = [hw, hh, wv_n as u32, 0];
        let params_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("gu params"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        ctx.queue.write_buffer(&params_buf, 0, bytemuck::cast_slice(&params));

        let bind = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("gu bind"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: src_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: t2_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: guide_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: dst_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 4, resource: dtab_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 5, resource: wv_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 6, resource: params_buf.as_entire_binding() },
            ],
        });

        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups((hw * 2).div_ceil(16), (hh * 2).div_ceil(16), 1);
        }
        let readback = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("gu readback"),
            size: dst_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        encoder.copy_buffer_to_buffer(&dst_buf, 0, &readback, 0, dst_size);
        ctx.queue.submit([encoder.finish()]);

        let slice = readback.slice(..dst_size);
        let (send, recv) = mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |res| {
            let _ = send.send(res.map_err(|e| e.to_string()));
        });
        wait_map(ctx, recv)?;
        // SAFETY: dst 可写 n_dst 个 f32（调用方保证）。
        unsafe {
            let mapped = slice.get_mapped_range();
            readback_f32_par(&mapped, dst, n_dst);
        }
        readback.unmap();
        Ok(())
    })
}

fn mk_storage_buf(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    label: &str,
    data: &[f32],
) -> std::result::Result<wgpu::Buffer, String> {
    let size = (data.len() * 4) as u64;
    let buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    queue.write_buffer(&buf, 0, bytemuck::cast_slice(data));
    Ok(buf)
}

/// bilateral5 的 GPU 版（own_decoder.c）：两 pass 可分离滤波，结果写回 img。
/// rlut/sw 为 C 侧预计算表（f32 位模式原样上传）。成功返 0，失败返 -1。
pub fn gpu_bilateral5(
    img: *mut f32,
    w: i32,
    h: i32,
    rlut: *const f32,
    rlut_n: i32,
    sw: *const f32,
) -> i32 {
    let t_start = Instant::now();
    let r = gpu_bilateral5_impl(img, w, h, rlut, rlut_n, sw);
    if std::env::var("X3FDEC_TIME").is_ok() {
        let tag = match r {
            Ok(()) => "gpu",
            Err(_) => "GPU-FAIL->cpu",
        };
        eprintln!(
            "TIME   own bilateral5 {}x{} {}: {:.3} s",
            w,
            h,
            tag,
            t_start.elapsed().as_secs_f64()
        );
    }
    match r {
        Ok(()) => 0,
        Err(e) => {
            if dbg_enabled() {
                eprintln!("own_gpu: bilateral5 GPU 失败：{e}（回退 CPU）");
            }
            -1
        }
    }
}

fn gpu_bilateral5_impl(
    img: *mut f32,
    w: i32,
    h: i32,
    rlut: *const f32,
    rlut_n: i32,
    sw: *const f32,
) -> std::result::Result<(), String> {
    if w <= 0 || h <= 0 || rlut_n <= 0 {
        return Err("bad dimensions".into());
    }
    let w = w as u32;
    let h = h as u32;
    let n = (w as usize) * (h as usize);

    // SAFETY: 调用方保证 img 可读写 n 个 f32；rlut/sw 可读各自长度。
    let img_s: &mut [f32] = unsafe { std::slice::from_raw_parts_mut(img, n) };
    let rlut_s: &[f32] = unsafe { std::slice::from_raw_parts(rlut, rlut_n as usize) };
    let sw_s: &[f32] = unsafe { std::slice::from_raw_parts(sw, 5) };

    with_ctx(|ctx| {
        let (ph, pv) = ctx.bilat_pipelines_for()?;
        let plane_size = (n * 4) as u64;
        let lim = ctx.device.limits();
        let cap = lim
            .max_buffer_size
            .min(u64::from(lim.max_storage_buffer_binding_size));
        if plane_size > cap {
            return Err(format!("bilateral5 buffer {plane_size} bytes over device limit {cap}"));
        }

        let mkbuf = |label: &str, extra: wgpu::BufferUsages| {
            ctx.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: plane_size,
                usage: wgpu::BufferUsages::STORAGE | extra,
                mapped_at_creation: false,
            })
        };
        let img_buf = mkbuf("bilat img", wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC);
        let tmp_buf = mkbuf("bilat tmp", wgpu::BufferUsages::COPY_SRC);
        let out_buf = mkbuf("bilat out", wgpu::BufferUsages::COPY_SRC);
        let rlut_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("bilat rlut"),
            size: (rlut_s.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let sw_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("bilat sw"),
            size: 32,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let params: [u32; 4] = [w, h, rlut_n as u32, 0];
        let params_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("bilat params"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        ctx.queue.write_buffer(&img_buf, 0, bytemuck::cast_slice(&*img_s));
        ctx.queue.write_buffer(&rlut_buf, 0, bytemuck::cast_slice(rlut_s));
        ctx.queue.write_buffer(&sw_buf, 0, bytemuck::cast_slice(sw_s));
        ctx.queue.write_buffer(&params_buf, 0, bytemuck::cast_slice(&params));

        let mkbind = |input: &wgpu::Buffer, output: &wgpu::Buffer, pl: &wgpu::ComputePipeline| {
            ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("bilat bind"),
                layout: &pl.get_bind_group_layout(0),
                entries: &[
                    wgpu::BindGroupEntry { binding: 0, resource: input.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 1, resource: output.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 2, resource: rlut_buf.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 3, resource: sw_buf.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 4, resource: params_buf.as_entire_binding() },
                ],
            })
        };
        let bh = mkbind(&img_buf, &tmp_buf, &ph);
        let bv = mkbind(&tmp_buf, &out_buf, &pv);

        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&ph);
            pass.set_bind_group(0, &bh, &[]);
            pass.dispatch_workgroups(n.div_ceil(256) as u32, 1, 1);
        }
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&pv);
            pass.set_bind_group(0, &bv, &[]);
            pass.dispatch_workgroups(n.div_ceil(256) as u32, 1, 1);
        }
        let readback = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("bilat readback"),
            size: plane_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        encoder.copy_buffer_to_buffer(&out_buf, 0, &readback, 0, plane_size);
        ctx.queue.submit([encoder.finish()]);

        let slice = readback.slice(..plane_size);
        let (send, recv) = mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |res| {
            let _ = send.send(res.map_err(|e| e.to_string()));
        });
        wait_map(ctx, recv)?;
        // SAFETY: img 可写 n 个 f32（见上）。
        unsafe {
            let mapped = slice.get_mapped_range();
            readback_f32_par(&mapped, img, n);
        }
        readback.unmap();
        Ok(())
    })
}

/* ============================ chrmed（色度中值） ============================
 * x3f_render.c apply_chroma_median 的 GPU 平面步骤（own_gpu_chrmed_plane）：
 * 单平面 (2r+1)² 窗口中值，输入/输出都是 w×h f32。算法见 own_chrmed_sel.wgsl
 * 头注释（fkey 键域二分，与 CPU chrmed_select 值精确一致）。
 * pipeline 按 r 缓存（半径是模板编译期常量）。 */

impl Ctx {
    fn chrmed_pipeline_for(
        &self,
        r: u32,
    ) -> std::result::Result<wgpu::ComputePipeline, String> {
        if let Some(p) = self
            .pipelines
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&(r, 9999))   // (r, 9999) 占位键，避免再开一个 HashMap
        {
            return Ok(p.clone());
        }
        // workgroup 边长：8 与 16 实测无差（瓶颈是二分趟内 TG 读带宽），
        // 取 16（对拍即在此配置下做的）
        const WG: u32 = 16;
        let source = include_str!("own_chrmed_sel.wgsl")
            .replace("__R__", &r.to_string())
            .replace("__WG__", &WG.to_string())
            .replace(
                "__TSN__",
                &(((WG + 2 * r) * (WG + 2 * r)) as u32).to_string(),
            );
        let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("x3f own chrmed sel"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
        self.check()?;
        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("x3f own chrmed sel"),
                layout: None,
                module: &shader,
                entry_point: Some("chrmed_sel"),
                compilation_options: Default::default(),
                cache: None,
            });
        self.check()?;
        self.pipelines
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert((r, 9999), pipeline.clone());
        Ok(pipeline)
    }
}

/* ---- 单趟排序网络中值（2026-10-31）--------------------------------------
 * 旧版 sel 走「键域二分 32 趟」：每一趟都要把整个窗口从 workgroup 内存重读
 * 一遍（32 × N 次 TG 读）。Intel 核显 5424×3616 r=3 实测 2.2s/平面，瓶颈是
 * **shared 读次数 × 趟数**，不是算力——这也解释了为什么之前把窗口搬进
 * per-thread scratch 反而更慢（3.1s，scratch spill 到 global）。
 *
 * 新版把窗口 N 个键**一次**读进标量变量（WGSL 里是 v0..v{N-1}，无数组下标，
 * 编译后全在寄存器），用 bitonic 排序网络（补到 2 的幂 m 槽，空槽填
 * 0xFFFFFFFF = 最大键，不影响第 n>>1 位）排完直接取 v_{n>>1}：
 *   shared 读 1568 → 49 次；代价是 ~2000 条纯 ALU（无内存、无长依赖链）。
 *
 * 正确性：排序网络把同一多重集合排成全序，第 n>>1 小值与选择算法无关，
 * 因此与 CPU chrmed_select **值精确一致**（−0.0/+0.0 的键映射差异同旧版）。
 *
 * 边界像素的窗口会收缩（n 可变 → k 不固定），仍走旧二分分支：占比只有
 * 周长级的 ~0.6%，留在同一 kernel 里 if 分流，不影响内部像素的寄存器压力。
 * 只有 N ≤ 64（r ≤ 3）才生成网络版；r ≥ 4 窗口 169/225 元素网络过大，
 * 调用点继续用 sel。X3FDEC_CHRMED_MED=0 可强制回旧 sel 做 A/B。 */

fn bitonic_net(m: usize) -> Vec<(usize, usize, bool)> {
    let mut out = Vec::new();
    let mut k = 2usize;
    while k <= m {
        let mut j = k / 2;
        while j > 0 {
            for i in 0..m {
                let l = i ^ j;
                if l > i {
                    out.push((i, l, (i & k) == 0));
                }
            }
            j /= 2;
        }
        k *= 2;
    }
    out
}

const CHRMED_MED_TOP: &str = r#"
struct Dims { w: u32, h: u32, pad0: u32, pad1: u32 };

@group(0) @binding(0) var<storage, read> pl: array<u32>;
@group(0) @binding(1) var<storage, read_write> omed: array<u32>;
@group(0) @binding(2) var<uniform> dims: Dims;

fn fkey(v: f32) -> u32 {
    let b = bitcast<u32>(v);
    return select(b | 0x80000000u, ~b, (b & 0x80000000u) != 0u);
}
fn unkey(k: u32) -> f32 {
    return bitcast<f32>(select(~k, k & 0x7fffffffu, (k & 0x80000000u) != 0u));
}

const R: u32 = __R__u;
const WG: u32 = __WG__u;
const TS: u32 = __TS__u;
var<workgroup> tile: array<u32, __TSN__u>;

@compute @workgroup_size(WG, WG)
fn chrmed_med(
    @builtin(global_invocation_id) g: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
    @builtin(workgroup_id) tid: vec3<u32>,
) {
    let w = dims.w;
    let h = dims.h;
    let x = g.x;
    let y = g.y;
    let bx = tid.x * WG;
    let by = tid.y * WG;
    let li = lid.y * WG + lid.x;

    for (var kk = li; kk < __TSN__u; kk = kk + WG * WG) {
        let tyy = kk / TS;
        let txx = kk - tyy * TS;
        var sy = by + tyy;
        sy = select(sy - R, 0u, sy < R);
        sy = min(sy, h - 1u);
        var sx = bx + txx;
        sx = select(sx - R, 0u, sx < R);
        sx = min(sx, w - 1u);
        tile[kk] = fkey(bitcast<f32>(pl[sy * w + sx]));
    }
    workgroupBarrier();

    if (x >= w || y >= h) { return; }

    var med: u32 = 0u;
    if (R <= x && x + R < w && R <= y && y + R < h) {
        let base = lid.y * TS + lid.x;
"#;

const CHRMED_MED_TAIL: &str = r#"    } else {
        let y0 = select(y - R, 0u, y < R);
        let y1 = min(y + R, h - 1u);
        let x0 = select(x - R, 0u, x < R);
        let x1 = min(x + R, w - 1u);
        let nq = (y1 - y0 + 1u) * (x1 - x0 + 1u);
        let kq = nq >> 1u;
        let ty0 = y0 + R - by;
        let ty1 = y1 + R - by;
        let tx0 = x0 + R - bx;
        let tx1 = x1 + R - bx;
        var lo: u32 = 0u;
        var hi: u32 = 0xffffffffu;
        loop {
            if (lo >= hi) { break; }
            let mid = lo + (hi - lo) / 2u;
            var cnt: u32 = 0u;
            for (var tyy = ty0; tyy <= ty1; tyy = tyy + 1u) {
                let rowbase = tyy * TS;
                for (var txx = tx0; txx <= tx1; txx = txx + 1u) {
                    if (tile[rowbase + txx] <= mid) { cnt = cnt + 1u; }
                }
            }
            if (cnt > kq) { hi = mid; } else { lo = mid + 1u; }
        }
        med = lo;
    }
    omed[y * w + x] = bitcast<u32>(unkey(med));
}
"#;

/// 按半径生成单趟排序网络中值 shader。r ≤ 3（N ≤ 49）才该调用。
fn build_chrmed_med_wgsl(r: u32, wg: u32) -> String {
    let side = (2 * r + 1) as usize;
    let n = side * side;
    let ts = wg + 2 * r;
    let tsn = (ts * ts) as u32;
    let m = n.next_power_of_two();
    let net = bitonic_net(m);
    let mut s = String::with_capacity(128 * 1024);
    s.push_str(CHRMED_MED_TOP);
    for dy in 0..side {
        for dx in 0..side {
            s.push_str(&format!(
                "        var v{}: u32 = tile[base + {}u];\n",
                dy * side + dx,
                (dy as u32) * ts + dx as u32
            ));
        }
    }
    for idx in n..m {
        s.push_str(&format!("        var v{}: u32 = 0xffffffffu;\n", idx));
    }
    for (c, (i, j, asc)) in net.iter().enumerate() {
        if *asc {
            s.push_str(&format!(
                "        let t{c} = v{i};\n        v{i} = min(t{c}, v{j});\n        v{j} = max(t{c}, v{j});\n"
            ));
        } else {
            s.push_str(&format!(
                "        let t{c} = v{i};\n        v{i} = max(t{c}, v{j});\n        v{j} = min(t{c}, v{j});\n"
            ));
        }
    }
    s.push_str(&format!("        med = v{};\n", n / 2));
    s.push_str(CHRMED_MED_TAIL);
    s.replace("__TSN__", &tsn.to_string())
        .replace("__TS__", &ts.to_string())
        .replace("__WG__", &wg.to_string())
        .replace("__R__", &r.to_string())
}

/// 单趟排序网络中值是否可用（r ≤ 3）且未被 X3FDEC_CHRMED_MED=0 关掉。
fn chrmed_med_ok(r: u32) -> bool {
    let side = 2 * r + 1;
    side * side <= 64
        && std::env::var("X3FDEC_CHRMED_MED")
            .map(|v| v != "0")
            .unwrap_or(true)
}

impl Ctx {
    fn chrmed_med_pipeline_for(
        &self,
        r: u32,
    ) -> std::result::Result<wgpu::ComputePipeline, String> {
        if let Some(p) = self
            .pipelines
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&(r, 8888))   // (r, 8888) 占位键，与 sel 的 (r, 9999) 区分
        {
            return Ok(p.clone());
        }
        const WG: u32 = 16;
        let source = build_chrmed_med_wgsl(r, WG);
        let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("x3f own chrmed med"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
        self.check()?;
        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("x3f own chrmed med"),
                layout: None,
                module: &shader,
                entry_point: Some("chrmed_med"),
                compilation_options: Default::default(),
                cache: None,
            });
        self.check()?;
        self.pipelines
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert((r, 8888), pipeline.clone());
        Ok(pipeline)
    }
}

/// 色度中值单平面窗口中值（x3f_render.c → own_gpu_win.c → 这里）。
/// pl 可读 w*h 项，omed 可写 w*h 项。成功返 0，任何失败返 -1（调用点回退
/// CPU 并行路径）。
pub fn gpu_chrmed_plane(
    pl: *const f32,
    omed: *mut f32,
    w: i32,
    h: i32,
    r: i32,
) -> i32 {
    let t_start = Instant::now();
    let res = gpu_chrmed_plane_impl(pl, omed, w, h, r);
    if std::env::var("X3FDEC_TIME").is_ok() {
        let tag = match res {
            Ok(()) => "gpu",
            Err(_) => "GPU-FAIL->cpu",
        };
        eprintln!(
            "TIME   own chrmed_plane r={} {}x{} {}: {:.3} s",
            r,
            w,
            h,
            tag,
            t_start.elapsed().as_secs_f64()
        );
    }
    match res {
        Ok(()) => 0,
        Err(e) => {
            if dbg_enabled() {
                eprintln!("own_gpu: chrmed GPU 失败：{e}（回退 CPU）");
            }
            -1
        }
    }
}

fn gpu_chrmed_plane_impl(
    pl: *const f32,
    omed: *mut f32,
    w: i32,
    h: i32,
    r: i32,
) -> std::result::Result<(), String> {
    if w <= 0 || h <= 0 || r < 1 || r > 7 {
        return Err("bad chrmed params".into());
    }
    let w = w as u32;
    let h = h as u32;
    let r = r as u32;
    let n = (w as usize) * (h as usize);

    // SAFETY: 调用方保证 pl 可读 n 项、omed 可写 n 项（own_gpu.h 注释）。
    let pl_s: &[f32] = unsafe { std::slice::from_raw_parts(pl, n) };

    with_ctx(|ctx| {
        ctx.check()?;
        // 单趟排序网络（r ≤ 3，N ≤ 49）优先；r ≥ 4 的 169/225 元素网络过大，
        // 仍用键域二分 sel。X3FDEC_CHRMED_MED=0 可回退做 A/B。
        let pipeline = if chrmed_med_ok(r) {
            ctx.chrmed_med_pipeline_for(r)?
        } else {
            ctx.chrmed_pipeline_for(r)?
        };
        let lim = ctx.device.limits();
        let cap = lim
            .max_buffer_size
            .min(u64::from(lim.max_storage_buffer_binding_size));
        let plane_size = (n * 4) as u64;
        if plane_size > cap {
            return Err(format!("chrmed buffer {plane_size} bytes over device limit {cap}"));
        }

        let pl_buf = mk_storage_buf(&ctx.device, &ctx.queue, "chrmed pl", pl_s)?;
        let omed_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chrmed omed"),
            size: plane_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let dims: [u32; 4] = [w, h, 0, 0];
        let dims_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chrmed dims"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        ctx.queue.write_buffer(&dims_buf, 0, bytemuck::cast_slice(&dims));

        let bind = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chrmed bind"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: pl_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: omed_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: dims_buf.as_entire_binding() },
            ],
        });

        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            // WG 与 shader 模板一致（chrmed_pipeline_for 注入 16）
            pass.dispatch_workgroups(w.div_ceil(16), h.div_ceil(16), 1);
        }
        let readback = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chrmed readback"),
            size: plane_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        encoder.copy_buffer_to_buffer(&omed_buf, 0, &readback, 0, plane_size);
        ctx.queue.submit([encoder.finish()]);

        let slice = readback.slice(..plane_size);
        let (send, recv) = mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |res| {
            let _ = send.send(res.map_err(|e| e.to_string()));
        });
        wait_map(ctx, recv)?;
        // SAFETY: omed 可写 n 个 f32（见上）。
        unsafe {
            let mapped = slice.get_mapped_range();
            readback_f32_par(&mapped, omed, n);
        }
        readback.unmap();
        Ok(())
    })
}

// ===== 渲染层 apply 链 GPU 化（2026-10-27，对齐 Mac own_gpu.m 的 apply_lut /
// chrmed_prep+sel+emit / down8k / s1bilat 四 kernel）=====
//
// 与 C 调用点（x3f_render.c apply_ex_common、x3f_stage1.c s1_bilateral）的
// 内存布局逐字节对齐：u16 三通道一律打包 vec2<u32>（低16=ch0，高16=ch1，
// 第二 u32=ch2），打包/解包在宿主并行完成。任何失败返回 -1，由 C 侧回退
// CPU 对应段。预览专用（X3F_GPU_APPLY gate），导出/CLI 恒走 CPU 保 md5。

/// u16 交错 3ch（len/3 像素）→ 打包 vec2<u32>（低16=ch0，高16=ch1，第二=ch2）。
fn pack_rgb_u16(src: &[u16]) -> Vec<u32> {
    let n = src.len() / 3;
    let mut out = vec![0u32; n * 2];
    let threads = std::thread::available_parallelism()
        .map(|x| x.get())
        .unwrap_or(4)
        .min(16);
    let chunk = n.div_ceil(threads).max(1);
    std::thread::scope(|s| {
        for (t, oc) in out.chunks_mut(chunk * 2).enumerate() {
            let i0 = t * chunk;
            let inp = &src[i0 * 3..i0 * 3 + (oc.len() / 2) * 3];
            s.spawn(move || {
                for (p, o2) in oc.chunks_exact_mut(2).enumerate() {
                    o2[0] = (inp[p * 3] as u32) | ((inp[p * 3 + 1] as u32) << 16);
                    o2[1] = inp[p * 3 + 2] as u32;
                }
            });
        }
    });
    out
}

/// 打包 vec2<u32> → u16 交错 3ch（unpack_rgb_u16 的逆变）。
fn unpack_rgb_u16(packed: &[u32], dst: &mut [u16]) {
    let threads = std::thread::available_parallelism()
        .map(|x| x.get())
        .unwrap_or(4)
        .min(16);
    let chunk = (dst.len() / 3).div_ceil(threads).max(1);
    std::thread::scope(|s| {
        for (t, oc) in dst.chunks_mut(chunk * 3).enumerate() {
            let i0 = t * chunk;
            let pk = &packed[i0 * 2..i0 * 2 + (oc.len() / 3) * 2];
            s.spawn(move || {
                for (p, o3) in oc.chunks_exact_mut(3).enumerate() {
                    o3[0] = (pk[p * 2] & 0xffff) as u16;
                    o3[1] = (pk[p * 2] >> 16) as u16;
                    o3[2] = pk[p * 2 + 1] as u16;
                }
            });
        }
    });
}

/// apply_lut16 的 GPU 版（x3f_render.c apply_ex_common 显示域 LUT 管线）。
/// lin 为线性域 f32（w*h*3）；clip 可为 NULL；三张 tone LUT 各 4097 项 u16
/// （此处归一到 [0,1] 上传）；gain 可为 NULL；cm3 可为 NULL；cmcb/cmcr 可为
/// NULL（65×65 色彩匹配）；out 写 u16 交错 3ch（w*h*3）。成功返 0。
#[allow(clippy::too_many_arguments)]
pub fn gpu_apply_lut16(
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
    use_sat: i32,
    hl_desat: f32,
    desat_lo: f32,
    desat_hi: f32,
    out: *mut u16,
    w: i32,
    h: i32,
) -> i32 {
    let t_start = Instant::now();
    let res = gpu_apply_lut16_impl(
        lin, clip, lut, lutg, lutb, gain, cm3, cmcb, cmcr, sat, use_sat,
        hl_desat, desat_lo, desat_hi, out, w, h,
    );
    if std::env::var("X3FDEC_TIME").is_ok() {
        let tag = match res {
            Ok(()) => "gpu",
            Err(_) => "GPU-FAIL->cpu",
        };
        eprintln!(
            "TIME   own apply_lut16 {}x{} {}: {:.3} s",
            w,
            h,
            tag,
            t_start.elapsed().as_secs_f64()
        );
    }
    match res {
        Ok(()) => 0,
        Err(e) => {
            if dbg_enabled() {
                eprintln!("own_gpu: apply_lut16 GPU 失败：{e}（回退 CPU）");
            }
            -1
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn gpu_apply_lut16_impl(
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
    use_sat: i32,
    hl_desat: f32,
    desat_lo: f32,
    desat_hi: f32,
    out: *mut u16,
    w: i32,
    h: i32,
) -> std::result::Result<(), String> {
    if w <= 0 || h <= 0 {
        return Err("bad apply_lut16 params".into());
    }
    let w = w as u32;
    let h = h as u32;
    let n = (w as usize) * (h as usize);
    const LUT_N: usize = 4097;

    // SAFETY（调用方 x3f_render.c 保证）：lin 可读 n*3、out 可写 n*3、
    // 三张 LUT 各可读 4097；可选缓冲按指针判空。
    let lin_s: &[f32] = unsafe { std::slice::from_raw_parts(lin, n * 3) };
    let norm_lut = |p: *const u16| -> Vec<f32> {
        (0..LUT_N)
            .map(|i| unsafe { *p.add(i) } as f32 / 65535.0)
            .collect()
    };
    let lut_s = norm_lut(lut);
    let lutg_s = norm_lut(lutg);
    let lutb_s = norm_lut(lutb);
    let opt_f32 =
        |p: *const f32, k: usize| -> Option<Vec<f32>> {
            if p.is_null() {
                None
            } else {
                Some(unsafe { std::slice::from_raw_parts(p, k) }.to_vec())
            }
        };
    let clip_s = opt_f32(clip, n);
    let gain_s = opt_f32(gain, 3);
    let cm3_s = opt_f32(cm3, 9);
    let cmcb_s = opt_f32(cmcb, 65 * 65);
    let cmcr_s = opt_f32(cmcr, 65 * 65);
    let out_s: &mut [u16] = unsafe { std::slice::from_raw_parts_mut(out, n * 3) };

    with_ctx(|ctx| {
        ctx.check()?;
        let pipeline = ctx.apply_lut_pipeline_for()?;
        let lim = ctx.device.limits();
        let cap = lim
            .max_buffer_size
            .min(u64::from(lim.max_storage_buffer_binding_size));
        let lin_size = (n * 3 * 4) as u64;
        let out_size = (n * 8) as u64;
        if lin_size > cap || out_size > cap {
            return Err(format!("apply_lut16 buffer over device limit {cap}"));
        }

        let lin_buf = mk_storage_buf(&ctx.device, &ctx.queue, "alut lin", lin_s)?;
        let dummy16: [f32; 16] = [0.0; 16];
        let clip_buf = match &clip_s {
            Some(c) => mk_storage_buf(&ctx.device, &ctx.queue, "alut clip", c)?,
            None => mk_storage_buf(&ctx.device, &ctx.queue, "alut clip0", &dummy16)?,
        };
        let lut_buf = mk_storage_buf(&ctx.device, &ctx.queue, "alut lut", &lut_s)?;
        let lutg_buf = mk_storage_buf(&ctx.device, &ctx.queue, "alut lutg", &lutg_s)?;
        let lutb_buf = mk_storage_buf(&ctx.device, &ctx.queue, "alut lutb", &lutb_s)?;
        let out_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("alut out"),
            size: out_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let gain_buf = match &gain_s {
            Some(g) => mk_storage_buf(&ctx.device, &ctx.queue, "alut gain", g)?,
            None => mk_storage_buf(&ctx.device, &ctx.queue, "alut gain0", &dummy16)?,
        };
        let cm3_buf = match &cm3_s {
            Some(m) => mk_storage_buf(&ctx.device, &ctx.queue, "alut cm3", m)?,
            None => mk_storage_buf(&ctx.device, &ctx.queue, "alut cm30", &dummy16)?,
        };
        let cmcb_buf = match &cmcb_s {
            Some(m) => mk_storage_buf(&ctx.device, &ctx.queue, "alut cmcb", m)?,
            None => mk_storage_buf(&ctx.device, &ctx.queue, "alut cmcb0", &dummy16)?,
        };
        let cmcr_buf = match &cmcr_s {
            Some(m) => mk_storage_buf(&ctx.device, &ctx.queue, "alut cmcr", m)?,
            None => mk_storage_buf(&ctx.device, &ctx.queue, "alut cmcr0", &dummy16)?,
        };

        let pu: [u32; 4] = [
            w,
            h,
            u32::from(gain_s.is_some()),
            u32::from(use_sat != 0),
        ];
        let pv: [u32; 4] = [
            u32::from(cm3_s.is_some()),
            u32::from(cmcb_s.is_some() && cmcr_s.is_some()),
            u32::from(clip_s.is_some()),
            0,
        ];
        // pf 也走 u32（f32 位模式）对齐 nlm3：Vulkan 下 uniform/storage 的 f32
        // 读取有隐患，统一用 u32 + bitcast 规避。shader 端 APF 已改为 u32 字段。
        let pf: [u32; 4] = [
            bytemuck::cast(sat),
            bytemuck::cast(hl_desat),
            bytemuck::cast(desat_lo),
            bytemuck::cast(desat_hi),
        ];
        let mk_uni = |label: &str, data: &[u8]| -> Result<wgpu::Buffer, String> {
            let b = ctx.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: 16,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            ctx.queue.write_buffer(&b, 0, data);
            Ok(b)
        };
        let pu_buf = mk_uni("alut pu", bytemuck::cast_slice(&pu))?;
        let pv_buf = mk_uni("alut pv", bytemuck::cast_slice(&pv))?;
        let pf_buf = mk_uni("alut pf", bytemuck::cast_slice(&pf))?;

        let bind = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("alut bind"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: lin_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: clip_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: lut_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: lutg_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 4, resource: lutb_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 5, resource: out_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 6, resource: gain_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 7, resource: cm3_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 8, resource: cmcb_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 9, resource: cmcr_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 10, resource: pu_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 11, resource: pv_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 12, resource: pf_buf.as_entire_binding() },
            ],
        });

        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups(w.div_ceil(16), h.div_ceil(16), 1);
        }
        let readback = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("alut readback"),
            size: out_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        encoder.copy_buffer_to_buffer(&out_buf, 0, &readback, 0, out_size);
        ctx.queue.submit([encoder.finish()]);

        let slice = readback.slice(..out_size);
        let (send, recv) = mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |res| {
            let _ = send.send(res.map_err(|e| e.to_string()));
        });
        wait_map(ctx, recv)?;
        // 异步验证错误（管线/绑定组创建失败等）此刻已进 failure 槽——再查一次，
        // 把"提交成功但内核没跑、回读全零"的静默坏图转成显式回退 CPU。
        ctx.check()?;
        let vals: Vec<u32> = {
            let mapped = slice.get_mapped_range();
            bytemuck::cast_slice(&*mapped).to_vec()
        };
        readback.unmap();
        unpack_rgb_u16(&vals, out_s);
        Ok(())
    })
}

/// chrmed_u16 的 GPU 版（x3f_render.c apply_chroma_median 全链）：prep（U/V
/// 平面）→ 两趟窗口中值（复用 own_chrmed_sel.wgsl）→ emit（重建 RGB 原地写
/// 回 buf）。调用点 stride 恒为 w*6（行紧凑），其余 stride 返 -1 回 CPU。
/// 中值窗口与 sel 管线一致限 r∈[1,7]。成功返 0。
pub fn gpu_chrmed_u16(buf: *mut u16, w: i32, h: i32, stride: i32, r: i32) -> i32 {
    let t_start = Instant::now();
    let res = gpu_chrmed_u16_impl(buf, w, h, stride, r);
    if std::env::var("X3FDEC_TIME").is_ok() {
        let tag = match res {
            Ok(()) => "gpu",
            Err(_) => "GPU-FAIL->cpu",
        };
        eprintln!(
            "TIME   own chrmed_u16 r={} {}x{} {}: {:.3} s",
            r,
            w,
            h,
            tag,
            t_start.elapsed().as_secs_f64()
        );
    }
    match res {
        Ok(()) => 0,
        Err(e) => {
            if dbg_enabled() {
                eprintln!("own_gpu: chrmed_u16 GPU 失败：{e}（回退 CPU）");
            }
            -1
        }
    }
}

fn gpu_chrmed_u16_impl(
    buf: *mut u16,
    w: i32,
    h: i32,
    stride: i32,
    r: i32,
) -> std::result::Result<(), String> {
    if w <= 0 || h <= 0 {
        return Err("bad chrmed_u16 params".into());
    }
    if stride != w * 6 {
        return Err(format!("chrmed_u16 stride {stride} != w*6（行不紧凑）"));
    }
    if !(1..=7).contains(&r) {
        return Err(format!("chrmed_u16 r {r} 超出 sel 窗口上限"));
    }
    let w = w as u32;
    let h = h as u32;
    let r = r as u32;
    let n = (w as usize) * (h as usize);

    // SAFETY（调用方保证）：buf 可读写 n*3 个 u16（行紧凑，上方已核）。
    let buf_s: &mut [u16] = unsafe { std::slice::from_raw_parts_mut(buf, n * 3) };
    let packed = pack_rgb_u16(buf_s);

    with_ctx(|ctx| {
        ctx.check()?;
        let prep = ctx.chrmed_prep_pipeline_for()?;
        let emit = ctx.chrmed_emit_pipeline_for()?;
        let sel = if chrmed_med_ok(r) {
            ctx.chrmed_med_pipeline_for(r)?
        } else {
            ctx.chrmed_pipeline_for(r)?
        };
        let lim = ctx.device.limits();
        let cap = lim
            .max_buffer_size
            .min(u64::from(lim.max_storage_buffer_binding_size));
        let img_size = (n * 8) as u64;
        let plane_size = (n * 4) as u64;
        if img_size > cap || plane_size > cap {
            return Err(format!("chrmed_u16 buffer over device limit {cap}"));
        }

        let img_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("cu16 img"),
            size: img_size,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        ctx.queue
            .write_buffer(&img_buf, 0, bytemuck::cast_slice(&packed));
        let mk_plane = |label: &str| -> Result<wgpu::Buffer, String> {
            Ok(ctx
                .device
                .create_buffer(&wgpu::BufferDescriptor {
                    label: Some(label),
                    size: plane_size,
                    usage: wgpu::BufferUsages::STORAGE,
                    mapped_at_creation: false,
                }))
        };
        let upl_buf = mk_plane("cu16 upl")?;
        let vpl_buf = mk_plane("cu16 vpl")?;
        let omu_buf = mk_plane("cu16 omu")?;
        let omv_buf = mk_plane("cu16 omv")?;
        let dims: [u32; 4] = [w, h, 0, 0];
        let dims_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("cu16 dims"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        ctx.queue
            .write_buffer(&dims_buf, 0, bytemuck::cast_slice(&dims));

        let mk_bind = |label: &str,
                       pipe: &wgpu::ComputePipeline,
                       e: &[wgpu::BindGroupEntry]|
         -> Result<wgpu::BindGroup, String> {
            Ok(ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some(label),
                layout: &pipe.get_bind_group_layout(0),
                entries: e,
            }))
        };
        // ⚠️ prep/emit 共用一个模块但入口点实际用到的绑定不同（wgpu 自动布局
        // 按入口点静态使用生成，未用的绑定不在 layout 里——全量绑 6 项会触发
        // bind group 验证错误，Vulkan 上此前一直静默回退 CPU 的根因）：
        //   prep 用 0(img) 1(upl) 2(vpl) 5(dims)；emit 用 0(img) 3(omu) 4(omv) 5(dims)。
        let b_prep = mk_bind(
            "cu16 prep",
            &prep,
            &[
                wgpu::BindGroupEntry { binding: 0, resource: img_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: upl_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: vpl_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 5, resource: dims_buf.as_entire_binding() },
            ],
        )?;
        let b_sel_u = mk_bind(
            "cu16 sel u",
            &sel,
            &[
                wgpu::BindGroupEntry { binding: 0, resource: upl_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: omu_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: dims_buf.as_entire_binding() },
            ],
        )?;
        let b_sel_v = mk_bind(
            "cu16 sel v",
            &sel,
            &[
                wgpu::BindGroupEntry { binding: 0, resource: vpl_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: omv_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: dims_buf.as_entire_binding() },
            ],
        )?;
        let b_emit = mk_bind(
            "cu16 emit",
            &emit,
            &[
                wgpu::BindGroupEntry { binding: 0, resource: img_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: omu_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 4, resource: omv_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 5, resource: dims_buf.as_entire_binding() },
            ],
        )?;

        let gw = w.div_ceil(16);
        let gh = h.div_ceil(16);
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&prep);
            pass.set_bind_group(0, &b_prep, &[]);
            pass.dispatch_workgroups(gw, gh, 1);
            pass.set_pipeline(&sel);
            pass.set_bind_group(0, &b_sel_u, &[]);
            pass.dispatch_workgroups(gw, gh, 1);
            pass.set_bind_group(0, &b_sel_v, &[]);
            pass.dispatch_workgroups(gw, gh, 1);
            pass.set_pipeline(&emit);
            pass.set_bind_group(0, &b_emit, &[]);
            pass.dispatch_workgroups(gw, gh, 1);
        }
        let readback = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("cu16 readback"),
            size: img_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        encoder.copy_buffer_to_buffer(&img_buf, 0, &readback, 0, img_size);
        ctx.queue.submit([encoder.finish()]);

        let slice = readback.slice(..img_size);
        let (send, recv) = mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |res| {
            let _ = send.send(res.map_err(|e| e.to_string()));
        });
        wait_map(ctx, recv)?;
        // 异步验证错误（管线/绑定组创建失败等）此刻已进 failure 槽——再查一次，
        // 把"提交成功但内核没跑、回读全零"的静默坏图转成显式回退 CPU。
        ctx.check()?;
        let vals: Vec<u32> = {
            let mapped = slice.get_mapped_range();
            bytemuck::cast_slice(&*mapped).to_vec()
        };
        readback.unmap();
        unpack_rgb_u16(&vals, buf_s);
        Ok(())
    })
}

/// down8 的 GPU 版（x3f_render.c 显示域 8 位降位）：tmp 为紧凑 u16 交错 3ch
/// （w*h*3，调用点行距恒 w*3），out 按字节行距 stride 写 u8（row ≥ 3w）。
/// dither 非零时加 4×4 Bayer 抖动（与 CPU down8 循环逐字节一致，含负 dither
/// 的 unsigned 回绕语义）。成功返 0。
pub fn gpu_down8(
    tmp: *const u16,
    out: *mut u8,
    w: i32,
    h: i32,
    stride: i32,
    dither: i32,
) -> i32 {
    let t_start = Instant::now();
    let res = gpu_down8_impl(tmp, out, w, h, stride, dither);
    if std::env::var("X3FDEC_TIME").is_ok() {
        let tag = match res {
            Ok(()) => "gpu",
            Err(_) => "GPU-FAIL->cpu",
        };
        eprintln!(
            "TIME   own down8 {}x{} {}: {:.3} s",
            w,
            h,
            tag,
            t_start.elapsed().as_secs_f64()
        );
    }
    match res {
        Ok(()) => 0,
        Err(e) => {
            if dbg_enabled() {
                eprintln!("own_gpu: down8 GPU 失败：{e}（回退 CPU）");
            }
            -1
        }
    }
}

fn gpu_down8_impl(
    tmp: *const u16,
    out: *mut u8,
    w: i32,
    h: i32,
    stride: i32,
    dither: i32,
) -> std::result::Result<(), String> {
    if w <= 0 || h <= 0 || stride < w * 3 {
        return Err("bad down8 params".into());
    }
    let w = w as u32;
    let h = h as u32;
    let n = (w as usize) * (h as usize);

    // SAFETY（调用方保证）：tmp 可读 n*3（行紧凑）；out 可写 stride*h 字节。
    let tmp_s: &[u16] = unsafe { std::slice::from_raw_parts(tmp, n * 3) };
    let packed = pack_rgb_u16(tmp_s);
    let out_s: &mut [u8] =
        unsafe { std::slice::from_raw_parts_mut(out, stride as usize * h as usize) };

    with_ctx(|ctx| {
        ctx.check()?;
        let pipeline = ctx.down8_pipeline_for()?;
        let lim = ctx.device.limits();
        let cap = lim
            .max_buffer_size
            .min(u64::from(lim.max_storage_buffer_binding_size));
        let img_size = (n * 8) as u64;
        let out_size = (n * 4) as u64;
        if img_size > cap || out_size > cap {
            return Err(format!("down8 buffer over device limit {cap}"));
        }

        let tmp_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("d8 tmp"),
            size: img_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        ctx.queue
            .write_buffer(&tmp_buf, 0, bytemuck::cast_slice(&packed));
        let out_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("d8 out"),
            size: out_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let p: [u32; 4] = [w, h, stride as u32, dither as u32];
        let p_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("d8 p"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        ctx.queue.write_buffer(&p_buf, 0, bytemuck::cast_slice(&p));

        let bind = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("d8 bind"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: tmp_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: out_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: p_buf.as_entire_binding() },
            ],
        });

        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups(w.div_ceil(16), h.div_ceil(16), 1);
        }
        let readback = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("d8 readback"),
            size: out_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        encoder.copy_buffer_to_buffer(&out_buf, 0, &readback, 0, out_size);
        ctx.queue.submit([encoder.finish()]);

        let slice = readback.slice(..out_size);
        let (send, recv) = mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |res| {
            let _ = send.send(res.map_err(|e| e.to_string()));
        });
        wait_map(ctx, recv)?;
        // 异步验证错误（管线/绑定组创建失败等）此刻已进 failure 槽——再查一次，
        // 把"提交成功但内核没跑、回读全零"的静默坏图转成显式回退 CPU。
        ctx.check()?;
        let vals: Vec<u32> = {
            let mapped = slice.get_mapped_range();
            bytemuck::cast_slice(&*mapped).to_vec()
        };
        readback.unmap();

        // 行并行解包到 strided u8（r | g<<8 | b<<16 → 3 字节/像素）。
        let wu = w as usize;
        let stride_u = stride as usize;
        let threads = std::thread::available_parallelism()
            .map(|x| x.get())
            .unwrap_or(4)
            .min(16);
        let row_chunk = (h as usize).div_ceil(threads).max(1);
        std::thread::scope(|s| {
            for (t, rc) in out_s.chunks_mut(row_chunk * stride_u).enumerate() {
                let y0 = t * row_chunk;
                let rows = rc.len() / stride_u;
                let vals = &vals[y0 * wu..y0 * wu + rows * wu];
                s.spawn(move || {
                    for (dy, row) in rc.chunks_mut(stride_u).enumerate() {
                        for (x, vb) in row[..wu * 3].chunks_exact_mut(3).enumerate() {
                            let v = vals[dy * wu + x];
                            vb[0] = (v & 0xff) as u8;
                            vb[1] = ((v >> 8) & 0xff) as u8;
                            vb[2] = ((v >> 16) & 0xff) as u8;
                        }
                    }
                });
            }
        });
        Ok(())
    })
}

/// stage1 双边滤波的 GPU 版（x3f_stage1.c s1_bilateral）：空间表与 CPU 同式
/// 自建（exp(-d2/(2·(r/2)²))，f64 算完转 f32）；coef 为 9 项噪声二次式
/// （f64→f32）；σ 与 s1_v/s1_sigma 同式（见 own_stage1_bilat.wgsl 注释）。
/// src/dst 均为 f32 交错 3ch（w*h*3）。r∈[1,6]（C 侧 sptbuf[169] 上限）。
/// 成功返 0，失败返 -1（调用点回 CPU 行并行版）。
pub fn gpu_stage1_bilat(
    src: *const f32,
    dst: *mut f32,
    w: i32,
    h: i32,
    r: i32,
    coef: *const f64,
    lin_sat: f64,
    level_scale: f64,
    nscale: f64,
) -> i32 {
    let t_start = Instant::now();
    let res = gpu_stage1_bilat_impl(src, dst, w, h, r, coef, lin_sat, level_scale, nscale);
    if std::env::var("X3FDEC_TIME").is_ok() {
        let tag = match res {
            Ok(()) => "gpu",
            Err(_) => "GPU-FAIL->cpu",
        };
        eprintln!(
            "TIME   own s1bilat r={} {}x{} {}: {:.3} s",
            r,
            w,
            h,
            tag,
            t_start.elapsed().as_secs_f64()
        );
    }
    match res {
        Ok(()) => 0,
        Err(e) => {
            if dbg_enabled() {
                eprintln!("own_gpu: s1bilat GPU 失败：{e}（回退 CPU）");
            }
            -1
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn gpu_stage1_bilat_impl(
    src: *const f32,
    dst: *mut f32,
    w: i32,
    h: i32,
    r: i32,
    coef: *const f64,
    lin_sat: f64,
    level_scale: f64,
    nscale: f64,
) -> std::result::Result<(), String> {
    if w <= 0 || h <= 0 || !(1..=6).contains(&r) {
        return Err("bad stage1_bilat params".into());
    }
    let w = w as u32;
    let h = h as u32;
    let r = r as u32;
    let n = (w as usize) * (h as usize);

    // SAFETY（调用方保证）：src 可读 n*3、dst 可写 n*3、coef 可读 9。
    let src_s: &[f32] = unsafe { std::slice::from_raw_parts(src, n * 3) };
    let coef_f: [f32; 9] = unsafe {
        let mut a = [0.0f32; 9];
        for (i, v) in a.iter_mut().enumerate() {
            *v = *coef.add(i) as f32;
        }
        a
    };
    // 空间表：与 x3f_stage1.c s1_bilateral 入口逐字同式（f64 计算 → f32）。
    let d = (2 * r + 1) as usize;
    let mut spt = vec![0.0f32; d * d];
    let ss = 2.0 * (f64::from(r) / 2.0) * (f64::from(r) / 2.0);
    for dy in -i64::from(r)..=i64::from(r) {
        for dx in -i64::from(r)..=i64::from(r) {
            let d2 = (dx * dx + dy * dy) as f64;
            spt[((dy + i64::from(r)) as usize) * d + (dx + i64::from(r)) as usize] =
                (-(d2 / ss)).exp() as f32;
        }
    }

    with_ctx(|ctx| {
        ctx.check()?;
        let pipeline = ctx.s1bilat_pipeline_for()?;
        let lim = ctx.device.limits();
        let cap = lim
            .max_buffer_size
            .min(u64::from(lim.max_storage_buffer_binding_size));
        let src_size = (n * 3 * 4) as u64;
        if src_size > cap {
            return Err(format!("stage1_bilat buffer over device limit {cap}"));
        }

        let src_buf = mk_storage_buf(&ctx.device, &ctx.queue, "s1b src", src_s)?;
        let dst_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("s1b dst"),
            size: src_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let spt_buf = mk_storage_buf(&ctx.device, &ctx.queue, "s1b spt", &spt)?;
        let coef_buf = mk_storage_buf(&ctx.device, &ctx.queue, "s1b coef", &coef_f)?;
        let pu: [u32; 4] = [w, h, r, 0];
        let pf: [f32; 4] = [
            lin_sat as f32,
            level_scale as f32,
            nscale as f32,
            0.0,
        ];
        let pu_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("s1b pu"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        ctx.queue.write_buffer(&pu_buf, 0, bytemuck::cast_slice(&pu));
        let pf_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("s1b pf"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        ctx.queue.write_buffer(&pf_buf, 0, bytemuck::cast_slice(&pf));

        let bind = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("s1b bind"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: src_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: dst_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: spt_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: coef_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 4, resource: pu_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 5, resource: pf_buf.as_entire_binding() },
            ],
        });

        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups(w.div_ceil(16), h.div_ceil(16), 1);
        }
        let readback = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("s1b readback"),
            size: src_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        encoder.copy_buffer_to_buffer(&dst_buf, 0, &readback, 0, src_size);
        ctx.queue.submit([encoder.finish()]);

        let slice = readback.slice(..src_size);
        let (send, recv) = mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |res| {
            let _ = send.send(res.map_err(|e| e.to_string()));
        });
        wait_map(ctx, recv)?;
        // SAFETY: dst 可写 n*3 个 f32（见上）。
        unsafe {
            let mapped = slice.get_mapped_range();
            readback_f32_par(&mapped, dst, n * 3);
        }
        readback.unmap();
        Ok(())
    })
}
