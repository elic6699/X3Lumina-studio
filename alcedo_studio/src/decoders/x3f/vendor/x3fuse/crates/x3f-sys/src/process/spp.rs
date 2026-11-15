//! Merrill SPP export setup and conversion.

use super::*;
use crate::spp_denoise::params::{estimated_iso, NoiseParams, ISO_REFERENCE};
use crate::spp_denoise::pipeline::Pipeline;

#[cfg(test)]
mod image_tests;
#[cfg(test)]
mod prefilter_tests;

pub(super) fn validate_mode(
    encoding: x3f_color_encoding_t,
    cineon: bool,
    denoise: i32,
    backend: crate::DenoiseBackend,
    merrill: bool,
    resolution: &str,
) -> crate::Result<()> {
    if !merrill
        || resolution != "HI"
        || cineon
        || !matches!(
            encoding,
            x3f_color_encoding_e_SRGB | x3f_color_encoding_e_ARGB | x3f_color_encoding_e_PPRGB
        )
    {
        return Err(crate::Error::InvalidData(
            "SPP denoise requires a full-resolution Merrill color export",
        ));
    }
    if denoise != 10 {
        return Err(crate::Error::InvalidData(
            "SPP denoise requires full intensity and neutral noise sliders",
        ));
    }
    if backend == crate::DenoiseBackend::Gpu {
        return Err(crate::Error::Gpu(
            "SPP denoise requires the CPU backend".into(),
        ));
    }
    Ok(())
}

unsafe fn typed_scalar(x3f: *mut x3f_t, name: &CStr, kind: u32, default: f64) -> f64 {
    let entry = unsafe { crate::meta::camf_find_by_name(x3f, name.as_ptr()) };
    if entry.is_none_or(|e| e.matrix_type != kind || e.matrix_dim != 1 || e.matrix_elements != 1) {
        return default;
    }
    match kind {
        3 => {
            let mut value = default;
            unsafe {
                x3f_get_camf_float(x3f, name.as_ptr().cast_mut(), &mut value);
            }
            f64::from(value as f32)
        }
        0 => {
            let mut value = default as i32;
            unsafe {
                x3f_get_camf_signed(x3f, name.as_ptr().cast_mut(), &mut value);
            }
            f64::from(value)
        }
        _ => {
            let mut value = default as u32;
            unsafe {
                x3f_get_camf_unsigned(x3f, name.as_ptr().cast_mut(), &mut value);
            }
            f64::from(value)
        }
    }
}

unsafe fn scalar(x3f: *mut x3f_t, name: &CStr, default: i32) -> i32 {
    let Some(entry) = (unsafe { crate::meta::camf_find_by_name(x3f, name.as_ptr()) }) else {
        return default;
    };
    match entry.matrix_type {
        1 | 2 | 5 | 6 => unsafe {
            typed_scalar(x3f, name, entry.matrix_type, f64::from(default)) as u32 as i32
        },
        3 => unsafe { typed_scalar(x3f, name, 3, f64::from(default)) as i32 },
        _ => default,
    }
}

unsafe fn float_rows<const N: usize>(
    x3f: *mut x3f_t,
    name: &CStr,
    maximum: usize,
) -> crate::Result<Vec<[f32; N]>> {
    let entry = unsafe { crate::meta::camf_find_by_name(x3f, name.as_ptr()) };
    if entry.is_none_or(|e| e.matrix_type != 3) {
        return Ok(Vec::new());
    }
    let (mut rows, mut cols, mut data) = (0, 0, ptr::null_mut());
    if unsafe {
        x3f_get_camf_matrix_var(
            x3f,
            name.as_ptr().cast_mut(),
            &mut rows,
            &mut cols,
            ptr::null_mut(),
            matrix_type_t_M_FLOAT,
            &mut data,
        )
    } == 0
        || cols as usize != N
        || rows <= 0
        || rows as usize > maximum
    {
        return Err(crate::Error::InvalidData("invalid SPP calibration table"));
    }
    Ok(
        unsafe { std::slice::from_raw_parts(data.cast::<f64>(), rows as usize * N) }
            .chunks_exact(N)
            .map(|r| std::array::from_fn(|c| r[c] as f32))
            .collect(),
    )
}

unsafe fn estimate(x3f: *mut x3f_t, gain: [f64; 3]) -> crate::Result<f64> {
    let (mut rows, mut cols, mut hist) = (0, 0, ptr::null_mut());
    let entry = unsafe { crate::meta::camf_find_by_name(x3f, c"Histogram".as_ptr()) };
    if entry.is_none_or(|e| e.matrix_type != 5) {
        return Ok(0.);
    }
    if unsafe {
        x3f_get_camf_matrix_var(
            x3f,
            c"Histogram".as_ptr().cast_mut(),
            &mut rows,
            &mut cols,
            ptr::null_mut(),
            matrix_type_t_M_UINT,
            &mut hist,
        )
    } == 0
        || rows < 3
        || !(1..=65536).contains(&cols)
    {
        return Err(crate::Error::InvalidData("invalid SPP ISO histogram"));
    }
    let histogram: Vec<_> =
        unsafe { std::slice::from_raw_parts(hist.cast::<u32>(), cols as usize * 3) }
            .iter()
            .map(|&v| v as u8)
            .collect();
    let (mut length, mut percent) = (0, ptr::null_mut());
    let entry = unsafe { crate::meta::camf_find_by_name(x3f, c"HistPercent".as_ptr()) };
    if entry.is_none_or(|e| e.matrix_type != 3) {
        return Ok(0.);
    }
    if unsafe {
        x3f_get_camf_matrix_var(
            x3f,
            c"HistPercent".as_ptr().cast_mut(),
            &mut length,
            ptr::null_mut(),
            ptr::null_mut(),
            matrix_type_t_M_FLOAT,
            &mut percent,
        )
    } == 0
        || length != cols
    {
        return Err(crate::Error::InvalidData(
            "invalid SPP histogram percentiles",
        ));
    }
    let percentages: Vec<_> =
        unsafe { std::slice::from_raw_parts(percent.cast::<f64>(), length as usize) }
            .iter()
            .map(|&v| v as f32)
            .collect();
    let reference = unsafe { float_rows::<3>(x3f, c"EstimatedISOreference", 15) }?;
    let reference: Vec<_> = reference.iter().map(|r| r.map(f64::from)).collect();
    let reference = if reference.is_empty() {
        &ISO_REFERENCE[..]
    } else {
        &reference
    };
    let outliers = unsafe { scalar(x3f, c"EstimatedISOoutliers", 1) };
    let count = unsafe { scalar(x3f, c"EstimatedISOoutliersNum", 3) };
    if count < 0 {
        return Err(crate::Error::InvalidData("invalid SPP ISO outlier count"));
    }
    estimated_iso(
        &histogram,
        &percentages,
        reference,
        gain,
        unsafe { typed_scalar(x3f, c"HistBitDepth", 6, 0.) } as u32,
        unsafe { scalar(x3f, c"EstimatedISOWBGains", 1) } != 0,
        (outliers != 0).then_some(count as usize),
        unsafe { scalar(x3f, c"EstimatedISOsnap", 0) } != 0,
    )
}

unsafe fn noise_params(
    x3f: *mut x3f_t,
    gain: [f64; 3],
    exif: &crate::jpeg_exif::JpegExif,
    nr_level: i32,
) -> crate::Result<NoiseParams> {
    let float = |name, default| unsafe { typed_scalar(x3f, name, 3, default) as f32 };
    let int = |name, default| {
        let value = unsafe { typed_scalar(x3f, name, 0, f64::from(default)) };
        if !(0. ..=f64::from(u32::MAX)).contains(&value) {
            Err(crate::Error::InvalidData(
                "invalid SPP integer noise setting",
            ))
        } else {
            Ok(value as u32)
        }
    };
    let mut capture = float(c"CaptureISO", 0.);
    if capture == 0. {
        let mut property = ptr::null_mut();
        if unsafe { x3f_get_prop_entry(x3f, c"ISO".as_ptr().cast_mut(), &mut property) } != 0
            && !property.is_null()
        {
            capture = unsafe { CStr::from_ptr(property) }
                .to_str()
                .ok()
                .and_then(|s| s.parse::<u32>().ok())
                .unwrap_or(0) as f32;
        }
    }
    let sensor = float(c"SensorISO", 0.);
    let sensor = if sensor == 0. { 100. } else { sensor };
    let iso_range = unsafe { typed_scalar(x3f, c"ISOrange", 6, 0.) } != 0.;
    let adjusted_capture = if iso_range {
        match capture {
            800. => 666.,
            1600. => 1200.,
            3200. => 2133.,
            6400. => 4266.,
            _ => capture,
        }
    } else {
        capture
    };
    let iso_gain = adjusted_capture / sensor;
    if !iso_gain.is_finite() || iso_gain <= 0. {
        return Err(crate::Error::InvalidData("invalid SPP capture ISO"));
    }
    let mut estimated = unsafe { estimate(x3f, gain) }? as f32;
    if estimated == 0. {
        estimated = adjusted_capture;
    }
    let (mut count, mut ambient) = (0, ptr::null_mut());
    let entry = unsafe { crate::meta::camf_find_by_name(x3f, c"AESensorValueAmbient".as_ptr()) };
    let mean = if entry.is_none_or(|e| e.matrix_type != 0) {
        120.
    } else {
        if unsafe {
            x3f_get_camf_matrix_var(
                x3f,
                c"AESensorValueAmbient".as_ptr().cast_mut(),
                &mut count,
                ptr::null_mut(),
                ptr::null_mut(),
                matrix_type_t_M_INT,
                &mut ambient,
            )
        } == 0
            || !(1..=65536).contains(&count)
        {
            return Err(crate::Error::InvalidData(
                "invalid SPP ambient exposure calibration",
            ));
        }
        let sum = unsafe { std::slice::from_raw_parts(ambient.cast::<i32>(), count as usize) }
            .iter()
            .fold(0_i32, |sum, &v| sum.wrapping_add(v));
        f64::from(sum) / f64::from(count)
    };
    let actual_ev = (mean - 48.) * 0.125 + 5.;
    let rational = |r: Option<(u32, u32)>| {
        r.filter(|(n, d)| *n != 0 && *d != 0)
            .map_or(1., |(n, d)| f64::from(n) / f64::from(d))
    };
    let f_number = rational(exif.f_number);
    let shutter = rational(exif.exposure_time);
    let reference_ev =
        ((f_number * f_number / shutter).ln() - f64::from(iso_gain.ln())) / std::f64::consts::LN_2;
    let reference_ev = if reference_ev.is_finite() {
        reference_ev
    } else {
        0.
    };
    let exp_estimate = f64::from(float(c"ExpEstimate", 0.));
    let exp_net = f64::from(float(c"ExpNet", 0.));
    let difference = reference_ev - actual_ev;
    let exposure = if actual_ev < f64::from(float(c"EstimatedISOFlashAmbient", 8.))
        && difference > f64::from(float(c"EstimatedISODiffEV", 3.))
    {
        exp_estimate + exp_net + difference.min(3.)
    } else if reference_ev < 0. {
        exp_estimate - (actual_ev - reference_ev) + exp_net
    } else {
        exp_estimate + exp_net
    };
    let adjusted = adjusted_capture / exposure.exp2() as f32;
    if estimated < adjusted {
        estimated = adjusted;
    }
    let iso_rows = unsafe { float_rows::<13>(x3f, c"ISONoiseSettings", 19) }?;
    let additional = unsafe { float_rows::<13>(x3f, c"AdditionalISONoiseSettings", 19) }?;
    let sliders = unsafe { float_rows::<13>(x3f, c"DenoisingSlidersStep", 5) }?;
    let mut params = NoiseParams {
        iso_gain,
        subsample: int(c"SubSampleKernel", 1)? != 0,
        edge_bias: float(c"EdgeMapLinShift", 10.),
        smooth_edge: int(c"EdgeMapDoSmoothEdge", 1)? != 0,
        accumulate: int(c"EdgeMapAccumulate", 0)? != 0,
        local_mean_iters: int(c"LocalMeanIters", 1)?,
        aniso_iters: int(c"ResidualAnisoDiffIters2", 1)?,
        aniso_k: int(c"ResidualAnisoDiffK2", 2)?,
        lambda: float(c"AnisoLambda", 0.25),
        aniso_noise_scale: float(c"ResidualAnisoDiffScaleNM", 1.),
        epsilon: float(c"ResidualEpsilon", 5.),
        perm_half: unsafe { scalar(x3f, c"PermSizeHalf", 4) } as u32,
        sharpen_limit: float(c"SharpenLimit", 0.25),
        fpn_suppression: unsafe { typed_scalar(x3f, c"FPNSuppression", 6, 0.) } as u32,
        ..NoiseParams::default()
    };
    let color = unsafe { float_rows::<3>(x3f, c"ColorNoiseModel", 3) }?;
    if !color.is_empty() {
        if color.len() != 3 {
            return Err(crate::Error::InvalidData("invalid SPP color noise model"));
        }
        params.color_model = std::array::from_fn(|r| color[r].map(f64::from));
    }
    let slider_row = if (0..5).contains(&nr_level) {
        nr_level as usize
    } else {
        2
    };
    params.setup_neutral(
        estimated,
        capture,
        &iso_rows,
        &additional,
        &sliders,
        slider_row,
    )?;
    // GatherParams scales only the color model, after the intensity model is set.
    for (c, &g) in gain.iter().enumerate() {
        params.color_model[0][c] *= f64::from((g * g) as f32);
        params.color_model[1][c] *= f64::from(g as f32);
    }
    Ok(params)
}

fn compose_color_matrix(
    cc: [f64; 9],
    iso_gain: f32,
    compensation: [f64; 9],
) -> crate::Result<[[f64; 3]; 3]> {
    if !iso_gain.is_finite()
        || iso_gain <= 0.
        || !cc.iter().chain(&compensation).all(|v| v.is_finite())
    {
        return Err(crate::Error::InvalidData("invalid SPP color matrix"));
    }
    let matrix = cc.map(|v| v * f64::from(iso_gain));
    let result = std::array::from_fn(|r| {
        std::array::from_fn(|c| {
            if compensation[0] == 0. {
                matrix[3 * r + c]
            } else {
                matrix[3 * r + 2].mul_add(
                    compensation[6 + c],
                    matrix[3 * r + 1].mul_add(
                        compensation[3 + c],
                        matrix[3 * r].mul_add(compensation[c], 0.),
                    ),
                )
            }
        })
    });
    if !result.iter().flatten().all(|v| v.is_finite()) {
        return Err(crate::Error::InvalidData(
            "invalid SPP composed color matrix",
        ));
    }
    Ok(result)
}

unsafe fn color_matrix(
    x3f: *mut x3f_t,
    wb: *mut libc::c_char,
    iso_gain: f32,
) -> crate::Result<[[f64; 3]; 3]> {
    let mode = merrill_render_color_mode(&unsafe { (*x3f).header.color_mode.map(|b| b as u8) })
        .ok_or(crate::Error::InvalidData(
            "SPP monochrome color modes are unsupported",
        ))?;
    let mut cc = [0.; 9];
    let mut found = unsafe {
        x3f_get_camf_matrix_for_wb(
            x3f,
            c"WhiteBalanceColorCorrections".as_ptr().cast_mut(),
            wb,
            3,
            3,
            cc.as_mut_ptr(),
        )
    };
    if found == 0 && !wb.is_null() && unsafe { CStr::from_ptr(wb) } == c"Sunlight" {
        found = unsafe {
            x3f_get_camf_matrix_for_wb(
                x3f,
                c"WhiteBalanceColorCorrections".as_ptr().cast_mut(),
                c"Daylight".as_ptr().cast_mut(),
                3,
                3,
                cc.as_mut_ptr(),
            )
        };
    }
    if found == 0 {
        return Err(crate::Error::InvalidData(
            "missing SPP white balance color matrix",
        ));
    }
    let mut compensation = [0.; 9];
    let (mut names, mut values, mut count) = (ptr::null_mut(), ptr::null_mut(), 0);
    unsafe {
        x3f_get_camf_property_list(
            x3f,
            c"ColorModeCompensations".as_ptr().cast_mut(),
            &mut names,
            &mut values,
            &mut count,
        );
    }
    unsafe {
        x3f_get_camf_matrix_for_wb(
            x3f,
            c"ColorModeCompensations".as_ptr().cast_mut(),
            mode.as_ptr().cast_mut(),
            3,
            3,
            compensation.as_mut_ptr(),
        );
    }
    if count != 0 && mode == c"FCBlue" && compensation[0] == 0. {
        // Native Stage2 supplies model-specific FCBlue matrices when CAMF has none.
        let bits = if unsafe { merrill_count_scale(x3f) } == Some(2.) {
            [
                0x3ff8d42c3c9eecc0,
                0x3fd46edaa92e6213,
                0xbfebdfce3150dae4,
                0x3fd69d2bf5515055,
                0x3ff6160956c0d6f5,
                0xbfe77aa8a82a5615,
                0x3fc08ad688377186,
                0x3fb73b13d244dbd2,
                0x3fe8f5e84f09528f,
            ]
        } else {
            [
                0x3ff9b97785729b28,
                0x3fd2a44417870542,
                0xbfecc5197a24894c,
                0x3fd8915bdd76683c,
                0x3ff5d78811b1d92b,
                0xbfe7f7b9e060fe48,
                0x3fc2fbe33acd5b68,
                0x3fb4ca0c282c6ef4,
                0x3fe8a7c7c5260f5e,
            ]
        };
        compensation = bits.map(f64::from_bits);
    }
    compose_color_matrix(cc, iso_gain, compensation)
}

unsafe fn selected_color_matrix(
    x3f: *mut x3f_t,
    wb: *mut libc::c_char,
    iso_gain: f32,
    mode: Option<&crate::ColorModeParameters>,
) -> crate::Result<[[f64; 3]; 3]> {
    let Some(mode) = mode else {
        return unsafe { color_matrix(x3f, wb, iso_gain) };
    };
    let (cc, _, _) =
        unsafe { color_mode_base_calibration(x3f, wb, crate::ColorModeSensor::Merrill) }?;
    compose_color_matrix(cc, iso_gain, mode.creative_matrix.unwrap_or([0.0; 9]))
}

pub(super) fn active_in_keep(
    active: [i32; 4],
    keep: [i32; 4],
    size: [usize; 2],
) -> crate::Result<[usize; 4]> {
    let invalid = || crate::Error::InvalidData("invalid SPP image rectangles");
    if (0..2).any(|i| i64::from(keep[i + 2]) - i64::from(keep[i]) != size[i] as i64) {
        return Err(invalid());
    }
    let mut result = [0; 4];
    for i in 0..4 {
        result[i] = usize::try_from(i64::from(active[i]) - i64::from(keep[i % 2]))
            .map_err(|_| invalid())?;
    }
    if result[0] >= result[2]
        || result[1] >= result[3]
        || result[2] > size[0]
        || result[3] > size[1]
    {
        return Err(invalid());
    }
    Ok(result)
}

fn zeroed<T: Default + Clone>(length: usize) -> crate::Result<Vec<T>> {
    let mut result = Vec::new();
    result
        .try_reserve_exact(length)
        .map_err(|_| crate::Error::Allocation)?;
    result.resize(length, T::default());
    Ok(result)
}

fn sub_size(size: [usize; 2], scale: f32) -> [usize; 2] {
    size.map(|n| ((f64::from(n as f32) * f64::from(scale)).ceil() as usize + 1) & !1)
}

fn prefilter(
    source: &[f32],
    source_stride: usize,
    size: [usize; 2],
    noise: &NoiseParams,
    pipeline: &Pipeline,
    control: Control<'_>,
) -> crate::Result<Vec<f32>> {
    use crate::spp_denoise::pixmap::{
        bilateral_filter_rgb_image, local_mean, BilateralImageParams,
    };
    use crate::spp_denoise::{compute_edge_map, compute_resample_params, resample_image};
    if noise.alpha != 1. {
        return Err(crate::Error::InvalidData(
            "SPP prefilter requires neutral alpha",
        ));
    }
    let stride = size[0].checked_mul(3).ok_or(crate::Error::Allocation)?;
    let samples = stride
        .checked_mul(size[1])
        .ok_or(crate::Error::Allocation)?;
    let mut down = zeroed(samples)?;
    let mut filtered = zeroed(samples)?;
    control.check()?;
    resample_image(
        source,
        source_stride,
        &mut down,
        size,
        stride,
        &compute_resample_params(noise.scale),
    )?;
    control.check()?;
    bilateral_filter_rgb_image(
        &down,
        stride,
        &mut filtered,
        stride,
        size,
        &BilateralImageParams {
            filter_size: noise.filter_size,
            noise_model: noise.color_model,
            subsample: noise.subsample,
            exp_lut: pipeline.exp_lut(),
            lut_scale: pipeline.exp_lut_scale(),
            rows: [0, size[1] as u32 - 1],
        },
        control,
    )?;
    let mut edge = zeroed(size[0] * size[1])?;
    compute_edge_map(
        &filtered,
        stride,
        &mut edge,
        size,
        size[0],
        &pipeline.edge_params(),
    )?;
    pipeline.remap_edge_map(&mut edge, size[0], size, 0)?;
    control.check()?;
    down.copy_from_slice(&filtered);
    local_mean(
        &down,
        stride,
        &edge,
        size[0],
        size,
        4,
        &mut filtered,
        stride,
    )?;
    control.check()?;
    Ok(filtered)
}

fn denoise_image(
    data: &mut [f32],
    stride: usize,
    size: [usize; 2],
    active: [usize; 4],
    noise: &NoiseParams,
    pipeline: &Pipeline,
    control: Control<'_>,
    // SPP "Sharpness" slider override. `None` (or NaN) keeps the native,
    // denoise-calibrated sharpen amount. A value of 0.0 also reduces to the
    // native amount, matching SPP's neutral sharpness.
    sharpness: Option<f64>,
) -> crate::Result<()> {
    use crate::spp_denoise::pipeline::TileInput;
    use crate::spp_denoise::tiling::{copy_image_to_tile, TileLayout};
    let layout = TileLayout::new(size, [8, 8], [40, 40])?;
    let reduced_size = sub_size(size, noise.scale);
    let filtered = if f64::from(noise.scale) <= 0.54 {
        Some(prefilter(
            data,
            stride,
            reduced_size,
            noise,
            pipeline,
            control,
        )?)
    } else {
        None
    };
    control.check()?;
    let mut tiles = layout.initialize(data, stride)?;
    let sub_tile = sub_size(layout.tile, noise.scale);
    let halo = layout
        .halo
        .map(|h| (h as f64 * f64::from(noise.scale)) as usize);
    let process_tile = |(index, tile): (usize, &mut [f32])| -> crate::Result<Vec<i16>> {
        control.check()?;
        let tile_row = index / layout.grid[0];
        let first = if tile_row == 0 { active[1] } else { 0 };
        let last = if tile_row + 1 == layout.grid[1] {
            layout.tile[1]
                .checked_sub(size[1] - active[3])
                .ok_or(crate::Error::InvalidData("invalid SPP active tile rows"))?
        } else {
            layout.tile[1]
        };
        let extracted;
        let input = if let Some(filtered) = &filtered {
            let mut tile = zeroed(sub_tile[0] * sub_tile[1] * 3)?;
            copy_image_to_tile(
                filtered,
                reduced_size[0] * 3,
                reduced_size,
                layout.grid,
                halo,
                index,
                &mut tile,
                sub_tile[0] * 3,
                sub_tile,
            )?;
            extracted = tile;
            TileInput::Prefiltered {
                rgb: &extracted,
                stride: sub_tile[0] * 3,
            }
        } else {
            TileInput::Ordinary
        };
        let mono = pipeline.process_tile(
            tile,
            layout.tile[0] * 3,
            layout.tile,
            first..last,
            input,
            control,
        )?;
        let mut sharp: Vec<i16> = zeroed(layout.core[0] * layout.core[1])?;
        for y in 0..layout.core[1] {
            for x in 0..layout.core[0] {
                let value = mono[(y + layout.halo[1]) * layout.tile[0] + x + layout.halo[0]];
                let value = if value > 8191. { 8191. } else { value };
                sharp[y * layout.core[0] + x] = f64::from(value).mul_add(4., 0.5) as i64 as i16;
            }
        }
        Ok(sharp)
    };
    #[cfg(not(target_arch = "wasm32"))]
    let sharp_tiles: Vec<_> = {
        use rayon::prelude::*;
        tiles
            .par_chunks_mut(layout.samples)
            .enumerate()
            .map(process_tile)
            .collect::<crate::Result<_>>()?
    };
    #[cfg(target_arch = "wasm32")]
    let sharp_tiles: Vec<_> = tiles
        .chunks_mut(layout.samples)
        .enumerate()
        .map(process_tile)
        .collect::<crate::Result<_>>()?;
    control.check()?;
    layout.blend(&tiles, data, stride, 4)?;
    drop(tiles);
    drop(filtered);
    let mut sharp = zeroed(size[0] * size[1])?;
    for (index, tile) in sharp_tiles.into_iter().enumerate() {
        for y in 0..layout.core[1] {
            let from = y * layout.core[0];
            let to = (index / layout.grid[0] * layout.core[1] + y) * size[0]
                + index % layout.grid[0] * layout.core[0];
            sharp[to..to + layout.core[0]].copy_from_slice(&tile[from..from + layout.core[0]]);
        }
    }
    // Native sharpen amount comes from the denoise calibration. The SPP
    // "Sharpness" slider is an additional amount on top of it: 0.0 leaves the
    // native amount intact, positive/negative pushes more/less. The exact
    // slider→amount scale is calibrated against SPP reference samples.
    let native_amount = f64::from(noise.sharpen_amount);
    let sharpen_amount = match sharpness {
        Some(v) if v != 0.0 => {
            const SHARPEN_SLIDER_SCALE: f64 = 0.5;
            (native_amount + v * SHARPEN_SLIDER_SCALE).max(0.0)
        }
        _ => native_amount,
    };
    crate::spp_denoise::residual::sharpen(
        data,
        stride,
        &sharp,
        size[0],
        size,
        sharpen_amount,
        control,
    )?;
    drop(sharp);
    control.check()
}

/// SPP "Fill Light" — Retinex-style local tone correction.
///
/// Ported from the reverse-engineering report (`SPP6_FillLight逆向分析报告.md`,
/// §3/§4.0/§6) and applied in the **linear RGB domain** (`data` values are
/// 0..65535), exactly where SPP applies it (before the tone curve).
///
/// Our linear working domain is 0..65535 while SPP's Retinex works in a
/// 0..32766 domain (`L = clamp(int(0.25R+0.65G+0.10B), 0, 32766)`). We remap
/// the log-luma by 0.5 so the log table, `w25`, `K` and the index formula stay
/// calibrated to SPP's constants; the resulting gain is then applied to the
/// full-scale RGB. This 2× remap is the documented Phase-2 assumption and needs
/// SPP Fill-Light ±2 reference samples to confirm bit-exactness.
///
/// Surround is a two-scale box-blur pyramid (radii ≈ width/80 and width/20,
/// matching the report's black-box measured 1/80, 1/20 scales; the
/// `PyramidFilter` tower with `SmoothDoubleBlend` is the bit-exact target, its
/// phase kernel still unresolved). A z-score noise gate (`zfloor` in the
/// σ/32 domain) suppresses the sky artifact the report attributes to un-denoised
/// z-terms.
fn fill_light_retinex(
    data: &mut [f32],
    stride: usize,
    size: [usize; 2],
    strength: f64,
    control: Control<'_>,
) -> crate::Result<()> {
    let (w, h) = (size[0], size[1]);
    if w == 0 || h == 0 {
        return Ok(());
    }
    let n = w * h;

    // 1. log-luma table: `logTable[i] = (int16)(0.5 + 1024·ln(i + 30))`, i ∈ 0..32767.
    let logtab: Vec<i32> = (0..32768)
        .map(|i| (0.5 + 1024.0 * (i as f64 + 30.0).ln()).round() as i32)
        .collect();

    // 2. per-pixel pLogLum in SPP's 0..32767 domain (our 0..65535 remapped by 0.5).
    let mut p = vec![0f64; n];
    for y in 0..h {
        for x in 0..w {
            let i = y * stride + x * 3;
            let l =
                (0.25 * data[i] as f64 + 0.65 * data[i + 1] as f64 + 0.10 * data[i + 2] as f64)
                    * 0.5;
            let l = l.clamp(0.0, 32767.0);
            p[y * w + x] = logtab[l as usize] as f64;
        }
    }
    control.check()?;

    // 3. two-scale box-blur surround (σ ≈ w/236 and w/59 → radii w/80, w/20).
    let r1 = (w / 80).max(1);
    let r2 = (w / 20).max(1);
    let surround1 = box_blur(&p, w, h, r1);
    let surround2 = box_blur(&p, w, h, r2);
    // Weighted combination of the two scales (0.9·0.65^i weights, normalized).
    let w0 = 0.9f64;
    let w1 = 0.9 * 0.65;
    let wsum = w0 + w1;
    let p_mean: f64 = p.iter().sum::<f64>() / n as f64;
    let w25 = (6761.438964354475 + p_mean) / 2.0 + 3.0;

    // 4. σ = smooth(|p − surround|) / 32  (sqrtLUT domain), with a small floor.
    let mut diff = vec![0f64; n];
    for i in 0..n {
        diff[i] = (p[i] - (w0 * surround1[i] + w1 * surround2[i]) / wsum).abs();
    }
    control.check()?;
    let sigma = box_blur(&diff, w, h, r2); // large-radius smoothing of the abs-deviation
    let zfloor = 0.75f64; // σ/32 domain; SPP compensates for our higher noise
    let sigma_floor = 1.0 / 32.0; // hard floor to avoid divide-by-zero

    // 5. adaptive coefficient K = 32·√(meanStdDev).
    let mean_sigma = sigma.iter().sum::<f64>() / n as f64;
    let k = 32.0 * mean_sigma.sqrt();

    // 6. gain LUT: s' = strength · 2/3, gain(pos) = exp(s'·tanh(pos/A)) (unity 256).
    let sp = strength * 2.0 / 3.0;
    let a = 1024.0 * (sp.abs() + 0.0625).powf(0.25);
    let mut lut = vec![0i16; 65536];
    let clamp_max = (sp.exp() * 256.0).round() as i16;
    lut[0] = 256;
    let mut hi: i16 = 0;
    let mut lo: i16 = 0;
    for idx in 1..=0x8000 {
        if (hi as i32 & 0xFFFF) != clamp_max as i32 {
            let g = (sp * (idx as f64 / a).tanh()).exp();
            hi = (g * 256.0 + 0.5).round() as i16;
            lo = ((1.0 / g) * 256.0 + 0.5).round() as i16;
        }
        lut[idx & 0xFFFF] = hi;
        lut[(-(idx as i32)) as usize & 0xFFFF] = lo;
    }

    // 7. per-pixel index → gain → multiply linear RGB.
    for y in 0..h {
        for x in 0..w {
            let i = y * w + x;
            let sur = (w0 * surround1[i] + w1 * surround2[i]) / wsum;
            let sig = sigma[i].max(sigma_floor);
            let gate = if sig <= zfloor {
                (1.0 - zfloor / sig).clamp(0.0, 1.0)
            } else {
                1.0
            };
            let z = k * (p[i] - sur) / sig * gate;
            // index = z-score term + global tone term −(p − w25); fcvtzs-truncated.
            let idx = (z - p[i] + w25) as i32;
            let gain = lut[(idx as u32 & 0xFFFF) as usize] as f64 / 256.0;
            let o = y * stride + x * 3;
            for c in 0..3 {
                data[o + c] = (data[o + c] as f64 * gain).clamp(0.0, 65535.0) as f32;
            }
        }
        if y % 64 == 0 {
            control.check()?;
        }
    }
    Ok(())
}

/// Separable box blur with edge clamping. `src` is row-major `w`×`h`.
/// Uses per-line prefix sums so the cost is O(w·h) regardless of the radius.
fn box_blur(src: &[f64], w: usize, h: usize, r: usize) -> Vec<f64> {
    if w == 0 || h == 0 {
        return Vec::new();
    }
    let r = r.min(w / 2).min(h / 2).max(1);
    let mut tmp = vec![0f64; w * h];
    // Horizontal pass: row prefix sums → O(1) sliding window.
    for y in 0..h {
        let base = y * w;
        let row = &src[base..base + w];
        let mut pref = vec![0f64; w + 1];
        for i in 0..w {
            pref[i + 1] = pref[i] + row[i];
        }
        for x in 0..w {
            let lo = (x as isize - r as isize).max(0) as usize;
            let hi = (x + r).min(w - 1);
            tmp[base + x] = (pref[hi + 1] - pref[lo]) / (hi - lo + 1) as f64;
        }
    }
    // Vertical pass.
    let mut out = vec![0f64; w * h];
    for x in 0..w {
        let mut pref = vec![0f64; h + 1];
        for y in 0..h {
            pref[y + 1] = pref[y] + tmp[y * w + x];
        }
        for y in 0..h {
            let lo = (y as isize - r as isize).max(0) as usize;
            let hi = (y + r).min(h - 1);
            out[y * w + x] = (pref[hi + 1] - pref[lo]) / (hi - lo + 1) as f64;
        }
    }
    out
}

/// The expensive half of an SPP export: the float front end and the SPP
/// noise reduction, but without the tone curve.
///
/// The host keeps one of these alive while the user drags the tone sliders.
/// On a 24 MP body the work in here costs ~4.8 s (the noise reduction alone
/// ~3.4 s) while the tone stage in [`apply_tone`] costs ~0.1 s, so splitting
/// the export in two is what makes the tone sliders feel live. The denoise
/// row is baked in, so a different `nr_level` needs a fresh `Prepared`.
///
/// `x3f` stays borrowed — the caller owns it and must keep it alive for as
/// long as the buffer is used, because the tone stage reads the CAMF
/// calibration back out of it.
pub(crate) struct Prepared {
    x3f: *mut x3f_t,
    data: Vec<f32>,
    row_stride: usize,
    size: [usize; 2],
    active: [usize; 4],
    crop: bool,
    nr_level: i32,
    exif: crate::jpeg_exif::JpegExif,
    encoding: x3f_color_encoding_t,
    mode: Option<crate::ColorModeParameters>,
}

impl Prepared {
    /// The output rectangle: the active area when cropping, the whole sensor
    /// frame otherwise.
    fn rect(&self) -> [usize; 4] {
        if self.crop {
            self.active
        } else {
            [0, 0, self.size[0], self.size[1]]
        }
    }

    /// Output geometry in pixels.
    pub(crate) fn output_size(&self) -> (usize, usize) {
        let rect = self.rect();
        (rect[2] - rect[0], rect[3] - rect[1])
    }

    /// The denoise row this buffer was reduced with. A tone pass that asks
    /// for a different row has to be refused: the noise reduction itself
    /// depends on the row, not just the tone curve.
    pub(crate) fn nr_level(&self) -> i32 {
        self.nr_level
    }
}

/// Run the expensive half of the SPP export.
#[allow(clippy::too_many_arguments)]
pub(super) unsafe fn prepare(
    x3f: *mut x3f_t,
    apply_sgain: bool,
    fix_bad: bool,
    crop: bool,
    encoding: x3f_color_encoding_t,
    wb: *mut libc::c_char,
    control: Control<'_>,
    mode: Option<&crate::ColorModeParameters>,
    // SPP "Fill Light" slider (−2..+2). NaN or 0 keeps the native path. Baked
    // into the prepared buffer (Retinex runs in the linear domain), so a
    // different value needs a fresh `Prepared`.
    fill_light: f64,
    // SPP "Sharpness" slider (−2..+2). NaN keeps the native denoise-derived
    // sharpen amount. Added on top of the native sharpening; also baked into
    // the prepared buffer.
    sharpness: f64,
    nr_level: i32,
) -> crate::Result<Prepared> {
    let gain = unsafe { merrill_request_gain(x3f, wb, control) }
        .ok_or(crate::Error::InvalidData("unsupported SPP request gains"))?;
    let exif = unsafe { merrill_exif(x3f, control) }?;
    let noise = unsafe { noise_params(x3f, gain, &exif, nr_level) }?;
    let matrix = unsafe { selected_color_matrix(x3f, wb, noise.iso_gain, mode) }?;
    let sensor = unsafe { typed_scalar(x3f, c"SensorISO", 3, 0.) } as f32;
    let sensor_scale = if sensor == 0. { 1. } else { sensor / 100. };
    let mut strengths = [0.; 3];
    if unsafe {
        x3f_get_camf_float_vector(
            x3f,
            c"ColorDQCamRGB".as_ptr().cast_mut(),
            strengths.as_mut_ptr(),
        )
    } == 0
    {
        return Err(crate::Error::InvalidData("missing SPP ColorDQ calibration"));
    }
    let mut skip = 0;
    unsafe {
        x3f_get_camf_unsigned(x3f, c"SkipStepFlags".as_ptr().cast_mut(), &mut skip);
    }
    if skip & (1 << 23) != 0 || noise.fpn_suppression != 0 {
        return Err(crate::Error::InvalidData(
            "unsupported SPP denoise bypass or FPN mode",
        ));
    }
    let dq_scale = f64::from(noise.color_dq_scale.min(noise.iso_gain));
    let pipeline = Pipeline::new(
        noise.clone(),
        matrix,
        strengths.map(|v| v * dq_scale),
        sensor_scale,
        gain.map(|v| v as f32),
    )?;
    let mut area = x3f_area16_t::default();
    if unsafe { x3f_image_area(x3f, &mut area) } == 0 {
        return Err(crate::Error::InvalidData("missing SPP image"));
    }
    let size = [area.columns as usize, area.rows as usize];
    let (active, keep, _) = unsafe { merrill_rects(x3f) }?;
    let active = active_in_keep(active, keep, size)?;
    let trace = std::env::var_os("X3F_FLOAT_FRONT_TRACE").is_some();
    let mut observe = |stage: &'static str, data: &[f32], detail: String| {
        if trace {
            eprintln!(
                "X3F float front: stage={stage} fnv1a={:016x} {detail}",
                crate::float_front::fnv1a_f32(data)
            );
        }
    };
    let mut prepared = unsafe {
        merrill_prepare_float(
            x3f,
            wb,
            apply_sgain,
            fix_bad,
            Some(noise.despeckle),
            control,
            &mut observe,
        )
    }?;
    denoise_image(
        &mut prepared.data,
        prepared.row_stride,
        size,
        active,
        &noise,
        &pipeline,
        control,
        if sharpness.is_nan() {
            None
        } else {
            Some(sharpness)
        },
    )?;
    // Fill Light (Retinex local tone) runs in the linear domain, after the
    // denoise front end and before the tone curve — exactly where SPP applies
    // it. It is part of the prepared buffer, so changing it needs a re-prepare.
    if !fill_light.is_nan() && fill_light != 0.0 {
        fill_light_retinex(
            &mut prepared.data,
            prepared.row_stride,
            size,
            fill_light,
            control,
        )?;
    }
    Ok(Prepared {
        x3f,
        data: prepared.data,
        row_stride: prepared.row_stride,
        size,
        active,
        crop,
        nr_level,
        exif,
        encoding,
        mode: mode.copied(),
    })
}

/// Apply the tone curve and the output transform to a prepared buffer.
///
/// This is the cheap half: it rebuilds the tone lookup tables and walks the
/// pixels, ~0.1 s on a 24 MP body. The rows are independent, so the walk is
/// parallel — every pixel keeps its own arithmetic, and the result is bit
/// identical to the serial path.
pub(super) unsafe fn apply_tone(
    prepared: &Prepared,
    tone: Option<&SppTone>,
    control: Control<'_>,
    output: &mut [u16],
) -> crate::Result<()> {
    let luts = unsafe {
        make_color_mode_render_luts(
            prepared.x3f,
            prepared.encoding,
            &prepared.exif,
            true,
            prepared.mode.as_ref(),
            tone,
            control,
        )
    }?;
    let rect = prepared.rect();
    let width = rect[2] - rect[0];
    let height = rect[3] - rect[1];
    let stride = width * 3;
    let row_stride = prepared.row_stride;
    let data = &prepared.data[..];

    // Post-tone creative overrides, applied after the tone curve + output
    // matrix (the 48-bit RGB domain), matching SPP's ProcessToOutput row
    // template. Both are identity at their neutral defaults, so a zeroed tone
    // keeps the output bit-identical.
    let highlight_control = tone
        .map(|t| t.highlight_control)
        .filter(|v| !v.is_nan())
        .unwrap_or(1.0); // fHighB = 1 - slider; 1.0 is native (no blend).
    let apply_hl = (highlight_control - 1.0).abs() > 1e-9;
    let coladj = tone.map(|t| t.coladj).unwrap_or([f64::NAN; 3]);
    let coladj = if coladj[0].is_nan() {
        [1.0f64; 3]
    } else {
        coladj
    };
    let apply_coladj = coladj.iter().any(|&c| (c - 1.0).abs() > 1e-9);
    {
        use rayon::prelude::*;
        output
            .par_chunks_mut(stride)
            .take(height)
            .enumerate()
            .try_for_each(|(y, row)| {
                control.check()?;
                let from = (rect[1] + y) * row_stride + rect[0] * 3;
                for (out, input) in row
                    .chunks_exact_mut(3)
                    .zip(data[from..][..stride].chunks_exact(3))
                {
                    let mut pix = luts.render(input.try_into().unwrap());
                    if apply_coladj {
                        for c in 0..3 {
                            pix[c] = (pix[c] as f64 * coladj[c]).clamp(0.0, 65535.0) as u16;
                        }
                    }
                    if apply_hl {
                        // Highlight Control (fHighB): blend highlight pixels
                        // toward neutral gray. `t` ramps in over a 12-bit luma
                        // domain (output >> 4); 2432.265625/950 are the SPP
                        // Sigma_HNE thresholds. Documented domain assumption.
                        let y16 = 0.299 * pix[0] as f64
                            + 0.587 * pix[1] as f64
                            + 0.114 * pix[2] as f64;
                        let t = (((y16 / 16.0) - 2432.265625) / 950.0).clamp(0.0, 1.0);
                        let f = highlight_control;
                        for c in 0..3 {
                            let mixed = (1.0 - t) * pix[c] as f64 + t * y16;
                            pix[c] = (pix[c] as f64 * f + (1.0 - f) * mixed).clamp(0.0, 65535.0)
                                as u16;
                        }
                    }
                    out.copy_from_slice(&pix);
                }
                Ok::<(), crate::Error>(())
            })?;
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub(super) unsafe fn render(
    x3f: *mut x3f_t,
    image: *mut x3f_area16_t,
    levels: *mut x3f_image_levels_t,
    encoding: x3f_color_encoding_t,
    crop: bool,
    fix_bad: bool,
    apply_sgain: bool,
    wb: *mut libc::c_char,
    control: Control<'_>,
    mode: Option<&crate::ColorModeParameters>,
    tone: Option<&SppTone>,
) -> crate::Result<()> {
    control.check()?;
    let timing = std::env::var_os("X3F_SPP_TIMING").is_some();
    let t_start = std::time::Instant::now();
    let prepared = unsafe {
        prepare(
            x3f,
            apply_sgain,
            fix_bad,
            crop,
            encoding,
            wb,
            control,
            mode,
            tone.map(|t| t.fill_light).unwrap_or(f64::NAN),
            tone.map(|t| t.sharpness).unwrap_or(f64::NAN),
            tone.map(|t| t.nr_level).unwrap_or(-1),
        )?
    };
    if timing {
        eprintln!(
            "X3F SPP timing: prepare = {:.3} s",
            t_start.elapsed().as_secs_f64()
        );
    }
    let t_tone = std::time::Instant::now();
    let (width, height) = prepared.output_size();
    let stride = width.checked_mul(3).ok_or(crate::Error::Allocation)?;
    let samples = stride.checked_mul(height).ok_or(crate::Error::Allocation)?;
    let bytes = samples
        .checked_mul(std::mem::size_of::<u16>())
        .ok_or(crate::Error::Allocation)?;
    let buffer = unsafe { libc::malloc(bytes) };
    if buffer.is_null() {
        return Err(crate::Error::Allocation);
    }
    // The controlled caller owns this allocation and frees it on any error.
    unsafe {
        *image = x3f_area16_t {
            data: buffer.cast(),
            buf: buffer,
            rows: height as u32,
            columns: width as u32,
            channels: 3,
            row_stride: stride as u32,
        };
    }
    let output = unsafe { std::slice::from_raw_parts_mut(buffer.cast::<u16>(), samples) };
    unsafe { apply_tone(&prepared, tone, control, output) }?;
    if timing {
        eprintln!(
            "X3F SPP timing: apply_tone = {:.3} s",
            t_tone.elapsed().as_secs_f64()
        );
        eprintln!(
            "X3F SPP timing: total = {:.3} s",
            t_start.elapsed().as_secs_f64()
        );
    }
    if !levels.is_null() {
        unsafe {
            *levels = x3f_image_levels_t {
                black: [0.; 3],
                white: [65535; 3],
            };
        }
    }
    control.check()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn active_crop_is_relative_to_keep() {
        assert_eq!(
            active_in_keep([64, 44, 4768, 3180], [8, 8, 4936, 3272], [4928, 3264]).unwrap(),
            [56, 36, 4760, 3172]
        );
        assert!(active_in_keep([0, 0, 20, 20], [8, 8, 40, 40], [32, 32]).is_err());
        assert!(active_in_keep([8, 8, 50, 20], [8, 8, 40, 40], [32, 32]).is_err());
        assert!(active_in_keep([8, 8, 20, 20], [8, 8, 40, 40], [31, 32]).is_err());
        assert!(active_in_keep([20, 20, 8, 8], [8, 8, 40, 40], [32, 32]).is_err());
    }

    #[test]
    fn color_matrix_keeps_native_fused_cancellation() {
        let epsilon = 1. / 134_217_728.;
        let cc = [-1., 1. + epsilon, 0., 0., 0., 0., 0., 0., 0.];
        let compensation = [1., 0., 0., 1. - epsilon, 0., 0., 0., 0., 0.];
        let result = compose_color_matrix(cc, 1., compensation).unwrap();
        assert_eq!(result[0][0].to_bits(), 0xbc90_0000_0000_0000);
        assert_eq!(
            compose_color_matrix(cc, 2., [0.; 9]).unwrap()[0],
            [-2., 2. + 2. * epsilon, 0.]
        );
        assert!(compose_color_matrix(cc, f32::NAN, compensation).is_err());
    }

    #[test]
    fn reference_noise_setup_uses_native_iso_and_scale() {
        let root = std::env::var_os("X3F_TEST_FILES")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| {
                std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../x3f_test_files")
            });
        let mut checked = 0;
        for (name, iso, scale) in [
            ("_P2M0358.X3F", 100_f32, 1_f32),
            ("sigma_sd1_merrill_15.x3f", 800_f32, 0.5_f32),
        ] {
            let path = root.join(name);
            if !path.exists() {
                continue;
            }
            let path = std::ffi::CString::new(path.to_str().unwrap()).unwrap();
            unsafe {
                let file = libc::fopen(path.as_ptr(), c"rb".as_ptr());
                assert!(!file.is_null());
                let x3f = crate::x3f_new_from_file(file.cast());
                assert!(!x3f.is_null());
                for section in [crate::x3f_get_camf(x3f), crate::x3f_get_prop(x3f)] {
                    assert!(!section.is_null());
                    assert_eq!(
                        crate::x3f_load_data(x3f, section),
                        crate::x3f_return_e_X3F_OK
                    );
                }
                let gain = merrill_request_gain(x3f, x3f_get_wb(x3f), Control::none()).unwrap();
                let exif = merrill_exif(x3f, Control::none()).unwrap();
                let params = noise_params(x3f, gain, &exif, -1).unwrap();
                assert_eq!(params.iso.to_bits(), iso.to_bits(), "{name}");
                assert_eq!(params.scale.to_bits(), scale.to_bits(), "{name}");
                assert_eq!(params.filter_size, 21);
                assert_eq!(params.despeckle, 1.);
                assert_eq!(params.fpn_suppression, 0);
                crate::x3f_delete(x3f);
                libc::fclose(file);
                checked += 1;
            }
        }
        if checked == 0 {
            eprintln!("SPP noise setup: skipped, reference corpus is absent");
        }
    }

    #[test]
    fn rejects_unsupported_export_options() {
        use crate::DenoiseBackend::{Auto, Cpu, Gpu};
        for encoding in [
            x3f_color_encoding_e_SRGB,
            x3f_color_encoding_e_ARGB,
            x3f_color_encoding_e_PPRGB,
        ] {
            for backend in [Auto, Cpu] {
                assert!(validate_mode(encoding, false, 10, backend, true, "HI").is_ok());
            }
        }
        for resolution in ["", "MED", "LOW"] {
            assert!(
                validate_mode(x3f_color_encoding_e_SRGB, false, 10, Cpu, true, resolution).is_err()
            );
        }
        for encoding in [
            x3f_color_encoding_e_NONE,
            x3f_color_encoding_e_UNPROCESSED,
            x3f_color_encoding_e_QTOP,
        ] {
            assert!(validate_mode(encoding, false, 10, Auto, true, "HI").is_err());
        }
        assert!(validate_mode(x3f_color_encoding_e_SRGB, true, 10, Cpu, true, "HI").is_err());
        assert!(validate_mode(x3f_color_encoding_e_SRGB, false, 10, Cpu, false, "HI").is_err());
        assert!(validate_mode(x3f_color_encoding_e_SRGB, false, 0, Cpu, true, "HI").is_err());
        assert!(validate_mode(x3f_color_encoding_e_SRGB, false, 9, Cpu, true, "HI").is_err());
        assert!(matches!(
            validate_mode(x3f_color_encoding_e_SRGB, false, 10, Gpu, true, "HI"),
            Err(crate::Error::Gpu(_))
        ));
    }
}
