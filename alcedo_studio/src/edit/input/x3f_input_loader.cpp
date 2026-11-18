//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "edit/input/x3f_input_loader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "decoders/x3f/x3f_foveon.hpp"
#include "decoders/x3f/x3f_support.hpp"
#include "edit/input/raw_input_loader.hpp"

namespace alcedo::x3f {
namespace {

/// Mirrors RasterInputLoader::FlipForOrientation. EXIF orientation 1..8 is
/// applied once: rotation/transposition by the GPU pack step, horizontal mirror
/// here on the CPU (the Metal/CUDA pack kernels have no mirror variant).
auto FlipForOrientation(int orientation, bool& mirror) -> int {
  mirror = false;
  switch (orientation) {
    case 2: mirror = true; return 0;
    case 3: return 3;
    case 4: mirror = true; return 3;
    case 5: mirror = true; return 5;
    case 6: return 6;
    case 7: mirror = true; return 6;
    case 8: return 5;
    default: return 0;
  }
}

void MirrorHorizontally(float* rgba, std::uint32_t width, std::uint32_t height) {
  for (std::uint32_t y = 0; y < height; ++y) {
    auto* row = rgba + static_cast<std::size_t>(y) * width * 4;
    for (std::uint32_t x = 0; x < width / 2; ++x) {
      float*       left  = row + static_cast<std::size_t>(x) * 4;
      float*       right = row + static_cast<std::size_t>(width - 1 - x) * 4;
      float        tmp[4];
      std::memcpy(tmp, left, sizeof(tmp));
      std::memcpy(left, right, sizeof(tmp));
      std::memcpy(right, tmp, sizeof(tmp));
    }
  }
}

/// Colour context for a frame that the Foveon native chain already rendered to
/// unclamped **linear sRGB primaries**. The camera->sRGB matrix and the camera
/// white balance are folded into the pixels, so the host must be told the input
/// primaries are sRGB: colour_matrix_1_ carries XYZ->sRGB (Develop inverts it
/// internally, DNG convention) and the neutral is unit RGB. Composing that with
/// the DAG's working-space transform reproduces camera->XYZ exactly, which is
/// why this feed costs no gamut: the values are not clamped, so out-of-sRGB
/// colours survive as negative components.
constexpr double kSrgbToXyz[9] = {0.4124564, 0.3575761, 0.1804375,
                                  0.2126729, 0.7151522, 0.0721750,
                                  0.0193339, 0.1191920, 0.9503041};
constexpr double kXyzToSrgb[9] = {3.2404542, -1.5371385, -0.4985314,
                                  -0.9692660, 1.8760108, 0.0415560,
                                  0.0556434, -0.2040259, 1.0572252};

void FillLinearSrgbColorContext(PreparedRawInput& input) {
  auto& ctx                   = input.color_context;
  ctx.valid_                  = true;
  ctx.output_in_camera_space_ = true;
  ctx.camera_make_            = "SIGMA";
  ctx.cam_mul_[0] = ctx.cam_mul_[1] = ctx.cam_mul_[2] = 1.0f;
  ctx.pre_mul_[0] = ctx.pre_mul_[1] = ctx.pre_mul_[2] = 1.0f;
  for (int i = 0; i < 9; ++i) {
    ctx.cam_xyz_[i]        = static_cast<float>(kSrgbToXyz[i]);
    ctx.color_matrix_1_[i] = kXyzToSrgb[i];
    ctx.color_matrix_2_[i] = kXyzToSrgb[i];
  }
  ctx.color_matrices_valid_          = true;
  ctx.calibration_illuminants_valid_ = false;
  ctx.color_matrix_1_cct_            = 6504.0;
  ctx.color_matrix_2_cct_            = 6504.0;
  ctx.as_shot_neutral_valid_         = true;
  // The native chain already applied the camera white balance, and its output
  // primaries are sRGB, so unit RGB really is the neutral here.
  ctx.as_shot_neutral_[0] = ctx.as_shot_neutral_[1] = ctx.as_shot_neutral_[2] = 1.0;
  ctx.forward_matrices_valid_ = false;
}

/// DecodeRes -> integer box downsample of the native chain's linear frame.
auto NativeDown(DecodeRes decode_res) -> int {
  switch (decode_res) {
    case DecodeRes::HALF: return 2;
    case DecodeRes::QUARTER: return 4;
    case DecodeRes::EIGHTH: return 8;
    case DecodeRes::FULL:
    default: return 1;
  }
}

}  // namespace

auto LoadX3fEncoded(std::span<const std::byte> encoded, DecodeRes decode_res)
    -> PreparedRawInput {
  // The decode itself is always full quality (AGENTS.md forbids any degraded
  // fallback such as the embedded JPEG); decode_res only controls the LOD of the
  // resulting host plane, exactly like the LibRaw path.
  const auto hash = HashContentBytes(encoded);

  // An X3F always decodes through the Foveon native chain: it is the only place
  // fill light, highlight recovery, detail and noise reduction exist, and it is
  // the chain whose camera->sRGB matrix and white balance are calibrated. The
  // editor's published parameters override the product defaults; with none
  // published (bulk import, thumbnails) the defaults apply, so every consumer
  // sees the same primaries and the document profile stays truthful.
  const auto published = ActiveFoveonParams(hash);
  auto       params    = published.value_or(FoveonEditParams{});
  params.down = std::max(1, NativeDown(decode_res));
  // Fill light keeps the Foveon Retinex path for X3F. ApplicationModuleHost
  // mirrors the Tone panel's fill_light value into FoveonEditParams, so the
  // decode-side gain map carries exactly the slider value. The Develop DAG's
  // LLF stand-in is disabled for this frame via foveon_fill_light_baked below;
  // the two paths are mutually exclusive and the slider is applied once.
  // Every other input (JPEG/TIFF/Bayer) has no decode stage, so it keeps the
  // Grade's universal fill light.
  // The frame itself is kept between calls: fill light, detail, sharpening,
  // highlight recovery, NR and SDQ highlight are all apply-level operators, so
  // dragging one re-runs the apply pass instead of decoding the mosaic again
  // (X3Lumina's linCache behaves the same way and that is why its fill light is
  // instant). Only a decode-level switch prepares a new frame.
  const auto lin = FoveonLinearFeed(encoded, params);
  if (!lin || lin->Empty()) {
    throw std::runtime_error(std::string("X3fInputLoader: Foveon native "
                                         "decode failed: ") +
                             FoveonLastError());
  }
  const auto  width  = static_cast<std::uint32_t>(lin->width);
  const auto  height = static_cast<std::uint32_t>(lin->height);
  const auto  pixels = static_cast<std::size_t>(width) * height;
  auto        owner  = std::shared_ptr<float[]>(new float[pixels * 4]);
  float*      dst    = owner.get();
  const float* src   = lin->rgb.get();
  for (std::size_t i = 0; i < pixels; ++i) {
    // Deliberately no clamp: negatives and >1 are real wide-gamut / highlight
    // headroom and must reach the working space untouched.
    dst[i * 4 + 0] = src[i * 3 + 0];
    dst[i * 4 + 1] = src[i * 3 + 1];
    dst[i * 4 + 2] = src[i * 3 + 2];
    dst[i * 4 + 3] = 1.0f;
  }

  bool mirror = false;
  int  flip   = FlipForOrientation(lin->orientation, mirror);
  if (mirror) {
    MirrorHorizontally(dst, width, height);
  }

  HostImagePlane plane;
  plane.bytes = std::shared_ptr<const std::byte>(
      owner, reinterpret_cast<const std::byte*>(owner.get()));
  plane.extent       = Extent2D{width, height};
  plane.stride_bytes = width * 16U;  // F32Rgba
  plane.format       = HostPixelFormat::F32Rgba;

  RawSensorGeometry sensor;
  sensor.raw_width        = static_cast<std::int32_t>(width);
  sensor.raw_height       = static_cast<std::int32_t>(height);
  sensor.width            = static_cast<std::int32_t>(width);
  sensor.height           = static_cast<std::int32_t>(height);
  sensor.orientation_flip = flip;

  auto input = RawInputLoader::FromDirectRgb(std::move(plane), sensor, decode_res);
  FillLinearSrgbColorContext(input);
  input.content_key.content_hash        = hash;
  input.source_key.encoded_content_hash = hash;
  input.source_key.encoded_byte_count   = encoded.size();
  // The plane records the operators it was decoded with, so the pipeline can
  // tell a default decode from one carrying the editor's sliders even when the
  // parameter registry has moved on since (that race is what made the first
  // preview of a file ignore fill light).
  input.source_key.decode_variant =
      published ? FoveonLinearKey(params) : 0;
  // Non-zero means the Retinex gain map was actually applied to `pixels`. The
  // zero case leaves the Grade free to apply its own fill light (value 0 there
  // too, so byte-identical), and keeps a stale published entry from blaming an
  // un-baked frame on the Grade.
  input.foveon_fill_light_baked = params.fill_light != 0.0;
  return input;
}

}  // namespace alcedo::x3f
