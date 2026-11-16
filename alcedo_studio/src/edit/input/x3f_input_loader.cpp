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

/// Develop reads the camera profile from the document (bound at import), but
/// PreparedRawInput still carries the same matrices for lens calibration,
/// diagnostics and any consumer that reads the decode-side context.
void FillColorContext(PreparedRawInput& input, const X3fColorProfile& profile) {
  auto& ctx                          = input.color_context;
  ctx.valid_                         = true;
  ctx.output_in_camera_space_        = true;
  ctx.camera_make_                   = "SIGMA";
  ctx.cam_mul_[0] = ctx.cam_mul_[1] = ctx.cam_mul_[2] = 1.0f;
  ctx.pre_mul_[0] = ctx.pre_mul_[1] = ctx.pre_mul_[2] = 1.0f;
  // Develop consumes color_matrix_1_ in the DNG ColorMatrix convention
  // (XYZ -> camera) and inverts it to obtain camera -> XYZ. The decoder hands
  // us cam_to_xyz, so the inverse direction is the one to store; passing
  // cam_to_xyz here makes Develop invert twice and skews every hue.
  for (int i = 0; i < 9; ++i) {
    ctx.cam_xyz_[i]        = static_cast<float>(profile.cam_to_xyz[i]);
    ctx.color_matrix_1_[i] = profile.xyz_to_cam[i];
    ctx.color_matrix_2_[i] = profile.xyz_to_cam[i];
  }
  ctx.color_matrices_valid_          = true;
  ctx.calibration_illuminants_valid_ = false;
  ctx.color_matrix_1_cct_            = 6504.0;
  ctx.color_matrix_2_cct_            = 6504.0;
  ctx.as_shot_neutral_valid_         = true;
  // The decoder emits pixels whose white balance is already carried by
  // cam_to_xyz, so unit camera RGB is NOT the neutral: the camera-space white
  // point is xyz_to_cam * D65. Naming that neutral lets Develop's as-shot
  // solve land on D65 with unit gain instead of inventing a second balance
  // (which used to read unit RGB as ~2000 K and drag the whole frame green).
  {
    constexpr double kD65[3] = {0.95047, 1.0, 1.08883};
    double           neutral[3];
    for (int r = 0; r < 3; ++r) {
      neutral[r] = profile.xyz_to_cam[r * 3 + 0] * kD65[0] +
                   profile.xyz_to_cam[r * 3 + 1] * kD65[1] +
                   profile.xyz_to_cam[r * 3 + 2] * kD65[2];
    }
    const double g = neutral[1] > 1e-9 ? neutral[1] : 1.0;
    ctx.as_shot_neutral_[0] = neutral[0] / g;
    ctx.as_shot_neutral_[1] = 1.0;
    ctx.as_shot_neutral_[2] = neutral[2] / g;
  }
  ctx.forward_matrices_valid_ = false;
  // dng_profile_ stays an empty (unbound) reference: X3F has no DNG profile and
  // DngColorProfileRef::RequireBound() must not throw for such a reference.
}

}  // namespace

auto LoadX3fEncoded(std::span<const std::byte> encoded, DecodeRes decode_res)
    -> PreparedRawInput {
  // The decode itself is always full quality (AGENTS.md forbids any degraded
  // fallback such as the embedded JPEG); decode_res only controls the LOD of the
  // resulting host plane, exactly like the LibRaw path.
  const auto decoded = DecodeEncoded(encoded);
  if (!decoded || !decoded->rgb) {
    throw std::runtime_error(std::string("X3fInputLoader: scene-linear decode failed: ") +
                             "unsupported frame, missing calibration, or singular matrix");
  }
  const auto  width  = static_cast<std::uint32_t>(decoded->width);
  const auto  height = static_cast<std::uint32_t>(decoded->height);
  const auto  pixels = static_cast<std::size_t>(width) * height;
  auto        owner  = std::shared_ptr<float[]>(new float[pixels * 4]);
  float*      dst    = owner.get();
  const auto* src    = decoded->rgb.get();
  for (std::size_t i = 0; i < pixels; ++i) {
    // Deliberately no clamp: these are camera-native extended-linear values.
    dst[i * 4 + 0] = src[i * 3 + 0];
    dst[i * 4 + 1] = src[i * 3 + 1];
    dst[i * 4 + 2] = src[i * 3 + 2];
    dst[i * 4 + 3] = 1.0f;
  }

  bool mirror    = false;
  int  flip      = FlipForOrientation(decoded->orientation, mirror);
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
  FillColorContext(input, decoded->profile);

  // Raster/LibRaw LoadEncoded paths key their content by the encoded bytes.
  // FromDirectRgb hashes the pixels instead, so realign both keys here or the
  // prepared-source cache and document content key would disagree on identity.
  const auto hash                          = HashContentBytes(encoded);
  input.content_key.content_hash           = hash;
  input.source_key.encoded_content_hash    = hash;
  input.source_key.encoded_byte_count      = encoded.size();
  return input;
}

}  // namespace alcedo::x3f
