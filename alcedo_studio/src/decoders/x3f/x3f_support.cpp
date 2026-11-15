//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "decoders/x3f/x3f_support.hpp"

#include <x3f_alcedo_export.h>
#include <x3fcore.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>

namespace alcedo::x3f {
namespace {

auto PathText(const std::filesystem::path& path) -> std::string {
  const auto text = path.u8string();
  return std::string(text.begin(), text.end());
}

auto LowerAscii(std::string value) -> std::string {
  for (char& c : value) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return value;
}

/// Adjugate/determinant inverse. A degenerate matrix would collapse the gamut,
/// so callers must fail rather than silently keep a wide-gamut label.
auto Invert3x3(const double m[9], double out[9]) -> bool {
  const double a = m[0], b = m[1], c = m[2];
  const double d = m[3], e = m[4], f = m[5];
  const double g = m[6], h = m[7], i = m[8];
  const double A = e * i - f * h;
  const double B = f * g - d * i;
  const double C = d * h - e * g;
  const double det = a * A + b * B + c * C;
  if (!(std::abs(det) > 1e-12) || !std::isfinite(det)) {
    return false;
  }
  const double t[9] = {A / det, (c * h - b * i) / det, (b * f - c * e) / det,
                       B / det, (a * i - c * g) / det, (c * d - a * f) / det,
                       C / det, (b * g - a * h) / det, (a * e - b * d) / det};
  std::memcpy(out, t, sizeof(t));
  return true;
}

auto ProfileFrom(const x3f_alcedo_linear_t& decoded) -> std::optional<X3fColorProfile> {
  X3fColorProfile profile;
  std::memcpy(profile.cam_to_xyz, decoded.cam_xyz, sizeof(profile.cam_to_xyz));
  if (!Invert3x3(profile.cam_to_xyz, profile.xyz_to_cam)) {
    return std::nullopt;
  }
  profile.model_class  = decoded.model_class;
  profile.decoder_used = decoded.decoder_used;
  return profile;
}

/// The decoder's override is a process global; both probe and render serialize
/// on the C-side mutex, so a shared import cache can be a plain map + lock.
struct ColorProfileCache {
  std::mutex                            mutex;
  std::map<std::string, X3fColorProfile> entries;
};

auto SharedColorProfileCache() -> ColorProfileCache& {
  static ColorProfileCache cache;
  return cache;
}

auto Read16(const std::uint8_t* p, bool little) -> std::uint16_t {
  return little ? static_cast<std::uint16_t>(p[0] | (p[1] << 8))
                : static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}

auto Read32(const std::uint8_t* p, bool little) -> std::uint32_t {
  return little ? (static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
                   (static_cast<std::uint32_t>(p[2]) << 16) |
                   (static_cast<std::uint32_t>(p[3]) << 24))
                : ((static_cast<std::uint32_t>(p[0]) << 24) |
                   (static_cast<std::uint32_t>(p[1]) << 16) |
                   (static_cast<std::uint32_t>(p[2]) << 8) | p[3]);
}

auto ColorProfileKey(const std::filesystem::path& path, int frame) -> std::string {
  std::error_code ec;
  const auto      size = std::filesystem::file_size(path, ec);
  const auto      time = std::filesystem::last_write_time(path, ec);
  return PathText(path) + "|" + std::to_string(frame) + "|" +
         (ec ? "na" : std::to_string(static_cast<std::uint64_t>(size))) + "|" +
         (ec ? "na"
             : std::to_string(static_cast<std::int64_t>(time.time_since_epoch().count())));
}

/// x3fcore returns malloc-owned buffers; never let one leak on an error path.
struct CoreBuffer {
  uint8_t* data = nullptr;
  size_t   size = 0;

  ~CoreBuffer() {
    if (data != nullptr) {
      x3f_free(data);
    }
  }
  CoreBuffer()                       = default;
  CoreBuffer(const CoreBuffer&)      = delete;
  auto operator=(const CoreBuffer&) -> CoreBuffer& = delete;
};

}  // namespace

auto IsX3fFile(const std::filesystem::path& path) -> bool {
  const auto ext = LowerAscii(path.extension().string());
  return ext == ".x3f" || ext == ".x3i";
}

auto Probe(const std::filesystem::path& path, int frame) -> std::optional<ProbeInfo> {
  const std::string text = PathText(path);
  x3fjpeg_info_t    info{};
  if (x3f_probe_frame(text.c_str(), frame, &info) != X3FJPEG_OK) {
    return std::nullopt;
  }
  ProbeInfo out;
  out.is_container = info.is_container != 0;
  out.frame_count  = info.frame_count > 0 ? info.frame_count : 1;
  out.width        = info.width;
  out.height       = info.height;
  out.has_jpeg     = info.has_jpeg != 0;
  out.has_exif     = info.has_exif != 0;
  const auto serial_end =
      std::find(info.serial, info.serial + sizeof(info.serial), '\0');
  out.serial.assign(info.serial, serial_end);
  return out;
}

auto FrameCount(const std::filesystem::path& path) -> int {
  const int count = x3f_frame_count(PathText(path).c_str());
  return count > 0 ? count : 0;
}

auto ReadEmbeddedJpeg(const std::filesystem::path& path, int frame) -> std::vector<uint8_t> {
  CoreBuffer  buffer;
  const auto  text = PathText(path);
  if (x3f_get_jpeg_frame(text.c_str(), frame, &buffer.data, &buffer.size, nullptr) !=
      X3FJPEG_OK) {
    return {};
  }
  if (buffer.data == nullptr || buffer.size == 0) {
    return {};
  }
  return {buffer.data, buffer.data + buffer.size};
}

auto MaterializeFrame(const std::filesystem::path& path, int frame) -> std::vector<uint8_t> {
  CoreBuffer  buffer;
  const auto  text = PathText(path);
  const int   rc   = x3f_frame_read(text.c_str(), frame, &buffer.data, &buffer.size);
  // A plain X3F has no container layer; the caller keeps the original path.
  if (rc != X3FJPEG_OK || buffer.data == nullptr || buffer.size == 0) {
    return {};
  }
  return {buffer.data, buffer.data + buffer.size};
}

auto EmbeddedJpegOrientation(std::span<const std::uint8_t> jpeg) -> int {
  if (jpeg.size() < 4 || jpeg[0] != 0xFF || jpeg[1] != 0xD8) {
    return 1;
  }
  std::size_t pos = 2;
  while (pos + 4 <= jpeg.size()) {
    if (jpeg[pos] != 0xFF) {
      break;
    }
    const std::uint8_t marker = jpeg[pos + 1];
    if (marker == 0xDA || marker == 0xD9) {  // SOS / EOI: metadata is over
      break;
    }
    if (marker == 0xFF) {
      ++pos;
      continue;
    }
    const std::size_t seg_len =
        (static_cast<std::size_t>(jpeg[pos + 2]) << 8) | jpeg[pos + 3];
    if (seg_len < 2 || pos + 2 + seg_len > jpeg.size()) {
      break;
    }
    const std::uint8_t* seg      = jpeg.data() + pos + 4;
    const std::size_t   seg_body = seg_len - 2;
    if (marker == 0xE1 && seg_body >= 14 && std::memcmp(seg, "Exif\0\0", 6) == 0) {
      const std::uint8_t* tiff     = seg + 6;
      const std::size_t   tiff_len = seg_body - 6;
      bool                little   = false;
      if (tiff[0] == 'I' && tiff[1] == 'I') {
        little = true;
      } else if (tiff[0] == 'M' && tiff[1] == 'M') {
        little = false;
      } else {
        return 1;
      }
      const std::uint32_t ifd0 = Read32(tiff + 4, little);
      if (ifd0 + 2 <= tiff_len) {
        const std::uint16_t count = Read16(tiff + ifd0, little);
        for (std::uint16_t i = 0; i < count; ++i) {
          const std::size_t e = static_cast<std::size_t>(ifd0) + 2 + std::size_t{i} * 12;
          if (e + 12 > tiff_len) {
            break;
          }
          if (Read16(tiff + e, little) == 0x0112) {  // Orientation
            return static_cast<int>(Read16(tiff + e + 8, little));
          }
        }
      }
    }
    pos += 2 + seg_len;
  }
  return 1;
}

auto ResolveOrientation(const std::filesystem::path& path, int exif_orientation) -> int {
  if (exif_orientation < 1 || exif_orientation > 8) {
    return 1;
  }
  if (exif_orientation != 1) {
    return exif_orientation;  // Merrill+ dual-write: EXIF is authoritative
  }
  return x3f_alcedo_header_orientation(PathText(path).c_str());
}

auto IsX3fEncoded(std::span<const std::byte> encoded) -> bool {
  if (encoded.size() < 4) {
    return false;
  }
  const auto* head = reinterpret_cast<const char*>(encoded.data());
  return std::memcmp(head, "FOVb", 4) == 0 || std::memcmp(head, "FOVi", 4) == 0;
}

auto ColorProfile(const std::filesystem::path& path, int frame)
    -> std::optional<X3fColorProfile> {
  const auto key = ColorProfileKey(path, frame);
  {
    auto&                 cache = SharedColorProfileCache();
    const std::lock_guard lock(cache.mutex);
    if (const auto it = cache.entries.find(key); it != cache.entries.end()) {
      return it->second;
    }
  }

  // Decode-free: bulk import must never decompress the sensor planes. This
  // reads only the header, trailing directory and CAMF metadata (~3 MB).
  // x3f_alcedo_export_linear uses the very same matrix for its pixels, so the
  // document camera profile and the rendered scene-linear data stay paired.
  const auto          text = PathText(path);
  x3f_alcedo_linear_t decoded{};
  const int           rc = x3f_alcedo_probe_color(text.c_str(), frame, decoded.cam_xyz);
  if (rc != X3F_ALCEDO_OK) {
    return std::nullopt;
  }
  decoded.model_class = x3f_alcedo_model_class(text.c_str());
  auto profile        = ProfileFrom(decoded);
  if (!profile) {
    return std::nullopt;
  }

  auto&                 cache = SharedColorProfileCache();
  const std::lock_guard lock(cache.mutex);
  // Import walks a library sequentially; a bounded cache avoids unbounded growth.
  if (cache.entries.size() >= 512) {
    cache.entries.clear();
  }
  cache.entries.emplace(key, *profile);
  return profile;
}

auto DecodeEncoded(std::span<const std::byte> encoded, int decoder)
    -> std::optional<X3fLinearFrame> {
  if (!IsX3fEncoded(encoded)) {
    return std::nullopt;
  }
  x3f_alcedo_linear_t decoded{};
  const int rc = x3f_alcedo_export_linear_memory(encoded.data(), encoded.size(), decoder,
                                                 &decoded);
  if (rc != X3F_ALCEDO_OK || decoded.rgb == nullptr || decoded.width <= 0 ||
      decoded.height <= 0) {
    x3f_alcedo_export_free(&decoded);
    return std::nullopt;
  }
  auto profile = ProfileFrom(decoded);
  if (!profile) {
    x3f_alcedo_export_free(&decoded);
    return std::nullopt;
  }

  X3fLinearFrame frame;
  frame.width        = decoded.width;
  frame.height       = decoded.height;
  frame.orientation  = decoded.orientation;
  frame.decoder_used = decoded.decoder_used;
  frame.model_class  = decoded.model_class;
  frame.profile      = *profile;
  // Both backends allocate with libc malloc. Take ownership instead of copying
  // ~200 MB of full-resolution pixels, and never clamp on the way out.
  frame.rgb = std::shared_ptr<const float>(decoded.rgb, [](const float* p) {
    std::free(const_cast<float*>(p));
  });
  return frame;
}

}  // namespace alcedo::x3f
