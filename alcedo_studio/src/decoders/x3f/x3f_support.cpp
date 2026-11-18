//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "decoders/x3f/x3f_support.hpp"

#include <x3fcore.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>

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

/// Reads the rotation word at header offset 36. Mirrors the decode layer's
/// x3f_alcedo_header_orientation so import stays independent of the render core.
auto HeaderOrientation(const std::filesystem::path& path) -> int {
  FILE* f = std::fopen(PathText(path).c_str(), "rb");
  if (f == nullptr) {
    return 1;
  }
  std::uint8_t b[40] = {0};
  const auto   n = std::fread(b, 1, sizeof(b), f);
  std::fclose(f);
  if (n < 40 || std::memcmp(b, "FOVb", 4) != 0) {
    return 1;
  }
  if (Read32(b + 4, true) >= 0x00040000u) {
    return 1;  // v4+ Quattro headers do not carry the rotation word
  }
  switch (Read32(b + 36, true)) {
    case 90:  return 6;
    case 180: return 3;
    case 270: return 8;
    default:  return 1;
  }
}

auto ResolveOrientation(const std::filesystem::path& path, int exif_orientation) -> int {
  if (exif_orientation < 1 || exif_orientation > 8) {
    return 1;
  }
  if (exif_orientation != 1) {
    return exif_orientation;  // Merrill+ dual-write: EXIF is authoritative
  }
  return HeaderOrientation(path);
}

}  // namespace alcedo::x3f
