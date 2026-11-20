//  Copyright 2026 Yurun Zi
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "image/image_content_class.hpp"

#include <libraw/libraw.h>

#include <initializer_list>
#include <memory>

#include "image/raster_container_reader.hpp"

namespace alcedo {
namespace {

auto StartsWith(std::span<const std::byte> bytes, std::initializer_list<unsigned char> magic)
    -> bool {
  if (bytes.size() < magic.size()) {
    return false;
  }
  std::size_t i = 0;
  for (const auto value : magic) {
    if (std::to_integer<unsigned char>(bytes[i++]) != value) {
      return false;
    }
  }
  return true;
}

/// LibRaw identifies a supported RAW image from the container's image structure and encoding.
auto LibRawRecognizesRawImage(std::span<const std::byte> bytes) -> bool {
  // Heap allocation: LibRaw is large (see AGENTS.md on WebGPU RAW tests).
  auto raw = std::make_unique<LibRaw>();
  // Camera color calibration is independent of whether the file contains RAW samples.
  return raw->open_buffer(bytes.data(), bytes.size()) == LIBRAW_SUCCESS &&
         raw->imgdata.idata.raw_count > 0;
}

}  // namespace

auto ClassifyImageContent(std::span<const std::byte> bytes) -> ImageContentClass {
  if (StartsWith(bytes, {0xFF, 0xD8, 0xFF})) {
    return ImageContentClass::Jpeg;
  }
  if (StartsWith(bytes, {0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A})) {
    return ImageContentClass::Png;
  }
  if (StartsWith(bytes, {0x76, 0x2F, 0x31, 0x01})) {
    return ImageContentClass::OpenExr;
  }
  if (StartsWith(bytes, {'I', 'I', 0x2A, 0x00}) || StartsWith(bytes, {'M', 'M', 0x00, 0x2A})) {
    TiffContainerInfo info;
    try {
      info = ReadTiffContainer(bytes);
    } catch (const RasterContainerError&) {
      // A TIFF-magic file whose first directory cannot be read is left to LibRaw.
      return ImageContentClass::Unknown;
    }
    if (info.has_dng_version_) {
      return ImageContentClass::Raw;
    }
    // Photometric (tag 262) is the first-order decoder hint. 32803 = CFA and 34892 =
    // LinearRaw are photosite data; the raster interpretations (0 gray, 2 RGB, 3 palette,
    // 5 separated/CMYK, 6 YCbCr) are already encoded samples. Unlike a bare LibRaw
    // open_buffer probe, this does not send an exported RGB TIFF with camera EXIF to the
    // RAW loader ("LibRaw open_buffer failed").
    const bool photosite_data = info.photometric_ == 32803 || info.photometric_ == 34892;
    const bool raster_pixels  = info.photometric_ == 0 || info.photometric_ == 2 ||
                                info.photometric_ == 3 || info.photometric_ == 5 ||
                                info.photometric_ == 6;
    if (photosite_data) {
      return ImageContentClass::Raw;
    }
    if (raster_pixels) {
      // TIFF-based RAW files may put a JPEG/RGB preview in IFD0. UniqueCameraModel (50708)
      // is a RAW-only container tag; a first IFD without dimensions but with a camera Make
      // is the other observed shape (Sony ARW). An exported raster TIFF has neither: it has
      // a complete width/height layout and no RAW container tag, even when EXIF still says
      // Nikon/Sigma. This is a byte-level layout decision; no colour values are converted.
      const bool raw_container_markers =
          info.has_unique_camera_model_ ||
          (info.width_ == 0 && info.height_ == 0 && !info.make_.empty());
      if (!raw_container_markers) {
        return ImageContentClass::Tiff;
      }
      return ImageContentClass::Raw;
    }
    // Ambiguous layout (e.g. 1 = BlackIsZero): keep the previous LibRaw fallback so less common
    // RAW containers are not regressed.
    return LibRawRecognizesRawImage(bytes) ? ImageContentClass::Raw : ImageContentClass::Tiff;
  }
  return ImageContentClass::Unknown;
}

auto RasterFileKindFor(ImageContentClass content_class) -> std::optional<RasterFileKind> {
  switch (content_class) {
    case ImageContentClass::Jpeg:
      return RasterFileKind::Jpeg;
    case ImageContentClass::Png:
      return RasterFileKind::Png;
    case ImageContentClass::Tiff:
      return RasterFileKind::Tiff;
    case ImageContentClass::OpenExr:
      return RasterFileKind::OpenExr;
    case ImageContentClass::Raw:
    case ImageContentClass::Unknown:
      break;
  }
  return std::nullopt;
}

}  // namespace alcedo
