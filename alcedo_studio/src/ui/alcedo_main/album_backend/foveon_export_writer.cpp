//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "ui/alcedo_main/album_backend/foveon_export_writer.hpp"

#include <OpenImageIO/imageio.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "edit/operators/utils/color_utils.hpp"
#include "io/image/export_color_profile_config.hpp"
#include "io/image/export_icc_profile_resolver.hpp"

namespace alcedo::ui {
namespace {

using alcedo::x3f::FoveonImage;

/// Apply an EXIF orientation 1..8 to sensor-order RGB16. The formulas mirror
/// the preview's QTransform sequence in foveon_session_controller.cpp; doing it
/// on the pixel buffer (not a TIFF orientation tag) also makes PNG correct.
auto OrientRgb16(const FoveonImage& source, std::vector<std::uint16_t>* out,
                 int* out_width, int* out_height) -> bool {
  if (out == nullptr || out_width == nullptr || out_height == nullptr ||
      source.Empty()) {
    return false;
  }
  const int src_w = source.width;
  const int src_h = source.height;
  int orientation = source.orientation;
  if (orientation < 1 || orientation > 8) {
    orientation = 1;
  }
  const bool swapped = orientation >= 5 && orientation <= 8;
  const int dst_w = swapped ? src_h : src_w;
  const int dst_h = swapped ? src_w : src_h;
  const auto count = static_cast<std::size_t>(dst_w) *
                     static_cast<std::size_t>(dst_h) * 3U;
  try {
    out->resize(count);
  } catch (const std::bad_alloc&) {
    return false;
  }
  const auto* src = source.rgb.get();
  auto* dst = out->data();
  for (int y = 0; y < dst_h; ++y) {
    for (int x = 0; x < dst_w; ++x) {
      int sx = x;
      int sy = y;
      switch (orientation) {
        case 2:
          sx = src_w - 1 - x;
          break;
        case 3:
          sx = src_w - 1 - x;
          sy = src_h - 1 - y;
          break;
        case 4:
          sy = src_h - 1 - y;
          break;
        case 5:
          sx = y;
          sy = x;
          break;
        case 6:
          sx = y;
          sy = src_h - 1 - x;
          break;
        case 7:
          sx = src_w - 1 - y;
          sy = src_h - 1 - x;
          break;
        case 8:
          sx = src_w - 1 - y;
          sy = x;
          break;
        default:
          break;
      }
      const auto src_index =
          (static_cast<std::size_t>(sy) * src_w + sx) * 3U;
      const auto dst_index =
          (static_cast<std::size_t>(y) * dst_w + x) * 3U;
      dst[dst_index + 0] = src[src_index + 0];
      dst[dst_index + 1] = src[src_index + 1];
      dst[dst_index + 2] = src[src_index + 2];
    }
  }
  *out_width = dst_w;
  *out_height = dst_h;
  return true;
}

auto LowerExtension(const std::filesystem::path& path) -> std::string {
  auto ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return ext;
}

}  // namespace

auto WriteFoveon16Bit(const FoveonImage& image,
                      const std::filesystem::path& destination,
                      std::string* error) -> bool {
  if (error != nullptr) {
    error->clear();
  }
  const auto fail = [error](std::string message) {
    if (error != nullptr) {
      *error = std::move(message);
    }
    return false;
  };

  if (image.Empty()) {
    return fail("Foveon export: empty image");
  }
  if (destination.empty()) {
    return fail("Foveon export: empty destination");
  }
  const auto ext = LowerExtension(destination);
  if (ext != ".tif" && ext != ".tiff" && ext != ".png") {
    return fail("Foveon export: destination must be .tif/.tiff/.png");
  }

  std::vector<std::uint16_t> pixels;
  int width = 0;
  int height = 0;
  if (!OrientRgb16(image, &pixels, &width, &height)) {
    return fail("Foveon export: cannot orient image");
  }

  const ExportColorProfileConfig profile{
      ColorUtils::ColorSpace::REC709, ColorUtils::EOTF::SRGB_PIECEWISE, 100.0F};
  const auto icc = ExportIccProfileResolver::ResolveIccProfileBytes(profile);

  // Keep the real extension at the end: ImageOutput::create() chooses the
  // writer from the suffix, and a trailing ".part-<id>" would hide it.
  auto temp = destination;
  temp.replace_filename(
      destination.stem().string() + ".part-" +
      std::to_string(std::chrono::steady_clock::now()
                         .time_since_epoch()
                         .count()) +
      destination.extension().string());

  {
    std::unique_ptr<OIIO::ImageOutput> out =
        OIIO::ImageOutput::create(temp.string());
    if (!out) {
      const std::string detail = OIIO::geterror();
      return fail("Foveon export: OpenImageIO has no writer for this format" +
                  (detail.empty() ? std::string{} : ": " + detail));
    }

    OIIO::ImageSpec spec(width, height, 3, OIIO::TypeDesc::UINT16);
    // The samples are already display-encoded sRGB. Do not add any transfer
    // function or float conversion on the way out.
    spec.attribute("oiio:ColorSpace", "sRGB");
    if (!icc.empty()) {
      spec.attribute(
          "ICCProfile",
          OIIO::TypeDesc(OIIO::TypeDesc::UINT8, OIIO::TypeDesc::SCALAR,
                         OIIO::TypeDesc::NOSEMANTICS,
                         static_cast<int>(icc.size())),
          icc.data());
    }
    if (ext == ".tif" || ext == ".tiff") {
      spec.attribute("tiff:compression", "zip");
    }

    if (!out->open(temp.string(), spec)) {
      std::error_code ignored;
      std::filesystem::remove(temp, ignored);
      return fail("Foveon export: cannot open output: " + out->geterror());
    }
    const OIIO::stride_t xstride =
        static_cast<OIIO::stride_t>(3U * sizeof(std::uint16_t));
    const OIIO::stride_t ystride =
        static_cast<OIIO::stride_t>(static_cast<std::size_t>(width) * 3U *
                                    sizeof(std::uint16_t));
    if (!out->write_image(OIIO::TypeDesc::UINT16, pixels.data(), xstride,
                          ystride, OIIO::AutoStride)) {
      const std::string message = out->geterror();
      out->close();
      std::error_code ignored;
      std::filesystem::remove(temp, ignored);
      return fail("Foveon export: write failed: " + message);
    }
    if (!out->close()) {
      const std::string message = out->geterror();
      std::error_code ignored;
      std::filesystem::remove(temp, ignored);
      return fail("Foveon export: close failed: " + message);
    }
  }

  std::error_code ec;
  std::filesystem::rename(temp, destination, ec);
  if (ec) {
    // Windows does not replace an existing destination with rename(). The
    // temporary file is complete at this point, so replacing the user-chosen
    // target is safe (the file dialog already asked for overwrite consent).
    std::error_code remove_ec;
    std::filesystem::remove(destination, remove_ec);
    ec.clear();
    std::filesystem::rename(temp, destination, ec);
  }
  if (ec) {
    std::error_code ignored;
    std::filesystem::remove(temp, ignored);
    return fail("Foveon export: cannot commit output: " + ec.message());
  }
  return true;
}

}  // namespace alcedo::ui
