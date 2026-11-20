//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

// Content classification decides which decoder receives the encoded bytes. A rendered RGB TIFF
// still carries camera EXIF (Make/Model); it must go to the raster loader, never LibRaw. This
// regresses the external-editor handoff, where an exported 16-bit TIFF was intermittently sent to
// RawInputLoader and failed with "LibRaw open_buffer failed".

#include "image/image_content_class.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

namespace alcedo {
namespace {

struct TiffEntry {
  uint16_t             tag;
  uint16_t             type;  // 2 = ASCII, 3 = SHORT, 4 = LONG
  uint32_t             count;
  std::vector<uint8_t> value;
};

auto Put16(std::vector<uint8_t>& out, uint16_t value, bool little) -> void {
  if (little) {
    out.push_back(static_cast<uint8_t>(value));
    out.push_back(static_cast<uint8_t>(value >> 8));
  } else {
    out.push_back(static_cast<uint8_t>(value >> 8));
    out.push_back(static_cast<uint8_t>(value));
  }
}

auto Put32(std::vector<uint8_t>& out, uint32_t value, bool little) -> void {
  for (int i = 0; i < 4; ++i) {
    const int shift = little ? i * 8 : (3 - i) * 8;
    out.push_back(static_cast<uint8_t>(value >> shift));
  }
}

auto ShortEntry(uint16_t tag, uint16_t value) -> TiffEntry {
  return {tag, 3, 1, {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8)}};
}

auto LongEntry(uint16_t tag, uint32_t value) -> TiffEntry {
  return {tag, 4, 1,
          {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
           static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 24)}};
}

auto AsciiEntry(uint16_t tag, const std::string& text) -> TiffEntry {
  std::vector<uint8_t> value(text.begin(), text.end());
  value.push_back(0);
  return {tag, 2, static_cast<uint32_t>(value.size()), std::move(value)};
}

/// Builds the smallest valid IFD0 carrying the tags the classifier inspects. No pixel data is
/// needed: classification is a container-layout decision and must not depend on LibRaw parsing a
/// synthetic payload.
auto MakeTiff(std::vector<TiffEntry> entries, bool little = true) -> std::vector<std::byte> {
  std::sort(entries.begin(), entries.end(),
            [](const TiffEntry& a, const TiffEntry& b) { return a.tag < b.tag; });
  const std::size_t ifd_offset  = 8;
  const std::size_t data_offset = ifd_offset + 2 + entries.size() * 12 + 4;

  std::vector<std::vector<uint8_t>> fields(entries.size());
  std::vector<uint8_t>              external;
  std::size_t                       cursor = data_offset;
  for (std::size_t i = 0; i < entries.size(); ++i) {
    const auto& entry   = entries[i];
    const auto  unit    = entry.type == 3 ? 2U : (entry.type == 4 ? 4U : 1U);
    const auto  bytes   = static_cast<std::size_t>(entry.count) * unit;
    if (bytes <= 4) {
      fields[i] = entry.value;
      fields[i].resize(4, 0);
      continue;
    }
    const auto offset = static_cast<uint32_t>(cursor);
    fields[i]         = {static_cast<uint8_t>(offset), static_cast<uint8_t>(offset >> 8),
                         static_cast<uint8_t>(offset >> 16), static_cast<uint8_t>(offset >> 24)};
    external.insert(external.end(), entry.value.begin(), entry.value.end());
    cursor += entry.value.size();
  }

  std::vector<uint8_t> out;
  out.reserve(cursor + 8);
  if (little) {
    out.push_back('I');
    out.push_back('I');
  } else {
    out.push_back('M');
    out.push_back('M');
  }
  Put16(out, 42, little);
  Put32(out, static_cast<uint32_t>(ifd_offset), little);
  Put16(out, static_cast<uint16_t>(entries.size()), little);
  for (std::size_t i = 0; i < entries.size(); ++i) {
    Put16(out, entries[i].tag, little);
    Put16(out, entries[i].type, little);
    Put32(out, entries[i].count, little);
    out.insert(out.end(), fields[i].begin(), fields[i].end());
  }
  Put32(out, 0, little);
  out.insert(out.end(), external.begin(), external.end());

  std::vector<std::byte> bytes;
  bytes.reserve(out.size());
  for (const auto value : out) {
    bytes.push_back(static_cast<std::byte>(value));
  }
  return bytes;
}

auto FixturePath(const std::string& name) -> std::filesystem::path {
  return std::filesystem::path(ALCEDO_RASTER_FIXTURE_DIR) / name;
}

auto ReadFixture(const std::string& name) -> std::vector<std::byte> {
  std::ifstream in(FixturePath(name), std::ios::binary);
  if (!in) {
    ADD_FAILURE() << "missing fixture " << FixturePath(name).string();
    return {};
  }
  const std::vector<char> chars((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
  std::vector<std::byte>  bytes(chars.size());
  for (std::size_t i = 0; i < chars.size(); ++i) {
    bytes[i] = static_cast<std::byte>(chars[i]);
  }
  return bytes;
}

TEST(ImageContentClassTest, RenderedRgbTiffWithCameraExifIsRaster) {
  // The exact shape that the external editor exports: 16-bit RGB TIFF whose EXIF still says
  // "Nikon". A LibRaw open_buffer probe can report a raw_count for this; classification must not
  // trust it and must route by the container's RGB photometric tag.
  const auto bytes = ReadFixture("nikon_rgb_with_camera_metadata.tif");
  ASSERT_FALSE(bytes.empty());
  EXPECT_EQ(ClassifyImageContent(bytes), ImageContentClass::Tiff);
  EXPECT_EQ(RasterFileKindFor(ClassifyImageContent(bytes)), RasterFileKind::Tiff);
}

TEST(ImageContentClassTest, PlainRgbTiffsAreRaster) {
  for (const char* name : {"alcedo_export_srgb_piecewise.tif", "uncompressed_rgb16_no_make.tif",
                           "rec2020_icc_16bit.tif", "prophoto_icc_16bit.tif"}) {
    const auto bytes = ReadFixture(name);
    ASSERT_FALSE(bytes.empty()) << name;
    EXPECT_EQ(ClassifyImageContent(bytes), ImageContentClass::Tiff) << name;
  }
}

TEST(ImageContentClassTest, TiffBasedRawPreviewWithUniqueCameraModelIsRaw) {
  // Hasselblad 3FR exposes a JPEG preview (photometric 6) in IFD0. UniqueCameraModel is a
  // RAW container tag and must override the raster-looking photometric.
  const auto bytes = MakeTiff({ShortEntry(256, 64), ShortEntry(257, 64), ShortEntry(259, 7),
                               ShortEntry(262, 6), AsciiEntry(271, "Hasselblad"),
                               AsciiEntry(50708, "Hasselblad X2D")});
  EXPECT_EQ(ClassifyImageContent(bytes), ImageContentClass::Raw);
}

TEST(ImageContentClassTest, ArwLikeFirstIfdWithoutDimensionsIsRaw) {
  // Sony ARW often starts with a metadata-only IFD: no width/height and no photometric, so the
  // default photometric 0 would otherwise look like a raster TIFF.
  const auto bytes = MakeTiff({LongEntry(254, 1), ShortEntry(259, 6), AsciiEntry(271, "SONY"),
                               LongEntry(34665, 0)});
  EXPECT_EQ(ClassifyImageContent(bytes), ImageContentClass::Raw);
}

TEST(ImageContentClassTest, NonTiffMagicIsUnaffected) {
  EXPECT_EQ(ClassifyImageContent(std::vector<std::byte>{
                std::byte{0xFF}, std::byte{0xD8}, std::byte{0xFF}}),
            ImageContentClass::Jpeg);
  const std::vector<std::byte> png{std::byte{0x89}, std::byte{0x50}, std::byte{0x4E},
                                   std::byte{0x47}, std::byte{0x0D}, std::byte{0x0A},
                                   std::byte{0x1A}, std::byte{0x0A}};
  EXPECT_EQ(ClassifyImageContent(png), ImageContentClass::Png);
}

}  // namespace
}  // namespace alcedo
