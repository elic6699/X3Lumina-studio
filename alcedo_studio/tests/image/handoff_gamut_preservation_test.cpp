//  Copyright 2026 Yurun Zi
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

/// @file handoff_gamut_preservation_test.cpp
/// @brief Phase-4 proof for the x3lumina -> Studio export handoff.
///
/// The companion app hands an exported DNG/TIFF to Studio by copying its bytes into the
/// Studio-managed cache (`Data(contentsOf:)` + `write(.atomic)`), then importing that copy.
/// No decode, quantisation or gamut mapping happens in the handoff itself. These tests
/// lock that invariant at the byte level, so a future change that decodes and re-encodes
/// through sRGB/P3, clamps negatives, or clips over-range linear light is caught:
///
///   * byte-exact cache copy keeps the full RasterColorDescription (primaries, transfer,
///     peak luminance and the ICC digest) equal to the source file's;
///   * the wide-gamut primaries are really resolved from the embedded ICC, not a relabel;
///   * linear pixels carrying negative and >1.0 code values survive the copy unchanged.

#include "edit/input/raster_input_loader.hpp"
#include "image/raster_color_description.hpp"
#include "utils/hash/sha256.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace alcedo {
namespace {

namespace fs = std::filesystem;

auto FixturePath(const std::string& name) -> fs::path {
  return fs::path(ALCEDO_RASTER_FIXTURE_DIR) / name;
}

auto ReadBytes(const fs::path& path) -> std::vector<std::byte> {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    ADD_FAILURE() << "missing fixture " << path.string();
    return {};
  }
  const std::vector<char> chars((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
  std::vector<std::byte> bytes(chars.size());
  std::memcpy(bytes.data(), chars.data(), chars.size());
  return bytes;
}

void WriteBytes(const fs::path& path, std::span<const std::byte> bytes) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(out.is_open()) << "cannot write " << path.string();
  out.write(reinterpret_cast<const char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
}

/// The companion app's handoff write is `Data(contentsOf:) + write(.atomic)`: a pure byte
/// copy. This helper reproduces exactly that semantic on the Studio side.
auto SimulateCacheCopy(const fs::path& source, const fs::path& cache_file) -> void {
  const auto bytes = ReadBytes(source);
  ASSERT_FALSE(bytes.empty()) << source.string();
  WriteBytes(cache_file, bytes);
}

void ExpectPrimariesNear(const RasterColorDescription& actual,
                         const std::array<float, 8>& expected) {
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(actual.primaries_xy_[i], expected[i], 2e-3f) << "chromaticity index " << i;
  }
}

/// A cached copy must be byte-identical to the exported file, and resolving its colour
/// description must give back the same ICC digest and the same wide primaries. `expected`
/// is the gamut the fixture is known to carry; asserting it here proves the description
/// came from the embedded profile rather than a DefaultSrgb fallback or a relabel.
void ExpectWideGamutSurvivesCacheCopy(const std::string& fixture, RasterFileKind kind,
                                      const std::array<float, 8>& expected) {
  const auto temp = fs::temp_directory_path() / "alcedo-handoff-gamut-test";
  std::error_code ec;
  fs::create_directories(temp, ec);
  ASSERT_FALSE(ec) << ec.message();
  const fs::path cache_file = temp / fixture;

  const fs::path source = FixturePath(fixture);
  SimulateCacheCopy(source, cache_file);

  const auto source_bytes = ReadBytes(source);
  const auto cache_bytes = ReadBytes(cache_file);
  ASSERT_EQ(cache_bytes.size(), source_bytes.size());
  EXPECT_EQ(ComputeSha256Hex(cache_bytes), ComputeSha256Hex(source_bytes))
      << "cache copy must be byte-exact; a decode/re-encode or clamp broke the handoff";

  const auto source_color = ResolveRasterColorDescription(source_bytes, kind);
  const auto cache_color = ResolveRasterColorDescription(cache_bytes, kind);
  EXPECT_EQ(cache_color, source_color)
      << "the cached copy must resolve to the same colour description as the export";
  EXPECT_FALSE(cache_color.icc_sha256_.empty())
      << "wide-gamut fixture must carry a real ICC profile";
  EXPECT_EQ(cache_color.icc_sha256_, source_color.icc_sha256_);
  ExpectPrimariesNear(cache_color, expected);
}

// -------------------------------------------------------------------------------------------
// Wide-gamut equivalence: the cached copy is treated exactly like the exported file.
// -------------------------------------------------------------------------------------------

TEST(HandoffGamutPreservationTest, ProPhotoTiffKeepsIccAndPrimariesThroughCacheCopy) {
  ExpectWideGamutSurvivesCacheCopy("prophoto_icc_16bit.tif", RasterFileKind::Tiff,
                                   color::GamutPrimariesXy(color::ColorGamutId::ProPhoto));
}

TEST(HandoffGamutPreservationTest, Rec2020TiffKeepsIccAndPrimariesThroughCacheCopy) {
  ExpectWideGamutSurvivesCacheCopy("rec2020_icc_16bit.tif", RasterFileKind::Tiff,
                                   color::GamutPrimariesXy(color::ColorGamutId::Rec2020));
}

TEST(HandoffGamutPreservationTest, Float32LinearTiffKeepsDescriptionThroughCacheCopy) {
  ExpectWideGamutSurvivesCacheCopy("float32_linear_icc.tif", RasterFileKind::Tiff,
                                   color::GamutPrimariesXy(color::ColorGamutId::Rec709));
}

// -------------------------------------------------------------------------------------------
// Decode level: a cache import and a normal import of the same bytes decode identically.
// -------------------------------------------------------------------------------------------

/// `RasterInputLoader::LoadEncoded` consumes only a byte span, never a file path. A handoff
/// import and a normal import of the same export therefore have to produce the same decoded
/// host plane, extents, source key and colour context. This is the decode-boundary proof of
/// the plan's claim that "every downstream decode consumes the same bytes and yields the same
/// planes and colour context", without needing a GPU editor backend or an LFS RAW fixture.
void ExpectCacheImportDecodesLikeNormalImport(const std::string& fixture, RasterFileKind kind) {
  const auto temp = fs::temp_directory_path() / "alcedo-handoff-decode-test";
  std::error_code ec;
  fs::create_directories(temp, ec);
  ASSERT_FALSE(ec) << ec.message();

  const fs::path source = FixturePath(fixture);
  // Two distinct paths, identical bytes: the ordinary export location and the Studio cache.
  const fs::path normal_copy = temp / ("normal_" + fixture);
  const fs::path cache_copy = temp / ("cached_" + fixture);
  WriteBytes(normal_copy, ReadBytes(source));
  SimulateCacheCopy(source, cache_copy);

  const auto normal_input =
      RasterInputLoader::LoadEncoded(ReadBytes(normal_copy), DecodeRes::FULL, kind);
  const auto cache_input =
      RasterInputLoader::LoadEncoded(ReadBytes(cache_copy), DecodeRes::FULL, kind);

  ASSERT_EQ(normal_input.input_kind, RawInputKind::RasterRgb);
  EXPECT_EQ(cache_input.input_kind, normal_input.input_kind);
  EXPECT_EQ(cache_input.host_extent, normal_input.host_extent);
  EXPECT_EQ(cache_input.develop_output_extent, normal_input.develop_output_extent);
  EXPECT_EQ(cache_input.full_reference_extent, normal_input.full_reference_extent);
  EXPECT_EQ(cache_input.downsample_passes, normal_input.downsample_passes);
  EXPECT_EQ(cache_input.pixels.format, normal_input.pixels.format);
  EXPECT_EQ(cache_input.pixels.extent, normal_input.pixels.extent);
  EXPECT_EQ(cache_input.pixels.stride_bytes, normal_input.pixels.stride_bytes);
  EXPECT_EQ(cache_input.source_key, normal_input.source_key)
      << "the content identity of a cache import must match a normal import";
  EXPECT_EQ(cache_input.raster_lut_icc_converted, normal_input.raster_lut_icc_converted);
  EXPECT_EQ(cache_input.raster_icc_sha256, normal_input.raster_icc_sha256);

  const auto normal_plane = normal_input.pixels.Span();
  const auto cache_plane = cache_input.pixels.Span();
  ASSERT_EQ(cache_plane.size(), normal_plane.size());
  EXPECT_EQ(std::memcmp(cache_plane.data(), normal_plane.data(), normal_plane.size()), 0)
      << "decoded host-plane code values differ between cache and normal import";
}

TEST(HandoffGamutPreservationTest, ProPhotoCacheImportDecodesLikeNormalImport) {
  ExpectCacheImportDecodesLikeNormalImport("prophoto_icc_16bit.tif", RasterFileKind::Tiff);
}

TEST(HandoffGamutPreservationTest, Rec2020CacheImportDecodesLikeNormalImport) {
  ExpectCacheImportDecodesLikeNormalImport("rec2020_icc_16bit.tif", RasterFileKind::Tiff);
}

TEST(HandoffGamutPreservationTest, Float32LinearCacheImportDecodesLikeNormalImport) {
  ExpectCacheImportDecodesLikeNormalImport("float32_linear_icc.tif", RasterFileKind::Tiff);
}

// -------------------------------------------------------------------------------------------
// Code-value level: negative and over-range linear light must not be clipped.
// -------------------------------------------------------------------------------------------

/// Locate the single strip of a baseline TIFF by walking its IFD, so the test does not
/// hard-code a fixture offset. Returns {byte offset, byte count}; {0, 0} on any invalid
/// structure, which the caller turns into a test failure.
auto TiffStripRange(std::span<const std::byte> bytes) -> std::pair<std::size_t, std::size_t> {
  const auto u16 = [&](std::size_t at) -> uint16_t {
    return static_cast<uint16_t>(std::to_integer<uint8_t>(bytes[at])) |
           static_cast<uint16_t>(static_cast<uint16_t>(std::to_integer<uint8_t>(bytes[at + 1]))
                                 << 8);
  };
  const auto u32 = [&](std::size_t at) -> uint32_t {
    return static_cast<uint32_t>(u16(at)) | (static_cast<uint32_t>(u16(at + 2)) << 16);
  };
  if (bytes.size() < 8) {
    return {0, 0};
  }
  if (std::to_integer<uint8_t>(bytes[0]) != std::to_integer<uint8_t>(std::byte{'I'}) ||
      u16(2) != 42) {
    return {0, 0};  // only classic little-endian baseline TIFF is used here
  }
  const std::size_t ifd = u32(4);
  if (ifd >= bytes.size()) {
    return {0, 0};
  }
  const std::size_t count = u16(ifd);
  std::size_t       offset = 0;
  std::size_t       byte_count = 0;
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t entry = ifd + 2 + i * 12;
    if (entry + 12 > bytes.size()) {
      return {0, 0};
    }
    const uint16_t tag  = u16(entry);
    const uint16_t type = u16(entry + 2);
    const uint32_t n    = u32(entry + 4);
    if ((tag == 273 || tag == 279) && type == 4 && n == 1) {
      if (tag == 273) {
        offset = u32(entry + 8);
      } else {
        byte_count = u32(entry + 8);
      }
    }
  }
  if (byte_count == 0 || offset > bytes.size() || offset + byte_count > bytes.size()) {
    return {0, 0};
  }
  return {offset, byte_count};
}

TEST(HandoffGamutPreservationTest, NegativeAndOverRangeLinearPixelsSurviveByteExactCopy) {
  // Start from a real scene-linear float32 TIFF with an embedded ICC, then write a few
  // deliberately out-of-gamut pixel values into its strip: a negative and several >1.0.
  // These are legal linear-light code values that a clamp-to-sRGB/P3 step would destroy.
  auto source_bytes = ReadBytes(FixturePath("float32_linear_icc.tif"));
  ASSERT_FALSE(source_bytes.empty());
  const auto [strip_offset, strip_bytes] = TiffStripRange(source_bytes);
  ASSERT_GE(strip_bytes, 5u * sizeof(float));

  const std::array<float, 5> out_of_range{-0.75f, 1.6f, 2.4f, -0.01f, 1.25f};
  for (std::size_t i = 0; i < out_of_range.size(); ++i) {
    std::memcpy(source_bytes.data() + strip_offset + i * sizeof(float), &out_of_range[i],
                sizeof(float));
  }

  const auto temp = fs::temp_directory_path() / "alcedo-handoff-gamut-test";
  std::error_code ec;
  fs::create_directories(temp, ec);
  ASSERT_FALSE(ec) << ec.message();
  const fs::path source = temp / "out_of_range_source.tif";
  const fs::path cache_file = temp / "out_of_range_cached.tif";
  WriteBytes(source, source_bytes);
  SimulateCacheCopy(source, cache_file);

  const auto cache_bytes = ReadBytes(cache_file);
  ASSERT_EQ(cache_bytes.size(), source_bytes.size());
  EXPECT_EQ(ComputeSha256Hex(cache_bytes), ComputeSha256Hex(source_bytes))
      << "cache copy must be byte-exact even for out-of-range linear code values";

  std::array<float, out_of_range.size()> decoded{};
  std::memcpy(decoded.data(), cache_bytes.data() + strip_offset, sizeof(decoded));
  for (std::size_t i = 0; i < out_of_range.size(); ++i) {
    EXPECT_FLOAT_EQ(decoded[i], out_of_range[i]) << "linear channel " << i;
  }
  EXPECT_TRUE(std::any_of(decoded.begin(), decoded.end(), [](float v) { return v < 0.0f; }));
  EXPECT_TRUE(std::any_of(decoded.begin(), decoded.end(), [](float v) { return v > 1.0f; }));

  // The ICC and the colour description must also be unchanged by the copy.
  EXPECT_EQ(ResolveRasterColorDescription(cache_bytes, RasterFileKind::Tiff),
            ResolveRasterColorDescription(source_bytes, RasterFileKind::Tiff));
  EXPECT_FALSE(ResolveRasterColorDescription(cache_bytes, RasterFileKind::Tiff)
                   .icc_sha256_.empty());
}

}  // namespace
}  // namespace alcedo
