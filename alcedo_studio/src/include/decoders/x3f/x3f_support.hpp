//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace alcedo::x3f {

/// True for a Sigma Foveon file (.x3f or the .x3i SFD container), ASCII case-insensitive.
[[nodiscard]] auto IsX3fFile(const std::filesystem::path& path) -> bool;

struct ProbeInfo {
  bool        is_container = false;  ///< FOVi (SFD) container with 7 frames
  int         frame_count  = 1;
  int         width        = 0;  ///< full-size embedded JPEG
  int         height       = 0;
  bool        has_jpeg     = false;
  bool        has_exif     = false;
  std::string serial;
};

/// Reads only the header and the trailing directory. Never maps the whole file,
/// which matters for .X3I SFD containers that can be gigabytes large.
/// @p frame < 0 selects the default frame (0 EV for a container).
[[nodiscard]] auto Probe(const std::filesystem::path& path, int frame = -1)
    -> std::optional<ProbeInfo>;

/// 1 for a plain X3F, 7 for an X3I container, 0 when the file cannot be probed.
[[nodiscard]] auto FrameCount(const std::filesystem::path& path) -> int;

/// Raw SOI..EOI bytes of the camera's full-size embedded JPEG. No re-encode,
/// no colour transform and no ICC rewrite. Empty when unavailable.
[[nodiscard]] auto ReadEmbeddedJpeg(const std::filesystem::path& path, int frame = -1)
    -> std::vector<uint8_t>;

/// EXIF Orientation (1..8) parsed from an embedded JPEG's APP1/Exif IFD0.
/// Returns 1 when the tag is absent. No image decoding, no external deps.
[[nodiscard]] auto EmbeddedJpegOrientation(std::span<const std::uint8_t> jpeg) -> int;

/// Foveon bodies disagree on where the rotation lives. Older bodies (X3F v2.x,
/// e.g. SD14) write it only in the header at offset 36 while EXIF stays 1;
/// Merrill+ dual-write it (Rotation=270 <-> EXIF 8); Quattro v4+ use EXIF only.
/// Mirrors X3Lumina's Thumbnailer.orientation(fromX3F:): fall back to the
/// header only when EXIF says 1. FOVi containers and v4+ always keep EXIF.
[[nodiscard]] auto ResolveOrientation(const std::filesystem::path& path,
                                      int exif_orientation) -> int;

/// Materializes one frame of an X3I container as a standalone, valid X3F byte
/// stream (the trailing directory self-pointer is patched). Returns empty for a
/// plain X3F: callers should use the original path in that case.
[[nodiscard]] auto MaterializeFrame(const std::filesystem::path& path, int frame = -1)
    -> std::vector<uint8_t>;

/// Sniffs the X3F/X3I magic on an opaque encoded buffer. The decode cache hands
/// LoadEncodedImage bytes, not a path, so the format branch must be content-based.
[[nodiscard]] auto IsX3fEncoded(std::span<const std::byte> encoded) -> bool;

/// Camera-native colour description. Both matrices are row-major 3x3:
///   cam_to_xyz : camera BMT linear -> XYZ (D65)
///   xyz_to_cam : its exact inverse (DNG ColorMatrix convention, XYZ -> camera)
/// The decoder emits exactly these together with its scene-linear pixels, so
/// import and render always agree on the same source primaries.
struct X3fColorProfile {
  double cam_to_xyz[9]{};
  double xyz_to_cam[9]{};
  int    model_class  = 0;
  int    decoder_used = 0;
};

/// Resolves the colour matrix of a source file without keeping the pixels.
/// Results are cached per (path, frame, size, mtime). A miss decodes one frame
/// and frees the pixels immediately; see the implementation note on import cost.
[[nodiscard]] auto ColorProfile(const std::filesystem::path& path, int frame = -1)
    -> std::optional<X3fColorProfile>;

/// One decoded camera-native scene-linear frame. rgb is width*height*3 floats,
/// RGB interleaved, with negative channels and >1 headroom preserved (no clamp).
struct X3fLinearFrame {
  std::shared_ptr<const float> rgb;
  int                          width        = 0;
  int                          height       = 0;
  int                          orientation  = 0;
  int                          decoder_used = 0;
  int                          model_class  = 0;
  X3fColorProfile              profile{};
};

/// Decodes an in-memory standalone X3F frame (FOVb). `decoder` is one of the
/// X3F_ALCEDO_DECODER_* values; 0 (AUTO) selects own for Quattro, fuse otherwise.
/// SPP is rejected: it is a finished-image sidecar, not a scene-linear source.
[[nodiscard]] auto DecodeEncoded(std::span<const std::byte> encoded, int decoder = 0)
    -> std::optional<X3fLinearFrame>;

}  // namespace alcedo::x3f
