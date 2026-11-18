//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
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

// Import-only support. X3F/X3I files are identified, probed and their embedded
// preview JPEG read here — no sensor-plane decoding happens in the main app.
// Editing is owned by the external companion app through the thin bridge, so
// the colour-matrix probe and in-memory decode bridge are intentionally absent
// from this header (they live with the decode sources, which are not built into
// the application).

}  // namespace alcedo::x3f
