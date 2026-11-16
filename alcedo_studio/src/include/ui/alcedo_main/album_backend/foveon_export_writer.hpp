//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <filesystem>
#include <string>

#include "decoders/x3f/x3f_foveon.hpp"

namespace alcedo::ui {

/// Write the path-B / SPP finished image as a 16-bit sRGB TIFF or PNG.
///
/// The input is already display-encoded 16-bit sRGB in sensor order. This
/// writer applies the EXIF orientation once, writes UINT16 samples verbatim
/// (no gamma, no clamp, no float conversion) and embeds the bundled sRGB ICC
/// profile when it is available. It is deliberately independent of the
/// wide-gamut path-A export writer: path-A remains scene-linear float.
///
/// The destination must end in .tif/.tiff/.png. Writing is atomic: a temporary
/// file in the destination directory is renamed only after a complete write.
[[nodiscard]] auto WriteFoveon16Bit(
    const alcedo::x3f::FoveonImage& image,
    const std::filesystem::path& destination, std::string* error) -> bool;

}  // namespace alcedo::ui
