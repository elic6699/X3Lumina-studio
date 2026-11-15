//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <cstddef>
#include <span>

#include "edit/input/prepared_raw_input.hpp"
#include "type/type.hpp"

namespace alcedo::x3f {

/// Builds a Develop-ready input from an opaque X3F/X3I byte buffer.
///
/// The decoder emits camera-native scene-linear f32 RGB (no clamp, negative
/// channels and >1 headroom preserved). The buffer is repacked into a tightly
/// packed F32Rgba plane and routed through RawInputLoader::FromDirectRgb, which
/// maps to DevelopInputKind::DirectRgb and bypasses demosaic / is_foveon checks.
/// The colour matrix must also be bound at import time (Image's raw colour
/// context); Develop reads the camera profile from the document, not from here.
[[nodiscard]] auto LoadX3fEncoded(std::span<const std::byte> encoded, DecodeRes decode_res)
    -> PreparedRawInput;

}  // namespace alcedo::x3f
