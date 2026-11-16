//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <filesystem>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include <json.hpp>

#include "decoders/x3f/x3f_foveon.hpp"

namespace alcedo::x3f {

// Path B adjustment persistence. X3Lumina stores its RawEditParams in a
// per-file sidecar "<source>.x3fedit.json" under the `raw` object. We read and
// write only the keys owned by the Foveon native chain and leave every other
// key (Fotufilm / SpektraLab / film / HDR / HSL / denoise ...) untouched, so a
// sidecar produced by X3Lumina stays valid for the main application.
//
// This state is deliberately NOT part of Alcedo's DevelopPayload, parameter
// catalog, history or content key: path B is a display-referred CPU sidecar and
// changing one of its sliders must not dirty the wide-gamut path A document or
// invalidate the GPU DAG cache.

enum class FoveonControlKind { Scalar, Toggle, Combo };

/// One control in the Foveon adjustment panel. The table is the single source
/// of truth for ranges, labels, model gating, JSON binding and QML controls.
struct FoveonParameterSpec {
  std::string                                        key;
  std::string                                        label;
  std::string                                        group;
  FoveonControlKind                                  kind = FoveonControlKind::Scalar;
  double                                             minimum   = 0.0;
  double                                             maximum   = 1.0;
  double                                             step      = 0.01;
  int                                                decimals  = 2;
  std::vector<std::string>                           options;
  /// "" = always available; "quattro" / "nonquattro" / "sdq" / "hl_chroma"
  /// gate the control exactly like X3Lumina's Views.swift.
  std::string                                        gate;
  std::function<void(FoveonEditParams&, double)>     set;
  std::function<double(const FoveonEditParams&)>     get;
};

/// Full ordered control table (highlight group in X3Lumina's panel order).
[[nodiscard]] auto FoveonParameterSpecs()
    -> const std::vector<FoveonParameterSpec>&;

/// Specs plus current values, ready for QML (`QJsonArray`-shaped object).
[[nodiscard]] auto FoveonParametersJson(const FoveonEditParams& params)
    -> nlohmann::json;

/// Value-only snapshot, used for the panel's live state and equality checks.
[[nodiscard]] auto FoveonValuesJson(const FoveonEditParams& params)
    -> nlohmann::json;

/// Numeric view of any control (toggles and combo indices included).
[[nodiscard]] auto FoveonNumericParam(const FoveonEditParams& params,
                                      std::string_view key) -> double;

[[nodiscard]] auto FoveonSetParam(FoveonEditParams& params, std::string_view key,
                                  double value) -> bool;
[[nodiscard]] auto FoveonSetStringParam(FoveonEditParams& params,
                                        std::string_view key,
                                        std::string_view value) -> bool;
[[nodiscard]] auto FoveonStringParam(const FoveonEditParams& params,
                                     std::string_view key) -> std::string;

/// Sidecar conversion. ToSidecarJson emits only the keys we own, in X3Lumina's
/// camelCase naming. FromSidecarJson starts from @p base (usually the product
/// defaults) and applies every recognized key; unknown keys are ignored.
[[nodiscard]] auto FoveonToSidecarJson(const FoveonEditParams& params)
    -> nlohmann::json;
[[nodiscard]] auto FoveonFromSidecarJson(const nlohmann::json& document,
                                         FoveonEditParams base)
    -> FoveonEditParams;

[[nodiscard]] auto FoveonSidecarPath(const std::filesystem::path& source)
    -> std::filesystem::path;

/// Loads the sidecar if present. Returns false when absent or unparsable; the
/// caller then keeps the product defaults. Never throws.
[[nodiscard]] auto LoadFoveonSidecar(const std::filesystem::path& source,
                                     FoveonEditParams* params) -> bool;

/// Merges our keys into the existing sidecar (preserving all unowned keys) and
/// writes it atomically via a temporary file in the same directory.
[[nodiscard]] auto SaveFoveonSidecar(const std::filesystem::path& source,
                                     const FoveonEditParams& params) -> bool;

}  // namespace alcedo::x3f
