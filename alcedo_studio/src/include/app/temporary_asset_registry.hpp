//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <filesystem>
#include <optional>
#include <vector>

#include "type/type.hpp"

namespace alcedo {

/// One library asset that came from a temporary export handoff file. The cache path is kept
/// so the file can be removed when the editor no longer holds the image.
struct TemporaryAsset {
  sl_element_id_t            element_id_ = 0;
  image_id_t                 image_id_   = 0;
  std::filesystem::path      cache_path_;
};

/// In-memory record of the temporary assets of one running session. Not thread-safe; the GUI
/// thread calls it from import completion and from the editor close handler.
class TemporaryAssetRegistry {
 public:
  /// Records a handoff asset. Ignores an entry without ids or with an empty path.
  void               Track(TemporaryAsset asset);
  /// Returns and removes the asset bound to @p image_id, if any. Used when one image closes.
  [[nodiscard]] auto TakeForImage(image_id_t image_id) -> std::optional<TemporaryAsset>;
  /// Returns and removes every remaining asset. Used on shutdown.
  [[nodiscard]] auto TakeAll() -> std::vector<TemporaryAsset>;
  /// True when an asset with @p image_id is tracked.
  [[nodiscard]] auto Contains(image_id_t image_id) const -> bool;
  /// Number of tracked assets, for tests.
  [[nodiscard]] auto Count() const -> std::size_t;

 private:
  std::vector<TemporaryAsset> assets_;
};

}  // namespace alcedo
