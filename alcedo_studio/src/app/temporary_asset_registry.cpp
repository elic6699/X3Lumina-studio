//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "app/temporary_asset_registry.hpp"

#include <algorithm>

namespace alcedo {

void TemporaryAssetRegistry::Track(TemporaryAsset asset) {
  if (asset.element_id_ == 0 || asset.image_id_ == 0 || asset.cache_path_.empty()) {
    return;
  }
  assets_.push_back(std::move(asset));
}

auto TemporaryAssetRegistry::TakeForImage(image_id_t image_id)
    -> std::optional<TemporaryAsset> {
  const auto found = std::find_if(assets_.begin(), assets_.end(),
                                  [image_id](const TemporaryAsset& asset) {
                                    return asset.image_id_ == image_id;
                                  });
  if (found == assets_.end()) {
    return std::nullopt;
  }
  TemporaryAsset result = std::move(*found);
  assets_.erase(found);
  return result;
}

auto TemporaryAssetRegistry::TakeAll() -> std::vector<TemporaryAsset> {
  std::vector<TemporaryAsset> result;
  result.swap(assets_);
  return result;
}

auto TemporaryAssetRegistry::Contains(image_id_t image_id) const -> bool {
  return std::any_of(assets_.begin(), assets_.end(), [image_id](const TemporaryAsset& asset) {
    return asset.image_id_ == image_id;
  });
}

auto TemporaryAssetRegistry::Count() const -> std::size_t { return assets_.size(); }

}  // namespace alcedo
