//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "app/temporary_asset_registry.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace alcedo {
namespace {

auto Asset(sl_element_id_t element, image_id_t image, const char* path) -> TemporaryAsset {
  TemporaryAsset asset;
  asset.element_id_ = element;
  asset.image_id_   = image;
  asset.cache_path_ = std::filesystem::path(path);
  return asset;
}

TEST(TemporaryAssetRegistryTest, TakeForImageReturnsAndRemovesOnlyTheMatchingAsset) {
  TemporaryAssetRegistry registry;
  registry.Track(Asset(10, 100, "/cache/a.tif"));
  registry.Track(Asset(11, 101, "/cache/b.dng"));
  ASSERT_EQ(registry.Count(), 2U);

  const auto taken = registry.TakeForImage(100);
  ASSERT_TRUE(taken.has_value());
  EXPECT_EQ(taken->element_id_, 10U);
  EXPECT_EQ(taken->image_id_, 100U);
  EXPECT_EQ(taken->cache_path_, std::filesystem::path("/cache/a.tif"));
  EXPECT_EQ(registry.Count(), 1U);

  EXPECT_FALSE(registry.TakeForImage(100).has_value());
  EXPECT_EQ(registry.Count(), 1U);
  EXPECT_TRUE(registry.TakeForImage(101).has_value());
  EXPECT_EQ(registry.Count(), 0U);
}

TEST(TemporaryAssetRegistryTest, TakeAllReturnsEveryAssetAndEmptiesTheRegistry) {
  TemporaryAssetRegistry registry;
  registry.Track(Asset(20, 200, "/cache/c.tif"));
  registry.Track(Asset(21, 201, "/cache/d.dng"));

  const auto all = registry.TakeAll();
  ASSERT_EQ(all.size(), 2U);
  EXPECT_EQ(all[0].image_id_, 200U);
  EXPECT_EQ(all[1].image_id_, 201U);
  EXPECT_EQ(registry.Count(), 0U);
  EXPECT_TRUE(registry.TakeAll().empty());
}

TEST(TemporaryAssetRegistryTest, EntriesWithoutIdsOrPathAreIgnored) {
  TemporaryAssetRegistry registry;
  registry.Track(Asset(0, 100, "/cache/a.tif"));
  registry.Track(Asset(10, 0, "/cache/a.tif"));
  registry.Track(Asset(10, 100, ""));
  EXPECT_EQ(registry.Count(), 0U);
}

}  // namespace
}  // namespace alcedo
