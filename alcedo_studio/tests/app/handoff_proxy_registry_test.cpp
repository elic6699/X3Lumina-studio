//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "app/handoff_proxy_registry.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace alcedo {
namespace {

auto ProxyFor(image_id_t image, const char* cache) -> HandoffProxyRegistry::Proxy {
  HandoffProxyRegistry::Proxy proxy;
  proxy.element_id_  = 7;
  proxy.image_id_    = image;
  proxy.source_path_ = std::filesystem::path("/library/original.x3f");
  proxy.cache_path_  = std::filesystem::path(cache);
  return proxy;
}

class HandoffProxyRegistryTest : public ::testing::Test {
 protected:
  void SetUp() override { HandoffProxyRegistry::Instance().Reset(); }
  void TearDown() override { HandoffProxyRegistry::Instance().Reset(); }
};

TEST_F(HandoffProxyRegistryTest, ProxyPathForReturnsCachePathWithoutRemoving) {
  auto& registry = HandoffProxyRegistry::Instance();
  registry.Track(ProxyFor(100, "/cache/a.dng"));

  const auto path = registry.ProxyPathFor(100);
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, std::filesystem::path("/cache/a.dng"));
  // Read-only query keeps the entry registered.
  EXPECT_TRUE(registry.Contains(100));
  EXPECT_EQ(registry.Count(), 1U);
}

TEST_F(HandoffProxyRegistryTest, NoProxyForUnregisteredImage) {
  auto& registry = HandoffProxyRegistry::Instance();
  registry.Track(ProxyFor(100, "/cache/a.dng"));
  EXPECT_FALSE(registry.ProxyPathFor(999).has_value());
  EXPECT_FALSE(registry.Contains(999));
}

TEST_F(HandoffProxyRegistryTest, ReTrackReplacesProxyForSameImage) {
  auto& registry = HandoffProxyRegistry::Instance();
  registry.Track(ProxyFor(100, "/cache/first.dng"));
  registry.Track(ProxyFor(100, "/cache/second.dng"));

  EXPECT_EQ(registry.Count(), 1U);
  EXPECT_EQ(registry.ProxyPathFor(100),
            std::optional<std::filesystem::path>(std::filesystem::path("/cache/second.dng")));
}

TEST_F(HandoffProxyRegistryTest, TakeForImageReturnsAndRemovesOnlyMatch) {
  auto& registry = HandoffProxyRegistry::Instance();
  registry.Track(ProxyFor(100, "/cache/a.dng"));
  registry.Track(ProxyFor(101, "/cache/b.tif"));

  const auto taken = registry.TakeForImage(100);
  ASSERT_TRUE(taken.has_value());
  EXPECT_EQ(taken->cache_path_, std::filesystem::path("/cache/a.dng"));
  EXPECT_EQ(taken->source_path_, std::filesystem::path("/library/original.x3f"));
  EXPECT_FALSE(registry.ProxyPathFor(100).has_value());
  EXPECT_TRUE(registry.ProxyPathFor(101).has_value());
}

TEST_F(HandoffProxyRegistryTest, EntriesWithoutImageOrCachePathAreIgnored) {
  auto& registry = HandoffProxyRegistry::Instance();
  auto no_image  = ProxyFor(0, "/cache/a.dng");
  auto no_cache  = ProxyFor(100, "");
  registry.Track(no_image);
  registry.Track(no_cache);
  EXPECT_EQ(registry.Count(), 0U);
}

TEST_F(HandoffProxyRegistryTest, TakeAllEmptiesRegistry) {
  auto& registry = HandoffProxyRegistry::Instance();
  registry.Track(ProxyFor(100, "/cache/a.dng"));
  registry.Track(ProxyFor(101, "/cache/b.tif"));
  const auto all = registry.TakeAll();
  EXPECT_EQ(all.size(), 2U);
  EXPECT_EQ(registry.Count(), 0U);
}

}  // namespace
}  // namespace alcedo
