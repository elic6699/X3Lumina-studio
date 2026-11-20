//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "app/handoff_color_rebind.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include "app/handoff_proxy_registry.hpp"
#include "edit/graph/pipeline_document.hpp"
#include "edit/graph/pipeline_graph_snapshot.hpp"

namespace alcedo {
namespace {

auto FixturePath(const std::string& name) -> std::filesystem::path {
  return std::filesystem::path(ALCEDO_RASTER_FIXTURE_DIR) / name;
}

class HandoffColorRebindTest : public ::testing::Test {
 protected:
  void SetUp() override { HandoffProxyRegistry::Instance().Reset(); }
  void TearDown() override { HandoffProxyRegistry::Instance().Reset(); }
};

TEST_F(HandoffColorRebindTest, NullAndEmptyInputsPassThrough) {
  EXPECT_EQ(RebindSnapshotColorForHandoff(1, {}, nullptr), nullptr);
  EXPECT_EQ(RebindSnapshotColorIfHandoff(1, nullptr), nullptr);
  ClearHandoffColorRebind(1);  // clearing an unknown image is a safe no-op
}

TEST_F(HandoffColorRebindTest, TiffRebindKeepsWorkingAdjustments) {
  auto document = std::make_shared<PipelineDocument>(CreateDefaultPipelineDocument());
  ASSERT_NE(document->Develop(), nullptr);
  auto develop_payload              = document->Develop()->Params().Params();
  develop_payload.highlights_reconstruct = true;
  document->Develop()->Params().ReplaceParams(develop_payload);

  auto source = PipelineGraphSnapshot::Preview(document->Freeze(), 7, PipelineLineageId::Next(),
                                               transaction_chain_hash_t{});
  const auto rebound =
      RebindSnapshotColorForHandoff(7, FixturePath("rec2020_icc_16bit.tif"), source);
  ASSERT_NE(rebound, source);
  ASSERT_NE(rebound->Document().Develop(), nullptr);

  // The TIFF's own raster description is installed, but the editor's current develop value
  // survives; this is what makes the viewport react after a colour rebind.
  EXPECT_TRUE(rebound->Document().Develop()->Params().RasterInput().has_value());
  EXPECT_TRUE(rebound->Document().Develop()->Params().Params().highlights_reconstruct);
}

}  // namespace
}  // namespace alcedo
