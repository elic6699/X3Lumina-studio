//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "app/handoff_color_rebind.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include "app/handoff_proxy_registry.hpp"
#include "edit/graph/drt_node_model.hpp"
#include "edit/graph/pipeline_document.hpp"
#include "edit/graph/pipeline_graph_snapshot.hpp"
#include "image/image.hpp"
#include "image/metadata_extractor.hpp"

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

TEST_F(HandoffColorRebindTest, TiffRebindLeavesEntireDrtUntouched) {
  // A rec2020 TIFF must NOT rewrite the DRT output encoding. Doing so previously forced
  // encoding_space=rec2020 onto a FilmPrint DRT, and CheckFilmPrintEncoding rejects anything but
  // Rec709 -- every interactive frame then failed and the viewport froze (exposure/film edits
  // appeared dead). The DRT is display intent and stays exactly as the working document had it.
  const auto path = FixturePath("rec2020_icc_16bit.tif");

  auto document = std::make_shared<PipelineDocument>(CreateDefaultPipelineDocument());
  ASSERT_NE(document->Drt(), nullptr);
  auto payload             = document->Drt()->Params().Params();
  payload.method           = DrtMethod::FilmPrint;
  payload.film_print.negative       = "kodak_portra_800";
  payload.film_print.print_material = "cine_look";
  payload.film_print.scene_light    = "daylight";
  payload.encoding_space            = DrtColorSpace::Rec709;   // film-print legal display intent
  payload.encoding_eotf             = DrtEotf::SrgbPiecewise;
  payload.limiting_space            = DrtColorSpace::Rec709;
  payload.peak_luminance            = 777.0f;
  document->Drt()->Params().ReplaceParams(std::move(payload));
  const auto expected = document->Drt()->Params().Params();

  auto source = PipelineGraphSnapshot::Preview(document->Freeze(), 8, PipelineLineageId::Next(),
                                               transaction_chain_hash_t{});
  const auto rebound = RebindSnapshotColorForHandoff(8, path, source);
  ASSERT_NE(rebound, source);
  ASSERT_NE(rebound->Document().Drt(), nullptr);

  // Only Develop input is transplanted from the TIFF; the whole DRT is bit-for-bit the user's edit.
  const auto& drt  = rebound->Document().Drt()->Params().Params();
  EXPECT_EQ(drt.method, expected.method);
  EXPECT_EQ(drt.film_print.negative, expected.film_print.negative);
  EXPECT_EQ(drt.film_print.print_material, expected.film_print.print_material);
  EXPECT_EQ(drt.film_print.scene_light, expected.film_print.scene_light);
  EXPECT_EQ(drt.encoding_space, expected.encoding_space);
  EXPECT_EQ(drt.encoding_eotf, expected.encoding_eotf);
  EXPECT_EQ(drt.limiting_space, expected.limiting_space);
  EXPECT_FLOAT_EQ(drt.peak_luminance, expected.peak_luminance);
}

}  // namespace
}  // namespace alcedo
