//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "app/handoff_color_rebind.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include "app/handoff_proxy_registry.hpp"
#include "edit/graph/develop_color_transform.hpp"
#include "edit/graph/pipeline_document.hpp"
#include "image/image.hpp"
#include "image/metadata_extractor.hpp"

namespace alcedo {
namespace {

struct CacheKey {
  const PipelineGraphSnapshot* source = nullptr;
  std::filesystem::path         cache_path;
};

struct CacheKeyHash {
  auto operator()(const CacheKey& key) const noexcept -> std::size_t {
    const auto ptr = std::hash<const PipelineGraphSnapshot*>{}(key.source);
    const auto path = std::hash<std::filesystem::path>{}(key.cache_path);
    return ptr ^ (path << 1);
  }
};

auto operator==(const CacheKey& a, const CacheKey& b) -> bool {
  return a.source == b.source && a.cache_path == b.cache_path;
}

/// Binds the colour description of @p cache_path into a fresh clone of @p source's document.
/// Returns nullptr when the file cannot supply a usable description, in which case the caller
/// renders the untouched original snapshot.
auto BuildReboundSnapshot(const std::filesystem::path& cache_path,
                          const std::shared_ptr<const PipelineGraphSnapshot>& source)
    -> std::shared_ptr<const PipelineGraphSnapshot> {
  if (!source) {
    return nullptr;
  }

  // A DNG carries its own colour matrices, as-shot neutral and an embedded DNG profile. Reuse the
  // exact import-time binding the first-stage handoff proved correct: extracting the metadata
  // yields a *bound* DngColorProfileRef, so BindImportedCameraProfile's RequireBound is satisfied.
  const auto ext = cache_path.extension().string();
  std::string lower_ext = ext;
  std::transform(lower_ext.begin(), lower_ext.end(), lower_ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lower_ext == ".dng") {
    Image dng_probe;
    if (MetadataExtractor::ExtractRawMetadata_ToImage(cache_path, dng_probe) &&
        dng_probe.HasRawColorContext()) {
      auto document = std::make_shared<PipelineDocument>(ClonePipelineDocument(source->Document()));
      BindImportedCameraProfile(*document, dng_probe.GetRawColorContext());
      return PipelineGraphSnapshot::Preview(document->Freeze(), source->ElementId(),
                                            source->Lineage(), source->Chain());
    }
  }

  // A 16-bit TIFF is a raster: its pixels are already encoded, so the Develop `input` object must
  // come from the handoff TIFF, not from the X3F RAW document. Replacing the whole document with
  // CreateDefaultRasterPipelineDocument would also drop the editor's current adjustment chain,
  // which is why the viewport stopped reacting while an out-of-band batch render still changed.
  // Clone the working document and transplant only the Develop input; every grade/adjustment and
  // the entire DRT are left untouched. No colour math or gamut clipping happens here: the raster
  // description is copied byte-for-byte into the render-only clone.
  //
  // The DRT is deliberately NEVER replaced here. Its output encoding (encoding_space / eotf /
  // limiting_space / peak_luminance) is the user's *display intent*, independent of whether the
  // input bytes came from an X3F, a DNG or an exported TIFF -- exactly why the DNG branch above
  // binds only the camera profile and leaves the DRT alone. Overwriting those fields from the
  // TIFF's raster description forced encoding_space to rec2020 and made every interactive frame
  // fail DrtOutputResolver::CheckFilmPrintEncoding ("film print encodes only Rec. 709"), freezing
  // the viewport so *all* edits (exposure, film stock, ...) appeared to do nothing. Keeping the
  // working document's DRT also preserves the user's Film Print negative/material/scene light and
  // every other DRT edit verbatim.
  if (const auto kind = MetadataExtractor::ClassifyRasterFile(cache_path); kind.has_value()) {
    Image raster_probe;
    MetadataExtractor::ExtractRasterMetadata_ToImage(cache_path, *kind, raster_probe);
    if (raster_probe.HasRasterColorDescription()) {
      auto document = std::make_shared<PipelineDocument>(ClonePipelineDocument(source->Document()));
      const auto raster_defaults =
          CreateDefaultRasterPipelineDocument(raster_probe.GetRasterColorDescription());
      if (auto* develop = document->Develop(); develop != nullptr && raster_defaults.Develop()) {
        auto payload              = develop->Params().Params();
        payload.input             = raster_defaults.Develop()->Params().Params().input;
        payload.lens_enabled      = false;
        develop->Params().ReplaceParams(std::move(payload));
      }
      return PipelineGraphSnapshot::Preview(document->Freeze(), source->ElementId(),
                                            source->Lineage(), source->Chain());
    }
  }

  return nullptr;
}

class ColorRebindCache {
 public:
  static auto Instance() -> ColorRebindCache& {
    static ColorRebindCache instance;
    return instance;
  }

  auto Get(image_id_t image_id, const std::filesystem::path& cache_path,
           const std::shared_ptr<const PipelineGraphSnapshot>& source)
      -> std::shared_ptr<const PipelineGraphSnapshot> {
    const CacheKey key{source.get(), cache_path};
    std::lock_guard lock(mutex_);
    auto& by_image = entries_[image_id];
    if (auto it = by_image.find(key); it != by_image.end()) {
      return it->second;
    }
    // Metadata extraction needs no lock and is independent of other images; compute outside the
    // lock would risk duplicate work, but this path is one-shot per (snapshot, cache) pair and the
    // critical section is short, so building under the lock keeps the logic simple and correct.
    // Any extraction/binding failure degrades to the untouched source snapshot so a malformed
    // cache file can never break a render or an export.
    std::shared_ptr<const PipelineGraphSnapshot> rebound;
    try {
      rebound = BuildReboundSnapshot(cache_path, source);
    } catch (...) {
      rebound.reset();
    }
    auto result = rebound ? rebound : source;
    by_image.emplace(key, result);
    return result;
  }

  void Clear(image_id_t image_id) {
    std::lock_guard lock(mutex_);
    entries_.erase(image_id);
  }

 private:
  ColorRebindCache() = default;
  mutable std::mutex mutex_;
  std::unordered_map<image_id_t, std::unordered_map<CacheKey, std::shared_ptr<const PipelineGraphSnapshot>,
                                                   CacheKeyHash>> entries_;
};

}  // namespace

auto RebindSnapshotColorForHandoff(image_id_t image_id, const std::filesystem::path& cache_path,
                                   const std::shared_ptr<const PipelineGraphSnapshot>& source)
    -> std::shared_ptr<const PipelineGraphSnapshot> {
  if (!source || cache_path.empty()) {
    return source;
  }
  return ColorRebindCache::Instance().Get(image_id, cache_path, source);
}

auto RebindSnapshotColorIfHandoff(image_id_t image_id,
                                  const std::shared_ptr<const PipelineGraphSnapshot>& source)
    -> std::shared_ptr<const PipelineGraphSnapshot> {
  if (!source) {
    return source;
  }
  const auto proxy = HandoffProxyRegistry::Instance().ProxyPathFor(image_id);
  if (!proxy.has_value()) {
    return source;
  }
  return RebindSnapshotColorForHandoff(image_id, *proxy, source);
}

void ClearHandoffColorRebind(image_id_t image_id) { ColorRebindCache::Instance().Clear(image_id); }

}  // namespace alcedo
