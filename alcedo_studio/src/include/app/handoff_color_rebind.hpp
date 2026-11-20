//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <filesystem>
#include <memory>

#include "edit/graph/pipeline_graph_snapshot.hpp"
#include "type/type.hpp"

namespace alcedo {

/// Returns a *render-only* copy of @p source whose Develop colour description is bound to the
/// handoff file at @p cache_path, or @p source unchanged when no rebind is needed/possible.
///
/// A handoff (see HandoffProxyRegistry) keeps the library image as the original X3F/X3I while the
/// decoded bytes are temporarily redirected to an external editor's DNG / 16-bit TIFF. The editor
/// session's working document (and its Develop camera profile / raster input) still belongs to the
/// X3F, so rendering those DNG/TIFF bytes with it produces a wrong colour cast or a black frame.
/// This helper clones the snapshot's document and rebinds only the colour description from the
/// cache file -- the same import-time binding path the first-stage "import DNG as a new photo"
/// uses -- then freezes it into a new preview snapshot for this render. The working document, the
/// stored history, `image_path_`, the X3F file and the encoded bytes are never modified; the
/// rebind disappears together with the proxy when the editor closes.
///
/// Wide gamut: this is a pure colour-metadata binding. No pixel is decoded, quantised, gamut-mapped
/// or clipped here. Matrices/descriptions are read from the handoff file itself, so out-of-gamut
/// and negative linear values keep their exact code values and the file's own ICC/colour tags win.
///
/// Results are cached per (source snapshot, cache path) so a frame that reuses the same snapshot
/// does no per-frame metadata work. @p source is kept alive by the caller for the render; the
/// returned snapshot owns its cloned document independently.
[[nodiscard]] auto RebindSnapshotColorForHandoff(
    image_id_t image_id, const std::filesystem::path& cache_path,
    const std::shared_ptr<const PipelineGraphSnapshot>& source)
    -> std::shared_ptr<const PipelineGraphSnapshot>;

/// Returns the rebound snapshot for @p image_id when a handoff proxy is active and the colour
/// rebind produced a usable snapshot; otherwise returns @p source. Convenience wrapper that
/// consults HandoffProxyRegistry directly, for the export path.
[[nodiscard]] auto RebindSnapshotColorIfHandoff(
    image_id_t image_id, const std::shared_ptr<const PipelineGraphSnapshot>& source)
    -> std::shared_ptr<const PipelineGraphSnapshot>;

/// Drop cached rebindings for @p image_id. Called when the proxy/caches are released so a later
/// handoff of the same image cannot reuse a stale colour binding.
void ClearHandoffColorRebind(image_id_t image_id);

}  // namespace alcedo
