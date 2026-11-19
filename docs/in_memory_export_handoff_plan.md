# Temporary handoff: x3lumina to Alcedo Studio for continued editing

This plan covers the "export from x3lumina, then keep editing in Alcedo" flow. The exported
DNG or 16-bit TIFF is placed in a Studio-managed cache directory, imported as a temporary
library asset, opened in the built-in editor, and removed from both the library and the cache
when the editor closes. The intermediate file is automatic, invisible to the user as a normal
library photo, and discarded after use.

This is an extension of the protocol in `x3lumina-external-editor.md`, section 3b. It does
not replace the existing disk handoff. Both paths stay available; the user picks one at export
time in the companion app.

## Decision change found during design (recorded per AGENTS.md)

The first version of this plan proposed a pure in-memory path: put the encoded bytes in an
`Image`, skip `library.import`, and open the editor with no database record. A source review
on 2026-11-19 showed that path cannot reach `Interactive` state. The editor session has four
independent hard requirements that bytes alone do not satisfy:

- `editor.open` resolves `element_id` through `ReadAlbumFileRows`
  (`editor_commands.cpp:686-702`).
- `AcquireGuards` needs a history guard; `AcquireEditorLease` throws
  `"has no edit history root"` when the element has no commit graph and root row in DuckDB
  (`editor_session_lifecycle.cpp:49-67`, `pipeline_service.cpp:58-92`).
- `EnsureContext` checks `CurrentPreview(element_id)` and a non-empty `image_path_`
  (`editor_session_render_scheduler_port.cpp:464-496`); a missing lease means no first frame
  and no `Interactive`.
- Export rebuilds the source `Image` from `image_path_`
  (`export_service.cpp:219-227`), so a path-less image cannot be exported.

Building a memory-only lease/root and changing those checks would reach into the editor and
export core with a large regression surface. The decision is therefore changed to the
smallest on-disk form that reuses the existing, verified import/editor/export path. This is
still a temporary cache with discard-on-close; the source X3F/X3I is never touched.

## Decisions fixed for implementation

1. **Cache, then import, then edit, then discard.** The companion app writes the exported
   file into a Studio-managed cache directory under `AppLocalDataLocation`. Studio imports
   that path into a dedicated library folder, opens it in the built-in editor, and deletes
   the library asset, pipeline, pool entry, and cache file when the editor closes.
2. **Reuse the existing editor path unchanged.** No edits to the editor session, Develop
   graph, history system, or `image_path_` checks. Every capability the user exercises in
   the editor is the product's existing behavior.
3. **Encoded bytes stay untouched.** The file is moved as bytes through the cache with no
   re-encode, quantisation, or gamut mapping. Colour is resolved by the existing import and
   decode path from the file's own ICC or DNG tags.
4. **Wide-gamut is inherited, not reimplemented.** A 16-bit TIFF with an embedded ICC goes
   through `ResolveRasterColorDescription` and LittleCMS to linear Rec.2020 F32; a DNG goes
   through LibRaw to camera scene-linear then AP1. Nothing clamps into sRGB or [0, 1].
5. **The companion app's direct-export option stays.** It is a user choice, not a failure
   path. A failed handoff reports the error and keeps the app open.

## Wide-gamut invariant (mandatory)

Rule from the project constraints: every pipeline change must represent colours outside
sRGB at the code-value level with a self-consistent ICC, and must not add a step that first
clamps to sRGB/P3 then attaches a wide label, or that cuts negative or out-of-range linear
values.

How this plan satisfies it:

- The cache holds the original exported file. No conversion, quantisation, or gamut mapping
  runs in the handoff.
- The existing import path resolves the source colour description from the file bytes
  (`ResolveRasterColorDescription`, `raster_color_description.hpp:121`). The
  `icc_sha256_` field (`raster_color_description.hpp:87-88`) travels with the document so
  Develop can confirm the profile survived.
- F32 raster storage is not limited to 0..1 (`prepared_raw_input.hpp:184-186`). DNG colour
  comes from DNG metadata, not an ICC, and the container is copied verbatim.
- Verification is a required release check (phase 4): an out-of-gamut ProPhotoRGB TIFF and a
  DNG must produce decoded `PreparedRawInput` pixels and colour metadata equal to the same
  file opened through a normal import.
- Known limit, unchanged by this plan: x3lumina's TIFF writer offers only sRGB, AdobeRGB and
  ProPhotoRGB as encoded spaces (`x3f-core/src/lib.rs:69-83`). ProPhotoRGB is wide enough
  for this handoff.

## Verified code facts

| Area | Fact | Location |
| --- | --- | --- |
| Import service | `ImportToFolder(paths, dest, options, job)`; `dest` is a library folder virtual path, not a disk path; the source file is not copied | `import_service.hpp:146-149,219-221`, `import_service.cpp:286-318` |
| Import dest | A library FOLDER element resolved by `fs.Get(dest,false)`; root is `L"/"` | `import_service.cpp:286-300` |
| Delete | `library.delete` calls `StartDeleteTargets`; removes DB records, pipeline, pool entry, export tasks; does not delete the disk source | `library_commands.cpp:1428-1485`, `image_controller.cpp:875-951` |
| Delete API | `LibraryMutationOperations::StartDeleteTargets`, `DeleteTargetsFinished` signal | `library_mutation_operations.cpp:62,119` |
| Editor release | Backend enters `NoImage` and clears ids in `CompleteClose`; `StateChanged` fires | `editor_session_lifecycle.cpp:114-121`, `editor_session_controller.cpp:406` |
| Close command | Answers `closed:true` only at `NoImage`/`ShuttingDown` after persist | `editor_commands.cpp:747-784` |
| Host accessors | `project()`, `import_export()`, `editor_session()`, `workspace_router()`, `library_mutations()` | `application_module_host.hpp:147-207` |
| History root | Import creates the root via `EncodeImportedImageRoot` + `WriteImageRoots` | `import_service.cpp:41-53,181` |
| Session file | JSON file clients read to find the socket; place for the cache directory | `automation_session_file.hpp:16-25`, `.cpp:45-52` |

## Architecture

```
x3lumina (separate process)
  | 1. export to the Studio cache directory named in the session file
  | 2. alcedo-cli call library.import {"paths":["<cache file>"]}
  | 3. alcedo-cli call editor.open {"element_id": <id>}
  v
Alcedo Studio
  library.import -> ImportService::ImportToFolder
      -> SleeveFile + pinned Image + history root (existing path)
  editor.open -> WorkspaceRouter::OpenEditor -> full existing editor path
  TemporaryAssetRegistry: records (element_id, cache file path)
  on editor NoImage -> delete library asset + pipeline + pool entry + cache file
```

The only new Studio component is the registry and its close trigger. Import, edit, and export
are unchanged product behavior.

## Phase 1: cache directory and session field (Studio side)

Goal: the companion app can discover a Studio-owned cache directory. No editor changes.

1. Add a `cache_directory` string field to `AutomationSessionFile` and its JSON form. Make it
   optional on read (older session files have no field). Additive field; keep protocol version
   at 1.
   - `src/include/automation/automation_session_file.hpp`
   - `src/automation/automation_session_file.cpp` (`ToJson` at `:45-52`, parse at `:107-177`)
2. The GUI host creates the cache directory under
   `QStandardPaths::writableLocation(AppLocalDataLocation)/handoff-cache`, clears stale files
   from a previous run at startup, and writes the absolute path into the session file. Wire
   this where the automation session file is currently written.
   - `src/ui/alcedo_main/album_backend/application_module_host.cpp`
3. Verification: a unit test for writing and reading a session file with and without the new
   field; a manual check that the directory exists while Studio runs and that stale files are
   gone after restart.

## Phase 2: temporary asset registry and discard on close (Studio side)

Goal: assets imported from the cache directory are tracked and removed when editing ends.

### 2.1 Registry component

New class owned by `ApplicationModuleHost`:

```cpp
// include/app/temporary_asset_registry.hpp
namespace alcedo {
struct TemporaryAsset {
  sl_element_id_t element_id_ = 0;
  image_id_t      image_id_   = 0;
  std::filesystem::path cache_path_;
};

class TemporaryAssetRegistry {
 public:
  void               Track(TemporaryAsset asset);
  [[nodiscard]] auto TakeForImage(image_id_t image_id) -> std::optional<TemporaryAsset>;
  [[nodiscard]] auto TakeAll() -> std::vector<TemporaryAsset>;
  [[nodiscard]] auto Contains(image_id_t image_id) const -> bool;
  [[nodiscard]] auto Count() const -> std::size_t;
 private:
  std::vector<TemporaryAsset> assets_;
};
}  // namespace alcedo
```

- `Track` is called after an import finishes, with the imported ids and the cache path.
- `TakeForImage` supports a single-image close; `TakeAll` supports a batch discarded together.
- `Contains` lets the editor-close handler recognise a handoff image while it is open.
- Pure in-memory; no file and no database.

Implemented in `include/app/temporary_asset_registry.hpp` and `src/app/temporary_asset_registry.cpp`.

### 2.2 Track imported cache files

Implemented with a new `ImportExportHandler::ImportLogReady` signal emitted from
`CompleteImport` (`import_export.cpp`). `ApplicationModuleHost::OnImportLogReady` checks each
created entry's `source_path_` against the handoff cache directory by path prefix and records
the ones inside it. A normal import of an unrelated file is never tracked.

### 2.3 Discard when the editor closes

Subscribe to `EditorSessionController::StateChanged` in the host. When the backend reaches
`NoImage` with `image_id() == 0` and no persist or close in flight, discard the tracked asset
that was open:

1. Delete the library asset with `LibraryMutationOperations::StartDeleteTargetsFromLibrary`
   using the tracked `element_id`. This dedicated path forces library-level removal (folder 0,
   `delete_from_library_ = true`), so the DB record, pipeline, pool entry, and export tasks are
   removed even when the asset was imported into a subfolder. The ordinary user delete path is
   unchanged.
2. On `DeleteTargetsFinished`, delete the cache file with `QFile::remove`. The delete is
   asynchronous; wait for the signal before touching the file.
3. If the session is discarded without a library delete (for example on shutdown), at least
   remove the tracked cache files so no temporary image remains on disk.

Cleanup must not run while `close_in_flight()` or `persist_in_flight()` is true. Match by the
image id that was open so switching between images does not delete the wrong asset. Use
`RouteChanged` to `library` as an additional condition only.

### 2.4 Verification

- Unit test: `Track` then `TakeForImage` returns the asset and removes it; a second call
  returns nothing; `TakeAll` returns all remaining assets.
- Unit test: a path outside the cache directory is not tracked.
- Integration test: import a cache file, open it, close the editor, and confirm the database
  record and the cache file are gone. Verify an unrelated library photo is untouched.

## Phase 3: companion app changes (x3lumina repository)

This phase lives in the companion app, not in this tree. Studio phase 1 and 2 must be complete
and running first.

1. Read the cache directory from the live session file.
2. Add a third export destination choice for TIFF/DNG: "Hand back to Alcedo for editing". On
   that choice, write the export into the cache directory, then call:
   ```sh
   alcedo-cli --json --session gui call library.import '{"paths":["<cache file>"]}'
   alcedo-cli --json --session gui call editor.open '{"element_id":<id>}'
   ```
   Recover the element id by matching the imported file name, as `AlcedoHandoff.swift`
   already does for the current handoff.
3. Keep "direct export" and the current handoff as explicit user choices. A failed handoff
   reports the error and leaves the app open. No automatic retry through another path.

## Phase 4: verification, wide-gamut proof, and hardening

Release condition, not optional cleanup.

- **Byte-equivalence test (done):** `tests/image/handoff_gamut_preservation_test.cpp` reproduces
  the companion app's exact write (`Data(contentsOf:)` + atomic write) on ProPhoto 16-bit,
  Rec.2020 16-bit, and float32-linear TIFF fixtures. It asserts byte-identical copies, equal
  `RasterColorDescription` (primaries, transfer, peak, `icc_sha256_`), and wide primaries that
  really resolve from the embedded ICC rather than a `DefaultSrgb` fallback. Because the cache
  file is byte-identical to the export, every downstream decode consumes the same bytes and
  yields the same planes and colour context.
- **Wide-gamut code-value test (done):** the same suite writes negative and over-range (>1.0)
  linear floats into a real float32 TIFF strip, copies it through the cache path, and asserts
  the exact bit patterns and the ICC survive. A future change that clamps to [0, 1] or maps to
  sRGB/P3 fails these tests.
- **Decode-equivalence test (done):** the same suite feeds the identical bytes from two distinct
  paths — one standing in for the ordinary export location, one for the Studio cache — through
  `RasterInputLoader::LoadEncoded` for all three wide-gamut TIFFs. It asserts the decoded
  `PreparedRawInput` matches on `input_kind`, all extents, pixel format, stride, `source_key`,
  ICC-conversion flag and digest, and the entire host plane byte for byte. This proves a cache
  import decodes exactly like a normal import, without a GPU editor backend or an LFS RAW
  fixture (which is why the rendered-export comparison can stay manual, see below).
- **Lifecycle tests (done):** `tests/ui/handoff_asset_lifecycle_test.cpp` publishes the import
  log through the production signal and asserts that a source file inside the cache is tracked
  and its cache file removed at shutdown, while a normal import is never treated as temporary.
  `tests/ui/album_backend_image_delete_test.cpp` covers the forced library-level delete on a
  subfolder. Neither needs a GPU editor backend.
- **Still manual before public release (not automated here):**
  - a real DNG exported by x3lumina, handed off to a running GUI Studio, opened to
    `Interactive`, closed, and checked end to end (the local CI RAW fixtures are LFS
    placeholders, so an editor `Interactive` E2E cannot run in this checkout);
  - a *rendered* export byte comparison of cache import versus normal import through the GPU
    executor. The decode boundary is already covered automatically above; the render step needs
    the GPU backend and a real image, so it stays a manual check;
  - a cache size and file-count cap. The cache is currently cleared at every Studio start
    (`PrepareHandoffCacheDirectory`) and drained on editor close and shutdown, so it does not
    grow without bound across sessions; only a single-session cap is not enforced.
- **Manual acceptance steps:** `docs/handoff_acceptance_checklist.md` lists the executable
  steps for the three items above, plus the expected failure behaviour. Run it on a machine
  with a real X3F/X3I, a built `x3lumina`, and a running GUI Studio.
- **Docs (done):** `x3lumina-external-editor.md` section 3b carries the third destination and
  the temporary-asset lifecycle.

## Files changed, by area

| New | `include/app/temporary_asset_registry.hpp`, `src/app/temporary_asset_registry.cpp` |
| --- | --- |
| Session | `src/include/automation/automation_session_file.hpp`, `src/automation/automation_session_file.cpp` |
| Host / cache / wiring | `src/ui/alcedo_main/album_backend/application_module_host.cpp`, `application_module_host.hpp` |
| Import completion | the `ImportExportHandler` completion path that resolves imported ids |
| Forced delete | `image_controller.hpp/.cpp`, `library_mutation_operations.hpp/.cpp` (library-level delete for temporary assets) |
| Build | `src/app/CMakeLists.txt` (new registry library), `src/ui/alcedo_main/CMakeLists.txt` (link) |
| Tests | `tests/app/temporary_asset_registry_test.cpp`, `tests/automation/{automation_session_file_test,handoff_cache_test}.cpp`, `tests/image/handoff_gamut_preservation_test.cpp`, `tests/ui/handoff_asset_lifecycle_test.cpp`, `tests/ui/album_backend_image_delete_test.cpp` |
| Companion app (done) | `outputs/X3FExtractor/Sources/X3FExtractor/{AlcedoHandoff,ExternalLaunch,Views}.swift` (separate repository) |
| Docs | `docs/x3lumina-external-editor.md` |

## Risks and how they are handled

- **Cleanup races with save.** Do not delete while persist or close is in flight; wait for
  `NoImage` and for `DeleteTargetsFinished`.
- **Deleting the wrong asset on a switch.** Match by the open image id; use `TakeForImage`, not
  `TakeAll`, for a single close.
- **Disk file survives a database delete.** The delete path never removes source files; the
  registry removes the cache file explicitly after the delete finishes.
- **A stale cache file from a crashed run.** Clear the cache directory at startup.
- **Delete scope depends on the current folder — closed by a dedicated path.** By default
  `ImageController::PrepareDeleteBatch` sets `delete_from_library_ = CurrentFolderElementId() == 0`,
  and `library.import` imports into the current folder with no destination parameter, so a normal
  delete in a subfolder would only unlink the album row. The handoff cleanup therefore calls
  `LibraryMutationOperations::StartDeleteTargetsFromLibrary`, which runs
  `ImageController::StartDeleteImagesFromLibrary`. That path forces `delete_from_library_ = true`
  and `folder_id = 0`, so the delete job calls `DeleteFilesEverywhere` and removes the library
  row, pipeline, and image-pool entry regardless of which folder is selected or was imported
  into. The ordinary user delete path is unchanged. The cleanup checks `deletedElementIds` in
  `DeleteTargetsFinished` so an unrelated delete cannot remove a cache file that is still in use.
- **Colour metadata is lost.** The ICC hash check in phase 4 catches it; never rebuild an
  image from decoded pixels.

## Rollout order

1. Phase 1 (cache directory and session field) — harmless on its own.
2. Phase 2 (registry and discard on close) — completes the Studio side.
3. Phase 3 (companion app) — makes the flow reachable by users.
4. Phase 4 (proof and hardening) — a precondition for the public release.

Each phase builds and runs its own verification before the next starts.
