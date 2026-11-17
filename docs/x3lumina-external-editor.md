# X3Lumina companion editor (external bridge)

Alcedo Studio manages every photo, including Sigma X3F/X3I files. It does **not** decode or
edit a Foveon file itself. Editing is owned by a separate app, `x3lumina`, that runs as its own
process and can be upgraded independently of Alcedo Studio.

This document is the protocol both sides follow. It has four parts: import and culling,
the handoff on double-click, the refresh after an edit, and batch export.

## Invariants

- The source `.x3f` / `.x3i` file is **read-only**. Nothing in this protocol writes to it.
- Alcedo Studio's Develop graph, colour pipeline, and export path never receive a Foveon
  source or a companion-app product. Only the DAM preview (a display-referred JPEG) changes.
- Wide-gamut rendering and export happen only inside `x3lumina`, whose C/Rust decode stack is
  not linked into Alcedo Studio.

## 1. Import and culling (Alcedo Studio side)

`.x3f` and `.x3i` are import categories. Import metadata comes from the camera's embedded
JPEG, so no Foveon decode runs during a bulk import. Rating, color label, reject, and album
membership are ordinary DAM operations and work on these files like any other photo.

The culling preview is a **sidecar JPEG** next to the source:

```
<source directory>/<source name>.x3fproxy.jpg
```

Example: `photos/2017/SD14_0014.X3F` → `photos/2017/SD14_0014.X3F.x3fproxy.jpg`.

The suffix is `.x3fproxy.jpg` and is appended to the full source name. `ThumbnailService`
reads this file when present; when absent it falls back to the built-in reader. The sidecar
is a plain camera JPEG (SOI..EOI, no colour transform), so no ICC rewrite is needed for a
display-referred preview.

## 2. Handoff on double-click (Alcedo Studio → x3lumina)

Double-clicking an X3F/X3I in the library does **not** open the built-in editor. Alcedo Studio
resolves the image id to its source path and starts the companion app:

```
<app> [extra args...] <absolute source path>
```

Configuration is in QSettings (NativeFormat, UserScope):

| Key | Meaning | Default |
| --- | --- | --- |
| `externalEditor/x3luminaAppPath` | Path to the `x3lumina` executable or `.app` | empty (handoff disabled) |
| `externalEditor/x3luminaArgs` | Extra arguments, split with shell-like quoting | empty |

The source path is always appended as the last argument. If the app path is empty, the
handoff is disabled and the ordinary editor route proceeds. Configure the app path once;
it is not stored in the project.

### How to write these settings on macOS (important)

The app reads these with `QSettings` in `NativeFormat`. On macOS, Qt 6 stores the `/` group
separator as a **dot** in the plist. So the on-disk key is `externalEditor.x3luminaAppPath`,
and that is the spelling `defaults` must use:

```sh
defaults write com.alcedo-app.Alcedo "externalEditor.x3luminaAppPath" \
  "/Applications/X3Lumina.app/Contents/MacOS/X3FExtractor"
killall cfprefsd
```

Do **not** use `externalEditor/x3luminaAppPath` (slash) with `defaults` — Qt reads that as a
different key and returns empty. Do **not** use `defaults write ... externalEditor -dict-add`
either: that creates a nested `QVariantMap`. Current builds accept the nested form as a
fallback, but the dot-separated flat key above is what QSettings itself writes and is the
reliable choice.

To confirm what the app will read:

```sh
defaults read com.alcedo-app.Alcedo "externalEditor.x3luminaAppPath"
```

`x3lumina` must accept the handoff and open a window with a preview and an editing surface.
It needs no other UI module.

### Companion-app side: the `--external-edit` entry point

The stock X3Lumina Mac app (Swift, `outputs/X3FExtractor`) does **not** read dropped file
paths or Apple Events — it only remembers a browse directory and selects the first photo.
So we added a dedicated entry point in that project (`ExternalLaunch.swift`, `App.swift`,
`Views.swift`):

```sh
X3FExtractor --external-edit "/absolute/path/IMG.X3F"
```

On launch it opens the photo's directory, selects exactly that file (by path), and hides
the bottom filmstrip — leaving the left directory pane, center editor, and right parameter
pane. No decode/colour/export pipeline code was touched. This is why `x3luminaArgs` must be
set to `--external-edit`: the launcher appends the source path as the final argument,
producing `--external-edit <path>`.

Only the *UI layout and startup selection* changed in the companion app; the handoff here
in Alcedo Studio is unchanged. The companion app is built independently with its own
`make_app.sh` and can be upgraded on its own schedule.

## 3. Refresh after an edit (x3lumina → Alcedo Studio)

After writing a new sidecar JPEG, `x3lumina` tells Alcedo Studio to drop the cached previews.
Alcedo Studio exposes a local JSON-RPC socket; its session file lives in the automation
session directory. Use the repository CLI (`alcedo-cli`), which discovers the socket:

```sh
alcedo-cli call external.refreshThumbnail '{"element_ids": [12, 34]}'
```

`element_ids` are the library element ids of the photos whose proxies changed. The command
invalidates the cached thumbnails of exactly those elements; the library renders the new
previews on the next request. It does not touch source files, Develop documents, or history.

`x3lumina` needs the element ids. Two ways to get them:

- When Alcedo Studio starts the handoff, also pass the element id (add it to
  `externalEditor/x3luminaArgs`, for example `--element-id 12`). The app echoes it back on
  export. This is the simple path for one-photo edits.
- For a batch, call `alcedo-cli call library.info` to map source paths to element ids first.

The refresh command is best-effort: a failed call leaves the old preview cached, which is
harmless. There is no retry protocol.

## 3b. Export destination in external mode (x3lumina → Alcedo Studio editor)

In external-edit mode `--external-edit <path>`, `x3lumina` also changes what happens **after**
an export. This is the only behavioural difference; the decode/colour/export pipeline is
untouched (the TIFF/DNG is still produced by x3lumina's own wide-gamut pipeline).

- **JPEG (embedded extract) and JPEG (RAW re-encode)**: on export completion, x3lumina
  terminates itself. No dialog — the user is handing a finished JPEG back to the DAM.
- **16-bit TIFF and DNG**: pressing "start export" first shows a two-way choice:
  1. **直接导出 (Direct export)** — write the file with x3lumina's pipeline, then terminate.
  2. **回传 Alcedo 继续编辑 (Hand back to Alcedo)** — write the file, then import it into the
     running Alcedo Studio library and open it in Alcedo's built-in editor, where the user
     continues editing and exports. x3lumina terminates once the handoff succeeds.

The handoff (option 2) is implemented in `AlcedoHandoff.swift` as a sequence of `alcedo-cli`
automation calls over the GUI host's JSON-RPC socket:

```sh
alcedo-cli --json --session gui call library.import  '{"paths":["<exported.tif>"]}'   # -> task_id
alcedo-cli --json --session gui call tasks.wait      '{"task_id":"<id>"}'              # wait for import
alcedo-cli --json --session gui call search.query    '{"text":"<stem>","mode":"fuzzy"}' # -> element_id
alcedo-cli --json --session gui call editor.open     '{"element_id":<id>}'              # open in editor
```

Notes:

- `element_id` is recovered by matching the imported file's `file_name` against the exported
  basename (search is retried a few times because its index is asynchronous).
- The `--session` name is read from the live GUI session file; without it, `alcedo-cli`
  refuses when more than one session file is live.
- The `alcedo-cli` path is taken from an optional `--alcedo-cli <path>` launch argument, then
  `ALCEDO_CLI`, then a list of well-known locations (installed app / build trees). Pass it from
  `externalEditor/x3luminaArgs` if the app is installed somewhere non-standard.
- TIFF/DNG are **not** `.x3f`/`.x3i`, so `editor.open` does not re-trigger the external bridge;
  it lands in Alcedo's own editor. This is what makes "continue editing in Alcedo" possible.
- The handoff is best-effort. If any CLI call fails, x3lumina stays open, shows the error with
  the output path, and the user can import the file manually. Nothing is written back to the
  source X3F/X3I.

Relevant companion-app files: `ExternalLaunch.swift` (post-export dispatch, foreground/maximize),
`AlcedoHandoff.swift` (CLI chain), `Engine.swift` (`onExportFinished` + exported URL list),
`Views.swift` (`ExportDestinationSheet`).

## 4. Batch export (x3lumina CLI)

Batch export does not involve Alcedo Studio at all. `x3lumina` provides a CLI that takes a
list of source files and writes rendered outputs with its own wide-gamut pipeline:

```sh
x3lumina-cli export --preset <name> --out <dir> <source1.x3f> <source2.x3i> ...
```

The exact flags are owned by the `x3lumina` project. The only requirement from Alcedo
Studio is that batch export never writes into the source directory's `.x3fproxy.jpg` files
unless it also issues the refresh call from section 3.

## Thin-bridge source map

These are the only Alcedo Studio files that know about the external editor:

| File | Role |
| --- | --- |
| `src/include/app/external_editor_bridge.hpp` | Extension check, proxy path, proxy byte reader; no decoder dependency |
| `src/app/thumbnail_service.cpp` | Culling preview from the proxy JPEG |
| `src/include/ui/alcedo_main/album_backend/workspace_router.hpp`, `src/ui/alcedo_main/album_backend/workspace_router.cpp` | Double-click handoff |
| `src/ui/alcedo_main/album_backend/application_module_host.cpp` | Injects path resolver and process launcher |
| `src/include/ui/alcedo_main/automation/automation_external_commands.hpp`, `src/ui/alcedo_main/automation/commands/external_commands.cpp` | `external.refreshThumbnail` command |

Keeping the bridge in these files means an Alcedo Studio upgrade only needs these small
patches re-applied, not the full Foveon decode stack.

## 5. Upgrading Alcedo Studio

The bridge is a **source-level patch**, not an installed plugin. It survives an upgrade only
if you keep building from your fork; replacing the app with an official pre-built binary
drops the bridge, because that binary has none of these patches.

What survives an upgrade untouched:

- Handoff config — `externalEditor.*` in `com.alcedo-app.Alcedo.plist` (user scope). It is
  tied to the bundle id, not the build; re-run the `defaults write` only if the bundle id
  changes.
- `.x3fproxy.jpg` sidecars — they live next to the photos, outside the app.
- `X3Lumina.app` — a separate app with its own build/release cycle.

What must be redone: re-apply the bridge and rebuild Studio once per upgrade.

```sh
cd /Users/daniellee/WorkBuddy/X3Lumina-Studio

# 1. Fetch the official upstream
git fetch upstream

# 2. Sync your fork's main, then bring the work branch forward
git checkout main
git merge --ff-only upstream/main      # or merge if main has fork-only commits
git push origin main
git checkout feature/x3f-x3i-decoder
git merge main                         # or: git rebase main

# 3. Resolve conflicts only at the bridge touchpoints listed in the source map above
#    (thumbnail_service.cpp, workspace_router.*, application_module_host.cpp,
#     automation_host_session.cpp, CMakeLists.txt). The bridge is ~470 lines across 10 files.

# 4. Rebuild Studio and redeploy your fork's build
```

To make re-application explicit (useful if a release branch is started from clean upstream),
save the bridge as a patch and apply it with `git am`:

```sh
git format-patch -1 1ebb063 -o patches/
git am patches/0001-feat-external-*.patch
```

Post-upgrade smoke check:

```sh
defaults read com.alcedo-app.Alcedo "externalEditor.x3luminaAppPath"
alcedo-cli --json --session gui call session.ping '{}'
```

Then double-click one X3F/X3I to confirm handoff, and export a JPEG to confirm quit-after-export.
If upstream changed the automation command schema, only `AlcedoHandoff.swift` (the CLI call
sequence in §3b) needs adjusting — the source X3F/X3I and the wide-gamut pipeline are unaffected.
