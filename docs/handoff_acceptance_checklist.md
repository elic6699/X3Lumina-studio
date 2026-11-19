# Export handoff — release acceptance checklist

Manual steps that cannot run in this checkout, kept separate from the automated tests. Run them
on a machine with a real X3F/X3I file, a build of `x3lumina` and a running GUI Alcedo Studio.

The automated coverage already proves, on the CPU with real wide-gamut TIFFs:

- the cache copy is byte-exact and keeps ICC primaries, transfer, peak and digest
  (`tests/image/handoff_gamut_preservation_test.cpp`);
- negative and over-range linear floats are not clipped (same file);
- a cache import decodes to the same `PreparedRawInput` as a normal import (same file);
- a cache-origin asset is tracked and its file removed at shutdown, while a normal import is
  never discarded (`tests/ui/handoff_asset_lifecycle_test.cpp`);
- a forced library-level delete removes the row and pipeline even from a subfolder
  (`tests/ui/album_backend_image_delete_test.cpp`).

The steps below cover what those tests cannot: the GPU render step, the live GUI session, and
a real DNG produced by `x3lumina`.

## Before you start

Set the companion-app path and confirm both processes are the builds you intend to test:

```sh
defaults read com.alcedo-app.Alcedo "externalEditor.x3luminaAppPath"
defaults read com.alcedo-app.Alcedo "externalEditor.x3luminaArgs"   # must contain --external-edit
alcedo-cli --json --session gui call session.ping '{}'
```

Paths used below (macOS, organization and application name are both `Alcedo`):

| Item | Path |
| --- | --- |
| handoff cache | `~/Library/Application Support/Alcedo/Alcedo/handoff-cache` |
| live GUI session file | `~/Library/Application Support/Alcedo/Alcedo/automation/sessions/gui*.json` |

Read `cache_directory` from the live session file rather than relying on the table; Studio
publishes the authoritative path there and older builds omit the field:

```sh
python3 -c 'import json,glob,os; p=glob.glob(os.path.expanduser("~/Library/Application Support/Alcedo/Alcedo/automation/sessions/gui*.json")); print(json.load(open(p[0])).get("cache_directory") if p else "no live GUI session")'
```

If this prints `no live GUI session`, start Studio and wait for `session.ping` to answer first.

## Step 1 — real DNG end to end, including cleanup

Checks the whole path with a file the real `x3lumina` pipeline produced.

1. In Studio, import one `.x3f`/`.x3i` and open it by double-click. Confirm `x3lumina` launches
   with the photo selected (the external bridge, not Studio's own editor).
2. In `x3lumina`, export as **DNG** and choose **回传并自动清理**. Note the exported basename.
3. Confirm all of the following:
   - Studio comes to the foreground and opens the DNG in its **own** editor (Develop panels
     present; no second `x3lumina` window). If Studio was on a subfolder album, this still works.
   - The cache directory now contains one file whose name is the export stem plus a UUID
     suffix. Its bytes equal a copy exported to a normal directory with the same settings
     (`shasum -a 256` on both).
   - The editor shows the wide-gamut image with correct colours. Open the colour/ICC readout
     and confirm the DNG colour metadata is present, not a default sRGB label.
4. Close the editor back to the library. Then check that all three are gone:
   ```sh
   # cache file removed
   ls -la ~/Library/Application Support/Alcedo/Alcedo/handoff-cache
   # library row removed — the exported stem must not resolve any more
   alcedo-cli --json --session gui call search.query "{\"text\":\"<export-stem>\",\"mode\":\"fuzzy\"}"
   ```
   Repeat step 1–4 with a subfolder selected in Studio to confirm the library-level delete
   leaves no row there either.

**Pass**: editor opens in Studio; cache bytes match a normal export; colours and DNG metadata
are intact; on close the cache file and the library row are both gone from root and subfolder.

## Step 2 — rendered export comparison

The decode boundary is already covered automatically. This step checks the GPU render and
image-writer path after import.

1. Export the same 16-bit TIFF twice from `x3lumina` with identical settings. Choose
   **回传 Alcedo 继续编辑** (option 2) for the first, so Studio opens the copy in the ordinary
   export directory. Choose **回传并自动清理** (option 3) for the second. Option 3 also leaves a
   file in the export directory; the copy Studio opens is the byte-identical one in the cache.
2. In Studio, apply the same Develop adjustments and the same export preset to both, then
   export each to a fresh directory.
3. Compare the two exports:
   ```sh
   shasum -a 256 <normal-import>/export.tif <cache-import>/export.tif
   ```
   The files must be byte-identical. If a writer embeds a timestamp, compare pixels instead
   with an image tool that reports exact sample values; the samples must match.

**Pass**: normal-import and cache-import renders are byte-identical (or pixel-identical where
the container embeds volatile metadata).

## Step 3 — cache capacity behaviour

There is no hard size or file-count limit. This step verifies the cache cannot accumulate
across sessions.

1. With Studio running, repeat step 1 twice more so several temporary assets exist at once.
   Confirm the cache holds one file per open/imported handoff.
2. Close each editor and confirm files disappear as each asset is discarded.
3. Quit Studio while a handoff asset is still open, then check the cache: shutdown removes any
   remaining tracked file.
4. As a final check, place a dummy file in the cache, start Studio, and confirm
   `PrepareHandoffCacheDirectory` cleared it at startup.

**Pass**: files are removed on editor close and on shutdown; stale files are cleared at the next
Studio start; no cache file survives into a fresh session.

## Failure handling to verify

These are best-effort by design. Confirm the observable behaviour rather than expecting a retry:

- Point `externalEditor/x3luminaArgs` at a missing `alcedo-cli` (or stop Studio). Start a
  handoff: `x3lumina` stays open, shows the error with the output path, and the exported file
  remains in the chosen directory. The source `.x3f`/`.x3i` is never modified.
- For option 3 specifically, a failed handoff removes the temporary cache copy (no half-written
  asset) while the normal export directory file stays intact.

## Where to report a failure

If a step fails, record the exact command and output, the Studio version, the `x3lumina`
version, and the session file's `cache_directory` value. The behaviour to fix is the one this
checklist names; the automated tests in the sections above stay as the regression baseline.
