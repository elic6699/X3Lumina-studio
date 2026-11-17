# Upstream feature request: external editor handoff

Draft for `zidage/AlcedoStudio`. Title and body follow `CONTRIBUTING.md` (issue title
type in brackets, no proposed cause or fix inside the issue, no prohibited jargon).

Title:

```
[FEATURE] Send a photo to an external editor by file extension, with a sidecar preview and a preview refresh
```

Body:

---

## What I want to do

I want Alcedo Studio to be the single library for all my photos, and to edit one specific
kind of raw file in a separate application of my own: Sigma X3F/X3I files from Foveon
cameras. All other file types should keep using the built-in editor.

## Why the current application does not let me do this

A double-click on such a photo always opens the built-in editor. The application has no
setting that names another program for a file type, and no way for the Library to show a
preview that the other program wrote.

I currently keep a fork with a small patch set and re-apply it on every upstream sync. The
patch set adds the editor route, the preview source, the program launcher, one automation
command, and the build entry. As a result I cannot use an official build, and every upgrade
costs a merge.

A system file association does not solve it: the double-click is handled inside Alcedo
Studio, so the operating system never sees it.

## Expected result

1. One documented settings location, outside the project file, that maps one or more file
   extensions to an external program with optional arguments. Alcedo Studio appends the
   photo path as the last argument. If the setting is absent, or an extension is not
   listed, all behavior stays as it is now.
2. For a listed extension, a double-click starts that program with the source path, and
   Alcedo Studio does not open the built-in editor or decode the raw data.
3. An optional preview file next to the photo, for example `<photo name>.preview.jpg`. The
   Library shows it while the external program owns the edit. Without the file, the Library
   uses its current preview path.
4. A Library operation that drops the cached previews of given photos, so a new preview
   file written by the external program appears without a restart. The existing
   `library.thumbnails` command asks for a render; I need the invalidation step before it.

## Constraints I care about

- Default off. No change for any user who does not write the setting.
- The source raw file stays read-only for Alcedo Studio.
- No decoder or colour pipeline involvement on the Alcedo Studio side. The external program
  renders and exports with its own pipeline, including its own colour space.
- Any file type can be listed. The behavior is not specific to one camera brand.

## Related

#487 requests the opposite direction: Alcedo Studio started by another program as its
external editor. This request is Alcedo Studio starting another program for a file type.
The two are complementary.

---

## Notes kept out of the issue

`CONTRIBUTING.md` asks that a `[FEATURE]` issue state what, why, and the expected result,
and that a proposed fix go into a pull request instead. The implementation sketch therefore
stays here, not in the issue:

- A registry module reads the settings location once at startup and answers two questions:
  which extensions are delegated, and which preview file to prefer.
- The editor route asks the registry before it opens the built-in editor.
- The thumbnail service asks the registry before it selects its preview source.
- One automation command drops the cached previews of the given element ids. The existing
  `external.refreshThumbnail` command in the local fork is this operation; it can be
  renamed to a neutral name in the upstream version.
- The local fork implements the same behavior in 10 files and about 470 lines, with no
  change to the decoder stack.

A pull request can follow if the maintainers accept the request.
