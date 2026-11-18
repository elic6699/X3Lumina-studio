#!/usr/bin/env bash
#
# Upgrade this fork to a newer Alcedo Studio and confirm that the external editor
# bridge survived the merge.
#
#   scripts/upstream_upgrade.sh sync     # fork sync check, merge, verify (one command)
#   scripts/upstream_upgrade.sh check    # default: report what the merge would cost
#   scripts/upstream_upgrade.sh apply    # merge upstream/main into the current branch
#   scripts/upstream_upgrade.sh verify   # confirm every bridge file and call site
#   scripts/upstream_upgrade.sh build    # configure and build with the Metal dir set
#
# Options:
#   --no-fetch          use the ref that is already stored locally
#   --remote <name>     upstream remote name (default: upstream)
#   --branch <name>     upstream branch name (default: main)
#   --preset <name>     CMake preset used by build (default: macos_release)
#   --build             sync also builds once the merge succeeds
#
# sync assumes the fork was synced in the web UI first. It compares
# origin/<branch> with upstream/<branch> by SHA, so the large upstream history is
# never downloaded; the merge then only fetches the fork branch.
#
# The bridge can only live in the files below, so these files are the only place
# where the merge can touch bridge code. See docs/x3lumina-external-editor.md.

set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"

REMOTE=${UPSTREAM_REMOTE:-upstream}
BRANCH=${UPSTREAM_BRANCH:-main}
PRESET=${BUILD_PRESET:-macos_release}
MODE=check
DO_FETCH=1
DO_BUILD=0
MISSING=0
TRIAL_OK=0

BRIDGE_FILES="
alcedo_studio/src/include/app/external_editor_bridge.hpp
alcedo_studio/src/app/thumbnail_service.cpp
alcedo_studio/src/include/ui/alcedo_main/album_backend/workspace_router.hpp
alcedo_studio/src/ui/alcedo_main/album_backend/workspace_router.cpp
alcedo_studio/src/ui/alcedo_main/album_backend/application_module_host.cpp
alcedo_studio/src/include/ui/alcedo_main/automation/automation_external_commands.hpp
alcedo_studio/src/ui/alcedo_main/automation/commands/external_commands.cpp
alcedo_studio/src/ui/alcedo_main/automation/automation_host_session.cpp
alcedo_studio/src/ui/alcedo_main/CMakeLists.txt
"

usage() {
  cat <<'TEXT'
Usage: scripts/upstream_upgrade.sh [sync|check|apply|verify|build] [options]

  sync     Compare the fork with upstream, merge the fork branch, then verify.
           Stops when the fork was not synced or the merge would conflict.
  check    Report the commits to merge, the bridge files that upstream changed,
           and the files that would conflict. Changes nothing.
  apply    Merge upstream into the current branch. Refuses a dirty working tree.
           Creates a backup tag first.
  verify   Confirm that every bridge file and call site is still present.
  build    Configure and build the preset. Sets DEVELOPER_DIR when the Metal
           toolchain is not on PATH.

Options:
  --no-fetch      use the ref already stored locally
  --remote <n>    upstream remote name (default: upstream)
  --branch <n>    upstream branch name (default: main)
  --preset <n>    CMake preset used by build (default: macos_release)
  --build         sync also builds once the merge succeeds
TEXT
}

info() { printf '%s\n' "$*"; }
warn() { printf 'WARN  %s\n' "$*" >&2; }
fail() { printf 'ERROR %s\n' "$*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    sync | check | apply | verify | build) MODE=$1 ;;
    --no-fetch) DO_FETCH=0 ;;
    --build) DO_BUILD=1 ;;
    --remote)
      shift
      REMOTE=${1:?--remote needs a value}
      ;;
    --branch)
      shift
      BRANCH=${1:?--branch needs a value}
      ;;
    --preset)
      shift
      PRESET=${1:?--preset needs a value}
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      fail "unknown argument: $1"
      ;;
  esac
  shift
done

UPSTREAM_REF="$REMOTE/$BRANCH"

require_upstream() {
  git rev-parse --verify --quiet "$UPSTREAM_REF" >/dev/null ||
    fail "missing $UPSTREAM_REF. Add the remote first: git remote add $REMOTE <url>"
}

fetch_upstream() {
  if [ "$DO_FETCH" -eq 1 ]; then
    info "Fetching $REMOTE. This repository is large, so it can take minutes."
    git fetch "$REMOTE" --prune
  fi
  require_upstream
}

require_clean_tree() {
  # Untracked files never block a merge (a stray probe file in the tree is fine).
  [ -z "$(git status --porcelain --untracked-files=no)" ] ||
    fail "working tree is not clean. Commit or stash the changes first."
}

tag_and_merge() {
  tag="backup/pre-upgrade-$(date +%Y%m%d-%H%M%S)"
  git tag "$tag"
  info "Backup tag: $tag"

  if git merge --no-ff "$UPSTREAM_REF"; then
    info ""
    info "Merge completed."
  else
    info ""
    warn "Merge stopped with conflicts:"
    git diff --name-only --diff-filter=U | as_lines "  "
    info ""
    info "Resolve them, then: git add <files> && git merge --continue"
    info "Undo the whole merge: git merge --abort"
    info "Undo and return to the backup tag: git reset --hard $tag"
    exit 1
  fi
}

as_lines() {
  prefix=$1
  while IFS= read -r line; do
    [ -n "$line" ] && printf '%s%s\n' "$prefix" "$line"
  done
}

report_state() {
  base=$(git merge-base HEAD "$UPSTREAM_REF" 2>/dev/null || true)
  [ -n "$base" ] || fail "no common ancestor with $UPSTREAM_REF. Merge by hand."
  behind=$(git rev-list --count "HEAD..$UPSTREAM_REF")
  ahead=$(git rev-list --count "$UPSTREAM_REF..HEAD")

  info ""
  info "Branch:           $(git branch --show-current)"
  info "Upstream:         $UPSTREAM_REF at $(git rev-parse --short "$UPSTREAM_REF")"
  info "Commits to merge: $behind    local commits: $ahead"
  info ""

  info "Upstream changes inside bridge files since the fork point:"
  touched=$(git log --name-only --pretty=format: "$base..$UPSTREAM_REF" -- $BRIDGE_FILES |
    grep -v '^$' | sort -u || true)
  if [ -z "$touched" ]; then
    info "  none"
  else
    printf '%s\n' "$touched" | while IFS= read -r line; do info "  $line"; done
  fi

  info ""
  info "Trial merge (nothing is written to the working tree):"
  set +e
  trial=$(git merge-tree --write-tree --name-only HEAD "$UPSTREAM_REF" 2>&1)
  rc=$?
  set -e
  TRIAL_OK=0
  if [ "$rc" -eq 0 ]; then
    TRIAL_OK=1
    info "  clean, the merge applies without a conflict"
  else
    info "  conflicts below. Resolve them where they fall, then run apply."
    printf '%s\n' "$trial" | tail -n +2 | as_lines "  "
  fi
  info ""
}

mode_check() {
  fetch_upstream
  report_state
  info "Next:"
  info "  scripts/upstream_upgrade.sh apply    # merge it"
  info "  scripts/upstream_upgrade.sh verify   # after the merge"
}

mode_apply() {
  require_clean_tree
  fetch_upstream
  tag_and_merge
  info "Next:"
  info "  1. scripts/upstream_upgrade.sh verify --no-fetch"
  info "  2. scripts/upstream_upgrade.sh build"
}

remote_head() {
  git ls-remote --refs "$1" "refs/heads/$BRANCH" 2>/dev/null | cut -f1 | head -n 1
}

mode_sync() {
  fork_head=$(remote_head origin || true)
  upstream_head=$(remote_head upstream || true)

  [ -n "$fork_head" ] || fail "cannot read origin/$BRANCH. Check the remote and the network."
  [ -n "$upstream_head" ] || fail "cannot read upstream/$BRANCH. Check the remote and the network."

  info "fork $BRANCH:     $(printf '%.7s' "$fork_head")"
  info "upstream $BRANCH: $(printf '%.7s' "$upstream_head")"

  if [ "$fork_head" != "$upstream_head" ]; then
    fail "the fork does not carry upstream yet. Sync it in the web UI (Fork > Sync fork), then run sync again.
       Merging now would silently merge nothing."
  fi

  # The fork already carries upstream's head, so fetching it is small.
  info ""
  info "Fetching origin/$BRANCH (the fork already matches upstream)."
  git fetch origin "$BRANCH"
  UPSTREAM_REF="origin/$BRANCH"
  require_upstream

  report_state

  if [ "$TRIAL_OK" -ne 1 ]; then
    fail "the merge conflicts. Run apply, resolve the files above, then verify."
  fi

  require_clean_tree
  tag_and_merge
  mode_verify

  info "Next:"
  info "  scripts/upstream_upgrade.sh build    # or build/run-dev.sh for the run you use daily"
  if [ "$DO_BUILD" -eq 1 ]; then
    info ""
    mode_build
  fi
}

check_symbol() {
  file=$1
  needle=$2
  label=$3
  if grep -qF -- "$needle" "$file" 2>/dev/null; then
    info "  OK      $label"
  else
    warn "MISSING $label  ($file)"
    MISSING=1
  fi
}

mode_verify() {
  MISSING=0

  info "Bridge files:"
  for f in $BRIDGE_FILES; do
    if [ -f "$f" ]; then
      info "  OK      $f"
    else
      warn "MISSING $f"
      MISSING=1
    fi
  done

  info ""
  info "Bridge call sites:"
  check_symbol alcedo_studio/src/ui/alcedo_main/album_backend/workspace_router.cpp \
    "IsExternallyManaged" "editor route asks the bridge"
  check_symbol alcedo_studio/src/app/thumbnail_service.cpp \
    "TryReadProxyJpeg" "thumbnail source reads the sidecar preview"
  check_symbol alcedo_studio/src/ui/alcedo_main/automation/automation_host_session.cpp \
    "RegisterAutomationExternalCommands" "automation host registers the refresh command"
  check_symbol alcedo_studio/src/ui/alcedo_main/automation/commands/external_commands.cpp \
    "external.refreshThumbnail" "refresh command name"
  check_symbol alcedo_studio/src/ui/alcedo_main/CMakeLists.txt \
    "external_commands.cpp" "build entry"

  info ""
  info "X3F/X3I import layer (fork commits, not part of the bridge patch):"
  for f in \
    alcedo_studio/src/decoders/x3f/x3f_support.cpp \
    alcedo_studio/src/include/decoders/x3f/x3f_support.hpp \
    alcedo_studio/src/io/image/embedded_preview_reader.cpp; do
    if [ -f "$f" ]; then
      info "  OK      $f"
    else
      warn "MISSING $f"
      MISSING=1
    fi
  done
  check_symbol alcedo_studio/src/image/metadata_extractor.cpp \
    "ReadEmbeddedJpeg" "import metadata reads the embedded JPEG"
  check_symbol alcedo_studio/src/io/image/image_loader.cpp \
    "IsX3fFile" "image loader routes X3F/X3I sources"
  check_symbol alcedo_studio/src/decoders/x3f/CMakeLists.txt \
    "alcedo_x3f_import" "import-only decode target"
  check_symbol alcedo_studio/src/decoders/CMakeLists.txt \
    "alcedo_x3f_import" "ImageLoader links the import target"

  info ""
  if [ "$MISSING" -eq 0 ]; then
    info "Bridge and import layer are complete. Rebuild, then double-click one X3F to confirm the handoff."
  else
    fail "the bridge or the import layer is incomplete after the upgrade"
  fi
}

mode_build() {
  xcode_dir=/Applications/Xcode.app/Contents/Developer
  if ! xcrun -f metal >/dev/null 2>&1; then
    info "metal is not on PATH (xcode-select may point at the Command Line Tools)."
    info "Using $xcode_dir for this build."
    export DEVELOPER_DIR="$xcode_dir"
  fi

  info "Configuring preset $PRESET"
  cmake --preset "$PRESET"
  info ""
  info "Building preset $PRESET"
  cmake --build --preset "$PRESET"
  info ""
  info "Build finished."
}

case "$MODE" in
  sync) mode_sync ;;
  check) mode_check ;;
  apply) mode_apply ;;
  verify) mode_verify ;;
  build) mode_build ;;
esac
