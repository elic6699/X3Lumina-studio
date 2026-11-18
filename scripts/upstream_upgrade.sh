#!/usr/bin/env bash
#
# Upgrade this fork to a newer Alcedo Studio and confirm that the external editor
# bridge survived the merge.
#
#   scripts/upstream_upgrade.sh check    # default: report what the merge would cost
#   scripts/upstream_upgrade.sh apply    # merge upstream/main into the current branch
#   scripts/upstream_upgrade.sh verify   # confirm every bridge file and call site
#
# Options:
#   --no-fetch      use the upstream ref that is already stored locally
#   --remote <name> upstream remote name (default: upstream)
#   --branch <name> upstream branch name (default: main)
#
# The bridge can only live in the files below, so these files are the only place
# where the merge can touch bridge code. See docs/x3lumina-external-editor.md.

set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"

REMOTE=${UPSTREAM_REMOTE:-upstream}
BRANCH=${UPSTREAM_BRANCH:-main}
MODE=check
DO_FETCH=1
MISSING=0

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
Usage: scripts/upstream_upgrade.sh [check|apply|verify] [options]

  check    Report the commits to merge, the bridge files that upstream changed,
           and the files that would conflict. Changes nothing.
  apply    Merge upstream into the current branch. Refuses a dirty working tree.
           Creates a backup tag first.
  verify   Confirm that every bridge file and call site is still present.

Options:
  --no-fetch      use the upstream ref already stored locally
  --remote <n>    upstream remote name (default: upstream)
  --branch <n>    upstream branch name (default: main)
TEXT
}

info() { printf '%s\n' "$*"; }
warn() { printf 'WARN  %s\n' "$*" >&2; }
fail() { printf 'ERROR %s\n' "$*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    check | apply | verify) MODE=$1 ;;
    --no-fetch) DO_FETCH=0 ;;
    --remote)
      shift
      REMOTE=${1:?--remote needs a value}
      ;;
    --branch)
      shift
      BRANCH=${1:?--branch needs a value}
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

mode_check() {
  fetch_upstream

  base=$(git merge-base HEAD "$UPSTREAM_REF")
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
  if [ "$rc" -eq 0 ]; then
    info "  clean, the merge applies without a conflict"
  else
    info "  conflicts below. Resolve them where they fall, then run apply."
    printf '%s\n' "$trial" | tail -n +2 | as_lines "  "
  fi
  info ""
}

as_lines() {
  prefix=$1
  while IFS= read -r line; do
    [ -n "$line" ] && printf '%s%s\n' "$prefix" "$line"
  done
}

mode_apply() {
  [ -z "$(git status --porcelain)" ] ||
    fail "working tree is not clean. Commit or stash the changes first."

  fetch_upstream

  tag="backup/pre-upgrade-$(date +%Y%m%d-%H%M%S)"
  git tag "$tag"
  info "Backup tag: $tag"

  if git merge --no-ff "$UPSTREAM_REF"; then
    info ""
    info "Merge completed."
    info "Next:"
    info "  1. scripts/upstream_upgrade.sh verify --no-fetch"
    info "  2. cmake --preset macos_debug && cmake --build build/macos-debug"
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
  if [ "$MISSING" -eq 0 ]; then
    info "Bridge is complete. Rebuild, then double-click one X3F to confirm the handoff."
  else
    fail "bridge is incomplete after the upgrade"
  fi
}

case "$MODE" in
  check) mode_check ;;
  apply) mode_apply ;;
  verify) mode_verify ;;
esac
