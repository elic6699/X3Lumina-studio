#!/bin/sh
# Normalize installed macOS Mach-O load commands so the app never resolves
# third-party dependencies from Homebrew or another developer-machine path.
# macdeployqt copies the files but can leave LC_RPATH values such as
# /opt/homebrew/Cellar/opencv/... and /Users/.../.local/Qt/... behind. Those
# rpaths win over @rpath references and can load a second OpenMP/Qt runtime,
# which aborts during dyld/Qt initialization.
#
# Usage: normalize_macos_dylib_paths.sh <path-to-bundle.app>

set -eu

app_dir="${1:?usage: normalize_macos_dylib_paths.sh <bundle.app>}"
frameworks_dir="${app_dir}/Contents/Frameworks"
bundle_name="$(basename "${app_dir}" .app)"
main_bin="${app_dir}/Contents/MacOS/${bundle_name}"

if [ ! -d "${frameworks_dir}" ]; then
  echo "macOS bundle Frameworks directory not found: ${frameworks_dir}" >&2
  exit 1
fi

# Rewrites every dependency that has a bundled sibling to a bundle-relative
# path, and removes every absolute LC_RPATH (only the app's own @rpath entries
# and system paths may remain). $1 = Mach-O file, $2 = prefix for bundled deps.
normalize_macho() {
  macho="$1"
  prefix="$2"
  chmod u+w "${macho}"

  install_id="$(otool -D "${macho}" 2>/dev/null | sed -n '2p')"
  skip_lines=1
  if [ -n "${install_id}" ]; then
    skip_lines=2
  fi

  otool -L "${macho}" 2>/dev/null | awk -v skip="${skip_lines}" 'NR > skip {print $1}' |
    while IFS= read -r dep; do
      case "${dep}" in
        @rpath/*)
          rel="${dep#@rpath/}"
          ;;
        @executable_path/../Frameworks/*)
          rel="${dep#@executable_path/../Frameworks/}"
          ;;
        @loader_path/*)
          rel="${dep#@loader_path/}"
          ;;
        /*)
          case "${dep}" in
            /System/Library/*|/usr/lib/*)
              continue
              ;;
          esac
          rel="$(basename "${dep}")"
          ;;
        *)
          continue
          ;;
      esac

      if [ -e "${frameworks_dir}/${rel}" ]; then
        target="${prefix}/${rel}"
        if [ "${dep}" != "${target}" ]; then
          install_name_tool -change "${dep}" "${target}" "${macho}"
        fi
      fi
    done

  if [ -n "${install_id}" ]; then
    case "${install_id}" in
      @loader_path/*|@rpath/*)
        ;;
      *)
        install_name_tool -id "@loader_path/$(basename "${macho}")" "${macho}"
        ;;
    esac
  fi

  # An installed bundle must not carry absolute search paths: no developer
  # Qt, Homebrew, or /usr/local copy may participate in resolution.
  while :; do
    bad_rpath="$(otool -l "${macho}" 2>/dev/null | awk '$1 == "path" {print $2}' |
      grep '^/' | grep -v -E '^(/usr/lib|/System/Library)(/|$)' | head -n 1 || true)"
    [ -n "${bad_rpath}" ] || break
    install_name_tool -delete_rpath "${bad_rpath}" "${macho}"
  done
}

find "${frameworks_dir}" -maxdepth 1 -type f -name '*.dylib' -print | while IFS= read -r dylib; do
  normalize_macho "${dylib}" "@loader_path"
done

if [ -f "${main_bin}" ]; then
  normalize_macho "${main_bin}" "@executable_path/../Frameworks"
fi

echo "normalized bundle load commands in ${app_dir}"
