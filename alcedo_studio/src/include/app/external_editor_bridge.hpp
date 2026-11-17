//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string_view>
#include <system_error>
#include <vector>

namespace alcedo::external_editor {

/// Suffix appended to a source path for the sidecar JPEG that the external editor writes.
inline constexpr std::string_view kProxySuffix = ".x3fproxy.jpg";

/// QSettings keys for the external editor command. NativeFormat, UserScope.
inline constexpr const char* kAppPathKey = "externalEditor/x3luminaAppPath";
inline constexpr const char* kArgsKey    = "externalEditor/x3luminaArgs";

/// True for a source whose editing is owned by the external app. Extension only: no file
/// access and no decoder dependency, so every thin bridge can call this.
[[nodiscard]] inline auto IsExternallyManaged(const std::filesystem::path& source) -> bool {
  const auto ext = source.extension().wstring();
  const auto equal_ignore_case = [&ext](std::wstring_view candidate) {
    return ext.size() == candidate.size() &&
           std::equal(ext.begin(), ext.end(), candidate.begin(), [](wchar_t a, wchar_t b) {
             return a == b || (a >= L'A' && a <= L'Z' && a - L'A' + L'a' == b);
           });
  };
  return equal_ignore_case(L".x3f") || equal_ignore_case(L".x3i");
}

/// Path of the sidecar JPEG: "<source>.x3fproxy.jpg", in the same directory as the source.
[[nodiscard]] inline auto ProxyJpegPath(const std::filesystem::path& source)
    -> std::filesystem::path {
  std::wstring name = source.wstring();
  name.append(kProxySuffix.begin(), kProxySuffix.end());
  return std::filesystem::path(name);
}

/// Reads the sidecar JPEG bytes. Returns false when it is absent, unreadable, empty, or
/// implausibly large. The caller keeps its existing source when this returns false.
[[nodiscard]] inline auto TryReadProxyJpeg(const std::filesystem::path& source,
                                           std::vector<std::uint8_t>* bytes) -> bool {
  if (bytes == nullptr) {
    return false;
  }
  std::error_code ec;
  const auto path = ProxyJpegPath(source);
  if (!std::filesystem::is_regular_file(path, ec) || ec) {
    return false;
  }
  const auto size = std::filesystem::file_size(path, ec);
  if (ec || size == 0 || size > (64u << 20)) {
    return false;
  }
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return false;
  }
  bytes->resize(static_cast<std::size_t>(size));
  in.read(reinterpret_cast<char*>(bytes->data()), static_cast<std::streamsize>(size));
  if (!in || static_cast<std::size_t>(in.gcount()) != bytes->size()) {
    bytes->clear();
    return false;
  }
  return true;
}

}  // namespace alcedo::external_editor
