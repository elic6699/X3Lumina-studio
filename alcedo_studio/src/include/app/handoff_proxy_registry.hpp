//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <algorithm>
#include <filesystem>
#include <mutex>
#include <optional>
#include <vector>

#include "type/type.hpp"

namespace alcedo {

/// A temporary redirect of one library image's *decoding source* to a handoff file produced by
/// the external editor (a DNG / 16-bit TIFF in the Studio-managed handoff cache).
///
/// The library image itself is untouched: its element id, file name, thumbnail and stored
/// `image_path_` (the original X3F/X3I) stay as they are. Only the two byte-reading entry points
/// that feed the editor preview and the full-resolution export consult this registry. It is a
/// pure path redirect: no decoding, quantisation, gamut mapping or clipping happens here, so the
/// wide-gamut bytes and the ICC carried by the handoff file are preserved exactly.
///
/// One process-wide instance is shared by the automation command (which registers/removes), the
/// editor input loader and the export service. All access is mutex-protected because preview and
/// export run on worker threads while registration/removal runs on the GUI thread.
class HandoffProxyRegistry {
 public:
  struct Proxy {
    sl_element_id_t       element_id_  = 0;
    image_id_t            image_id_    = 0;
    std::filesystem::path source_path_;  // original X3F/X3I, kept for diagnostics/restore
    std::filesystem::path cache_path_;   // temporary DNG/TIFF to decode from
  };

  /// The shared process-wide registry.
  [[nodiscard]] static auto Instance() -> HandoffProxyRegistry& {
    static HandoffProxyRegistry instance;
    return instance;
  }

  /// Registers (or replaces) the proxy for @p image_id. Entries without an image id or with an
  /// empty cache path are ignored. Re-registering an image replaces its previous proxy.
  void Track(Proxy proxy) {
    if (proxy.image_id_ == 0 || proxy.cache_path_.empty()) {
      return;
    }
    std::lock_guard lock(mutex_);
    std::erase_if(proxies_, [&proxy](const Proxy& entry) {
      return entry.image_id_ == proxy.image_id_;
    });
    proxies_.push_back(std::move(proxy));
  }

  /// Returns the proxy cache path for @p image_id, if one is active. Read-only: the entry stays
  /// registered until TakeForImage / TakeAll / the editor-close handler removes it.
  [[nodiscard]] auto ProxyPathFor(image_id_t image_id) const
      -> std::optional<std::filesystem::path> {
    std::lock_guard lock(mutex_);
    const auto found = std::find_if(proxies_.begin(), proxies_.end(),
                                    [image_id](const Proxy& entry) {
                                      return entry.image_id_ == image_id;
                                    });
    if (found == proxies_.end()) {
      return std::nullopt;
    }
    return found->cache_path_;
  }

  /// Returns and removes the proxy bound to @p image_id, if any.
  [[nodiscard]] auto TakeForImage(image_id_t image_id) -> std::optional<Proxy> {
    std::lock_guard lock(mutex_);
    const auto found = std::find_if(proxies_.begin(), proxies_.end(),
                                    [image_id](const Proxy& entry) {
                                      return entry.image_id_ == image_id;
                                    });
    if (found == proxies_.end()) {
      return std::nullopt;
    }
    Proxy result = std::move(*found);
    proxies_.erase(found);
    return result;
  }

  /// Returns and removes every remaining proxy. Used on shutdown as a safety net.
  [[nodiscard]] auto TakeAll() -> std::vector<Proxy> {
    std::lock_guard lock(mutex_);
    std::vector<Proxy> result;
    result.swap(proxies_);
    return result;
  }

  [[nodiscard]] auto Contains(image_id_t image_id) const -> bool {
    std::lock_guard lock(mutex_);
    return std::any_of(proxies_.begin(), proxies_.end(), [image_id](const Proxy& entry) {
      return entry.image_id_ == image_id;
    });
  }

  [[nodiscard]] auto Count() const -> std::size_t {
    std::lock_guard lock(mutex_);
    return proxies_.size();
  }

  /// Drops all proxies. Intended for tests so each case starts from a clean registry.
  void Reset() { (void)TakeAll(); }

 private:
  HandoffProxyRegistry() = default;
  mutable std::mutex     mutex_;
  std::vector<Proxy>     proxies_;
};

}  // namespace alcedo
