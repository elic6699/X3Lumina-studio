//  Copyright 2026 Yurun Zi
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "ui/alcedo_main/editor_support/controllers/image_controller.hpp"

#include <stdexcept>

#include "app/handoff_proxy_registry.hpp"
#include "image/image.hpp"
#include "io/image/image_loader.hpp"

namespace alcedo::ui::controllers {

auto LoadImageInputBuffer(const std::shared_ptr<ImagePoolService>& image_pool, image_id_t image_id)
    -> std::shared_ptr<ImageBuffer> {
  if (!image_pool) {
    throw std::runtime_error("Image controller: image pool service is null");
  }

  auto img_desc = image_pool->Read<std::shared_ptr<Image>>(
      image_id, [](const std::shared_ptr<Image>& img) { return img; });

  // A handoff proxy temporarily redirects the *decoding source* to the external editor's
  // DNG/TIFF while the library image (its element id, name, history and stored path) stays
  // untouched. This is a pure byte-source redirect: no decode/quantise/gamut work happens here,
  // so the wide-gamut bytes and ICC of the handoff file reach the editor exactly as exported.
  std::shared_ptr<std::vector<uint8_t>> bytes;
  const auto proxy = HandoffProxyRegistry::Instance().ProxyPathFor(image_id);
  if (proxy.has_value()) {
    try {
      bytes = std::make_shared<std::vector<uint8_t>>(
          ByteBufferLoader::LoadByteBufferFromPath(*proxy));
    } catch (...) {
      bytes = nullptr;
    }
  } else {
    bytes = ByteBufferLoader::LoadFromImage(img_desc);
  }
  if (!bytes) {
    throw std::runtime_error("Image controller: failed to load image bytes");
  }

  return std::make_shared<ImageBuffer>(std::move(*bytes));
}

}  // namespace alcedo::ui::controllers
