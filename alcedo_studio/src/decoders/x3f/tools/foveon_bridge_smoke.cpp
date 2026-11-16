//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//
// Standalone smoke for the C++ path-B (Foveon native chain) bridge.
//   foveon_bridge_smoke <file.x3f|.x3i> [preview_long_edge]
//
// Verifies:
//   * capability probing and product defaults (three switches ON),
//   * prepare/apply caching: an apply-level slider changes the image but NOT
//     the decode key (no re-decode),
//   * a decode-level switch (mode8) invalidates the key and changes the image,
//   * path A stays a separate, unclamped scene-linear wide-gamut source.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "decoders/x3f/x3f_foveon.hpp"
#include "decoders/x3f/x3f_support.hpp"

namespace {

using alcedo::x3f::FoveonImage;

auto ImageHash(const FoveonImage& image) -> std::uint64_t {
  if (image.Empty()) {
    return 0;
  }
  std::uint64_t h = 1469598103934665603ULL;
  const auto*   p = image.rgb.get();
  const auto    n = static_cast<std::size_t>(image.width) * image.height * 3U;
  for (std::size_t i = 0; i < n; ++i) {
    h ^= p[i];
    h *= 1099511628211ULL;
  }
  return h;
}

auto ReadBytes(const std::filesystem::path& path) -> std::vector<std::byte> {
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in) {
    return {};
  }
  const auto size = in.tellg();
  if (size <= 0) {
    return {};
  }
  std::vector<std::byte> bytes(static_cast<std::size_t>(size));
  in.seekg(0);
  in.read(reinterpret_cast<char*>(bytes.data()), size);
  return bytes;
}

}  // namespace

auto main(int argc, char** argv) -> int {
  if (argc < 2) {
    std::cerr << "usage: foveon_bridge_smoke <file.x3f|.x3i> [preview_long_edge]\n";
    return 2;
  }
  const std::filesystem::path path(argv[1]);
  const int long_edge = argc > 2 ? std::max(64, std::atoi(argv[2])) : 960;

  const auto caps = alcedo::x3f::FoveonProbe(path);
  if (!caps) {
    std::cerr << "FAIL: FoveonProbe rejected " << path << "\n";
    return 1;
  }
  std::cout << "probe: class=" << caps->model_class
            << " gen=" << caps->model_gen
            << " sdq_hl=" << caps->sdq_hl_capable
            << " sdq_eff=" << caps->sdq_hl_effective
            << " decoders=0x" << std::hex << caps->available_decoders << std::dec
            << " spp=" << caps->spp_capable
            << " offers_choice=" << caps->OffersDecoderChoice()
            << " container=" << caps->is_container
            << " frames=" << caps->frame_count
            << " jpeg=" << caps->has_jpeg
            << " " << caps->width << "x" << caps->height << "\n";
  // Only Merrill may offer the fuse / SPP backend choice.
  if (caps->OffersDecoderChoice() &&
      caps->model_class != alcedo::x3f::kFoveonModelMerrill) {
    std::cerr << "FAIL: a non-Merrill body offers a decode-backend choice\n";
    return 1;
  }

  alcedo::x3f::FoveonEditParams params;
  std::cout << "defaults: oe=" << params.oe
            << " hole_fix=" << params.hole_fix
            << " merrill_hl_neut=" << params.merrill_hl_neut
            << " hl_keep=" << params.hl_keep
            << " detail=" << params.detail
            << " color_mode=" << params.color_mode
            << " color_strength=" << params.color_strength << "\n";
  if (params.oe != 1 || params.hole_fix != 1 || params.merrill_hl_neut != 1) {
    std::cerr << "FAIL: the three default-on switches are not ON\n";
    return 1;
  }

  const int down = alcedo::x3f::FoveonPreviewDown(*caps, long_edge);
  params.down = down;
  std::cout << "preview down=" << down
            << " resolved_oe=" << alcedo::x3f::FoveonResolveOe(params, caps->model_gen)
            << "\n";

  alcedo::x3f::FoveonRenderer renderer;
  const auto key0 = alcedo::x3f::FoveonDecodeKey(params);
  const auto image0 = renderer.Render(path, params);
  if (!image0 || image0->Empty()) {
    std::cerr << "FAIL: Render #1: " << renderer.LastError() << " / "
              << alcedo::x3f::FoveonLastError() << "\n";
    return 1;
  }
  const auto hash0 = ImageHash(*image0);
  std::cout << "render #1: " << image0->width << "x" << image0->height
            << " orientation=" << image0->orientation
            << " hash=0x" << std::hex << hash0 << std::dec << "\n";

  // Apply-level: exposure must not invalidate the prepared linear frame.
  params.exposure += 0.5;
  const auto key1 = alcedo::x3f::FoveonDecodeKey(params);
  const auto image1 = renderer.Render(path, params);
  if (key1 != key0) {
    std::cerr << "FAIL: exposure changed the decode key (unexpected re-decode)\n";
    return 1;
  }
  if (!image1 || ImageHash(*image1) == hash0) {
    std::cerr << "FAIL: exposure did not visibly change the preview\n";
    return 1;
  }
  std::cout << "render #2 (exposure +0.5): hash=0x" << std::hex
            << ImageHash(*image1) << std::dec << " (decode key unchanged)\n";

  // Decode-level: switching mode8 off must invalidate the prepared frame.
  params.oe = 0;
  const auto key2 = alcedo::x3f::FoveonDecodeKey(params);
  if (key2 == key0) {
    std::cerr << "FAIL: mode8 switch did not invalidate the decode key\n";
    return 1;
  }
  const auto image2 = renderer.Render(path, params);
  if (!image2) {
    std::cerr << "FAIL: Render #3 after re-prepare: " << renderer.LastError() << "\n";
    return 1;
  }
  std::cout << "render #3 (mode8 off, re-decoded): hash=0x" << std::hex
            << ImageHash(*image2) << std::dec << "\n";

  // "SDQ 高光" must actually reach the C chain. It used to be declared in the
  // edit struct but never forwarded, so the slider was inert. Apply-level:
  // changing it must NOT invalidate the prepared frame.
  if (caps->sdq_hl_capable) {
    params.oe      = 1;
    params.sdq_hl  = 2.0;
    const auto key3 = alcedo::x3f::FoveonDecodeKey(params);
    const auto image3 = renderer.Render(path, params);
    if (key3 != key0) {
      std::cerr << "FAIL: sdq_hl changed the decode key (it is apply-level)\n";
      return 1;
    }
    if (!image3) {
      std::cerr << "FAIL: sdq_hl render failed: " << renderer.LastError() << "\n";
      return 1;
    }
    if (caps->sdq_hl_effective && ImageHash(*image3) == ImageHash(*image2)) {
      std::cerr << "FAIL: sdq_hl=2 did not change the preview (slider is inert)\n";
      return 1;
    }
    std::cout << "render #4 (sdq_hl=2): hash=0x" << std::hex
              << ImageHash(*image3) << std::dec << " (decode key unchanged"
              << (caps->sdq_hl_effective ? "" : ", body has no DigitalISOGain")
              << ")\n";
    params.sdq_hl = 0.0;
  }

  // One-shot entry point (export / batch path).
  alcedo::x3f::FoveonEditParams once = alcedo::x3f::FoveonEditParams{};
  once.down = down;
  const auto once_image = alcedo::x3f::RenderFoveonOnce(path, once);
  if (!once_image || once_image->Empty()) {
    std::cerr << "FAIL: RenderFoveonOnce: " << alcedo::x3f::FoveonLastError() << "\n";
    return 1;
  }
  std::cout << "one-shot: " << once_image->width << "x" << once_image->height
            << " hash=0x" << std::hex << ImageHash(*once_image) << std::dec << "\n";

  // Merrill 原链解码: SPP native chain straight-out (16-bit sRGB finished
  // image). Quattro / classic must reject it.
  if (caps->OffersDecoderChoice()) {
    const auto spp = alcedo::x3f::RenderSppOnce(path);
    if (!spp || spp->Empty()) {
      std::cerr << "FAIL: SPP native-chain straight-out failed\n";
      return 1;
    }
    std::cout << "spp straight-out: " << spp->width << "x" << spp->height
              << " orientation=" << spp->orientation
              << " hash=0x" << std::hex << ImageHash(*spp) << std::dec << "\n";

    // SPP six-parameter tone bridge: each of the five look controls must
    // change the finished image, and the luma-NR control must re-prepare.
    alcedo::x3f::FoveonRenderer spp_renderer;
    alcedo::x3f::FoveonEditParams spp_params;
    const auto spp_base = spp_renderer.RenderSpp(path, spp_params);
    if (!spp_base || spp_base->Empty()) {
      std::cerr << "FAIL: RenderSpp base: " << spp_renderer.LastError() << "\n";
      return 1;
    }
    const auto spp_base_hash = ImageHash(*spp_base);
    std::cout << "spp tone base: " << spp_base->width << "x" << spp_base->height
              << " hash=0x" << std::hex << spp_base_hash << std::dec << "\n";

    const auto check_tone = [&](const char* name, auto setter) -> bool {
      alcedo::x3f::FoveonEditParams p;
      setter(p);
      const auto image = spp_renderer.RenderSpp(path, p);
      if (!image || ImageHash(*image) == spp_base_hash) {
        std::cerr << "FAIL: SPP tone '" << name
                  << "' did not visibly change the finished image\n";
        return false;
      }
      std::cout << "spp tone " << name << ": hash=0x" << std::hex
                << ImageHash(*image) << std::dec << "\n";
      return true;
    };
    bool tone_ok = true;
    tone_ok &= check_tone("exposure", [](auto& p) { p.exposure = 0.5; });
    tone_ok &= check_tone("contrast", [](auto& p) { p.contrast = 0.5; });
    tone_ok &= check_tone("shadow", [](auto& p) { p.shadow = 0.5; });
    tone_ok &= check_tone("highlight", [](auto& p) { p.highlight = 0.5; });
    tone_ok &= check_tone("saturation", [](auto& p) { p.saturation = 0.5; });

    // nr_luma is prepare-level: this must force a new SPP prepare and still
    // produce a different finished image.
    alcedo::x3f::FoveonEditParams nr_params;
    nr_params.nr_luma = 0.75;
    const auto nr_image = spp_renderer.RenderSpp(path, nr_params);
    if (!nr_image || ImageHash(*nr_image) == spp_base_hash) {
      std::cerr << "FAIL: SPP nr_luma re-prepare did not change the image\n";
      tone_ok = false;
    } else {
      std::cout << "spp tone nr_luma: hash=0x" << std::hex
                << ImageHash(*nr_image) << std::dec << "\n";
    }
    if (!tone_ok) {
      return 1;
    }
  } else {
    const auto spp = alcedo::x3f::RenderSppOnce(path);
    if (spp) {
      std::cerr << "FAIL: SPP chain accepted a non-Merrill / incapable file\n";
      return 1;
    }
    std::cout << "spp straight-out: correctly unavailable\n";
  }

  // Path A regression: the scene-linear wide-gamut decoder is a separate
  // entry point and still preserves negative channels / >1 headroom.
  if (!caps->is_container) {
    const auto bytes = ReadBytes(path);
    int        decoder = 0;
    if (const char* want = std::getenv("X3F_SMOKE_DECODER")) {
      decoder = std::string(want) == "own" ? 2 : (std::string(want) == "fuse" ? 1 : 0);
    }
    const auto linear = alcedo::x3f::DecodeEncoded(bytes, decoder);
    if (!linear || !linear->rgb) {
      std::cerr << "FAIL: path A scene-linear decode broke\n";
      return 1;
    }
    const auto n = static_cast<std::size_t>(linear->width) * linear->height * 3U;
    float lo = 0.0F, hi = 0.0F;
    std::size_t negative = 0, over_one = 0;
    const auto* p = linear->rgb.get();
    for (std::size_t i = 0; i < n; ++i) {
      lo = std::min(lo, p[i]);
      hi = std::max(hi, p[i]);
      negative += p[i] < 0.0F ? 1U : 0U;
      over_one += p[i] > 1.0F ? 1U : 0U;
    }
    std::cout << "path A: " << linear->width << "x" << linear->height
              << " decoder=" << linear->decoder_used
              << " range=[" << lo << "," << hi << "]"
              << " negative=" << negative << " over1=" << over_one << "\n";
    // Diagnostic: dump the scene-linear frame encoded to 8-bit sRGB as a PPM,
    // for comparing path A's colour against the camera's embedded JPEG.
    if (const char* dump = std::getenv("X3F_SMOKE_DUMP_A")) {
      if (FILE* f = std::fopen(dump, "wb")) {
        std::fprintf(f, "P6\n%d %d\n255\n", linear->width, linear->height);
        std::vector<unsigned char> row(static_cast<std::size_t>(linear->width) * 3U);
        for (int y = 0; y < linear->height; ++y) {
          const float* s = p + static_cast<std::size_t>(y) * linear->width * 3U;
          for (int x = 0; x < linear->width * 3; ++x) {
            const double v = static_cast<double>(s[x]);
            const double e = v <= 0.0031308 ? 12.92 * v
                                            : 1.055 * std::pow(v, 1.0 / 2.4) - 0.055;
            row[static_cast<std::size_t>(x)] = static_cast<unsigned char>(
                std::clamp(e * 255.0 + 0.5, 0.0, 255.0));
          }
          std::fwrite(row.data(), 1, row.size(), f);
        }
        std::fclose(f);
        std::cout << "path A dump: " << dump << "\n";
      }
    }
    // Quattro is fixed to own; Merrill (fuse choice) and classic are fixed to
    // fuse. AUTO must resolve exactly that way, with no UI picker involved.
    const int expected_decoder =
        caps->model_class == alcedo::x3f::kFoveonModelQuattro ? 1 : 0;
    if (linear->decoder_used != expected_decoder) {
      std::cerr << "FAIL: path A AUTO picked decoder " << linear->decoder_used
                << ", expected " << expected_decoder << "\n";
      return 1;
    }
    if (lo > 0.0F && hi <= 1.0F) {
      std::cerr << "NOTE: this sample has no out-of-sRGB values; "
                   "wide-gamut smoke needs a saturated sample\n";
    }
  }

  std::cout << "PASS: foveon bridge\n";
  return 0;
}
