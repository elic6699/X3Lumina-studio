//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "decoders/x3f/x3f_foveon.hpp"

#include <x3f_alcedo_export.h>
#include <x3f_alcedo_foveon.h>

#include <algorithm>
#include <atomic>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string_view>
#include <utility>

#include "decoders/x3f/x3f_support.hpp"

namespace alcedo::x3f {
namespace {

void CopyToC(std::string_view src, char* dst, std::size_t cap) {
  if (cap == 0) {
    return;
  }
  const auto n = std::min(src.size(), cap - 1U);
  std::memcpy(dst, src.data(), n);
  dst[n] = '\0';
}

auto FromCString(const char* src, std::size_t cap) -> std::string {
  const auto end = std::find(src, src + cap, '\0');
  return std::string(src, end);
}

/// X3Lumina maps its 0..1 辉度杂讯 control to one of the five SPP denoise
/// rows: 0 -> native row 2, 0.25/0.5 -> 3, 0.75/1 -> 4. The value is baked
/// into the SPP prepare stage, unlike the other five tone controls.
auto SppNoiseLevel(double nr_luma) -> int {
  if (!std::isfinite(nr_luma) || nr_luma <= 0.0) {
    return 2;
  }
  const double v = std::clamp(nr_luma, 0.0, 1.0);
  return std::clamp(2 + static_cast<int>(std::lround(v * 2.0)), 2, 4);
}

/// Translate the product-facing FoveonEditParams into SPP's own tone struct.
/// `highlight` is negated because X3Lumina's UI says positive = darken while
/// the SPP tone stage documents positive = lift. Everything else is already
/// in SPP's units.
auto ToSppTone(const FoveonEditParams& p) -> x3f_alcedo_spp_tone_t {
  x3f_alcedo_spp_tone_t tone{};
  tone.exposure   = p.exposure;
  tone.contrast   = p.contrast;
  tone.shadows    = p.shadow;
  tone.highlights = -p.highlight;
  tone.saturation = std::clamp(p.saturation, -2.0, 2.0);
  tone.nr_level   = SppNoiseLevel(p.nr_luma);
  return tone;
}

/// The C default is authoritative. Every field is assigned here, so the
/// initializers in the header are documentation only and cannot drift.
void AssignFromC(FoveonEditParams& p, const x3f_alcedo_edit_t& e) {
  p.exposure   = e.exposure;
  p.contrast   = e.contrast;
  p.shadow     = e.shadow;
  p.highlight  = e.highlight;
  p.saturation = e.saturation;

  p.detail    = e.detail;
  p.sharpness = e.sharpness;
  p.sharp_res = e.sharp_res;
  std::copy_n(e.tc, p.tc.size(), p.tc.begin());

  p.wb_temp  = e.wb_temp;
  p.wb_tint  = e.wb_tint;
  p.wb_red   = e.wb_red;
  p.wb_green = e.wb_green;
  p.wb_blue  = e.wb_blue;
  p.wb_preset = FromCString(e.wb_preset, sizeof(e.wb_preset));

  p.hl_recover          = e.hl_recover;
  p.hl_keep             = e.hl_keep;
  p.oe                  = e.oe;
  p.hole_fix            = e.hole_fix;
  p.merrill_hl_neut     = e.merrill_hl_neut;
  p.hlp_fix             = e.hlp_fix;
  p.sdq_hl              = e.sdq_hl;
  p.hl_chroma           = e.hl_chroma;
  p.hl_chroma_hue_gate  = e.hl_chroma_hue_gate;
  p.oe_m8od             = e.oe_m8od;

  p.fill_light  = e.fill_light;
  p.fl_tune     = e.fl_tune;
  p.fl_hlzred   = e.fl_hlzred;
  p.fl_lmean_r  = e.fl_lmean_r;
  p.fl_msat     = e.fl_msat;
  p.fl_mapgain  = e.fl_mapgain;
  p.fl_k0off    = e.fl_k0off;
  p.fl_zgain    = e.fl_zgain;

  p.nr_luma       = e.nr_luma;
  p.nr_chroma     = e.nr_chroma;
  p.nr_banding    = e.nr_banding;
  p.nr_q_baseline = e.nr_q_baseline;
  p.nr_q_base     = e.nr_q_base;
  p.nr_q_gate     = e.nr_q_gate;
  p.nr_q_gate_k   = e.nr_q_gate_k;

  p.mono                  = e.mono;
  p.mono_mix_r            = e.mono_mix_r;
  p.mono_mix_g            = e.mono_mix_g;
  p.mono_mix_b            = e.mono_mix_b;
  p.mono_tone_r           = e.mono_tone_r;
  p.mono_tone_g           = e.mono_tone_g;
  p.mono_tone_b           = e.mono_tone_b;
  p.mono_grain            = e.mono_grain;
  p.mono_grain_size       = e.mono_grain_size;
  p.mono_grain_roughness  = e.mono_grain_roughness;

  p.split_sh_hue   = e.split_sh_hue;
  p.split_sh_sat   = e.split_sh_sat;
  p.split_hl_hue   = e.split_hl_hue;
  p.split_hl_sat   = e.split_hl_sat;
  p.split_balance  = e.split_balance;
  p.split_blending = e.split_blending;
  p.glow           = e.glow;
  p.glow_radius    = e.glow_radius;
  p.glow_threshold = e.glow_threshold;

  p.color_mode           = FromCString(e.color_mode, sizeof(e.color_mode));
  p.detected_color_mode  = FromCString(e.detected_color_mode,
                                       sizeof(e.detected_color_mode));
  p.color_strength       = e.color_strength;

  p.sfd_tone       = e.sfd_tone;
  p.out_space      = e.out_space;
  p.look_defer     = e.look_defer;
  p.align_look_off = e.align_look_off;

  p.nlm   = e.nlm;
  p.frame = e.frame;
  p.down  = e.down;
}

auto ToC(const FoveonEditParams& p) -> x3f_alcedo_edit_t {
  // Start from the C defaults: out_space / hidden switches / calibration
  // constants stay in sync with the chain even when a field is added there.
  x3f_alcedo_edit_t e{};
  x3f_alcedo_edit_default(&e);

  e.exposure   = p.exposure;
  e.contrast   = p.contrast;
  e.shadow     = p.shadow;
  e.highlight  = p.highlight;
  e.saturation = p.saturation;

  e.detail    = p.detail;
  e.sharpness = p.sharpness;
  e.sharp_res = p.sharp_res;
  std::copy_n(p.tc.begin(), p.tc.size(), e.tc);

  e.wb_temp  = p.wb_temp;
  e.wb_tint  = p.wb_tint;
  e.wb_red   = p.wb_red;
  e.wb_green = p.wb_green;
  e.wb_blue  = p.wb_blue;
  CopyToC(p.wb_preset, e.wb_preset, sizeof(e.wb_preset));

  e.hl_recover         = p.hl_recover;
  e.hl_keep            = p.hl_keep;
  e.oe                 = p.oe;
  e.hole_fix           = p.hole_fix;
  e.merrill_hl_neut    = p.merrill_hl_neut;
  e.hlp_fix            = p.hlp_fix;
  e.sdq_hl             = p.sdq_hl;
  e.hl_chroma          = p.hl_chroma;
  e.hl_chroma_hue_gate = p.hl_chroma_hue_gate;
  e.oe_m8od            = p.oe_m8od;

  e.fill_light = p.fill_light;
  e.fl_tune    = p.fl_tune;
  e.fl_hlzred  = p.fl_hlzred;
  e.fl_lmean_r = p.fl_lmean_r;
  e.fl_msat    = p.fl_msat;
  e.fl_mapgain = p.fl_mapgain;
  e.fl_k0off   = p.fl_k0off;
  e.fl_zgain   = p.fl_zgain;

  e.nr_luma       = p.nr_luma;
  e.nr_chroma     = p.nr_chroma;
  e.nr_banding    = p.nr_banding;
  e.nr_q_baseline = p.nr_q_baseline;
  e.nr_q_base     = p.nr_q_base;
  e.nr_q_gate     = p.nr_q_gate;
  e.nr_q_gate_k   = p.nr_q_gate_k;

  e.mono                 = p.mono;
  e.mono_mix_r           = p.mono_mix_r;
  e.mono_mix_g           = p.mono_mix_g;
  e.mono_mix_b           = p.mono_mix_b;
  e.mono_tone_r          = p.mono_tone_r;
  e.mono_tone_g          = p.mono_tone_g;
  e.mono_tone_b          = p.mono_tone_b;
  e.mono_grain           = p.mono_grain;
  e.mono_grain_size      = p.mono_grain_size;
  e.mono_grain_roughness = p.mono_grain_roughness;

  e.split_sh_hue   = p.split_sh_hue;
  e.split_sh_sat   = p.split_sh_sat;
  e.split_hl_hue   = p.split_hl_hue;
  e.split_hl_sat   = p.split_hl_sat;
  e.split_balance  = p.split_balance;
  e.split_blending = p.split_blending;
  e.glow           = p.glow;
  e.glow_radius    = p.glow_radius;
  e.glow_threshold = p.glow_threshold;

  CopyToC(p.color_mode, e.color_mode, sizeof(e.color_mode));
  CopyToC(p.detected_color_mode, e.detected_color_mode,
          sizeof(e.detected_color_mode));
  e.color_strength = p.color_strength;

  e.sfd_tone       = p.sfd_tone;
  e.out_space      = p.out_space;
  e.look_defer     = p.look_defer;
  e.align_look_off = p.align_look_off;

  e.nlm   = p.nlm;
  e.frame = p.frame;
  e.down  = p.down;
  return e;
}

/// Private temp file used when only an opaque encoded buffer is available.
/// It must outlive the prepared frame: x3f_alcedo_foveon_apply() resolves the
/// colour-mode tables against the frame's recorded source path.
class StagedX3fFile {
 public:
  static auto Create(std::span<const std::byte> encoded)
      -> std::shared_ptr<StagedX3fFile> {
    static std::atomic<std::uint64_t> sequence{0};
    std::error_code ec;
    const auto dir = std::filesystem::temp_directory_path(ec);
    if (ec) {
      return nullptr;
    }
    const auto name = "alcedo_x3f_foveon_" + std::to_string(sequence.fetch_add(1)) +
                      "_" + std::to_string(encoded.size()) + ".x3f";
    auto path = dir / name;
    // A stale file from a previous crashed run must never be decoded.
    for (int attempt = 0; attempt < 8 && std::filesystem::exists(path, ec); ++attempt) {
      path = dir / ("alcedo_x3f_foveon_" +
                    std::to_string(sequence.fetch_add(1)) + "_" +
                    std::to_string(encoded.size()) + ".x3f");
    }
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
      return nullptr;
    }
    out.write(reinterpret_cast<const char*>(encoded.data()),
              static_cast<std::streamsize>(encoded.size()));
    out.close();
    if (!out) {
      std::filesystem::remove(path, ec);
      return nullptr;
    }
    return std::shared_ptr<StagedX3fFile>(new StagedX3fFile(std::move(path)));
  }

  ~StagedX3fFile() {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }

  [[nodiscard]] auto Path() const -> const std::filesystem::path& { return path_; }

  StagedX3fFile(const StagedX3fFile&) = delete;
  auto operator=(const StagedX3fFile&) -> StagedX3fFile& = delete;

 private:
  explicit StagedX3fFile(std::filesystem::path path) : path_(std::move(path)) {}
  std::filesystem::path path_;
};

auto PathText(const std::filesystem::path& path) -> std::string {
  const auto text = path.u8string();
  return std::string(text.begin(), text.end());
}

auto Fnv1a(std::span<const std::byte> bytes) -> std::uint64_t {
  std::uint64_t h = 1469598103934665603ULL;
  for (const auto b : bytes) {
    h ^= std::to_integer<std::uint8_t>(b);
    h *= 1099511628211ULL;
  }
  return h;
}

}  // namespace

auto FoveonFrame::PrepareImpl(const std::filesystem::path& path,
                              const FoveonEditParams& params,
                              std::string source_id,
                              std::shared_ptr<void> staged_guard)
    -> std::optional<FoveonFrame> {
  const auto c_params = ToC(params);
  const auto text     = PathText(path);
  x3f_alcedo_foveon_frame_t* raw = nullptr;
  if (x3f_alcedo_foveon_prepare(text.c_str(), &c_params, &raw) != X3F_ALCEDO_OK ||
      raw == nullptr) {
    return std::nullopt;
  }

  FoveonFrame frame;
  // The deleter retains the staged guard (if any), so the temp file is removed
  // only after the C frame has been freed and no apply() can use its path.
  frame.handle_ = std::shared_ptr<void>(
      raw, [staged = std::move(staged_guard)](void* p) mutable {
        x3f_alcedo_foveon_free(static_cast<x3f_alcedo_foveon_frame_t*>(p));
        staged.reset();
      });
  frame.source_id_  = std::move(source_id);
  frame.decode_key_ = x3f_alcedo_foveon_decode_key(&c_params);
  x3f_alcedo_foveon_geometry(raw, &frame.geometry_.width, &frame.geometry_.height,
                             &frame.geometry_.orientation);
  if (frame.geometry_.orientation < 1 || frame.geometry_.orientation > 8) {
    frame.geometry_.orientation = 1;
  }
  return frame;
}

FoveonEditParams::FoveonEditParams() {
  x3f_alcedo_edit_t defaults{};
  x3f_alcedo_edit_default(&defaults);
  AssignFromC(*this, defaults);
}

auto FoveonDecodeKey(const FoveonEditParams& params) -> std::uint64_t {
  const auto c = ToC(params);
  return x3f_alcedo_foveon_decode_key(&c);
}

auto FoveonResolveOe(const FoveonEditParams& params, int model_class) -> int {
  const auto c = ToC(params);
  return x3f_alcedo_foveon_resolve_oe(&c, model_class);
}

auto FoveonProbe(const std::filesystem::path& path)
    -> std::optional<FoveonCapabilities> {
  const auto probe = Probe(path);
  if (!probe) {
    return std::nullopt;
  }
  const auto text = PathText(path);
  FoveonCapabilities caps;
  caps.model_class        = x3f_alcedo_model_class(text.c_str());
  caps.model_gen          = x3f_alcedo_model_gen(text.c_str());
  caps.sdq_hl_capable     = x3f_alcedo_sdq_hl_capable(text.c_str()) != 0;
  caps.sdq_hl_effective   = x3f_alcedo_sdq_hl_effective(text.c_str()) != 0;
  caps.available_decoders = x3f_alcedo_available_decoders(text.c_str());
  int spp_w = 0, spp_h = 0;
  caps.spp_capable   = x3f_alcedo_spp_capable(text.c_str(), -1, &spp_w, &spp_h) != 0;
  caps.is_container  = probe->is_container;
  caps.frame_count   = probe->frame_count;
  caps.width         = probe->width;
  caps.height        = probe->height;
  caps.has_jpeg      = probe->has_jpeg;
  return caps;
}

auto FoveonPreviewDown(const FoveonCapabilities& caps, int long_edge) -> int {
  if (long_edge <= 0 || caps.width <= 0 || caps.height <= 0) {
    return 1;
  }
  const double longest = static_cast<double>(std::max(caps.width, caps.height));
  const double down    = std::ceil(longest / static_cast<double>(long_edge));
  return static_cast<int>(std::max(1.0, down));
}

auto FoveonFrame::Prepare(const std::filesystem::path& path,
                          const FoveonEditParams& params)
    -> std::optional<FoveonFrame> {
  if (!IsX3fFile(path)) {
    return std::nullopt;
  }
  return PrepareImpl(path, params, PathText(path), nullptr);
}

auto FoveonFrame::Prepare(std::span<const std::byte> encoded,
                          const FoveonEditParams& params)
    -> std::optional<FoveonFrame> {
  if (!IsX3fEncoded(encoded)) {
    return std::nullopt;
  }
  auto staged = StagedX3fFile::Create(encoded);
  if (!staged) {
    return std::nullopt;
  }
  const std::string id =
      "mem:" + std::to_string(encoded.size()) + ":" +
      std::to_string(Fnv1a(encoded));
  return PrepareImpl(staged->Path(), params, id, std::move(staged));
}

auto FoveonFrame::Apply(const FoveonEditParams& params) const
    -> std::optional<FoveonImage> {
  if (!handle_) {
    return std::nullopt;
  }
  const auto c_params = ToC(params);
  x3f_alcedo_foveon_image_t raw{};
  if (x3f_alcedo_foveon_apply(
          static_cast<x3f_alcedo_foveon_frame_t*>(handle_.get()), &c_params,
          &raw) != X3F_ALCEDO_OK ||
      raw.rgb == nullptr || raw.width <= 0 || raw.height <= 0) {
    return std::nullopt;
  }

  FoveonImage image;
  image.rgb = std::shared_ptr<const std::uint16_t>(
      raw.rgb, [](const std::uint16_t* p) {
        std::free(const_cast<std::uint16_t*>(p));
      });
  // Path B only exposes the 16-bit sRGB finished image. The optional
  // look_defer linear scratch buffer belongs to the engine front-end and is
  // dropped here; it is never routed into the wide-gamut path A.
  if (raw.linear_rgb != nullptr) {
    std::free(raw.linear_rgb);
  }
  image.width       = raw.width;
  image.height      = raw.height;
  image.orientation = (raw.orientation >= 1 && raw.orientation <= 8)
                          ? raw.orientation
                          : 1;
  return image;
}

auto FoveonRenderer::Render(const std::filesystem::path& path,
                            const FoveonEditParams& params)
    -> std::optional<FoveonImage> {
  const std::lock_guard lock(mutex_);
  const std::string id  = PathText(path);
  const auto        key = FoveonDecodeKey(params);
  if (!frame_ || !frame_->Valid() || source_id_ != id ||
      frame_->DecodeKey() != key) {
    auto prepared = FoveonFrame::Prepare(path, params);
    if (!prepared) {
      error_ = FoveonLastError();
      frame_.reset();
      return std::nullopt;
    }
    frame_     = std::move(prepared);
    source_id_ = id;
  }
  auto image = frame_->Apply(params);
  if (!image) {
    error_ = FoveonLastError();
    frame_.reset();  // force a clean re-prepare on the next call
    return std::nullopt;
  }
  error_.clear();
  return image;
}

auto FoveonRenderer::Render(std::span<const std::byte> encoded,
                            const FoveonEditParams& params)
    -> std::optional<FoveonImage> {
  const std::lock_guard lock(mutex_);
  const std::string id =
      "mem:" + std::to_string(encoded.size()) + ":" +
      std::to_string(Fnv1a(encoded));
  const auto key = FoveonDecodeKey(params);
  if (!frame_ || !frame_->Valid() || source_id_ != id ||
      frame_->DecodeKey() != key) {
    auto prepared = FoveonFrame::Prepare(encoded, params);
    if (!prepared) {
      error_ = FoveonLastError();
      frame_.reset();
      return std::nullopt;
    }
    frame_     = std::move(prepared);
    source_id_ = id;
  }
  auto image = frame_->Apply(params);
  if (!image) {
    error_ = FoveonLastError();
    frame_.reset();
    return std::nullopt;
  }
  error_.clear();
  return image;
}

auto FoveonRenderer::RenderSpp(const std::filesystem::path& path,
                               const FoveonEditParams& params, int frame)
    -> std::optional<FoveonImage> {
  if (!IsX3fFile(path)) {
    const std::lock_guard lock(mutex_);
    error_ = "not an X3F/X3I file";
    return std::nullopt;
  }

  const std::lock_guard lock(mutex_);
  const std::string id = PathText(path);
  const auto tone = ToSppTone(params);

  if (!spp_session_ || spp_source_id_ != id || spp_frame_ != frame ||
      spp_nr_level_ != tone.nr_level) {
    x3f_alcedo_spp_session* raw = nullptr;
    int w = 0;
    int h = 0;
    if (x3f_alcedo_spp_session_open(id.c_str(), frame, tone.nr_level, &raw, &w,
                                    &h) != X3F_ALCEDO_OK ||
        raw == nullptr || w <= 0 || h <= 0) {
      const char* text = x3f_alcedo_export_error();
      error_ = text == nullptr ? std::string("SPP prepare failed")
                               : std::string(text);
      spp_session_.reset();
      spp_source_id_.clear();
      spp_frame_ = -1;
      spp_nr_level_ = -1;
      return std::nullopt;
    }
    spp_session_ = std::shared_ptr<void>(
        raw, [](void* p) {
          x3f_alcedo_spp_session_close(
              static_cast<x3f_alcedo_spp_session*>(p));
        });
    spp_source_id_  = id;
    spp_width_      = w;
    spp_height_     = h;
    spp_frame_      = frame;
    spp_nr_level_   = tone.nr_level;
    spp_orientation_ = x3f_alcedo_spp_session_orientation(raw);
  }

  const auto n = static_cast<std::size_t>(spp_width_) *
                 static_cast<std::size_t>(spp_height_) * 3U;
  auto* buffer = static_cast<std::uint16_t*>(std::calloc(n, sizeof(std::uint16_t)));
  if (buffer == nullptr) {
    error_ = "SPP tone: out of memory";
    return std::nullopt;
  }

  int out_w = spp_width_;
  int out_h = spp_height_;
  const int stride = spp_width_ * 3 * static_cast<int>(sizeof(std::uint16_t));
  if (x3f_alcedo_spp_session_apply(
          static_cast<x3f_alcedo_spp_session*>(spp_session_.get()), &tone, buffer,
          stride, &out_w, &out_h) != X3F_ALCEDO_OK) {
    const char* text = x3f_alcedo_export_error();
    error_ = text == nullptr ? std::string("SPP tone stage failed")
                             : std::string(text);
    std::free(buffer);
    // A failed tone apply can mean the prepared buffer is stale (for example a
    // process-global SPP gate change); force the next call to prepare again.
    spp_session_.reset();
    spp_source_id_.clear();
    spp_frame_ = -1;
    spp_nr_level_ = -1;
    return std::nullopt;
  }

  FoveonImage image;
  image.rgb = std::shared_ptr<const std::uint16_t>(
      buffer, [](const std::uint16_t* p) {
        std::free(const_cast<std::uint16_t*>(p));
      });
  image.width       = out_w;
  image.height      = out_h;
  image.orientation = (spp_orientation_ >= 1 && spp_orientation_ <= 8)
                          ? spp_orientation_
                          : 1;
  error_.clear();
  return image;
}

void FoveonRenderer::Invalidate() {
  const std::lock_guard lock(mutex_);
  frame_.reset();
  source_id_.clear();
  spp_session_.reset();
  spp_source_id_.clear();
  spp_width_       = 0;
  spp_height_      = 0;
  spp_orientation_ = 1;
  spp_frame_       = -1;
  spp_nr_level_    = -1;
}

auto FoveonRenderer::LastError() const -> std::string {
  const std::lock_guard lock(mutex_);
  return error_;
}

auto RenderFoveonOnce(const std::filesystem::path& path,
                      const FoveonEditParams& params)
    -> std::optional<FoveonImage> {
  if (!IsX3fFile(path)) {
    return std::nullopt;
  }
  const auto c_params = ToC(params);
  x3f_alcedo_foveon_image_t raw{};
  if (x3f_alcedo_foveon_render(PathText(path).c_str(), &c_params, &raw) !=
          X3F_ALCEDO_OK ||
      raw.rgb == nullptr || raw.width <= 0 || raw.height <= 0) {
    return std::nullopt;
  }
  FoveonImage image;
  image.rgb = std::shared_ptr<const std::uint16_t>(
      raw.rgb, [](const std::uint16_t* p) {
        std::free(const_cast<std::uint16_t*>(p));
      });
  if (raw.linear_rgb != nullptr) {
    std::free(raw.linear_rgb);
  }
  image.width       = raw.width;
  image.height      = raw.height;
  image.orientation = (raw.orientation >= 1 && raw.orientation <= 8)
                          ? raw.orientation
                          : 1;
  return image;
}

auto RenderSppOnce(const std::filesystem::path& path, int frame)
    -> std::optional<FoveonImage> {
  if (!IsX3fFile(path)) {
    return std::nullopt;
  }
  x3f_alcedo_spp16_t raw{};
  if (x3f_alcedo_export_spp16(PathText(path).c_str(), frame, &raw) !=
          X3F_ALCEDO_OK ||
      raw.rgb == nullptr || raw.width <= 0 || raw.height <= 0) {
    x3f_alcedo_spp16_free(&raw);
    return std::nullopt;
  }
  FoveonImage image;
  // Take ownership instead of copying ~90 MB of full-resolution pixels.
  image.rgb = std::shared_ptr<const std::uint16_t>(
      raw.rgb, [](const std::uint16_t* p) {
        std::free(const_cast<std::uint16_t*>(p));
      });
  image.width       = raw.width;
  image.height      = raw.height;
  image.orientation = (raw.orientation >= 1 && raw.orientation <= 8)
                          ? raw.orientation
                          : 1;
  return image;
}

auto FoveonLastError() -> std::string {
  const char* text = x3f_alcedo_foveon_error();
  return text == nullptr ? std::string{} : std::string(text);
}

}  // namespace alcedo::x3f
