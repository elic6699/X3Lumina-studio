//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>

namespace alcedo::x3f {

// ---------------------------------------------------------------------------
// Path B: the X3Lumina Foveon native chain.
//
// This is a DISPLAY-REFERRED CPU sidecar. It runs the legacy f64 tone/gamma
// chain and yields 16-bit sRGB *finished image* code values. It is never fed
// into path A (x3f_support.hpp::DecodeEncoded), the DirectRgb Develop DAG or
// any wide-gamut export: negative channels and >1 scene-linear headroom do not
// survive this chain, and feeding it into the DAG would apply a second tone
// curve. The user opts into path B explicitly; path A stays the default and is
// not touched by any of the types below.
//
// Mirrors X3Lumina's Engine.swift two-stage preview:
//   Prepare() decodes one (optionally downsampled) linear frame and caches it;
//   Apply()   re-runs only the display chain for look-slider changes.
// Decode-level switches (overexposure / hole-fix / HLP / WB preset / NLM /
// frame / downsample) invalidate the prepared frame; DecodeKey() exposes that.
// ---------------------------------------------------------------------------

/// Full adjustment state, 1:1 with X3Lumina's RawEditParams and the C
/// x3f_alcedo_edit_t. The C default is the single source of truth: the default
/// constructor reads x3f_alcedo_edit_default(), so a plain
/// `FoveonEditParams{}` already has the product defaults (including the three
/// switches that are ON in this fork) and cannot drift from the C chain.
struct FoveonEditParams {
  FoveonEditParams();

  // Tone (Model.swift:323-327)
  double exposure   = 0.0;  // -2..+2 EV
  double contrast   = 0.0;  // -1..+1
  double shadow     = 0.0;  // -1..+1, positive lifts shadows
  double highlight  = 0.0;  // -1..+1, positive darkens highlights
  double saturation = 0.0;  // -1..+1

  // Detail / sharpening (:346, :398)
  double detail    = 0.5;  // 0..1
  double sharpness = 0.0;  // 0..2, 0 = mode default
  double sharp_res = 1.0;  // 0<v<1 scales sharpening for a preview frame

  // Tone curve four bands + three range breakpoints (:388-394)
  std::array<double, 7> tc{};  // tcSh, tcDk, tcLi, tcHi, tcR2(64), tcR3(128), tcR4(192)

  // White balance (:587-588, :612, :622-624)
  double wb_temp  = 0.0;  // -1..+1, APPLY level
  double wb_tint  = 0.0;  // -1..+1, APPLY level
  double wb_red   = 1.0;  // 0..2
  double wb_green = 1.0;
  double wb_blue  = 1.0;
  /// Empty = as-shot. A preset name selects a different per-illuminant colour
  /// matrix, so it is DECODE level and forces a re-prepare.
  std::string wb_preset;

  // Highlight panel (Views.swift:2739-3575). Overexposure correction is a
  // TWO-STATE switch ("关闭" / "mode8"), never a slider.
  double hl_recover = 0.0;  // 0..1
  double hl_keep    = 0.5;  // 0..1
  int    oe         = 1;    // 0 = off, 1 = mode8 (SPP Stage2 replay)
  int    hole_fix   = 1;    // 高光去粉, Quattro only
  int    merrill_hl_neut = 1;  // 过曝天空中性化, Merrill / classic only
  int    hlp_fix    = 0;    // hidden HLP grid-point fix
  double sdq_hl     = 0.0;  // 0..2, SDQ / SDQH only
  double hl_chroma  = 1.0;  // 1.0..2.4, 1.0 = off
  int    hl_chroma_hue_gate = 1;  // 只作用于黄/橙
  int    oe_m8od    = 0;    // hidden mode8 od delta

  // Fill light + six calibration knobs (:593-604)
  double fill_light = 0.0;  // -2..+2 step 0.1
  int    fl_tune    = 1;
  double fl_hlzred  = 0.95;
  double fl_lmean_r = 0.0;
  double fl_msat    = 0.0;
  double fl_mapgain = 0.75;
  double fl_k0off   = 0.0;
  double fl_zgain   = 1.0;

  // SPP noise sliders (:781-790), quantised to 5 steps by the C chain
  double nr_luma    = 0.0;
  double nr_chroma  = 0.0;
  double nr_banding = 0.0;
  int    nr_q_baseline = 0;
  double nr_q_base     = 0.8;
  int    nr_q_gate     = 1;
  double nr_q_gate_k   = 0.75;

  // Monochrome page (:840-873)
  int    mono = 0;
  double mono_mix_r = 100.0, mono_mix_g = 100.0, mono_mix_b = 100.0;
  double mono_tone_r = 100.0, mono_tone_g = 100.0, mono_tone_b = 100.0;
  int    mono_grain = 0;
  double mono_grain_size      = 0.5;
  double mono_grain_roughness = 0.5;

  // Split toning (:989-999) / glow (:1008-1012)
  double split_sh_hue = 0.0, split_sh_sat = 0.0;
  double split_hl_hue = 0.0, split_hl_sat = 0.0;
  double split_balance  = 0.0;
  double split_blending = 50.0;
  double glow = 0.0, glow_radius = 0.5, glow_threshold = 0.7;

  // Colour mode (:973-979). "AsShot" resolves against the host-provided
  // MakerNote detection, then the header probe, then Standard.
  std::string color_mode           = "AsShot";
  std::string detected_color_mode;
  double      color_strength       = 50.0;  // 0..100, 50 = mode as-is

  // Display chain misc
  int sfd_tone       = 0;  // X3I SFD tone table (APPLY level)
  int out_space      = 0;  // X3F_OUT_*, sRGB by default
  int look_defer     = 0;  // 1 = also return unclamped scene-linear floats
  int align_look_off = 0;

  // Decode-level extras
  int nlm   = 0;  // 0..10, 0 = off
  int frame = 0;  // X3I frame index, 0 = 0EV
  int down  = 1;  // >=1 integer box downsample of the decoded linear frame
};

/// Hash of the decode-level fields. A change means the prepared frame is stale.
/// Look sliders deliberately do not appear here.
[[nodiscard]] auto FoveonDecodeKey(const FoveonEditParams& params) -> std::uint64_t;

/// Hash of every field that changes the scene-linear result the Develop DAG
/// receives: the decode key plus the apply-level operators that run before the
/// tone stage (fill light and its tuning, detail, sharpness, highlight
/// recovery/keep, overexposure handling). The DAG owns exposure, contrast,
/// curves, colour and WB, so those are deliberately excluded — otherwise every
/// exposure drag would force a multi-second re-decode.
[[nodiscard]] auto FoveonLinearKey(const FoveonEditParams& params) -> std::uint64_t;

/// Per-generation gate, exactly like Model.swift:664-670. Returns the value for
/// x3f_render_set_oe_neutralize(): 8 for Quattro+mode8, 4 for Quattro off,
/// 3 for Merrill/classic sky neutralisation, 4 otherwise.
[[nodiscard]] auto FoveonResolveOe(const FoveonEditParams& params, int model_class) -> int;

/// Decoder/model capabilities for UI gating. Probe only reads headers and the
/// trailing directory; no sensor plane is decompressed.
/// Mirrors X3F_ALCEDO_CLASSIC / MERRILL / QUATTRO without exposing the C
/// header to the UI layer.
inline constexpr int kFoveonModelClassic = 0;
inline constexpr int kFoveonModelMerrill = 1;
inline constexpr int kFoveonModelQuattro = 2;

struct FoveonCapabilities {
  int  model_class        = kFoveonModelClassic;
  /// Gate generation. Differs from model_class on SD Quattro / SD Quattro H:
  /// their RAW type_format is SDQ (0x25) / SDQH (0x27) rather than QUATTRO
  /// (0x23), but every overexposure / hole-fix / SDQ-highlight decision in
  /// X3Lumina treats them as Quattro. All panel gating uses this field.
  int  model_gen          = kFoveonModelClassic;
  /// This file is an SD Quattro / SD Quattro H body: the only generations on
  /// which "SDQ 高光" is offered in the panel.
  bool sdq_hl_capable     = false;
  /// DigitalISOGain != (1,1,1): the slider actually changes pixels. SDQH
  /// normalises (4,4,2); SDQ is (1,1,1) so its slider moves but is a no-op.
  bool sdq_hl_effective   = false;
  int  available_decoders = 0;  // bitmask of X3F_ALCEDO_HAS_*
  bool spp_capable        = false;
  bool is_container       = false;
  int  frame_count        = 1;
  int  width              = 0;
  int  height             = 0;
  bool has_jpeg           = false;

  /// The only generation that offers a decode-backend choice (fuse vs SPP).
  [[nodiscard]] auto OffersDecoderChoice() const -> bool {
    return model_class == kFoveonModelMerrill && spp_capable;
  }
};

[[nodiscard]] auto FoveonProbe(const std::filesystem::path& path)
    -> std::optional<FoveonCapabilities>;

/// Suggested integer box downsample so the prepared preview frame is no larger
/// than @p long_edge pixels on its long side. load_wb() uses ceil(dim/down),
/// so any integer >= 1 is valid (not just powers of two).
[[nodiscard]] auto FoveonPreviewDown(const FoveonCapabilities& caps,
                                     int long_edge) -> int;

/// 16-bit sRGB display code values, RGB interleaved, sensor order.
/// This is a finished image, NOT scene-linear, and must not be clamped again.
struct FoveonImage {
  std::shared_ptr<const std::uint16_t> rgb;
  int width       = 0;
  int height      = 0;
  int orientation = 1;  // EXIF 1..8; the renderer applies it once

  [[nodiscard]] auto Empty() const -> bool {
    return !rgb || width <= 0 || height <= 0;
  }
};

struct FoveonGeometry {
  int width       = 0;
  int height      = 0;
  int orientation = 1;
};

/// Unclamped scene-linear RGB in **linear sRGB primaries**, RGB interleaved,
/// sensor order. This is the only path-B product that may enter the Develop
/// DAG: values below 0 and above 1 are real data (out-of-sRGB-gamut colours and
/// highlight headroom) and must not be clamped, gamma-encoded or tagged as
/// display-referred. Because the camera->sRGB matrix is already folded in, the
/// DAG must be told the input primaries are sRGB (colour matrix = XYZ->sRGB),
/// which composes back to camera->XYZ without loss.
struct FoveonLinearImage {
  std::shared_ptr<const float> rgb;
  int                         width       = 0;
  int                         height      = 0;
  int                         orientation = 1;

  [[nodiscard]] auto Empty() const -> bool {
    return !rgb || width <= 0 || height <= 0;
  }
};

/// RAII owner of one prepared (cached) linear frame. Copyable: the underlying
/// C frame is refcounted, exactly like X3Lumina's linCache.
class FoveonFrame {
 public:
  FoveonFrame() = default;

  /// Uses the source path directly. prepare() materializes X3I frames itself,
  /// so a container path is fine and no caller-side staging is required.
  [[nodiscard]] static auto Prepare(const std::filesystem::path& path,
                                    const FoveonEditParams& params)
      -> std::optional<FoveonFrame>;

  /// For callers that only hold an opaque encoded buffer (e.g. the decode
  /// cache). Stages the bytes to a private temp file that is kept alive until
  /// this frame is destroyed, because apply() resolves colour modes against
  /// the frame's source path.
  [[nodiscard]] static auto Prepare(std::span<const std::byte> encoded,
                                    const FoveonEditParams& params)
      -> std::optional<FoveonFrame>;

  /// Re-runs only the display chain. Fails when a decode-level field changed
  /// since Prepare(); call Prepare() again in that case.
  [[nodiscard]] auto Apply(const FoveonEditParams& params) const
      -> std::optional<FoveonImage>;

  /// Scene-linear companion of Apply(): WB / fill light / detail / colour-mode
  /// matrix, no tone curve, no clamp. Feeds the Develop DAG.
  [[nodiscard]] auto ApplyLinear(const FoveonEditParams& params) const
      -> std::optional<FoveonLinearImage>;

  [[nodiscard]] auto Geometry() const -> FoveonGeometry { return geometry_; }
  [[nodiscard]] auto DecodeKey() const -> std::uint64_t { return decode_key_; }
  [[nodiscard]] auto Valid() const -> bool { return handle_ != nullptr; }

 private:
  /// Shared assembly for path-backed and memory-backed prepares. The staged
  /// guard is type-erased here (the concrete temp-file type lives in the .cpp);
  /// the handle's deleter retains it so the file outlives the C frame.
  [[nodiscard]] static auto PrepareImpl(const std::filesystem::path& path,
                                        const FoveonEditParams& params,
                                        std::string source_id,
                                        std::shared_ptr<void> staged_guard)
      -> std::optional<FoveonFrame>;

  // deleter = x3f_alcedo_foveon_free; the deleter also retains the staged temp
  // file guard for memory-backed frames.
  std::shared_ptr<void> handle_;
  std::string           source_id_;
  std::uint64_t         decode_key_ = 0;
  FoveonGeometry        geometry_{};

  friend class FoveonRenderer;
};

/// Serial preview renderer: keeps one prepared frame and re-prepares only when
/// the source or a decode-level parameter changes. All calls are serialized on
/// an internal mutex because the legacy C chain uses process-global switches.
class FoveonRenderer {
 public:
  FoveonRenderer() = default;

  [[nodiscard]] auto Render(const std::filesystem::path& path,
                            const FoveonEditParams& params)
      -> std::optional<FoveonImage>;
  [[nodiscard]] auto Render(std::span<const std::byte> encoded,
                            const FoveonEditParams& params)
      -> std::optional<FoveonImage>;

  /// Scene-linear companion of Render(encoded): same prepared-frame cache, but
  /// the deferred-tone product the Develop DAG consumes. Fill light, detail,
  /// sharpening, highlight handling and NR are APPLY-level, so changing one
  /// re-runs the apply pass on the cached frame instead of decoding again —
  /// which is what X3Lumina's linCache does and why its fill light is instant.
  [[nodiscard]] auto RenderLinear(std::span<const std::byte> encoded,
                                  const FoveonEditParams& params)
      -> std::optional<FoveonLinearImage>;

  /// Merrill SPP native chain with its six tone-stage controls. The expensive
  /// prepare() (decode + float front end + noise reduction) is cached; tone
  /// changes only re-run the ~30 ms tonal stage. nr_luma is a prepare-level
  /// field, so changing it re-prepares the full-resolution SPP buffer.
  [[nodiscard]] auto RenderSpp(const std::filesystem::path& path,
                               const FoveonEditParams& params, int frame = -1)
      -> std::optional<FoveonImage>;

  void Invalidate();
  [[nodiscard]] auto LastError() const -> std::string;

 private:
  mutable std::mutex     mutex_;
  std::optional<FoveonFrame> frame_;
  std::string            source_id_;
  std::string            error_;

  // SPP two-stage cache. Kept separate from path-B's FoveonFrame: the SPP
  // product is a finished display image and must never feed path A/B mixing.
  std::shared_ptr<void>  spp_session_;
  std::string            spp_source_id_;
  int                    spp_width_       = 0;
  int                    spp_height_      = 0;
  int                    spp_orientation_ = 1;
  int                    spp_frame_       = -1;
  int                    spp_nr_level_    = -1;
};

/// One-shot prepare + apply for export / batch / tests (no cache retained).
[[nodiscard]] auto RenderFoveonOnce(const std::filesystem::path& path,
                                    const FoveonEditParams& params)
    -> std::optional<FoveonImage>;

/// Scene-linear feed for the Develop DAG, with the decoded frame kept between
/// calls. A slider that only touches APPLY-level operators (fill light, detail,
/// sharpness, highlight recovery, NR, SDQ highlight) re-runs the apply pass; a
/// DECODE-level switch (overexposure / hole fix / HLP / WB preset / NLM / LOD)
/// prepares a new frame. Keyed by the encoded bytes, so switching images costs
/// one decode and coming back is free (two slots, LRU).
[[nodiscard]] auto FoveonLinearFeed(std::span<const std::byte> encoded,
                                    const FoveonEditParams& params)
    -> std::optional<FoveonLinearImage>;

/// Drop the cached frames (export finished, session closed, memory pressure).
void FoveonLinearFeedFlush();

/// Merrill "原链解码" (SPP native chain) straight-out output. X3Lumina's
/// MerrillDecodeBackend.sppPipeline: the SPP chain produces a FINISHED 16-bit
/// sRGB image, not scene-linear data, so it bypasses both path A (wide-gamut
/// Develop DAG) and the path-B apply_ex display chain. Only Merrill HI files
/// qualify; check FoveonProbe().spp_capable first. The C entry point owns the
/// explicit SPP gate (set_enabled 1 -> -1), so no process-global setup is
/// needed here. out is released with x3f_alcedo_spp16_free by the wrapper.
[[nodiscard]] auto RenderSppOnce(const std::filesystem::path& path,
                                 int frame = -1) -> std::optional<FoveonImage>;

/// Last error reported by the C path-B chain (static string, no free).
[[nodiscard]] auto FoveonLastError() -> std::string;

// ── Active decode parameters for the Develop pipeline ──────────────────────
//
// The Foveon operators (fill light, highlight, detail, noise reduction) run
// inside the native decode, not in the host DAG, so the decode must know them.
// The Develop payload only carries a digest (it drives the re-render); these
// accessors carry the parameters themselves, keyed by the encoded content hash
// so a stale entry can never be applied to another file.
//
// Thread-safe: the UI thread publishes, the render worker reads.

/// Publish the parameters that apply to the file whose encoded bytes hash to
/// @p content_hash. Replaces any previous entry for the same hash.
void SetActiveFoveonParams(std::uint64_t content_hash, const FoveonEditParams& params);

/// Drop every published entry (image closed, or a session shutdown).
void ClearActiveFoveonParams();

/// Parameters for @p content_hash, or null when the file decodes through the
/// plain path (no Foveon operators, no SPP).
[[nodiscard]] auto ActiveFoveonParams(std::uint64_t content_hash)
    -> std::optional<FoveonEditParams>;

/// Cache-key variant for @p content_hash: 0 when no entry is published,
/// otherwise the digest of everything that changes the scene-linear result.
/// PreparedSourceCache folds this into its lookup key so a slider change is a
/// miss and an exposure change is a hit.
[[nodiscard]] auto ActiveFoveonVariant(std::uint64_t content_hash) -> std::uint64_t;

}  // namespace alcedo::x3f
