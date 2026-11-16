//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "app/foveon_edit_params_io.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <type_traits>
#include <utility>

namespace alcedo::x3f {
namespace {

using Json = nlohmann::json;

template <auto Member>
auto Scalar(std::string key, std::string label, std::string group, double lo,
            double hi, double step, int decimals, std::string gate = {})
    -> FoveonParameterSpec {
  FoveonParameterSpec spec;
  spec.key      = std::move(key);
  spec.label    = std::move(label);
  spec.group    = std::move(group);
  spec.kind     = FoveonControlKind::Scalar;
  spec.minimum  = lo;
  spec.maximum  = hi;
  spec.step     = step;
  spec.decimals = decimals;
  spec.gate     = std::move(gate);
  spec.set      = [](FoveonEditParams& p, double v) {
    using T = std::remove_cvref_t<decltype(p.*Member)>;
    if constexpr (std::is_integral_v<T>) {
      p.*Member = static_cast<T>(v >= 0.5 ? 1 : 0);
    } else {
      p.*Member = static_cast<T>(v);
    }
  };
  spec.get = [](const FoveonEditParams& p) -> double {
    return static_cast<double>(p.*Member);
  };
  return spec;
}

auto Toggle(std::string key, std::string label, std::string group,
            std::string gate = {}) -> FoveonParameterSpec {
  FoveonParameterSpec spec;
  spec.key      = std::move(key);
  spec.label    = std::move(label);
  spec.group    = std::move(group);
  spec.kind     = FoveonControlKind::Toggle;
  spec.minimum  = 0.0;
  spec.maximum  = 1.0;
  spec.step     = 1.0;
  spec.decimals = 0;
  spec.gate     = std::move(gate);
  return spec;
}

auto Combo(std::string key, std::string label, std::string group,
           std::vector<std::string> options) -> FoveonParameterSpec {
  FoveonParameterSpec spec;
  spec.key     = std::move(key);
  spec.label   = std::move(label);
  spec.group   = std::move(group);
  spec.kind    = FoveonControlKind::Combo;
  spec.decimals = 0;
  spec.options = std::move(options);
  return spec;
}

auto BuildSpecs() -> std::vector<FoveonParameterSpec> {
  std::vector<FoveonParameterSpec> s;
  // clang-format off: the table mirrors X3Lumina's panel order on purpose.
  // X3Lumina's tone rows: exposure 0.1 EV, the other four 0.05. This also
  // keeps the SPP six-parameter bridge numerically identical to the master
  // panel (the C bridge handles the SPP highlight sign inversion).
  s.push_back(Scalar<&FoveonEditParams::exposure>("exposure", "曝光", "tone", -2, 2, 0.1, 2));
  s.push_back(Scalar<&FoveonEditParams::contrast>("contrast", "对比度", "tone", -1, 1, 0.05, 2));
  s.push_back(Scalar<&FoveonEditParams::shadow>("shadow", "阴影", "tone", -1, 1, 0.05, 2));
  s.push_back(Scalar<&FoveonEditParams::highlight>("highlight", "高光", "tone", -1, 1, 0.05, 2));
  s.push_back(Scalar<&FoveonEditParams::saturation>("saturation", "饱和度", "tone", -1, 1, 0.05, 2));

  s.push_back(Scalar<&FoveonEditParams::detail>("detail", "细节", "detail", 0, 1, 0.1, 2));
  s.push_back(Scalar<&FoveonEditParams::sharpness>("sharpness", "锐度", "detail", 0, 2, 0.05, 2));

  // ---- Highlight panel, in Views.swift:2739-3575 order ----
  s.push_back(Scalar<&FoveonEditParams::hl_recover>("hl_recover", "高光恢复", "highlight", 0, 1, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::hl_keep>("hl_keep", "高光保留", "highlight", 0, 1, 0.01, 2));
  // Overexposure correction is a TWO-STATE switch (关闭 / mode8), never a
  // slider. X3Lumina serializes it as jpegHlFusion + oeAlgo, with m8od hidden.
  {
    auto oe = Combo("oe", "过曝矫正", "highlight", {"关闭", "mode8"});
    oe.gate = "quattro";
    s.push_back(std::move(oe));
  }
  s.push_back(Toggle("hole_fix", "高光去粉（hole-fix）", "highlight", "quattro"));
  s.push_back(Toggle("merrill_hl_neut", "过曝天空中性化", "highlight", "nonquattro"));
  s.push_back(Scalar<&FoveonEditParams::sdq_hl>("sdq_hl", "SDQ 高光", "highlight", 0, 2, 0.05, 2, "sdq"));
  s.push_back(Scalar<&FoveonEditParams::hl_chroma>("hl_chroma", "亮部保色", "highlight", 1, 2.4, 0.05, 2));
  s.push_back(Toggle("hl_chroma_hue_gate", "只作用于黄/橙", "highlight", "hl_chroma"));
  s.push_back(Toggle("oe_m8od", "mode8 输出域修正（隐藏）", "advanced", "quattro"));
  s.push_back(Toggle("hlp_fix", "HLP 网格修正（隐藏）", "advanced"));

  s.push_back(Scalar<&FoveonEditParams::wb_temp>("wb_temp", "色温", "wb", -1, 1, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::wb_tint>("wb_tint", "色调", "wb", -1, 1, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::wb_red>("wb_red", "红增益", "wb", 0, 2, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::wb_green>("wb_green", "绿增益", "wb", 0, 2, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::wb_blue>("wb_blue", "蓝增益", "wb", 0, 2, 0.01, 2));
  s.push_back(Combo("wb_preset", "白平衡预设", "wb",
                    {"", "Sunlight", "Shade", "Overcast", "Incandescent",
                     "Fluorescent", "Flash", "Auto", "Custom"}));

  s.push_back(Scalar<&FoveonEditParams::fill_light>("fill_light", "填充光", "fill", -2, 2, 0.1, 2));
  s.push_back(Scalar<&FoveonEditParams::fl_hlzred>("fl_hlzred", "高光压缩", "advanced", 0, 2, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::fl_lmean_r>("fl_lmean_r", "局部均值半径", "advanced", -2, 2, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::fl_msat>("fl_msat", "饱和度映射", "advanced", -2, 2, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::fl_mapgain>("fl_mapgain", "映射增益", "advanced", 0, 2, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::fl_k0off>("fl_k0off", "k0 偏移", "advanced", -2, 2, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::fl_zgain>("fl_zgain", "z 增益", "advanced", 0, 2, 0.01, 2));

  // Five SPP denoise positions in X3Lumina (0, .25, .5, .75, 1). In SPP mode
  // the C bridge maps them onto rows 2/3/3/4/4.
  s.push_back(Scalar<&FoveonEditParams::nr_luma>("nr_luma", "亮度降噪", "nr", 0, 1, 0.25, 2));
  s.push_back(Scalar<&FoveonEditParams::nr_chroma>("nr_chroma", "色度降噪", "nr", 0, 1, 0.2, 2));
  s.push_back(Scalar<&FoveonEditParams::nr_banding>("nr_banding", "条带抑制", "nr", 0, 1, 0.2, 2));
  s.push_back(Scalar<&FoveonEditParams::nr_q_base>("nr_q_base", "量化基线", "advanced", 0, 2, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::nr_q_gate_k>("nr_q_gate_k", "量化门限增益", "advanced", 0, 2, 0.01, 2));
  s.push_back(Toggle("nr_q_gate", "量化门限", "advanced"));
  s.push_back(Toggle("nr_q_baseline", "量化基线 V2", "advanced"));

  s.push_back(Toggle("mono", "黑白", "mono"));
  s.push_back(Scalar<&FoveonEditParams::mono_mix_r>("mono_mix_r", "混合 红", "mono", 0, 100, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::mono_mix_g>("mono_mix_g", "混合 绿", "mono", 0, 100, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::mono_mix_b>("mono_mix_b", "混合 蓝", "mono", 0, 100, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::mono_tone_r>("mono_tone_r", "色调 红", "mono", 0, 100, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::mono_tone_g>("mono_tone_g", "色调 绿", "mono", 0, 100, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::mono_tone_b>("mono_tone_b", "色调 蓝", "mono", 0, 100, 1, 0));
  s.push_back(Toggle("mono_grain", "颗粒", "mono"));
  s.push_back(Scalar<&FoveonEditParams::mono_grain_size>("mono_grain_size", "颗粒尺寸", "mono", 0, 1, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::mono_grain_roughness>("mono_grain_roughness", "颗粒粗糙度", "mono", 0, 1, 0.01, 2));

  s.push_back(Scalar<&FoveonEditParams::split_sh_hue>("split_sh_hue", "阴影 色相", "split", 0, 360, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::split_sh_sat>("split_sh_sat", "阴影 饱和", "split", 0, 100, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::split_hl_hue>("split_hl_hue", "高光 色相", "split", 0, 360, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::split_hl_sat>("split_hl_sat", "高光 饱和", "split", 0, 100, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::split_balance>("split_balance", "平衡", "split", -100, 100, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::split_blending>("split_blending", "混合", "split", 0, 100, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::glow>("glow", "辉光", "split", 0, 1, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::glow_radius>("glow_radius", "辉光半径", "split", 0, 1, 0.01, 2));
  s.push_back(Scalar<&FoveonEditParams::glow_threshold>("glow_threshold", "辉光阈值", "split", 0, 1, 0.01, 2));

  s.push_back(Combo("color_mode", "色彩模式", "color",
                    {"AsShot", "Standard", "Vivid", "Neutral", "Portrait",
                     "Landscape", "Sunset", "Monochrome", "FOV Classic",
                     "FOV Vivid", "FOV Neutral", "FOV Portrait"}));
  s.push_back(Scalar<&FoveonEditParams::color_strength>("color_strength", "模式强度", "color", 0, 100, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::sfd_tone>("sfd_tone", "SFD 色调表", "sfd", 0, 8, 1, 0));
  s.push_back(Scalar<&FoveonEditParams::frame>("frame", "X3I 帧", "sfd", 0, 6, 1, 0));
  // "后降噪" is the decode-stage NLM pass that runs after demosaic and before
  // the display chain — it is the only noise control that is not part of SPP's
  // own three sliders. Decode-level: changing it re-prepares the frame.
  s.push_back(Scalar<&FoveonEditParams::nlm>("nlm", "后降噪", "nr_post", 0, 10, 1, 0));
  // `down` is intentionally not exposed: preview resolution is derived from
  // the viewport long edge, while the sidecar value remains available for
  // full-resolution one-shot renders and export.
  // clang-format on

  // Toggle / Combo entries have no typed setter; bind them through the name.
  for (auto& spec : s) {
    if (spec.set) {
      continue;
    }
    spec.set = [key = spec.key](FoveonEditParams& p, double v) {
      (void)FoveonSetParam(p, key, v);
    };
    spec.get = [key = spec.key](const FoveonEditParams& p) -> double {
      return FoveonNumericParam(p, key);
    };
  }
  return s;
}

auto ReadDouble(const Json& obj, const char* key, double fallback) -> double {
  if (!obj.is_object() || !obj.contains(key)) {
    return fallback;
  }
  const auto& v = obj.at(key);
  if (v.is_number()) {
    return v.get<double>();
  }
  if (v.is_boolean()) {
    return v.get<bool>() ? 1.0 : 0.0;
  }
  return fallback;
}

auto ReadBool(const Json& obj, const char* key, bool fallback) -> bool {
  if (!obj.is_object() || !obj.contains(key)) {
    return fallback;
  }
  const auto& v = obj.at(key);
  if (v.is_boolean()) {
    return v.get<bool>();
  }
  if (v.is_number()) {
    return v.get<double>() != 0.0;
  }
  return fallback;
}

auto ReadString(const Json& obj, const char* key, std::string fallback) -> std::string {
  if (!obj.is_object() || !obj.contains(key) || !obj.at(key).is_string()) {
    return fallback;
  }
  return obj.at(key).get<std::string>();
}

}  // namespace

auto FoveonParameterSpecs() -> const std::vector<FoveonParameterSpec>& {
  static const std::vector<FoveonParameterSpec> specs = BuildSpecs();
  return specs;
}

auto FoveonNumericParam(const FoveonEditParams& params, std::string_view key)
    -> double {
  if (key == "oe") return static_cast<double>(params.oe);
  if (key == "hole_fix") return static_cast<double>(params.hole_fix);
  if (key == "merrill_hl_neut") return static_cast<double>(params.merrill_hl_neut);
  if (key == "hl_chroma_hue_gate") return static_cast<double>(params.hl_chroma_hue_gate);
  if (key == "oe_m8od") return static_cast<double>(params.oe_m8od);
  if (key == "hlp_fix") return static_cast<double>(params.hlp_fix);
  if (key == "nr_q_gate") return static_cast<double>(params.nr_q_gate);
  if (key == "nr_q_baseline") return static_cast<double>(params.nr_q_baseline);
  if (key == "mono") return static_cast<double>(params.mono);
  if (key == "mono_grain") return static_cast<double>(params.mono_grain);
  for (const auto& spec : FoveonParameterSpecs()) {
    if (spec.key == key && spec.get) {
      return spec.get(params);
    }
  }
  return 0.0;
}

auto FoveonSetParam(FoveonEditParams& params, std::string_view key, double value)
    -> bool {
  // Named-only fields (toggles / combo-as-index) are declared here; typed
  // fields are handled by the table's member-pointer setter.
  if (key == "oe") { params.oe = value >= 0.5 ? 1 : 0; return true; }
  if (key == "hole_fix") { params.hole_fix = value >= 0.5 ? 1 : 0; return true; }
  if (key == "merrill_hl_neut") { params.merrill_hl_neut = value >= 0.5 ? 1 : 0; return true; }
  if (key == "hl_chroma_hue_gate") { params.hl_chroma_hue_gate = value >= 0.5 ? 1 : 0; return true; }
  if (key == "oe_m8od") { params.oe_m8od = value >= 0.5 ? 1 : 0; return true; }
  if (key == "hlp_fix") { params.hlp_fix = value >= 0.5 ? 1 : 0; return true; }
  if (key == "nr_q_gate") { params.nr_q_gate = value >= 0.5 ? 1 : 0; return true; }
  if (key == "nr_q_baseline") { params.nr_q_baseline = value >= 0.5 ? 1 : 0; return true; }
  if (key == "mono") { params.mono = value >= 0.5 ? 1 : 0; return true; }
  if (key == "mono_grain") { params.mono_grain = value >= 0.5 ? 1 : 0; return true; }
  // Typed fields carry a member-pointer setter in the table. Toggle / combo
  // entries use the recursive adapter, so they must not be reached here.
  for (const auto& spec : FoveonParameterSpecs()) {
    if (spec.key == key && spec.kind == FoveonControlKind::Scalar && spec.set) {
      spec.set(params, value);
      return true;
    }
  }
  return false;
}

auto FoveonStringParam(const FoveonEditParams& params, std::string_view key)
    -> std::string {
  if (key == "wb_preset") return params.wb_preset;
  if (key == "color_mode") return params.color_mode;
  if (key == "oe") return params.oe ? "mode8" : "关闭";
  return {};
}

auto FoveonSetStringParam(FoveonEditParams& params, std::string_view key,
                          std::string_view value) -> bool {
  if (key == "wb_preset") { params.wb_preset = std::string(value); return true; }
  if (key == "color_mode") { params.color_mode = std::string(value); return true; }
  if (key == "oe") {
    params.oe = (value == "mode8" || value == "1" || value == "m8od") ? 1 : 0;
    if (value == "m8od") params.oe_m8od = 1;
    return true;
  }
  return false;
}

auto FoveonValuesJson(const FoveonEditParams& params) -> Json {
  Json values = Json::object();
  for (const auto& spec : FoveonParameterSpecs()) {
    if (spec.kind == FoveonControlKind::Combo) {
      values[spec.key] = FoveonStringParam(params, spec.key);
    } else if (spec.get) {
      values[spec.key] = spec.get(params);
    }
  }
  return values;
}

auto FoveonParametersJson(const FoveonEditParams& params) -> Json {
  Json out = Json::array();
  const auto values = FoveonValuesJson(params);
  for (const auto& spec : FoveonParameterSpecs()) {
    Json entry = Json::object();
    entry["key"]      = spec.key;
    entry["label"]    = spec.label;
    entry["group"]    = spec.group;
    entry["kind"]     = spec.kind == FoveonControlKind::Scalar ? "scalar"
                        : spec.kind == FoveonControlKind::Toggle ? "toggle"
                                                                 : "combo";
    entry["minimum"]  = spec.minimum;
    entry["maximum"]  = spec.maximum;
    entry["step"]     = spec.step;
    entry["decimals"] = spec.decimals;
    entry["options"]  = spec.options;
    entry["gate"]     = spec.gate;
    entry["value"]    = values.value(spec.key, Json(0.0));
    out.push_back(std::move(entry));
  }
  return out;
}

auto FoveonToSidecarJson(const FoveonEditParams& p) -> Json {
  Json raw = Json::object();
  raw["exposure"]       = p.exposure;
  raw["contrast"]       = p.contrast;
  raw["shadow"]         = p.shadow;
  raw["highlight"]      = p.highlight;
  raw["saturation"]     = p.saturation;
  raw["detail"]         = p.detail;
  raw["sharpness"]      = p.sharpness;
  raw["sharpRes"]       = p.sharp_res;
  raw["tcSh"] = p.tc[0]; raw["tcDk"] = p.tc[1];
  raw["tcLi"] = p.tc[2]; raw["tcHi"] = p.tc[3];
  raw["tcR2"] = p.tc[4]; raw["tcR3"] = p.tc[5]; raw["tcR4"] = p.tc[6];

  raw["temp"]           = p.wb_temp;
  raw["tint"]           = p.wb_tint;
  raw["wbRed"]          = p.wb_red;
  raw["wbGreen"]        = p.wb_green;
  raw["wbBlue"]         = p.wb_blue;
  raw["wbPreset"]       = p.wb_preset;

  // Model.swift:639-670 / Views.swift:3327-3355. The two-state picker is
  // serialized as jpegHlFusion>0 + oeAlgo 8 (mode8) or 7 (off). m8od is a
  // hidden legacy option kept readable for existing sidecars.
  raw["jpegHlFusion"]   = p.oe ? 1 : 0;
  raw["oeAlgo"]         = p.oe ? 8 : 7;
  raw["oeM8od"]         = p.oe_m8od != 0;
  raw["hlRecover"]      = p.hl_recover;
  raw["hlKeep"]         = p.hl_keep;
  raw["holeFix"]        = p.hole_fix != 0;
  raw["merrillHlNeut"]  = p.merrill_hl_neut != 0;
  raw["hlpFix"]         = p.hlp_fix != 0;
  raw["sdqHl"]          = p.sdq_hl;
  raw["hlChroma"]       = p.hl_chroma;
  raw["hlChromaHueGate"] = p.hl_chroma_hue_gate != 0;

  raw["fillLight"]      = p.fill_light;
  raw["flHlzred"]       = p.fl_hlzred;
  raw["flLmeanR"]       = p.fl_lmean_r;
  raw["flMsat"]         = p.fl_msat;
  raw["flMapgain"]      = p.fl_mapgain;
  raw["flK0Off"]        = p.fl_k0off;
  raw["flZgain"]        = p.fl_zgain;
  raw["flTune"]         = p.fl_tune != 0;

  raw["nrLuma"]         = p.nr_luma;
  raw["nrChroma"]       = p.nr_chroma;
  raw["nrBanding"]      = p.nr_banding;
  raw["nrQBase"]        = p.nr_q_base;
  raw["nrQBaselineV2"]  = p.nr_q_baseline != 0;
  raw["nrQGate"]        = p.nr_q_gate != 0;
  raw["nrQGateK"]       = p.nr_q_gate_k;

  raw["mono"]               = p.mono != 0;
  raw["monoMixR"]           = p.mono_mix_r;
  raw["monoMixG"]           = p.mono_mix_g;
  raw["monoMixB"]           = p.mono_mix_b;
  raw["monoToneR"]          = p.mono_tone_r;
  raw["monoToneG"]          = p.mono_tone_g;
  raw["monoToneB"]          = p.mono_tone_b;
  raw["monoGrain"]          = p.mono_grain != 0;
  raw["monoGrainSize"]      = p.mono_grain_size;
  raw["monoGrainRoughness"] = p.mono_grain_roughness;

  raw["splitShadowHue"]     = p.split_sh_hue;
  raw["splitShadowSat"]     = p.split_sh_sat;
  raw["splitHighlightHue"]  = p.split_hl_hue;
  raw["splitHighlightSat"]  = p.split_hl_sat;
  raw["splitBalance"]       = p.split_balance;
  raw["splitBlending"]      = p.split_blending;
  raw["glow"]               = p.glow;
  raw["glowRadius"]         = p.glow_radius;
  raw["glowThreshold"]      = p.glow_threshold;

  raw["colorMode"]          = p.color_mode;
  raw["colorStrength"]      = p.color_strength;
  raw["sfdTone"]            = p.sfd_tone;
  raw["frame"]              = p.frame;
  raw["down"]               = p.down;

  Json doc = Json::object();
  doc["raw"] = std::move(raw);
  doc["nlm"] = p.nlm;
  return doc;
}

auto FoveonFromSidecarJson(const Json& document, FoveonEditParams base)
    -> FoveonEditParams {
  if (!document.is_object()) {
    return base;
  }
  const Json raw = document.value("raw", Json::object());
  FoveonEditParams p = std::move(base);
  const auto d = [&raw](const char* key, double fallback) {
    return ReadDouble(raw, key, fallback);
  };
  const auto b = [&raw](const char* key, bool fallback) {
    return ReadBool(raw, key, fallback);
  };
  const auto s = [&raw](const char* key, std::string fallback) {
    return ReadString(raw, key, std::move(fallback));
  };

  p.exposure   = d("exposure", p.exposure);
  p.contrast   = d("contrast", p.contrast);
  p.shadow     = d("shadow", p.shadow);
  p.highlight  = d("highlight", p.highlight);
  p.saturation = d("saturation", p.saturation);
  p.detail     = d("detail", p.detail);
  p.sharpness  = d("sharpness", p.sharpness);
  p.sharp_res  = d("sharpRes", p.sharp_res);
  p.tc[0] = d("tcSh", p.tc[0]); p.tc[1] = d("tcDk", p.tc[1]);
  p.tc[2] = d("tcLi", p.tc[2]); p.tc[3] = d("tcHi", p.tc[3]);
  p.tc[4] = d("tcR2", p.tc[4]); p.tc[5] = d("tcR3", p.tc[5]);
  p.tc[6] = d("tcR4", p.tc[6]);

  p.wb_temp  = d("temp", p.wb_temp);
  p.wb_tint  = d("tint", p.wb_tint);
  p.wb_red   = d("wbRed", p.wb_red);
  p.wb_green = d("wbGreen", p.wb_green);
  p.wb_blue  = d("wbBlue", p.wb_blue);
  p.wb_preset = s("wbPreset", p.wb_preset);

  // Only mode8 (oeAlgo 8/9/10 + fusion>0) maps to our ON state. Legacy algo 7
  // sidecars are normalized to "off" because the product picker offers only
  // 关闭 / mode8 (Views.swift:3449).
  const double fusion  = d("jpegHlFusion", p.oe ? 1.0 : 0.0);
  const int    oe_algo = static_cast<int>(d("oeAlgo", p.oe ? 8.0 : 7.0));
  p.oe = (fusion > 0.0 && (oe_algo == 8 || oe_algo == 9 || oe_algo == 10)) ? 1 : 0;
  p.oe_m8od = b("oeM8od", p.oe_m8od != 0) ? 1 : 0;
  p.hl_recover          = d("hlRecover", p.hl_recover);
  p.hl_keep             = d("hlKeep", p.hl_keep);
  p.hole_fix            = b("holeFix", p.hole_fix != 0) ? 1 : 0;
  p.merrill_hl_neut     = b("merrillHlNeut", p.merrill_hl_neut != 0) ? 1 : 0;
  p.hlp_fix             = b("hlpFix", p.hlp_fix != 0) ? 1 : 0;
  p.sdq_hl              = d("sdqHl", p.sdq_hl);
  p.hl_chroma           = d("hlChroma", p.hl_chroma);
  p.hl_chroma_hue_gate  = b("hlChromaHueGate", p.hl_chroma_hue_gate != 0) ? 1 : 0;

  p.fill_light = d("fillLight", p.fill_light);
  p.fl_hlzred  = d("flHlzred", p.fl_hlzred);
  p.fl_lmean_r = d("flLmeanR", p.fl_lmean_r);
  p.fl_msat    = d("flMsat", p.fl_msat);
  p.fl_mapgain = d("flMapgain", p.fl_mapgain);
  p.fl_k0off   = d("flK0Off", p.fl_k0off);
  p.fl_zgain   = d("flZgain", p.fl_zgain);
  p.fl_tune    = b("flTune", p.fl_tune != 0) ? 1 : 0;

  p.nr_luma       = d("nrLuma", p.nr_luma);
  p.nr_chroma     = d("nrChroma", p.nr_chroma);
  p.nr_banding    = d("nrBanding", p.nr_banding);
  p.nr_q_base     = d("nrQBase", p.nr_q_base);
  p.nr_q_baseline = b("nrQBaselineV2", p.nr_q_baseline != 0) ? 1 : 0;
  p.nr_q_gate     = b("nrQGate", p.nr_q_gate != 0) ? 1 : 0;
  p.nr_q_gate_k   = d("nrQGateK", p.nr_q_gate_k);

  p.mono                 = b("mono", p.mono != 0) ? 1 : 0;
  p.mono_mix_r           = d("monoMixR", p.mono_mix_r);
  p.mono_mix_g           = d("monoMixG", p.mono_mix_g);
  p.mono_mix_b           = d("monoMixB", p.mono_mix_b);
  p.mono_tone_r          = d("monoToneR", p.mono_tone_r);
  p.mono_tone_g          = d("monoToneG", p.mono_tone_g);
  p.mono_tone_b          = d("monoToneB", p.mono_tone_b);
  p.mono_grain           = b("monoGrain", p.mono_grain != 0) ? 1 : 0;
  p.mono_grain_size      = d("monoGrainSize", p.mono_grain_size);
  p.mono_grain_roughness = d("monoGrainRoughness", p.mono_grain_roughness);

  p.split_sh_hue   = d("splitShadowHue", p.split_sh_hue);
  p.split_sh_sat   = d("splitShadowSat", p.split_sh_sat);
  p.split_hl_hue   = d("splitHighlightHue", p.split_hl_hue);
  p.split_hl_sat   = d("splitHighlightSat", p.split_hl_sat);
  p.split_balance  = d("splitBalance", p.split_balance);
  p.split_blending = d("splitBlending", p.split_blending);
  p.glow           = d("glow", p.glow);
  p.glow_radius    = d("glowRadius", p.glow_radius);
  p.glow_threshold = d("glowThreshold", p.glow_threshold);

  p.color_mode     = s("colorMode", p.color_mode);
  p.color_strength = d("colorStrength", p.color_strength);
  p.sfd_tone       = static_cast<int>(d("sfdTone", p.sfd_tone));
  p.frame          = static_cast<int>(d("frame", p.frame));
  p.down           = std::max(1, static_cast<int>(d("down", p.down)));
  p.nlm            = static_cast<int>(ReadDouble(document, "nlm", p.nlm));
  return p;
}

auto FoveonSidecarPath(const std::filesystem::path& source)
    -> std::filesystem::path {
  auto path = source;
  path += ".x3fedit.json";
  return path;
}

auto LoadFoveonSidecar(const std::filesystem::path& source,
                       FoveonEditParams* params) -> bool {
  if (params == nullptr) {
    return false;
  }
  const auto path = FoveonSidecarPath(source);
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    return false;
  }
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  try {
    Json document = Json::parse(in);
    *params = FoveonFromSidecarJson(document, *params);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

auto SaveFoveonSidecar(const std::filesystem::path& source,
                       const FoveonEditParams& params) -> bool {
  const auto path = FoveonSidecarPath(source);
  Json document = Json::object();
  {
    std::error_code ec;
    if (std::filesystem::exists(path, ec)) {
      std::ifstream in(path);
      if (in) {
        try {
          document = Json::parse(in);
        } catch (const std::exception&) {
          return false;  // never overwrite an unparsable user sidecar
        }
      }
    }
  }
  if (!document.is_object()) {
    document = Json::object();
  }
  if (!document.contains("raw") || !document["raw"].is_object()) {
    document["raw"] = Json::object();
  }
  const Json patch = FoveonToSidecarJson(params);
  // Merge, never replace: Fotufilm / SpektraLab / film / HDR / HSL keys belong
  // to the main application and must survive a Studio save.
  for (const auto& [key, value] : patch.at("raw").items()) {
    document["raw"][key] = value;
  }
  document["nlm"]    = patch.at("nlm");
  document["version"] = document.value("version", 1);
  std::error_code ec;
  document["sourceSize"] = static_cast<std::uint64_t>(
      std::filesystem::file_size(source, ec));
  if (ec) {
    document.erase("sourceSize");
  }

  auto temp = path;
  temp += ".tmp";
  {
    std::ofstream out(temp, std::ios::trunc);
    if (!out) {
      return false;
    }
    out << document.dump(2, ' ', false, Json::error_handler_t::replace);
    if (!out) {
      std::filesystem::remove(temp, ec);
      return false;
    }
  }
  std::filesystem::remove(path, ec);
  std::filesystem::rename(temp, path, ec);
  if (ec) {
    std::filesystem::remove(temp, ec);
    return false;
  }
  return true;
}

}  // namespace alcedo::x3f
