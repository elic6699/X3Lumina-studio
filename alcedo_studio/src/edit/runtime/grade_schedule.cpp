//  Copyright 2026 Yurun Zi
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "edit/runtime/grade_schedule.hpp"

#include <algorithm>
#include <stdexcept>

namespace alcedo {

auto CompactGradeOps(std::vector<GradeScheduledOp> ops, bool local_tone_active)
    -> std::vector<GradeScheduledOp> {
  std::vector<GradeScheduledOp> compacted;
  compacted.reserve(ops.size());
  for (auto& op : ops) {
    if (op.kind == CompiledGradeStageKind::LocalLaplacian && !local_tone_active) {
      continue;
    }
    if (op.kind == CompiledGradeStageKind::Pointwise && op.fused_offsets.empty()) {
      continue;
    }
    if (op.kind == CompiledGradeStageKind::Pointwise && !compacted.empty() &&
        compacted.back().kind == CompiledGradeStageKind::Pointwise) {
      compacted.back().fused_offsets.insert(compacted.back().fused_offsets.end(),
                                            op.fused_offsets.begin(), op.fused_offsets.end());
      continue;
    }
    compacted.push_back(std::move(op));
  }
  return compacted;
}

auto CountGradeGpuWrites(const std::vector<GradeScheduledOp>& ops, bool skip_final_mix)
    -> std::size_t {
  (void)skip_final_mix;
  std::size_t count = 0;
  for (const auto& op : ops) {
    if (op.kind == CompiledGradeStageKind::Pointwise ||
        op.kind == CompiledGradeStageKind::Neighborhood ||
        op.kind == CompiledGradeStageKind::LocalLaplacian) {
      ++count;
    }
  }
  return count;
}

auto GradeWriteSlots(std::size_t write_count) -> std::vector<GradeImageSlot> {
  std::vector<GradeImageSlot> slots;
  slots.reserve(write_count);
  for (std::size_t index = 0; index < write_count; ++index) {
    if (index + 1 == write_count) {
      slots.push_back(GradeImageSlot::Output);
      continue;
    }
    slots.push_back(index % 2 == 0 ? GradeImageSlot::Ping : GradeImageSlot::Pong);
  }
  return slots;
}

auto MakeGradeSchedule(std::span<const CompiledGradeStage> stages,
                       std::span<const GradeScheduleInput> bound, float mix, bool has_mask)
    -> GradeSchedule {
  GradeSchedule schedule;
  if (mix == 0.0f) {
    schedule.alias_to_input = true;
    return schedule;
  }

  std::vector<GradeScheduledOp> ops;
  ops.reserve(stages.size());
  for (const auto& stage : stages) {
    const auto begin = static_cast<std::size_t>(stage.begin);
    const auto end   = begin + stage.count;
    if (end > bound.size()) {
      throw std::runtime_error("MakeGradeSchedule: compiled grade stage is out of range");
    }
    switch (stage.kind) {
      case CompiledGradeStageKind::Pointwise: {
        GradeScheduledOp op;
        op.kind = CompiledGradeStageKind::Pointwise;
        op.fused_offsets.reserve(stage.count);
        for (std::size_t index = begin; index < end; ++index) {
          op.fused_offsets.push_back(bound[index].fused_offset);
        }
        ops.push_back(std::move(op));
        break;
      }
      case CompiledGradeStageKind::LocalLaplacian: {
        float fill_light = 0.0f;
        for (std::size_t index = begin; index < end; ++index) {
          if (bound[index].behavior == AdjustmentBehavior::Shadows) {
            schedule.shadows_slider = bound[index].llf_control;
          } else if (bound[index].behavior == AdjustmentBehavior::Highlights) {
            schedule.highlights_slider = bound[index].llf_control;
          } else if (bound[index].behavior == AdjustmentBehavior::FillLight) {
            fill_light = bound[index].llf_control;
          }
        }
        // Fill light rides the same LLF: it is a shadow lift measured against a
        // local luminance map, which is exactly what the LocalLaplacian stage
        // does, and the reason it works on every input instead of only on the
        // Foveon decode. +1 (the SPP panel's midpoint) maps to a +60 shadow
        // slider, and the highlights give back a little so the whole frame does
        // not drift brighter. Zero leaves both sliders byte-identical.
        if (fill_light != 0.0f) {
          schedule.shadows_slider =
              std::clamp(schedule.shadows_slider + fill_light * 60.0f, -100.0f, 150.0f);
          schedule.highlights_slider =
              std::clamp(schedule.highlights_slider - fill_light * 8.0f, -150.0f, 150.0f);
        }
        if (ops.empty() || ops.back().kind != CompiledGradeStageKind::LocalLaplacian) {
          ops.push_back(GradeScheduledOp{CompiledGradeStageKind::LocalLaplacian, {}, {}});
        }
        break;
      }
      case CompiledGradeStageKind::Neighborhood: {
        for (std::size_t index = begin; index < end; ++index) {
          if (bound[index].neighbor.enabled == 0U) {
            continue;
          }
          GradeScheduledOp detail;
          detail.kind          = CompiledGradeStageKind::Neighborhood;
          detail.fused_offsets = {bound[index].fused_offset};
          detail.neighbor      = bound[index].neighbor;
          ops.push_back(std::move(detail));
        }
        break;
      }
    }
  }

  schedule.local_tone_active = local_tone_mapping::ShouldRun(
      schedule.shadows_slider * local_tone_mapping::kHighlightStrengthScale / 80.0f,
      -schedule.highlights_slider * local_tone_mapping::kHighlightStrengthScale / 100.0f);
  schedule.ops            = CompactGradeOps(std::move(ops), schedule.local_tone_active);
  schedule.skip_final_mix = mix == 1.0f && !has_mask;
  if (schedule.ops.empty()) {
    schedule.alias_to_input  = true;
    schedule.skip_final_mix  = false;
    schedule.gpu_write_count = 0;
    return schedule;
  }
  schedule.gpu_write_count = CountGradeGpuWrites(schedule.ops, schedule.skip_final_mix);
  return schedule;
}

auto MakeGradeDecisionTrace(const GradeSchedule& schedule) -> GradeDecisionTrace {
  GradeDecisionTrace trace;
  trace.alias_to_input    = schedule.alias_to_input;
  trace.skip_final_mix    = schedule.skip_final_mix;
  trace.local_tone_active = schedule.local_tone_active;
  trace.gpu_write_count   = schedule.gpu_write_count;
  trace.op_kinds.reserve(schedule.ops.size());
  for (const auto& op : schedule.ops) {
    trace.op_kinds.push_back(op.kind);
    trace.fused_command_count += op.fused_offsets.size();
  }
  return trace;
}

}  // namespace alcedo
