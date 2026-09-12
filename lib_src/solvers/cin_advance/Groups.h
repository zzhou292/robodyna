// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Input.h"
#include "../../constraints/NodalRigidGroupCandidate.h"
#include "../NodalNodeStep.h"

namespace tl::fea::cin_advance::groups {
TL_SURFACE_HD inline Report ScreenGroup(const cin_timestep::Sources& source,
    double factor, std::uint32_t group) noexcept {
  const auto value = cin_timestep::detail::EvaluateGroup(source, factor, group);
  return {value.limit.dt, value.first_node, value.last_node,
      value.valid ? NodalStatus::Ok : NodalStatus::InvalidOutput,
      value.visited, value.limit.bounded};
}
TL_SURFACE_HD inline bool CompleteScreen(const cin_timestep::Sources& source,
    const screen::Summary& ordinary, const Report* reports,
    cin_timestep::Result& output, std::uint32_t& invalid_node) noexcept {
  if (ordinary.invalid_node != UINT32_MAX) {
    invalid_node = ordinary.invalid_node;
    return false;
  }
  cin_timestep::Result next;
  next.minimum_dt = ordinary.minimum_dt;
  next.limiting_node = ordinary.limiting_node;
  invalid_node = source.nodes-1;
  for (std::uint32_t group = 0; group < source.rigid.group_count; ++group) {
    const auto& value = reports[group];
    if (value.visited) invalid_node = value.last_node;
    if (value.status != NodalStatus::Ok) return false;
    cin_timestep::ScalarLimit limit;
    limit.dt = value.minimum_dt;
    limit.bounded = value.bounded;
    cin_timestep::Include(limit, value.first_node, group, next);
  }
  next.valid = true;
  invalid_node = UINT32_MAX;
  output = next;
  return true;
}
TL_SURFACE_HD inline Report AdvanceGroup(const Input& input, std::uint32_t group) {
  Report out{};
  out.first_node = out.last_node = UINT32_MAX;
  const auto result = input.capture.node
      ? rigid::PrepareGroupCandidate<true>(input.groups, group, input.accepted,
          input.trial, input.loads, input.model.node_count, input.durations, input.capture)
      : rigid::PrepareGroupCandidate<false>(input.groups, group, input.accepted,
          input.trial, input.loads, input.model.node_count, input.durations);
  if (result.status != rigid::StepStatus::Success) {
    out.status = result.status == rigid::StepStatus::RotationLimit
        ? NodalStatus::StepTooLarge : NodalStatus::InvalidOutput;
    out.last_node = result.node;
    return out;
  }
  const auto range = input.groups.groups[group];
  for (std::uint32_t local = 0; local < range.count; ++local) {
    const auto node = input.groups.members[range.offset+local].node;
    out.status = nodal_detail::PrepareNodeOrientation(input.accepted, input.trial,
        node, input.model.node_count, input.durations.drift_dt, input.maximum_angle, false);
    if (out.status != NodalStatus::Ok) {
      out.last_node = node;
      return out;
    }
  }
  return out;
}
cudaError_t LaunchMotion(const Input&, cudaStream_t);
} // namespace tl::fea::cin_advance::groups
