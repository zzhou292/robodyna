// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Input.h"
#include "../NodalNodeStep.h"
namespace tl::fea::cin_advance {
TL_SURFACE_HD inline NodalStatus AdvanceNode(const Input& input, std::uint32_t i) {
  const auto n = input.model.node_count;
  auto* current_inverse = input.tail+2*n;
  // There is no conventional inverse/kick for a dependent CIN DOF.
  if (input.model.dependent_nodes[i]) {
    current_inverse[i] = 0;
    current_inverse[n+i] = 0;
    return NodalStatus::Ok;
  }
  const auto& groups = input.groups;
  const auto* tail = input.tail;
  const auto* fixed = input.fixed;
  const auto* rotation_present = input.rotation_present;
  const bool rigid_dependent = groups.member_nodes && rigid::UsesDependentCoefficients(groups.member_nodes[i]);
  current_inverse[i] = fixed[n+i] == 7 || (rigid_dependent && tail[i] == 0) ? 0 : 1/tail[i];
  current_inverse[n+i] = fixed[2*n+i] || (rotation_present && !rotation_present[i]) ||
      (rigid_dependent && tail[n+i] == 0) ? 0 : 1/tail[n+i];
  if (!std::isfinite(current_inverse[i]) || !std::isfinite(current_inverse[n+i])) {
    return NodalStatus::InvalidOutput;
  }
  if (groups.member_nodes && groups.member_nodes[i]) return NodalStatus::Ok;
  return nodal_detail::AdvanceOrdinaryNode<true>(input.accepted, input.trial, input.loads,
    current_inverse, fixed, i, n, input.durations.drift_dt, input.durations.kick_dt, input.maximum_angle,
    input.work+3*n, input.work+6*n, rotation_present);
}
} // namespace tl::fea::cin_advance
