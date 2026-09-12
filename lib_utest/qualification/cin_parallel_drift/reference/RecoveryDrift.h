// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Input.h"
#include "../NodalNodeStep.h"

namespace tl::fea::cin_advance::recovery {
TL_SURFACE_HD inline void Fail(Input input, NodalStatus status, std::uint32_t node) noexcept {
  input.control->status = status;
  input.control->node = node;
  if (status == NodalStatus::StepTooLarge) input.control->limit.dt = 0;
}
// The entire original ordered dependent drift/orientation suffix. Recovery of
// every row must succeed before any of these writes are allowed.
TL_SURFACE_HD inline void Drift(Input input) noexcept {
  const auto model = input.model;
  const auto n = model.node_count;
  const auto r = model.row_count;
  const auto* accepted = input.accepted;
  auto* trial = input.trial;
  const auto durations = input.durations;
  const auto maximum_angle = input.maximum_angle;
  for (std::uint32_t row = 0; row < r; ++row) {
    const auto node = model.rows[row].secondary;
    for (unsigned a = 0; a < 3; ++a) {
      const auto j = 3*node+a;
      trial[j] = accepted[j]+durations.drift_dt*trial[3*n+j];
      // A dependent is not a prescribed fixed DOF. Its transfer is represented
      // by the retained native loads, not a fabricated fixed reaction.
      trial[13*n+j] = 0;
      trial[16*n+j] = 0;
      if (!std::isfinite(trial[j])) {
        Fail(input, NodalStatus::InvalidOutput, node);
        return;
      }
    }
    const auto status = nodal_detail::PrepareNodeOrientation(accepted, trial, node, n,
        durations.drift_dt, maximum_angle, false);
    if (status != NodalStatus::Ok) {
      Fail(input, status, node);
      return;
    }
  }
}
} // namespace tl::fea::cin_advance::recovery
