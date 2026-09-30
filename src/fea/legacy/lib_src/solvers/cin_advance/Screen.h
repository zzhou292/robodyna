// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Input.h"
#include "ScreenSummary.h"

namespace tl::fea::cin_advance::screen {
TL_SURFACE_HD inline cin_timestep::Sources Sources(const Input& input) noexcept {
  const auto n = input.model.node_count;
  return {input.accepted, input.tail, input.tail+n, input.work, input.work+n,
      input.fixed+n, input.fixed+2*n, input.rotation_present, input.model.dependent_nodes,
      input.groups, n, input.durations.previous_drift_dt};
}
cudaError_t Launch(const Input&, cudaStream_t);
} // namespace tl::fea::cin_advance::screen
