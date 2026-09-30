// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "RecoveryDrift.h"

namespace tl::fea::cin_advance::drift {
// CIN admission proves unique secondaries. No leaf reads another row's
// secondary, a reaction or any position that this phase will publish.
TL_SURFACE_HD inline Row Prepare(Input input, std::uint32_t row) noexcept {
  Row result;
  const auto n = input.model.node_count;
  const auto node = input.model.rows[row].secondary;
  for (unsigned axis = 0; axis < 3; ++axis) {
    const auto index = 3*node+axis;
    result.position[axis] = input.accepted[index]+
        input.durations.drift_dt*input.trial[3*n+index];
    result.stored_axes = axis+1;
    if (!std::isfinite(result.position[axis])) {
      result.status = NodalStatus::InvalidOutput;
      return result;
    }
  }
  result.status = nodal_detail::PrepareNodeOrientationValue(input.accepted,
      input.trial, node, n, input.durations.drift_dt, input.maximum_angle,
      false, result.orientation);
  return result;
}

TL_SURFACE_HD inline void Publish(Input input, std::uint32_t row,
    recovery::FailureRow first_failure) noexcept {
  if (row > first_failure) return;
  const auto& result = input.prepared_drift[row];
  const auto n = input.model.node_count;
  const auto node = input.model.rows[row].secondary;
  for (unsigned axis = 0; axis < result.stored_axes; ++axis) {
    const auto index = 3*node+axis;
    input.trial[index] = result.position[axis];
    input.trial[13*n+index] = 0;
    input.trial[16*n+index] = 0;
  }
  if (result.status != NodalStatus::Ok) return;
  auto* quaternion = input.trial+9*n+4*node;
  quaternion[0] = result.orientation.w;
  quaternion[1] = result.orientation.x;
  quaternion[2] = result.orientation.y;
  quaternion[3] = result.orientation.z;
}

TL_SURFACE_HD inline void Complete(Input input) noexcept {
  if (input.control->status != NodalStatus::Ok) return;
  const auto failed = *input.recovery_failure;
  if (failed != recovery::NoFailure) {
    recovery::Fail(input, input.prepared_drift[failed].status,
        input.model.rows[failed].secondary);
  }
}
// Internal recovery continuation: positive admitted row_count, counted row tail
// and recovery key are required. The caller retains the null-tail serial route.
cudaError_t Launch(const Input&, cudaStream_t);
} // namespace tl::fea::cin_advance::drift
