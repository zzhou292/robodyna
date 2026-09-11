// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../solvers/NodalNativePhysicalCoefficients.h"

namespace tl::fea::shell_activity_detail {
// Shared host preflight only. Each family still validates its complete typed
// history through its existing owning readback before publishing any flag.
template<class State, class Batch, class Diagnostics>
auto PreparedPreflight(State& state, const Batch& batch, FENodalState& owner,
    const NodalTrialToken& token, const Diagnostics& expected, bool same_diagnostics,
    std::uint8_t* output, std::size_t capacity) -> decltype(state.PendingError()) {
  using Report = decltype(state.PendingError());
  using Status = decltype(Report{}.status);
  if (!state.usable) return {Status::DeviceFailure, "Shell activity storage is poisoned"};
  if (!state.bound) return {Status::NotBound, "Shell initial sources are not bound"};
  if (capacity != state.config.element_count) {
    return {Status::ResourceLimit, "Prepared activity requires exact complete family capacity"};
  }
  if (!state.pending || !same_diagnostics) {
    return {Status::StaleTrial, "Prepared activity requires this family's complete pending candidate"};
  }
  using trial_identity::Disjoint;
  if (!state.OutputDisjoint(output, capacity) ||
      !Disjoint(output, capacity, &batch, sizeof(batch)) ||
      !Disjoint(output, capacity, &owner, sizeof(owner)) ||
      !Disjoint(output, capacity, &token, sizeof(token)) ||
      !Disjoint(output, capacity, &expected, sizeof(expected))) {
    return {Status::InvalidInput, "Prepared activity output aliases input or retained source/state"};
  }
  const auto authenticated = native_physical_coefficients::AuthenticatePrepared(
      owner, token, state.accepted_stamp, state.candidate_view);
  if (authenticated.status != NodalStatus::Ok) {
    if (authenticated.status == NodalStatus::DeviceFailure) state.usable = false;
    return {authenticated.status == NodalStatus::DeviceFailure ? Status::DeviceFailure : Status::StaleTrial,
            authenticated.message};
  }
  return {Status::Success, "Prepared activity owner/token and complete output are authenticated"};
}
} // namespace tl::fea::shell_activity_detail
