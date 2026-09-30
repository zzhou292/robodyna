// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OperationChecks.h"

namespace tl::fea::beam18 {
BatchReport Batch::AssembleAccepted(FENodalState& owner, const NodalTrialToken& token,
    const NodalAssemblyView& view) {
  if (!impl_) return {BatchStatus::NotInitialized, "Beam18 batch is not initialized"};
  auto& state = *impl_;
  state.Discard();
  auto report = state.PendingError();
  if (!report) return report;
  if (!state.bound || !state.publication_scope)
    return {BatchStatus::NotBound, "Beam18 common publication claim required"};
  const auto& stamp = state.accepted_stamp;
  const auto authenticated = native_physical_coefficients::AuthenticateAccepted(owner, stamp, view);
  if (authenticated.status != NodalStatus::Ok) {
    if (authenticated.status == NodalStatus::DeviceFailure) state.usable = false;
    return batch_detail::NodalFailure(authenticated);
  }
  if (!batch_detail::CompleteAssembly(view, stamp) || view.owner_id != stamp.owner_id ||
      view.temporal_scheme != stamp.temporal_scheme || view.velocity_phase != stamp.velocity_phase ||
      view.position_time != stamp.time || view.velocity_time != stamp.velocity_time ||
      !view.attempt || view.attempt <= state.assembled_attempt ||
      (state.assembled_attempt && view.stream != state.stream)) {
    owner.Discard();
    return {BatchStatus::StaleTrial, "Beam18 assembly source phase or attempt differs"};
  }
  NodalCinAssemblyView cin;
  const auto borrowed = shell_physical_owner::BorrowAssembly(owner, token, stamp, view,
      state.config.cin_witness_count, &cin);
  if (borrowed.status != NodalStatus::Ok) {
    if (borrowed.status == NodalStatus::DeviceFailure) state.usable = false;
    owner.Discard();
    return batch_detail::NodalFailure(borrowed);
  }
  // No device write is permitted after a failed borrow above.
  state.assembled_attempt = view.attempt;
  state.assembled_epoch = UINT64_MAX;
  state.stream = view.stream;
  batch_detail::LaunchAssembly(state.device, state.accepted_slab, view, cin);
  report = state.ReadControl();
  if (!report) {
    owner.Discard();
    return report;
  }
  state.assembled_epoch = stamp.epoch;
  return {};
}
} // namespace tl::fea::beam18
