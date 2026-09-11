// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OperationChecks.h"

namespace tl::fea::type13 {
BatchReport Batch::PreflightPublication(FENodalState& owner, const NodalTrialToken& token,
                                        const NodalPreparedView& prepared,
                                        const BatchDiagnostics& expected,
                                        const ShellBatchPublication* claimant) const noexcept {
  if (!impl_) {
    return {BatchStatus::NotInitialized, "TYPE13 batch is not initialized"};
  }
  const auto& state = *impl_;
  if (!state.usable) {
    return {BatchStatus::Unusable, "TYPE13 CUDA storage is poisoned"};
  }
  if ((state.physical && state.physical->owner != &owner) ||
      !claimant || state.publication_scope != claimant || !state.bound || !state.pending ||
      !batch_detail::SameDiagnostics(expected, state.candidate_diagnostics) ||
      !trial_identity::SamePrepared(prepared, state.candidate_view)) {
    return {BatchStatus::StaleTrial, "TYPE13 pending publication identity differs"};
  }
  const auto authenticated = native_physical_coefficients::AuthenticatePrepared(
      owner, token, state.accepted_stamp, prepared);
  if (authenticated.status != NodalStatus::Ok) {
    return batch_detail::NodalFailure(authenticated);
  }
  return {};
}
void Batch::Publish(const NodalStamp& stamp) noexcept {
  // Common coordinator only, AFTER every participant and the sole nodal owner
  // commit succeeds. No CUDA, allocation, callbacks or fallible work here.
  auto& state = *impl_;
  state.accepted_slab = state.TrialSlab();
  state.accepted_stamp = stamp;
  state.accepted_diagnostics = state.candidate_diagnostics;
  state.accepted_diagnostics.phase = BatchPhase::Accepted;
  state.Discard();
}
BatchReport Batch::CopyAcceptedDiagnostics(const NodalStamp& expected,
                                          BatchDiagnostics* output) const noexcept {
  if (!impl_) {
    return {BatchStatus::NotInitialized, "TYPE13 batch is not initialized"};
  }
  const auto& state = *impl_;
  if (!state.usable) {
    return {BatchStatus::Unusable, "TYPE13 CUDA storage is poisoned"};
  }
  if (!state.bound) {
    return {BatchStatus::NotBound, "TYPE13 initial live-source binding required"};
  }
  if (!trial_identity::SameStamp(expected, state.accepted_stamp)) {
    return {BatchStatus::StaleTrial, "TYPE13 accepted owner stamp differs"};
  }
  if (!state.OutputDisjoint(output, sizeof(*output)) ||
      !trial_identity::Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !trial_identity::Disjoint(output, sizeof(*output), &expected, sizeof(expected))) {
    return {BatchStatus::InvalidInput, "TYPE13 diagnostic output overlaps input or storage"};
  }
  *output = state.accepted_diagnostics;
  return {};
}
} // namespace tl::fea::type13
