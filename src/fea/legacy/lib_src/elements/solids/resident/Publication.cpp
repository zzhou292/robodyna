// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OperationChecks.h"

namespace tl::fea::solids {
BatchReport Batch::PreflightPublication(FENodalState& owner, const NodalTrialToken& token,
    const NodalPreparedView& view, const BatchDiagnostics& expected,
    const ShellBatchPublication* claimant) const noexcept {
  if (!impl_) return {BatchStatus::NotInitialized, "Solid batch is not initialized"};
  const auto& state = *impl_;
  if (!state.usable) return {BatchStatus::Unusable, "Solid device storage is poisoned"};
  if (!claimant || state.publication_scope != claimant || !state.bound || !state.pending ||
      !batch_detail::SameDiagnostics(expected, state.candidate_diagnostics) ||
      !trial_identity::SamePrepared(view, state.candidate_view))
    return {BatchStatus::StaleTrial, "Solid pending publication identity differs"};
  const auto authenticated = native_physical_coefficients::AuthenticatePrepared(owner, token,
      state.accepted_stamp, view);
  return authenticated.status == NodalStatus::Ok ? BatchReport{} : batch_detail::NodalFailure(authenticated);
}
void Batch::Publish(const NodalStamp& stamp) noexcept {
  // Common coordinator only, after every participant preflight and the sole
  // owner commit. No allocations, CUDA calls or other fallible work below.
  auto& state = *impl_;
  state.accepted_slab = state.TrialSlab();
  state.accepted_stamp = stamp;
  state.accepted_diagnostics = state.candidate_diagnostics;
  state.accepted_diagnostics.phase = BatchPhase::Accepted;
  state.Discard();
}
BatchReport Batch::CopyAcceptedDiagnostics(const NodalStamp& expected,
    BatchDiagnostics* output) const noexcept {
  if (!impl_) return {BatchStatus::NotInitialized, "Solid batch is not initialized"};
  const auto& state = *impl_;
  if (!state.usable) return {BatchStatus::Unusable, "Solid device storage is poisoned"};
  if (!state.bound) return {BatchStatus::NotBound, "Solid common publication claim required"};
  if (!trial_identity::SameStamp(expected, state.accepted_stamp))
    return {BatchStatus::StaleTrial, "Solid accepted owner stamp differs"};
  if (reinterpret_cast<std::uintptr_t>(output) % alignof(BatchDiagnostics) ||
      !state.OutputDisjoint(output, sizeof(*output)) ||
      !trial_identity::Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !trial_identity::Disjoint(output, sizeof(*output), &expected, sizeof(expected)))
    return {BatchStatus::InvalidInput, "Solid diagnostic output overlaps input or storage"};
  *output = state.accepted_diagnostics;
  return {};
}
} // namespace tl::fea::solids
