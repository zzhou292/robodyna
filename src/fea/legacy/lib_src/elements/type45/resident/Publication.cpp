// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OperationChecks.h"

namespace tl::fea::type45 {
BatchReport Batch::PreflightPublication(FENodalState& owner,const NodalTrialToken& token,
    const NodalPreparedView& view,const BatchDiagnostics& expected,const ShellBatchPublication* claimant) const noexcept {
  if(!impl_) return {BatchStatus::NotInitialized,"Joint batch is not initialized"};
  const auto& s=*impl_;
  if(!s.usable) return {BatchStatus::Unusable,"Joint device storage is poisoned"};
  if(!claimant || s.publication_scope!=claimant || !s.bound || !s.pending ||
      !resident_detail::SameDiagnostics(expected,s.candidate_diagnostics) ||
      !trial_identity::SamePrepared(view,s.candidate_view))
    return {BatchStatus::StaleTrial,"Joint pending publication identity differs"};
  const auto checked=native_physical_coefficients::AuthenticatePrepared(owner,token,s.accepted_stamp,view);
  return checked.status==NodalStatus::Ok?BatchReport{}:resident_detail::NodalFailure(checked);
}
void Batch::Publish(const NodalStamp& stamp) noexcept {
  // The sole common owner commit has succeeded. No allocation, CUDA call or
  // other fallible operation occurs while publishing this accepted selector.
  auto& s=*impl_;s.accepted_slab=s.TrialSlab();s.accepted_stamp=stamp;
  s.accepted_diagnostics=s.candidate_diagnostics;s.accepted_diagnostics.phase=BatchPhase::Accepted;s.Discard();
}
BatchReport Batch::CopyAcceptedDiagnostics(const NodalStamp& expected,BatchDiagnostics* output) const noexcept {
  if(!impl_) return {BatchStatus::NotInitialized,"Joint batch is not initialized"};
  const auto& s=*impl_;
  if(!s.usable) return {BatchStatus::Unusable,"Joint device storage is poisoned"};
  if(!s.bound) return {BatchStatus::NotBound,"Joint common publication claim required"};
  if(!trial_identity::SameStamp(expected,s.accepted_stamp)) return {BatchStatus::StaleTrial,"Joint accepted stamp differs"};
  if(reinterpret_cast<std::uintptr_t>(output)%alignof(BatchDiagnostics) ||
      !s.OutputDisjoint(output,sizeof(*output)) ||
      !trial_identity::Disjoint(output,sizeof(*output),this,sizeof(*this)) ||
      !trial_identity::Disjoint(output,sizeof(*output),&expected,sizeof(expected)))
    return {BatchStatus::InvalidInput,"Joint diagnostic output overlaps input or storage"};
  *output=s.accepted_diagnostics;return {};
}
} // namespace tl::fea::type45
