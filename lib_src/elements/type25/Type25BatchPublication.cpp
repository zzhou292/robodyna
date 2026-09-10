// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type25BatchStorage.h"
#include "../../solvers/NodalNativePhysicalCoefficients.h"

namespace tl::fea::type25 {
BatchReport Batch::PreflightAttach(const NodalStamp& stamp,const NodalMassBinding& mass,
    std::uint64_t configuration,std::uint64_t qualification,const ShellBatchStartup& startup,
    const ShellBatchPublication* claimant) const noexcept {
  if(!impl_)return {BatchStatus::NotInitialized,"TYPE25 batch is not initialized"};
  const auto& s=*impl_;
  if(!s.usable)return {BatchStatus::Unusable,"TYPE25 CUDA storage is poisoned"};
  if(!s.bound)return {BatchStatus::NotBound,"TYPE25 original live sources are not bound"};
  if(!claimant||s.publication_scope||s.pending||s.accepted_stamp.epoch||
     !trial_identity::SameStamp(stamp,s.accepted_stamp)||!s.combined->Matches(mass)||
     configuration!=s.config.configuration_id||qualification!=s.config.qualification_id||
     !shell_startup_detail::SameStartup(startup,s.config.startup))
    return {BatchStatus::InvalidInput,"TYPE25 publication requires the same bound initial scope"};
  return {BatchStatus::Success,"OK"};
}
void Batch::AttachPublication(const ShellBatchPublication* scope) noexcept {impl_->publication_scope=scope;}
void Batch::ReleasePublication(const ShellBatchPublication* scope) noexcept {
  if(impl_&&impl_->publication_scope==scope)impl_->publication_scope=nullptr;
}
BatchReport Batch::PreflightPublication(FENodalState& owner,const NodalTrialToken& token,
    const NodalPreparedView& prepared,const BatchDiagnostics& expected,const ShellBatchPublication* claimant) const noexcept {
  if(!impl_)return {BatchStatus::NotInitialized,"TYPE25 batch is not initialized"};
  const auto& s=*impl_;
  if(!s.usable)return {BatchStatus::Unusable,"TYPE25 CUDA storage is poisoned"};
  if(!claimant||s.publication_scope!=claimant||!s.bound||!s.pending||
     !batch_detail::SameDiagnostics(expected,s.candidate_diagnostics)||
     !trial_identity::SamePrepared(prepared,s.candidate_view))
    return {BatchStatus::StaleTrial,"TYPE25 complete pending candidate/publication scope differs"};
  const auto authenticated=native_physical_coefficients::AuthenticatePrepared(owner,token,s.accepted_stamp,prepared);
  if(authenticated.status!=NodalStatus::Ok)
    return {authenticated.status==NodalStatus::DeviceFailure?BatchStatus::NodalFailure:BatchStatus::StaleTrial,
            authenticated.message,UINT32_MAX,authenticated.node,Status::Success,authenticated.status};
  return {BatchStatus::Success,"OK"};
}
void Batch::Publish(const NodalStamp& stamp) noexcept {
  // Called only by the sole common publisher after successful owner commit.
  // No allocation, CUDA, validation, callback or fallible work belongs here.
  impl_->Publish(stamp);
}
BatchReport Batch::CopyAcceptedDiagnostics(const NodalStamp& expected,BatchDiagnostics* output) const noexcept {
  if(!impl_)return {BatchStatus::NotInitialized,"TYPE25 batch is not initialized"};
  const auto& s=*impl_;
  if(!s.usable)return {BatchStatus::Unusable,"TYPE25 CUDA storage is poisoned"};
  if(!s.bound)return {BatchStatus::NotBound,"TYPE25 initial live-source binding is required"};
  if(!trial_identity::SameStamp(expected,s.accepted_stamp))return {BatchStatus::StaleTrial,"TYPE25 accepted identity differs"};
  if(!s.OutputDisjoint(output,sizeof(*output))||!trial_identity::Disjoint(output,sizeof(*output),this,sizeof(*this))||
     !trial_identity::Disjoint(output,sizeof(*output),&expected,sizeof(expected)))
    return {BatchStatus::InvalidInput,"TYPE25 diagnostic output overlaps input or owned data"};
  *output=s.accepted_diagnostics;return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::type25
