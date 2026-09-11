// SPDX-License-Identifier: AGPL-3.0-or-later
#include "QbatBatchStorage.h"
#include "../../solvers/NodalNativePhysicalCoefficients.h"

namespace tl::fea::qbat {
BatchReport Batch::PreflightAttach(FENodalState& owner,const ShellFormulationScope& scope,
    std::uint64_t configuration,std::uint64_t qualification,const ShellBatchStartup& startup,
    const ShellBatchPublication* claimant) const noexcept {
  if(!impl_) return {BatchStatus::NotInitialized,"QBAT batch is not initialized"};
  const auto& s=*impl_;
  if(!s.usable) return {BatchStatus::DeviceFailure,"QBAT CUDA storage is poisoned"};
  if(!s.bound) return {BatchStatus::NotBound,"QBAT live sources are not bound"};
  if(ValidateShellFormulationScope(scope).status!=ShellPlasticityBindingStatus::Success||
      !claimant||s.publication_scope||s.pending||s.accepted_stamp.epoch||
      !trial_identity::SameStamp(owner.accepted(),s.accepted_stamp)||
      s.binding->inventory()!=scope.binding->inventory()||!s.failure->SameScope(*scope.failure)||
      bool(s.combined)!=bool(scope.mass)||(scope.mass&&!s.combined->Matches(*scope.mass))||
      configuration!=s.config.configuration_id||qualification!=s.config.qualification_id||
      !shell_startup_detail::SameStartup(startup,s.config.startup)) {
    return {BatchStatus::InvalidInput,"QBAT publication requires the same bound initial formulation scope"};
  }
  const auto initial=owner.ValidateAcceptedAssemblySources(s.initial_sources);
  if(initial.status!=NodalStatus::Ok) {
    return {initial.status==NodalStatus::DeviceFailure?BatchStatus::NodalFailure:BatchStatus::StaleTrial,
        initial.message,UINT32_MAX,initial.node,Status::kSuccess,initial.status};
  }
  return {BatchStatus::Success,"OK"};
}
void Batch::AttachPublication(const ShellBatchPublication* scope) noexcept { impl_->publication_scope=scope; }
void Batch::ReleasePublication(const ShellBatchPublication* scope) noexcept {
  if(impl_&&impl_->publication_scope==scope) impl_->publication_scope=nullptr;
}
BatchReport Batch::PreflightPublication(FENodalState& owner,const NodalTrialToken& token,
    const NodalPreparedView& prepared,const BatchDiagnostics& expected,
    const ShellBatchPublication* claimant) const noexcept {
  if(!impl_) return {BatchStatus::NotInitialized,"QBAT batch is not initialized"};
  const auto& s=*impl_;
  if(!s.usable) return {BatchStatus::DeviceFailure,"QBAT CUDA storage is poisoned"};
  if(!claimant||s.publication_scope!=claimant||!s.bound||!s.pending||
      !batch_detail::SameDiagnostics(expected,s.candidate_diagnostics)||
      !trial_identity::SamePrepared(prepared,s.candidate_view)) {
    return {BatchStatus::StaleTrial,"QBAT complete pending candidate/publication scope differs"};
  }
  if(!s.accepted_stamp.epoch) {
    const auto initial=owner.ValidateAcceptedAssemblySources(s.initial_sources);
    if(initial.status!=NodalStatus::Ok) {
      return {initial.status==NodalStatus::DeviceFailure?BatchStatus::NodalFailure:BatchStatus::StaleTrial,
          initial.message,UINT32_MAX,initial.node,Status::kSuccess,initial.status};
    }
  }
  const auto authenticated=native_physical_coefficients::AuthenticatePrepared(owner,token,s.accepted_stamp,prepared);
  if(authenticated.status!=NodalStatus::Ok) {
    return {authenticated.status==NodalStatus::DeviceFailure?BatchStatus::NodalFailure:BatchStatus::StaleTrial,
        authenticated.message,UINT32_MAX,authenticated.node,Status::kSuccess,authenticated.status};
  }
  return {BatchStatus::Success,"OK"};
}
void Batch::Publish(const NodalStamp& stamp) noexcept { impl_->Publish(stamp); }
BatchReport Batch::CopyAcceptedDiagnostics(const NodalStamp& expected,BatchDiagnostics* output) const noexcept {
  if(!impl_) return {BatchStatus::NotInitialized,"QBAT batch is not initialized"};
  const auto& s=*impl_;
  if(!s.usable) return {BatchStatus::DeviceFailure,"QBAT CUDA storage is poisoned"};
  if(!s.bound) return {BatchStatus::NotBound,"QBAT live sources are not bound"};
  if(!trial_identity::SameStamp(expected,s.accepted_stamp)) return {BatchStatus::StaleTrial,"QBAT accepted identity differs"};
  if(!s.OutputDisjoint(output,sizeof(*output))||!trial_identity::Disjoint(output,sizeof(*output),this,sizeof(*this))||
      !trial_identity::Disjoint(output,sizeof(*output),&expected,sizeof(expected))) {
    return {BatchStatus::InvalidInput,"QBAT diagnostic output overlaps input or owned data"};
  }
  *output=s.accepted_diagnostics;
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::qbat
