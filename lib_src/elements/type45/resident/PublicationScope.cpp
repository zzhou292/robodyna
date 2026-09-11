// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OperationChecks.h"
#include "../../../assembly/ShellPhysicalBinding.h"

namespace tl::fea::type45 {
BatchReport Batch::PreflightAttach(FENodalState& owner,const ShellPhysicalBinding& physical,
    const NodalCinWitnessSource& cin,const Model& model,const BatchConfig& config,
    const ShellBatchPublication* claimant) {
  if(!impl_) return {BatchStatus::NotInitialized,"Joint batch is not initialized"};
  auto& s=*impl_;s.preflight_claimant=nullptr;
  if(!s.usable) return {BatchStatus::Unusable,"Joint device storage is poisoned"};
  if(!claimant || s.publication_scope || s.bound || s.pending || s.accepted_stamp.epoch ||
      !resident_detail::SameConfig(config,s.config) || !s.model.SharesStorage(model) ||
      !physical.prepared() || !physical.coefficients() ||
      !model.rigid_binding()->coefficients()->Matches(*physical.coefficients()) ||
      !model.domain()->SharesStorage(*physical.domain()) ||
      cin.range_count!=config.cin_attachment_count || cin.witness_count!=config.cin_witness_count ||
      !trial_identity::SameStamp(owner.accepted(),s.accepted_stamp))
    return {BatchStatus::InvalidInput,"Joint publication requires the same complete initial authorities"};
  auto checked=owner.ValidateRigidAssemblyBinding(*model.rigid_binding());
  if(checked.status==NodalStatus::Ok)
    checked=shell_physical_owner::AuthenticateInitial(*physical.coefficients(),owner,s.accepted_stamp,
        config.startup,cin,s.layout.proof);
  if(checked.status!=NodalStatus::Ok) {
    if(checked.status==NodalStatus::DeviceFailure) s.usable=false;
    return resident_detail::NodalFailure(checked);
  }
  // No borrowed scope is installed until every participant's preflight passes.
  s.preflight_claimant=claimant;return {};
}
void Batch::AttachPublication(const ShellBatchPublication* claimant,const ShellPhysicalBinding& physical) noexcept {
  impl_->publication_scope=claimant;impl_->physical_scope=&physical;impl_->bound=true;
  impl_->preflight_claimant=nullptr;
}
void Batch::ReleasePublication(const ShellBatchPublication* claimant) noexcept {
  if(impl_ && impl_->publication_scope==claimant) {
    impl_->publication_scope=nullptr;impl_->physical_scope=nullptr;impl_->bound=false;impl_->Discard();
  }
}
void Batch::Poison() noexcept {if(impl_) {impl_->usable=false;impl_->Discard();}}
} // namespace tl::fea::type45
