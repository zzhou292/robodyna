// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../T3BatchStorage.h"

namespace tl::fea::t3 {
const ShellPhysicalBinding* T3Batch::MappedBinding() const noexcept {
  return impl_ && impl_->physical ? &*impl_->physical : nullptr;
}
BatchReport T3Batch::PreflightAttachMapped(FENodalState& owner,const ShellPhysicalBinding& physical,
    std::uint64_t configuration,std::uint64_t qualification,const ShellBatchStartup& startup,
    const ShellBatchPublication* claimant) const noexcept {
  if (!impl_) return {BatchStatus::NotInitialized,"T3 batch is not initialized"};
  const auto& state=*impl_;
  if (!state.usable) return {BatchStatus::DeviceFailure,"T3 CUDA storage is poisoned"};
  if (!state.bound) return {BatchStatus::NotBound,"Mapped T3 live sources are not bound"};
  if (!claimant || state.publication_scope || state.pending || state.accepted_stamp.epoch ||
      !state.physical || !state.physical->Matches(physical) ||
      !trial_identity::SameStamp(owner.accepted(),state.accepted_stamp) ||
      configuration!=state.config.configuration_id || qualification!=state.config.qualification_id ||
      !shell_startup_detail::SameStartup(startup,state.config.startup)) {
    return {BatchStatus::InvalidInput,"Mapped T3 publication requires the same complete initial physical scope"};
  }
  if (state.physical->execution()) {
    const auto roles=owner.ValidateRigidAssemblyBinding(*state.physical->execution()->rigid());
    if (roles.status!=NodalStatus::Ok) return {BatchStatus::InvalidInput,roles.message};
  }
  const auto initial=owner.ValidateAcceptedAssemblySources(state.initial_sources);
  if (initial.status!=NodalStatus::Ok) return {
      initial.status==NodalStatus::DeviceFailure?BatchStatus::DeviceFailure:BatchStatus::StaleTrial,
      initial.message,UINT32_MAX,initial.node,Status::kSuccess,initial.status};
  return {BatchStatus::Success,"Mapped T3 common publication source is complete"};
}
} // namespace tl::fea::t3
