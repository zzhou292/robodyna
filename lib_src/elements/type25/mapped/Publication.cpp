// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../Type25BatchStorage.h"

namespace tl::fea::type25 {
const ShellPhysicalBinding* Batch::MappedBinding() const noexcept {
  return impl_ && impl_->physical ? &*impl_->physical : nullptr;
}
BatchReport Batch::PreflightAttachMapped(FENodalState& owner,const ShellPhysicalBinding& physical,
    std::uint64_t configuration,std::uint64_t qualification,const ShellBatchStartup& startup,
    const ShellBatchPublication* claimant) const noexcept {
  if (!impl_) return {BatchStatus::NotInitialized,"TYPE25 batch is not initialized"};
  const auto& state=*impl_;
  if (!state.usable) return {BatchStatus::DeviceFailure,"TYPE25 CUDA storage is poisoned"};
  if (!state.bound) return {BatchStatus::NotBound,"Mapped TYPE25 live sources are not bound"};
  if (!claimant || state.publication_scope || state.pending || state.accepted_stamp.epoch ||
      !state.physical || !state.physical->Matches(physical) ||
      !trial_identity::SameStamp(owner.accepted(),state.accepted_stamp) ||
      configuration!=state.config.configuration_id || qualification!=state.config.qualification_id ||
      !shell_startup_detail::SameStartup(startup,state.config.startup)) {
    return {BatchStatus::InvalidInput,"Mapped TYPE25 publication requires the same complete initial physical scope"};
  }
  const auto initial=owner.ValidateAcceptedAssemblySources(state.initial_sources);
  if (initial.status!=NodalStatus::Ok) return {
      initial.status==NodalStatus::DeviceFailure?BatchStatus::DeviceFailure:BatchStatus::StaleTrial,
      initial.message,UINT32_MAX,initial.node,Status::Success,initial.status};
  return {BatchStatus::Success,"Mapped TYPE25 common publication source is complete"};
}
} // namespace tl::fea::type25
