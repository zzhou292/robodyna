// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../Storage.h"

namespace tl::fea::type13 {
const ShellPhysicalBinding* Batch::MappedBinding() const noexcept {
  return impl_ && impl_->physical ? &impl_->physical->physical : nullptr;
}
BatchReport Batch::PreflightAttachMapped(FENodalState& owner, const ShellPhysicalBinding& physical,
    const NodalRigidAssemblyBinding& rigid, std::uint64_t configuration,
    std::uint64_t qualification, const ShellBatchStartup& startup,
    const ShellBatchPublication* claimant) const noexcept {
  if (!impl_) return {BatchStatus::NotInitialized, "TYPE13 batch is not initialized"};
  const auto& state = *impl_;
  if (!state.usable) return {BatchStatus::Unusable, "TYPE13 CUDA storage is poisoned"};
  if (!state.bound) return {BatchStatus::NotBound, "Mapped TYPE13 live sources are not bound"};
  if (!claimant || state.publication_scope || state.pending || state.accepted_stamp.epoch ||
      !state.physical || state.physical->owner != &owner ||
      !state.physical->physical.Matches(physical) ||
      !trial_identity::SameStamp(owner.accepted(), state.accepted_stamp) ||
      configuration != state.config.configuration_id || qualification != state.config.qualification_id ||
      !shell_startup_detail::SameStartup(startup, state.config.startup)) {
    return {BatchStatus::InvalidInput, "Mapped TYPE13 publication requires the same complete initial physical scope"};
  }
  auto nodal = owner.ValidateRigidAssemblyBinding(rigid);
  if (nodal.status == NodalStatus::Ok) {
    nodal = owner.ValidateAcceptedAssemblySources(state.physical->initial_sources);
  }
  if (nodal.status != NodalStatus::Ok) {
    return {nodal.status == NodalStatus::DeviceFailure ? BatchStatus::DeviceFailure : BatchStatus::StaleTrial,
            nodal.message, SIZE_MAX, nodal.node, Status::Success, nodal.status};
  }
  return {};
}
} // namespace tl::fea::type13
