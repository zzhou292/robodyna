// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../../../constraints/NodalRigidAssemblyBinding.h"

namespace tl::fea::solids {
BatchReport Batch::PreflightAttach(FENodalState& owner, const NodalCoefficientLedger& ledger,
    const NodalRigidAssemblyBinding& rigid, const NodalCinWitnessSource& cin,
    const Model& model, const BatchConfig& config, const ShellBatchPublication* claimant) {
  if (!impl_) return {BatchStatus::NotInitialized, "Solid batch is not initialized"};
  auto& state = *impl_;
  state.preflight_claimant = nullptr;
  if (!state.usable) return {BatchStatus::Unusable, "Solid device storage is poisoned"};
  if (!claimant || state.publication_scope || state.bound || state.pending ||
      state.accepted_stamp.epoch || !batch_detail::SameConfig(config, state.config) ||
      !state.model.Matches(model) || !ledger.prepared() || !ledger.solids() ||
      !rigid.prepared() || !rigid.coefficients() ||
      !rigid.coefficients()->Matches(ledger) ||
      !state.model.contributions()->Matches(*ledger.solids()) ||
      !state.model.domain()->SharesStorage(*ledger.domain()) ||
      cin.range_count != config.cin_attachment_count ||
      cin.witness_count != config.cin_witness_count ||
      !trial_identity::SameStamp(owner.accepted(), state.accepted_stamp)) {
    return {BatchStatus::InvalidInput, "Solid publication requires the same complete initial authorities"};
  }
  auto report = owner.ValidateRigidAssemblyBinding(rigid);
  if (report.status == NodalStatus::Ok) {
    report = shell_physical_owner::AuthenticateInitial(ledger, owner, state.accepted_stamp,
        config.startup, cin, state.layout.proof);
  }
  if (report.status != NodalStatus::Ok) {
    if (report.status == NodalStatus::DeviceFailure) state.usable = false;
    return {BatchStatus::NodalFailure, report.message, Family::Solid18, SIZE_MAX,
        report.node, 0, report.status};
  }
  // Compact proof receipt only. Caller still must finish all other participants
  // before the infallible common claim; this does not publish a physical step.
  state.preflight_claimant = claimant;
  return {};
}
void Batch::AttachPublication(const ShellBatchPublication* claimant) noexcept {
  // Private coordinator precondition: successful exact PreflightAttach above.
  impl_->publication_scope = claimant;
  impl_->bound = true;
  impl_->preflight_claimant = nullptr;
}
void Batch::ReleasePublication(const ShellBatchPublication* claimant) noexcept {
  if (impl_ && impl_->publication_scope == claimant) {
    impl_->publication_scope = nullptr;
    impl_->bound = false;
    impl_->Discard();
  }
}
void Batch::Poison() noexcept {
  if (impl_) {
    impl_->usable = false;
    impl_->Discard();
  }
}
} // namespace tl::fea::solids
