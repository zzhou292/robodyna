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
      (ledger.order() != CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_Beam18_V5 &&
       ledger.order() != (model.profile() == ModelProfile::ExtendedLaw44Law90
          ? CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_V4
          : CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_V3)) ||
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
  if (report.status == NodalStatus::Ok &&
      config.startup.kind == ShellBatchStartupKind::ReferenceConstrainedUniformTranslation) {
    // TT0 force/history caches use one common velocity for every actual parent
    // slot. Unrelated fixed shell nodes are allowed; constrained solid support
    // needs a separate cache implementation. Rotations and inverse M are not
    // part of this translation-only role proof.
    const auto support = [&](const auto& parents, Family family, unsigned slots) -> BatchReport {
      for (std::size_t p=0;p<parents.size();++p) {
        std::size_t distinct[8], count=0;
        for (unsigned slot=0;slot<slots;++slot) {
          const auto node=parents[p].domain_nodes[slot];
          bool seen=false;
          for (std::size_t i=0;i<count;++i) seen=seen || distinct[i]==node;
          if (!seen) distinct[count++]=node;
        }
        const auto checked=owner.ValidateFreeTranslationalNodes(distinct,count);
        if (checked.status!=NodalStatus::Ok) {
          if (checked.status==NodalStatus::DeviceFailure) state.usable=false;
          return {BatchStatus::NodalFailure,checked.message,family,p,checked.node,0,checked.status};
        }
      }
      return {};
    };
    auto checked=support(model.solid18(),Family::Solid18,batch_detail::Traits18::nodes);
    if (checked) checked=support(model.solid24(),Family::Solid24,batch_detail::Traits24::nodes);
    if (checked) checked=support(model.solid6z(),Family::Solid6z,batch_detail::Traits6z::nodes);
    if (checked) checked=support(model.solid18_law44(),Family::Solid18Law44,batch_detail::Traits18Law44::nodes);
    if (checked) checked=support(model.solid18_law90(),Family::Solid18Law90,batch_detail::Traits18Law90::nodes);
    if (!checked) return checked;
  }
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
