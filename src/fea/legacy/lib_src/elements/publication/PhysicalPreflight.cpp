// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../ShellBatchPublicationImpl.h"
#include "PhysicalChecks.h"
#include "../type25/Type25BatchStorage.h"
#include "../type13/resident/Storage.h"
#include "../solids/resident/Storage.h"
#include "../type45/resident/Storage.h"
#include "../beam18/resident/Storage.h"

namespace tl::fea {
namespace {
template<class State>
bool Claimed(const State& state,const NodalStamp& stamp,const ShellBatchPublication* scope) noexcept {
  return state.usable && state.bound && state.publication_scope == scope &&
      trial_identity::SameStamp(state.accepted_stamp,stamp);
}
template<class State,class Diagnostics,class Equal>
bool ShellCandidate(const State& state,const Diagnostics& diagnostics,
    const NodalPreparedView& authentic,Equal equal) noexcept {
  return state.pending && equal(diagnostics,state.candidate_diagnostics) &&
      UnavailableKinetic(diagnostics) && diagnostics.accepted_force_assembled &&
      trial_identity::SamePrepared(authentic,state.candidate_view);
}
}
bool ShellBatchPublication::Impl::PhysicalUsable() const noexcept {
  return usable && physical &&
      (!qbatch || !qbatch->impl_ || qbatch->impl_->usable) &&
      (!tbatch || !tbatch->impl_ || tbatch->impl_->usable) &&
      (!bbatch || !bbatch->impl_ || bbatch->impl_->usable) &&
      (!connector || !connector->impl_ || connector->impl_->usable) &&
      (!physical->beams || !physical->beams->impl_ || physical->beams->impl_->usable) &&
      (!physical->solids || !physical->solids->impl_ || physical->solids->impl_->usable) &&
      (!physical->joints || !physical->joints->impl_ || physical->joints->impl_->usable) &&
      (!physical->structural_beams || !physical->structural_beams->impl_ || physical->structural_beams->impl_->usable);
}
bool ShellBatchPublication::Impl::SamePhysicalScope(const NodalStamp& stamp) const noexcept {
  if (!usable || !physical || !physical->owner ||
      !trial_identity::SameStamp(stamp,physical->accepted_stamp) ||
      !trial_identity::SameStamp(stamp,physical->owner->accepted())) return false;
  const auto& binding = physical->binding;
  if (!CompletePhysicalParticipants(binding,{qbatch,tbatch,bbatch,connector,
      physical->beams,physical->solids,physical->joints,physical->structural_beams})) return false;
  if (qbatch && (!qbatch->impl_ || !qbatch->MappedBinding() ||
      !qbatch->MappedBinding()->Matches(binding) || !Claimed(*qbatch->impl_,stamp,scope))) return false;
  if (tbatch && (!tbatch->impl_ || !tbatch->MappedBinding() ||
      !tbatch->MappedBinding()->Matches(binding) || !Claimed(*tbatch->impl_,stamp,scope))) return false;
  if (bbatch && (!bbatch->impl_ || !bbatch->MappedBinding() ||
      !bbatch->MappedBinding()->Matches(binding) || !Claimed(*bbatch->impl_,stamp,scope))) return false;
  if (connector && (!connector->impl_ || !connector->MappedBinding() ||
      !connector->MappedBinding()->Matches(binding) || !Claimed(*connector->impl_,stamp,scope))) return false;
  if (physical->beams && (!physical->beams->impl_ ||
      !Claimed(*physical->beams->impl_,stamp,scope) || !physical->beams->MappedBinding() ||
      !physical->beams->MappedBinding()->Matches(binding) ||
      physical->beams->impl_->physical->owner != physical->owner ||
      !physical->beams->impl_->source.Matches(*binding.coefficients()->type13()))) return false;
  if (physical->solids && (!physical->solids->impl_ ||
      !Claimed(*physical->solids->impl_,stamp,scope) ||
      !physical->solids->impl_->model.contributions()->Matches(*binding.coefficients()->solids()))) return false;
  if (physical->structural_beams && (!physical->structural_beams->impl_ ||
      !Claimed(*physical->structural_beams->impl_,stamp,scope) ||
      !binding.coefficients()->beam18()->Matches(physical->structural_beams->impl_->model,*binding.domain()))) return false;
  if (bool(physical->joints)!=physical->joint_model.prepared()) return false;
  if (physical->joints && (!physical->joints->impl_ ||
      !Claimed(*physical->joints->impl_,stamp,scope) ||
      !physical->joints->impl_->model.SharesStorage(physical->joint_model) ||
      physical->joints->impl_->physical_scope!=&physical->binding)) return false;
  return true;
}
ShellPublicationReport ShellBatchPublication::Impl::PreflightPhysical(FENodalState& owner,
    const NodalTrialToken& token,const ShellPhysicalCandidates& candidates,
    NodalPreparedView& authentic) noexcept {
  if (physical && !PhysicalUsable())
    return {S::DeviceFailure,"A complete physical participant is poisoned"};
  if (!physical || physical->owner != &owner || !SamePhysicalScope(owner.accepted()))
    return {S::NotJoined,"Physical participants do not share the actual accepted owner scope"};
  if (bool(candidates.qeph) != bool(qbatch) || bool(candidates.t3) != bool(tbatch) ||
      bool(candidates.qbat) != bool(bbatch) || bool(candidates.type25) != bool(connector) ||
      bool(candidates.type13) != bool(physical->beams) || bool(candidates.solids) != bool(physical->solids) ||
      bool(candidates.type45) != bool(physical->joints) ||
      bool(candidates.beam18) != bool(physical->structural_beams))
    return {S::NotJoined,"Every declared physical candidate is required exactly once"};
  const auto borrowed = owner.BorrowPrepared(token,&authentic);
  if (borrowed.status != NodalStatus::Ok) return Nodal(borrowed);
  if (qbatch && !ShellCandidate(*qbatch->impl_,*candidates.qeph,authentic,
      qeph::batch_detail::SameDiagnostics))
    return {S::StaleTrial,"Mapped QEPH candidate differs from the actual assembled interval"};
  if (tbatch && !ShellCandidate(*tbatch->impl_,*candidates.t3,authentic,
      t3::batch_detail::SameDiagnostics))
    return {S::StaleTrial,"Mapped T3 candidate differs from the actual assembled interval"};
  if (bbatch) {
    const auto checked = Qbat(bbatch->PreflightPublication(owner,token,authentic,*candidates.qbat,scope));
    if (checked.status != S::Success) return checked;
    if (!candidates.qbat->accepted_force_assembled)
      return {S::StaleTrial,"Mapped QBAT candidate lacks its accepted force contribution"};
  }
  if (connector) {
    const auto checked = Connector(connector->PreflightPublication(owner,token,authentic,*candidates.type25,scope));
    if (checked.status != S::Success) return checked;
  }
  if (physical->beams) {
    const auto checked = PhysicalReport(physical->beams->PreflightPublication(owner,token,authentic,
        *candidates.type13,scope));
    if (checked.status != S::Success) return checked;
  }
  if (physical->solids) {
    const auto checked = PhysicalReport(physical->solids->PreflightPublication(owner,token,authentic,
        *candidates.solids,scope));
    if (checked.status != S::Success) return checked;
  }
  if (physical->joints) {
    const auto checked=PhysicalReport(physical->joints->PreflightPublication(owner,token,authentic,
        *candidates.type45,scope));
    if (checked.status!=S::Success) return checked;
  }
  if (physical->structural_beams) {
    const auto checked = PhysicalReport(physical->structural_beams->PreflightPublication(owner,token,authentic,
        *candidates.beam18,scope));
    if (checked.status != S::Success) return checked;
  }
  return Ok();
}
} // namespace tl::fea
