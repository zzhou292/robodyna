// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../ShellBatchPublicationImpl.h"
#include "../../solvers/ExplicitNodalStep.h"

namespace tl::fea {
ShellPublicationReport ShellBatchPublication::PreparePhysical(FENodalState& owner,
    const NodalTrialToken& token,const ShellPhysicalCandidates& candidates,
    ShellPhysicalDiagnostics* output) {
  auto fail = [&](ShellPublicationReport report) {
    if (impl_ && (report.status == S::DeviceFailure || report.nodal_status == NodalStatus::DeviceFailure))
      impl_->Poison();
    if (impl_ && impl_->physical && impl_->physical->owner) impl_->physical->owner->Discard();
    else owner.Discard();
    DiscardTrial();
    return report;
  };
  if (!impl_ || !impl_->physical)
    return fail({S::NotInitialized,"Physical publication is not initialized"});
  auto& state = *impl_;
  using trial_identity::Disjoint;
  if (!state.PhysicalOutputDisjoint(output,sizeof(*output)) ||
      !Disjoint(output,sizeof(*output),&owner,sizeof(owner)) ||
      !Disjoint(output,sizeof(*output),&token,sizeof(token)) ||
      !Disjoint(output,sizeof(*output),&candidates,sizeof(candidates)) ||
      (candidates.qeph && !Disjoint(output,sizeof(*output),candidates.qeph,sizeof(*candidates.qeph))) ||
      (candidates.t3 && !Disjoint(output,sizeof(*output),candidates.t3,sizeof(*candidates.t3))) ||
      (candidates.qbat && !Disjoint(output,sizeof(*output),candidates.qbat,sizeof(*candidates.qbat))) ||
      (candidates.type25 && !Disjoint(output,sizeof(*output),candidates.type25,sizeof(*candidates.type25))) ||
      (candidates.type13 && !Disjoint(output,sizeof(*output),candidates.type13,sizeof(*candidates.type13))) ||
      (candidates.solids && !Disjoint(output,sizeof(*output),candidates.solids,sizeof(*candidates.solids))) ||
      (candidates.type45 && !Disjoint(output,sizeof(*output),candidates.type45,sizeof(*candidates.type45))) ||
      (candidates.beam18 && !Disjoint(output,sizeof(*output),candidates.beam18,sizeof(*candidates.beam18))))
    return fail({S::InvalidInput,"Physical diagnostic output overlaps an input or retained source"});
  state.pending = false;
  state.physical->candidate = {};
  state.candidate_view = {};
  NodalPreparedView authentic;
  const auto checked = state.PreflightPhysical(owner,token,candidates,authentic);
  if (checked.status != S::Success) return fail(checked);
  ShellPhysicalDiagnostics next;
  next.base_stamp = owner.accepted();
  next.has_qeph = candidates.qeph != nullptr;
  next.has_t3 = candidates.t3 != nullptr;
  next.has_qbat = candidates.qbat != nullptr;
  next.has_type25 = candidates.type25 != nullptr;
  next.has_type13 = candidates.type13 != nullptr;
  next.has_solids = candidates.solids != nullptr;
  next.has_type45 = candidates.type45 != nullptr;
  next.has_beam18 = candidates.beam18 != nullptr;
  state.CapturePhysicalDiagnostics(next,true);
  next.valid = true;
  state.physical->candidate = next;
  state.candidate_view = authentic;
  state.pending = true;
  *output = next;
  return Ok();
}
ShellPublicationReport ShellBatchPublication::CommitPhysical(FENodalState& owner,
    const NodalTrialToken& token,const ShellPhysicalDiagnostics& expected,
    const NodalValidationReceipt& receipt) noexcept {
  auto fail = [&](ShellPublicationReport report) {
    if (impl_ && (report.status == S::DeviceFailure || report.nodal_status == NodalStatus::DeviceFailure))
      impl_->Poison();
    if (impl_ && impl_->physical && impl_->physical->owner) impl_->physical->owner->Discard();
    else owner.Discard();
    DiscardTrial();
    return report;
  };
  if (!impl_ || !impl_->physical)
    return fail({S::NotInitialized,"Physical publication is not initialized"});
  auto& state = *impl_;
  auto& physical = *state.physical;
  if (!state.pending || !SamePhysicalDiagnostics(expected,physical.candidate))
    return fail({S::StaleTrial,"Physical publication differs from its complete prepared candidate"});
  NodalPreparedView authentic;
  const auto checked = state.PreflightPhysical(owner,token,Candidates(expected),authentic);
  if (checked.status != S::Success) return fail(checked);
  if (!trial_identity::SamePrepared(authentic,state.candidate_view) || !receipt.passed ||
      receipt.owner_id != authentic.owner_id || receipt.base_epoch != owner.accepted().epoch ||
      receipt.attempt != authentic.attempt || receipt.qualification_id != physical.identity.qualification_id)
    return fail({S::StaleTrial,"Physical capture receipt differs from the authentic interval"});
  auto nodal = CompleteNodalValidation(owner,token,receipt);
  if (nodal.status != NodalStatus::Ok) return fail(Nodal(nodal));
  if (physical.HasScratchParticipation()) {
    const auto participation=ValidatePhysicalScratchSeal(owner,authentic);
    if (participation.status!=S::Success) return fail(participation);
    // Consume fixed issuer identity only. Any prepared typed native selector
    // plan remains private until owner success; common failure discards it.
    ConsumePhysicalScratchSeal();
  }
  nodal = owner.Commit(token);
  if (nodal.status != NodalStatus::Ok) return fail(Nodal(nodal));
  // The only owner commit has succeeded. Nothing below allocates, reads CUDA,
  // validates arithmetic, calls user code, or returns a fallible status.
  const auto stamp = owner.accepted();
  PublishNativeContactState(stamp);
  if (state.qbatch) state.qbatch->impl_->Publish(stamp);
  if (state.tbatch) state.tbatch->impl_->Publish(stamp);
  if (state.bbatch) state.bbatch->Publish(stamp);
  if (state.connector) state.connector->Publish(stamp);
  if (physical.beams) physical.beams->Publish(stamp);
  if (physical.solids) physical.solids->Publish(stamp);
  if (physical.joints) physical.joints->Publish(stamp);
  if (physical.structural_beams) physical.structural_beams->Publish(stamp);
  physical.accepted = physical.candidate;
  physical.accepted_stamp = stamp;
  if (state.qbatch) physical.accepted.qeph.phase = qeph::BatchPhase::Accepted;
  if (state.tbatch) physical.accepted.t3.phase = t3::BatchPhase::Accepted;
  if (state.bbatch) physical.accepted.qbat.phase = qbat::BatchPhase::Accepted;
  if (state.connector) physical.accepted.type25.phase = type25::BatchPhase::Accepted;
  if (physical.beams) physical.accepted.type13.phase = type13::BatchPhase::Accepted;
  if (physical.solids) physical.accepted.solids.phase = solids::BatchPhase::Accepted;
  if (physical.joints) physical.accepted.type45.phase = type45::BatchPhase::Accepted;
  if (physical.structural_beams) physical.accepted.beam18.phase = beam18::BatchPhase::Accepted;
  state.pending = false;
  state.candidate_view = {};
  physical.candidate = {};
  return Ok();
}
} // namespace tl::fea
