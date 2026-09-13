// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../ShellBatchPublicationImpl.h"
#include "../ShellPhysicalOwner.h"
#include "PhysicalScratchParticipationState.h"
#include <limits>
#include <new>

namespace tl::fea {
namespace {
using shell_publication_detail::PhysicalScratchKindCount;
using shell_publication_detail::PhysicalScratchKindIndex;
using shell_publication_detail::PhysicalScratchParticipationState;

const ShellPhysicalScratchRosterEntry& RosterEntry(
    const ShellPhysicalScratchRoster& roster,
    ShellPhysicalScratchContributorKind kind) noexcept {
  return kind == ShellPhysicalScratchContributorKind::MappedWall
      ? roster.mapped_wall : roster.self_contact;
}
const ShellPhysicalScratchParticipationReceipt* ReceiptEntry(
    const ShellPhysicalScratchReceiptRoster& roster,
    ShellPhysicalScratchContributorKind kind) noexcept {
  return kind == ShellPhysicalScratchContributorKind::MappedWall
      ? roster.mapped_wall : roster.self_contact;
}
ShellPhysicalScratchContributorKind Kind(std::size_t slot) noexcept {
  return slot == 0 ? ShellPhysicalScratchContributorKind::MappedWall
                   : ShellPhysicalScratchContributorKind::SelfContact;
}
bool Present(const ShellPhysicalScratchRosterEntry& entry) noexcept {
  return entry.issuer != nullptr || entry.source_id != 0;
}
bool Complete(const ShellPhysicalScratchRosterEntry& entry) noexcept {
  return entry.issuer != nullptr && entry.source_id != 0;
}
bool RosterShape(const ShellPhysicalScratchRoster& roster,
                 std::size_t& count) noexcept {
  count = 0;
  for (std::size_t slot = 0; slot < PhysicalScratchKindCount; ++slot) {
    const auto& entry = RosterEntry(roster,Kind(slot));
    if (Present(entry) && !Complete(entry)) return false;
    if (Complete(entry)) ++count;
  }
  return count != 0 &&
      (!Complete(roster.mapped_wall) || !Complete(roster.self_contact) ||
       roster.mapped_wall.issuer != roster.self_contact.issuer);
}
}  // namespace

namespace shell_publication_detail {
PhysicalState::~PhysicalState() {
  if (HasScratchParticipation()) delete runtime.scratch.state;
}
void PhysicalState::SetCinCounts(std::size_t attachments,
                                 std::size_t witnesses) noexcept {
  runtime.cin = {attachments,witnesses};
}
bool PhysicalState::HasScratchParticipation() const noexcept {
  return runtime.scratch.configured_marker == 0 &&
      runtime.scratch.state != nullptr;
}
PhysicalScratchParticipationState*
PhysicalState::ScratchParticipation() noexcept {
  return HasScratchParticipation() ? runtime.scratch.state : nullptr;
}
const PhysicalScratchParticipationState*
PhysicalState::ScratchParticipation() const noexcept {
  return HasScratchParticipation() ? runtime.scratch.state : nullptr;
}
void PhysicalState::DiscardScratchParticipation() noexcept {
  auto* participation = ScratchParticipation();
  if (!participation) return;
  participation->sealed = false;
  participation->sealed_owner = nullptr;
  participation->sealed_stream = nullptr;
  participation->sealed_owner_id = 0;
  participation->sealed_base_epoch = 0;
  participation->sealed_attempt = 0;
  for (std::size_t slot = 0; slot < PhysicalScratchKindCount; ++slot) {
    participation->sealed_generation[slot] = 0;
    if (participation->entries[slot].issuer)
      participation->entries[slot].issuer->DiscardTrial();
  }
}
}  // namespace shell_publication_detail

ShellPhysicalScratchParticipation::~ShellPhysicalScratchParticipation() noexcept {
  if (publication_) publication_->ReleasePhysicalScratchParticipation(*this);
}
void ShellPhysicalScratchParticipation::Bind(
    ShellBatchPublication& publication,FENodalState& owner,
    ShellPhysicalScratchContributorKind kind,std::uint64_t source_id,
    std::size_t witness_count) noexcept {
  publication_=&publication;
  owner_=&owner;
  kind_=kind;
  source_id_=source_id;
  witness_count_=witness_count;
  phase_=Phase::Idle;
}
void ShellPhysicalScratchParticipation::Unbind(
    const ShellBatchPublication* publication) noexcept {
  if (publication_ != publication) return;
  publication_=nullptr;
  owner_=nullptr;
  stream_=nullptr;
  source_id_=0;
  owner_id_=base_epoch_=attempt_=0;
  last_base_epoch_=last_attempt_=generation_=0;
  witness_count_=0;
  phase_=Phase::Idle;
}
void ShellPhysicalScratchParticipation::DiscardTrial() noexcept {
  stream_=nullptr;
  owner_id_=base_epoch_=attempt_=0;
  phase_=Phase::Idle;
}
void ShellPhysicalScratchParticipation::Consume() noexcept {
  DiscardTrial();
}

ShellPublicationReport
ShellBatchPublication::ForecastPhysicalScratchParticipation(
    const ShellPhysicalScratchRoster& roster,
    const ShellPhysicalScratchParticipationLimits& limits,
    ShellPhysicalScratchParticipationForecast& output) noexcept {
  using trial_identity::Disjoint;
  std::size_t count=0;
  if (!RosterShape(roster,count))
    return {S::InvalidInput,
            "Scratch participation roster entries are incomplete, empty or duplicate"};
  if (!limits.max_host_bytes ||
      limits.max_host_bytes > MaxShellPhysicalScratchParticipationHostBytes)
    return {S::ResourceLimit,
            "Scratch participation host limit is outside the fixed profile"};
  if (!Disjoint(&output,sizeof(output),&roster,sizeof(roster)) ||
      !Disjoint(&output,sizeof(output),&limits,sizeof(limits)))
    return {S::InvalidInput,
            "Scratch participation forecast overlaps its input"};
  for (std::size_t slot=0;slot<PhysicalScratchKindCount;++slot) {
    const auto& entry=RosterEntry(roster,Kind(slot));
    if (!Complete(entry)) continue;
    if (!Disjoint(&output,sizeof(output),entry.issuer,sizeof(*entry.issuer)))
      return {S::InvalidInput,
              "Scratch participation forecast overlaps an issuer"};
  }
  ShellPhysicalScratchParticipationForecast next;
  next.publication_host_bytes=sizeof(PhysicalScratchParticipationState);
  next.configured_issuer_host_bytes=count*sizeof(ShellPhysicalScratchParticipation);
  if (next.publication_host_bytes >
      std::numeric_limits<std::size_t>::max()-next.configured_issuer_host_bytes)
    return {S::ResourceLimit,
            "Scratch participation host byte arithmetic overflowed"};
  next.total_host_bytes=next.publication_host_bytes+
      next.configured_issuer_host_bytes;
  if (next.total_host_bytes>limits.max_host_bytes)
    return {S::ResourceLimit,
            "Scratch participation fixed host payload exceeds its cap"};
  output=next;
  return Ok();
}

ShellPublicationReport
ShellBatchPublication::ConfigurePhysicalScratchParticipation(
    FENodalState& owner,const ShellPhysicalBinding& binding,
    const ShellPhysicalParticipants& participants,
    const ShellPhysicalPublicationIdentity& identity,
    const ShellPhysicalScratchRoster& roster,
    const ShellPhysicalScratchParticipationLimits& limits) noexcept {
  if (!impl_ || !impl_->physical)
    return {S::NotInitialized,"Physical publication is not initialized"};
  auto& state=*impl_;
  auto& physical=*state.physical;
  if (physical.HasScratchParticipation())
    return {S::InvalidInput,
            "Physical scratch participation roster is already configured"};
  if (physical.owner!=&owner || state.pending || owner.accepted().epoch!=0 ||
      physical.accepted_stamp.epoch!=0 ||
      !trial_identity::SameStamp(owner.accepted(),physical.accepted_stamp))
    return {S::StaleTrial,
            "Scratch participation must bind the actual owner before interval 1"};
  const auto source=ValidatePhysicalSources(owner,binding,participants,identity);
  if (source.status!=S::Success) return source;
  for (std::size_t slot=0;slot<PhysicalScratchKindCount;++slot) {
    const auto& entry=RosterEntry(roster,Kind(slot));
    if (entry.issuer && entry.issuer->configured())
      return {S::InvalidInput,
              "Scratch participation issuer is already configured"};
  }
  ShellPhysicalScratchParticipationForecast forecast;
  const auto planned=ForecastPhysicalScratchParticipation(roster,limits,forecast);
  if (planned.status!=S::Success) return planned;
  auto* participation=new(std::nothrow) PhysicalScratchParticipationState;
  if (!participation)
    return {S::ResourceLimit,
            "Scratch participation fixed host allocation failed"};
  participation->cin_attachment_count=physical.runtime.cin.attachment_count;
  participation->cin_witness_count=physical.runtime.cin.witness_count;
  for (std::size_t slot=0;slot<PhysicalScratchKindCount;++slot) {
    const auto kind=Kind(slot);
    const auto& entry=RosterEntry(roster,kind);
    participation->entries[slot]={entry.issuer,entry.source_id};
    if (entry.issuer)
      entry.issuer->Bind(*this,owner,kind,entry.source_id,
                         participation->cin_witness_count);
  }
  physical.runtime.scratch={participation,0};
  return Ok();
}

ShellPublicationReport
ShellPhysicalScratchParticipation::RecordAcceptedAssembly(
    std::uint64_t source_id,FENodalState& owner,const NodalTrialToken& token,
    const NodalAssemblyView& view) noexcept {
  if (!publication_)
    return {ShellPublicationStatus::NotInitialized,
            "Scratch participation issuer is not configured"};
  return publication_->RecordPhysicalScratchAssembly(
      *this,source_id,owner,token,view);
}

ShellPublicationReport ShellBatchPublication::RecordPhysicalScratchAssembly(
    ShellPhysicalScratchParticipation& issuer,std::uint64_t source_id,
    FENodalState& owner,const NodalTrialToken& token,
    const NodalAssemblyView& view) noexcept {
  auto fail=[&](ShellPublicationReport report) {
    if (impl_ && impl_->physical && impl_->physical->owner)
      impl_->physical->owner->Discard();
    else owner.Discard();
    if (impl_) impl_->Discard();
    return report;
  };
  if (!impl_ || !impl_->physical)
    return fail({S::NotInitialized,"Physical publication is not initialized"});
  auto& state=*impl_;
  auto& physical=*state.physical;
  auto* participation=physical.ScratchParticipation();
  const auto slot=PhysicalScratchKindIndex(issuer.kind_);
  if (!participation || slot>=PhysicalScratchKindCount ||
      participation->entries[slot].issuer!=&issuer ||
      participation->entries[slot].source_id!=issuer.source_id_ ||
      issuer.publication_!=this || issuer.owner_!=physical.owner ||
      issuer.witness_count_!=participation->cin_witness_count)
    return fail({S::ParticipationFailure,
                 "Scratch assembly issuer is foreign or not in the fixed roster"});
  if (&owner!=physical.owner)
    return fail({S::ParticipationFailure,
                 "Scratch assembly belongs to a foreign physical owner"});
  if (!source_id || source_id!=issuer.source_id_)
    return fail({S::ParticipationFailure,
                 "Scratch assembly source identity differs from registration"});
  if (issuer.phase_!=ShellPhysicalScratchParticipation::Phase::Idle)
    return fail({S::ParticipationFailure,
                 "Scratch contributor assembly was recorded more than once"});
  const auto stamp=owner.accepted();
  if (!state.SamePhysicalScope(stamp) ||
      view.attempt<=issuer.last_attempt_ ||
      (!issuer.last_attempt_ && stamp.epoch!=0) ||
      (issuer.last_attempt_ &&
       (stamp.epoch<issuer.last_base_epoch_ ||
        stamp.epoch-issuer.last_base_epoch_>1)))
    return fail({S::ParticipationFailure,
                 "Scratch contributor attempt is replayed, stale or skipped"});
  NodalCinAssemblyView cin;
  const auto authenticated=shell_physical_owner::BorrowAssembly(
      owner,token,stamp,view,participation->cin_witness_count,&cin);
  if (authenticated.status!=NodalStatus::Ok)
    return fail({S::ParticipationFailure,authenticated.message,
                 authenticated.status});
  issuer.stream_=view.stream;
  issuer.owner_id_=view.owner_id;
  issuer.base_epoch_=view.accepted.base_epoch;
  issuer.attempt_=view.attempt;
  issuer.last_base_epoch_=stamp.epoch;
  issuer.last_attempt_=view.attempt;
  issuer.phase_=ShellPhysicalScratchParticipation::Phase::AssemblyRecorded;
  return Ok();
}

ShellPublicationReport ShellPhysicalScratchParticipation::SealCandidate(
    std::uint64_t source_id,FENodalState& owner,const NodalTrialToken& token,
    const NodalPreparedView& prepared,
    ShellPhysicalScratchParticipationReceipt* output) noexcept {
  if (!publication_)
    return {ShellPublicationStatus::NotInitialized,
            "Scratch participation issuer is not configured"};
  return publication_->SealPhysicalScratchCandidate(
      *this,source_id,owner,token,prepared,output);
}

ShellPublicationReport ShellBatchPublication::SealPhysicalScratchCandidate(
    ShellPhysicalScratchParticipation& issuer,std::uint64_t source_id,
    FENodalState& owner,const NodalTrialToken& token,
    const NodalPreparedView& prepared,
    ShellPhysicalScratchParticipationReceipt* output) noexcept {
  auto fail=[&](ShellPublicationReport report) {
    if (impl_ && impl_->physical && impl_->physical->owner)
      impl_->physical->owner->Discard();
    else owner.Discard();
    if (impl_) impl_->Discard();
    return report;
  };
  if (!impl_ || !impl_->physical)
    return fail({S::NotInitialized,"Physical publication is not initialized"});
  auto& state=*impl_;
  auto& physical=*state.physical;
  auto* participation=physical.ScratchParticipation();
  const auto slot=PhysicalScratchKindIndex(issuer.kind_);
  if (!participation || slot>=PhysicalScratchKindCount ||
      participation->entries[slot].issuer!=&issuer ||
      participation->entries[slot].source_id!=issuer.source_id_ ||
      issuer.publication_!=this || issuer.owner_!=physical.owner ||
      issuer.witness_count_!=participation->cin_witness_count)
    return fail({S::ParticipationFailure,
                 "Scratch candidate issuer is foreign or not in the fixed roster"});
  if (&owner!=physical.owner)
    return fail({S::ParticipationFailure,
                 "Scratch candidate belongs to a foreign physical owner"});
  if (!source_id || source_id!=issuer.source_id_)
    return fail({S::ParticipationFailure,
                 "Scratch candidate source identity differs from registration"});
  using trial_identity::Disjoint;
  if (!output || !state.PhysicalOutputDisjoint(output,sizeof(*output)) ||
      !Disjoint(output,sizeof(*output),&issuer,sizeof(issuer)) ||
      !Disjoint(output,sizeof(*output),&token,sizeof(token)) ||
      !Disjoint(output,sizeof(*output),&prepared,sizeof(prepared)))
    return fail({S::InvalidInput,
                 "Scratch candidate receipt output overlaps retained input"});
  if (!state.pending ||
      issuer.phase_!=ShellPhysicalScratchParticipation::Phase::AssemblyRecorded)
    return fail({S::ParticipationFailure,
                 "Scratch candidate lacks one same-attempt accepted assembly"});
  NodalPreparedView authentic;
  const auto borrowed=owner.BorrowPrepared(token,&authentic);
  if (borrowed.status!=NodalStatus::Ok)
    return fail({S::ParticipationFailure,borrowed.message,borrowed.status});
  if (!trial_identity::SamePrepared(prepared,authentic) ||
      !trial_identity::SamePrepared(authentic,state.candidate_view) ||
      prepared.owner_id!=issuer.owner_id_ ||
      prepared.kinematics.base_epoch!=issuer.base_epoch_ ||
      prepared.attempt!=issuer.attempt_ || prepared.stream!=issuer.stream_)
    return fail({S::ParticipationFailure,
                 "Scratch candidate differs from its exact assembly and prepared token"});
  if (issuer.generation_==std::numeric_limits<std::uint64_t>::max())
    return fail({S::ParticipationFailure,
                 "Scratch participation generation is exhausted"});
  ShellPhysicalScratchParticipationReceipt next;
  next.issuer_=&issuer;
  next.publication_=this;
  next.owner_=&owner;
  next.kind_=issuer.kind_;
  next.source_id_=issuer.source_id_;
  next.generation_=issuer.generation_+1;
  next.prepared_=authentic;
  issuer.generation_=next.generation_;
  issuer.phase_=ShellPhysicalScratchParticipation::Phase::CandidateSealed;
  *output=next;
  return Ok();
}

ShellPublicationReport ShellBatchPublication::SealPhysicalScratchParticipation(
    FENodalState& owner,const NodalTrialToken& token,
    const ShellPhysicalScratchReceiptRoster& receipts) noexcept {
  auto fail=[&](ShellPublicationReport report) {
    if (impl_ && impl_->physical && impl_->physical->owner)
      impl_->physical->owner->Discard();
    else owner.Discard();
    if (impl_) impl_->Discard();
    return report;
  };
  if (!impl_ || !impl_->physical)
    return fail({S::NotInitialized,"Physical publication is not initialized"});
  auto& state=*impl_;
  auto& physical=*state.physical;
  auto* participation=physical.ScratchParticipation();
  if (!participation) {
    if (!receipts.mapped_wall && !receipts.self_contact) return Ok();
    return fail({S::ParticipationFailure,
                 "Unexpected scratch receipt supplied without a configured roster"});
  }
  if (&owner!=physical.owner || !state.pending)
    return fail({S::ParticipationFailure,
                 "Scratch roster seal requires the actual prepared physical owner"});
  if (participation->sealed)
    return fail({S::ParticipationFailure,
                 "Scratch participation roster was sealed more than once"});
  if (receipts.mapped_wall && receipts.self_contact &&
      receipts.mapped_wall==receipts.self_contact)
    return fail({S::ParticipationFailure,
                 "One scratch receipt cannot satisfy two configured kinds"});
  NodalPreparedView authentic;
  const auto borrowed=owner.BorrowPrepared(token,&authentic);
  if (borrowed.status!=NodalStatus::Ok)
    return fail({S::ParticipationFailure,borrowed.message,borrowed.status});
  if (!trial_identity::SamePrepared(authentic,state.candidate_view))
    return fail({S::ParticipationFailure,
                 "Scratch roster seal differs from structural preparation"});
  std::uint64_t generations[PhysicalScratchKindCount]{};
  for (std::size_t slot=0;slot<PhysicalScratchKindCount;++slot) {
    const auto kind=Kind(slot);
    const auto& expected=participation->entries[slot];
    const auto* receipt=ReceiptEntry(receipts,kind);
    if (!expected.source_id) {
      if (receipt)
        return fail({S::ParticipationFailure,
                     "Unexpected contact kind supplied to the fixed roster"});
      continue;
    }
    if (!expected.issuer || !receipt)
      return fail({S::ParticipationFailure,
                   "Configured contact kind is missing its final receipt"});
    const auto& issuer=*expected.issuer;
    if (!receipt->valid() || receipt->issuer_!=expected.issuer ||
        receipt->publication_!=this || receipt->owner_!=&owner ||
        receipt->kind_!=kind || receipt->source_id_!=expected.source_id ||
        receipt->generation_!=issuer.generation_ ||
        issuer.phase_!=ShellPhysicalScratchParticipation::Phase::CandidateSealed ||
        issuer.publication_!=this || issuer.owner_!=&owner ||
        issuer.kind_!=kind || issuer.source_id_!=expected.source_id ||
        issuer.witness_count_!=participation->cin_witness_count ||
        issuer.owner_id_!=authentic.owner_id ||
        issuer.base_epoch_!=authentic.kinematics.base_epoch ||
        issuer.attempt_!=authentic.attempt || issuer.stream_!=authentic.stream ||
        !trial_identity::SamePrepared(receipt->prepared_,authentic))
      return fail({S::ParticipationFailure,
                   "Scratch receipt is stale, foreign, replayed or mismatched"});
    generations[slot]=receipt->generation_;
  }
  participation->sealed_owner=&owner;
  participation->sealed_stream=authentic.stream;
  participation->sealed_owner_id=authentic.owner_id;
  participation->sealed_base_epoch=authentic.kinematics.base_epoch;
  participation->sealed_attempt=authentic.attempt;
  for (std::size_t slot=0;slot<PhysicalScratchKindCount;++slot)
    participation->sealed_generation[slot]=generations[slot];
  participation->sealed=true;
  return Ok();
}

ShellPublicationReport ShellBatchPublication::ValidatePhysicalScratchSeal(
    FENodalState& owner,const NodalPreparedView& authentic) const noexcept {
  if (!impl_ || !impl_->physical)
    return {S::NotInitialized,"Physical publication is not initialized"};
  const auto* participation=impl_->physical->ScratchParticipation();
  if (!participation) return Ok();
  if (!participation->sealed || participation->sealed_owner!=&owner ||
      participation->sealed_stream!=authentic.stream ||
      participation->sealed_owner_id!=authentic.owner_id ||
      participation->sealed_base_epoch!=authentic.kinematics.base_epoch ||
      participation->sealed_attempt!=authentic.attempt)
    return {S::ParticipationFailure,
            "Configured scratch participation is not sealed for this attempt"};
  for (std::size_t slot=0;slot<PhysicalScratchKindCount;++slot) {
    const auto& entry=participation->entries[slot];
    if (!entry.source_id) continue;
    if (!entry.issuer)
      return {S::ParticipationFailure,
              "Configured scratch issuer no longer belongs to the publication"};
    const auto& issuer=*entry.issuer;
    if (issuer.publication_!=this || issuer.owner_!=&owner ||
        issuer.kind_!=Kind(slot) || issuer.source_id_!=entry.source_id ||
        issuer.witness_count_!=participation->cin_witness_count ||
        issuer.phase_!=ShellPhysicalScratchParticipation::Phase::CandidateSealed ||
        issuer.owner_id_!=authentic.owner_id ||
        issuer.base_epoch_!=authentic.kinematics.base_epoch ||
        issuer.attempt_!=authentic.attempt || issuer.stream_!=authentic.stream ||
        !participation->sealed_generation[slot] ||
        participation->sealed_generation[slot]!=issuer.generation_)
      return {S::ParticipationFailure,
              "Configured scratch completion changed before common commit"};
  }
  return Ok();
}

void ShellBatchPublication::ConsumePhysicalScratchSeal() noexcept {
  if (!impl_ || !impl_->physical) return;
  auto* participation=impl_->physical->ScratchParticipation();
  if (!participation) return;
  for (std::size_t slot=0;slot<PhysicalScratchKindCount;++slot)
    if (participation->entries[slot].issuer)
      participation->entries[slot].issuer->Consume();
  participation->sealed=false;
  participation->sealed_owner=nullptr;
  participation->sealed_stream=nullptr;
  participation->sealed_owner_id=participation->sealed_base_epoch=
      participation->sealed_attempt=0;
  for (auto& generation:participation->sealed_generation) generation=0;
}

void ShellBatchPublication::ReleasePhysicalScratchParticipation(
    ShellPhysicalScratchParticipation& issuer) noexcept {
  if (!impl_ || !impl_->physical) {
    issuer.Unbind(this);
    return;
  }
  auto* participation=impl_->physical->ScratchParticipation();
  if (participation) {
    for (auto& entry:participation->entries)
      if (entry.issuer==&issuer) entry.issuer=nullptr;
    participation->sealed=false;
  }
  issuer.Unbind(this);
}

}  // namespace tl::fea
