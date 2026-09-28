// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeContactPublicationState.h"
#include "NativeContactActivitySelectors.h"
#include "ShellPhysicalScratchParticipation.h"
#include "../../solvers/NodalTrialIdentity.h"
namespace tl::fea {
NativeContactPublicationState::~NativeContactPublicationState() noexcept {
  if(issuer_)issuer_->DetachNativeContactState(*this);
}
bool NativeContactPublicationState::Attach(FENodalState& owner,std::uint64_t source,
    ShellPhysicalScratchParticipation& issuer,bool track_activity) noexcept {
  const auto stamp=owner.accepted();
  if(attached_||generation_||force_phase_available_||issuer.native_contact_||issuer.configured()||!source||!stamp.owner_id||stamp.epoch)
    return false;
  issuer_=&issuer;owner_=&owner;source_id_=source;accepted_stamp_=stamp;
  issuer_lifetime_=issuer.lifetime_id_;issuer.native_contact_=this;
  accepted_.activity_generation=track_activity?1:0;
  attached_=true;return true;
}
bool NativeContactPublicationState::CanBind(const FENodalState& owner,std::uint64_t source) const noexcept {
  const auto stamp=owner.accepted();
  return attached_&&!publication_&&issuer_&&owner_==&owner&&source_id_==source&&
      issuer_lifetime_==issuer_->lifetime_id_&&trial_identity::SameStamp(accepted_stamp_,stamp)&&
      accepted_stamp_.epoch==0&&stamp.epoch==0&&generation_==0;
}
void NativeContactPublicationState::Bind(ShellBatchPublication& publication,FENodalState& owner,
    std::uint64_t lifetime,std::uint64_t binding) noexcept {
  publication_=&publication;owner_=&owner;issuer_lifetime_=lifetime;binding_id_=binding;Discard();
}
void NativeContactPublicationState::Unbind() noexcept {
  publication_=nullptr;binding_id_=0;Discard();
}
bool NativeContactPublicationState::Stage(const NodalPreparedView& view,NativeContactSelectors next) noexcept {
  if(!publication_||!issuer_||pending_||generation_==UINT64_MAX||
     issuer_->phase_!=ShellPhysicalScratchParticipation::Phase::AssemblyRecorded||
     issuer_->generation_==UINT64_MAX||issuer_->owner_id_!=view.owner_id||
     issuer_->base_epoch_!=view.kinematics.base_epoch||issuer_->attempt_!=view.attempt||
     issuer_->stream_!=view.stream||view.owner_id!=accepted_stamp_.owner_id||
     view.kinematics.base_epoch!=accepted_stamp_.epoch||!view.attempt||
     next.history!=(accepted_.history^1u)||next.reference>1||!next.has_reference||!next.reference_generation||
     !native_contact_publication::ValidActivityPlan(accepted_,next))
    return false;
  const bool reuse=accepted_.has_reference&&next.reference==accepted_.reference&&
      next.reference_generation==accepted_.reference_generation&&
      next.reference_activity_generation==accepted_.reference_activity_generation;
  const bool rebuilt=accepted_.reference_generation!=UINT64_MAX&&
      next.reference_generation==accepted_.reference_generation+1&&
      (!accepted_.has_reference||next.reference!=accepted_.reference);
  if(!reuse&&!rebuilt)return false;
  prepared_=view;staged_=next;staged_issuer_generation_=issuer_->generation_+1;pending_=true;return true;
}
bool NativeContactPublicationState::Ready(const FENodalState& owner,const NodalPreparedView& view,
    std::uint64_t issuer_generation) const noexcept {
  const auto stamp=owner.accepted();
  return pending_&&publication_&&owner_==&owner&&issuer_&&
      issuer_lifetime_==issuer_->lifetime_id_&&binding_id_==issuer_->binding_id_&&
      source_id_==issuer_->source_id_&&generation_!=UINT64_MAX&&
      trial_identity::SameStamp(stamp,accepted_stamp_)&&
      staged_issuer_generation_==issuer_generation&&trial_identity::SamePrepared(prepared_,view);
}
void NativeContactPublicationState::Discard() noexcept {
  pending_=false;staged_issuer_generation_=0;prepared_={};staged_={};
}
void NativeContactPublicationState::Publish(const NodalStamp& stamp) noexcept {
  // Common commit already validated every field. Infallible selector stores
  // only: no CUDA, allocation, caller callback, arithmetic admission or status.
  accepted_=staged_;force_base_stamp_=accepted_stamp_;force_phase_available_=true;
  accepted_stamp_=stamp;++generation_;Discard();
}
NativeContactPublicationSnapshot NativeContactPublicationState::Accepted(const FENodalState& owner) const noexcept {
  NativeContactPublicationSnapshot out;if(!publication_||owner_!=&owner)return out;
  const auto stamp=owner.accepted();
  if(!trial_identity::SameStamp(stamp,accepted_stamp_))return out;
  out.stamp=accepted_stamp_;out.force_base_stamp=force_base_stamp_;out.force_phase_available=force_phase_available_;
  out.selectors=accepted_;out.generation=generation_;out.available=true;return out;
}
} // namespace tl::fea
