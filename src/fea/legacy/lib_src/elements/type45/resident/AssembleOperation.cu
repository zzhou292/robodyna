// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OperationChecks.h"

namespace tl::fea::type45 {
BatchReport Batch::AssembleAccepted(FENodalState& owner,const NodalTrialToken& token,const NodalAssemblyView& view) {
  if(!impl_) return {BatchStatus::NotInitialized,"Joint batch is not initialized"};
  auto& s=*impl_;s.Discard();
  auto r=s.PendingError();if(!r) return r;
  if(!s.bound || !s.publication_scope) return {BatchStatus::NotBound,"Joint common publication claim required"};
  const auto& stamp=s.accepted_stamp;
  const auto checked=native_physical_coefficients::AuthenticateAccepted(owner,stamp,view);
  if(checked.status!=NodalStatus::Ok) {
    if(checked.status==NodalStatus::DeviceFailure) s.usable=false;
    return resident_detail::NodalFailure(checked);
  }
  if(!resident_detail::CompleteAssembly(view,stamp) || view.owner_id!=stamp.owner_id ||
      view.temporal_scheme!=stamp.temporal_scheme || view.velocity_phase!=stamp.velocity_phase ||
      view.position_time!=stamp.time || view.velocity_time!=stamp.velocity_time ||
      !view.attempt || view.attempt<=s.assembled_attempt || (s.assembled_attempt && view.stream!=s.stream)) {
    owner.Discard();return {BatchStatus::StaleTrial,"Joint assembly phase or attempt differs"};
  }
  NodalCinAssemblyView cin;
  const auto borrowed=shell_physical_owner::BorrowAssembly(owner,token,stamp,view,s.config.cin_witness_count,&cin);
  if(borrowed.status!=NodalStatus::Ok) {
    if(borrowed.status==NodalStatus::DeviceFailure) s.usable=false;
    owner.Discard();return resident_detail::NodalFailure(borrowed);
  }
  if(s.cin_qualification_id && s.cin_qualification_id!=cin.qualification_id) {
    owner.Discard();return {BatchStatus::InvalidInput,"Joint actual CIN qualification changed"};
  }
  s.cin_qualification_id=cin.qualification_id;
  s.assembled_attempt=view.attempt;s.assembled_epoch=UINT64_MAX;s.stream=view.stream;
  resident_detail::LaunchAssembly(s.device,s.accepted_slab,view,cin);
  r=s.ReadControl();if(!r) {owner.Discard();return r;}
  s.assembled_epoch=stamp.epoch;return {};
}
} // namespace tl::fea::type45
