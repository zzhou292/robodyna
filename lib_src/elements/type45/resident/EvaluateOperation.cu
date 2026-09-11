// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OperationChecks.h"

namespace tl::fea::type45 {
BatchReport Batch::EvaluateCandidate(FENodalState& owner,const NodalTrialToken& token,
    const NodalPreparedView& view,BatchDiagnostics* output) {
  if(!impl_) return {BatchStatus::NotInitialized,"Joint batch is not initialized"};
  auto& s=*impl_;
  using trial_identity::Disjoint;
  if(reinterpret_cast<std::uintptr_t>(output)%alignof(BatchDiagnostics) ||
      !s.OutputDisjoint(output,sizeof(*output)) || !Disjoint(output,sizeof(*output),this,sizeof(*this)) ||
      !Disjoint(output,sizeof(*output),&owner,sizeof(owner)) || !Disjoint(output,sizeof(*output),&token,sizeof(token)) ||
      !Disjoint(output,sizeof(*output),&view,sizeof(view)))
    return {BatchStatus::InvalidInput,"Joint diagnostic output overlaps input or retained storage"};
  s.Discard();auto r=s.PendingError();if(!r) return r;
  if(!s.bound || !s.publication_scope) return {BatchStatus::NotBound,"Joint common publication claim required"};
  const auto checked=native_physical_coefficients::AuthenticatePrepared(owner,token,s.accepted_stamp,view);
  if(checked.status!=NodalStatus::Ok) {
    if(checked.status==NodalStatus::DeviceFailure) s.usable=false;
    return resident_detail::NodalFailure(checked);
  }
  const auto& stamp=s.accepted_stamp;
  if(!resident_detail::CandidatePhase(stamp,view) || view.stream!=s.stream || !view.attempt ||
      view.attempt<=s.last_candidate_attempt || s.assembled_epoch!=stamp.epoch || s.assembled_attempt!=view.attempt)
    return {BatchStatus::StaleTrial,"Joint candidate differs from its assembled accepted interval"};
  s.last_candidate_attempt=view.attempt;
  if(!stamp.epoch) {r=s.PrepareContexts(owner,token,view);if(!r) return r;}
  BatchDiagnostics identity;
  identity.source_instance_id=s.model.source_instance_id();identity.owner_id=stamp.owner_id;
  identity.configuration_id=s.config.configuration_id;identity.qualification_id=s.config.qualification_id;
  identity.epoch=stamp.epoch+1;identity.base_epoch=stamp.epoch;identity.attempt=view.attempt;
  identity.time=view.proposed_time;identity.base_time=view.base_time;identity.velocity_time=view.velocity_time;
  identity.base_velocity_time=view.base_velocity_time;identity.kick_dt=view.kick_dt;
  identity.phase=BatchPhase::Prepared;identity.has_completed_interval=true;identity.accepted_force_assembled=true;
  resident_detail::LaunchCandidate(s.device,s.accepted_slab,s.TrialSlab(),view,identity);
  r=s.ReadControl();if(!r) return r;
  s.candidate_diagnostics=s.control.diagnostics;s.candidate_view=view;s.pending=true;
  *output=s.candidate_diagnostics;return {};
}
} // namespace tl::fea::type45
