// Lifecycle adapted from the qualified QEPH participant; no force equations.
#include "T3BatchStorage.h"
#include <cmath>

namespace tl::fea::t3 {
namespace {
BatchReport MarkRejected(const NodalAssemblyView& view,BatchReport report) {
  if(view.result&&view.bounds) {
    // An invalid launch error is recoverable, but the fresh assembly must still
    // be marked sticky. A context failure cannot promise device write recovery.
    cudaGetLastError(); batch_detail::LaunchFailure(view);
    if(cudaGetLastError()!=cudaSuccess||cudaStreamSynchronize(view.stream)!=cudaSuccess)
      return {BatchStatus::DeviceFailure,"Cannot mark rejected T3 contribution"};
  }
  return report;
}
}
BatchReport T3Batch::AssembleAccepted(const NodalAssemblyView& v) {
  if(!impl_) return MarkRejected(v,{BatchStatus::NotInitialized,"T3 batch is not initialized"});
  auto& s=*impl_; s.Discard();
  auto fail=[&](BatchReport r) { const auto marked=MarkRejected(v,r); if(marked.status==BatchStatus::DeviceFailure) s.usable=false; return marked; };
  auto report=s.PendingError(); if(report.status!=BatchStatus::Success) return fail(report);
  const auto& a=s.accepted_stamp; const auto n=a.node_count;
  if(v.owner_id!=a.owner_id) return fail({BatchStatus::WrongOwner,"T3 assembly belongs to another owner"});
  if(!batch_detail::ValidKinematics(v.accepted,n,a.epoch)||v.forces.node_count!=n||v.mass.node_count!=n||
     v.forces.base_epoch!=a.epoch||v.mass.base_epoch!=a.epoch||!v.mass.inverse_mass||!v.mass.fixed||
     !v.inverse_inertia||!v.translation_fixed_bits||!v.rotation_fixed||!v.bounds||!v.result||
     !v.forces.force_x||!v.forces.force_y||!v.forces.force_z||!v.forces.couple_x||!v.forces.couple_y||!v.forces.couple_z)
    return fail({BatchStatus::InvalidInput,"Incomplete T3 assembly view"});
  if(v.temporal_scheme!=a.temporal_scheme||v.velocity_phase!=a.velocity_phase||v.position_time!=a.time||
     v.velocity_time!=a.velocity_time||!v.attempt||v.attempt<=s.assembled_attempt||
     (s.bound&&v.stream!=s.stream))
    return fail({BatchStatus::StaleTrial,"T3 assembly phase/attempt/cache mismatch"});
  s.assembled_attempt=v.attempt; s.assembled_epoch=UINT64_MAX; s.stream=v.stream;
  batch_detail::LaunchAssembly(s.storage,s.accepted,v,!s.bound);
  report=s.ReadControl(); if(report.status!=BatchStatus::Success) return fail(report);
  if(!s.bound) s.initial_sources=v;
  s.bound=true; s.assembled_epoch=a.epoch;
  return {BatchStatus::Success,"OK"};
}
BatchReport T3Batch::EvaluateCandidate(const NodalPreparedView& v,BatchDiagnostics* output) {
  if(!impl_) return {BatchStatus::NotInitialized,"T3 batch is not initialized"};
  auto& s=*impl_; s.Discard();
  auto report=s.PendingError(); if(report.status!=BatchStatus::Success) return report;
  if(!s.bound) return {BatchStatus::NotBound,"Initial reference-at-rest mass binding is required"};
  if(!output) return {BatchStatus::InvalidInput,"Missing T3 diagnostic output"};
  const auto& a=s.accepted_stamp; const auto n=a.node_count; const auto h=a.fixed_dt;
  if(v.owner_id!=a.owner_id) return {BatchStatus::WrongOwner,"T3 candidate belongs to another owner"};
  if(!batch_detail::ValidKinematics(v.kinematics,n,a.epoch)||!batch_detail::ValidKinematics(v.base_kinematics,n,a.epoch)||
     v.temporal_scheme!=a.temporal_scheme||v.base_velocity_phase!=a.velocity_phase||
     v.velocity_phase!=NodalVelocityPhase::PreviousMidpoint||v.base_time!=a.time||
     v.base_velocity_time!=a.velocity_time||v.proposed_time!=a.time+h||!std::isfinite(v.proposed_time)||
     v.proposed_time<=a.time||v.velocity_time!=a.time+.5*h||v.kick_dt!=(a.epoch?h:.5*h)||
     v.stream!=s.stream||a.epoch==UINT64_MAX||!v.attempt||v.attempt<=s.last_candidate_attempt)
    return {BatchStatus::StaleTrial,"T3 prepared endpoint/phase/attempt does not match accepted material state"};
  const bool assembled=s.assembled_epoch==a.epoch&&s.assembled_attempt==v.attempt;
  if(assembled!=(s.config.usage==BatchUsage::CoupledForces))
    return {BatchStatus::StaleTrial,"T3 immutable usage does not match this attempt's force participation"};
  s.last_candidate_attempt=v.attempt;
  BatchDiagnostics d; d.owner_id=a.owner_id; d.configuration_id=s.config.configuration_id;
  d.qualification_id=s.config.qualification_id; d.epoch=a.epoch+1; d.base_epoch=a.epoch; d.attempt=v.attempt;
  d.time=v.proposed_time; d.base_time=v.base_time; d.velocity_time=v.velocity_time;
  d.base_velocity_time=v.base_velocity_time; d.kick_dt=v.kick_dt; d.phase=BatchPhase::Prepared;
  d.kinetic_available=!s.joined_binding.has_value();
  d.has_completed_interval=true; d.accepted_force_assembled=assembled; d.usage=s.config.usage;
  batch_detail::LaunchCandidate(s.storage,s.accepted,s.trial,v,d);
  report=s.ReadControl(); if(report.status!=BatchStatus::Success) return report;
  s.candidate_diagnostics=s.control.diagnostics; s.candidate_view=v; s.pending=true;
  *output=s.candidate_diagnostics; return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::t3
