// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type25BatchStorage.h"
#include "../../solvers/NodalNativePhysicalCoefficients.h"
#include <cmath>

namespace tl::fea::type25 {
namespace {
BatchReport Nodal(const NodalReport& r) noexcept {
  return {r.status==NodalStatus::DeviceFailure?BatchStatus::NodalFailure:BatchStatus::StaleTrial,
          r.message,UINT32_MAX,r.node,Status::Success,r.status};
}
BatchReport MarkRejected(const NodalAssemblyView& view,BatchReport report) {
  if(view.result&&view.bounds) {
    cudaGetLastError();batch_detail::LaunchFailure(view);
    if(cudaGetLastError()!=cudaSuccess||cudaStreamSynchronize(view.stream)!=cudaSuccess)
      return {BatchStatus::DeviceFailure,"Cannot mark rejected TYPE25 contribution"};
  }
  return report;
}
}
BatchReport Batch::AssembleAccepted(FENodalState& owner,const NodalAssemblyView& view) {
  if(!impl_)return {BatchStatus::NotInitialized,"TYPE25 batch is not initialized"};
  auto& s=*impl_;
  if(s.physical)return {BatchStatus::InvalidInput,"Mapped TYPE25 requires its token-bound assembly API"};
  s.Discard();
  if(!s.usable)return {BatchStatus::Unusable,"TYPE25 CUDA storage is poisoned"};
  // Always authenticate actual immutable sources before any borrowed device
  // read or sticky failure write, including the unconstrained startup path.
  const auto authenticated=native_physical_coefficients::AuthenticateAccepted(owner,s.accepted_stamp,view);
  if(authenticated.status!=NodalStatus::Ok) {
    if(authenticated.status==NodalStatus::DeviceFailure)s.usable=false;
    return Nodal(authenticated);
  }
  auto fail=[&](BatchReport report) {auto r=MarkRejected(view,report);if(r.status==BatchStatus::DeviceFailure)s.usable=false;return r;};
  auto report=s.PendingError();if(report.status!=BatchStatus::Success)return fail(report);
  const auto& a=s.accepted_stamp;const auto n=a.node_count;
  if(!trial_identity::ValidKinematics(view.accepted,n,a.epoch)||view.forces.node_count!=n||view.mass.node_count!=n||
     view.forces.base_epoch!=a.epoch||view.mass.base_epoch!=a.epoch||!view.mass.inverse_mass||!view.mass.fixed||
     !view.inverse_inertia||!view.translation_fixed_bits||!view.rotation_fixed||!view.bounds||!view.result||
     !view.forces.force_x||!view.forces.force_y||!view.forces.force_z||!view.forces.couple_x||!view.forces.couple_y||!view.forces.couple_z)
    return fail({BatchStatus::InvalidInput,"Incomplete TYPE25 assembly view"});
  if(view.owner_id!=a.owner_id||view.temporal_scheme!=a.temporal_scheme||view.velocity_phase!=a.velocity_phase||
     view.position_time!=a.time||view.velocity_time!=a.velocity_time||!view.attempt||view.attempt<=s.assembled_attempt||
     (s.bound&&view.stream!=s.stream))return fail({BatchStatus::StaleTrial,"TYPE25 assembly phase/attempt/cache mismatch"});
  if(!s.bound)for(std::size_t e=0;e<s.config.element_count;++e) {
    const auto free_nodes=owner.ValidateNonRigidNodes(s.source->connections()[e].global_node,2);
    if(free_nodes.status!=NodalStatus::Ok) {
      auto rejected=Nodal(free_nodes);rejected.element=static_cast<std::uint32_t>(e);
      rejected.status=free_nodes.status==NodalStatus::DeviceFailure?BatchStatus::NodalFailure:BatchStatus::InvalidInput;
      if(free_nodes.status==NodalStatus::DeviceFailure)s.usable=false;
      return fail(rejected);
    }
  }
  s.assembled_attempt=view.attempt;s.assembled_epoch=UINT64_MAX;s.stream=view.stream;
  batch_detail::LaunchAssembly(s.storage,s.accepted,view,!s.bound);
  report=s.ReadControl();if(report.status!=BatchStatus::Success)return fail(report);
  s.bound=true;s.assembled_epoch=a.epoch;return {BatchStatus::Success,"OK"};
}
BatchReport Batch::EvaluateCandidate(FENodalState& owner,const NodalTrialToken& token,
                                    const NodalPreparedView& view,BatchDiagnostics* output) {
  if(!impl_)return {BatchStatus::NotInitialized,"TYPE25 batch is not initialized"};
  auto& s=*impl_;
  if(!s.OutputDisjoint(output,sizeof(*output))||!trial_identity::Disjoint(output,sizeof(*output),this,sizeof(*this))||
     !trial_identity::Disjoint(output,sizeof(*output),&owner,sizeof(owner))||
     !trial_identity::Disjoint(output,sizeof(*output),&token,sizeof(token))||
     !trial_identity::Disjoint(output,sizeof(*output),&view,sizeof(view)))
    return {BatchStatus::InvalidInput,"TYPE25 diagnostic output overlaps an inspected input"};
  s.Discard();auto report=s.PendingError();if(report.status!=BatchStatus::Success)return report;
  if(!s.bound)return {BatchStatus::NotBound,"TYPE25 original geometry/combined mass binding is required"};
  const auto authenticated=native_physical_coefficients::AuthenticatePrepared(owner,token,s.accepted_stamp,view);
  if(authenticated.status!=NodalStatus::Ok) {
    if(authenticated.status==NodalStatus::DeviceFailure)s.usable=false;
    return Nodal(authenticated);
  }
  const auto& a=s.accepted_stamp;const auto n=a.node_count;const double h=a.fixed_dt;
  if(!trial_identity::ValidKinematics(view.kinematics,n,a.epoch)||!trial_identity::ValidKinematics(view.base_kinematics,n,a.epoch)||
     view.temporal_scheme!=a.temporal_scheme||view.base_velocity_phase!=a.velocity_phase||
     view.velocity_phase!=NodalVelocityPhase::PreviousMidpoint||view.base_time!=a.time||view.base_velocity_time!=a.velocity_time||
     view.proposed_time!=a.time+h||!std::isfinite(view.proposed_time)||view.proposed_time<=a.time||
     view.velocity_time!=a.time+.5*h||view.kick_dt!=(a.epoch?h:.5*h)||view.stream!=s.stream||a.epoch==UINT64_MAX||
     !view.attempt||view.attempt<=s.last_candidate_attempt||s.assembled_epoch!=a.epoch||s.assembled_attempt!=view.attempt)
    return {BatchStatus::StaleTrial,"TYPE25 candidate does not match this accepted RHS interval"};
  s.last_candidate_attempt=view.attempt;
  BatchDiagnostics d;d.source_instance_id=s.source->source_instance_id();d.owner_id=a.owner_id;
  d.configuration_id=s.config.configuration_id;d.qualification_id=s.config.qualification_id;
  d.epoch=a.epoch+1;d.base_epoch=a.epoch;d.attempt=view.attempt;d.time=view.proposed_time;d.base_time=view.base_time;
  d.velocity_time=view.velocity_time;d.base_velocity_time=view.base_velocity_time;d.kick_dt=view.kick_dt;
  d.phase=BatchPhase::Prepared;d.has_completed_interval=true;d.accepted_force_assembled=true;
  batch_detail::LaunchCandidate(s.storage,s.accepted,s.trial,view,d,s.config.element_count);
  report=s.ReadControl();if(report.status!=BatchStatus::Success)return report;
  s.candidate_diagnostics=s.control.diagnostics;s.candidate_view=view;s.pending=true;
  *output=s.candidate_diagnostics;return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::type25
