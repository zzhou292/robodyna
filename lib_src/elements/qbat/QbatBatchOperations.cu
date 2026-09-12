// SPDX-License-Identifier: AGPL-3.0-or-later
#include "QbatBatchStorage.h"
#include "../../solvers/NodalNativePhysicalCoefficients.h"
#include <cmath>

namespace tl::fea::qbat {
namespace {
BatchReport Nodal(const NodalReport& report) noexcept {
  return {report.status==NodalStatus::DeviceFailure?BatchStatus::NodalFailure:BatchStatus::StaleTrial,
      report.message,UINT32_MAX,report.node,Status::kSuccess,report.status};
}
BatchReport MarkRejected(const NodalAssemblyView& view,BatchReport report) {
  if(view.result&&view.bounds) {
    cudaGetLastError();
    batch_detail::LaunchFailure(view);
    if(cudaGetLastError()!=cudaSuccess||cudaStreamSynchronize(view.stream)!=cudaSuccess) {
      return {BatchStatus::DeviceFailure,"Cannot mark rejected QBAT contribution"};
    }
  }
  return report;
}
}
BatchReport Batch::AssembleAccepted(FENodalState& owner,const NodalAssemblyView& view) {
  if(!impl_) return {BatchStatus::NotInitialized,"QBAT batch is not initialized"};
  auto& s=*impl_;
  if(s.physical) return {BatchStatus::InvalidInput,"Mapped QBAT requires token-aware CIN assembly"};
  s.Discard();
  if(!s.usable) return {BatchStatus::DeviceFailure,"QBAT CUDA storage is poisoned"};
  // Authenticate before borrowed device reads or any sticky assembly write.
  const auto authenticated=native_physical_coefficients::AuthenticateAccepted(owner,s.accepted_stamp,view);
  if(authenticated.status!=NodalStatus::Ok) {
    if(authenticated.status==NodalStatus::DeviceFailure) s.usable=false;
    return Nodal(authenticated);
  }
  auto fail=[&](BatchReport report) {
    auto marked=MarkRejected(view,report);
    if(marked.status==BatchStatus::DeviceFailure) s.usable=false;
    return marked;
  };
  auto report=s.PendingError();
  if(report.status!=BatchStatus::Success) return fail(report);
  const auto& accepted=s.accepted_stamp;
  const auto count=accepted.node_count;
  if(!trial_identity::ValidKinematics(view.accepted,count,accepted.epoch)||view.forces.node_count!=count||
      view.mass.node_count!=count||view.forces.base_epoch!=accepted.epoch||view.mass.base_epoch!=accepted.epoch||
      !view.mass.inverse_mass||!view.mass.fixed||!view.inverse_inertia||!view.translation_fixed_bits||
      !view.rotation_fixed||!view.bounds||!view.result||!view.forces.force_x||!view.forces.force_y||
      !view.forces.force_z||!view.forces.couple_x||!view.forces.couple_y||!view.forces.couple_z) {
    return fail({BatchStatus::InvalidInput,"Incomplete QBAT assembly view"});
  }
  if(view.temporal_scheme!=accepted.temporal_scheme||view.velocity_phase!=accepted.velocity_phase||
      view.position_time!=accepted.time||view.velocity_time!=accepted.velocity_time||!view.attempt||
      view.attempt<=s.assembled_attempt||(s.bound&&view.stream!=s.stream)) {
    return fail({BatchStatus::StaleTrial,"QBAT assembly phase/attempt/cache mismatch"});
  }
  s.assembled_attempt=view.attempt;
  s.assembled_epoch=UINT64_MAX;
  s.stream=view.stream;
  batch_detail::LaunchAssembly(s.storage,s.accepted,view,!s.bound);
  report=s.ReadControl();
  if(report.status!=BatchStatus::Success) return fail(report);
  if(!s.bound) s.initial_sources=view;
  s.bound=true;
  s.assembled_epoch=accepted.epoch;
  return {BatchStatus::Success,"OK"};
}
BatchReport Batch::EvaluateCandidate(FENodalState& owner,const NodalTrialToken& token,
    const NodalPreparedView& view,BatchDiagnostics* output) {
  if(!impl_) return {BatchStatus::NotInitialized,"QBAT batch is not initialized"};
  auto& s=*impl_;
  if(!s.OutputDisjoint(output,sizeof(*output))||!trial_identity::Disjoint(output,sizeof(*output),this,sizeof(*this))||
      !trial_identity::Disjoint(output,sizeof(*output),&owner,sizeof(owner))||
      !trial_identity::Disjoint(output,sizeof(*output),&token,sizeof(token))||
      !trial_identity::Disjoint(output,sizeof(*output),&view,sizeof(view))) {
    return {BatchStatus::InvalidInput,"QBAT diagnostic output overlaps an inspected input or owned data"};
  }
  s.Discard();
  auto report=s.PendingError();
  if(report.status!=BatchStatus::Success) return report;
  if(!s.bound) return {BatchStatus::NotBound,"QBAT live reference/mass binding is required"};
  const auto authenticated=native_physical_coefficients::AuthenticatePrepared(owner,token,s.accepted_stamp,view);
  if(authenticated.status!=NodalStatus::Ok) {
    if(authenticated.status==NodalStatus::DeviceFailure) s.usable=false;
    return Nodal(authenticated);
  }
  const auto& accepted=s.accepted_stamp;
  const auto count=accepted.node_count;
  const double dt=accepted.fixed_dt;
  if(!trial_identity::ValidKinematics(view.kinematics,count,accepted.epoch)||
      !trial_identity::ValidKinematics(view.base_kinematics,count,accepted.epoch)||
      view.temporal_scheme!=accepted.temporal_scheme||view.base_velocity_phase!=accepted.velocity_phase||
      view.velocity_phase!=NodalVelocityPhase::PreviousMidpoint||view.base_time!=accepted.time||
      view.base_velocity_time!=accepted.velocity_time||view.proposed_time!=accepted.time+dt||
      !std::isfinite(view.proposed_time)||view.proposed_time<=accepted.time||
      view.velocity_time!=accepted.time+.5*dt||view.kick_dt!=(accepted.epoch?dt:.5*dt)||
      view.stream!=s.stream||accepted.epoch==UINT64_MAX||!view.attempt||view.attempt<=s.last_candidate_attempt) {
    return {BatchStatus::StaleTrial,"QBAT prepared endpoint/phase/attempt differs from accepted history"};
  }
  const bool assembled=s.assembled_epoch==accepted.epoch&&s.assembled_attempt==view.attempt;
  if(assembled!=(s.config.usage==BatchUsage::CoupledForces)) {
    return {BatchStatus::StaleTrial,"QBAT usage differs from this attempt's force participation"};
  }
  s.last_candidate_attempt=view.attempt;
  const auto identity=batch_detail::CandidateIdentity(s.config,view);
  batch_detail::LaunchCandidate(s.storage,s.accepted,s.trial,view,identity,s.config.element_count,s.physical?count:0);
  report=s.ReadControl();
  if(report.status!=BatchStatus::Success) return report;
  s.candidate_diagnostics=s.control.diagnostics;
  s.candidate_view=view;
  s.pending=true;
  *output=s.candidate_diagnostics;
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::qbat
