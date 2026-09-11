// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../T3BatchStorage.h"
#include "../../ShellPhysicalOwner.h"

namespace tl::fea::t3 {
BatchReport T3Batch::AssembleMappedAccepted(FENodalState& owner,const NodalTrialToken& token,
    const NodalAssemblyView& view) {
  if (!impl_) return {BatchStatus::NotInitialized,"T3 batch is not initialized"};
  auto& state=*impl_;
  if (!state.physical) return {BatchStatus::InvalidInput,"T3 batch has no mapped physical profile"};
  state.Discard();
  auto report=state.PendingError();
  if (report.status!=BatchStatus::Success) return report;
  NodalCinAssemblyView cin;
  const auto borrowed=shell_physical_owner::BorrowAssembly(owner,token,state.accepted_stamp,
      view,state.cin_witness_count,&cin);
  if (borrowed.status!=NodalStatus::Ok) {
    if (borrowed.status==NodalStatus::DeviceFailure) state.usable=false;
    // A token rejection can expire the old view; do not write through it.
    owner.Discard();
    return {borrowed.status==NodalStatus::DeviceFailure?BatchStatus::DeviceFailure:BatchStatus::StaleTrial,
        borrowed.message,UINT32_MAX,borrowed.node,Status::kSuccess,borrowed.status};
  }
  auto reject=[&](BatchReport failure) {
    batch_detail::LaunchFailure(view);
    auto marked=state.Runtime(cudaGetLastError(),"Mapped T3 rejection launch failed");
    if (marked.status!=BatchStatus::Success) return marked;
    marked=state.Runtime(cudaStreamSynchronize(view.stream),"Mapped T3 rejection stream failed");
    return marked.status==BatchStatus::Success?failure:marked;
  };
  const auto& accepted=state.accepted_stamp;
  const auto count=accepted.node_count;
  if (!trial_identity::ValidKinematics(view.accepted,count,accepted.epoch) ||
      view.forces.node_count!=count || view.mass.node_count!=count ||
      view.forces.base_epoch!=accepted.epoch || view.mass.base_epoch!=accepted.epoch ||
      !view.mass.inverse_mass || !view.mass.fixed || !view.inverse_inertia ||
      !view.translation_fixed_bits || !view.rotation_fixed || !view.bounds || !view.result ||
      !view.forces.force_x || !view.forces.force_y || !view.forces.force_z ||
      !view.forces.couple_x || !view.forces.couple_y || !view.forces.couple_z ||
      (accepted.has_rotation_presence && !view.rotation_present)) {
    return reject({BatchStatus::InvalidInput,"Mapped T3 assembly view is incomplete"});
  }
  if (view.temporal_scheme!=accepted.temporal_scheme || view.velocity_phase!=accepted.velocity_phase ||
      view.position_time!=accepted.time || view.velocity_time!=accepted.velocity_time ||
      !view.attempt || view.attempt<=state.assembled_attempt ||
      (state.bound && view.stream!=state.stream)) {
    return reject({BatchStatus::StaleTrial,"Mapped T3 assembly phase/attempt differs"});
  }
  state.assembled_attempt=view.attempt;
  state.assembled_epoch=UINT64_MAX;
  state.stream=view.stream;
  // A discarded attempt binds the source but does not create an accepted force
  // packet. Every epoch-zero retry still requires the virgin native stiffness.
  batch_detail::LaunchMappedAssembly(state.storage,state.accepted,view,cin,state.plasticity->mixed_device(),accepted.epoch==0);
  report=state.ReadControl();
  if (report.status!=BatchStatus::Success) return report;
  if (!state.bound) state.initial_sources=view;
  state.bound=true;
  state.assembled_epoch=accepted.epoch;
  return {BatchStatus::Success,"Accepted T3 force and native CIN stiffness assembled"};
}
} // namespace tl::fea::t3
