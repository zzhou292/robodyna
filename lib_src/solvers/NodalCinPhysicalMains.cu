// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCinPhysicalMains.h"
#include "cin_physical_mains/Values.h"
#include "cin_physical_mains/OutputRanges.h"
#include "FENodalStateStorage.h"

namespace tl::fea {
namespace {
namespace values = cin_physical_mains;
__global__ void StagePhysicalMains(nodal_detail::Control* control,
    rigid::GroupDeviceView groups, const double* accepted,
    const double* translation, const double* rotation, std::uint32_t nodes,
    double* output) {
  if (control->status != NodalStatus::Ok) return;
  for (std::uint32_t group = 0; group < groups.group_count; ++group) {
    values::Values next;
    std::uint32_t invalid_node = UINT32_MAX;
    if (!values::Reduce(groups,group,accepted,translation,rotation,nodes,next,invalid_node)) {
      control->status = NodalStatus::InvalidOutput;
      control->node = invalid_node;
      return;
    }
    values::Write(output+values::ValuesPerGroup*group,next);
  }
}
NodalCinPhysicalMain Row(const RigidBindingGroup& source, const values::Values& value) noexcept {
  return {source.source_kind,source.source_id,source.source_node_set_id,value.center,
    value.mass,value.minimum_inertia,value.translation,value.rotation};
}
} // namespace

NodalReport FENodalState::CopyPreparedCinPhysicalMains(const NodalTrialToken& token,
    NodalCinPhysicalMainBuffer output, NodalCinPhysicalMainStamp* stamp) {
  if (!impl_) return {NodalStatus::NotInitialized,"Owner is not initialized"};
  auto& state = *impl_;
  if (!state.usable) return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  if (!state.cin || !state.rigid_groups)
    return {NodalStatus::InvalidInput,"Physical main summary requires CIN and actual rigid groups"};
  if (!state.Matches(token.owner_id_,token.base_epoch_,token.attempt_))
    return {NodalStatus::StaleTrial,"Physical main token is not the current owner attempt"};
  if (state.stamp.epoch != 0 ||
      (state.phase != nodal_detail::Phase::AwaitingValidation && state.phase != nodal_detail::Phase::Ready))
    return {NodalStatus::WrongPhase,"Physical main summary requires prepared TT0 before first acceptance"};
  const auto& cin = *state.cin;
  const auto& rigid = *state.rigid_groups;
  const auto n = state.config.node_count;
  const auto count = rigid.info.group_count;
  if (!output.groups || !stamp) return {NodalStatus::InvalidInput,"Missing physical main output"};
  if (output.capacity_groups < count)
    return {NodalStatus::ResourceLimit,"Physical main output must hold every actual rigid group"};
  if (count != rigid.properties.size() || count != rigid.device.group_count ||
      !values::Fits(n,count,cin.layout.scratch_values,state.staging.size()))
    return {NodalStatus::InvalidInput,"Physical main source or preallocated scratch shape is invalid"};
  const std::size_t bytes = count*sizeof(NodalCinPhysicalMain);
  const void* destinations[]{output.groups,stamp};
  const std::size_t lengths[]{bytes,sizeof(*stamp)};
  if (!trial_identity::Disjoint(output.groups,bytes,stamp,sizeof(*stamp)))
    return {NodalStatus::InvalidInput,"Physical main outputs overlap"};
  for (unsigned i = 0; i < 2; ++i) {
    if (!values::Outside(destinations[i],lengths[i],&token) ||
        !values::Outside(destinations[i],lengths[i],this) ||
        !values::Outside(destinations[i],lengths[i],impl_.get()) ||
        !values::OutsideSources(destinations[i],lengths[i],cin,rigid,
            state.staging,state.constraint_staging))
      return {NodalStatus::InvalidInput,"Physical main output overlaps owner, token or retained source"};
  }
  // Entry-inertia and A/AR are dead after the completed CIN motion stage. Any
  // requested force-stage A/AR capture already resides in separate owner scratch.
  // Preserve both stiffness arrays for repeat reads and later candidate users.
  auto* device_values = cin.work+2*n;
  StagePhysicalMains<<<1,1,0,state.stream>>>(state.control,rigid.device,state.accepted,
      cin.work,cin.work+n,std::uint32_t(n),device_values);
  auto report = state.Check(cudaGetLastError());
  if (report.status != NodalStatus::Ok) return report;
  report = state.SynchronizeControl();
  if (report.status != NodalStatus::Ok) return report;
  report = state.Check(cudaMemcpyAsync(state.staging.data(),device_values,
      values::ValuesPerGroup*count*sizeof(double),cudaMemcpyDeviceToHost,state.stream));
  if (report.status != NodalStatus::Ok) return report;
  report = state.Check(cudaStreamSynchronize(state.stream));
  if (report.status != NodalStatus::Ok) return report;
  for (std::size_t group = 0; group < count; ++group) {
    const auto value = values::Read(state.staging.data()+values::ValuesPerGroup*group);
    const auto& source = rigid.properties[group];
    const auto inertia = source.principal.inertia;
    const double minimum = ::fmin(inertia.x,::fmin(inertia.y,inertia.z));
    if (!values::Valid(value) || value.mass != source.mass_kg || value.minimum_inertia != minimum)
      return state.Reject(NodalStatus::InvalidOutput,"Physical main packet differs from actual aggregate");
  }
  const NodalCinPhysicalMainStamp next{NodalCinPhysicalMainPolicy::PhysicalAggregateV1,
    state.stamp.owner_id,state.stamp.epoch,state.attempt,cin.qualification_id,
    state.stamp.time,state.config.fixed_dt,rigid.info};
  // All fallible work is complete before either caller destination is written.
  for (std::size_t group = 0; group < count; ++group) {
    output.groups[group] = Row(rigid.properties[group],
        values::Read(state.staging.data()+values::ValuesPerGroup*group));
  }
  *stamp = next;
  return {NodalStatus::Ok,"Complete TT0 physical main summary copied"};
}
} // namespace tl::fea
