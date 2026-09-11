// SPDX-License-Identifier: AGPL-3.0-or-later
#include "AssemblyValues.h"
#include "../QephBatchStorage.h"
#include "../../ShellMixedSectionArenaLayout.h"
#include "../../../solvers/NodalCinRuntime.h"

namespace tl::fea::qeph::batch_detail {
namespace {
__global__ void BeginAssembly(Storage* storage,NodalAssemblyView view,NodalCinAssemblyView cin,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  auto& state=*storage;
  state.control={};
  const auto& memory=state.assembly;
  if (!state.model.mapped || !mixed || !memory.offsets || !memory.incidence ||
      !memory.parent || !memory.node || !memory.failure ||
      view.result->base_epoch!=view.accepted.base_epoch || view.result->attempt!=view.attempt ||
      view.bounds->base_epoch!=view.accepted.base_epoch || view.bounds->attempt!=view.attempt ||
      !view.bounds->initialized || !view.bounds->valid || view.bounds->sealed ||
      view.result->status!=tlfea::contact::Status::kOk ||
      !cin.translational_stiffness || !cin.rotational_stiffness ||
      cin.translational_stiffness==cin.rotational_stiffness || cin.node_count!=view.forces.node_count) {
    state.control.status=BatchStatus::AssemblyFailure;
    return;
  }
  const double* arrays[]{view.forces.force_x,view.forces.force_y,view.forces.force_z,
      view.forces.couple_x,view.forces.couple_y,view.forces.couple_z};
  for (unsigned channel=0;channel<6;++channel) for (unsigned prior=0;prior<channel;++prior) {
    if (arrays[channel]==arrays[prior]) state.control.status=BatchStatus::AssemblyFailure;
  }
  *memory.failure=mapped::NoAssemblyFailure;
}
__global__ void PrepareParents(Storage* storage,const Slab* accepted,NodalAssemblyView view,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed,bool initial) {
  auto& state=*storage;
  if (state.control.status!=BatchStatus::Success) return;
  for (std::size_t parent=blockIdx.x*blockDim.x+threadIdx.x;parent<state.model.config.element_count;
      parent+=blockDim.x*gridDim.x) {
    auto& record=state.assembly.parent[parent];
    record=mapped::PrepareAssemblyParent(state.model,accepted->element[parent],parent,mixed->law[parent],view,initial);
    if (record.status!=BatchStatus::Success) atomicMin(state.assembly.failure,2ull*parent);
  }
}
__global__ void GatherNodes(Storage* storage,const Slab* accepted,NodalAssemblyView view,
    NodalCinAssemblyView cin,const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  auto& state=*storage;
  if (state.control.status!=BatchStatus::Success) return;
  for (std::size_t node=blockIdx.x*blockDim.x+threadIdx.x;node<state.model.config.owner.node_count;
      node+=blockDim.x*gridDim.x) {
    const auto parent=mapped::GatherAssemblyNode(node,state.assembly,accepted->element,mixed->law,
        view.forces,cin.translational_stiffness,cin.rotational_stiffness,state.assembly.node[node]);
    if (parent!=UINT32_MAX) atomicMin(state.assembly.failure,2ull*parent+1);
  }
}
__global__ void FinishAssembly(Storage* storage,NodalAssemblyView view) {
  auto& state=*storage;
  if (state.control.status==BatchStatus::Success && *state.assembly.failure!=mapped::NoAssemblyFailure) {
    const auto key=*state.assembly.failure;
    const auto parent=static_cast<std::uint32_t>(key/2);
    const auto& record=state.assembly.parent[parent];
    state.control.status=key%2?BatchStatus::AssemblyFailure:record.status;
    state.control.node=key%2?UINT32_MAX:record.node;
    // The original node-validation branch reports its node, with no parent.
    if (state.control.status!=BatchStatus::InvalidInput) state.control.element=parent;
  }
  if (state.control.status!=BatchStatus::Success) {
    RecordNodalAssemblyFailure(view,tlfea::contact::Status::kInvalidArgument,state.control.node);
  }
}
__global__ void PublishNodes(Storage* storage,NodalAssemblyView view,NodalCinAssemblyView cin) {
  auto& state=*storage;
  if (state.control.status!=BatchStatus::Success) return;
  for (std::size_t node=blockIdx.x*blockDim.x+threadIdx.x;node<state.model.config.owner.node_count;
      node+=blockDim.x*gridDim.x) {
    mapped::PublishAssemblyNode(node,state.assembly.node[node],view.forces,
        cin.translational_stiffness,cin.rotational_stiffness);
  }
}
} // namespace
void LaunchMappedAssembly(Storage* storage,const Slab* accepted,NodalAssemblyView view,
    NodalCinAssemblyView cin,const shell_batch_plasticity_detail::MixedDeviceStorage* mixed,bool initial) {
  // Fixed bounded launch dimensions, grid-stride ownership, and one stream.
  // Only integer diagnostic minima are atomic; nodal sums retain source order.
  BeginAssembly<<<1,1,0,view.stream>>>(storage,view,cin,mixed);
  PrepareParents<<<256,128,0,view.stream>>>(storage,accepted,view,mixed,initial);
  GatherNodes<<<256,128,0,view.stream>>>(storage,accepted,view,cin,mixed);
  FinishAssembly<<<1,1,0,view.stream>>>(storage,view);
  PublishNodes<<<256,128,0,view.stream>>>(storage,view,cin);
}
} // namespace tl::fea::qeph::batch_detail
