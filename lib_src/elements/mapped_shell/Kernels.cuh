// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodeGather.h"
#include "../ShellMixedSectionArenaLayout.h"
#include "../../solvers/NodalCinRuntime.h"

namespace tl::fea::mapped_shell {
// Two private family adapters supply storage types and their unchanged parent
// validation. This is the common ordered-gather schedule, not a participant API.
template<class Family>
__global__ void BeginAssembly(typename Family::Storage* storage, NodalAssemblyView view,
    NodalCinAssemblyView cin, const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  using BatchStatus = typename Family::BatchStatus;
  auto& state = *storage;
  state.control = {};
  const auto& memory = state.assembly;
  if (!state.model.mapped || !mixed || !memory.offsets || !memory.incidence ||
      !memory.parent || !memory.node || !memory.failure ||
      view.result->base_epoch != view.accepted.base_epoch || view.result->attempt != view.attempt ||
      view.bounds->base_epoch != view.accepted.base_epoch || view.bounds->attempt != view.attempt ||
      !view.bounds->initialized || !view.bounds->valid || view.bounds->sealed ||
      view.result->status != tlfea::contact::Status::kOk ||
      !cin.translational_stiffness || !cin.rotational_stiffness ||
      cin.translational_stiffness == cin.rotational_stiffness || cin.node_count != view.forces.node_count) {
    state.control.status = BatchStatus::AssemblyFailure;
    return;
  }
  const double* arrays[]{view.forces.force_x, view.forces.force_y, view.forces.force_z,
      view.forces.couple_x, view.forces.couple_y, view.forces.couple_z};
  for (unsigned channel = 0; channel < 6; ++channel) {
    for (unsigned prior = 0; prior < channel; ++prior) {
      if (arrays[channel] == arrays[prior]) state.control.status = BatchStatus::AssemblyFailure;
    }
  }
  *memory.failure = NoAssemblyFailure;
}

template<class Family>
__global__ void PrepareParents(typename Family::Storage* storage, const typename Family::Slab* accepted,
    NodalAssemblyView view, const shell_batch_plasticity_detail::MixedDeviceStorage* mixed, bool initial) {
  using BatchStatus = typename Family::BatchStatus;
  auto& state = *storage;
  if (state.control.status != BatchStatus::Success) return;
  for (std::size_t parent = blockIdx.x * blockDim.x + threadIdx.x;
      parent < state.model.config.element_count; parent += blockDim.x * gridDim.x) {
    auto& record = state.assembly.parent[parent];
    record = Family::Prepare(state.model, accepted->element[parent], parent, mixed->law[parent], view, initial);
    if (record.status != BatchStatus::Success) atomicMin(state.assembly.failure, 2ull * parent);
  }
}

template<class Family>
__global__ void GatherNodes(typename Family::Storage* storage, const typename Family::Slab* accepted,
    NodalAssemblyView view, NodalCinAssemblyView cin,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  using BatchStatus = typename Family::BatchStatus;
  auto& state = *storage;
  if (state.control.status != BatchStatus::Success) return;
  for (std::size_t node = blockIdx.x * blockDim.x + threadIdx.x;
      node < state.model.config.owner.node_count; node += blockDim.x * gridDim.x) {
    const auto parent = GatherNode<Family::Slots, typename Family::Memory,
        typename Family::ForceTrial, BatchStatus>(node, state.assembly, accepted->element, mixed->law,
        view.forces, cin.translational_stiffness, cin.rotational_stiffness, state.assembly.node[node]);
    if (parent != UINT32_MAX) atomicMin(state.assembly.failure, 2ull * parent + 1);
  }
}

template<class Family>
__global__ void FinishAssembly(typename Family::Storage* storage, NodalAssemblyView view) {
  using BatchStatus = typename Family::BatchStatus;
  auto& state = *storage;
  if (state.control.status == BatchStatus::Success && *state.assembly.failure != NoAssemblyFailure) {
    const auto key = *state.assembly.failure;
    const auto parent = static_cast<std::uint32_t>(key / 2);
    const auto& record = state.assembly.parent[parent];
    state.control.status = key % 2 ? BatchStatus::AssemblyFailure : record.status;
    state.control.node = key % 2 ? UINT32_MAX : record.node;
    // Original node validation reports its node with no parent. Other errors
    // retain the first source parent, before any destination is published.
    if (state.control.status != BatchStatus::InvalidInput) state.control.element = parent;
  }
  if (state.control.status != BatchStatus::Success) {
    RecordNodalAssemblyFailure(view, tlfea::contact::Status::kInvalidArgument, state.control.node);
  }
}

template<class Family>
__global__ void PublishNodes(typename Family::Storage* storage, NodalAssemblyView view,
    NodalCinAssemblyView cin) {
  using BatchStatus = typename Family::BatchStatus;
  auto& state = *storage;
  if (state.control.status != BatchStatus::Success) return;
  for (std::size_t node = blockIdx.x * blockDim.x + threadIdx.x;
      node < state.model.config.owner.node_count; node += blockDim.x * gridDim.x) {
    PublishNode(node, state.assembly.node[node], view.forces,
        cin.translational_stiffness, cin.rotational_stiffness);
  }
}

template<class Family>
void LaunchAssembly(typename Family::Storage* storage, const typename Family::Slab* accepted,
    NodalAssemblyView view, NodalCinAssemblyView cin,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed, bool initial) {
  // One stream, fixed grid-stride ownership. Only diagnostic integer minima are
  // atomic; every floating-point addition remains in source incidence order.
  BeginAssembly<Family><<<1, 1, 0, view.stream>>>(storage, view, cin, mixed);
  PrepareParents<Family><<<256, 128, 0, view.stream>>>(storage, accepted, view, mixed, initial);
  GatherNodes<Family><<<256, 128, 0, view.stream>>>(storage, accepted, view, cin, mixed);
  FinishAssembly<Family><<<1, 1, 0, view.stream>>>(storage, view);
  PublishNodes<Family><<<256, 128, 0, view.stream>>>(storage, view, cin);
}
} // namespace tl::fea::mapped_shell
