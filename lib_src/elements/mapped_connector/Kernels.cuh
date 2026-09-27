// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Values.h"
#include "../../solvers/NodalCinRuntime.h"
namespace tl::fea::mapped_connector {
template<class Family>
__global__ void Begin(typename Family::Storage* storage, NodalAssemblyView view) {
  auto& state = *storage;
  state.control = {};
  const auto& memory = state.assembly;
  if (!memory.offsets || !memory.incidence || !memory.touched_nodes || !memory.touched_count || !memory.parent || !memory.node || !memory.failure ||
      view.result->base_epoch != view.accepted.base_epoch || view.result->attempt != view.attempt ||
      view.bounds->base_epoch != view.accepted.base_epoch || view.bounds->attempt != view.attempt ||
      !view.bounds->initialized || !view.bounds->valid || view.bounds->sealed ||
      view.result->status != tlfea::contact::Status::kOk) {
    state.control.status = Family::AssemblyFailure();
    return;
  }
  *memory.failure = mapped_shell::NoAssemblyFailure;
}
template<class Family>
__global__ void PrepareParents(typename Family::Storage* storage, typename Family::Accepted accepted,
    NodalAssemblyView view, NodalCinAssemblyView cin, bool initial) {
  auto& state = *storage;
  if (state.control.status != Family::Success()) return;
  const auto count = Family::Count(state);
  for (std::size_t parent = blockIdx.x * blockDim.x + threadIdx.x;
      parent < count; parent += blockDim.x * gridDim.x) {
    auto& record = state.assembly.parent[parent];
    record = Family::Prepare(state, accepted, parent, view, cin, initial);
    if (record.failure != Failure::None)
      atomicMin(state.assembly.failure, ParentKey(Family::Ordering, count, parent, record.failure));
  }
}
template<class Family>
__global__ void GatherNodes(typename Family::Storage* storage, typename Family::Accepted accepted,
    NodalAssemblyView view, NodalCinAssemblyView cin) {
  auto& state = *storage;
  if (state.control.status != Family::Success()) return;
  for (std::size_t row = blockIdx.x * blockDim.x + threadIdx.x;
      row < state.assembly.touched_count; row += blockDim.x * gridDim.x) {
    const auto node = state.assembly.touched_nodes[row];
    const auto failed = GatherNode(node, state.assembly, Family::Values(state, accepted),
        view.forces, cin.translational_stiffness, cin.rotational_stiffness, state.assembly.node[row]);
    if (failed != UINT32_MAX)
      atomicMin(state.assembly.failure, AdditionKey(Family::Ordering, Family::Count(state), failed));
  }
}
template<class Family>
__global__ void Finish(typename Family::Storage* storage, NodalAssemblyView view) {
  auto& state = *storage;
  if (state.control.status == Family::Success() && *state.assembly.failure != mapped_shell::NoAssemblyFailure) {
    const auto key = *state.assembly.failure;
    const auto parent = FailedParent(Family::Ordering, Family::Count(state), key);
    const auto& record = state.assembly.parent[parent];
    state.control.status = key % 2 ? Family::AssemblyFailure() : Family::Status(record.failure);
    state.control.element = parent;
    if (!(key % 2) && record.failure == Failure::Endpoint) state.control.node = record.node;
  }
  if (state.control.status != Family::Success())
    RecordNodalAssemblyFailure(view, tlfea::contact::Status::kInvalidArgument,
        static_cast<std::uint32_t>(state.control.node));
}
template<class Family>
__global__ void Publish(typename Family::Storage* storage, NodalAssemblyView view, NodalCinAssemblyView cin) {
  auto& state = *storage;
  if (state.control.status != Family::Success()) return;
  for (std::size_t row = blockIdx.x * blockDim.x + threadIdx.x;
      row < state.assembly.touched_count; row += blockDim.x * gridDim.x) {
    const auto node = state.assembly.touched_nodes[row];
    mapped_shell::PublishNode(node, state.assembly.node[row], view.forces,
        cin.translational_stiffness, cin.rotational_stiffness);
  }
}
template<class Family>
void Launch(typename Family::Storage* storage, typename Family::Accepted accepted,
    NodalAssemblyView view, NodalCinAssemblyView cin, bool initial) {
  Begin<Family><<<1, 1, 0, view.stream>>>(storage, view);
  PrepareParents<Family><<<256, 128, 0, view.stream>>>(storage, accepted, view, cin, initial);
  GatherNodes<Family><<<256, 128, 0, view.stream>>>(storage, accepted, view, cin);
  Finish<Family><<<1, 1, 0, view.stream>>>(storage, view);
  Publish<Family><<<256, 128, 0, view.stream>>>(storage, view, cin);
}
} // namespace tl::fea::mapped_connector
