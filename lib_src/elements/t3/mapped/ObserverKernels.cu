// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ObserverValues.h"
#include "../../mapped_shell/ObserverTree.cuh"

namespace tl::fea::t3::batch_detail {
namespace {
using mapped::ObserverSummary;
using mapped_shell::ReduceObserverBlock;
__global__ void PrepareMapped(Storage* storage, const Slab* trial, NodalPreparedView view,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  auto& state = *storage;
  const auto first = blockIdx.x * blockDim.x + threadIdx.x;
  const auto stride = gridDim.x * blockDim.x;
  const auto* roles = mixed ? mixed->law : nullptr;
  for (std::size_t parent = first; parent < state.model.config.element_count; parent += stride) {
    mapped::PrepareDiagnosticParent(state.model, trial->element[parent], state.candidate_status[parent],
        roles, parent, view, state.assembly.parent[parent]);
  }
  for (std::size_t node = first; node < state.model.config.owner.node_count; node += stride) {
    mapped::PrepareDiagnosticNode(state.model, view, node, state.assembly.node[node]);
  }
}
__global__ void ObserveMapped(Storage* storage, const Slab* accepted, const Slab* trial,
    NodalPreparedView view, BatchDiagnostics identity,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  auto& state = *storage;
  const auto blocks = mapped::ObserverBlocks(state.model.config.element_count, state.model.config.owner.node_count);
  if (blockIdx.x >= blocks) return;
  __shared__ ObserverSummary values[mapped::ObserverThreads];
  ObserverSummary out{};
  const auto first = blockIdx.x * blockDim.x + threadIdx.x;
  const auto stride = blocks * blockDim.x;
  const auto* roles = mixed ? mixed->law : nullptr;
  for (std::size_t parent = first; parent < state.model.config.element_count; parent += stride) {
    mapped::ObserveParent(state, *accepted, *trial, view, roles,
        identity.accepted_force_assembled, parent, out);
  }
  for (std::size_t node = first; node < state.model.config.owner.node_count; node += stride) {
    mapped::ObserveNode(state, node, out);
  }
  values[threadIdx.x] = out;
  ReduceObserverBlock(values);
  if (!threadIdx.x) state.assembly.observer[blockIdx.x] = values[0];
}
__global__ void FinishObservers(Storage* storage, const Slab* accepted, const Slab* trial,
    NodalPreparedView view, BatchDiagnostics identity,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  auto& state = *storage;
  const auto blocks = mapped::ObserverBlocks(state.model.config.element_count, state.model.config.owner.node_count);
  __shared__ ObserverSummary values[mapped::ObserverThreads];
  ObserverSummary out{};
  if (!blocks) out.serial = true;
  for (unsigned block = threadIdx.x; block < blocks; block += blockDim.x) {
    mapped::MergeObservations(out, state.assembly.observer[block]);
  }
  values[threadIdx.x] = out;
  ReduceObserverBlock(values);
  if (!threadIdx.x) {
    mapped::FinalizeObservations(state, *accepted, *trial, view, identity, mixed, values[0], state.control);
  }
}
} // namespace
void LaunchMappedObserverDiagnostics(Storage* storage, const Slab* accepted, const Slab* trial,
    NodalPreparedView view, BatchDiagnostics identity,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  PrepareMapped<<<256, 128, 0, view.stream>>>(storage, trial, view, mixed);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  ObserveMapped<<<mapped::ObserverMaxBlocks, mapped::ObserverThreads, 0, view.stream>>>(
      storage, accepted, trial, view, identity, mixed);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  FinishObservers<<<1, mapped::ObserverThreads, 0, view.stream>>>(storage, accepted, trial, view, identity, mixed);
}
} // namespace tl::fea::t3::batch_detail
