// SPDX-License-Identifier: AGPL-3.0-or-later
#include "AssemblySerial.cuh"
#include "AssemblyValues.h"
#include "AssemblyRanges.h"
namespace tl::fea::solids::batch_detail {
namespace {
__global__ void Prepare(Storage* storage, NodalAssemblyView view, NodalCinAssemblyView cin) {
  auto& state = *storage;
  state.control = {};
  state.assembly.fallback = 0;
  state.assembly.prepared = false;
  // Immutable metadata and range checks gate all parallel reads. Geometry
  // is checked separately; owner arrays remain untouched until Finish.
  if (view.result->base_epoch != view.accepted.base_epoch || view.result->attempt != view.attempt ||
      view.bounds->base_epoch != view.accepted.base_epoch || view.bounds->attempt != view.attempt ||
      !view.bounds->initialized || !view.bounds->valid || view.bounds->sealed ||
      view.result->status != tlfea::contact::Status::kOk ||
      !CanGatherAssembly(state, view, cin)) state.assembly.fallback = 1;
  state.assembly.prepared = !state.assembly.fallback;
}
__device__ void CheckGeometry(Storage& state, NodalAssemblyView view) {
  // prepared is immutable throughout CheckGeometry and Gather. Only fallback
  // changes, through integer atomics; neither kernel reads that flag.
  if (!state.assembly.prepared) return;
  const auto stride = std::size_t(gridDim.x)*blockDim.x;
  for (auto occurrence = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
       occurrence < state.assembly.occurrences; occurrence += stride) {
    AssemblyOccurrence value;
    if (!ReadAssemblyOccurrence<false>(state, 0, occurrence, value) ||
        value.node >= view.accepted.node_count ||
        !tl::math::fixed3::Finite(shell_batch_fields::ReadVector(
            view.accepted.position_xyz, value.node)) ||
        !tl::math::fixed3::Finite(shell_batch_fields::ReadVector(
            view.accepted.velocity_xyz, value.node)))
      atomicExch(&state.assembly.fallback, 1u);
  }
  // Geometry failure does not suppress private sums. Finish replays the
  // original serial check first, preserving its error priority and diagnostic.
}
__global__ void Gather(Storage* storage, unsigned slab, NodalAssemblyView view,
    NodalCinAssemblyView cin) {
  auto& state = *storage;
  // Validation and sums are independent private work. Finish waits for the
  // entire kernel before checking fallback or writing any owner array.
  // This kernel uses no shared floating-point atomics.
  if (!state.assembly.prepared) return;
  CheckGeometry(state, view);
  const auto node = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (node >= view.accepted.node_count) return;
  if (state.assembly.offsets[node] == state.assembly.offsets[node + 1]) return;
  if (!GatherAssemblyNode(state, slab, node, view.forces,
      cin.translational_stiffness, state.assembly.nodes[node]))
    atomicExch(&state.assembly.fallback, 1u);
}
__global__ void Finish(Storage* storage, unsigned slab, NodalAssemblyView view,
    NodalCinAssemblyView cin) {
  auto& state = *storage;
  if (state.assembly.fallback) {
    if (!blockIdx.x && !threadIdx.x) assembly_serial::Assemble(storage, slab, view, cin);
    return;
  }
  const auto node = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (node >= view.accepted.node_count ||
      state.assembly.offsets[node] == state.assembly.offsets[node + 1]) return;
  const auto& value = state.assembly.nodes[node];
  view.forces.force_x[node] = value.force[0];
  view.forces.force_y[node] = value.force[1];
  view.forces.force_z[node] = value.force[2];
  cin.translational_stiffness[node] = value.translation;
  // Solids never write couples or STIFR, including their signed-zero bits.
}
} // namespace
cudaError_t LaunchAssembly(Storage* storage, unsigned slab,
    NodalAssemblyView view, NodalCinAssemblyView cin) {
  constexpr unsigned threads = 128;
  auto blocks = unsigned((view.accepted.node_count+threads-1)/threads);
  if (!blocks) blocks = 1;
  Prepare<<<1, 1, 0, view.stream>>>(storage, view, cin);
  auto error = cudaPeekAtLastError();
  if (error != cudaSuccess) return error;
  Gather<<<blocks, threads, 0, view.stream>>>(storage, slab, view, cin);
  error = cudaPeekAtLastError();
  if (error != cudaSuccess) return error;
  Finish<<<blocks, threads, 0, view.stream>>>(storage, slab, view, cin);
  return cudaPeekAtLastError();
}
} // namespace tl::fea::solids::batch_detail
