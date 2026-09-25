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
  // Preserve the original all-parent geometry precheck before any sums.
  if (view.result->base_epoch != view.accepted.base_epoch || view.result->attempt != view.attempt ||
      view.bounds->base_epoch != view.accepted.base_epoch || view.bounds->attempt != view.attempt ||
      !view.bounds->initialized || !view.bounds->valid || view.bounds->sealed ||
      view.result->status != tlfea::contact::Status::kOk ||
      !assembly_serial::CheckNodes<Traits18>(state, view) ||
      !assembly_serial::CheckNodes<Traits24>(state, view) ||
      !assembly_serial::CheckNodes<Traits6z>(state, view) ||
      !assembly_serial::CheckNodes<Traits18Law44>(state, view) ||
      !assembly_serial::CheckNodes<Traits18Law90>(state, view) ||
      !CanGatherAssembly(state, view, cin)) state.assembly.fallback = 1;
  state.assembly.prepared = !state.assembly.fallback;
}
__global__ void Gather(Storage* storage, unsigned slab, NodalAssemblyView view,
    NodalCinAssemblyView cin) {
  auto& state = *storage;
  // Prepare has completed on this stream. This kernel writes only private
  // nodes and an integer failure flag, never shared floating-point atomics.
  if (!state.assembly.prepared) return;
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
