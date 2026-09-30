// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "AssemblyValidation.h"

namespace tlfea::contact::nodal_wall_mapped::assembly_validation {
static __global__ void Begin(d::Storage* pointer, Sidecar side, fe::NodalAssemblyView view) {
  auto& storage = *pointer;
  storage.control = {};
  *side.summary = {};
  side.summary->points_admitted = Header(view, storage.control);
}
static __global__ void Nodes(d::Storage* pointer, Sidecar side, fe::NodalAssemblyView view) {
  auto& storage = *pointer;
  if (!side.summary->points_admitted) return;
  const unsigned first = blockIdx.x*blockDim.x+threadIdx.x;
  const unsigned stride = gridDim.x*blockDim.x;
  for (unsigned i = first; i < storage.model.node_count; i += stride) {
    double inverse = 0;
    d::Control status;
    if (!Mass(storage, side, view, i, inverse, status))
      atomicMin(&side.summary->parent_failure, Failure(i, false));
    else if (!Geometry(storage, view, i, status))
      atomicMin(&side.summary->parent_failure, Failure(i, true));
  }
}
static __global__ void CopyInversePrefix(d::Storage* pointer, Sidecar side, fe::NodalAssemblyView view) {
  auto& storage = *pointer;
  if (!side.summary->points_admitted) return;
  const auto count = InversePrefix(storage.model.node_count, side.summary->parent_failure);
  const unsigned first = blockIdx.x*blockDim.x+threadIdx.x;
  const unsigned stride = gridDim.x*blockDim.x;
  for (unsigned i = first; i < count; i += stride) {
    const auto node = storage.model.nodes[i].node;
    // The authenticated accepted view is immutable for the whole attempt.
    // Re-read only the already consumed scalar; no staging array is needed.
    side.inverse[i] = view.mass.inverse_mass[node];
  }
}
static __global__ void Finish(d::Storage* pointer, Sidecar side) {
  Complete(*pointer, side);
}
inline void Launch(d::Storage* storage, Sidecar side, fe::NodalAssemblyView view,
    std::size_t nodes, cudaStream_t stream) {
  Begin<<<1, 1, 0, stream>>>(storage, side, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Nodes<<<Blocks(nodes), Threads, 0, stream>>>(storage, side, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  CopyInversePrefix<<<Blocks(nodes), Threads, 0, stream>>>(storage, side, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Finish<<<1, 1, 0, stream>>>(storage, side);
}
} // namespace tlfea::contact::nodal_wall_mapped::assembly_validation
