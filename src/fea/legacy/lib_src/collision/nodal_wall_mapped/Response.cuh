// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Kernels.cuh"
#include "ResponseValues.h"

namespace tlfea::contact::nodal_wall_mapped::response {
static __global__ void Begin(d::Storage* pointer, Sidecar side) {
  if (pointer->control.status == Code::Ok) side.summary->parent_failure = NoFailure;
}
static __global__ void OrdinaryNodes(d::Storage* pointer, Sidecar side) {
  auto& storage = *pointer;
  if (storage.control.status != Code::Ok) return;
  __shared__ double maxima[Threads];
  double maximum = 0;
  const unsigned first = blockIdx.x*blockDim.x+threadIdx.x;
  const unsigned stride = gridDim.x*blockDim.x;
  for (unsigned row = first; row < storage.model.node_count; row += stride)
    if (!Ordinary(storage, side, row, maximum))
      atomicMin(&side.summary->parent_failure, static_cast<unsigned long long>(row));
  maxima[threadIdx.x] = maximum;
  __syncthreads();
  for (unsigned width = Threads/2; width; width /= 2) {
    if (threadIdx.x < width) maxima[threadIdx.x] = ::fmax(maxima[threadIdx.x], maxima[threadIdx.x+width]);
    __syncthreads();
  }
  if (!threadIdx.x) side.response.maxima[blockIdx.x] = maxima[0];
}
static __global__ void RigidGroups(d::Storage* pointer, Sidecar side,
    fe::DeviceNodalKinematicsView k) {
  auto& storage = *pointer;
  if (storage.control.status != Code::Ok) return;
  const unsigned group = blockIdx.x*blockDim.x+threadIdx.x;
  if (group < side.groups && !Rigid(storage, side, k, group))
    atomicMin(&side.summary->parent_failure, static_cast<unsigned long long>(0));
}
static __global__ void Finish(d::Storage* pointer, Sidecar side,
    fe::DeviceNodalKinematicsView k) {
  auto& storage = *pointer;
  if (storage.control.status != Code::Ok) return;
  // Only the original serial traversal can publish an error or its partial
  // prefix. It starts from the same untouched result diagnostics and resets
  // rate/traces itself. Parallel workers never publish control or force data.
  if (CompleteRate(storage, side) || nodal_wall_mapped::Response(storage, side, k))
    CheckStep(storage, side);
}
inline void Launch(d::Storage* storage, Sidecar side, fe::NodalAssemblyView view,
    cudaStream_t stream) {
  Begin<<<1, 1, 0, stream>>>(storage, side);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  OrdinaryNodes<<<side.response.block_count, Threads, 0, stream>>>(storage, side);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  if (side.groups) {
    RigidGroups<<<(side.groups+Threads-1)/Threads, Threads, 0, stream>>>(storage, side, view.accepted);
    if (cudaPeekAtLastError() != cudaSuccess) return;
  }
  Finish<<<1, 1, 0, stream>>>(storage, side, view.accepted);
}
} // namespace tlfea::contact::nodal_wall_mapped::response
