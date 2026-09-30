// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "IntervalFinalizer.h"

namespace tlfea::contact::nodal_wall_mapped::parallel {
namespace interval_detail {
namespace d = nodal_wall_device_detail;
static __device__ void ReduceBlock(IntervalSummary* values) {
  for (unsigned offset = ObserverThreads/2; offset; offset /= 2) {
    __syncthreads();
    if (threadIdx.x < offset) MergeIntervals(values[threadIdx.x], values[threadIdx.x+offset]);
  }
  __syncthreads();
}
static __global__ void Serial(d::Storage* storage, tl::fea::NodalPreparedView view, bool* tree_used) {
  if (tree_used) *tree_used = false;
  if (storage->control.status == NodalWallDeviceStatus::Ok) d::MeasureInterval(*storage, view);
}
static __global__ void Observe(d::Storage* storage, tl::fea::NodalPreparedView view,
    IntervalScratch scratch) {
  if (storage->control.status != NodalWallDeviceStatus::Ok) return;
  const auto blocks = ObserverBlocks(storage->model.node_count);
  if (blockIdx.x >= blocks) return;
  __shared__ IntervalSummary values[ObserverThreads];
  IntervalSummary local{};
  const unsigned first = blockIdx.x*blockDim.x+threadIdx.x, stride = blocks*blockDim.x;
  for (unsigned node = first; node < storage->model.node_count; node += stride) {
    ObserveIntervalNode(*storage, view, node, local);
  }
  values[threadIdx.x] = local;
  ReduceBlock(values);
  if (!threadIdx.x) scratch.data[blockIdx.x] = values[0];
}
static __global__ void Finish(d::Storage* storage, tl::fea::NodalPreparedView view,
    IntervalScratch scratch, bool* tree_used) {
  if (!threadIdx.x && tree_used) *tree_used = false;
  if (storage->control.status != NodalWallDeviceStatus::Ok) return;
  const auto blocks = ObserverBlocks(storage->model.node_count);
  __shared__ IntervalSummary values[ObserverThreads];
  IntervalSummary local{};
  for (unsigned block = threadIdx.x; block < blocks; block += blockDim.x) {
    MergeIntervals(local, scratch.data[block]);
  }
  values[threadIdx.x] = local;
  ReduceBlock(values);
  if (!threadIdx.x) FinalizeInterval(*storage, view, values[0], tree_used);
}
} // namespace interval_detail
inline void MeasureInterval(nodal_wall_device_detail::Storage* storage,
    tl::fea::NodalPreparedView view, std::size_t nodes, cudaStream_t stream,
    IntervalScratch scratch = {}, bool* tree_used = nullptr) {
  const auto blocks = ObserverBlocks(nodes);
  if (!scratch.data || !blocks || scratch.count < blocks) {
    interval_detail::Serial<<<1, 1, 0, stream>>>(storage, view, tree_used);
    return;
  }
  interval_detail::Observe<<<ObserverMaxBlocks, ObserverThreads, 0, stream>>>(storage, view, scratch);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  interval_detail::Finish<<<1, ObserverThreads, 0, stream>>>(storage, view, scratch, tree_used);
}
} // namespace tlfea::contact::nodal_wall_mapped::parallel
