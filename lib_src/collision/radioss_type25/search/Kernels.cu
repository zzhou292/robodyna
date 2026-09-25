// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ReductionTile.h"
#include "Launch.h"
namespace tlfea::contact::radioss_type25::search::detail {
namespace {
__global__ void CapturePositions(Device d, Current input, unsigned slab) {
  const auto begin = std::size_t(blockIdx.x) * blockDim.x + threadIdx.x;
  const auto stride = std::size_t(gridDim.x) * blockDim.x;
  for (auto i = begin; i < d.reference_count; i += stride) {
    const auto node = d.compact ? d.roles[i] : std::uint32_t(i);
    d.reference[slab][i] = node == UINT32_MAX ? Vector{} : Position(d, input.positions, node);
  }
}
__device__ void Reduce(ReductionTile& tile) {
  for (unsigned stride = Threads / 2; stride; stride /= 2) {
    __syncthreads();
    if (threadIdx.x < stride) {
      auto value = Load(tile, threadIdx.x);
      Merge(value, Load(tile, threadIdx.x + stride));
      Store(tile, threadIdx.x, value);
    }
  }
  __syncthreads();
}
__global__ void ObserveRows(Device d, Current input, unsigned slab,
    bool capture, bool has_reference) {
  __shared__ ReductionTile tile;
  Partial value;
  const auto begin = std::size_t(blockIdx.x) * blockDim.x + threadIdx.x;
  const auto stride = std::size_t(gridDim.x) * blockDim.x;
  for (auto i = begin; i < RoleCount(d); i += stride)
    ObserveRole(d, input, slab, i, capture, has_reference, value);
  for (auto i = begin; i < d.segments; i += stride)
    ObserveGap(d, input, slab, i, capture, value);
  Store(tile, threadIdx.x, value);
  Reduce(tile);
  if (!threadIdx.x) d.partials[blockIdx.x] = Load(tile, 0);
}
__global__ void Complete(Device d, unsigned blocks, bool capture,
    double margin, double previous_dt, bool force_sort) {
  __shared__ ReductionTile tile;
  Partial value;
  for (unsigned i = threadIdx.x; i < blocks; i += Threads)
    Merge(value, d.partials[i]);
  Store(tile, threadIdx.x, value);
  Reduce(tile);
  if (threadIdx.x) return;
  DeviceControl result;
  result.partial = Load(tile, 0);
  if (!d.gap_changes) result.partial.extrema.maximum_gap_change = 0;
  if (result.partial.status == Status::Ok &&
      (!result.partial.extrema.secondary_uses || !result.partial.extrema.main_uses))
    result.partial.status = Status::UnsupportedLifecycle;
  if (result.partial.status == Status::Ok && !capture) {
    result.partial.status = EvaluateBudget(result.partial.extrema, margin,
        previous_dt, force_sort, result.budget);
  }
  *d.control = result;
}
}
cudaError_t Run(Device d, const Current& input, unsigned slab, bool capture,
    double margin, double previous_dt, bool force_sort, bool has_reference,
    cudaStream_t stream) noexcept {
  // CheckSource admits a nonempty role roster before allocating this Device.
  const auto count = RoleCount(d) > d.segments ? RoleCount(d) : d.segments;
  const auto requested = (count + Threads - 1) / Threads;
  const unsigned blocks = unsigned(requested > MaximumBlocks ? MaximumBlocks : requested);
  if (capture) {
    CapturePositions<<<blocks, Threads, 0, stream>>>(d, input, slab);
    const auto error = cudaGetLastError();
    if (error != cudaSuccess) return error;
  }
  ObserveRows<<<blocks, Threads, 0, stream>>>(d, input, slab, capture, has_reference);
  const auto error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  Complete<<<1, Threads, 0, stream>>>(d, blocks, capture, margin, previous_dt, force_sort);
  return cudaGetLastError();
}
} // namespace tlfea::contact::radioss_type25::search::detail
