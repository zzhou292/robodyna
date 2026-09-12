#include "Storage.h"
#include "../broadphase/Sweep.h"
#include <cub/cub.cuh>

namespace tlfea::contact::self_contact_broadphase {
namespace {
// Same key/index extraction and reorder as the existing SAP. CUB stable radix
// sorting gives source-index order for equal keys; final pair sorting removes
// the chosen axis from the public candidate order.
__global__ void Keys(const AABB* boxes, int n, unsigned axis, double* keys, int* indices) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n; i += blockDim.x * gridDim.x) {
    keys[i] = broadphase_detail::AxisValue(boxes[i].min, axis);
    indices[i] = i;
  }
}
__global__ void Reorder(const AABB* input, AABB* output, const int* indices, int n) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n; i += blockDim.x * gridDim.x)
    output[i] = input[indices[i]];
}
unsigned Blocks(std::size_t n) { const auto b = (n + 255) / 256; return b < 256 ? b : 256; }
}
cudaError_t QueryScratch(int parents, int pairs, ScratchRequirements& out) noexcept {
  ScratchRequirements next;
  auto error = cub::DeviceRadixSort::SortPairs(nullptr, next.sort,
      static_cast<double*>(nullptr), static_cast<double*>(nullptr),
      static_cast<int*>(nullptr), static_cast<int*>(nullptr), parents);
  if (error != cudaSuccess) return error;
  error = cub::DeviceScan::ExclusiveSum(nullptr, next.scan,
      static_cast<unsigned long long*>(nullptr), static_cast<unsigned long long*>(nullptr), parents + 1);
  if (error != cudaSuccess) return error;
  error = cub::DeviceRadixSort::SortKeys(nullptr, next.pair_sort,
      static_cast<SelfContactPairKey*>(nullptr), static_cast<SelfContactPairKey*>(nullptr), pairs);
  if (error == cudaSuccess) out = next;
  return error;
}
cudaError_t SortBoxes(void* arena, const Layout& layout, unsigned axis, cudaStream_t stream) noexcept {
  auto* keys = tl::util::ArenaPointer<double>(arena, layout.keys);
  auto* indices = tl::util::ArenaPointer<int>(arena, layout.indices);
  Keys<<<Blocks(layout.forecast.parents),256,0,stream>>>(
      tl::util::ArenaPointer<AABB>(arena, layout.boxes), layout.forecast.parents, axis, keys, indices);
  auto error = cudaPeekAtLastError(); if (error != cudaSuccess) return error;
  auto bytes = layout.cub_temp.bytes;
  error = cub::DeviceRadixSort::SortPairs(tl::util::ArenaPointer<std::byte>(arena, layout.cub_temp), bytes,
      keys, tl::util::ArenaPointer<double>(arena, layout.sorted_keys), indices,
      tl::util::ArenaPointer<int>(arena, layout.sorted_indices), static_cast<int>(layout.forecast.parents),
      0, 64, stream);
  if (error != cudaSuccess) return error;
  Reorder<<<Blocks(layout.forecast.parents),256,0,stream>>>(
      tl::util::ArenaPointer<AABB>(arena, layout.boxes), tl::util::ArenaPointer<AABB>(arena, layout.sorted_boxes),
      tl::util::ArenaPointer<int>(arena, layout.sorted_indices), layout.forecast.parents);
  return cudaPeekAtLastError();
}
cudaError_t ScanCounts(void* arena, const Layout& layout, cudaStream_t stream) noexcept {
  auto bytes = layout.cub_temp.bytes;
  return cub::DeviceScan::ExclusiveSum(tl::util::ArenaPointer<std::byte>(arena, layout.cub_temp), bytes,
      tl::util::ArenaPointer<unsigned long long>(arena, layout.counts),
      tl::util::ArenaPointer<unsigned long long>(arena, layout.offsets),
      static_cast<int>(layout.forecast.parents + 1), stream);
}
cudaError_t SortPairs(void* arena, const Layout& layout, int count, cudaStream_t stream) noexcept {
  auto bytes = layout.cub_temp.bytes;
  return cub::DeviceRadixSort::SortKeys(tl::util::ArenaPointer<std::byte>(arena, layout.cub_temp), bytes,
      tl::util::ArenaPointer<SelfContactPairKey>(arena, layout.pair_keys),
      tl::util::ArenaPointer<SelfContactPairKey>(arena, layout.sorted_pair_keys), count, 0, 64, stream);
}
} // namespace tlfea::contact::self_contact_broadphase
