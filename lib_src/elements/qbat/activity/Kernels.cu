// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../QbatBatchStorage.h"
#include "Values.h"
namespace tl::fea::qbat::batch_detail {
namespace {
__global__ void ValidateActivity(Storage* storage, std::size_t parents, unsigned slab,
    double time, std::uint64_t epoch) {
  const auto parent = std::size_t(blockIdx.x) * blockDim.x + threadIdx.x;
  if (parent >= parents) return;
  auto& packet = storage->activity;
  // Every row is overwritten on every read, even if its current result is bad.
  packet.active[parent] = activity::InvalidFlag;
  if (!activity::ValidParent(*storage,slab,parent,time,epoch)) {
    atomicMin(packet.first_invalid,static_cast<std::uint32_t>(parent));
    return;
  }
  packet.active[parent] = storage->slab[slab].element[parent].history.element_active ? 1 : 0;
}
}
void LaunchParentActivity(Storage* storage, std::size_t parents, unsigned slab,
    double time, std::uint64_t epoch, cudaStream_t stream) {
  constexpr unsigned Threads = 128;
  const auto blocks = static_cast<unsigned>((parents + Threads - 1) / Threads);
  ValidateActivity<<<blocks,Threads,0,stream>>>(storage,parents,slab,time,epoch);
}
} // namespace tl::fea::qbat::batch_detail
