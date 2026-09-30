// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../QephBatchStorage.h"
#include "Result.h"
#include "ActivityValues.h"
#include "../../ShellMixedSectionStorage.h"

namespace tl::fea::qeph::batch_detail {
namespace {
__global__ void ValidateActivity(Storage* storage, unsigned slab, double time,
    std::uint64_t epoch, const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  const auto parent = std::size_t(blockIdx.x) * blockDim.x + threadIdx.x;
  if (parent >= storage->model.config.element_count) return;
  const auto& reference = storage->model.element[parent].reference;
  const auto& result = storage->slab[slab].element[parent];
  const bool skin = mixed->law[parent] == ShellSectionLaw::RigidSkin;
  auto& packet = storage->assembly.activity;
  // Uploaded source references/roles belong to this immutable mapped binding.
  // All result fields are freshly checked; no previous verdict is reused.
  if (!mapped::ValidResult(reference,result,time,epoch,skin)) {
    atomicMin(packet.first_invalid,static_cast<std::uint32_t>(parent));
    return;
  }
  packet.active[parent] = static_cast<std::uint8_t>(result.proposed_history.data().active);
}
} // namespace

void LaunchMappedActivity(Storage* storage, std::size_t parents, unsigned slab, double time, std::uint64_t epoch,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed, cudaStream_t stream) {
  constexpr unsigned Threads = 128;
  const auto blocks = static_cast<unsigned>((parents + Threads - 1) / Threads);
  ValidateActivity<<<blocks,Threads,0,stream>>>(storage,slab,time,epoch,mixed);
}
} // namespace tl::fea::qeph::batch_detail
