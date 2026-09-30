// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../QephBatchStorage.h"
#include "MixedActivityValues.h"
#include "../../ShellMixedSectionStorage.h"

namespace tl::fea::qeph::batch_detail {
namespace {
__global__ void ValidateMixedActivity(Storage* storage, unsigned slab,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  const auto parent = std::size_t(blockIdx.x) * blockDim.x + threadIdx.x;
  if (parent >= storage->model.config.element_count) return;
  const auto law = mixed->law[parent];
  const auto error = mapped::CheckMixedActivity(law,mixed->plastic.section[slab][parent],
      mixed->elastic_section[slab][parent]);
  auto& packet = storage->assembly.activity;
  if (error != mapped::MixedActivityError::None) {
    atomicMin(packet.first_invalid,mapped::MixedActivityKey(static_cast<std::uint32_t>(parent),error));
    return;
  }
  // The role is emitted only after validation of its actual selected payload.
  // Inactive union capacity is not interpreted or manufactured into a history.
  packet.active[parent] = static_cast<std::uint8_t>(law);
}
} // namespace

void LaunchMappedMixedActivity(Storage* storage, std::size_t parents, unsigned slab,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed, cudaStream_t stream) {
  constexpr unsigned Threads = 128;
  const auto blocks = static_cast<unsigned>((parents + Threads - 1) / Threads);
  ValidateMixedActivity<<<blocks,Threads,0,stream>>>(storage,slab,mixed);
}
} // namespace tl::fea::qeph::batch_detail
