// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../QephBatchStorage.h"
#include "FailureActivityValues.h"
#include "../../ShellMixedSectionStorage.h"
#include "../../failure/ShellFailureArenaLayout.h"

namespace tl::fea::qeph::batch_detail {
namespace {
__global__ void ValidateFailureActivity(Storage* storage, unsigned slab, double time,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed,
    const shell_batch_plasticity_detail::FailureDeviceStorage* failure) {
  const auto parent = std::size_t(blockIdx.x) * blockDim.x + threadIdx.x;
  if (parent >= storage->model.config.element_count) return;
  const auto* section = mixed->law[parent] == ShellSectionLaw::LayeredLaw44Nip3
      ? &mixed->plastic.section[slab][parent] : nullptr;
  const auto& value = failure->state[slab][parent];
  auto& packet = storage->assembly.activity;
  if (!mapped::ValidFailureActivity(value,failure->policy[parent],section,time)) {
    atomicMin(packet.first_invalid,static_cast<std::uint32_t>(parent));
    return;
  }
  // Never evaluate/narrow a raw bool until all encoding/history checks pass.
  packet.active[parent] = value.active ? 1 : 0;
}
} // namespace

void LaunchMappedFailureActivity(Storage* storage, std::size_t parents, unsigned slab, double time,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed,
    const shell_batch_plasticity_detail::FailureDeviceStorage* failure, cudaStream_t stream) {
  constexpr unsigned Threads = 128;
  const auto blocks = static_cast<unsigned>((parents + Threads - 1) / Threads);
  ValidateFailureActivity<<<blocks,Threads,0,stream>>>(storage,slab,time,mixed,failure);
}
} // namespace tl::fea::qeph::batch_detail
