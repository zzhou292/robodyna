// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ActivityValues.h"
#include "../T3BatchStorage.h"
#include "../../ShellMixedSectionArenaLayout.h"
#include "../../failure/ShellFailureArenaLayout.h"
#include "../../one_point/ShellOnePointArenaLayout.h"
namespace tl::fea::t3::mapped {
// Private read-only query; owner/token/source admission remains on the host.
// Point and force phases deliberately retain their original distinct operands.
struct ActivityQuery {
  batch_detail::Storage* storage = nullptr;
  const shell_batch_plasticity_detail::MixedDeviceStorage* mixed = nullptr;
  const shell_batch_plasticity_detail::FailureDeviceStorage* failure = nullptr;
  const shell_batch_plasticity_detail::OnePointDeviceStorage* point = nullptr;
  std::size_t parents = 0;
  unsigned slab = 0;
  double time = 0, force_time = 0;
  std::uint64_t epoch = 0, force_epoch = 0;
  bool execution = false;
};
cudaError_t LaunchActivity(const ActivityQuery&, ActivityPhase, cudaStream_t);
BatchReport ActivityErrorReport(ActivityPhase, std::uint32_t key, std::size_t parents) noexcept;
} // namespace tl::fea::t3::mapped
