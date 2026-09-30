// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellMixedSectionArenaLayout.h"
#include "failure/ShellFailureArenaLayout.h"
namespace tl::fea::shell_batch_plasticity_detail {
// Immutable host mirrors of owned device regions, for bounded diagnostic reads.
// These addresses are not host data and never belong to persisted evidence.
struct DiagnosticDeviceSources {
  const DeviceStorage* plain_device=nullptr;
  DeviceStorage plain;
  const MixedDeviceStorage* mixed_device=nullptr;
  MixedDeviceStorage mixed;
  const FailureDeviceStorage* failure_device=nullptr;
  FailureDeviceStorage failure;
  std::size_t parent_count=0,curve_points=0;
};
} // namespace tl::fea::shell_batch_plasticity_detail
