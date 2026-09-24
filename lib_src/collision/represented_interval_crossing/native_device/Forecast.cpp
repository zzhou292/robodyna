// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerStorage.h"
namespace tlfea::contact {
RepresentedIntervalGpuPreflight RepresentedIntervalCrossingGpu::Preflight(
    RepresentedIntervalGpuLimits limits) noexcept {
  represented_interval_crossing::native_device::Layout layout;
  return represented_interval_crossing::native_device::MakeLayout(
      limits, sizeof(RepresentedIntervalCrossingGpu) + sizeof(Impl), layout);
}
}  // namespace tlfea::contact
