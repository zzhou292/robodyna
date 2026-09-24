// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../RepresentedIntervalCrossingGpu.h"
#include "Workspace.h"
#include <atomic>
namespace tlfea::contact {
// Private retained layout shared by startup and pure CXX forecasting. Merely
// inspecting sizeof this layout constructs no owner and executes no CUDA work.
struct RepresentedIntervalCrossingGpu::Impl {
  RepresentedIntervalCrossing native;
  represented_interval_crossing::native_device::Workspace workspace;
  RepresentedIntervalGpuForecast forecast;
  std::atomic<bool> busy{false};
};
}  // namespace tlfea::contact
