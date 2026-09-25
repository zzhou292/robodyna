// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../../RadiossType25AssemblyDevice.h"
#include "Launch.h"
namespace tlfea::contact::radioss_type25::assembly {
struct DeviceIncidenceBuilder::Impl {
  ~Impl();
  IncidenceLimits limits;
  device_detail::Layout layout;
  device_detail::Device device;
  void* arena = nullptr;
  cudaStream_t stream = nullptr;
  std::uint64_t identity = 0, sequence = 0;
  unsigned long long failure = ~0ull;
  bool usable = true, pending = false;
  DeviceConnectivity current;
  IncidenceReport report;
  IncidenceStatus Check(const DeviceConnectivity&) const noexcept;
};
} // namespace tlfea::contact::radioss_type25::assembly
