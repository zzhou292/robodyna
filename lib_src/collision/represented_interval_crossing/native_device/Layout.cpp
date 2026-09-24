// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Workspace.h"
#include "KernelTypes.h"

namespace tlfea::contact::represented_interval_crossing::native_device {
namespace {
RepresentedIntervalGpuPreflight Failure(RepresentedIntervalStatus status,
    const char* message, RepresentedIntervalDeviceStatus device) noexcept {
  RepresentedIntervalGpuPreflight result;
  result.report.native.status = status;
  result.report.native.message = message;
  result.report.device.status = device;
  result.report.device.message = message;
  return result;
}
}  // namespace
RepresentedIntervalGpuPreflight MakeLayout(RepresentedIntervalGpuLimits limits,
    std::size_t owner_bytes, Layout& output) noexcept {
  RepresentedIntervalGpuPreflight result;
  const auto cpu = RepresentedIntervalCrossing::Preflight(limits.native);
  result.report.native = cpu.report;
  if (cpu.report.status != RepresentedIntervalStatus::Ok) return result;
  if (!limits.max_device_bytes || !limits.max_workspace_host_bytes ||
      !limits.device_workers || limits.device_workers > MaximumDeviceWorkers)
    return Failure(RepresentedIntervalStatus::InvalidInput,
        "Native CUDA limits are invalid", RepresentedIntervalDeviceStatus::InvalidInput);
  Layout next;
  next.dfs_capacity = cpu.forecast.dfs_frame_capacity;
  tl::util::BoundedArenaLayout device(limits.max_device_bytes);
  if (!device.Append<RepresentedTrianglePath>(limits.native.max_paths, next.paths) ||
      !device.Append<DeviceJob>(limits.native.max_results, next.jobs) ||
      !device.Append<DeviceResult>(limits.native.max_results, next.results) ||
      !device.Append<ExactScratch>(limits.device_workers, next.exact_scratch) ||
      !device.Append<native::Cell>(limits.device_workers * next.dfs_capacity, next.dfs))
    return Failure(RepresentedIntervalStatus::ResourceLimit,
        "Native CUDA arena exceeds the device cap", RepresentedIntervalDeviceStatus::ResourceLimit);
  tl::util::BoundedArenaLayout host(limits.max_workspace_host_bytes);
  if (!host.Append<DeviceJob>(limits.native.max_results, next.host_jobs) ||
      !host.Append<DeviceResult>(limits.native.max_results, next.host_results) ||
      owner_bytes > limits.max_workspace_host_bytes - host.bytes())
    return Failure(RepresentedIntervalStatus::ResourceLimit,
        "Native CUDA staging exceeds the host cap", RepresentedIntervalDeviceStatus::ResourceLimit);
  next.host_payload_bytes = host.bytes();
  auto& forecast = next.forecast;
  forecast.native = cpu.forecast;
  forecast.device_bytes = device.bytes();
  forecast.device_scratch_bytes = next.exact_scratch.bytes;
  forecast.device_dfs_bytes = next.dfs.bytes;
  forecast.device_workers = limits.device_workers;
  forecast.workspace_host_bytes = owner_bytes + host.bytes();
  // Include bounded startup layouts/return values. CUDA runtime bookkeeping and
  // driver-managed call frames remain covered by the outer process/device guard.
  constexpr std::size_t startup_fixed = 2 * sizeof(Layout) + 2 * sizeof(RepresentedIntervalGpuPreflight);
  if (startup_fixed > limits.max_workspace_host_bytes - forecast.workspace_host_bytes ||
      forecast.workspace_host_bytes > SIZE_MAX - cpu.forecast.owned_host_bytes ||
      forecast.workspace_host_bytes + startup_fixed > SIZE_MAX - cpu.forecast.startup_host_bytes)
    return Failure(RepresentedIntervalStatus::ResourceLimit,
        "Native CUDA startup forecast exceeds the host cap", RepresentedIntervalDeviceStatus::ResourceLimit);
  forecast.owned_host_bytes = cpu.forecast.owned_host_bytes + forecast.workspace_host_bytes;
  forecast.startup_host_bytes = cpu.forecast.startup_host_bytes + forecast.workspace_host_bytes + startup_fixed;
  result.forecast = forecast;
  result.report.device = {RepresentedIntervalDeviceStatus::Ok, "OK"};
  output = next;
  return result;
}
}  // namespace tlfea::contact::represented_interval_crossing::native_device
