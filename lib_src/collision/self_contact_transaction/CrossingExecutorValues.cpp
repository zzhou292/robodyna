// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CrossingExecutor.h"
#include "../SelfContactTransactionTypes.h"

namespace tlfea::contact::self_contact_transaction {
RepresentedIntervalGpuLimits CrossingGpuLimits(const SelfContactTransactionConfig& config,
    const SelfContactTransactionLimits& limits) noexcept {
  RepresentedIntervalGpuLimits gpu;
  gpu.native = limits.crossing;
  gpu.max_device_bytes = limits.max_device_bytes;
  gpu.max_workspace_host_bytes = limits.max_host_bytes;
  gpu.device_workers = config.native_crossing_device_workers;
  gpu.numeric_cohort_pairs = config.native_crossing_numeric_cohort_pairs;
  return gpu;
}
CrossingExecutionForecast CrossingExecutor::Preflight(const SelfContactTransactionConfig& config,
    const SelfContactTransactionLimits& limits) noexcept {
  CrossingExecutionForecast result;
  if (!config.enable_cuda_native_crossing) {
    const auto cpu = RepresentedIntervalCrossing::Preflight(limits.crossing);
    result.report = cpu.report;
    result.native = cpu.forecast;
    result.owned_host_bytes = cpu.forecast.owned_host_bytes;
    result.startup_host_bytes = cpu.forecast.startup_host_bytes;
    return result;
  }
  const auto gpu = RepresentedIntervalCrossingGpu::Preflight(CrossingGpuLimits(config, limits));
  result.report = gpu.report.native;
  result.device_status = gpu.report.device.status;
  if (result.report.status != RepresentedIntervalStatus::Ok) return result;
  if (gpu.forecast.owned_host_bytes < sizeof(RepresentedIntervalCrossingGpu) ||
      gpu.forecast.startup_host_bytes < gpu.forecast.owned_host_bytes) {
    result.report.status = RepresentedIntervalStatus::ResourceLimit;
    result.report.message = "CUDA crossing forecast is smaller than its retained handle";
    return result;
  }
  result.native = gpu.forecast.native;
  // The selected GPU handle is already embedded in the transaction's executor.
  result.owned_host_bytes = gpu.forecast.owned_host_bytes - sizeof(RepresentedIntervalCrossingGpu);
  result.startup_host_bytes = gpu.forecast.startup_host_bytes - sizeof(RepresentedIntervalCrossingGpu);
  result.device_bytes = gpu.forecast.device_bytes;
  result.device_allocations = gpu.forecast.device_allocations;
  return result;
}
void DescribeCrossingDeviceFailure(const RepresentedIntervalDeviceReport& device,
    SelfContactTransactionReport& report) noexcept {
  // Ordinary native input/work failures retain their original report fields.
  // Successful routing belongs in optional diagnostics, not failure metadata.
  if (device.status == RepresentedIntervalDeviceStatus::Ok ||
      device.status == RepresentedIntervalDeviceStatus::NotInvoked) return;
  report.crossing_device_status = device.status;
  report.crossing_fault_cohort_begin = device.fault_cohort_begin;
  report.crossing_fault_cohort_count = device.fault_cohort_count;
  report.crossing_fault_pair_ordinal = device.fault_pair_ordinal;
}
}  // namespace tlfea::contact::self_contact_transaction
