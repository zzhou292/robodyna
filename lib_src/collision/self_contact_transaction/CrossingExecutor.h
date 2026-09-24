// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "CrossingBatch.h"
#include "../RepresentedIntervalCrossingGpu.h"

namespace tlfea::contact {
struct SelfContactTransactionConfig;
struct SelfContactTransactionLimits;
struct SelfContactTransactionReport;
namespace self_contact_transaction {
struct CrossingExecutionForecast {
  RepresentedIntervalReport report;
  RepresentedIntervalDeviceStatus device_status = RepresentedIntervalDeviceStatus::NotInvoked;
  RepresentedIntervalForecast native;
  std::size_t owned_host_bytes = 0, startup_host_bytes = 0;
  std::size_t device_bytes = 0, device_allocations = 0;
};
struct CrossingExecutionReport {
  CrossingBatchReport native;
  RepresentedIntervalDeviceReport device;
};

// One selected executor, never a physical state owner. Both handles start empty;
// exactly one Initialize branch executes. The GPU facade already contains its
// sole CPU fallback pool. No second CPU pool or clock is initialized beside it.
class CrossingExecutor {
 public:
  static CrossingExecutionForecast Preflight(const SelfContactTransactionConfig&,
      const SelfContactTransactionLimits&) noexcept;
  RepresentedIntervalGpuReport Initialize(const SelfContactTransactionConfig&,
      const SelfContactTransactionLimits&, cudaStream_t) noexcept;
  CrossingExecutionReport Certify(const RepresentedTrianglePath*, std::size_t,
      const RepresentedTrianglePair*, std::size_t, std::size_t,
      RepresentedIntervalResult*, std::size_t) noexcept;
  RepresentedIntervalResultView results() const noexcept;
 private:
  RepresentedIntervalCrossing cpu_;
  RepresentedIntervalCrossingGpu gpu_;
  cudaStream_t stream_ = nullptr;
  bool initialized_ = false, use_gpu_ = false;
};
// Pure parameter mapping shared by forecast and startup. Original native limits
// remain the only per-pair/per-slice authority; GPU options only size execution.
RepresentedIntervalGpuLimits CrossingGpuLimits(const SelfContactTransactionConfig&,
    const SelfContactTransactionLimits&) noexcept;
void DescribeCrossingDeviceFailure(const RepresentedIntervalDeviceReport&,
    SelfContactTransactionReport&) noexcept;
}  // namespace self_contact_transaction
}  // namespace tlfea::contact
