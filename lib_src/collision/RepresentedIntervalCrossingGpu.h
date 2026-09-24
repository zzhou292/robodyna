// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "RepresentedIntervalCrossing.h"
#include <cuda_runtime_api.h>

namespace tlfea::contact {
namespace represented_interval_crossing::native_device { class Workspace; }
namespace represented_interval_crossing { struct DeviceBatchAccess; }

enum class RepresentedIntervalDeviceStatus : std::uint8_t {
  NotInvoked, Ok, InvalidInput, ResourceLimit, DeviceFailure,
};
struct RepresentedIntervalGpuLimits {
  RepresentedIntervalLimits native;
  std::size_t max_device_bytes = 64u << 20;
  std::size_t max_workspace_host_bytes = 16u << 20;
  unsigned device_workers = 128;
};
struct RepresentedIntervalGpuForecast {
  RepresentedIntervalForecast native;
  std::size_t device_bytes = 0;
  std::size_t device_scratch_bytes = 0;
  std::size_t device_dfs_bytes = 0;
  std::size_t workspace_host_bytes = 0;
  std::size_t owned_host_bytes = 0;
  std::size_t startup_host_bytes = 0;
  unsigned device_workers = 0;
};
// Counts describe numerical routing, never physical work or an exclusion.
struct RepresentedIntervalDeviceReport {
  RepresentedIntervalDeviceStatus status = RepresentedIntervalDeviceStatus::NotInvoked;
  const char* message = "CUDA certificate execution was not invoked";
  std::size_t device_pairs = 0;
  std::size_t host_pairs = 0;
  std::size_t batches = 0;
  std::size_t scene_uploads = 0;
};
struct RepresentedIntervalGpuReport {
  RepresentedIntervalReport native;
  RepresentedIntervalDeviceReport device;
};
struct RepresentedIntervalGpuPreflight {
  RepresentedIntervalGpuReport report;
  RepresentedIntervalGpuForecast forecast;
};

// Optional standalone CUDA executor over the existing native owner. The native
// frontend still authenticates the complete roster, sorts/deduplicates pairs,
// folds physical work in canonical order, and atomically publishes results.
// Only the proved fixed-integer domain runs on CUDA. Other pairs use the
// unchanged CPU implementation before the common publication step. CUDA errors
// poison this owner; they never trigger a numerical CPU retry.
//
// Externally serialized. Use one explicit stream throughout the lifetime.
// Borrowed input remains immutable until Certify returns; all CUDA transfers
// are drained before return, including error paths. Failed Certify calls retain
// the previous complete results() address, count and bytes, as on the CPU owner.
class RepresentedIntervalCrossingGpu {
 public:
  RepresentedIntervalCrossingGpu() noexcept;
  ~RepresentedIntervalCrossingGpu();
  RepresentedIntervalCrossingGpu(const RepresentedIntervalCrossingGpu&) = delete;
  RepresentedIntervalCrossingGpu& operator=(const RepresentedIntervalCrossingGpu&) = delete;
  static RepresentedIntervalGpuPreflight Preflight(RepresentedIntervalGpuLimits = {}) noexcept;
  RepresentedIntervalGpuReport Initialize(RepresentedIntervalGpuLimits, cudaStream_t) noexcept;
  RepresentedIntervalGpuReport Certify(const RepresentedTrianglePath*, std::size_t,
      const RepresentedTrianglePair*, std::size_t, cudaStream_t) noexcept;
  bool initialized() const noexcept;
  RepresentedIntervalGpuForecast forecast() const noexcept;
  RepresentedIntervalResultView results() const noexcept;
 private:
  friend struct represented_interval_crossing::DeviceBatchAccess;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace tlfea::contact
