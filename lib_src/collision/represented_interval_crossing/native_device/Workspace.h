// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../RepresentedIntervalCrossingGpu.h"
#include "../DeviceExecution.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::represented_interval_crossing::native_device {
struct DeviceJob {
  CanonicalPair pair;
  std::size_t ordinal = 0;
};
enum class PairExecution : std::uint8_t {
  Incomplete, Complete, InvalidIndex, IdentityMismatch, DomainMismatch,
};
struct DeviceResult {
  RepresentedIntervalResult value;
  PairExecution execution = PairExecution::Incomplete;
};
struct Layout {
  tl::util::ArenaRegion paths, jobs, results, exact_scratch, dfs;
  tl::util::ArenaRegion host_jobs, host_results;
  RepresentedIntervalGpuForecast forecast;
  std::size_t host_payload_bytes = 0;
  std::size_t dfs_capacity = 0;
};
RepresentedIntervalGpuPreflight MakeLayout(RepresentedIntervalGpuLimits,
    std::size_t owner_bytes, Layout&) noexcept;
cudaError_t Launch(void*, const Layout&, std::size_t path_count,
    std::size_t job_count, RepresentedIntervalLimits, cudaStream_t) noexcept;

// One retained arena and one explicit stream. This type is private to the GPU
// facade; only a native-authenticated lexical work borrow enters Execute.
class Workspace final : public DeviceExecution {
 public:
  Workspace() noexcept = default;
  ~Workspace();
  Workspace(const Workspace&) = delete;
  Workspace& operator=(const Workspace&) = delete;
  RepresentedIntervalGpuReport Initialize(const Layout&, cudaStream_t,
      const void* facade, std::size_t facade_bytes,
      const void* owner, std::size_t owner_bytes) noexcept;
  RepresentedIntervalReport BeginAttempt(cudaStream_t) noexcept;
  RepresentedIntervalDeviceReport report() const noexcept { return report_; }
 private:
  bool Disjoint(const void*, std::size_t) const noexcept override;
  RepresentedIntervalReport Execute(const AuthenticatedWork&) noexcept override;
  RepresentedIntervalReport DeviceFailure(cudaError_t) noexcept;
  Layout layout_;
  tl::util::HostArena host_;
  DeviceJob* jobs_ = nullptr;
  DeviceResult* results_ = nullptr;
  void* device_ = nullptr;
  cudaStream_t stream_ = nullptr;
  int device_ordinal_ = -1;
  const void* facade_ = nullptr;
  std::size_t facade_bytes_ = 0;
  const void* owner_ = nullptr;
  std::size_t owner_bytes_ = 0;
  bool usable_ = false;
  RepresentedIntervalDeviceReport report_;
};
}  // namespace tlfea::contact::represented_interval_crossing::native_device
