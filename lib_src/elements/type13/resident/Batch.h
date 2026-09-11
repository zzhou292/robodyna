// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Type13RecurrenceTypes.h"
#include "../../ShellBatchStartup.h"
#include "../../../solvers/FENodalState.h"
#include <memory>

namespace tl::fea { class Type13NodeContributions; class ShellBatchPublication; }
namespace tl::fea::type13 {
class BatchQualificationPeer; // Defined only by the owning qualification.

// CIN stiffness is written only through this owner's authenticated optional
// assembly view. The ordinary profile has no nodal stiffness destination.
enum class BatchAssembly { OrdinaryForces, CinNativeStiffness };
struct BatchLimits {
  std::size_t max_connections = 8192;
  std::size_t max_properties = 1024;
  std::size_t max_nodes = 524288;
  std::size_t max_device_bytes = 32u << 20;
  std::size_t max_host_bytes = 256u << 20;
};
struct BatchConfig {
  NodalStamp owner;
  std::uint64_t configuration_id = 0;
  std::uint64_t qualification_id = 0;
  ShellBatchStartup startup;
  BatchAssembly assembly = BatchAssembly::OrdinaryForces;
  BatchLimits limits;
};
enum class BatchStatus {
  Success, InvalidInput, NotInitialized, NotBound, ResourceLimit, StaleTrial,
  ElementFailure, AssemblyFailure, NonfiniteResult, DeviceFailure, NodalFailure,
  Unusable
};
struct BatchReport {
  BatchStatus status = BatchStatus::Success;
  const char* message = "OK";
  std::size_t element = SIZE_MAX;
  std::size_t node = SIZE_MAX;
  Status element_status = Status::Success;
  NodalStatus nodal_status = NodalStatus::Ok;
  explicit operator bool() const noexcept { return status == BatchStatus::Success; }
};
enum class BatchPhase { Unspecified, Accepted, Prepared };
struct BatchDiagnostics {
  std::uint64_t source_instance_id = 0, owner_id = 0;
  std::uint64_t configuration_id = 0, qualification_id = 0;
  std::uint64_t epoch = 0, base_epoch = 0, attempt = 0;
  double time = 0, base_time = 0, velocity_time = 0, base_velocity_time = 0;
  double kick_dt = 0;
  BatchPhase phase = BatchPhase::Unspecified;
  bool valid = false, has_completed_interval = false;
  bool accepted_force_assembled = false;
  std::size_t element_count = 0, active_count = 0, newly_failed_count = 0;
  double internal_work_J[ChannelCount]{};
  double internal_work_increment_J[ChannelCount]{};
  double internal_kick_work_J = 0, internal_drift_work_J = 0;
  double minimum_native_dt_s = 0;
};
struct BatchForecast {
  std::size_t device_bytes = 0;
  // Complete conservative simultaneous host reservation, including retained
  // immutable source/domain, startup arena and reusable readback staging.
  std::size_t startup_host_bytes = 0;
};

// One complete immutable TYPE13 model, one bounded device arena, two history /
// force-cache slabs. No node owner, clock, inverse coefficient construction,
// per-step allocation or independent batch commit. Actual source/domain and
// owner fields are authenticated before the first accepted contribution.
class Batch {
 public:
  Batch();
  ~Batch();
  Batch(const Batch&) = delete;
  Batch& operator=(const Batch&) = delete;
  static BatchReport Forecast(const BatchConfig&, const Type13NodeContributions&,
                              BatchForecast&) noexcept;
  BatchReport InitializeJoined(const BatchConfig&, const Type13NodeContributions&);
  BatchReport AssembleAccepted(FENodalState&, const NodalTrialToken&,
                               const NodalAssemblyView&);
  BatchReport EvaluateCandidate(FENodalState&, const NodalTrialToken&,
                                const NodalPreparedView&, BatchDiagnostics*);
  BatchReport CopyAcceptedResults(const NodalStamp&, Evaluation*,
                                  std::size_t exact_count, BatchDiagnostics*);
  BatchReport CopyPreparedResults(const BatchDiagnostics&, Evaluation*,
                                  std::size_t exact_count);
  BatchReport CopyAcceptedDiagnostics(const NodalStamp&, BatchDiagnostics*) const noexcept;
  void DiscardTrial() noexcept;
  NodalAllocationInfo allocations() const noexcept;
  std::size_t startup_host_bytes() const noexcept;
 private:
  friend class ::tl::fea::ShellBatchPublication;
  friend class BatchQualificationPeer;
  BatchReport PreflightAttach(const NodalStamp&, const Type13NodeContributions&,
      std::uint64_t configuration, std::uint64_t qualification,
      const ShellBatchStartup&, BatchAssembly,
      const ShellBatchPublication* claimant) const noexcept;
  void AttachPublication(const ShellBatchPublication*) noexcept;
  void ReleasePublication(const ShellBatchPublication*) noexcept;
  void Poison() noexcept;
  BatchReport PreflightPublication(FENodalState&, const NodalTrialToken&,
                                    const NodalPreparedView&,
                                    const BatchDiagnostics&,
                                    const ShellBatchPublication* claimant) const noexcept;
  void Publish(const NodalStamp&) noexcept;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tl::fea::type13
