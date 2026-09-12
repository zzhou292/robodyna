// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Results.h"
#include "lib_src/elements/beam18/Model.h"
#include "../../ShellBatchStartup.h"
#include "../../../solvers/FENodalState.h"
#include <memory>

namespace tl::fea {
class ShellBatchPublication;
class NodalCoefficientLedger;
class NodalRigidAssemblyBinding;
struct NodalCinWitnessSource;
}
namespace tl::fea::beam18 {
class BatchQualificationPeer;
enum class BatchProfile { Unspecified, PhysicalCinCircularFourPointLaw44V1 };
struct BatchLimits {
  std::size_t max_parents = 1024, max_materials = 256;
  std::size_t max_curve_points = 65536, max_nodes = 524288;
  std::size_t max_device_bytes = 128u << 20, max_host_bytes = 256u << 20;
};
struct BatchConfig {
  NodalStamp owner;
  std::uint64_t configuration_id = 0, qualification_id = 0;
  ShellBatchStartup startup;
  BatchProfile profile = BatchProfile::Unspecified;
  std::size_t cin_attachment_count = 0, cin_witness_count = 0;
  BatchLimits limits;
};
enum class BatchStatus {
  Success, InvalidInput, NotInitialized, NotBound, ResourceLimit, StaleTrial,
  ElementFailure, AssemblyFailure, NonfiniteResult, DeviceFailure, NodalFailure, Unusable
};
struct BatchReport {
  BatchStatus status = BatchStatus::Success;
  const char* message = "OK";
  std::size_t parent = SIZE_MAX, node = SIZE_MAX;
  int element_status = 0;
  NodalStatus nodal_status = NodalStatus::Ok;
  explicit operator bool() const noexcept { return status == BatchStatus::Success; }
};
enum class BatchPhase { Unspecified, Accepted, Prepared };
struct BatchDiagnostics {
  std::uint64_t source_instance_id = 0, owner_id = 0;
  std::uint64_t configuration_id = 0, qualification_id = 0;
  std::uint64_t epoch = 0, base_epoch = 0, attempt = 0;
  double time = 0, base_time = 0, velocity_time = 0, base_velocity_time = 0, kick_dt = 0;
  BatchPhase phase = BatchPhase::Unspecified;
  bool valid = false, has_completed_interval = false, accepted_force_assembled = false;
  std::size_t parent_count = 0;
  double native_internal_work_increment_j[2]{};
  double plastic_work_increment_j = 0;
  double internal_kick_work_j = 0, internal_drift_work_j = 0;
  double minimum_native_dt_s = 0;
};
struct BatchForecast {
  std::size_t device_bytes = 0;
  // Complete model backing, typed upload/readback and temporary initial proof.
  std::size_t startup_host_bytes = 0;
};
class Batch {
 public:
  Batch();
  ~Batch();
  Batch(const Batch&) = delete;
  Batch& operator=(const Batch&) = delete;
  static BatchReport Forecast(const BatchConfig&, const Model&, BatchForecast&) noexcept;
  BatchReport InitializeJoined(const BatchConfig&, const Model&);
  BatchReport AssembleAccepted(FENodalState&, const NodalTrialToken&, const NodalAssemblyView&);
  BatchReport EvaluateCandidate(FENodalState&, const NodalTrialToken&, const NodalPreparedView&, BatchDiagnostics*);
  BatchReport CopyAcceptedResults(const NodalStamp&, ResultBuffer, BatchDiagnostics*);
  BatchReport CopyPreparedResults(const BatchDiagnostics&, ResultBuffer);
  BatchReport CopyAcceptedDiagnostics(const NodalStamp&, BatchDiagnostics*) const noexcept;
  void DiscardTrial() noexcept;
  NodalAllocationInfo allocations() const noexcept;
  std::size_t startup_host_bytes() const noexcept;
 private:
  friend class ::tl::fea::ShellBatchPublication;
  friend class BatchQualificationPeer;
  BatchReport PreflightAttach(FENodalState&, const NodalCoefficientLedger&,
      const NodalRigidAssemblyBinding&, const NodalCinWitnessSource&, const Model&,
      const BatchConfig&, const ShellBatchPublication*);
  void AttachPublication(const ShellBatchPublication*) noexcept;
  void ReleasePublication(const ShellBatchPublication*) noexcept;
  void Poison() noexcept;
  BatchReport PreflightPublication(FENodalState&, const NodalTrialToken&,
      const NodalPreparedView&, const BatchDiagnostics&, const ShellBatchPublication*) const noexcept;
  void Publish(const NodalStamp&) noexcept;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tl::fea::beam18
