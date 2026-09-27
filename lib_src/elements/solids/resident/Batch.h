// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Results.h"
#include "ProfiledResults.h"
#include "../Model.h"
#include "../../ShellBatchStartup.h"
#include "../../../solvers/FENodalState.h"
#include <memory>

namespace tl::fea {
class ShellBatchPublication;
class NodalCoefficientLedger;
class NodalRigidAssemblyBinding;
struct NodalCinWitnessSource;
}
namespace tl::fea::solids {
class BatchQualificationPeer;
enum class BatchProfile { Unspecified, PhysicalCinV1, PhysicalCinExtendedLaw44Law90V2, PhysicalCinSourceControlsV3 };
struct BatchLimits {
  std::size_t max_parents = 16384;
  std::size_t max_materials = 1024;
  std::size_t max_curve_points = 1048576;
  std::size_t max_nodes = 524288;
  std::size_t max_device_bytes = 128u << 20;
  std::size_t max_host_bytes = 256u << 20;
};
// Explicit caller opt-in for native-unit controlled references and worker storage.
// Legacy profiles retain the default hard ceiling; this does not allocate the cap.
inline constexpr BatchLimits SourceControlledBatchLimits() noexcept {
  BatchLimits limits;
  limits.max_device_bytes = 192u << 20;
  return limits;
}
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
  ElementFailure, AssemblyFailure, NonfiniteResult, DeviceFailure, NodalFailure,
  Unusable
};
struct BatchReport {
  BatchStatus status = BatchStatus::Success;
  const char* message = "OK";
  Family family = Family::Solid18;
  std::size_t parent = SIZE_MAX, node = SIZE_MAX;
  // Nonnegative values use the family's force Status enum. -1 means the
  // resulting cache failed its shared stiffness/result validation.
  int element_status = 0;
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
  bool valid = false, has_completed_interval = false, accepted_force_assembled = false;
  // Fixed family order: Solid18, Solid24, Solid6z, Solid18Law44, Solid18Law90. Every admitted solid is active;
  // the qualified rubber cutoff rejects the entire trial instead of deleting it.
  std::size_t parent_count[5]{};
  double native_internal_work_increment_j[5]{};
  double physical_hourglass_work_increment_j[5]{};
  double distortion_work_increment_j[5]{}; // Separate native EINT_DISTOR increment.
  double plastic_work_increment_j = 0; // LAW36/44 only; already part of EINT.
  double internal_kick_work_j = 0, internal_drift_work_j = 0;
  double minimum_native_dt_s = 0;
};
struct BatchForecast {
  std::size_t device_bytes = 0;
  // Includes complete retained Model backing, upload/staging and the temporary
  // initial owner proof. Proof storage and external ledgers are not retained.
  std::size_t startup_host_bytes = 0;
};

// One model, five explicitly profiled typed spans and one private common-publication selector.
// InitializeJoined constructs device TT0 caches only. Live owner admission is
// the private proof below; no unclaimed batch can assemble physical forces.
class Batch {
 public:
  Batch();
  ~Batch();
  Batch(const Batch&) = delete;
  Batch& operator=(const Batch&) = delete;
  static BatchReport Forecast(const BatchConfig&, const Model&, BatchForecast&) noexcept;
  BatchReport InitializeJoined(const BatchConfig&, const Model&);
  BatchReport AssembleAccepted(FENodalState&, const NodalTrialToken&, const NodalAssemblyView&);
  BatchReport EvaluateCandidate(FENodalState&, const NodalTrialToken&,
      const NodalPreparedView&, BatchDiagnostics*);
  BatchReport CopyAcceptedResults(const NodalStamp&, ResultBuffers, BatchDiagnostics*);
  BatchReport CopyPreparedResults(const BatchDiagnostics&, ResultBuffers);
  BatchReport CopyAcceptedResultsWithControls(const NodalStamp&,ProfiledResultBuffers,BatchDiagnostics*);
  BatchReport CopyPreparedResultsWithControls(const BatchDiagnostics&,ProfiledResultBuffers);
  BatchReport CopyAcceptedDiagnostics(const NodalStamp&, BatchDiagnostics*) const noexcept;
  void DiscardTrial() noexcept;
  NodalAllocationInfo allocations() const noexcept;
  std::size_t startup_host_bytes() const noexcept;
 private:
  friend class ::tl::fea::ShellBatchPublication;
  friend class BatchQualificationPeer;
  // All fallible source/owner readbacks occur before any participant is claimed.
  BatchReport PreflightAttach(FENodalState&, const NodalCoefficientLedger&,
      const NodalRigidAssemblyBinding&, const NodalCinWitnessSource&, const Model&,
      const BatchConfig&, const ShellBatchPublication* claimant);
  void AttachPublication(const ShellBatchPublication*) noexcept;
  void ReleasePublication(const ShellBatchPublication*) noexcept;
  void Poison() noexcept;
  BatchReport PreflightPublication(FENodalState&, const NodalTrialToken&,
      const NodalPreparedView&, const BatchDiagnostics&,
      const ShellBatchPublication* claimant) const noexcept;
  void Publish(const NodalStamp&) noexcept;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tl::fea::solids
