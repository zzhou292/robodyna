// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Model.h"
#include "../Type45History.h"
#include "../../ShellBatchStartup.h"
#include "../../../solvers/FENodalState.h"

namespace tl::fea {
class ShellBatchPublication;
class ShellPhysicalBinding;
struct NodalCinWitnessSource;
}
namespace tl::fea::type45 {
class BatchQualificationPeer;
enum class BatchProfile { Unspecified, PhysicalAggregateV1 };
struct BatchLimits {
  std::size_t max_joints=4096, max_nodes=524288;
  std::size_t max_host_bytes=1024u<<20, max_device_bytes=16u<<20;
};
struct BatchConfig {
  NodalStamp owner;
  std::uint64_t configuration_id=0, qualification_id=0;
  ShellBatchStartup startup;
  BatchProfile profile=BatchProfile::Unspecified;
  std::size_t cin_attachment_count=0, cin_witness_count=0;
  BatchLimits limits;
};
enum class BatchStatus {
  Success, InvalidInput, NotInitialized, NotBound, ResourceLimit, StaleTrial,
  JointFailure, AssemblyFailure, NonfiniteResult, DeviceFailure, NodalFailure, Unusable
};
struct BatchReport {
  BatchStatus status=BatchStatus::Success;
  const char* message="OK";
  std::size_t joint=SIZE_MAX, node=SIZE_MAX;
  Status joint_status=Status::Success;
  NodalStatus nodal_status=NodalStatus::Ok;
  std::size_t group=SIZE_MAX;
  explicit operator bool() const noexcept {return status==BatchStatus::Success;}
};
enum class BatchPhase { Unspecified, Accepted, Prepared };
struct BatchDiagnostics {
  std::uint64_t source_instance_id=0, owner_id=0, configuration_id=0, qualification_id=0;
  std::uint64_t epoch=0, base_epoch=0, attempt=0;
  double time=0, base_time=0, velocity_time=0, base_velocity_time=0, kick_dt=0;
  BatchPhase phase=BatchPhase::Unspecified;
  std::size_t joint_count=0;
  double native_internal_work_increment_j=0, internal_kick_work_j=0, internal_drift_work_j=0;
  bool valid=false, has_completed_interval=false, accepted_force_assembled=false;
  bool automatic_stiffness_initialized=false;
};
struct Result {
  std::uint64_t source_joint_id=0;
  HistoryValues history;
  Stamp stamp;
  EndpointResult endpoint[2]{};
  Diagnostics diagnostics;
  AutomaticStiffness automatic;
  // At sample zero automatic=false; no future dt or main coefficients are
  // claimed. After first common acceptance this same context stays immutable.
  AutomaticStiffnessContext context;
  bool automatic_stiffness_initialized=false;
};
struct ResultBuffer { Result* joints=nullptr; std::size_t count=0; };
struct BatchForecast {
  std::size_t device_bytes=0, startup_host_bytes=0;
  // Complete startup = retained_model_backing_bytes + incremental_host_bytes.
  // The latter contains Batch/Impl handles, retained readback/context/main
  // staging, and max(upload, initial owner proof); these phases do not overlap.
  std::size_t retained_model_backing_bytes=0, incremental_host_bytes=0;
};

// One physical owner and private common-publication selector. Allocation builds
// only the native pre-automatic TT0 cache. The first candidate authenticates
// owner PhysicalAggregateV1 main coefficients and actual configured dt before
// preparing automatic stiffness. No caller raw coefficient override or clock.
class Batch {
 public:
  Batch();
  ~Batch();
  Batch(const Batch&)=delete;
  Batch& operator=(const Batch&)=delete;
  static BatchReport Forecast(const BatchConfig&,const Model&,BatchForecast&) noexcept;
  BatchReport InitializeJoined(const BatchConfig&,const Model&);
  BatchReport AssembleAccepted(FENodalState&,const NodalTrialToken&,const NodalAssemblyView&);
  BatchReport EvaluateCandidate(FENodalState&,const NodalTrialToken&,const NodalPreparedView&,BatchDiagnostics*);
  BatchReport CopyAcceptedResults(const NodalStamp&,ResultBuffer,BatchDiagnostics*);
  BatchReport CopyPreparedResults(const BatchDiagnostics&,ResultBuffer);
  BatchReport CopyAcceptedDiagnostics(const NodalStamp&,BatchDiagnostics*) const noexcept;
  void DiscardTrial() noexcept;
  NodalAllocationInfo allocations() const noexcept;
  std::size_t startup_host_bytes() const noexcept;
 private:
  friend class ::tl::fea::ShellBatchPublication;
  friend class BatchQualificationPeer;
  BatchReport PreflightAttach(FENodalState&,const ShellPhysicalBinding&,const NodalCinWitnessSource&,const Model&,
      const BatchConfig&,const ShellBatchPublication*);
  void AttachPublication(const ShellBatchPublication*,const ShellPhysicalBinding&) noexcept;
  void ReleasePublication(const ShellBatchPublication*) noexcept;
  BatchReport PreflightPublication(FENodalState&,const NodalTrialToken&,const NodalPreparedView&,
      const BatchDiagnostics&,const ShellBatchPublication*) const noexcept;
  void Publish(const NodalStamp&) noexcept;
  void Poison() noexcept;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tl::fea::type45
