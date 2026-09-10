// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type25Types.h"
#include "../ShellBatchStartup.h"
#include "../../solvers/FENodalState.h"
#include <memory>

namespace tl::fea { class NodalMassBinding; class NodalRigidGroupModel; class ShellBatchPublication; }
namespace tl::fea::type25 {
class Model;
struct BatchConfig {
  NodalStamp owner;
  std::uint64_t configuration_id=0,qualification_id=0;
  std::size_t element_count=0,max_nodes=2048,max_connections=1024;
  std::size_t max_device_bytes=2*1024*1024,max_host_bytes=8*1024*1024;
  ShellBatchStartup startup;
};
enum class BatchStatus {
  Success,InvalidInput,NotInitialized,NotBound,ResourceLimit,WrongOwner,StaleTrial,
  InvalidMass,ElementFailure,AssemblyFailure,NonfiniteResult,DeviceFailure,NodalFailure,Unusable
};
struct BatchReport {
  BatchStatus status=BatchStatus::InvalidInput;
  const char* message="Invalid TYPE25 batch request";
  std::uint32_t element=UINT32_MAX,node=UINT32_MAX;
  Status element_status=Status::Success;
  NodalStatus nodal_status=NodalStatus::Ok;
};
enum class BatchPhase { Unspecified,Accepted,Prepared };
struct BatchDiagnostics {
  std::uint64_t source_instance_id=0,owner_id=0,configuration_id=0,qualification_id=0;
  std::uint64_t epoch=0,base_epoch=0,attempt=0;
  double time=0,base_time=0,velocity_time=0,base_velocity_time=0,kick_dt=0;
  BatchPhase phase=BatchPhase::Unspecified;
  bool valid=false,has_completed_interval=false,accepted_force_assembled=false;
  std::size_t element_count=0,active_count=0,newly_failed_count=0;
  double internal_work_J[4]{},internal_work_increment_J[4]{};
  // Signed work of this participant's accepted RHS only; no kinetic sum,
  // dissipative interpretation, energy closure or timestep policy is implied.
  double internal_kick_work=0,internal_drift_work=0;
  // Conservative minimum of all finite native property-coefficient bounds,
  // including inactive elements whose geometry is still tracked in this subset.
  double minimum_native_dt=0;
};

// Joined coupled contributor only. The source model and combined M/J binding
// are immutable retained handles. One bounded arena owns the device model and
// two Evaluation slabs; one accepted selector governs force/history together.
// No nodal owner, time advancement, independent commit or per-step allocation.
// Serialized calls use the actual owner's stream and authenticated live views.
// Initial rest/common translation binds original reference coordinates and
// native X0, with the known zero-stress cache; no dt=0 force call is made.
// The first drifted candidate is the subsequent interval, not native TT=0.
class Batch {
 public:
  Batch();~Batch();
  Batch(const Batch&)=delete;
  Batch& operator=(const Batch&)=delete;
  BatchReport InitializeJoined(const BatchConfig&,const Model&,const NodalMassBinding&);
  BatchReport AssembleAccepted(FENodalState&,const NodalAssemblyView&);
  BatchReport EvaluateCandidate(FENodalState&,const NodalTrialToken&,const NodalPreparedView&,BatchDiagnostics*);
  BatchReport CopyAcceptedDiagnostics(const NodalStamp&,BatchDiagnostics*) const noexcept;
  BatchReport CopyAcceptedResults(const NodalStamp&,Evaluation*,std::size_t capacity,BatchDiagnostics*);
  BatchReport CopyPreparedResults(const BatchDiagnostics&,Evaluation*,std::size_t capacity);
  void DiscardTrial() noexcept;
  NodalAllocationInfo allocations() const noexcept;
  std::size_t host_bytes() const noexcept;
 private:
  friend class ::tl::fea::ShellBatchPublication;
  BatchReport PreflightAttach(const NodalStamp&,const NodalMassBinding&,std::uint64_t configuration_id,
      std::uint64_t qualification_id,const ShellBatchStartup&,const ShellBatchPublication*) const noexcept;
  void AttachPublication(const ShellBatchPublication*) noexcept;
  void ReleasePublication(const ShellBatchPublication*) noexcept;
  BatchReport PreflightPublication(FENodalState&,const NodalTrialToken&,const NodalPreparedView&,
      const BatchDiagnostics&,const ShellBatchPublication*) const noexcept;
  void Publish(const NodalStamp&) noexcept;
  void Poison() noexcept;
  struct Impl;std::unique_ptr<Impl> impl_;
};
namespace batch_detail {
bool SameDiagnostics(const BatchDiagnostics&,const BatchDiagnostics&) noexcept;
}
} // namespace tl::fea::type25
