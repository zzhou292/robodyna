// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatForceTypes.h"
#include "../ShellBatchStartup.h"
#include "../ShellResidentLimits.h"
#include "../../solvers/FENodalState.h"
#include <type_traits>

namespace tl::fea::qbat {
// Complete numerical cache, with no borrowed reference/material/curve pointer.
// History owns all four actual surface points and their distinct saved/current
// channels. Immutable source identity and parameters belong to the batch model.
struct BatchResult {
  HistoryValues history;
  HistoryStamp stamp;
  Kinematics kinematics;
  PointObservation point[4];
  Vec3 internal_force_n[4]{};
  Vec3 internal_couple_nm[4]{};
  ForceDiagnostics diagnostics;
};
static_assert(std::is_trivially_copyable_v<BatchResult>);
static_assert(sizeof(BatchResult)==3216,"Four-point cache has an explicit bounded layout");

enum class BatchUsage { Unspecified,PrescribedFields,CoupledForces };
struct BatchConfig {
  NodalStamp owner;
  std::uint64_t configuration_id=0,qualification_id=0;
  std::size_t element_count=0,max_device_bytes=1024*1024;
  BatchUsage usage=BatchUsage::Unspecified;
  ShellBatchStartup startup;
  ShellResidentLimits storage_limits;
};
enum class BatchStatus {
  Success,InvalidInput,NotInitialized,NotBound,ResourceLimit,WrongOwner,StaleTrial,
  InvalidMass,ElementFailure,AssemblyFailure,NonfiniteResult,DeviceFailure,NodalFailure
};
struct BatchReport {
  BatchStatus status=BatchStatus::InvalidInput;
  const char* message="Invalid QBAT batch request";
  std::uint32_t element=UINT32_MAX,node=UINT32_MAX;
  Status element_status=Status::kSuccess;
  NodalStatus nodal_status=NodalStatus::Ok;
};
enum class BatchPhase { Unspecified,Accepted,Prepared };
struct BatchDiagnostics {
  std::uint64_t owner_id=0,configuration_id=0,qualification_id=0;
  std::uint64_t epoch=0,base_epoch=0,attempt=0;
  double time=0,base_time=0,velocity_time=0,base_velocity_time=0,kick_dt=0;
  BatchPhase phase=BatchPhase::Unspecified;
  BatchUsage usage=BatchUsage::Unspecified;
  bool valid=false,has_completed_interval=false,accepted_force_assembled=false;
  std::size_t element_count=0,active_count=0,newly_removed_count=0;
  // Native ledgers are separate. WPLA/EVIS are not additional EINT terms.
  double internal_work_j[2]{},internal_work_increment_j[2]{};
  double plastic_work_j=0,plastic_work_increment_j=0;
  double numerical_viscous_work_j=0,numerical_viscous_work_increment_j=0;
  double minimum_area_ratio=1,minimum_thickness_ratio=1;
  double maximum_displacement=0,maximum_absolute_strain=0;
  double minimum_native_dt=0;
  // This participant's accepted RHS only, using actual base/prepared phases.
  // No family kinetic subtotal, energy closure or timestep policy is exposed.
  double internal_kick_work=0,internal_drift_work=0;
};
} // namespace tl::fea::qbat
