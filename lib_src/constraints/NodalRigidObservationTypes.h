// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidGroupModel.h"
#include "NodalRigidGroupState.h"
#if defined(__CUDACC__)
#define TL_RIGID_OBSERVATION_HD __host__ __device__
#else
#define TL_RIGID_OBSERVATION_HD
#endif
namespace tl::fea::rigid {
enum class ObservationStatus { Success,InvalidInput,UnsupportedPhase,InvalidMetric,NonfiniteResult,KickMismatch };
struct ObservationReport {
  ObservationStatus status=ObservationStatus::Success;
  std::size_t member=SIZE_MAX;
  unsigned dof=6; // 0..2 translation, 3..5 rotation; 6 is a group-wide check.
  double residual=0,roundoff_budget=0;
  TL_RIGID_OBSERVATION_HD explicit operator bool() const noexcept { return status==ObservationStatus::Success; }
};
enum class ObservationPhaseKind {
  Unspecified,PhysicalInitialization,
  // Position endpoint / previous velocity midpoint / lagged force-stage axes.
  StoredMidpointWithLaggedFrame
};
struct ObservationPhase {
  ObservationPhaseKind kind=ObservationPhaseKind::Unspecified;
  double position_time=0,velocity_time=0,frame_time=0;
};
struct MemberMotion { Vec3 velocity{},omega{}; };
struct GroupObservationMetric {
  const NodalRigidGroupProperties* group=nullptr;
  const NodalRigidGroupMember* members=nullptr; // Exact group-local source order.
  std::size_t member_count=0;
};
struct GroupKineticInput {
  GroupObservationMetric metric;
  const MemberMotion* members=nullptr;
  NodalRigidGroupState group;
  ObservationPhase phase;
};
struct MemberKineticChannels {
  double translation=0,native_rotation=0,physical_rotation=0,added_rotation=0;
  double total=0; // Translation + authoritative native rotation, exactly once.
  double inertia_partition_residual=0; // Native minus supplied physical/added.
};
struct AggregateKineticChannels {
  double translation=0,rotation=0,total=0;
  double structural_translation=0,primary_translation=0;
  double member_orbital_rotation=0,native_member_rotation=0;
  double physical_member_rotation=0,added_member_rotation=0;
  double primary_parallel_axis_rotation=0,primary_isotropic_rotation=0;
  double principal_correction_rotation=0;
  double decomposition_residual=0,decomposition_roundoff_budget=0;
};
struct GroupKineticObservation {
  ObservationPhase phase;
  MemberKineticChannels members;
  AggregateKineticChannels aggregate;
  double replacement=0; // Aggregate minus only THIS group's native member K.
};
struct GroupKickInput {
  GroupObservationMetric metric;
  const MemberMotion* before_members=nullptr;
  const MemberMotion* after_members=nullptr;
  NodalRigidGroupState before_group,after_group;
  ObservationPhase before_phase,after_phase;
  const Wrench* applied=nullptr;
  const Wrench* reaction=nullptr; // m*a-F and native J*alpha-C, not dissipation.
  double kick_dt=0;
};
struct KickWorkChannels { double translation=0,rotation=0,total=0; };
struct GroupKickObservation {
  GroupKineticObservation before,after;
  KickWorkChannels applied,reaction;
  // Stable quadratic differences from stored values, not subtraction of two
  // large total kinetic samples. Frame change is included in aggregate delta.
  double native_delta=0,aggregate_delta=0,replacement_delta=0;
  double native_residual=0,effective_residual=0,roundoff_budget=0;
};
} // namespace tl::fea::rigid
