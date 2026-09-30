// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidObservationTypes.h"
#include "NodalRigidGroupStepMath.h"
namespace tl::fea::rigid {
// Native RGBCOR force-stage observation, not an accepted endpoint velocity phase.
struct ForceStageObservationPhase {
  double force_time=0,input_velocity_time=0,previous_frame_time=0;
  StepDurations durations{}; // Native DT1, DT12, DT2; no independently owned clock.
};
struct ForceStageAcceleration { Vec3 translation{},rotation{}; };
struct GroupForceStageKineticInput {
  GroupObservationMetric metric;
  const MemberMotion* before_members=nullptr; // Exact metric source order.
  const ForceStageAcceleration* member_acceleration=nullptr;
  MemberMotion before_primary;
  ForceStageAcceleration primary_acceleration;
  tl::math::Matrix3 force_frame{}; // Already updated by the actual primary packet.
  ForceStageObservationPhase phase;
};
struct GroupForceStageKineticObservation {
  std::uint64_t source_group_id=0,source_node_set_id=0;
  std::size_t member_count=0;
  ForceStageObservationPhase phase;
  MemberMotion collocated_primary;
  MemberKineticChannels members;
  AggregateKineticChannels aggregate;
  double replacement=0;
};
} // namespace tl::fea::rigid
