// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
#include "../../../solvers/NodalNativePhysicalCoefficients.h"

namespace tl::fea::solids::batch_detail {
inline BatchReport NodalFailure(const NodalReport& report) noexcept {
  return {BatchStatus::NodalFailure, report.message, Family::Solid18, SIZE_MAX,
      report.node, 0, report.status};
}
inline bool CompleteAssembly(const NodalAssemblyView& view, const NodalStamp& stamp) noexcept {
  return trial_identity::ValidKinematics(view.accepted, stamp.node_count, stamp.epoch) &&
      view.forces.node_count == stamp.node_count && view.forces.base_epoch == stamp.epoch &&
      view.bounds && view.result && view.forces.force_x && view.forces.force_y && view.forces.force_z;
}
inline bool CandidatePhase(const NodalStamp& stamp, const NodalPreparedView& view) noexcept {
  const auto count = stamp.node_count;
  const double h = stamp.fixed_dt;
  return trial_identity::ValidKinematics(view.kinematics, count, stamp.epoch) &&
      trial_identity::ValidKinematics(view.base_kinematics, count, stamp.epoch) &&
      view.temporal_scheme == stamp.temporal_scheme && view.base_velocity_phase == stamp.velocity_phase &&
      view.velocity_phase == NodalVelocityPhase::PreviousMidpoint &&
      view.base_time == stamp.time && view.base_velocity_time == stamp.velocity_time &&
      view.proposed_time == stamp.time + h && tl::math::Finite(view.proposed_time) &&
      view.proposed_time > stamp.time && view.velocity_time == stamp.time + .5 * h &&
      view.kick_dt == (stamp.epoch ? h : .5 * h) && stamp.epoch != UINT64_MAX;
}
} // namespace tl::fea::solids::batch_detail
