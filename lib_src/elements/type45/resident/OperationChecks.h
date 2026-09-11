// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
#include "../../../solvers/NodalNativePhysicalCoefficients.h"

namespace tl::fea::type45::resident_detail {
inline BatchReport NodalFailure(const NodalReport& r) noexcept {
  return {BatchStatus::NodalFailure,r.message,SIZE_MAX,r.node,Status::Success,r.status};
}
inline bool CompleteAssembly(const NodalAssemblyView& v,const NodalStamp& s) noexcept {
  return trial_identity::ValidKinematics(v.accepted,s.node_count,s.epoch) &&
      v.forces.node_count==s.node_count && v.forces.base_epoch==s.epoch &&
      v.bounds && v.result && v.forces.force_x && v.forces.force_y && v.forces.force_z &&
      v.forces.couple_x && v.forces.couple_y && v.forces.couple_z &&
      v.mass.inverse_mass && v.mass.fixed && v.inverse_inertia && v.translation_fixed_bits && v.rotation_fixed;
}
inline bool CandidatePhase(const NodalStamp& s,const NodalPreparedView& v) noexcept {
  const double h=s.fixed_dt;
  return trial_identity::ValidKinematics(v.kinematics,s.node_count,s.epoch) &&
      trial_identity::ValidKinematics(v.base_kinematics,s.node_count,s.epoch) &&
      v.temporal_scheme==s.temporal_scheme && v.base_velocity_phase==s.velocity_phase &&
      v.velocity_phase==NodalVelocityPhase::PreviousMidpoint && v.base_time==s.time &&
      v.base_velocity_time==s.velocity_time && v.proposed_time==s.time+h && detail::Finite(v.proposed_time) &&
      v.proposed_time>s.time && v.velocity_time==s.time+.5*h && v.kick_dt==(s.epoch?h:.5*h) && s.epoch!=UINT64_MAX;
}
} // namespace tl::fea::type45::resident_detail
