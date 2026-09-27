// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid6zForceInitialize.h"
#include "Solid6zForceGeometry.h"
#include "Solid6zForceResultants.h"
#include "Solid6zForceMaterial.h"
#include "Solid6zHourglassResponse.h"
#include "lib_src/materials/law42/Caller.h"

namespace tl::fea::solid6z {
namespace force_detail {
TL_BRICK_HD inline Status CalculateForce(const Reference& reference, const History& accepted,
    const PrescribedInterval& interval, const Material& material, const ForceProfile& profile,
    ForceTrial& output, bool initialization) noexcept {
  ForceTrial next;
  Status status = force_detail::Current(reference,interval,next.geometry);
  if (status != Status::Success) return status;
  status=force_detail::EvaluateMaterial(reference,material,next.geometry,interval.dt_s,
      accepted.data().material,initialization,next.material);
  if(status!=Status::Success)return status;
  if (!force_detail::MaterialForces(next.geometry,next.material,next.material_local_force_n)) return Status::NonfiniteResult;
  HistoryValues proposed = accepted.data();
  proposed.material = next.material.history;
  for (unsigned n = 0; n < 6; ++n) next.stabilized_local_force_n[n] = next.material_local_force_n[n];
  status = force_detail::Stabilize(reference,material,profile,next.geometry,interval.dt_s,
      next.material,proposed,next.stabilized_local_force_n,next.stabilization);
  if (status != Status::Success) return status;
  if (!force_detail::Project(reference,next.geometry.frame,next.stabilized_local_force_n,
                            next.rhs_force_n)) return Status::NonfiniteResult;
  next.total_internal_work_increment_j = next.material.internal_work_j+
      next.stabilization.first_work_j+next.stabilization.second_work_j;
  if (!tl::math::Finite(next.total_internal_work_increment_j)) return Status::NonfiniteResult;
  const HistoryStamp stamp{interval.base_time_s+interval.dt_s,initialization ? 0 : interval.sample_index+1};
  status = PreparePrescribedHistory(reference,material,profile,proposed,stamp,next.proposed_history);
  if (status != Status::Success) return status;
  output = next;
  return Status::Success;
}
} // namespace force_detail
TL_BRICK_HD inline Status EvaluateForce(const Reference& reference, const History& accepted,
    const PrescribedInterval& interval, const Material& material, const ForceProfile& profile,
    ForceTrial& output) noexcept {
  if (!accepted.prepared() || !force_detail::Same(reference,accepted.reference()) ||
      !force_detail::Same(material,accepted.material()) ||
      !force_detail::Same(profile,accepted.profile()) ||
      !force_detail::Valid(interval,accepted.stamp())) return Status::InvalidInput;
  return force_detail::CalculateForce(reference,accepted,interval,material,profile,output,false);
}
// Exact reference geometry and uniform world translation only. Constructor
// output has native TT0 fields at sample0, never a completed dt0 interval.
TL_BRICK_HD inline Status InitializeForce(const Reference& reference, const Material& material,
    const ForceProfile& profile, Vec3 uniform_velocity_m_s, ForceTrial& output) noexcept {
  if (!force_detail::common::Finite(uniform_velocity_m_s)) return Status::InvalidInput;
  History virgin;
  const Status status=InitializeHistory(reference,material,profile,virgin);
  if (status!=Status::Success) return status;
  PrescribedInterval initial;
  for (unsigned n=0; n<6; ++n) {
    initial.position_endpoint_m[n]=reference.input().position_m[n];
    initial.velocity_midpoint_m_s[n]=uniform_velocity_m_s;
  }
  return force_detail::CalculateForce(reference,virgin,initial,material,profile,output,true);
}
} // namespace tl::fea::solid6z
