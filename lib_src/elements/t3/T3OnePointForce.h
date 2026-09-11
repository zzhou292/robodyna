// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected native C3FORC3 NIP1 recurrence; OpenRadioss (C) 2026 Siemens.
#pragma once
#include "T3OnePointMaterial.h"
#include "T3ForceProjection.h"

namespace tl::fea::t3 {
// Pure endpoint/midpoint operation with caller-owned sample labels. No resident
// owner, allocation, force recomputation, native restart or independent clock.
// All accepted/output fields survive failure, including a prior-output alias.
TL_T3_HD inline Status EvaluateOnePointLaw44Force(const ReferenceData& reference,
    const OnePointMaterial& material,OnePointFailure failure,const OnePointHistory& accepted,
    const PrescribedInterval& interval,OnePointForceTrial& output) noexcept {
  if (!one_point_detail::ValidMaterial(reference,material,failure) ||
      !accepted.prepared() || !accepted.shell().matches_reference(reference) ||
      !tl::material::SameLaw44Parameters(material,accepted.material()) ||
      !detail::SameHistoryBits(failure.failure_strain,accepted.failure_parameters().failure_strain))
    return Status::kInvalidInput;
  const auto& stamp=accepted.shell().stamp();
  if (stamp.time!=interval.base_time || stamp.sample_index==UINT64_MAX ||
      interval.sample_index!=stamp.sample_index+1 ||
      !one_point_detail::ValidValues(accepted.values(),material,stamp.time)) return Status::kInvalidInput;
  double longest=0;
  auto status=detail::CheckPrescribed(reference,interval,longest);
  if (status!=Status::kSuccess) return status;
  detail::GeometryWork geometry;
  status=detail::CurrentGeometry(interval.position,longest,geometry);
  if (status!=Status::kSuccess) return status;
  detail::MaterialWork coefficients;
  if (!one_point_detail::PrepareCoefficients(reference.input,geometry.kinematics.area,
      accepted.shell().data().thickness,coefficients)) return Status::kNonfiniteResult;
  status=detail::EvaluateRates(interval,geometry);
  if (status!=Status::kSuccess) return status;
  auto& kinematics=geometry.kinematics;
  kinematics.base_time=interval.base_time;
  kinematics.position_time=interval.base_time+interval.dt;
  kinematics.velocity_time=interval.base_time+.5*interval.dt;
  kinematics.dt=interval.dt;
  kinematics.sample_index=interval.sample_index;
  kinematics.valid=true;

  OnePointForceTrial next;
  next.kinematics=kinematics;
  const double factor=interval.dt/kinematics.area;
  for (unsigned i=0;i<8;++i)
    next.strain_curvature_increment[i]=kinematics.raw_rate[i]*factor;
  auto values=accepted.values();
  if (!one_point_detail::UpdateMaterial(material,failure,coefficients,next.strain_curvature_increment,
      kinematics.area,interval.dt,kinematics.position_time,values,next.point)) return Status::kNonfiniteResult;
  // SIGEPS44C writes SOUNDSP=sqrt(A11/rho) on every OFF0/1 call;
  // C3DT3 consumes that returned value, not startup LAW1 sqrt(E/rho).
  coefficients.elastic.sound_speed=material.sound_speed;
  detail::StiffnessDiagnostics(geometry,coefficients,next.diagnostics,values.shell.active);
  detail::LocalForceWork local;
  detail::InternalForces(geometry,coefficients,values.shell,local);
  detail::ProjectForces(geometry,local,next.internal_force,next.internal_couple);
  for (unsigned i=0;i<2;++i)
    next.diagnostics.internal_work_increment[i]=values.shell.internal_work[i]-accepted.shell().data().internal_work[i];
  next.plastic_work_increment_j=values.plastic_work_j-accepted.plastic_work_j();
  next.removed_now=accepted.shell().data().active==1 && values.shell.active==0;
  if (!one_point_detail::ValidDiagnostics(next.diagnostics,values.shell.active) ||
      !tl::math::Finite(next.plastic_work_increment_j)) return Status::kNonfiniteResult;
  for (unsigned i=0;i<3;++i)
    if (!detail::Finite(next.internal_force[i]) || !detail::Finite(next.internal_couple[i]))
      return Status::kNonfiniteResult;
  status=PrepareOnePointLaw44History(reference,material,failure,values,
      {kinematics.position_time,interval.sample_index},next.proposed_history);
  if (status!=Status::kSuccess) return Status::kNonfiniteResult;
  output=next;
  return Status::kSuccess;
}
} // namespace tl::fea::t3
