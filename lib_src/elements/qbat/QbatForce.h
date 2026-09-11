// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected four-point CBAFORC3 recurrence: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "QbatHistory.h"
#include "QbatStrain.h"
#include "QbatMaterialPoint.h"
#include "QbatViscosity.h"
#include "QbatForceAssembly.h"

namespace tl::fea::qbat {
// Pure proposed interval. No GPU allocation, owner publication or new clock.
// All output fields, including aliased accepted history, survive any failure.
TL_QBAT_HD inline Status EvaluateForce(const Reference& reference,const Material& material,
    Failure failure,const History& accepted,const PrescribedInterval& interval,ForceTrial& output) noexcept {
  using tl::math::Finite;
  if (!accepted.prepared() || !detail::ValidMaterial(reference,material,failure) ||
      !detail::SameReference(reference,accepted.reference()) ||
      !detail::SameMaterial(material,accepted.material()) ||
      !detail::Same(failure.failure_strain,accepted.failure().failure_strain)) return Status::kInvalidReference;
  const auto stamp=accepted.stamp();
  const double endpoint=interval.base_time+interval.dt;
  if (!Finite(interval.base_time) || interval.base_time<0 || !detail::Positive(interval.dt) ||
      !Finite(endpoint) || endpoint<=interval.base_time || interval.base_time!=stamp.time ||
      stamp.sample_index==UINT64_MAX || interval.sample_index!=stamp.sample_index+1 ||
      !detail::ValidHistory(accepted.data(),material,stamp.time)) return Status::kInvalidInput;
  for (unsigned i=0;i<4;++i) {
    if (!detail::Finite(interval.position_endpoint[i]) || !detail::Finite(interval.velocity_midpoint[i]) ||
        !detail::Finite(interval.omega_midpoint[i])) return Status::kInvalidInput;
  }
  ForceTrial trial;
  auto& k=trial.kinematics;
  CurrentInput geometry_input;
  for (unsigned i=0;i<4;++i) geometry_input.position_m[i]=interval.position_endpoint[i];
  geometry_input.native_off=accepted.data().element_active?1.:0.;
  // For OFF0/1 CBACOOR has the same current shape operations; only the
  // lifecycle-dependent force/rate masks are handled by the stages below.
  auto status=detail::CurrentSurfaceGeometry(reference,geometry_input,k.geometry);
  if (status!=Status::kSuccess) return status;
  if (!detail::CorrectVelocity(interval,k) || !detail::CurrentLength(interval,k)) return Status::kNonfiniteResult;
  const auto& base=accepted.data();
  auto next=base;
  Vec3 local_force[4]{};
  const auto& options=reference.input().options;
  const double dn=options.numerical_viscosity==0 ? 1./1000. : options.numerical_viscosity;
  const double initial_volume=base.thickness_m*k.geometry.area_m2;
  if (!detail::Positive(initial_volume) ||
      !detail::PointStrain(0,interval.dt,next.thickness_m,k)) return Status::kNonfiniteResult;
  detail::AggregateShearWork(next,k.geometry.area_m2,k.rate[0][2],interval.dt,next.internal_work_j[0]);
  double reported_rate=0;
  for (unsigned point=0;point<4;++point) {
    if (point!=0 && !detail::PointStrain(point,interval.dt,next.thickness_m,k)) return Status::kNonfiniteResult;
    for (unsigned j=0;j<8;++j) {
      const double increment=k.strain_increment[point][j];
      next.strain[j]=next.strain[j]+increment*(1./4.);
      next.point[point].strain[j]=next.point[point].strain[j]+increment;
    }
    reported_rate=reported_rate+k.equivalent_rate_per_s[point]/4;
    if (!detail::UpdateMaterialPoint(point,material,failure,interval.dt,endpoint,
        options.membrane_viscosity,k,next,trial.point[point])) return Status::kNonfiniteResult;
    if (!detail::PointViscosity(point,material,dn,interval.dt,k,next)) return Status::kNonfiniteResult;
    detail::PointForces(k.geometry.point[point],next.point[point],
        trial.point[point].force_volume_m3,local_force);
  }
  next.reported_rate_per_s=1.*reported_rate+(1.-1.)*base.reported_rate_per_s;
  for (unsigned j=0;j<5;++j) {
    next.force_stress_pa[j]=.25*(next.point[0].force_stress_pa[j]+next.point[1].force_stress_pa[j]+
        next.point[2].force_stress_pa[j]+next.point[3].force_stress_pa[j]);
  }
  detail::ConstantShearForce(k.geometry,next,initial_volume,local_force);
  detail::AggregateShearWork(next,k.geometry.area_m2,k.rate[3][2],interval.dt,next.internal_work_j[0]);
  if (!detail::TransverseViscosity(material,dn,interval.dt,k,next,local_force) ||
      !detail::ProjectForces(k.geometry,local_force,trial.internal_force_n,trial.internal_couple_nm))
    return Status::kNonfiniteResult;
  auto& d=trial.diagnostics;
  if (!detail::ForceCoefficients(reference,material,base,next.element_active,k,d)) return Status::kNonfiniteResult;
  d.removed_now=base.element_active && !next.element_active;
  for (unsigned j=0;j<2;++j) d.internal_work_increment_j[j]=next.internal_work_j[j]-base.internal_work_j[j];
  d.plastic_work_increment_j=next.plastic_work_j-base.plastic_work_j;
  d.numerical_viscous_work_increment_j=next.numerical_viscous_work_j-base.numerical_viscous_work_j;
  if (!detail::FiniteValues(d.internal_work_increment_j) || !Finite(d.plastic_work_increment_j) ||
      !Finite(d.numerical_viscous_work_increment_j)) return Status::kNonfiniteResult;
  status=PreparePrescribedHistory(reference,material,failure,next,{endpoint,interval.sample_index},trial.proposed_history);
  if (status!=Status::kSuccess) return Status::kNonfiniteResult;
  output=trial;
  return Status::kSuccess;
}
} // namespace tl::fea::qbat
