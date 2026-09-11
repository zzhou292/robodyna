// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SZFORC3/SFINT3/SRROTA3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid24ForceHistory.h"
#include "Solid24ForceKinematics.h"
#include "Solid24Stabilization.h"

namespace tl::fea::solid24::force_detail {
TL_BRICK_HD inline void MaterialForces(const ForceGeometry& g,
    const tl::material::law42::CallerHistory& history,Vec3 (&force)[8]) noexcept {
  double s[6];
  for (unsigned k=0; k<3; ++k)
    s[k]=(history.stress_pa[k]+0.0-history.bulk_pressure_pa)*g.current.volume_m3;
  for (unsigned k=3; k<6; ++k) s[k]=(history.stress_pa[k]+0.0)*g.current.volume_m3;
  constexpr unsigned opposite[]{6,7,4,5};
  const auto& p=g.derivative_per_m;
  for (unsigned n=0; n<4; ++n) {
    const double x=s[0]*p[0][n]+s[3]*p[1][n]+s[5]*p[2][n];
    const double y=s[1]*p[1][n]+s[3]*p[0][n]+s[4]*p[2][n];
    const double z=s[2]*p[2][n]+s[5]*p[0][n]+s[4]*p[1][n];
    force[n].x=force[n].x-x; force[opposite[n]].x=force[opposite[n]].x+x;
    force[n].y=force[n].y-y; force[opposite[n]].y=force[opposite[n]].y+y;
    force[n].z=force[n].z-z; force[opposite[n]].z=force[opposite[n]].z+z;
  }
}
TL_BRICK_HD inline bool ValidInterval(const History& accepted,const PrescribedInterval& i) noexcept {
  if (!accepted.initialized() || !tl::math::Finite(i.base_time_s) ||
      i.base_time_s!=accepted.stamp().time_s || !brick::Positive(i.dt_s) ||
      !tl::math::Finite(i.base_time_s+i.dt_s) || i.base_time_s+i.dt_s<=i.base_time_s ||
      accepted.stamp().sample_index==UINT64_MAX ||
      i.sample_index!=accepted.stamp().sample_index+1) return false;
  for (unsigned n=0; n<8; ++n)
    if (!brick::Finite(i.position_m[n]) || !brick::Finite(i.velocity_m_s[n])) return false;
  return true;
}
} // namespace tl::fea::solid24::force_detail

namespace tl::fea::solid24 {
namespace force_detail {
TL_BRICK_HD inline ForceStatus CalculateForce(const Reference& reference,const History& accepted,
    const PrescribedInterval& interval,const Material& material,ForceTrial& trial,
    HistoryValues& next,bool initialization) noexcept {
  ForceStatus status=force_detail::CurrentKinematics(reference,interval,trial.geometry);
  if (status!=ForceStatus::Success) return status;
  tl::material::law42::CallerInput input;
  input.dt_s=interval.dt_s;
  input.current_volume_m3=trial.geometry.current.volume_m3;
  input.storage_volume_m3=reference.geometry().volume_m3;
  input.characteristic_length_m=trial.geometry.current.characteristic_length_m;
  for (unsigned k=0; k<9; ++k)
    input.displacement_gradient[k]=trial.geometry.material_displacement_gradient[k];
  for (unsigned k=0; k<6; ++k)
    input.engineering_rate_per_s[k]=trial.geometry.engineering_rate_per_s[k];
  const auto material_status=initialization ?
      tl::material::law42::InitializeCaller(material,input,trial.diagnostics.material) :
      tl::material::law42::UpdateCaller(material,accepted.values().material,input,trial.diagnostics.material);
  if (material_status!=tl::material::law42::Status::Ok) return ForceStatus::MaterialFailure;
  next=accepted.values();
  next.material=trial.diagnostics.material.history;
  Vec3 local_force[8];
  status=force_detail::Stabilization(reference,material,trial.geometry,interval.dt_s,
      next,trial.diagnostics,local_force);
  if (status!=ForceStatus::Success) return status;
  force_detail::MaterialForces(trial.geometry,next.material,local_force);
  const auto& f=trial.geometry.current.frame.v;
  for (unsigned n=0; n<8; ++n) {
    const auto& a=local_force[n];
    const Vec3 world{f[0]*a.x+f[1]*a.y+f[2]*a.z,
                     f[3]*a.x+f[4]*a.y+f[5]*a.z,
                     f[6]*a.x+f[7]*a.y+f[8]*a.z};
    if (!force_detail::brick::Finite(world)) return ForceStatus::NonfiniteResult;
    trial.rhs_force_n[reference.source_slot(n)]=world;
  }
  return ForceStatus::Success;
}
} // namespace force_detail
TL_BRICK_HD inline ForceStatus EvaluateForce(const Reference& reference,const History& accepted,
    const PrescribedInterval& interval,const Material& material,ForceTrial& output) noexcept {
  if (!force_detail::ValidMaterial(reference,material) ||
      !force_detail::ValidInterval(accepted,interval)) return ForceStatus::InvalidInput;
  if (!force_detail::SameReference(reference,accepted.reference()) ||
      !force_detail::SameMaterial(material,accepted.material())) return ForceStatus::ReferenceMismatch;
  ForceTrial trial;
  HistoryValues next;
  const auto status=force_detail::CalculateForce(reference,accepted,interval,material,trial,next,false);
  if (status!=ForceStatus::Success) return status;
  trial.proposed_history=accepted;
  trial.proposed_history.values_=next;
  trial.proposed_history.stamp_={interval.base_time_s+interval.dt_s,interval.sample_index};
  output=trial;
  return ForceStatus::Success;
}
// Constructor-only native TT0 initialization; no previous history is accepted.
TL_BRICK_HD inline ForceStatus InitializeForce(const Reference& reference,const Material& material,
    Vec3 uniform_velocity_m_s,ForceTrial& output) noexcept {
  if (!force_detail::brick::Finite(uniform_velocity_m_s)) return ForceStatus::InvalidInput;
  History virgin;
  auto status=InitializeHistory(reference,material,virgin);
  if (status!=ForceStatus::Success) return status;
  PrescribedInterval initial;
  for (unsigned n=0; n<8; ++n) {
    initial.position_m[n]=reference.input().position_m[n];
    initial.velocity_m_s[n]=uniform_velocity_m_s;
  }
  ForceTrial trial;
  HistoryValues next;
  status=force_detail::CalculateForce(reference,virgin,initial,material,trial,next,true);
  if (status!=ForceStatus::Success) return status;
  trial.proposed_history=virgin;
  trial.proposed_history.values_=next;
  output=trial;
  return ForceStatus::Success;
}
} // namespace tl::fea::solid24
