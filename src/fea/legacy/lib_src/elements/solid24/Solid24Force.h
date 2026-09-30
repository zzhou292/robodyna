// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SZFORC3/SFINT3/SRROTA3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid24ForceHistory.h"
#include "Solid24ForceKinematics.h"
#include "Solid24Stabilization.h"
#include "Solid24ForceMaterial.h"
#include "Solid24ForceResultants.h"

namespace tl::fea::solid24::force_detail {
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
  status=force_detail::EvaluateMaterial(reference,material,trial.geometry,interval.dt_s,
      accepted.values().material,initialization,trial.diagnostics.material);
  if (status!=ForceStatus::Success) return status;
  next=accepted.values();
  next.material=trial.diagnostics.material.history;
  Vec3 local_force[8];
  status=force_detail::Stabilization(reference,material,trial.geometry,interval.dt_s,
      next,trial.diagnostics,local_force);
  if (status!=ForceStatus::Success) return status;
  force_detail::MaterialForces(trial.geometry,next.material,local_force);
  return force_detail::RotateAndMapForces(reference,trial.geometry.current.frame,
      local_force,trial.rhs_force_n);
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
