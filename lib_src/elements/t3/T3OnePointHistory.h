// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "T3OnePointTypes.h"
#include "lib_src/elements/sections/ShellLayeredJ2Work.h"
#include "lib_src/materials/Law44MaterialScope.h"

namespace tl::fea::t3::one_point_detail {
TL_T3_HD inline bool ValidMaterial(const ReferenceData& reference,
    const OnePointMaterial& material,OnePointFailure failure) noexcept {
  return detail::SaneReference(reference) &&
      reference.input.placement==ShellReferencePlacement::Centered &&
      sections::ValidLayeredJ2Parameters(material) &&
      sections::MatchesLayeredJ2Material(material,reference.input) &&
      tl::math::Finite(failure.failure_strain) && failure.failure_strain>0;
}
TL_T3_HD inline bool ValidValues(const OnePointHistoryValues& values,
    const OnePointMaterial& material,double time) noexcept {
  using tl::math::Finite;
  const auto& h=values.shell;
  const auto& p=values.point;
  const auto& f=values.failure;
  if (!detail::ValidHistoryValues(h,true) ||
      !detail::FiniteHistoryArray(p.stress) || !Finite(p.plastic_strain) || p.plastic_strain<0 ||
      !tl::material::tabulated_shell_detail::HardeningDomain(material,p.plastic_strain) ||
      !Finite(p.filtered_rate_per_s) || p.filtered_rate_per_s<0 ||
      (!material.rate.enabled && p.filtered_rate_per_s!=0) ||
      !Finite(values.plastic_work_j) || values.plastic_work_j<0 ||
      !Finite(f.damage) || f.damage<0 || f.damage>1 ||
      !Finite(f.failure_time_s) || f.failure_time_s<0 || f.failure_time_s>time ||
      (f.point_active && (f.damage>=1 || f.failure_time_s!=0)) ||
      (!f.point_active && f.damage!=1) || h.active!=(f.point_active?1.:0.)) return false;
  // NPT1/DM0: the saved point and masked parent cache coincide. Arbitrary
  // transverse increments are retained; virgin GS0 transverse stress stays0.
  for (unsigned i=0;i<5;++i) {
    if (h.stress[i]!=p.stress[i] || h.material_stress[i]!=p.stress[i]) return false;
    if ((!f.point_active || i>=3) && p.stress[i]!=0) return false;
  }
  for (double moment:h.bending_stress) if (moment!=0) return false;
  return h.internal_work[1]==0;
}
} // namespace tl::fea::t3::one_point_detail

namespace tl::fea::t3 {
// Prescribed finite values, not native restart authentication. Material curve
// storage remains immutable caller-owned host/device backing.
TL_T3_HD inline Status PrepareOnePointLaw44History(const ReferenceData& reference,
    const OnePointMaterial& material,OnePointFailure failure,
    const OnePointHistoryValues& values,HistoryStamp stamp,OnePointHistory& output) noexcept {
  if (!one_point_detail::ValidMaterial(reference,material,failure) ||
      !tl::math::Finite(stamp.time) || stamp.time<0 ||
      !one_point_detail::ValidValues(values,material,stamp.time)) return Status::kInvalidInput;
  OnePointHistory next;
  const auto status=PrepareFailurePrescribedHistory(reference,values.shell,stamp,next.shell_);
  if (status!=Status::kSuccess) return status;
  next.material_=material;
  next.failure_parameters_=failure;
  next.point_=values.point;
  next.failure_=values.failure;
  next.plastic_work_j_=values.plastic_work_j;
  output=next;
  return Status::kSuccess;
}
TL_T3_HD inline Status InitializeOnePointLaw44History(const ReferenceData& reference,
    const OnePointMaterial& material,OnePointFailure failure,
    HistoryStamp stamp,OnePointHistory& output) noexcept {
  OnePointHistoryValues values;
  values.shell.thickness=reference.input.thickness;
  return PrepareOnePointLaw44History(reference,material,failure,values,stamp,output);
}
} // namespace tl::fea::t3
