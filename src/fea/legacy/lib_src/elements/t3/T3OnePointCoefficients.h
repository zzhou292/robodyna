// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected C3COEF3 NPT1 and source DM0; OpenRadioss (C) 2026 Siemens.
#pragma once
#include "T3StiffnessDiagnostics.h"

namespace tl::fea::t3::one_point_detail {
TL_T3_HD inline bool PrepareCoefficients(const ReferenceInput& input,double area,
    double reported_thickness,detail::MaterialWork& output) noexcept {
  detail::MaterialWork next;
  if (input.placement!=ShellReferencePlacement::Centered ||
      !tl::material::PrepareShellElasticLaw1(input.young_modulus,input.poisson_ratio,
          input.density,next.elastic)) return false;
  next.thickness=reported_thickness;
  next.thickness2=next.thickness*next.thickness;
  next.volume=next.thickness*area;
  next.nu=input.poisson_ratio;
  next.rho=input.density;
  next.shf=0;
  next.gs=0;
  next.offset=0;
  next.dm=0;
  if (!detail::Positive(next.thickness) || !detail::Positive(next.thickness2) ||
      !detail::Positive(next.volume)) return false;
  output=next;
  return true;
}
TL_T3_HD inline bool ValidDiagnostics(const ForceDiagnostics& value,double active) noexcept {
  return detail::Positive(value.effective_thickness) && detail::Positive(value.native_sound_speed) &&
      value.membrane_viscosity==0 && value.shear_factor==0 && value.transverse_shear_modulus==0 &&
      ((active==1 && detail::Positive(value.translational_stiffness) && detail::Positive(value.rotational_stiffness)) ||
       (active==0 && value.translational_stiffness==0 && value.rotational_stiffness==0)) &&
      detail::Positive(value.unscaled_element_dt) &&
      detail::FiniteHistoryArray(value.internal_work_increment);
}
} // namespace tl::fea::t3::one_point_detail
