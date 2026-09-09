// SPDX-License-Identifier: AGPL-3.0-or-later
// C3DT3 selected native expressions, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "T3ForceData.h"
#include "T3Material.h"
namespace tl::fea::t3::detail {
TL_T3_HD inline void StiffnessDiagnostics(const GeometryWork& g,const MaterialWork& m,ForceDiagnostics& d) {
  const double viscmx=::sqrt(1.+m.dm*m.dm)-m.dm;
  const double length=g.kinematics.characteristic_length*viscmx/::sqrt(g.kinematics.area_scale);
  const double f_offset=1.+.5*::fabs(m.offset)/m.thickness;
  const double athk=g.kinematics.area*m.thickness;
  d.effective_thickness=m.thickness; d.native_sound_speed=m.elastic.sound_speed;
  d.membrane_viscosity=m.dm; d.shear_factor=m.shf; d.transverse_shear_modulus=m.gs;
  d.translational_stiffness=athk*f_offset*m.elastic.a11/(length*length);
  d.rotational_stiffness=d.translational_stiffness*(m.thickness*m.thickness*force_constant::one_over_12+
      .5*m.shf*g.kinematics.area*m.elastic.g/m.elastic.a11)+d.translational_stiffness*m.offset*m.offset;
  d.unscaled_element_dt=1.*1.*length/m.elastic.sound_speed;
}
TL_T3_HD inline bool ValidForceDiagnostics(const ForceDiagnostics& d) {
  return Positive(d.effective_thickness)&&Positive(d.native_sound_speed)&&
      d.membrane_viscosity==force_constant::viscosity&&Positive(d.shear_factor)&&
      Positive(d.transverse_shear_modulus)&&Positive(d.translational_stiffness)&&
      Positive(d.rotational_stiffness)&&Positive(d.unscaled_element_dt)&&
      tl::math::Finite(d.internal_work_increment[0])&&tl::math::Finite(d.internal_work_increment[1]);
}
} // namespace tl::fea::t3::detail
