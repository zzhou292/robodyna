// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss (C) 2026 Siemens, CNDT3 centered LAW1/NODADT1/ALPE1 path.
#pragma once
#include "QephMaterial.h"
#include "QephForceData.h"

namespace tl::fea::qeph::detail {
TL_QEPH_HD inline void StiffnessDiagnostics(const GeometryWork& g,const MaterialWork& m,
                                           ForceDiagnostics& d) {
  const double maximum_viscosity=::fmax(m.dm,m.dn);
  const double reduction=::sqrt(1.+maximum_viscosity*maximum_viscosity)-maximum_viscosity;
  const double length=g.values.characteristic_length*reduction/::sqrt(1.);
  const double offset_factor=1.+.5*::fabs(0.)/m.thickness;
  d.effective_thickness=m.thickness; d.native_sound_speed=m.sound_speed;
  d.membrane_viscosity=m.dm; d.stabilization_viscosity=m.dn;
  d.translational_stiffness=.5*offset_factor*m.volume*m.a11/(length*length);
  d.rotational_stiffness=d.translational_stiffness*(m.thickness2+g.values.area)*
      force_constant::one_over_12+d.translational_stiffness*0.*0.;
  d.unscaled_element_dt=1.*length/m.sound_speed;
}
TL_QEPH_HD inline bool ValidForceDiagnostics(const ForceDiagnostics& d) {
  return Positive(d.effective_thickness)&&Positive(d.native_sound_speed)&&
      Positive(d.translational_stiffness)&&Positive(d.rotational_stiffness)&&
      Positive(d.unscaled_element_dt)&&tl::math::Finite(d.membrane_viscosity)&&
      d.membrane_viscosity>=0&&tl::math::Finite(d.stabilization_viscosity)&&
      d.stabilization_viscosity>=0&&tl::math::Finite(d.internal_work_increment[0])&&
      tl::math::Finite(d.internal_work_increment[1])&&tl::math::Finite(d.hourglass_viscous_work_increment);
}
} // namespace tl::fea::qeph::detail
