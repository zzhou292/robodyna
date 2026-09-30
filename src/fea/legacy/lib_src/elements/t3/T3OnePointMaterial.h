// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected C3STRA3/C3FORC3/MULAWC/FAIL_SETOFF_C, (C) 2026 Siemens.
#pragma once
#include "T3OnePointHistory.h"
#include "T3OnePointCoefficients.h"

namespace tl::fea::t3::one_point_detail {
TL_T3_HD inline double ForceWork(const double (&force)[5],const double (&dx)[8]) noexcept {
  return force[0]*dx[0]+force[1]*dx[1]+force[2]*dx[2]+force[3]*dx[3]+force[4]*dx[4];
}
TL_T3_HD inline double MomentWork(const double (&moment)[3],const double (&dx)[8]) noexcept {
  return moment[0]*dx[5]+moment[1]*dx[6]+moment[2]*dx[7];
}
TL_T3_HD inline double CallerRate(const double (&dx)[8],double thickness,double dt) noexcept {
  using namespace detail::force_constant;
  const double dtinv=dt/::fmax(dt*dt,em20);
  const double curvature=(dx[5]*dx[5]+dx[6]*dx[6]+dx[5]*dx[6]+fourth*(dx[7]*dx[7]))*
      one_over_9*(thickness*thickness);
  const double membrane=four_over_3*(dx[0]*dx[0]+dx[1]*dx[1]+dx[0]*dx[1]+fourth*(dx[2]*dx[2]));
  return ::sqrt(curvature+membrane)*dtinv;
}
TL_T3_HD inline bool UpdateMaterial(const OnePointMaterial& material,OnePointFailure failure,
    const detail::MaterialWork& coefficients,const double (&dx)[8],double area,double dt,double time,
    OnePointHistoryValues& next,sections::MembraneLaw44PointResult& observation) noexcept {
  using namespace detail::force_constant;
  auto& h=next.shell;
  const double old_active=h.active;
  const double thickness=coefficients.thickness;
  const double volume=area*thickness;
  // NPG1: the native NPG>1 old-work OFF mask does not execute.
  double membrane_work=ForceWork(h.stress,dx);
  double bending_work=MomentWork(h.bending_stress,dx);
  for (unsigned i=0;i<8;++i) h.strain_curvature[i]=h.strain_curvature[i]+dx[i];
  const double rate=CallerRate(dx,h.thickness,dt);
  h.equivalent_strain_rate=1.*rate+(1.-1.)*h.equivalent_strain_rate;

  tl::material::TabulatedShellPlasticityInput point_input;
  const double z=0.*thickness;
  for (unsigned i=0;i<3;++i) point_input.strain_increment[i]=dx[i]+z*dx[i+5];
  point_input.strain_increment[3]=dx[3];
  point_input.strain_increment[4]=dx[4];
  point_input.transverse_shear_modulus=0;
  point_input.dt=dt;
  point_input.total_strain_rate_per_s=rate;
  point_input.element_active=old_active==1;
  sections::MembraneLaw44PointResult packet;
  if (!sections::UpdateZeroShearLaw44Point(material,failure,next.point,next.failure,
      {point_input,thickness,area,time},packet)) return false;
  next.point=packet.saved;
  next.failure=packet.failure.history;
  next.plastic_work_j=next.plastic_work_j+packet.plastic_work_increment_j;

  // TYPE1 NPT1 broken fraction is1. FAIL_SETOFF_C marks OFF=.8 and MULAWC
  // publishes OFF0 before its final force mask. No surface-point delay.
  if (h.active==1 && !next.failure.point_active) h.active=0;
  const auto& stress=packet.current.history.stress;
  for (unsigned i=0;i<5;++i) {
    h.stress[i]=0.+1.*stress[i];
    h.material_stress[i]=h.stress[i]*h.active;
  }
  for (unsigned i=0;i<3;++i) h.bending_stress[i]=0.+0.*stress[i];
  const double dtinv=dt/::fmax(dt*dt,em20);
  const double fact=onep414*coefficients.dm;
  const double viscosity=fact*material.sound_speed*::sqrt(area)*dtinv*material.density_kg_m3;
  h.stress[0]=h.stress[0]+viscosity*(dx[0]+.5*dx[1]);
  h.stress[1]=h.stress[1]+viscosity*(dx[1]+.5*dx[0]);
  h.stress[2]=h.stress[2]+viscosity*dx[2]*third;
  for (double& value:h.stress) value=value*h.active*1.;
  for (double& value:h.bending_stress) value=value*h.active*1.;
  membrane_work=membrane_work+h.stress[0]*dx[0]+h.stress[1]*dx[1]+h.stress[2]*dx[2]+
      h.stress[3]*dx[3]+h.stress[4]*dx[4];
  bending_work=bending_work+h.bending_stress[0]*dx[5]+h.bending_stress[1]*dx[6]+h.bending_stress[2]*dx[7];
  const double vol2=.5*volume;
  h.internal_work[0]=h.internal_work[0]+membrane_work*vol2;
  h.internal_work[1]=h.internal_work[1]+bending_work*thickness*vol2;
  h.thickness=::fmax(packet.reported_thickness_m,em30);
  if (!tl::math::Finite(viscosity) || !tl::math::Finite(rate) ||
      !ValidValues(next,material,time)) return false;
  observation=packet;
  return true;
}
} // namespace tl::fea::t3::one_point_detail
