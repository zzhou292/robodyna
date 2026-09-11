// SPDX-License-Identifier: AGPL-3.0-or-later
// CBAVISC and CBAVISNP1: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "QbatForceCoefficients.h"

namespace tl::fea::qbat::detail {
TL_QBAT_HD inline bool PointViscosity(unsigned index,const Material& p,double dn,double dt,
    const Kinematics& k,HistoryValues& h) {
  const double area=k.geometry.point[index].jacobian_m2;
  const auto& rate=k.rate[index];
  const double viscosity=force_constant::onep414*(h.element_active?1.:0.)*dn*
      p.density_kg_m3*p.sound_speed*::sqrt(area);
  const double gg=.5/(1+p.poisson_ratio);
  const double fx=viscosity*(rate[0]+p.poisson_ratio*rate[1]);
  const double fy=viscosity*(rate[1]+p.poisson_ratio*rate[0]);
  const double fxy=viscosity*rate[2]*gg;
  auto& force=h.point[index].force_stress_pa;
  force[0]=force[0]+fx;
  force[1]=force[1]+fy;
  force[2]=force[2]+fxy;
  const double dv=area*h.thickness_m*dt;
  // Native EVIS deliberately has no FXY contribution in this branch.
  h.numerical_viscous_work_j=h.numerical_viscous_work_j+(fx*rate[0]+fy*rate[1])*dv;
  return FiniteValues(force) && tl::math::Finite(viscosity) && tl::math::Finite(dv) &&
      tl::math::Finite(h.numerical_viscous_work_j);
}
TL_QBAT_HD inline bool TransverseViscosity(const Material& p,double dn,double dt,
    const Kinematics& k,HistoryValues& h,Vec3 (&force)[4]) {
  const auto& core=k.geometry.native_vcore;
  const auto& r=k.local_spin;
  const double my13r=core[11]*r[6];
  const double mx13r=core[10]*r[7];
  const double vx=k.corrected_velocity[2].z-
      (my13r-core[7]*(r[0]+r[2])-mx13r+core[6]*(r[1]+r[3]))*.25;
  const double vy=k.corrected_velocity[2].z-
      (my13r-core[9]*(r[0]-r[2])-mx13r+core[8]*(r[1]-r[3]))*.25;
  const double area=k.geometry.area_m2;
  const double c2=(1./12.)*p.shear_modulus*p.density_kg_m3*area;
  const double hvl=2000.*dn*::sqrt(c2)*(h.element_active?1.:0.);
  const double cx=(core[9]*core[9]+core[8]*core[8])*hvl;
  const double cy=(core[7]*core[7]+core[6]*core[6])*hvl;
  const double hsura=h.thickness_m/area;
  const double sx=cx*vx*hsura;
  const double sy=cy*vy*hsura;
  const double sum=sx+sy;
  force[2].z=force[2].z+sum;
  force[3].z=force[3].z-sum;
  const double work=(sx*vx+sy*vy)*dt;
  h.numerical_viscous_work_j=h.numerical_viscous_work_j+work;
  return tl::math::Finite(vx) && tl::math::Finite(vy) && tl::math::Finite(c2) &&
      tl::math::Finite(hvl) && tl::math::Finite(h.numerical_viscous_work_j);
}
} // namespace tl::fea::qbat::detail
