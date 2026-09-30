// SPDX-License-Identifier: AGPL-3.0-or-later
// NPTT1 MULAWC and FAIL_SETOFF_NPG_C: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "QbatForceCoefficients.h"
#include "lib_src/elements/sections/ShellLaw44MembranePoint.h"

namespace tl::fea::qbat::detail {
TL_QBAT_HD inline double StressWork(const double (&force)[5],const double (&dx)[8]) {
  return force[0]*dx[0]+force[1]*dx[1]+force[2]*dx[2]+force[3]*dx[3]+force[4]*dx[4];
}
TL_QBAT_HD inline void PointShearWork(double stress,double shear,double volume,
    bool active,double& work) {
  const double thoff=.5*(active?1.:0.)*volume;
  work=work-stress*shear*thoff;
}
TL_QBAT_HD inline void AggregateShearWork(const HistoryValues& h,double area,double rate,
    double dt,double& work) {
  const double thoff=(h.element_active?1.:0.)*area*h.thickness_m*dt*.5;
  work=work+h.force_stress_pa[2]*rate*thoff;
}
TL_QBAT_HD inline bool UpdateMaterialPoint(unsigned index,const Material& material,Failure failure,
    double dt,double time,double dm,const Kinematics& k,HistoryValues& next,PointObservation& observation) {
  auto& surface=next.point[index];
  const auto& dx=k.strain_increment[index];
  const double area=k.geometry.point[index].jacobian_m2;
  const double thickness=next.thickness_m;
  const double volume=thickness*area;
  const bool old_active=next.element_active;
  double work=StressWork(surface.force_stress_pa,dx)*(old_active?1.:0.);
  PointShearWork(surface.force_stress_pa[2],dx[2],volume,old_active,next.internal_work_j[0]);
  tl::material::TabulatedShellPlasticityInput input;
  for (unsigned j=0;j<5;++j) input.strain_increment[j]=dx[j];
  input.transverse_shear_modulus=0;
  input.dt=dt;
  input.total_strain_rate_per_s=k.equivalent_rate_per_s[index];
  input.element_active=old_active;
  sections::MembraneLaw44PointResult packet;
  if (!sections::UpdateMembraneLaw44Point(material,failure,surface.material,surface.failure,
      {input,thickness,area,time},packet)) return false;
  const auto& result=packet.current;
  const auto& failed=packet.failure;
  observation.material=result;
  observation.thickness_before_m=thickness;
  observation.force_volume_m3=volume;
  double reported=packet.reported_thickness_m;
  next.plastic_work_j=next.plastic_work_j+packet.plastic_work_increment_j;
  surface.failure=failed.history;
  observation.failed_now=failed.failed_now;
  surface.material=packet.saved;
  if (old_active && !failed.history.point_active) surface.surface_active=false;
  if (index==3 && next.element_active) {
    bool any=false;
    for (const auto& point:next.point) any=any || point.surface_active;
    if (!any) next.element_active=false;
  }
  // MULAWC retains unmasked current stress until final parent OFF is known.
  // WF=1, WM=0. Constitutive transverse and moment channels remain exact zero.
  for (unsigned j=0;j<5;++j) surface.force_stress_pa[j]=0.+1.*result.history.stress[j];
  const double dtinv=dt/::fmax(dt*dt,1e-20);
  const double fact=force_constant::onep414*dm;
  const double viscosity=fact*material.sound_speed*::sqrt(area)*dtinv*material.density_kg_m3;
  auto& force=surface.force_stress_pa;
  force[0]=force[0]+viscosity*(dx[0]+.5*dx[1]);
  force[1]=force[1]+viscosity*(dx[1]+.5*dx[0]);
  force[2]=force[2]+viscosity*dx[2]*(1./3.);
  const double parent_mask=next.element_active?1.:0.;
  for (double& stress:force) stress=stress*parent_mask*1.;
  work=work+force[0]*dx[0]+force[1]*dx[1]+force[2]*dx[2]+force[3]*dx[3]+force[4]*dx[4];
  const double vol2=.5*volume;
  next.internal_work_j[0]=next.internal_work_j[0]+work*vol2;
  // The point's new shear correction uses its final OFF, before CBAVISC.
  PointShearWork(force[2],dx[2],volume,next.element_active,next.internal_work_j[0]);
  reported=::fmax(reported,force_constant::em30);
  observation.thickness_material_m=reported;
  next.thickness_m=reported-(3./4.)*(reported-thickness);
  observation.thickness_after_m=next.thickness_m;
  return Positive(next.thickness_m) && Positive(volume) && tl::math::Finite(viscosity) &&
      FiniteValues(force) && FiniteValues(next.internal_work_j) &&
      tl::math::Finite(next.plastic_work_j);
}
} // namespace tl::fea::qbat::detail
