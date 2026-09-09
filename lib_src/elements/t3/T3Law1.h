// SPDX-License-Identifier: AGPL-3.0-or-later
// C3STRA3, selected C3FORC3 EPSD and MULAWGLC expressions, (C) 2026 Siemens.
// Shared SIGEPS01G point update; T3 owns thickness/work/instantaneous DM.
#pragma once
#include "T3HistoryData.h"
#include "T3Material.h"
namespace tl::fea::t3::detail {
TL_T3_HD inline bool UpdateLaw1(const GeometryWork& geometry,const MaterialWork& m,double dt,HistoryValues& h) {
  using namespace force_constant;
  const auto& g=geometry.kinematics;
  const double fac1=dt/g.area;
  double dx[8]{};
  for(unsigned i=0;i<8;++i) { dx[i]=g.raw_rate[i]*fac1; h.strain_curvature[i]=h.strain_curvature[i]+dx[i]; }
  // T3's public raw order is already material YZ/ZX: no QEPH array swap.
  const double dtinv=dt/::fmax(dt*dt,em20);
  const double eps_k2=(dx[5]*dx[5]+dx[6]*dx[6]+dx[5]*dx[6]+fourth*(dx[7]*dx[7]))*one_over_9*(h.thickness*h.thickness);
  const double eps_m2=four_over_3*(dx[0]*dx[0]+dx[1]*dx[1]+dx[0]*dx[1]+fourth*(dx[2]*dx[2]));
  h.equivalent_strain_rate=1.*(::sqrt(eps_k2+eps_m2)*dtinv)+(1.-1.)*h.equivalent_strain_rate;
  double degmb=h.stress[0]*dx[0]+h.stress[1]*dx[1]+h.stress[2]*dx[2]+h.stress[3]*dx[3]+h.stress[4]*dx[4];
  double degfx=h.bending_stress[0]*dx[5]+h.bending_stress[1]*dx[6]+h.bending_stress[2]*dx[7];
  tl::material::ShellElasticLaw1Stress base,next;
  for(unsigned i=0;i<5;++i) base.stress[i]=h.material_stress[i];
  for(unsigned i=0;i<3;++i) base.bending_stress[i]=h.bending_stress[i];
  const double thk08=m.thickness*(one_over_12+0.*0.);
  if(!tl::material::UpdateShellElasticLaw1(m.elastic,m.gs,thk08,dx,base,next)) return false;
  const double ezz=-m.nu*(dx[0]+dx[1])/(1.-m.nu),thkn=h.thickness*(1.+ezz*1.);
  if(!tl::math::Finite(thkn)||!(thkn>=em30)) return false;
  h.thickness=::fmax(thkn,em30);
  for(unsigned i=0;i<5;++i) { h.material_stress[i]=next.stress[i]+0.; h.stress[i]=h.material_stress[i]; }
  for(unsigned i=0;i<3;++i) h.bending_stress[i]=next.bending_stress[i];
  const double visc=onep414*m.dm*m.rho*m.elastic.sound_speed*::sqrt(g.area)*dtinv;
  h.stress[0]=h.stress[0]+visc*(dx[0]+.5*dx[1]);
  h.stress[1]=h.stress[1]+visc*(dx[1]+.5*dx[0]);
  h.stress[2]=h.stress[2]+visc*dx[2]*third;
  for(double& value:h.stress) value=value*1.;
  for(double& value:h.bending_stress) value=value*1.;
  degmb=degmb+h.stress[0]*dx[0]+h.stress[1]*dx[1]+h.stress[2]*dx[2]+h.stress[3]*dx[3]+h.stress[4]*dx[4];
  degfx=degfx+h.bending_stress[0]*dx[5]+h.bending_stress[1]*dx[6]+h.bending_stress[2]*dx[7];
  const double vol2=.5*m.thickness*g.area*1.;
  h.internal_work[0]=h.internal_work[0]+degmb*vol2;
  h.internal_work[1]=h.internal_work[1]+degfx*m.thickness*vol2;
  return tl::math::Finite(fac1)&&tl::math::Finite(dtinv)&&tl::math::Finite(visc);
}
} // namespace tl::fea::t3::detail
