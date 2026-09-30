// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss (C) 2026 Siemens; CZSTRA3, SIGEPS01G and selected MULAWGLC.
// Fixed centered LAW1/ISMSTR-1; retains total/material stress distinction.
#pragma once
#include "QephHistoryData.h"
#include "QephMaterial.h"

namespace tl::fea::qeph::detail {
TL_QEPH_HD inline bool UpdateLaw1(const GeometryWork& g,const MaterialWork& m,
                                HistoryValues& h) {
  using namespace force_constant;
  double dx[8]{};
  for(unsigned i=0;i<8;++i) dx[i]=g.values.regular_rate[i]*m.dt;
  // CZSTRA3 actual dummy aliases put material YZ before ZX.
  const double xz=dx[3]; dx[3]=dx[4]; dx[4]=xz;
  for(unsigned i=0;i<8;++i) h.strain_curvature[i]=h.strain_curvature[i]+dx[i];
  double degmb=h.stress[0]*dx[0]+h.stress[1]*dx[1]+h.stress[2]*dx[2]+
      h.stress[3]*dx[3]+h.stress[4]*dx[4];
  double degfx=h.bending_stress[0]*dx[5]+h.bending_stress[1]*dx[6]+
      h.bending_stress[2]*dx[7];
  const double dtinv=m.dt/::fmax(m.dt*m.dt,em20);
  const double thk08=m.thickness*(one_over_12+0.*0.);
  tl::material::ShellElasticLaw1Stress old_stress,new_stress;
  for(unsigned i=0;i<5;++i) old_stress.stress[i]=h.material_stress[i];
  for(unsigned i=0;i<3;++i) old_stress.bending_stress[i]=h.bending_stress[i];
  if(!tl::material::UpdateShellElasticLaw1(m.elastic,m.gs,thk08,dx,old_stress,new_stress)) return false;
  for(unsigned i=0;i<5;++i) h.material_stress[i]=new_stress.stress[i];
  for(unsigned i=0;i<3;++i) h.bending_stress[i]=new_stress.bending_stress[i];
  const double ezz=-m.nu*(dx[0]+dx[1])/(1.-m.nu);
  const double thkn=h.thickness*(1.+ezz*1.);
  // Retained native wrapper's explicit supported-domain rejection before MAX.
  if(!(thkn>=em30)||!tl::math::Finite(thkn)) return false;
  h.thickness=::fmax(thkn,em30);
  for(unsigned i=0;i<5;++i) {
    h.material_stress[i]=h.material_stress[i]+0.;
    h.stress[i]=h.material_stress[i];
  }
  const double visc=onep414*m.dm*m.rho*m.sound_speed*::sqrt(g.values.area)*dtinv;
  h.stress[0]=h.stress[0]+visc*(dx[0]+.5*dx[1]);
  h.stress[1]=h.stress[1]+visc*(dx[1]+.5*dx[0]);
  h.stress[2]=h.stress[2]+visc*dx[2]*(1./3.);
  for(double& value:h.stress) value=value*1.;
  for(double& value:h.bending_stress) value=value*1.;
  degmb=degmb+h.stress[0]*dx[0]+h.stress[1]*dx[1]+h.stress[2]*dx[2]+
      h.stress[3]*dx[3]+h.stress[4]*dx[4];
  degfx=degfx+h.bending_stress[0]*dx[5]+h.bending_stress[1]*dx[6]+h.bending_stress[2]*dx[7];
  const double vol2=.5*m.thickness*g.values.area*1.;
  h.internal_work[0]=h.internal_work[0]+degmb*vol2;
  h.internal_work[1]=h.internal_work[1]+degfx*m.thickness*vol2;
  return true;
}
} // namespace tl::fea::qeph::detail
