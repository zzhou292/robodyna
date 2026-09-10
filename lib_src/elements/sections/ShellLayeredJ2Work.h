// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected MULAWC generalized work and instantaneous DM; (C) 2026 Siemens.
#pragma once
#include "ShellLayeredJ2.h"

namespace tl::fea::sections {
// Family histories remain unchanged. The caller stages h and the section as one
// trial. No plastic-work diagnostic is added to this total stress-work ledger.
template<class HistoryValues>
TL_SHELL_SECTION_HD inline bool MatchesLayeredJ2Resultants(const ShellLayeredJ2History& section,
    const HistoryValues& h) noexcept {
  double force[5]{},moment[3]{}; LayeredJ2Resultants(section,force,moment);
  for(unsigned i=0;i<5;++i) if(force[i]!=h.material_stress[i]) return false;
  for(unsigned i=0;i<3;++i) if(moment[i]!=h.bending_stress[i]) return false;
  return true;
}
template<class ReferenceInput>
TL_SHELL_SECTION_HD inline bool MatchesLayeredJ2Material(const PointParameters& p,
    const ReferenceInput& r) noexcept {
  return p.young_pa==r.young_modulus&&p.poisson_ratio==r.poisson_ratio&&p.density_kg_m3==r.density;
}
template<class HistoryValues>
TL_SHELL_SECTION_HD inline bool ApplyLayeredJ2Work(const ShellLayeredJ2Result& section,
    const double (&dx)[8],double reference_thickness,double area,double viscosity,
    HistoryValues& h) noexcept {
  for(unsigned i=0;i<8;++i) h.strain_curvature[i]=h.strain_curvature[i]+dx[i];
  double membrane=h.stress[0]*dx[0]+h.stress[1]*dx[1]+h.stress[2]*dx[2]+h.stress[3]*dx[3]+h.stress[4]*dx[4];
  double bending=h.bending_stress[0]*dx[5]+h.bending_stress[1]*dx[6]+h.bending_stress[2]*dx[7];
  h.thickness=section.reported_thickness;
  for(unsigned i=0;i<5;++i) { h.material_stress[i]=section.material_stress[i]; h.stress[i]=h.material_stress[i]; }
  for(unsigned i=0;i<3;++i) h.bending_stress[i]=section.bending_stress[i];
  h.stress[0]=h.stress[0]+viscosity*(dx[0]+.5*dx[1]);
  h.stress[1]=h.stress[1]+viscosity*(dx[1]+.5*dx[0]);
  h.stress[2]=h.stress[2]+viscosity*dx[2]*(1./3.);
  membrane=membrane+h.stress[0]*dx[0]+h.stress[1]*dx[1]+h.stress[2]*dx[2]+h.stress[3]*dx[3]+h.stress[4]*dx[4];
  bending=bending+h.bending_stress[0]*dx[5]+h.bending_stress[1]*dx[6]+h.bending_stress[2]*dx[7];
  const double half_volume=.5*reference_thickness*area*1.;
  h.internal_work[0]=h.internal_work[0]+membrane*half_volume;
  h.internal_work[1]=h.internal_work[1]+bending*reference_thickness*half_volume;
  return tl::math::Finite(viscosity)&&tl::math::Finite(h.internal_work[0])&&tl::math::Finite(h.internal_work[1]);
}
} // namespace tl::fea::sections
