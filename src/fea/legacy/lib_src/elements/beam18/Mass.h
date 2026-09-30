// SPDX-License-Identifier: AGPL-3.0-or-later
// Native PMASS selected TYPE18, no thermal or initial velocity contribution.
#pragma once
#include "Section.h"

namespace tl::fea::beam18::detail {
TL_BEAM18_HD inline Status PrepareMass(const Input& input, const Section& section,
    const Geometry& geometry, UnitFactors units, NativeMass& native, Endpoint& endpoint) noexcept {
  const double length = geometry.length;
  const double e = input.young;
  // HM_READ_MAT44's actual prepared shear modulus operation order.
  const double g = e/2/(1+input.poisson);
  const double rho = input.density;
  const double area = section.area;
  const double iy = section.inertia_y, iz = section.inertia_z, ix = section.inertia_x;
  NativeMass next{};
  const double cst = (6.0/5.0)*e/::fmax(g,(1.0/(1e20*1e10)));
  const double bb = ::fmax(::fmax(iy,iz),(1.0/(1e20*1e10)));
  const double sl2i = area*(length*length)/bb;
  next.facdt = (1.0/12.0)*sl2i;
  const double phmax = cst/next.facdt;
  const double phmin = ::fmin(iy,iz)*phmax/bb;
  next.kphi = (4+phmin)/(1+phmin);
  next.phii = ::fmax(1.0,next.kphi/(1+next.facdt));
  next.axial_coefficient = 1.0/12.0;
  // TYPE18 overwrites KPHI and FSH, but retains the earlier PHII.
  next.kphi = ::fmax(1.0,12*next.facdt);
  if (next.kphi > 12) next.axial_coefficient = 1;
  const double onep2 = 1+2.0/10.0;
  const double ll = onep2*length;
  next.endpoint_mass = rho*length*area*.5;
  next.axial_inertia_term = onep2*next.endpoint_mass*(ll*ll)*next.axial_coefficient;
  next.section_inertia_term = rho*(length*.5)*::fmax(iy,iz);
  next.endpoint_total_inertia = next.axial_inertia_term+next.section_inertia_term;
  if (next.facdt < 1) next.endpoint_total_inertia = next.phii*next.endpoint_total_inertia;
  next.torsional_floor = rho*length/2*ix;
  next.endpoint_total_inertia = ::fmax(next.endpoint_total_inertia,next.torsional_floor);
  next.interface_stiffness = e*area/length;
  double dmp = ::fmax(section.membrane_damping,section.flexural_damping);
  dmp = dmp*::sqrt(2.0);
  double aa = ::sqrt(1+dmp*dmp)-dmp;
  aa = length*aa*aa;
  next.rotation_stiffness = ::fmax(g*ix,next.kphi*e*::fmax(iy,iz))/aa;
  next.translation_stiffness = 1.0*area*e/aa;
  const double values[]{g,cst,bb,sl2i,phmax,phmin,next.facdt,next.phii,next.kphi,
      next.axial_coefficient,next.endpoint_mass,next.axial_inertia_term,next.section_inertia_term,
      next.torsional_floor,next.endpoint_total_inertia,next.interface_stiffness,aa,
      next.translation_stiffness,next.rotation_stiffness};
  for (double value : values) if (!Positive(value)) return Status::NonfiniteResult;
  Endpoint result{next.endpoint_mass*units.mass,next.endpoint_total_inertia*units.inertia,
      next.translation_stiffness*units.stiffness,next.rotation_stiffness*units.inertia,
      next.interface_stiffness*units.stiffness};
  if (!Positive(result.mass_kg) || !Positive(result.native_total_inertia_kg_m2) ||
      !Positive(result.translation_stiffness_n_m) || !Positive(result.rotation_stiffness_nm) ||
      !Positive(result.interface_stiffness_n_m)) return Status::NonfiniteResult;
  native = next;
  endpoint = result;
  return Status::Success;
}
} // namespace tl::fea::beam18::detail
