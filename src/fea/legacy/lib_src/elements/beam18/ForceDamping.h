// SPDX-License-Identifier: AGPL-3.0-or-later
// PDAMP3, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "ForceTypes.h"

namespace tl::fea::beam18::force_detail {
TL_BEAM18_HD inline bool Damping(const Section& section, const Material& material,
    const HistoryValues& state, const GeneralizedRate& rate, double dt, double length,
    ForceDiagnostics& out) noexcept {
  const double rhoe = ::sqrt(2.*material.material.young_pa*material.material.density_kg_m3);
  const double rhog = ::sqrt(2.*material.shear_pa*material.material.density_kg_m3);
  const double volume = section.area*length;
  const double dt_inverse = dt/::fmax(dt*dt,1e-20);
  const double membrane = dt_inverse*section.membrane_damping*1.;
  const double flexure = dt_inverse*section.flexural_damping*1.;
  const double dmm = volume*rhoe;
  auto& f = out.damped_section_force_n; auto& m = out.damped_section_moment_nm;
  f.x = state.section_force_n.x+membrane*(rate.axial*dt)*dmm;
  f.y = state.section_force_n.y+flexure*(rate.shear_y*dt)*dmm;
  f.z = state.section_force_n.z+flexure*(rate.shear_z*dt)*dmm;
  m.x = state.section_moment_nm.x+flexure*(rate.curvature_x*dt)*length*section.inertia_x*rhog;
  m.y = state.section_moment_nm.y+flexure*(rate.curvature_y*dt)*length*section.inertia_y*rhoe;
  m.z = state.section_moment_nm.z+flexure*(rate.curvature_z*dt)*length*section.inertia_z*rhoe;
  return tl::math::fixed3::Finite(f) && tl::math::fixed3::Finite(m) &&
      tl::math::Finite(rhoe) && tl::math::Finite(rhog) && tl::math::Finite(dt_inverse);
}
} // namespace tl::fea::beam18::force_detail
