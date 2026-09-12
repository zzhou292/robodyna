// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected PDLEN3 TYPE18, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "ForceTypes.h"

namespace tl::fea::beam18::force_detail {
// Nodal stiffness plus unscaled element monitor. No timestep adjustment,
// element deletion, mass scaling or owner admission occurs in this value API.
TL_BEAM18_HD inline bool Stiffness(const Reference& reference, const Section& section,
    const Material& material, double length, ForceDiagnostics& out) noexcept {
  const double length_scale = reference.input().units == WorkingUnits::TonneMillimetreSecond ? .001 : 1.;
  const double inertia_floor = 1e-30*length_scale*length_scale*length_scale*length_scale;
  const double bb = ::fmax(::fmax(section.inertia_y,section.inertia_z),inertia_floor);
  const double sl2i = section.area*(length*length)/bb;
  const double kphi = ::fmax(1.,sl2i);
  const double damping = ::fmax(section.membrane_damping,section.flexural_damping)*::sqrt(2.);
  const double factor = ::sqrt(1.+damping*damping)-damping;
  const double aa = length*factor*factor;
  const double bending = ::fmax(section.inertia_y,section.inertia_z);
  out.rotation_stiffness_nm = ::fmax(material.shear_pa*section.inertia_x,
      kphi*material.material.young_pa*bending)/aa;
  out.translation_stiffness_n_m = 1.*section.area*material.material.young_pa/aa;
  const double plane_modulus = material.material.young_pa/
      (1.-material.material.poisson_ratio*material.material.poisson_ratio);
  const double sound_speed = ::sqrt(plane_modulus/material.material.density_kg_m3);
  out.minimum_unscaled_dt_s = 1.*factor*length/sound_speed/::sqrt(1.);
  return detail::Positive(sl2i) && detail::Positive(aa) &&
      detail::Positive(out.rotation_stiffness_nm) && detail::Positive(out.translation_stiffness_n_m) &&
      detail::Positive(out.minimum_unscaled_dt_s);
}
} // namespace tl::fea::beam18::force_detail
