// SPDX-License-Identifier: AGPL-3.0-or-later
// SVALUE0 / SMASS3: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18Orientation.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline Status PrepareMass(double density, const StartupGeometry& g,
    const std::uint8_t (&native_to_source)[8], Mass& mass) noexcept {
  double global_density = 0;
  for (unsigned r = 0; r < 2; ++r) {
    for (unsigned s = 0; s < 2; ++s) {
      for (unsigned t = 0; t < 2; ++t) {
        const double factor = 1.0*g.point[r+2*s+4*t].initial_volume_m3/g.center_volume_m3;
        global_density = global_density+factor*density;
      }
    }
  }
  if (!Positive(global_density)) return Status::NonfiniteResult;
  mass.initial_global_density_kg_m3 = global_density;
  const double nodal_mass = 1.0*global_density*g.center_volume_m3*.125;
  mass.element_mass_kg = 8*nodal_mass;
  if (!Positive(mass.element_mass_kg)) return Status::NonfiniteResult;
  for (unsigned n = 0; n < 8; ++n) {
    mass.source_nodal_mass_kg[native_to_source[n]] = nodal_mass;
  }
  return Status::Success;
}
}  // namespace tl::fea::solid18::detail
