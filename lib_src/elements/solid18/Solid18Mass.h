// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid18Orientation.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline Status PrepareMass(double density, const StartupGeometry& g,
    const std::uint8_t (&native_to_source)[8], Mass& mass) noexcept {
  // SMASS3B retains its total/8 intermediate for the separate element ledger.
  const double eighth_mass = density*g.volume_m3*.125;
  mass.element_mass_kg = 8*eighth_mass;
  if (!Positive(mass.element_mass_kg)) return Status::NonfiniteResult;
  for (unsigned n = 0; n < 8; ++n) {
    const unsigned source = native_to_source[n];
    const double value = density*g.native_nodal_volume_m3[n];
    if (!Positive(value)) return Status::NonfiniteResult;
    mass.source_nodal_mass_kg[source] = value;
    mass.source_nodal_volume_m3[source] = g.native_nodal_volume_m3[n];
  }
  return Status::Success;
}
}  // namespace tl::fea::solid18::detail
