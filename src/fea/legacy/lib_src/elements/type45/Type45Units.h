// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45Types.h"

namespace tl::fea::type45::detail {
// Native constants are dimensioned in the explicitly selected working system.
// Current coordinates and coefficients remain SI; there is no inverse round trip.
struct UnitFloors {
  double axis_length, squared_length, mass20, inertia20, mass15, inertia15;
  double stiffness_warning, stiffness_sentinel;
};
TL_TYPE45_HD inline UnitFloors Floors(WorkingUnits units) {
  const double length = units==WorkingUnits::SI ? 1. : .001;
  const double mass = units==WorkingUnits::SI ? 1. : 1000.;
  const double inertia = mass*length*length;
  return {1e-10*length, 1e-20*length*length, 1e-20*mass,
          1e-20*inertia, 1e-15*mass, 1e-15*inertia,
          double(1e-8f)*mass, 1e30*mass};
}
} // namespace tl::fea::type45::detail
