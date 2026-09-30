// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../../math/Fixed3Operations.h"

namespace tl::fea::beam18::detail {
TL_BEAM18_HD inline bool Positive(double value) noexcept {
  return tl::math::fixed3::Finite(value) && value > 0;
}
struct UnitFactors { double length = 0, mass = 0, inertia = 0, stiffness = 0; };
TL_BEAM18_HD inline bool Units(WorkingUnits units, UnitFactors& output) noexcept {
  if (units == WorkingUnits::SI) output = {1,1,1,1};
  else if (units == WorkingUnits::TonneMillimetreSecond) {
    output = {.001,1000,1000*.001*.001,1000};
  } else return false;
  return true;
}
TL_BEAM18_HD inline bool SamePosition(Vec3 a, Vec3 b) noexcept {
  // Source identity includes signed zero coordinates.
  const double av[]{a.x,a.y,a.z}, bv[]{b.x,b.y,b.z};
  for (unsigned i = 0; i < 3; ++i)
    if (av[i] != bv[i] || (av[i] == 0 && ::copysign(1.0,av[i]) != ::copysign(1.0,bv[i]))) return false;
  return true;
}
} // namespace tl::fea::beam18::detail
