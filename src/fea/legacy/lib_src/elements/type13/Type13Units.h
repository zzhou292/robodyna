// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type13Types.h"
#include "../../math/Fixed3Operations.h"

namespace tl::fea::type13::detail {
TL_TYPE13_HD inline bool Positive(double v) { return tl::math::fixed3::Finite(v)&&v>0; }
TL_TYPE13_HD inline bool Nonnegative(double v) { return tl::math::fixed3::Finite(v)&&v>=0; }
struct UnitFactors { double mass=0,inertia=0,force=0,rotation_stiffness=0; };
TL_TYPE13_HD inline bool ResolveUnits(WorkingUnits u,UnitFactors& out) {
  if(!Positive(u.mass_to_kg)||!Positive(u.length_to_m)||!Positive(u.time_to_s))return false;
  const double t2=u.time_to_s*u.time_to_s;
  UnitFactors next;
  next.mass=u.mass_to_kg;
  next.inertia=u.mass_to_kg*u.length_to_m*u.length_to_m;
  next.force=u.mass_to_kg*u.length_to_m/t2;
  next.rotation_stiffness=next.force*u.length_to_m*u.length_to_m;
  if(!Positive(t2)||!Positive(next.inertia)||!Positive(next.force)||!Positive(next.rotation_stiffness))return false;
  out=next;return true;
}
} // namespace tl::fea::type13::detail
