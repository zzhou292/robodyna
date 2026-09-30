// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss a62b27e6 NORMA1D/NORMA4N, Copyright (C) 2026 Siemens.
// See ../LICENSE.md. Original double algebra, in native length units.
#pragma once
#include "../FrictionTypes.h"
#include "lib_src/math/Fixed3Operations.h"
#include "lib_src/math/HostDevice.h"
#include <cmath>
namespace tlfea::contact::radioss_type25::normal_math {
struct DoubleFaceResult {
  Vector normal;
  double area = 0;
};
TL_MATH_HOST_DEVICE inline bool DoubleFace(const Vector (&x)[4], DoubleFaceResult& out) noexcept {
  const auto a = x[0], b = x[1], c = x[2], d = x[3];
  const Vector u{c.x-a.x, c.y-a.y, c.z-a.z};
  const Vector v{d.x-b.x, d.y-b.y, d.z-b.z};
  const Vector n{u.y*v.z-u.z*v.y, u.z*v.x-u.x*v.z, u.x*v.y-u.y*v.x};
  const double raw = ::sqrt(n.x*n.x+n.y*n.y+n.z*n.z);
  if (!tl::math::fixed3::Finite(u) || !tl::math::fixed3::Finite(v) ||
      !tl::math::fixed3::Finite(n) || !tl::math::Finite(raw)) return false;
  // EM20 is an area floor in the declared native working coordinates.
  constexpr double floor = 1./1.e20;
  const double denominator = raw > floor ? raw : floor;
  const DoubleFaceResult next{{n.x/denominator, n.y/denominator, n.z/denominator}, .5*denominator};
  if (!tl::math::fixed3::Finite(next.normal)) return false;
  out = next;
  return true;
}
} // namespace tlfea::contact::radioss_type25::normal_math
