// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "BrickFrame.h"

namespace tl::fea::solid_common {
TL_BRICK_HD inline bool CyclicFrame(const Vec3 (&x)[8], Matrix3& result) noexcept {
  Matrix3 ordinary;
  if (!Frame(x, ordinary)) return false;
  // SRCOOR3 passes E2,E3,E1 as SORTHO3's first, second, third outputs.
  for (unsigned k = 0; k < 3; ++k) {
    result.v[3*k] = ordinary.v[3*k+2];
    result.v[3*k+1] = ordinary.v[3*k];
    result.v[3*k+2] = ordinary.v[3*k+1];
  }
  return true;
}
TL_BRICK_HD inline double FaceMeasure(const Vec3& a, const Vec3& b,
                                     const Vec3& c, const Vec3& d) noexcept {
  const Vec3 x13{c.x-a.x,c.y-a.y,c.z-a.z}, x24{d.x-b.x,d.y-b.y,d.z-b.z};
  const Vec3 s{x13.x-x24.x,x13.y-x24.y,x13.z-x24.z};
  const Vec3 t{x13.x+x24.x,x13.y+x24.y,x13.z+x24.z};
  const double e = Dot(s,s), f = Dot(s,t), g = Dot(t,t);
  return e*g-f*f;
}
}  // namespace tl::fea::solid_common
