// SPDX-License-Identifier: AGPL-3.0-or-later
// CHECKVOLUME_6N / S6ZJACIDP / S6ZDERI3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid6zTypes.h"

namespace tl::fea::solid6z::detail {
namespace brick = tl::fea::solid_common;
struct Jacobian {
  double j[9]{}, cross[3]{}, volume = 0;
};
TL_BRICK_HD inline Jacobian EvaluateJacobian(const Vec3 (&x)[6]) noexcept {
  Jacobian a;
  for (unsigned k = 0; k < 3; ++k) {
    const double x21 = brick::Component(x[1],k)-brick::Component(x[0],k);
    const double x31 = brick::Component(x[2],k)-brick::Component(x[0],k);
    const double x41 = brick::Component(x[3],k)-brick::Component(x[0],k);
    const double x54 = brick::Component(x[4],k)-brick::Component(x[3],k);
    const double x64 = brick::Component(x[5],k)-brick::Component(x[3],k);
    a.j[k] = x21+x54;
    a.j[3+k] = x31+x64;
    a.j[6+k] = (1.0/3.0)*(x41+brick::Component(x[4],k)-brick::Component(x[1],k)
                                           +brick::Component(x[5],k)-brick::Component(x[2],k));
  }
  const auto& j = a.j;
  a.cross[0] = j[4]*j[8]-j[5]*j[7];
  a.cross[1] = j[5]*j[6]-j[3]*j[8];
  a.cross[2] = j[3]*j[7]-j[4]*j[6];
  a.volume = (1.0/8.0)*(j[0]*a.cross[0]+j[1]*a.cross[1]+j[2]*a.cross[2]);
  return a;
}
TL_BRICK_HD inline bool Finite(const Jacobian& a) noexcept {
  for (double v : a.j) if (!tl::math::Finite(v)) return false;
  for (double v : a.cross) if (!tl::math::Finite(v)) return false;
  return tl::math::Finite(a.volume);
}
TL_BRICK_HD inline bool ReferenceInverse(const Jacobian& a, StartupGeometry& g) noexcept {
  const auto& j = a.j;
  const double factor = (1.0/8.0)/a.volume;
  auto& r = g.inverse_reference_jacobian;
  r[0] = factor*a.cross[0];
  r[3] = factor*a.cross[1];
  r[6] = factor*a.cross[2];
  r[1] = factor*(-j[1]*j[8]+j[2]*j[7]);
  r[4] = factor*( j[0]*j[8]-j[2]*j[6]);
  r[7] = factor*(-j[0]*j[7]+j[1]*j[6]);
  r[2] = factor*( j[1]*j[5]-j[2]*j[4]);
  r[5] = factor*(-j[0]*j[5]+j[2]*j[3]);
  r[8] = factor*( j[0]*j[4]-j[1]*j[3]);
  for (double v : r) if (!tl::math::Finite(v)) return false;
  g.reference_volume_m3 = a.volume;
  return true;
}
}  // namespace tl::fea::solid6z::detail
