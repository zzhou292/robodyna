// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid18Types.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline double Component(const Vec3& a, unsigned i) noexcept {
  return i == 0 ? a.x : i == 1 ? a.y : a.z;
}
TL_SOLID18_HD inline void SetComponent(Vec3& a, unsigned i, double v) noexcept {
  if (i == 0) a.x = v;
  else if (i == 1) a.y = v;
  else a.z = v;
}
TL_SOLID18_HD inline bool Finite(const Vec3& a) noexcept {
  return tl::math::Finite(a.x) && tl::math::Finite(a.y) && tl::math::Finite(a.z);
}
TL_SOLID18_HD inline bool Positive(double a) noexcept {
  return tl::math::Finite(a) && a > 0;
}
TL_SOLID18_HD inline double Dot(const Vec3& a, const Vec3& b) noexcept {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}
TL_SOLID18_HD inline Vec3 Cross(const Vec3& a, const Vec3& b) noexcept {
  return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
TL_SOLID18_HD inline Vec3 Add(const Vec3& a, const Vec3& b) noexcept {
  return {a.x+b.x, a.y+b.y, a.z+b.z};
}

// CHECKVOLUME_8N and SREPISO3 use the same isoparametric directions.
TL_SOLID18_HD inline void Directions(const Vec3 (&x)[8], Vec3& r, Vec3& s, Vec3& t) noexcept {
  for (unsigned k = 0; k < 3; ++k) {
    const double a = Component(x[6], k)-Component(x[0], k);
    const double b = Component(x[7], k)-Component(x[1], k);
    const double c = Component(x[4], k)-Component(x[2], k);
    const double d = Component(x[5], k)-Component(x[3], k);
    const double first = a+d;
    const double second = b+c;
    SetComponent(r, k, a+b-c-d);
    SetComponent(s, k, first+second);
    SetComponent(t, k, first-second);
  }
}

TL_SOLID18_HD inline double SignedCenterVolume(const Vec3 (&x)[8]) noexcept {
  Vec3 r, s, t;
  Directions(x, r, s, t);
  return (1.0/64.0) * Dot(r, Cross(s, t));
}

TL_SOLID18_HD inline bool Normalize(Vec3& a) noexcept {
  double factor = ::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
  if (!Positive(factor)) return false;
  factor = 1.0/factor;
  a.x = a.x*factor;
  a.y = a.y*factor;
  a.z = a.z*factor;
  return Finite(a);
}

// Complete selected SORTHO3 sequence: three simultaneous direction updates,
// normalizations, then its final cross-product orthogonalization.
TL_SOLID18_HD inline bool Frame(const Vec3 (&x)[8], Matrix3& frame) noexcept {
  Vec3 u, v, w;
  Directions(x, u, v, w);
  if (!Normalize(u) || !Normalize(v) || !Normalize(w)) return false;
  for (unsigned n = 0; n < 3; ++n) {
    Vec3 a = Add(Cross(v, w), u);
    Vec3 b = Add(Cross(w, u), v);
    Vec3 c = Add(Cross(u, v), w);
    if (!Normalize(a) || !Normalize(b) || !Normalize(c)) return false;
    u = a;
    v = b;
    w = c;
  }
  w = Cross(u, v);
  if (!Normalize(w)) return false;
  v = Cross(w, u);
  if (!Finite(v)) return false;
  for (unsigned k = 0; k < 3; ++k) {
    frame.v[3*k] = Component(u, k);
    frame.v[3*k+1] = Component(v, k);
    frame.v[3*k+2] = Component(w, k);
  }
  return true;
}

TL_SOLID18_HD inline Vec3 Local(const Matrix3& f, const Vec3& x) noexcept {
  return {f.v[0]*x.x+f.v[3]*x.y+f.v[6]*x.z,
          f.v[1]*x.x+f.v[4]*x.y+f.v[7]*x.z,
          f.v[2]*x.x+f.v[5]*x.y+f.v[8]*x.z};
}
}  // namespace tl::fea::solid18::detail
