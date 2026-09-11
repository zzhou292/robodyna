// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid18Basis.h"
#include "Solid18Orientation.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline Status PointGeometry(const Vec3 (&x)[8], const Basis& b,
                                         StartupPoint& p) noexcept {
  double j[3][3]{};
  for (unsigned k = 0; k < 3; ++k) {
    double a[8];
    for (unsigned n = 0; n < 8; ++n) a[n] = Component(x[n], k);
    j[0][k] = b.r[0]*(a[0]-a[1])+b.r[2]*(a[2]-a[3])+
              b.r[4]*(a[4]-a[5])+b.r[6]*(a[6]-a[7]);
    j[1][k] = b.s[0]*(a[0]-a[3])+b.s[1]*(a[1]-a[2])+
              b.s[4]*(a[4]-a[7])+b.s[5]*(a[5]-a[6]);
    j[2][k] = b.t[0]*(a[0]-a[4])+b.t[1]*(a[1]-a[5])+
              b.t[2]*(a[2]-a[6])+b.t[3]*(a[3]-a[7]);
  }
  double inv[3][3]{};
  inv[0][0] = j[1][1]*j[2][2]-j[1][2]*j[2][1];
  inv[1][0] = -j[1][0]*j[2][2]+j[1][2]*j[2][0];
  inv[2][0] = j[1][0]*j[2][1]-j[1][1]*j[2][0];
  const double volume = j[0][0]*inv[0][0]+j[0][1]*inv[1][0]+j[0][2]*inv[2][0];
  if (!tl::math::Finite(volume)) return Status::NonfiniteResult;
  if (volume <= 0) return Status::InvalidGeometry;
  const double reciprocal = 1/volume;
  inv[0][0] = reciprocal*inv[0][0];
  inv[1][0] = reciprocal*inv[1][0];
  inv[2][0] = reciprocal*inv[2][0];
  inv[0][1] = reciprocal*(-j[0][1]*j[2][2]+j[0][2]*j[2][1]);
  inv[1][1] = reciprocal*( j[0][0]*j[2][2]-j[0][2]*j[2][0]);
  inv[2][1] = reciprocal*(-j[0][0]*j[2][1]+j[0][1]*j[2][0]);
  inv[0][2] = reciprocal*( j[0][1]*j[1][2]-j[0][2]*j[1][1]);
  inv[1][2] = reciprocal*(-j[0][0]*j[1][2]+j[0][2]*j[1][0]);
  inv[2][2] = reciprocal*( j[0][0]*j[1][1]-j[0][1]*j[1][0]);
  p.jacobian_volume_m3 = volume;
  for (unsigned k = 0; k < 3; ++k) {
    const double a1 = inv[k][0]*b.r[0], a3 = inv[k][0]*b.r[2];
    const double a5 = inv[k][0]*b.r[4], a7 = inv[k][0]*b.r[6];
    const double b1 = inv[k][1]*b.s[0], b2 = inv[k][1]*b.s[1];
    const double b5 = inv[k][1]*b.s[4], b6 = inv[k][1]*b.s[5];
    const double c1 = inv[k][2]*b.t[0], c2 = inv[k][2]*b.t[1];
    const double c3 = inv[k][2]*b.t[2], c4 = inv[k][2]*b.t[3];
    const double derivative[8] = {a1+b1+c1, -a1+b2+c2, a3-b2+c3, -a3-b1+c4,
                                 a5+b5-c1, -a5+b6-c2, a7-b6-c3, -a7-b5-c4};
    for (unsigned n = 0; n < 8; ++n) {
      if (!tl::math::Finite(derivative[n])) return Status::NonfiniteResult;
      SetComponent(p.derivative_per_m[n], k, derivative[n]);
    }
  }
  for (unsigned n = 0; n < 8; ++n) p.shape[n] = b.h[n];
  return Status::Success;
}

TL_SOLID18_HD inline double CrossSquare(const Vec3& a, const Vec3& b) noexcept {
  const Vec3 c = Cross(a, b);
  return c.x*c.x + c.y*c.y + c.z*c.z;
}

TL_SOLID18_HD inline Status IntegrateGeometry(StartupGeometry& g) noexcept {
  double minimum = 0;
  for (unsigned point = 0; point < 8; ++point) {
    const Status status = PointGeometry(g.native_position_m, StartupBasis(point), g.point[point]);
    if (status != Status::Success) return status;
    const auto& p = g.point[point];
    const double volume = p.jacobian_volume_m3;
    g.volume_m3 = g.volume_m3+volume;
    minimum = point == 0 ? volume : ::fmin(minimum, volume);
    for (unsigned n = 0; n < 8; ++n) {
      auto& average = g.native_average_derivative_per_m[n];
      average.x = p.derivative_per_m[n].x*volume+average.x;
      average.y = p.derivative_per_m[n].y*volume+average.y;
      average.z = p.derivative_per_m[n].z*volume+average.z;
      g.native_nodal_volume_m3[n] = g.native_nodal_volume_m3[n]+volume*p.shape[n];
    }
  }
  Vec3 a, b, c;
  for (unsigned k = 0; k < 3; ++k) {
    double x[8];
    for (unsigned n = 0; n < 8; ++n) x[n] = Component(g.native_position_m[n], k);
    SetComponent(a, k, x[0]+x[1]+x[2]+x[3]-x[4]-x[5]-x[6]-x[7]);
    SetComponent(b, k, x[0]+x[1]+x[4]+x[5]-x[2]-x[3]-x[6]-x[7]);
    SetComponent(c, k, x[0]+x[3]+x[4]+x[7]-x[2]-x[1]-x[6]-x[5]);
  }
  double area_square = CrossSquare(a, b);
  area_square = ::fmax(area_square, CrossSquare(a, c));
  area_square = ::fmax(area_square, CrossSquare(c, b));
  if (!Positive(area_square) || !Positive(g.volume_m3)) return Status::NonfiniteResult;
  g.characteristic_length_m = 128*minimum/::sqrt(area_square);
  if (!Positive(g.characteristic_length_m)) return Status::NonfiniteResult;
  for (unsigned n = 0; n < 8; ++n) {
    auto& average = g.native_average_derivative_per_m[n];
    average.x = average.x/g.volume_m3;
    average.y = average.y/g.volume_m3;
    average.z = average.z/g.volume_m3;
    if (!Finite(average) || !Positive(g.native_nodal_volume_m3[n]))
      return Status::NonfiniteResult;
  }
  return Status::Success;
}
}  // namespace tl::fea::solid18::detail
