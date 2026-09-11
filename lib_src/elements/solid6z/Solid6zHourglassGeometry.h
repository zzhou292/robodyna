// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected S6ZHOUR3 geometry, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid6zForceTypes.h"

namespace tl::fea::solid6z::force_detail {
struct HourglassGeometry {
  Vec3 velocity_m_s[8]{};
  double projection[4][4]{}; // Opposite node pair, then native mode.
  double diagonal_m[3]{};
};
TL_BRICK_HD inline bool StabilizationGeometry(const CurrentGeometry& current,
                                               HourglassGeometry& output) noexcept {
  namespace common = tl::fea::solid_common;
  const auto& q = current.local_position_m;
  const Vec3 x[8]{q[0],q[1],q[2],q[2],q[3],q[4],q[5],q[5]};
  double j[9];
  for (unsigned axis = 0; axis < 3; ++axis) {
    const double a = common::Component(x[6],axis)-common::Component(x[0],axis);
    const double b = common::Component(x[7],axis)-common::Component(x[1],axis);
    const double c = common::Component(x[4],axis)-common::Component(x[2],axis);
    const double d = common::Component(x[5],axis)-common::Component(x[3],axis);
    j[3+axis] = a+b-c-d;
    const double first = a+d;
    const double second = b+c;
    j[6+axis] = first+second;
    j[axis] = first-second;
  }
  const double cross0 = j[4]*j[8]-j[5]*j[7];
  const double cross1 = j[5]*j[6]-j[3]*j[8];
  const double cross2 = j[3]*j[7]-j[4]*j[6];
  const double volume = (1.0/64.0)*(j[0]*cross0+j[1]*cross1+j[2]*cross2);
  if (!common::Positive(volume)) return false;
  const double factor = (1.0/64.0)/volume;
  double inverse[9];
  inverse[0] = factor*cross0;
  inverse[3] = factor*cross1;
  inverse[6] = factor*cross2;
  inverse[1] = factor*(-j[1]*j[8]+j[2]*j[7]);
  inverse[4] = factor*( j[0]*j[8]-j[2]*j[6]);
  inverse[7] = factor*(-j[0]*j[7]+j[1]*j[6]);
  inverse[2] = factor*( j[1]*j[5]-j[2]*j[4]);
  inverse[5] = factor*(-j[0]*j[5]+j[2]*j[3]);
  inverse[8] = factor*( j[0]*j[4]-j[1]*j[3]);
  double point[3][4];
  double coordinate[3][4];
  for (unsigned axis = 0; axis < 3; ++axis) {
    const double first = inverse[3*axis];
    const double second = inverse[3*axis+1];
    const double third = inverse[3*axis+2];
    double combined = first-second;
    point[axis][1] = combined-third;
    point[axis][3] = -combined-third;
    combined = first+second;
    point[axis][0] = -combined-third;
    point[axis][2] = combined-third;
    double a[8];
    for (unsigned n = 0; n < 8; ++n) a[n] = common::Component(x[n],axis);
    coordinate[axis][2] = (1.0/8.0)*(a[0]-a[1]+a[2]-a[3]+a[4]-a[5]+a[6]-a[7]);
    coordinate[axis][0] = (1.0/8.0)*(a[0]+a[1]-a[2]-a[3]-a[4]-a[5]+a[6]+a[7]);
    coordinate[axis][1] = (1.0/8.0)*(a[0]-a[1]-a[2]+a[3]-a[4]+a[5]+a[6]-a[7]);
    coordinate[axis][3] = (1.0/8.0)*(-a[0]+a[1]-a[2]+a[3]+a[4]-a[5]+a[6]-a[7]);
  }
  HourglassGeometry next;
  for (unsigned n = 0; n < 4; ++n) {
    for (unsigned mode = 0; mode < 4; ++mode) {
      next.projection[n][mode] = point[0][n]*coordinate[0][mode]+
          point[1][n]*coordinate[1][mode]+point[2][n]*coordinate[2][mode];
      if (!tl::math::Finite(next.projection[n][mode])) return false;
    }
  }
  for (unsigned axis = 0; axis < 3; ++axis) {
    next.diagonal_m[axis] = j[4*axis];
    if (!tl::math::Finite(next.diagonal_m[axis])) return false;
  }
  constexpr unsigned embedded[8]{0,1,2,2,3,4,5,5};
  for (unsigned n = 0; n < 8; ++n) next.velocity_m_s[n] = current.local_velocity_m_s[embedded[n]];
  output = next;
  return true;
}
} // namespace tl::fea::solid6z::force_detail
