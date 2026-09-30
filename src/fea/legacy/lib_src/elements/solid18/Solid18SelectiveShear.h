// SPDX-License-Identifier: AGPL-3.0-or-later
// S8ESELECSH/S8EDERISH2/S8EDERI_BIJ: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid18ForceTypes.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline double PlaneAverage(const CurrentGeometry& geometry,
    unsigned axis, unsigned node, const unsigned (&point)[4]) noexcept {
  return .25*(geometry.point[point[0]].regular_per_m[axis][node]+
              geometry.point[point[1]].regular_per_m[axis][node]+
              geometry.point[point[2]].regular_per_m[axis][node]+
              geometry.point[point[3]].regular_per_m[axis][node]);
}

TL_SOLID18_HD inline void SelectedPointShear(CurrentGeometry& geometry,
                                             unsigned ip) noexcept {
  constexpr unsigned xy[2][4] = {{0,4,2,6},{1,5,3,7}};
  constexpr unsigned xz[2][4] = {{0,2,1,3},{4,6,5,7}};
  constexpr unsigned yz[2][4] = {{0,4,1,5},{2,6,3,7}};
  auto& p = geometry.point[ip].shear_per_m;
  for (unsigned n = 0; n < 8; ++n) {
    p[0][n] = PlaneAverage(geometry, 1, n, xy[ip&1]);
    p[1][n] = PlaneAverage(geometry, 0, n, xy[ip&1]);
    p[2][n] = PlaneAverage(geometry, 2, n, xz[(ip>>2)&1]);
    p[3][n] = PlaneAverage(geometry, 0, n, xz[(ip>>2)&1]);
    p[4][n] = PlaneAverage(geometry, 2, n, yz[(ip>>1)&1]);
    p[5][n] = PlaneAverage(geometry, 1, n, yz[(ip>>1)&1]);
  }
}

TL_SOLID18_HD inline void SelectedPointCross(double nu, double nu1,
    CurrentGeometry& geometry, unsigned ip) noexcept {
  auto& point = geometry.point[ip];
  const auto& p = point.shear_per_m;
  auto& b = point.cross_per_m;
  for (unsigned n = 0; n < 8; ++n) {
    constexpr unsigned opposite[4] = {2,3,0,1};
    const unsigned center = n < 4 ? n : opposite[n-4];
    const double cx = geometry.center_gradient_per_m[0][center];
    const double cy = geometry.center_gradient_per_m[1][center];
    const double cz = geometry.center_gradient_per_m[2][center];
    // Keep native '+' for the four opposite-center slots; do not pre-negate.
    const double x1 = n < 4 ? p[3][n]-cx : p[3][n]+cx;
    const double x3 = n < 4 ? p[1][n]-cx : p[1][n]+cx;
    const double y1 = n < 4 ? p[5][n]-cy : p[5][n]+cy;
    const double y2 = n < 4 ? p[0][n]-cy : p[0][n]+cy;
    const double z2 = n < 4 ? p[2][n]-cz : p[2][n]+cz;
    const double z3 = n < 4 ? p[4][n]-cz : p[4][n]+cz;
    const double px = point.regular_per_m[0][n];
    const double py = point.regular_per_m[1][n];
    const double pz = point.regular_per_m[2][n];
    const double x24 = nu*(n < 4 ? px-cx-x1-x3 : px+cx-x1-x3);
    const double y34 = nu*(n < 4 ? py-cy-y1-y2 : py+cy-y1-y2);
    const double z14 = nu*(n < 4 ? pz-cz-z2-z3 : pz+cz-z2-z3);
    b[0][n] = -nu1*x1-x24;
    b[2][n] = -nu1*x3-x24;
    b[1][n] = -nu1*y1-y34;
    b[4][n] = -nu1*y2-y34;
    b[3][n] = -nu1*z3-z14;
    b[5][n] = -nu1*z2-z14;
  }
}

TL_SOLID18_HD inline void SelectedShearDerivatives(CurrentGeometry& geometry) noexcept {
  for (unsigned ip = 0; ip < 8; ++ip) SelectedPointShear(geometry, ip);
}

TL_SOLID18_HD inline void SelectiveDerivatives(double nu,
                                              CurrentGeometry& geometry) noexcept {
  const double nu1 = nu/(1-nu);
  for (unsigned ip = 0; ip < 8; ++ip) {
    SelectedPointShear(geometry, ip);
    SelectedPointCross(nu, nu1, geometry, ip);
  }
}
}  // namespace tl::fea::solid18::detail
