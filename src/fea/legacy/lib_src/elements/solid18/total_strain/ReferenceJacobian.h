// SPDX-License-Identifier: AGPL-3.0-or-later
// Native S8ZJAC_IC/S8ZJAC_I3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "ReferenceTypes.h"
#include "lib_src/elements/solid18/Solid18CurrentGeometry.h"

namespace tl::fea::solid18::total_strain::detail {
// S8ZJAC_I3 uses products (1 +/- coordinate), unlike current S8EPRST_INI.
TL_SOLID18_HD inline void ReferenceNatural(unsigned ip, double (&p)[3][8]) noexcept {
  constexpr double gauss = .577350269189625;
  const double r = ip&1 ? gauss : -gauss;
  const double s = ip&2 ? gauss : -gauss;
  const double t = ip&4 ? gauss : -gauss;
  const double rp = 1+r, sp = 1+s, tp = 1+t;
  const double rm = 1-r, sm = 1-s, tm = 1-t;
  p[0][0] = -sm*tm;
  p[0][1] = -p[0][0];
  p[0][2] = sp*tm;
  p[0][3] = -p[0][2];
  p[0][4] = -sm*tp;
  p[0][5] = -p[0][4];
  p[0][6] = sp*tp;
  p[0][7] = -p[0][6];
  p[1][0] = -rm*tm;
  p[1][1] = -rp*tm;
  p[1][2] = -p[1][1];
  p[1][3] = -p[1][0];
  p[1][4] = -rm*tp;
  p[1][5] = -rp*tp;
  p[1][6] = -p[1][5];
  p[1][7] = -p[1][4];
  p[2][0] = -rm*sm;
  p[2][1] = -rp*sm;
  p[2][2] = -rp*sp;
  p[2][3] = -rm*sp;
  p[2][4] = -p[2][0];
  p[2][5] = -p[2][1];
  p[2][6] = -p[2][2];
  p[2][7] = -p[2][3];
}

TL_SOLID18_HD inline Status ReferencePointGradient(const StartupGeometry& geometry,
    unsigned ip, double (&gradient)[3][8]) noexcept {
  constexpr double gauss = .577350269189625;
  const double r = ip&1 ? gauss : -gauss;
  const double s = ip&2 ? gauss : -gauss;
  const double t = ip&4 ? gauss : -gauss;
  Matrix3 jacobian;
  const auto& center = geometry.center_scaled_jacobian_m.v;
  const auto& h = geometry.higher_mode_m;
  for (unsigned k = 0; k < 3; ++k) {
    using solid18::detail::Component;
    jacobian.v[k] = center[k]+Component(h[2],k)*s+
        (Component(h[1],k)+Component(h[3],k)*s)*t;
    jacobian.v[3+k] = center[3+k]+Component(h[0],k)*t+
        (Component(h[2],k)+Component(h[3],k)*t)*r;
    jacobian.v[6+k] = center[6+k]+Component(h[1],k)*r+
        (Component(h[0],k)+Component(h[3],k)*r)*s;
  }
  const auto& j = jacobian.v;
  const double determinant = (1./512.)*(j[0]*(j[4]*j[8]-j[5]*j[7])+
      j[1]*(j[5]*j[6]-j[3]*j[8])+j[2]*(j[3]*j[7]-j[4]*j[6]));
  if (!tl::math::Finite(determinant)) return Status::NonfiniteResult;
  if (determinant <= 0) return Status::InvalidGeometry;
  Matrix3 inverse;
  solid18::detail::InverseScaledJacobian(jacobian, determinant, 1./512., inverse);
  double natural[3][8];
  ReferenceNatural(ip, natural);
  solid18::detail::RegularDerivativeValues(natural, inverse, gradient);
  for (const auto& axis : gradient) {
    for (double value : axis) if (!tl::math::Finite(value)) return Status::NonfiniteResult;
  }
  return Status::Success;
}
}  // namespace tl::fea::solid18::total_strain::detail
