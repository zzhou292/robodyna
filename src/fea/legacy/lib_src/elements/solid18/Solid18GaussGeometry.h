// SPDX-License-Identifier: AGPL-3.0-or-later
// S8EJACIP3/S8EDERI3: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18CenterGeometry.h"

namespace tl::fea::solid18::detail {
// Preserve the native left-to-right additions, including subtraction signs.
TL_SOLID18_HD inline double GaussEntry(double c, double first, double second,
    double product, bool positive_first, bool positive_second) noexcept {
  const double a = positive_first ? c+first : c-first;
  const double b = positive_second ? a+second : a-second;
  return positive_first == positive_second ? b+product : b-product;
}

TL_SOLID18_HD inline Status GaussGeometry(StartupGeometry& g) noexcept {
  constexpr double pg = .577350269189625;
  const double pg2 = pg*pg;
  double h[4][3];
  for (unsigned k = 0; k < 3; ++k) {
    for (unsigned mode = 0; mode < 3; ++mode) {
      h[mode][k] = Component(g.higher_mode_m[mode],k)*pg;
    }
    h[3][k] = Component(g.higher_mode_m[3],k)*pg2;
  }
  const auto& c = g.center_scaled_jacobian_m.v;
  for (unsigned ip = 0; ip < 8; ++ip) {
    const bool r = (ip&1) != 0;
    const bool s = (ip&2) != 0;
    const bool t = (ip&4) != 0;
    auto& j = g.point[ip].scaled_jacobian_m.v;
    for (unsigned k = 0; k < 3; ++k) {
      j[k] = GaussEntry(c[k],h[2][k],h[1][k],h[3][k],s,t);
      j[3+k] = GaussEntry(c[3+k],h[0][k],h[2][k],h[3][k],t,r);
      j[6+k] = GaussEntry(c[6+k],h[1][k],h[0][k],h[3][k],r,s);
    }
  }
  // S8ZINIT3 visits r outermost, t innermost; IP storage remains r+2s+4t.
  g.characteristic_length_m = 1e30;
  g.integrated_volume_m3 = 0;
  for (unsigned r = 0; r < 2; ++r) {
    for (unsigned s = 0; s < 2; ++s) {
      for (unsigned t = 0; t < 2; ++t) {
        auto& point = g.point[r+2*s+4*t];
        const auto& j = point.scaled_jacobian_m.v;
        const double c59 = j[4]*j[8]-j[5]*j[7];
        const double c67 = j[5]*j[6]-j[3]*j[8];
        const double c48 = j[3]*j[7]-j[4]*j[6];
        const double determinant = (1.0/512.0)*(j[0]*c59+j[1]*c67+j[2]*c48);
        if (!tl::math::Finite(determinant)) return Status::NonfiniteResult;
        if (determinant <= 0) return Status::InvalidGeometry;
        point.initial_volume_m3 = 1.0*determinant;
        g.characteristic_length_m = ::fmin(g.characteristic_length_m,
            128*determinant*g.inverse_center_face_scale_per_m2);
        g.integrated_volume_m3 += point.initial_volume_m3;
      }
    }
  }
  if (!Positive(g.characteristic_length_m) || !Positive(g.integrated_volume_m3))
    return Status::NonfiniteResult;
  return Status::Success;
}
}  // namespace tl::fea::solid18::detail
