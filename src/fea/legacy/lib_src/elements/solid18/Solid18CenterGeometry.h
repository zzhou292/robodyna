// SPDX-License-Identifier: AGPL-3.0-or-later
// S8ZDERIC3: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18Orientation.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline Status CenterGeometry(StartupGeometry& g) noexcept {
  const auto& x = g.native_position_m;
  auto& j = g.center_scaled_jacobian_m.v;
  for (unsigned k = 0; k < 3; ++k) {
    const double x1 = Component(x[0],k), x2 = Component(x[1],k);
    const double x3 = Component(x[2],k), x4 = Component(x[3],k);
    const double x5 = Component(x[4],k), x6 = Component(x[5],k);
    const double x7 = Component(x[6],k), x8 = Component(x[7],k);
    const double a = x7-x1;
    const double b = x8-x2;
    const double c = x5-x3;
    const double d = x6-x4;
    j[3+k] = a+b-c-d;
    const double first = a+d;
    const double second = b+c;
    j[6+k] = first+second;
    j[k] = first-second;
    SetComponent(g.higher_mode_m[0],k,x1+x2-x3-x4-x5-x6+x7+x8);
    SetComponent(g.higher_mode_m[1],k,x1-x2-x3+x4-x5+x6+x7-x8);
    SetComponent(g.higher_mode_m[2],k,x1-x2+x3-x4+x5-x6+x7-x8);
    SetComponent(g.higher_mode_m[3],k,-x1+x2-x3+x4+x5-x6+x7-x8);
  }
  const double c59 = j[4]*j[8]-j[5]*j[7];
  const double c67 = j[5]*j[6]-j[3]*j[8];
  const double c48 = j[3]*j[7]-j[4]*j[6];
  const double c38 = -j[1]*j[8]+j[2]*j[7];
  const double c19 = j[0]*j[8]-j[2]*j[6];
  const double c27 = -j[0]*j[7]+j[1]*j[6];
  const double c26 = j[1]*j[5]-j[2]*j[4];
  const double c34 = -j[0]*j[5]+j[2]*j[3];
  const double c15 = j[0]*j[4]-j[1]*j[3];
  g.center_volume_m3 = (1.0/64.0)*(j[0]*c59+j[1]*c67+j[2]*c48);
  if (!tl::math::Finite(g.center_volume_m3)) return Status::NonfiniteResult;
  if (g.center_volume_m3 <= 0) return Status::InvalidGeometry;
  double face = c59*c59+c67*c67+c48*c48;
  face = ::fmax(face,c38*c38+c19*c19+c27*c27);
  face = ::fmax(face,c26*c26+c34*c34+c15*c15);
  if (!Positive(face)) return Status::InvalidGeometry;
  g.inverse_center_face_scale_per_m2 = 1.0/::sqrt(face);
  for (const auto& h : g.higher_mode_m) {
    if (!Finite(h)) return Status::NonfiniteResult;
  }
  return Status::Success;
}
}  // namespace tl::fea::solid18::detail
