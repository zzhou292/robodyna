// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected S8ESELECSHT ISMSTR10: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "KinematicsTypes.h"
namespace tl::fea::solid18::total_strain::detail {
TL_SOLID18_HD inline Status SelectedTensor(PointKinematics (&point)[8]) noexcept {
  for (auto& p : point) {
    const auto& g = p.material_displacement_gradient;
    auto& b = p.selected_left_cauchy_green_minus_identity;
    b[0] = g[0]*(2+g[0])+g[1]*g[1]+g[2]*g[2];
    b[1] = g[4]*(2+g[4])+g[3]*g[3]+g[5]*g[5];
    b[2] = g[8]*(2+g[8])+g[6]*g[6]+g[7]*g[7];
    b[3] = g[1]+g[3]+g[0]*g[3]+g[1]*g[4]+g[2]*g[5];
    b[5] = g[2]+g[6]+g[0]*g[6]+g[1]*g[7]+g[2]*g[8];
    b[4] = g[7]+g[5]+g[6]*g[3]+g[7]*g[4]+g[8]*g[5];
    for (double x : b) if (!tl::math::Finite(x)) return Status::NonfiniteResult;
  }
  double xy[2]{}, xz[2]{}, yz[2]{};
  for (unsigned r = 0; r < 2; ++r) {
    for (unsigned s = 0; s < 2; ++s) {
      for (unsigned t = 0; t < 2; ++t) {
        const auto& b = point[r+2*s+4*t].selected_left_cauchy_green_minus_identity;
        xy[r] = xy[r]+b[3];
        xz[t] = xz[t]+b[5];
        yz[s] = yz[s]+b[4];
      }
    }
  }
  for (unsigned plane = 0; plane < 2; ++plane) {
    xy[plane] = xy[plane]*.25;
    xz[plane] = xz[plane]*.25;
    yz[plane] = yz[plane]*.25;
    if (!tl::math::Finite(xy[plane]) || !tl::math::Finite(xz[plane]) ||
        !tl::math::Finite(yz[plane])) return Status::NonfiniteResult;
  }
  for (unsigned r = 0; r < 2; ++r) {
    for (unsigned s = 0; s < 2; ++s) {
      for (unsigned t = 0; t < 2; ++t) {
        auto& b = point[r+2*s+4*t].selected_left_cauchy_green_minus_identity;
        b[3] = xy[r];
        b[5] = xz[t];
        b[4] = yz[s];
      }
    }
  }
  return Status::Success;
}
}  // namespace tl::fea::solid18::total_strain::detail
