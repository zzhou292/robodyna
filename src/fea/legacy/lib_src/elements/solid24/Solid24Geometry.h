// SPDX-License-Identifier: AGPL-3.0-or-later
// Native SZDERI3 / SDLEN3 / SLEN startup expressions, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid24Types.h"
#include "lib_src/elements/solid_common/BrickStartup.h"

namespace tl::fea::solid24::detail {
namespace brick = tl::fea::solid_common;
TL_BRICK_HD inline double CenterVolume(const Vec3 (&x)[8]) noexcept {
  Vec3 r, s, t;
  brick::Directions(x, r, s, t);
  // SZDERI3 uses T dot (R cross S), preserving its expression order.
  return (1.0/64.0)*brick::Dot(t, brick::Cross(r, s));
}
using brick::FaceMeasure;
using brick::CyclicFrame;
TL_BRICK_HD inline Status CharacteristicLength(StartupGeometry& geometry) noexcept {
  constexpr unsigned face[6][4]{{0,1,2,3},{4,5,6,7},{0,1,5,4},
                               {1,2,6,5},{2,3,7,6},{3,0,4,7}};
  const auto& x = geometry.local_position_m;
  double area[6], maximum = 0;
  for (unsigned f = 0; f < 6; ++f) {
    area[f] = FaceMeasure(x[face[f][0]],x[face[f][1]],x[face[f][2]],x[face[f][3]]);
    if (!tl::math::Finite(area[f])) return Status::NonfiniteResult;
    if (area[f] > maximum) maximum = area[f];
  }
  if (!brick::Positive(maximum)) return Status::InvalidGeometry;
  const double threshold = 1e-4*maximum;
  unsigned collapsed = 0;
  for (unsigned f = 0; f < 6; ++f) collapsed += area[f] < threshold;
  const double off = collapsed >= 3 ? 1e3 : 1;
  geometry.characteristic_length_m = 4*geometry.volume_m3*off/::sqrt(maximum);
  return brick::Positive(geometry.characteristic_length_m) ? Status::Success : Status::NonfiniteResult;
}
}  // namespace tl::fea::solid24::detail
