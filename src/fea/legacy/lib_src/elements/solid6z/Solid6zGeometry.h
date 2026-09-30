// SPDX-License-Identifier: AGPL-3.0-or-later
// Native S6ZDERI3 / SLEN expressions, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid6zJacobian.h"

namespace tl::fea::solid6z::detail {
TL_BRICK_HD inline Status LocalGeometry(const Vec3 (&world)[6], StartupGeometry& g) noexcept {
  // S6ZRCOOR3 expands its two triangle faces only for SREPISO3/SORTHO3.
  const Vec3 embedded[8]{world[0],world[1],world[2],world[2],
                         world[3],world[4],world[5],world[5]};
  if (!brick::CyclicFrame(embedded,g.frame)) return Status::InvalidGeometry;
  auto& x = g.local_position_m;
  for (unsigned n = 0; n < 6; ++n) {
    x[n] = brick::Local(g.frame,world[n]);
    if (!brick::Finite(x[n])) return Status::NonfiniteResult;
  }
  const Jacobian a = EvaluateJacobian(x);
  if (!Finite(a)) return Status::NonfiniteResult;
  if (!brick::Positive(a.volume)) return Status::InvalidGeometry;
  g.volume_m3 = a.volume;
  const Vec3 x21{x[1].x-x[0].x,x[1].y-x[0].y,x[1].z-x[0].z};
  const Vec3 x31{x[2].x-x[0].x,x[2].y-x[0].y,x[2].z-x[0].z};
  const Vec3 x54{x[4].x-x[3].x,x[4].y-x[3].y,x[4].z-x[3].z};
  const Vec3 x64{x[5].x-x[3].x,x[5].y-x[3].y,x[5].z-x[3].z};
  g.axial_volume_gradient_m3 = .25*(
      a.j[8]*(x54.x*x64.y-x21.x*x31.y-x64.x*x54.y+x31.x*x21.y)
     -a.j[7]*(x54.x*x64.z+x31.x*x21.z-x21.x*x31.z-x64.x*x54.z)
     +a.j[6]*(x54.y*x64.z+x31.y*x21.z-x21.y*x31.z-x64.y*x54.z));
  if (!tl::math::Finite(g.axial_volume_gradient_m3)) return Status::NonfiniteResult;

  // Preserve the donor's first-face Y order (1,2,4,5), even though its X/Z
  // orders are (1,2,5,4). Replacing these two values changes native DELTAX.
  const Vec3 c{x[4].x,x[3].y,x[4].z}, d{x[3].x,x[4].y,x[3].z};
  const double area[5]{brick::FaceMeasure(x[0],x[1],c,d),
                       brick::FaceMeasure(x[1],x[4],x[5],x[2]),
                       brick::FaceMeasure(x[0],x[3],x[5],x[2]),
                       brick::FaceMeasure(x[0],x[1],x[2],x[2]),
                       brick::FaceMeasure(x[3],x[4],x[5],x[5])};
  double maximum = 0;
  for (double value : area) {
    if (!tl::math::Finite(value)) return Status::NonfiniteResult;
    if (value > maximum) maximum = value;
  }
  if (!brick::Positive(maximum)) return Status::InvalidGeometry;
  unsigned collapsed = 0;
  const double threshold = 1e-4*maximum;
  for (double value : area) collapsed += value < threshold;
  const double factor = collapsed >= 2 ? 1e3 : 1;
  g.characteristic_length_m = 4*g.volume_m3*factor/::sqrt(maximum);
  return brick::Positive(g.characteristic_length_m) ? Status::Success : Status::NonfiniteResult;
}
}  // namespace tl::fea::solid6z::detail
