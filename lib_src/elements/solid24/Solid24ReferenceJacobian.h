// SPDX-License-Identifier: AGPL-3.0-or-later
// SJACIDP expressions, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid24Types.h"

namespace tl::fea::solid24::detail {
TL_BRICK_HD inline double WorkingLengthMetres(WorkingLengthUnit units) noexcept {
  return units == WorkingLengthUnit::Metre ? 1.0 : 0.001;
}
TL_BRICK_HD inline Status GlobalReferenceJacobian(const Vec3 (&world)[8],
                                                 WorkingLengthUnit units,
                                                 ReferenceJacobian& output) noexcept {
  namespace brick = tl::fea::solid_common;
  double j[9];
  for (unsigned k = 0; k < 3; ++k) {
    const double a = brick::Component(world[6],k)-brick::Component(world[0],k);
    const double b = brick::Component(world[7],k)-brick::Component(world[1],k);
    const double c = brick::Component(world[4],k)-brick::Component(world[2],k);
    const double d = brick::Component(world[5],k)-brick::Component(world[3],k);
    j[k] = a+b-c-d;
    // SJACIDP preserves these sequential sums. The frame helper's grouped
    // Direction sums are a different native caller expression.
    j[3+k] = a+d+b+c;
    j[6+k] = a+d-b-c;
  }
  for (double value : j) if (!tl::math::Finite(value)) return Status::NonfiniteResult;
  const double c1 = j[4]*j[8]-j[5]*j[7];
  const double c2 = j[5]*j[6]-j[3]*j[8];
  const double c3 = j[3]*j[7]-j[4]*j[6];
  const double volume = (1.0/64.0)*(j[0]*c1+j[1]*c2+j[2]*c3);
  if (!tl::math::Finite(c1) || !tl::math::Finite(c2) || !tl::math::Finite(c3) ||
      !tl::math::Finite(volume)) return Status::NonfiniteResult;
  if (volume <= 0) return Status::InvalidGeometry;
  const double scale = WorkingLengthMetres(units);
  const double native_volume_floor_m3 = 1e-20*(scale*scale*scale);
  const double factor = (1.0/64.0)/(volume > native_volume_floor_m3 ? volume : native_volume_floor_m3);
  ReferenceJacobian next;
  auto& r = next.inverse;
  r[0] = factor*c1;
  r[3] = factor*c2;
  r[6] = factor*c3;
  r[1] = factor*(-j[1]*j[8]+j[2]*j[7]);
  r[4] = factor*( j[0]*j[8]-j[2]*j[6]);
  r[7] = factor*(-j[0]*j[7]+j[1]*j[6]);
  r[2] = factor*( j[1]*j[5]-j[2]*j[4]);
  r[5] = factor*(-j[0]*j[5]+j[2]*j[3]);
  r[8] = factor*( j[0]*j[4]-j[1]*j[3]);
  for (double value : r) if (!tl::math::Finite(value)) return Status::NonfiniteResult;
  next.volume_m3 = volume;
  output = next;
  return Status::Success;
}
}  // namespace tl::fea::solid24::detail
