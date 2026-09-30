// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected engine SDLEN3/SLEN, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "BrickStartup.h"

namespace tl::fea::solid_common {
// Native eight-slot packet, including a caller's explicit S6Z expansion.
// LAW42, Lagrangian, IDT1SOL0/IDTS6=0: no ALE/special degeneration branch.
TL_BRICK_HD inline bool Law42CharacteristicLength(const Vec3 (&x)[8],
    double volume_m3,double& output) noexcept {
  if (!Positive(volume_m3)) return false;
  constexpr unsigned face[6][4]{{0,1,2,3},{4,5,6,7},{0,1,5,4},
                              {1,2,6,5},{2,3,7,6},{3,0,4,7}};
  // SDLEN3 initializes AREAM to native EM20 in this SI packet profile.
  double maximum=1e-20;
  for (const auto& f:face) {
    const double area=FaceMeasure(x[f[0]],x[f[1]],x[f[2]],x[f[3]]);
    if (!tl::math::Finite(area)) return false;
    if (area>maximum) maximum=area;
  }
  if (!Positive(maximum)) return false;
  const double length=4*volume_m3*1.0/::sqrt(maximum);
  if (!Positive(length)) return false;
  output=length;
  return true;
}
} // namespace tl::fea::solid_common
