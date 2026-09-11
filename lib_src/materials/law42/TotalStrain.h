// SPDX-License-Identifier: AGPL-3.0-or-later
// MULAW ISMSTR10/ISELECT0, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Types.h"

namespace tl::material::law42 {
TL_LAW42_HD inline bool TotalStrain(const double (&m)[9], double (&output)[6]) noexcept {
  for (double value : m) if (!tl::math::Finite(value)) return false;
  const double xx=m[0], xy=m[1], xz=m[2], yx=m[3], yy=m[4];
  const double yz=m[5], zx=m[6], zy=m[7], zz=m[8];
  double next[6];
  next[0] = xx*(2+xx)+xy*xy+xz*xz;
  next[1] = yy*(2+yy)+yx*yx+yz*yz;
  next[2] = zz*(2+zz)+zx*zx+zy*zy;
  next[3] = xy+yx+xx*yx+xy*yy+xz*yz;
  next[5] = xz+zx+xx*zx+xy*zy+xz*zz;
  next[4] = zy+yz+zx*yx+zy*yy+zz*yz;
  for (unsigned k=3; k<6; ++k) next[k] = 2*next[k];
  for (double value : next) if (!tl::math::Finite(value)) return false;
  for (unsigned k=0; k<6; ++k) output[k] = next[k];
  return true;
}
} // namespace tl::material::law42
