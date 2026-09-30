// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected S8EDEFO3 JCVT1/ISCAU0: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid18Orientation.h"
namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline double RateSum(const double (&p)[8], const Vec3 (&v)[8],
                                   unsigned axis) noexcept {
  double sum = p[0]*Component(v[0],axis);
  for (unsigned n = 1; n < 8; ++n) sum += p[n]*Component(v[n],axis);
  return sum;
}

TL_SOLID18_HD inline double AddRateSum(double sum, const double (&p)[8],
    const Vec3 (&v)[8], unsigned axis) noexcept {
  for (unsigned n = 0; n < 8; ++n) sum += p[n]*Component(v[n],axis);
  return sum;
}

TL_SOLID18_HD inline void QuadraticEngineeringRate(double xx, double yy, double zz,
    double xy, double xz, double yx, double yz, double zx, double zy,
    double dt, double (&rate)[6]) noexcept {
  const double half_dt = .5*dt;
  const double full_dt = 2*half_dt;
  rate[3] = xy+yx-full_dt*(xx*xy+yx*yy+zx*zy);
  rate[4] = yz+zy-full_dt*(yy*yz+zy*zz+xy*xz);
  rate[5] = xz+zx-full_dt*(zz*zx+xz*xx+yz*yx);
  rate[0] = xx-half_dt*(xx*xx+yx*yx+zx*zx);
  rate[1] = yy-half_dt*(yy*yy+zy*zy+xy*xy);
  rate[2] = zz-half_dt*(zz*zz+xz*xz+yz*yz);
}
}  // namespace tl::fea::solid18::detail
