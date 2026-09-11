// SPDX-License-Identifier: AGPL-3.0-or-later
// S6ZDERITO3 / S6ZDEFOT3 / S6ZDEFC3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid6zForceTypes.h"

namespace tl::fea::solid6z::force_detail {
TL_BRICK_HD inline void PointGradients(const double (&inverse)[9],
                                       double (&gradient)[3][6]) noexcept {
  for (unsigned axis = 0; axis < 3; ++axis) {
    const double first = inverse[3*axis];
    const double second = inverse[3*axis+1];
    const double third = inverse[3*axis+2];
    const double sum = first+second;
    gradient[axis][0] = -sum-(1.0/3.0)*third;
    gradient[axis][3] = -sum+(1.0/3.0)*third;
    gradient[axis][1] = first-(1.0/3.0)*third;
    gradient[axis][4] = first+(1.0/3.0)*third;
    gradient[axis][2] = second-(1.0/3.0)*third;
    gradient[axis][5] = second+(1.0/3.0)*third;
  }
}
TL_BRICK_HD inline void VectorGradient(const double (&gradient)[3][6],
    const Vec3 (&value)[6], double (&result)[9]) noexcept {
  for (unsigned component = 0; component < 3; ++component) {
    for (unsigned derivative = 0; derivative < 3; ++derivative) {
      const auto& p = gradient[derivative];
      result[3*component+derivative] =
          p[0]*solid_common::Component(value[0],component)+
          p[1]*solid_common::Component(value[1],component)+
          p[4]*solid_common::Component(value[4],component)+
          p[2]*solid_common::Component(value[2],component)+
          p[5]*solid_common::Component(value[5],component)+
          p[3]*solid_common::Component(value[3],component);
    }
  }
}
// S6ZDEFO3 explicit JCVT1/ISMSTR10 branch: keep all nine uncorrected
// velocity-gradient entries until each native quadratic product is formed.
TL_BRICK_HD inline void EngineeringRate(const double (&gradient)[9],
    double dt_s, double (&rate)[6]) noexcept {
  const double xx = gradient[0], xy = gradient[1], xz = gradient[2];
  const double yx = gradient[3], yy = gradient[4], yz = gradient[5];
  const double zx = gradient[6], zy = gradient[7], zz = gradient[8];
  const double half_dt = .5*dt_s;
  rate[0] = xx-half_dt*(xx*xx+yx*yx+zx*zx);
  rate[1] = yy-half_dt*(yy*yy+zy*zy+xy*xy);
  rate[2] = zz-half_dt*(zz*zz+xz*xz+yz*yz);
  double correction = half_dt*(xx*xy+yx*yy+zx*zy);
  rate[3] = (xy-correction)+(yx-correction);
  correction = half_dt*(yy*yz+zy*zz+xy*xz);
  rate[4] = (yz-correction)+(zy-correction);
  correction = half_dt*(zz*zx+xz*xx+yz*yx);
  rate[5] = (xz-correction)+(zx-correction);
}
} // namespace tl::fea::solid6z::force_detail
