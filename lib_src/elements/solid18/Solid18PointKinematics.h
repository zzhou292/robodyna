// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected S8EDEFO3 I_SH2/ICP2/JCVT1: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid18Orientation.h"
#include "Solid18ForceTypes.h"

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

TL_SOLID18_HD inline Status PointKinematics(const PointDerivatives& point,
    const Vec3 (&velocity)[8], double dt, PointHistory& staged,
    PointObservation& observation) noexcept {
  const auto& p = point.regular_per_m;
  const auto& s = point.shear_per_m;
  const auto& b = point.cross_per_m;
  const double xy = RateSum(s[0],velocity,0);
  const double xz = RateSum(s[2],velocity,0);
  const double yx = RateSum(s[1],velocity,1);
  const double yz = RateSum(s[4],velocity,1);
  const double zx = RateSum(s[3],velocity,2);
  const double zy = RateSum(s[5],velocity,2);
  double xx = RateSum(p[0],velocity,0);
  double yy = RateSum(p[1],velocity,1);
  double zz = RateSum(p[2],velocity,2);
  const double original_trace = xx+yy+zz;
  yy = AddRateSum(yy,b[0],velocity,0);
  zz = AddRateSum(zz,b[2],velocity,0);
  xx = AddRateSum(xx,b[1],velocity,1);
  zz = AddRateSum(zz,b[4],velocity,1);
  xx = AddRateSum(xx,b[3],velocity,2);
  yy = AddRateSum(yy,b[5],velocity,2);
  const double correction = (xx+yy+zz-original_trace)*dt;
  double volume_change = correction*1.0;
  observation.selective_volume_increment = volume_change;
  // Native SDV is retained even when this storage-volume correction resets.
  if (volume_change > 1.0-1e-20) volume_change = 0;
  const double factor = 1.0-volume_change;
  observation.storage_volume_factor = factor;
  staged.storage_volume_m3 = staged.storage_volume_m3*factor;
  staged.material.internal_energy_density_j_m3 =
      staged.material.internal_energy_density_j_m3/factor;
  // IRESP0: the initial double volume does not receive this correction.
  const double half_dt = .5*dt;
  const double full_dt = 2*half_dt;
  auto& rate = observation.engineering_rate_per_s;
  rate[3] = xy+yx-full_dt*(xx*xy+yx*yy+zx*zy);
  rate[4] = yz+zy-full_dt*(yy*yz+zy*zz+xy*xz);
  rate[5] = xz+zx-full_dt*(zz*zx+xz*xx+yz*yx);
  rate[0] = xx-half_dt*(xx*xx+yx*yx+zx*zx);
  rate[1] = yy-half_dt*(yy*yy+zy*zy+xy*xy);
  rate[2] = zz-half_dt*(zz*zz+xz*xz+yz*yz);
  if (!Positive(staged.storage_volume_m3) ||
      !tl::math::Finite(staged.material.internal_energy_density_j_m3) ||
      !tl::math::Finite(correction)) return Status::NonfiniteResult;
  for (double value : rate) {
    if (!tl::math::Finite(value)) return Status::NonfiniteResult;
  }
  return Status::Success;
}
}  // namespace tl::fea::solid18::detail
