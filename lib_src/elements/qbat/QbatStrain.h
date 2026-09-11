// SPDX-License-Identifier: AGPL-3.0-or-later
// CBADEFSH/CBADEF1/CBASTRA3: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "QbatVelocityCorrection.h"

namespace tl::fea::qbat::detail {
TL_QBAT_HD inline bool PointStrain(unsigned point,double dt,double thickness,Kinematics& k) {
  const auto& g=k.geometry;
  const auto& p=g.centered_projected_position_m;
  const double x13=((p[0].x-p[2].x)*.5)*g.reciprocal_area_per_m2;
  const double x24=((p[1].x-p[3].x)*.5)*g.reciprocal_area_per_m2;
  const double y13=((p[0].y-p[2].y)*.5)*g.reciprocal_area_per_m2;
  const double y24=((p[1].y-p[3].y)*.5)*g.reciprocal_area_per_m2;
  const auto& v=k.corrected_velocity;
  const auto& b=g.point[point].membrane_b_per_m;
  auto& r=k.rate[point];
  r[0]=b[0]*v[0].x+b[1]*v[1].x+b[2]*v[2].x;
  r[1]=b[4]*v[0].y+b[5]*v[1].y+b[6]*v[2].y;
  r[2]=y24*v[0].y-y13*v[1].y-x24*v[0].x+x13*v[1].x;
  for (unsigned j=3;j<8;++j) r[j]=0;
  auto& dx=k.strain_increment[point];
  for (unsigned j=0;j<8;++j) dx[j]=r[j]*dt;
  const double xz=dx[3];
  dx[3]=dx[4];
  dx[4]=xz;
  const double dtinv=dt/::fmax(dt*dt,1e-20);
  const double eps_k2=(dx[5]*dx[5]+dx[6]*dx[6]+dx[5]*dx[6]+.25*dx[7]*dx[7])*
      (1./9.)*thickness*thickness;
  const double eps_m2=(4./3.)*(dx[0]*dx[0]+dx[1]*dx[1]+dx[0]*dx[1]+.25*dx[2]*dx[2]);
  k.equivalent_rate_per_s[point]=::sqrt(eps_k2+eps_m2)*dtinv;
  for (double x:r) if (!tl::math::Finite(x)) return false;
  for (double x:dx) if (!tl::math::Finite(x)) return false;
  return tl::math::Finite(k.equivalent_rate_per_s[point]);
}
} // namespace tl::fea::qbat::detail
