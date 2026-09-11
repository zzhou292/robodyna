// SPDX-License-Identifier: AGPL-3.0-or-later
// CBACOOR explicit velocity packet: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "QbatForceTypes.h"

namespace tl::fea::qbat::detail {
TL_QBAT_HD inline Vec3 LocalVector(const Matrix3& q,Vec3 v) {
  return {q.v[0]*v.x+q.v[3]*v.y+q.v[6]*v.z,
      q.v[1]*v.x+q.v[4]*v.y+q.v[7]*v.z,
      q.v[2]*v.x+q.v[5]*v.y+q.v[8]*v.z};
}
TL_QBAT_HD inline bool CorrectVelocity(const PrescribedInterval& in,Kinematics& k) {
  const auto& v=in.velocity_midpoint;
  const auto& p=k.geometry.centered_projected_position_m;
  const auto& q=k.geometry.frame;
  auto& w=k.corrected_velocity;
  w[0]=LocalVector(q,{v[0].x-v[2].x,v[0].y-v[2].y,v[0].z-v[2].z});
  w[1]=LocalVector(q,{v[1].x-v[3].x,v[1].y-v[3].y,v[1].z-v[3].z});
  w[2]=LocalVector(q,{v[0].x-v[1].x+v[2].x-v[3].x,
      v[0].y-v[1].y+v[2].y-v[3].y,v[0].z-v[1].z+v[2].z-v[3].z});
  const double x13=(p[0].x-p[2].x)*.5;
  const double x24=(p[1].x-p[3].x)*.5;
  const double y13=(p[0].y-p[2].y)*.5;
  const double y24=(p[1].y-p[3].y)*.5;
  const double exz=y24*w[0].z-y13*w[1].z;
  const double eyz=-x24*w[0].z+x13*w[1].z;
  const double ddry=(.5*in.dt)*exz*k.geometry.reciprocal_area_per_m2;
  const double ddrx=(.5*in.dt)*eyz*k.geometry.reciprocal_area_per_m2;
  const double old_x[3]{w[0].x,w[1].x,w[2].x};
  const double denominator_x=x13-x24;
  const double denominator_y=y13+y24;
  // Native computes the quotient then overwrites it below EM10. Avoid a
  // discarded division by zero while preserving every admitted branch value.
  const double ddrz1=::fabs(denominator_x)<1e-10 ? 0. :
      (.25*in.dt)*(w[0].y-w[1].y)/denominator_x;
  for (unsigned j=0;j<3;++j) w[j].x=w[j].x-ddry*w[j].z-ddrz1*w[j].y;
  const double ddrz2=::fabs(denominator_y)<1e-10 ? 0. :
      (.25*in.dt)*(old_x[0]+old_x[1])/denominator_y;
  for (unsigned j=0;j<3;++j) w[j].y=w[j].y-ddrx*w[j].z-ddrz2*old_x[j];
  Vec3 spin[4];
  for (unsigned j=0;j<4;++j) spin[j]=LocalVector(q,in.omega_midpoint[j]);
  auto& r=k.local_spin;
  r[0]=spin[0].x-spin[2].x;
  r[1]=spin[0].y-spin[2].y;
  r[2]=spin[1].x-spin[3].x;
  r[3]=spin[1].y-spin[3].y;
  r[4]=spin[0].x-spin[1].x+spin[2].x-spin[3].x;
  r[5]=spin[0].y-spin[1].y+spin[2].y-spin[3].y;
  r[6]=spin[0].x+spin[1].x+spin[2].x+spin[3].x;
  r[7]=spin[0].y+spin[1].y+spin[2].y+spin[3].y;
  for (auto x:w) if (!Finite(x)) return false;
  for (double x:r) if (!tl::math::Finite(x)) return false;
  return tl::math::Finite(ddrx) && tl::math::Finite(ddry) &&
      tl::math::Finite(ddrz1) && tl::math::Finite(ddrz2);
}
} // namespace tl::fea::qbat::detail
