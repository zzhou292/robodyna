// SPDX-License-Identifier: AGPL-3.0-or-later
// SZDERI3/SZDERITO3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid24ForceTypes.h"

namespace tl::fea::solid24::force_detail {
namespace brick = tl::fea::solid_common;
TL_BRICK_HD inline void InverseDerivatives(const double (&inverse)[9],double (&p)[3][4]) noexcept {
  for (unsigned k=0; k<3; ++k) {
    const double a=inverse[3*k], b=inverse[3*k+1], c=inverse[3*k+2];
    const double minus=a-b;
    p[k][1]=minus-c;
    p[k][3]=-minus-c;
    const double plus=a+b;
    p[k][0]=-plus-c;
    p[k][2]=plus-c;
  }
}
TL_BRICK_HD inline ForceStatus CurrentDerivatives(ForceGeometry& g) noexcept {
  Vec3 r,s,t;
  brick::Directions(g.current.local_position_m,r,s,t);
  const double j[]{t.x,t.y,t.z,r.x,r.y,r.z,s.x,s.y,s.z};
  for (double value:j) if (!tl::math::Finite(value)) return ForceStatus::NonfiniteResult;
  const double c1=j[4]*j[8]-j[5]*j[7];
  const double c2=j[5]*j[6]-j[3]*j[8];
  const double c3=j[3]*j[7]-j[4]*j[6];
  g.current.volume_m3=(1.0/64.0)*(j[0]*c1+j[1]*c2+j[2]*c3);
  if (!brick::Positive(g.current.volume_m3)) return ForceStatus::InvalidGeometry;
  const double f=(1.0/64.0)/g.current.volume_m3;
  double inverse[9]{f*c1,f*(-j[1]*j[8]+j[2]*j[7]),f*(j[1]*j[5]-j[2]*j[4]),
                    f*c2,f*(j[0]*j[8]-j[2]*j[6]),f*(-j[0]*j[5]+j[2]*j[3]),
                    f*c3,f*(-j[0]*j[7]+j[1]*j[6]),f*(j[0]*j[4]-j[1]*j[3])};
  for (double value:inverse) if (!tl::math::Finite(value)) return ForceStatus::NonfiniteResult;
  InverseDerivatives(inverse,g.derivative_per_m);
  g.jacobian_diagonal_m[0]=j[0];
  g.jacobian_diagonal_m[1]=j[4];
  g.jacobian_diagonal_m[2]=j[8];
  double mode[3][4];
  for (unsigned k=0; k<3; ++k) {
    double x[8];
    for (unsigned n=0; n<8; ++n) x[n]=brick::Component(g.current.local_position_m[n],k);
    mode[k][2]=(1.0/8.0)*(x[0]-x[1]+x[2]-x[3]+x[4]-x[5]+x[6]-x[7]);
    mode[k][0]=(1.0/8.0)*(x[0]+x[1]-x[2]-x[3]-x[4]-x[5]+x[6]+x[7]);
    mode[k][1]=(1.0/8.0)*(x[0]-x[1]-x[2]+x[3]-x[4]+x[5]+x[6]-x[7]);
    mode[k][3]=(1.0/8.0)*(-x[0]+x[1]-x[2]+x[3]+x[4]-x[5]+x[6]-x[7]);
  }
  for (unsigned n=0; n<4; ++n) for (unsigned h=0; h<4; ++h) {
    const double value=g.derivative_per_m[0][n]*mode[0][h]+
        g.derivative_per_m[1][n]*mode[1][h]+g.derivative_per_m[2][n]*mode[2][h];
    if (!tl::math::Finite(value)) return ForceStatus::NonfiniteResult;
    g.hourglass_projection[n][h]=value;
  }
  return ForceStatus::Success;
}
TL_BRICK_HD inline void ReferenceDerivatives(const ReferenceJacobian& j,double (&p)[3][4]) noexcept {
  // SZDERITO3 maps H8 [r,s,t] to HEPH [t,r,s] before shape assembly.
  const auto& a=j.inverse;
  const double permuted[]{a[2],a[0],a[1],a[5],a[3],a[4],a[8],a[6],a[7]};
  InverseDerivatives(permuted,p);
}
TL_BRICK_HD inline double Gradient(const double (&p)[4],const Vec3 (&v)[8],unsigned k) noexcept {
  const double a=brick::Component(v[0],k)-brick::Component(v[6],k);
  const double b=brick::Component(v[1],k)-brick::Component(v[7],k);
  const double c=brick::Component(v[2],k)-brick::Component(v[4],k);
  const double d=brick::Component(v[3],k)-brick::Component(v[5],k);
  return p[0]*a+p[1]*b+p[2]*c+p[3]*d;
}
} // namespace tl::fea::solid24::force_detail
