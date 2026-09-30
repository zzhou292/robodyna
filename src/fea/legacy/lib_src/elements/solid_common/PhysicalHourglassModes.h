// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SZHOUR3 physical modes, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "BrickFrame.h"

namespace tl::fea::solid_common {
TL_BRICK_HD inline void ModeRates(const Vec3 (&velocity)[8],
    const double (&projection)[4][4],double (&rate)[3][4]) noexcept {
  namespace brick=tl::fea::solid_common;
  for (unsigned k=0; k<3; ++k) {
    double v[8];
    for (unsigned n=0; n<8; ++n) v[n]=brick::Component(velocity[n],k);
    const double a=v[2]-v[3]-v[6]+v[7];
    const double b=v[1]-v[2]-v[4]+v[7];
    const double c=v[0]-v[3]-v[5]+v[6];
    const double d=v[0]-v[1]-v[4]+v[5];
    rate[k][2]=(c-b)*(1.0/8.0);
    rate[k][0]=(c+b)*(1.0/8.0);
    rate[k][1]=(d-a)*(1.0/8.0);
    rate[k][3]=-(d+a)*(1.0/8.0);
    const double v17=v[0]-v[6],v28=v[1]-v[7],v35=v[2]-v[4],v46=v[3]-v[5];
    const auto& p=projection;
    for (unsigned h=0; h<4; ++h)
      rate[k][h]=rate[k][h]-(p[0][h]*v17+p[1][h]*v28+p[2][h]*v35+p[3][h]*v46);
  }
}
TL_BRICK_HD inline void CoupledModes(const double (&h)[6],
    const double (&f)[3][4],double (&n)[3][4]) noexcept {
  // ICP1: NU1=4/3, NU2=-2/3, NU3=100/225; no pressure-mode stiffness.
  const double h11=h[0],h22=h[1],h33=h[2],h12=h[3],h13=h[4],h23=h[5];
  n[0][0]=(h22+h33)*f[0][0]+h12*f[1][1]+h13*f[2][2];
  n[1][1]=(h11+h33)*f[1][1]+h23*f[2][2]+h12*f[0][0];
  n[2][2]=(h11+h22)*f[2][2]+h13*f[0][0]+h23*f[1][1];
  n[0][1]=(4.0/3.0)*h11*f[0][1]+(-2.0/3.0)*h12*f[1][0];
  n[0][2]=(4.0/3.0)*h11*f[0][2]+(-2.0/3.0)*h13*f[2][0];
  n[1][0]=(4.0/3.0)*h22*f[1][0]+(-2.0/3.0)*h12*f[0][1];
  n[2][0]=(4.0/3.0)*h33*f[2][0]+(-2.0/3.0)*h13*f[0][2];
  n[1][2]=(4.0/3.0)*h22*f[1][2]+(-2.0/3.0)*h23*f[2][1];
  n[2][1]=(4.0/3.0)*h33*f[2][1]+(-2.0/3.0)*h23*f[1][2];
  n[0][3]=(100.0/225.0)*h11*f[0][3];
  n[1][3]=(100.0/225.0)*h22*f[1][3];
  n[2][3]=(100.0/225.0)*h33*f[2][3];
}
TL_BRICK_HD inline double ModePower(const double (&f)[3][4],const double (&rate)[3][4]) noexcept {
  // Preserve native Z,X,Y accumulation order for signed work.
  return f[2][0]*rate[2][0]+f[2][1]*rate[2][1]+f[2][2]*rate[2][2]+f[2][3]*rate[2][3]+
         f[0][0]*rate[0][0]+f[0][1]*rate[0][1]+f[0][2]*rate[0][2]+f[0][3]*rate[0][3]+
         f[1][0]*rate[1][0]+f[1][1]*rate[1][1]+f[1][2]*rate[1][2]+f[1][3]*rate[1][3];
}
TL_BRICK_HD inline void ModeForces(const double (&projection)[4][4],
    const double (&mode)[3][4],Vec3 (&force)[8]) noexcept {
  namespace brick=tl::fea::solid_common;
  for (unsigned k=0; k<3; ++k) {
    const auto& n=mode[k]; const auto& p=projection;
    const double a=(n[0]+n[2])*(1.0/8.0),b=(n[0]-n[2])*(1.0/8.0);
    const double c=(n[1]+n[3])*(1.0/8.0),d=(n[1]-n[3])*(1.0/8.0);
    double v=-p[0][0]*n[0]-p[0][1]*n[1]-p[0][2]*n[2]-p[0][3]*n[3];
    brick::SetComponent(force[0],k,-(a+d+v));
    brick::SetComponent(force[6],k,-(a+c-v));
    v=-p[1][0]*n[0]-p[1][1]*n[1]-p[1][2]*n[2]-p[1][3]*n[3];
    brick::SetComponent(force[1],k,-(b-d+v));
    brick::SetComponent(force[7],k,-(b-c-v));
    v=-p[2][0]*n[0]-p[2][1]*n[1]-p[2][2]*n[2]-p[2][3]*n[3];
    brick::SetComponent(force[2],k,-(-b-c+v));
    brick::SetComponent(force[4],k,-(-b-d-v));
    v=-p[3][0]*n[0]-p[3][1]*n[1]-p[3][2]*n[2]-p[3][3]*n[3];
    brick::SetComponent(force[3],k,-(-a+c+v));
    brick::SetComponent(force[5],k,-(-a+d-v));
  }
}
} // namespace tl::fea::solid_common
