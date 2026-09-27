// SPDX-License-Identifier: AGPL-3.0-or-later
// Exact S6CHOUR_CTL native wrapper projection, OpenRadioss a62b27e6.
#pragma once
#include "Types.h"
namespace tl::fea::solid6z::controlled_hourglass::detail {
TL_BRICK_HD inline Status Projection(const Vec3 (&x)[6],double (&output)[4][3])noexcept {
  namespace c=tl::fea::solid_common;
  for(const auto& v:x)if(!c::Finite(v))return Status::InvalidInput;
  double j[9];
  for(unsigned k=0;k<3;++k){
    const double a=c::Component(x[5],k)-c::Component(x[0],k);
    const double b=c::Component(x[5],k)-c::Component(x[1],k);
    const double d=c::Component(x[3],k)-c::Component(x[2],k);
    const double e=c::Component(x[4],k)-c::Component(x[2],k);
    const unsigned axis=(k+2)%3;j[axis]=a+b-d-e;
    const double first=a+e,second=b+d;j[3+axis]=first+second;j[6+axis]=first-second;
  }
  const double c59_68=j[4]*j[8]-j[5]*j[7],c67_49=j[5]*j[6]-j[3]*j[8];
  const double c19_37=j[0]*j[8]-j[2]*j[6],c48_57=j[3]*j[7]-j[4]*j[6];
  const double determinant=(1./64.)*(j[0]*c59_68+j[1]*c67_49+j[2]*c48_57);
  if(!tl::math::Finite(determinant))return Status::NonfiniteResult;
  if(determinant==0)return Status::InvalidGeometry;
  const double factor=(1./64.)/determinant;
  const double inverse[]{factor*c59_68,factor*(-j[1]*j[8]+j[2]*j[7]),factor*(j[1]*j[5]-j[2]*j[4]),
    factor*c67_49,factor*c19_37,factor*(-j[0]*j[5]+j[2]*j[3]),
    factor*c48_57,factor*(-j[0]*j[7]+j[1]*j[6]),factor*(j[0]*j[4]-j[1]*j[3])};
  double p[3][4],mode[3][3];
  for(unsigned k=0;k<3;++k){
    const unsigned a=3*((k+2)%3);double pair=inverse[a]-inverse[a+1];
    p[k][2]=pair+inverse[a+2];p[k][3]=pair-inverse[a+2];
    pair=inverse[a]+inverse[a+1];p[k][0]=-pair-inverse[a+2];p[k][1]=-pair+inverse[a+2];
    double v[6];for(unsigned n=0;n<6;++n)v[n]=c::Component(x[n],k);
    mode[k][0]=(1./8.)*(v[0]+v[1]-v[2]-v[2]-v[3]-v[4]+v[5]+v[5]);
    mode[k][1]=(1./8.)*(v[0]-v[1]-v[3]+v[4]);
    mode[k][2]=(1./8.)*(v[0]-v[1]+v[3]-v[4]);
  }
  double next[4][3];for(unsigned n=0;n<4;++n)for(unsigned h=0;h<3;++h){
    next[n][h]=p[0][n]*mode[0][h]+p[1][n]*mode[1][h]+p[2][n]*mode[2][h];
    if(!tl::math::Finite(next[n][h]))return Status::NonfiniteResult;
  }
  for(unsigned n=0;n<4;++n)for(unsigned h=0;h<3;++h)output[n][h]=next[n][h];return Status::Success;
}
} // namespace tl::fea::solid6z::controlled_hourglass::detail
