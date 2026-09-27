// SPDX-License-Identifier: AGPL-3.0-or-later
// SSORT_N4, OpenRadioss a62b27e6 (C) 2026 Siemens.
#pragma once
#include "Projection.h"
namespace tl::fea::solid_common::distortion::native::detail {
TL_BRICK_HD inline int SortCenter(Vec3 point,const Vec3 (&q)[4],double margin,
                                  double stif0,int flag) noexcept {
  if(flag<=0) {
    // SSORT applies reciprocal length AFTER the three-term unnormalized dot.
    Vec3 r,s;
    for(unsigned k=0;k<3;++k) {
      SetComponent(r,k,Component(q[1],k)+Component(q[2],k)-Component(q[0],k)-Component(q[3],k));
      SetComponent(s,k,Component(q[2],k)+Component(q[3],k)-Component(q[0],k)-Component(q[1],k));
    }
    const Vec3 n=Cross(r,s);const double norm=1.0/::fmax(1e-20,::sqrt(Dot(n,n)));
    const double distance=PlaneDistance(q[2],point,n)*norm;
    if(::fabs(distance)<margin&&stif0>0)flag=2;
  }
  if(flag==0)return 0;
  if(Distance1(q[3],q[2])==0)return Distance1(q[1],q[0])==0?0:3;
  if(Distance1(q[1],q[0])==0)return 6;
  if(Distance1(q[3],q[0])==0)return Distance1(q[2],q[1])==0?0:4;
  if(Distance1(q[2],q[1])==0)return 5;
  return flag;
}
TL_BRICK_HD inline int CornerTriangle(const Vec3 (&q)[4],int flag,double length) noexcept {
  if(flag==0||length<1e-20)return -1;
  if(Distance1(q[3],q[2])==0)return Distance1(q[1],q[0])==0?-1:34;
  if(Distance1(q[1],q[0])==0)return 12;
  if(Distance1(q[3],q[0])==0)return Distance1(q[2],q[1])==0?-1:14;
  if(Distance1(q[2],q[1])==0)return 23;
  return 0;
}
TL_BRICK_HD inline int SecondaryDegeneracy(const Vec3 (&q)[4]) noexcept {
  if(Distance1(q[3],q[2])==0)return Distance1(q[1],q[0])==0?-2:4;
  if(Distance1(q[1],q[0])==0)return 2;
  if(Distance1(q[3],q[0])==0)return Distance1(q[2],q[1])==0?-3:4;
  if(Distance1(q[2],q[1])==0)return 3;
  return 0;
}
} // namespace tl::fea::solid_common::distortion::native::detail
