// SPDX-License-Identifier: AGPL-3.0-or-later
// Shared identical barycentric branch in native SFOR_N2S4 and SFOR_NS2S4.
#pragma once
#include "ForceTypes.h"
namespace tl::fea::solid_common::distortion::native::detail {
struct TriangleWeights {double a=0,b=0,c=0,minimum=0;};
TL_BRICK_HD inline Vec3 QuadNormal(const Vec3 (&q)[4],double& area2) noexcept {
  Vec3 r,s;
  for(unsigned k=0;k<3;++k) {
    SetComponent(r,k,Component(q[1],k)+Component(q[2],k)-Component(q[0],k)-Component(q[3],k));
    SetComponent(s,k,Component(q[2],k)+Component(q[3],k)-Component(q[0],k)-Component(q[1],k));
  }
  Vec3 n=Cross(r,s);area2=Dot(n,n);const double norm=1.0/::fmax(1e-20,::sqrt(area2));
  n.x=n.x*norm;n.y=n.y*norm;n.z=n.z*norm;return n;
}
TL_BRICK_HD inline Vec3 TriangleNormal(Vec3 a,Vec3 b,Vec3 c,double& area2) noexcept {
  Vec3 n=Cross(Difference(b,a),Difference(c,a));area2=Dot(n,n);
  const double norm=1.0/::fmax(1e-20,::sqrt(area2));
  n.x=n.x*norm;n.y=n.y*norm;n.z=n.z*norm;return n;
}
TL_BRICK_HD inline double PlaneDistance(Vec3 q,Vec3 point,Vec3 normal) noexcept {
  return (q.x-point.x)*normal.x+(q.y-point.y)*normal.y+(q.z-point.z)*normal.z;
}
TL_BRICK_HD inline bool Barycentric(Vec3 point,Vec3 a,Vec3 b,Vec3 c,TriangleWeights& out) noexcept {
  const Vec3 ab=Difference(b,a),ca=Difference(a,c);
  const Vec3 ia=Difference(a,point),ib=Difference(b,point),ic=Difference(c,point);
  const Vec3 s{-ab.y*ca.z+ab.z*ca.y,-ab.z*ca.x+ab.x*ca.z,-ab.x*ca.y+ab.y*ca.x};
  const double s2=Dot(s,s);
  if(!tl::math::Finite(s2)||s2<=0)return false;
  const Vec3 sa=Cross(ib,ic),sb=Cross(ic,ia);
  out.a=Dot(s,sa)/s2;out.b=Dot(s,sb)/s2;out.c=1.0-out.a-out.b;
  if(!tl::math::Finite(out.a)||!tl::math::Finite(out.b)||!tl::math::Finite(out.c))return false;
  out.minimum=::fmin(::fmin(out.a,out.b),out.c);
  if(out.a<0) {
    if(out.b<0){out.a=0;out.b=0;out.c=1;}
    else if(out.c<0){out.c=0;out.a=0;out.b=1;}
    else{out.a=0;const double sum=out.b+out.c;out.b=out.b/sum;out.c=out.c/sum;}
  } else if(out.b<0) {
    if(out.c<0){out.b=0;out.c=0;out.a=1;}
    else{out.b=0;const double sum=out.c+out.a;out.c=out.c/sum;out.a=out.a/sum;}
  } else if(out.c<0) {
    out.c=0;const double sum=out.a+out.b;out.a=out.a/sum;out.b=out.b/sum;
  }
  return true;
}
} // namespace tl::fea::solid_common::distortion::native::detail
