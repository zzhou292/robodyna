// SPDX-License-Identifier: AGPL-3.0-or-later
// SFOR_N2S4, OpenRadioss a62b27e6 (C) 2026 Siemens.
#pragma once
#include "FaceSort.h"
#include "Penalty.h"
namespace tl::fea::solid_common::distortion::native::detail {
TL_BRICK_HD inline bool CenterForce(const Parameters& p,const ForceInput& in,
    Vec3 center,Vec3 mean,const unsigned (&face)[4],int flag,double penmin,
    double penref,double margin,Work& w) noexcept {
  Vec3 q[4];for(unsigned n=0;n<4;++n)q[n]=in.position[face[n]];
  const int code=SortCenter(center,q,margin,p.control_stiffness,flag);
  if(code==0)return true;
  Vec3 centroid;
  for(unsigned k=0;k<3;++k)SetComponent(centroid,k,.25*(Component(q[0],k)+Component(q[1],k)+Component(q[2],k)+Component(q[3],k)));
  double area2;Vec3 normal=QuadNormal(q,area2);
  if(!tl::math::Finite(area2)||!Finite(normal))return false;
  double penetration=::fmax(0.0,-(PlaneDistance(centroid,center,normal)-penmin));
  if(4.0*area2<penmin*p.length)penetration=::fmin(penetration,(1.0/10.0)*penmin);
  if(area2<1e-20)penetration=0;
  if(penetration==0)return true;
  int triangle=1;Vec3 a,b,c;
  if(code>=3) {
    if(code==3){a=q[2];b=q[0];c=q[1];}
    else if(code==4){a=q[3];b=q[1];c=q[2];}
    else if(code==5){a=q[1];b=q[3];c=q[0];}
    else{a=q[0];b=q[2];c=q[3];}
  } else {
    a=centroid;
    const double p3=::fmax(0.0,-(PlaneDistance(q[2],center,normal)-penmin));
    const double p4=::fmax(0.0,-(PlaneDistance(q[3],center,normal)-penmin));
    if(p3>penetration&&p4>penetration){triangle=3;b=q[2];c=q[3];}
    else if(p3>penetration){triangle=2;b=q[1];c=q[2];}
    else if(p4>penetration){triangle=4;b=q[3];c=q[0];}
    else{b=q[0];c=q[1];}
  }
  if(triangle!=1) {
    normal=TriangleNormal(a,b,c,area2);
    penetration=::fmax(0.0,-(PlaneDistance(q[2],center,normal)-penmin));
    if(4.0*area2<penmin*p.length)penetration=::fmin(penetration,(1.0/10.0)*penmin);
  }
  if(penetration==0)return true;
  TriangleWeights weights;if(!Barycentric(center,a,b,c,weights))return false;
  if(weights.minimum<-2.0/1000.0)penetration=::fmin(penetration,penmin);
  double h[4]{};
  if(code>=3) {
    const unsigned ib=code==3?0:code==4?1:code==5?3:2;
    const unsigned ic=code==3?1:code==4?2:code==5?0:3;
    const unsigned ia=code==3?2:code==4?3:code==5?1:0;
    h[ib]=weights.b;h[ic]=weights.c;h[ia]=weights.a;
  } else {
    const double h0=.25*weights.a;for(double& value:h)value=h0;
    h[triangle-1]=h0+weights.b;h[triangle%4]=h0+weights.c;
  }
  Penalty(p,in,face,mean,normal,penetration,penref,h,w.center_force,w);
  if(p.control_stiffness*penetration>0)++w.center_contacts;
  return FiniteWork(w);
}
} // namespace tl::fea::solid_common::distortion::native::detail
