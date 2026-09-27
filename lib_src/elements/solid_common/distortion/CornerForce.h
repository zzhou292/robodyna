// SPDX-License-Identifier: AGPL-3.0-or-later
// SFOR_NS2S4 / SFOR_4N2S4, OpenRadioss a62b27e6 (C) 2026 Siemens.
#pragma once
#include "FaceSort.h"
#include "Penalty.h"
namespace tl::fea::solid_common::distortion::native::detail {
TL_BRICK_HD inline bool CornerPenalty(const Parameters& p,const ForceInput& in,
    const unsigned (&face)[4],unsigned secondary,const Vec3 (&q)[4],int triangle,
    double penmin,double penref,Work& w) noexcept {
  unsigned ia,ib,ic;
  if(triangle==1){ia=3;ib=0;ic=1;}
  else if(triangle==2){ia=0;ib=1;ic=2;}
  else if(triangle==3){ia=1;ib=2;ic=3;}
  else if(triangle==4){ia=2;ib=3;ic=0;}
  else if(triangle==34){ia=2;ib=0;ic=1;}
  else if(triangle==14){ia=3;ib=1;ic=2;}
  else if(triangle==23){ia=1;ib=3;ic=0;}
  else{ia=0;ib=2;ic=3;} // Native degenerate code12.
  double area2;const Vec3 normal=TriangleNormal(q[ia],q[ib],q[ic],area2);
  if(!tl::math::Finite(area2)||!Finite(normal))return false;
  const double penetration=::fmax(0.0,-(PlaneDistance(q[ib],in.position[secondary],normal)-penmin));
  if(penetration==0)return true;
  TriangleWeights weights;if(!Barycentric(in.position[secondary],q[ia],q[ib],q[ic],weights))return false;
  double h[4]{};h[ib]=weights.b;h[ic]=weights.c;
  // Preserve a62's case3 HJ(1)=LA even though its triangle XA is node2.
  h[triangle==3?0:ia]=weights.a;
  Penalty(p,in,face,in.velocity[secondary],normal,penetration,penref,h,w.force[secondary],w);
  if(p.control_stiffness*penetration>0)++w.corner_contacts;
  return FiniteWork(w);
}
TL_BRICK_HD inline bool CornerForces(const Parameters& p,const ForceInput& in,
    const unsigned (&face)[4],const unsigned (&opposite)[4],int flag,double penmin,
    double penref,double margin,Work& w) noexcept {
  Vec3 q[4],secondary[4];for(unsigned n=0;n<4;++n){q[n]=in.position[face[n]];secondary[n]=in.position[opposite[n]];}
  const int degenerate=CornerTriangle(q,flag,p.length);if(degenerate==-1)return true;
  double area2;const Vec3 normal=QuadNormal(q,area2);
  if(!tl::math::Finite(area2)||!Finite(normal))return false;
  const int secondary_code=SecondaryDegeneracy(secondary);
  for(unsigned n=0;n<4;++n) {
    if(n>0&&::abs(secondary_code)==static_cast<int>(n+1))continue;
    if(n==1&&Distance1(secondary[1],secondary[0])==0)continue;
    if(Distance1(secondary[n],q[n])==0)continue;
    const Vec3 difference=Difference(secondary[n],q[n]);
    const double distance=::fabs(Dot(normal,difference));
    if(distance<margin&&p.control_stiffness>0)
      if(!CornerPenalty(p,in,face,opposite[n],q,degenerate==0?static_cast<int>(n+1):degenerate,penmin,penref,w))return false;
  }
  return true;
}
} // namespace tl::fea::solid_common::distortion::native::detail
