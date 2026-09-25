// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
// Complete local retained I25DST3_1 boundary/cone tests.
#include "BoundaryValues.h"
namespace tlfea::contact::radioss_type25::selection::detail {
TL_MATH_HOST_DEVICE inline void QuadBoundary(const NativePairInput& in,const Work& w,
    NativeRetainedResult& out,unsigned i) {
  auto& s=out.sector[i];const unsigned next=(i+1)%4;
  const auto projected=v::Add(in.secondary,v::Scale(w.normal[i],w.plane_distance[i]));
  if(in.neighbors[i]==0) {
    if(v::Dot(v::Subtract(projected,w.frame.point[i]),w.frame.normal[i])>=in.secondary_gap&&w.shell)s.far=2;
  } else if((in.boundary_ids[i]&&!in.boundary_ids[next])||
            (in.boundary_ids[next]&&!in.boundary_ids[i])) {
    if(OutsideVertex(in,w,projected,in.boundary_ids[i]?i:next)&&w.shell)s.far=2;
  }
  if(s.far==1||w.plane_distance[i]<=0)
    if(ConeSide(w,i,next,i)<-zep01)s.far=2;
  if(s.far==2)s.penetration=0;
}
TL_MATH_HOST_DEVICE inline void TriangleBoundary(const NativePairInput& in,const Work& w,
    NativeRetainedResult& out) {
  const auto projected=v::Add(in.secondary,v::Scale(w.normal[0],w.plane_distance[0]));
  const double d1=v::Dot(v::Subtract(projected,w.frame.point[0]),w.frame.normal[0]);
  const double d2=v::Dot(v::Subtract(projected,w.frame.point[1]),w.frame.normal[1]);
  const double d3=v::Dot(v::Subtract(projected,w.frame.point[2]),w.frame.normal[3]);
  if(!in.neighbors[0]&&d1>=in.secondary_gap&&w.shell)out.sector[0].far=2;
  else if(!in.neighbors[1]&&d2>=in.secondary_gap&&w.shell)out.sector[1].far=2;
  else if(!in.neighbors[3]&&d3>=in.secondary_gap&&w.shell)out.sector[2].far=2;
  else {
    unsigned count=0,corner=0;
    for(unsigned i=0;i<3;++i)if(in.boundary_ids[i]){++count;corner=i;}
    if(count==1&&OutsideVertex(in,w,projected,corner)&&w.shell)out.sector[0].far=2;
  }
  if(out.sector[0].far==1||w.plane_distance[0]<=0) {
    if(in.neighbors[0]!=0&&ConeSide(w,0,1,0)<-zep01)out.sector[0].far=2;
    if(in.neighbors[1]!=0&&ConeSide(w,1,2,1)<-zep01)out.sector[1].far=2;
    if(in.neighbors[3]!=0&&ConeSide(w,2,0,3)<-zep01)out.sector[2].far=2;
  }
  if(out.sector[0].far==2||out.sector[1].far==2||out.sector[2].far==2)
    out.sector[0].penetration=0;
}
TL_MATH_HOST_DEVICE inline void RetainedGeometry(const NativePairInput& in,Work& w,
    NativeRetainedResult& out) {
  RawProjection(w,out);
  const unsigned count=w.frame.triangle?1:4;
  for(unsigned i=0;i<count;++i) {
    ProjectSector(in,w,out,i);
    auto& s=out.sector[i];
    if(s.raw_lb<-g::em03||s.raw_lc<-g::em03||s.raw_lb+s.raw_lc>1.+g::em03)s.far=1;
    if(w.frame.triangle)TriangleBoundary(in,w,out);
    else QuadBoundary(in,w,out,i);
  }
}
} // namespace tlfea::contact::radioss_type25::selection::detail
