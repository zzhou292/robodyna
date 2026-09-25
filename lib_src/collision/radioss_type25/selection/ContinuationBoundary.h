// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ContinuationPrepare.h"
#include "BoundaryValues.h"
namespace tlfea::contact::radioss_type25::selection::detail {
TL_MATH_HOST_DEVICE inline void ContinuationBoundary(const NativeContinuationInput& input,
    const Work& w,NativeContinuationResult& out) {
  if(out.selected_subtriangle==0)return;
  const auto& in=input.pair;const unsigned i=unsigned(out.selected_subtriangle-1);
  auto& s=out.sector[i];
  if(s.penetration==0)return;
  if(s.raw_lb<-g::em03||s.raw_lc<-g::em03||s.raw_lb+s.raw_lc>1.+g::em03)s.far=1;
  const auto projected=v::Add(in.secondary,v::Scale(w.normal[i],w.plane_distance[i]));
  if(!w.frame.triangle) {
    const unsigned next=(i+1)%4;
    if(in.neighbors[i]==0) {
      if(v::Dot(v::Subtract(projected,w.frame.point[i]),w.frame.normal[i])>=in.secondary_gap&&w.shell)s.far=2;
    } else if((in.boundary_ids[i]&&!in.boundary_ids[next])||
              (in.boundary_ids[next]&&!in.boundary_ids[i])) {
      if(OutsideVertex(in,w,projected,in.boundary_ids[i]?i:next)&&w.shell)s.far=2;
    }
    if((s.far==1||w.plane_distance[i]<=0)&&
       (out.cylindrical_gap[i]||out.sliding_match[i]||out.sliding_match[next]))
      if(ConeSide(w,i,next,i)<-zep01)s.far=2;
  } else {
    const double d1=v::Dot(v::Subtract(projected,w.frame.point[0]),w.frame.normal[0]);
    const double d2=v::Dot(v::Subtract(projected,w.frame.point[1]),w.frame.normal[1]);
    const double d3=v::Dot(v::Subtract(projected,w.frame.point[2]),w.frame.normal[3]);
    if(((!in.neighbors[0]&&d1>=in.secondary_gap)||
        (!in.neighbors[1]&&d2>=in.secondary_gap)||
        (!in.neighbors[3]&&d3>=in.secondary_gap))&&w.shell)s.far=2;
    else {
      unsigned count=0,corner=0;
      for(unsigned k=0;k<3;++k)if(in.boundary_ids[k]){++count;corner=k;}
      if(count==1&&OutsideVertex(in,w,projected,corner)&&w.shell)s.far=2;
    }
    if(s.far==1||w.plane_distance[i]<=0) {
      if(in.neighbors[0]&&(out.cylindrical_gap[i]||out.sliding_match[0]||out.sliding_match[1])&&
         ConeSide(w,0,1,0)<-zep01)s.far=2;
      if(in.neighbors[1]&&(out.cylindrical_gap[i]||out.sliding_match[1]||out.sliding_match[2])&&
         ConeSide(w,1,2,1)<-zep01)s.far=2;
      if(in.neighbors[3]&&(out.cylindrical_gap[i]||out.sliding_match[2]||out.sliding_match[0])&&
         ConeSide(w,2,0,3)<-zep01)s.far=2;
    }
  }
  if(s.far==2)s.penetration=0;
}
} // namespace tlfea::contact::radioss_type25::selection::detail
