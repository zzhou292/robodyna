// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NewImpactProjection.h"
namespace tlfea::contact::radioss_type25::selection::detail {
TL_MATH_HOST_DEVICE inline double ImpactConeSide(const Work& w,Vector normal,
    unsigned first,unsigned second,bool opposite) {
  const auto edge=v::Subtract(w.frame.point[second],w.frame.point[first]);
  const auto plane=v::Cross(normal,edge);
  const double squared=v::Dot(plane,plane),arm_squared=v::Dot(w.relative[first],w.relative[first]);
  const double sum=v::Dot(w.relative[first],plane);
  const double inverse=::sqrt(1./g::Max(native_constant::em30,arm_squared*squared));
  return opposite?sum*inverse:-sum*inverse;
}
TL_MATH_HOST_DEVICE inline void BoundNewImpactSide(const NativePairInput& source,
    const Work& w,const Vector* normal,ImpactSideValues& side,bool opposite) {
  if(side.subtriangle==0)return;
  const unsigned i=unsigned(side.subtriangle-1);
  if(side.penetration[i]==0)return;
  const auto projected=v::Add(source.secondary,v::Scale(w.normal[i],w.plane_distance[i]));
  const double bb=w.plane_distance[i];
  if(!w.frame.triangle) {
    const unsigned next=(i+1)%4;
    if(source.neighbors[i]==0) {
      if(v::Dot(v::Subtract(projected,w.frame.point[i]),normal[i])>=source.secondary_gap&&w.shell)
        side.far[i]=3;
    } else if((source.boundary_ids[i]&&!source.boundary_ids[next])||
              (source.boundary_ids[next]&&!source.boundary_ids[i])) {
      if(OutsideVertex(source,w,projected,source.boundary_ids[i]?i:next)&&w.shell)side.far[i]=3;
    }
    if(side.cylindrical_gap[i]==1&&(side.far[i]==1||(opposite?bb>=0:bb<=0)))
      if(ImpactConeSide(w,normal[i],i,next,opposite)<-zep01)side.far[i]=2;
  } else {
    const double d1=v::Dot(v::Subtract(projected,w.frame.point[0]),normal[0]);
    const double d2=v::Dot(v::Subtract(projected,w.frame.point[1]),normal[1]);
    const double d3=v::Dot(v::Subtract(projected,w.frame.point[2]),normal[3]);
    if(((!source.neighbors[0]&&d1>=source.secondary_gap)||
        (!source.neighbors[1]&&d2>=source.secondary_gap)||
        (!source.neighbors[3]&&d3>=source.secondary_gap))&&w.shell)side.far[i]=3;
    else {
      unsigned count=0,corner=0;
      for(unsigned k=0;k<3;++k)if(source.boundary_ids[k]){++count;corner=k;}
      if(count==1&&OutsideVertex(source,w,projected,corner)&&w.shell)side.far[i]=3;
    }
    if(side.cylindrical_gap[i]==1&&(side.far[i]==1||(opposite?bb>=0:bb<=0))) {
      if(source.neighbors[0]&&ImpactConeSide(w,normal[0],0,1,opposite)<-zep01)side.far[i]=2;
      if(source.neighbors[1]&&ImpactConeSide(w,normal[1],1,2,opposite)<-zep01)side.far[i]=2;
      if(source.neighbors[3]&&ImpactConeSide(w,normal[3],2,0,opposite)<-zep01)side.far[i]=2;
    }
  }
  if(side.far[i]==2&&side.intersection==0)side.penetration[i]=0;
  if(side.far[i]==3)side.penetration[i]=0;
}
} // namespace tlfea::contact::radioss_type25::selection::detail
