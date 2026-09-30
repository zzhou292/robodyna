// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Q4ContactBounds.h"
#include "lib_src/constraints/NodalRigidGroupMath.h"

namespace tlfea::contact::rigid_response_detail {
using Interval=Q4IntegralInterval;
// Reuse the contact interval primitives for signed linear combinations.
TL_SURFACE_HD inline bool Scale(Interval a,double coefficient,Interval& output) {
  Interval next;
  if(!q4_bounds::Scale(a,::fabs(coefficient),&next))return false;
  output=coefficient<0?Interval{-next.upper,-next.lower}:next;return true;
}
TL_SURFACE_HD inline bool Difference(Interval a,Interval b,Interval& output) {
  return q4_bounds::Add(a,{-b.upper,-b.lower},&output);
}
TL_SURFACE_HD inline bool LocalMomentBounds(tl::math::Vec3 point,tl::math::Vec3 center,
    tl::math::Vec3 normal,const tl::math::Matrix3& axes,Interval (&local)[3]) {
  const double p[]{point.x,point.y,point.z},c[]{center.x,center.y,center.z},
      n[]{normal.x,normal.y,normal.z};
  Interval arm[3],moment[3];
  for(unsigned i=0;i<3;++i)if(!q4_bounds::Difference(p[i],c[i],&arm[i]))return false;
  for(unsigned i=0;i<3;++i) {
    const auto j=(i+1)%3,k=(i+2)%3;Interval first,second;
    if(!Scale(arm[j],n[k],first)||!Scale(arm[k],n[j],second)||
       !Difference(first,second,moment[i]))return false;
  }
  for(unsigned axis=0;axis<3;++axis) {
    Interval sum;
    for(unsigned i=0;i<3;++i) {
      Interval term;
      if(!Scale(moment[i],axes.v[3*i+axis],term)||!q4_bounds::Add(sum,term,&sum))return false;
    }
    local[axis]=sum;
  }
  return true;
}
TL_SURFACE_HD inline bool SquareOver(Interval coefficient,double mass,double& output) {
  const double maximum=::fmax(::fabs(coefficient.lower),::fabs(coefficient.upper));
  double square=0;
  return q4_bounds::Finite(coefficient)&&mass_detail::UpperProduct(maximum,maximum,&square)&&
      mass_detail::UpperQuotient(square,mass,&output);
}
} // namespace tlfea::contact::rigid_response_detail
