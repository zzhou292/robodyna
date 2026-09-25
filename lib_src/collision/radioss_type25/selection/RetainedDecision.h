// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "RetainedBoundary.h"
namespace tlfea::contact::radioss_type25::selection::detail {
TL_MATH_HOST_DEVICE inline void SelectRetained(const NativePairInput& in,const Work& w,
    NativeRetainedResult& out) {
  int selected=out.prior_subtriangle;
  if(!w.frame.triangle) {
    const double minimum=g::Min(g::Min(g::Min(out.sector[0].distance_squared,
        out.sector[1].distance_squared),out.sector[2].distance_squared),out.sector[3].distance_squared);
    double best=native_constant::ep20;
    for(unsigned i=0;i<4;++i) {
      const auto& s=out.sector[i];
      if(s.distance_squared<=onep03*minimum) {
        const double lateral=(s.raw_lb>=0&&s.raw_lc>=0)?0.:
            g::Max(0.,s.distance_squared-w.plane_distance[i]*w.plane_distance[i]);
        if(lateral<best) {
          selected=int(i+1);out.distance_squared=s.distance_squared;best=lateral;
        }
      }
    }
  } else if(out.sector[0].distance_squared<=out.distance_squared) {
    selected=1;out.distance_squared=out.sector[0].distance_squared;
  }
  if(selected!=0&&out.sector[selected-1].penetration==0)selected=0;
  for(unsigned i=0;i<4;++i)if(int(i+1)!=selected)out.sector[i].penetration=0;
  out.selected_subtriangle=selected;
  auto& row=out.history.row;
  row.irtlm[1]=selected+5*out.prior_subtriangle;
  row.selection_metric[0]=out.distance_squared;
  const unsigned old=unsigned(out.prior_subtriangle-1);
  const bool slide=out.sector[old].penetration==0 ||
      (!w.frame.triangle ? out.sector[old].far>=2 :
       out.sector[0].far>=2||out.sector[1].far>=2||out.sector[2].far>=2);
  if(slide)row.irtlm[1]=-row.irtlm[1];
}
TL_MATH_HOST_DEVICE inline void PublishCache(NativeRetainedResult& out) {
  for(unsigned i=0;i<4;++i) {
    const auto& s=out.sector[i];auto& c=out.cache.sector[i];
    c.far=s.far;c.penetration=s.penetration;
    c.defined=s.defined&(FarDefined|PenetrationDefined|ClampedBarycentricDefined);
    if(s.defined&ClampedBarycentricDefined){c.lb=s.clamped_lb;c.lc=s.clamped_lc;}
    // Unassigned scratch is not a native observation. API-zero stays masked.
  }
}
TL_MATH_HOST_DEVICE inline bool Finite(const NativeRetainedResult& out) {
  if(!tl::math::Finite(out.classification_product)||!tl::math::Finite(out.distance_squared)||
     !g::ValidRow(out.history.row))return false;
  for(const auto& s:out.sector) {
    if((s.defined&PenetrationDefined)&&!tl::math::Finite(s.penetration))return false;
    if((s.defined&DistanceSquaredDefined)&&!tl::math::Finite(s.distance_squared))return false;
    if((s.defined&RawBarycentricDefined)&&(!tl::math::Finite(s.raw_lb)||!tl::math::Finite(s.raw_lc)))return false;
    if((s.defined&ClampedBarycentricDefined)&&(!tl::math::Finite(s.clamped_lb)||!tl::math::Finite(s.clamped_lc)))return false;
  }
  return true;
}
} // namespace tlfea::contact::radioss_type25::selection::detail
