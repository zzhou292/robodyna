// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ObserverTypes.h"
#include <cfloat>

namespace tl::fea::qeph::mapped {
TL_SURFACE_HD inline void AddObservation(ObserverSummary& out,unsigned channel,double term) noexcept {
  if(!tl::math::Finite(term)) {out.serial=true;return;}
  out.maximum_term=::fmax(out.maximum_term,::fabs(term));
  out.sum[channel]+=term;
  if(!tl::math::Finite(out.sum[channel])) out.serial=true;
}
// Reuses the existing cache-work expression, including all four slot terms.
// Its default double instantiation still performs the original serial -=.
struct WorkObservation {
  ObserverSummary& out;
  unsigned channel;
  TL_SURFACE_HD void operator-=(double term) noexcept {AddObservation(out,channel,-term);}
};
TL_SURFACE_HD inline void MergeObservations(ObserverSummary& a,const ObserverSummary& b) noexcept {
  a.serial=a.serial||b.serial;
  a.maximum_term=::fmax(a.maximum_term,b.maximum_term);
  for(unsigned c=0;c<ObserverChannels;++c) {
    a.sum[c]+=b.sum[c];
    if(!tl::math::Finite(a.sum[c])) a.serial=true;
  }
  if(b.material_count) {
    a.minimum_area=a.material_count?::fmin(a.minimum_area,b.minimum_area):b.minimum_area;
    a.minimum_thickness=a.material_count?::fmin(a.minimum_thickness,b.minimum_thickness):b.minimum_thickness;
    a.minimum_dt=a.material_count?::fmin(a.minimum_dt,b.minimum_dt):b.minimum_dt;
  }
  a.material_count+=b.material_count;
  a.maximum_displacement=::fmax(a.maximum_displacement,b.maximum_displacement);
  a.maximum_strain=::fmax(a.maximum_strain,b.maximum_strain);
  a.maximum_curvature=::fmax(a.maximum_curvature,b.maximum_curvature);
}
// Every channel has at most four already-rounded terms per parent plus its
// initial seed. Even a serial prefix's absolute sum is then below DBL_MAX/8.
// The factor-eight margin dominates binary64 addition error for the admitted
// <=UINT32_MAX term count, and covers rounded division in this sufficient test.
// Failed proof is NOT rejection: replay the unchanged serial diagnostic path.
TL_SURFACE_HD inline bool FiniteObserverPrefixes(std::size_t parents,double maximum_term) noexcept {
  if(!parents || parents>UINT32_MAX/4 || !tl::math::Finite(maximum_term) || maximum_term<0) return false;
  const double terms=4.0*static_cast<double>(parents)+1;
  return maximum_term<=DBL_MAX/(8.0*terms);
}
} // namespace tl::fea::qeph::mapped
