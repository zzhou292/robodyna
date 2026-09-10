// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss (C) 2026 Siemens, CZFINTN1 NPT=3 plastic yield correction.
#pragma once
#include "QephStabilizationState.h"

namespace tl::fea::qeph::detail {
// Apply after the common elastic increment and before force/work projection.
// FAC is the native section mean/minimum ETSE; no guessed tangent scaling.
// The caller supplies the actual material dispatch's SIGY value explicitly.
TL_QEPH_HD inline bool CorrectPlasticStabilization(const MaterialWork& m,
    double mean_tangent_ratio,double minimum_tangent_ratio,double yield_pa,
    HistoryValues& h,const StabilizationWork& w) noexcept {
  const double factors[]{mean_tangent_ratio,minimum_tangent_ratio};
  for(double x:factors)
    if(!tl::math::Finite(x)||x<0||x>1) return false;
  if(!tl::math::Finite(yield_pa)||yield_pa<=0) return false;
  if(yield_pa>=9.*(1e20*1e9)) return true; // Native ZEP9EP30 sentinel.
  constexpr double tolerance=1./1e18;
  constexpr double coefficient=1.-(1./10.+5./100.);
  constexpr double reduction=1.-1./1000.;
  const double ufac=::fabs(::fmin(mean_tangent_ratio,minimum_tangent_ratio)-1.);
  const double yield2=yield_pa*yield_pa;
  double equivalent2=0,membrane2=0;
  auto* vg=h.stabilization;
  if(ufac<tolerance) {
    const auto* s=h.stress; const auto* b=h.bending_stress;
    membrane2=s[0]*s[0]+s[1]*s[1]-s[0]*s[1]+3.*s[2]*s[2];
    double bending2=b[0]*b[0]+b[1]*b[1]-b[0]*b[1]+3.*b[2]*b[2];
    const double cn=coefficient,cm=coefficient*m.thickness*(1./16.);
    const double nx=cn*vg[0],ny=cn*vg[1],nxk=cn*vg[6],nyk=cn*vg[7];
    const double mx=cm*vg[2],my=cm*vg[3],mxk=cm*vg[8],myk=cm*vg[9];
    membrane2=membrane2+nx*nx+ny*ny-nx*ny;
    bending2=bending2+mx*mx+my*my-mx*my;
    membrane2=membrane2+nxk*nxk+nyk*nyk-nxk*nyk;
    bending2=bending2+mxk*mxk+myk*myk-mxk*myk;
    membrane2=membrane2+::fabs(nx*(2.*nxk-nyk)+ny*(2.*nyk-nxk));
    bending2=bending2+::fabs(mx*(2.*mxk-myk)+my*(2.*myk-mxk));
    equivalent2=membrane2+25.*bending2; // NPT3, not the global LAW1 NPT0 coefficient16.
  }
  if(!tl::math::Finite(yield2)||!tl::math::Finite(equivalent2)) return false;
  if(ufac>=tolerance||equivalent2>yield2) {
    double membrane_reduction=::fmin(membrane2/::fmax(yield2,tolerance),1.);
    membrane_reduction=::fmax(reduction*membrane_reduction,1.-mean_tangent_ratio);
    double bending_reduction=::fmax(reduction,1.-minimum_tangent_ratio);
    if(w.membrane_loading<0) membrane_reduction=0;
    if(w.bending_loading<0) bending_reduction=0;
    const unsigned membrane_indices[]{0,1,6,7},bending_indices[]{2,3,8,9};
    for(unsigned i:membrane_indices) vg[i]=vg[i]-membrane_reduction*w.delta[i];
    for(unsigned i:bending_indices) vg[i]=vg[i]-bending_reduction*w.delta[i];
  }
  for(unsigned i=0;i<12;++i) if(!tl::math::Finite(vg[i])) return false;
  return true;
}
}
