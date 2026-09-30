// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss (C) 2026 Siemens, common CZFINTN1 elastic increment and old work.
// LAW1 uses it directly; layered plastic shells apply the separate native
// yield correction before the existing force/work projection.
#pragma once
#include "QephHistoryData.h"
#include "QephMaterial.h"

namespace tl::fea::qeph::detail {
struct StabilizationWork {
  double increment[6]{}, delta[12]{};
  double bending_factor=0, membrane_loading=0, bending_loading=0;
};
TL_QEPH_HD inline void UpdateStabilization(const GeometryWork& g,const MaterialWork& m,
                                          HistoryValues& h,StabilizationWork& w,double active=1) {
  using namespace force_constant;
  auto* dhg=w.increment; auto* vg=h.stabilization;
  for(unsigned i=0;i<6;++i) dhg[i]=g.values.hourglass_rate[i]*m.dt;
  const double c3=4.*g.values.reciprocal_area;
  const double hxx=c3*g.my34,hyy=c3*g.mx34,hxxk=c3*g.my23,hyyk=c3*g.mx23;
  const double cxx=hxx*dhg[0],cyy=hyy*dhg[1],cxxk=hxxk*dhg[0],cyyk=hyyk*dhg[1];
  const double bxx=hxx*dhg[2],byy=hyy*dhg[3],bxxk=hxxk*dhg[2],byyk=hyyk*dhg[3];
  const double c1m=m.a11*1.,c2m=m.a12*1.;
  w.bending_factor=m.thickness2*one_over_12;
  const double ss1=g.my34*vg[0]+g.my23*vg[6];
  const double ss2=g.mx23*vg[7]+g.mx34*vg[1];
  const double sf1=g.my34*vg[2]+g.my23*vg[8];
  const double sf2=-g.mx23*vg[9]-g.mx34*vg[3];
  const double sc5=g.my34*vg[4]+g.mx34*vg[5];
  const double sc6=g.my23*vg[10]+g.mx23*vg[11];
  const double c5=.5*active*m.thickness*four_over_3;
  const double esx=ss1*dhg[0]+ss2*dhg[1];
  const double old_work0=c5*(esx+.25*(sc5*dhg[4]+sc6*dhg[5]));
  const double emx=(sf1*dhg[2]-sf2*dhg[3])*w.bending_factor;
  w.membrane_loading=esx; w.bending_loading=emx;
  const double old_work1=c5*emx;
  auto* delta=w.delta;
  delta[0]=c1m*cxx-c2m*cyy; delta[1]=c1m*cyy-c2m*cxx;
  delta[2]=c1m*bxx-c2m*byy; delta[3]=c1m*byy-c2m*bxx;
  delta[6]=c1m*cxxk-c2m*cyyk; delta[7]=c1m*cyyk-c2m*cxxk;
  delta[8]=c1m*bxxk-c2m*byyk; delta[9]=c1m*byyk-c2m*bxxk;
  const double c2=1.*m.g*m.shf*one_over_64;
  delta[4]=c2*hxx*dhg[4]; delta[5]=c2*hyy*dhg[4];
  delta[10]=c2*hxxk*dhg[5]; delta[11]=c2*hyyk*dhg[5];
  for(unsigned i=0;i<12;++i) vg[i]=vg[i]+delta[i];
  h.internal_work[0]=h.internal_work[0]+old_work0;
  h.internal_work[1]=h.internal_work[1]+old_work1;
  // The LAW1 caller's SIGY=EP30 disables the source yield block. Plastic callers
  // subsequently correct these twelve staged values with their section data.
}
} // namespace tl::fea::qeph::detail
