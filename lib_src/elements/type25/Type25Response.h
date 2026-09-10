// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Complete donor hashes: lib_utest/qualification/type25/native/source-manifest.json.
// GNU AGPL v3 or later; see LICENSE.md alongside.
#pragma once
#include "Type25Deformation.h"

#if defined(__CUDACC__)
#define TL_TYPE25_HD __host__ __device__
#else
#define TL_TYPE25_HD
#endif

namespace tl::fea::type25::detail {
struct Radial {
  double magnitude=0,sine=0,cosine=1;
};
TL_TYPE25_HD inline Radial Radius(double y,double z) {
  Radial r;r.magnitude=::sqrt(y*y+z*z);
  if(r.magnitude>0){r.sine=y/r.magnitude;r.cosine=z/r.magnitude;}
  return r;
}
TL_TYPE25_HD inline Status Response(const Property& p,const History& accepted,double dt,History& next) {
  const auto shear=Radius(next.displacement_m.y,next.displacement_m.z);
  const auto bend=Radius(next.rotation_rad.y,next.rotation_rad.z);
  const double x[4]={next.displacement_m.x,shear.magnitude,next.rotation_rad.x,bend.magnitude};
  const double oldx[4]={accepted.displacement_m.x,Radius(accepted.displacement_m.y,accepted.displacement_m.z).magnitude,
      accepted.rotation_rad.x,Radius(accepted.rotation_rad.y,accepted.rotation_rad.z).magnitude};
  const double oldf[4]={accepted.local_force_N.x,Radius(accepted.local_force_N.y,accepted.local_force_N.z).magnitude,
      accepted.local_couple_Nm.x,Radius(accepted.local_couple_Nm.y,accepted.local_couple_Nm.z).magnitude};
  double force[4]{},criterion=0;
  for(unsigned i=0;i<4;++i) {
    const double velocity=(x[i]-oldx[i])/dt;
    if(!tl::math::fixed3::Finite(velocity)||!tl::math::fixed3::Finite(oldf[i]))return Status::NonfiniteResult;
    // REDEF3: no curves, A=1/B=E=0/GF3=1, explicit dynamics. Keep its
    // final +0 and OFF multiplication; work uses the scalar radial channel.
    force[i]=(p.stiffness[i]*x[i]+p.damping[i]*velocity+0)*(accepted.active?1:0);
    next.internal_work_J[i]=accepted.internal_work_J[i]+(x[i]-oldx[i])*(force[i]+oldf[i])*.5;
    if(!tl::math::fixed3::Finite(force[i])||!tl::math::fixed3::Finite(next.internal_work_J[i]))return Status::NonfiniteResult;
    if(accepted.active) {
      const double limit=force[i]>0?p.failure_positive[i]:p.failure_negative[i];
      const double ratio=force[i]/limit;
      const double contribution=p.failure_weight[i]*::pow(ratio,p.failure_exponent[i]);
      if(!Nonnegative(contribution))return Status::NonfiniteResult;
      criterion=criterion+contribution;
      if(!Nonnegative(criterion))return Status::NonfiniteResult;
    }
  }
  next.local_force_N={force[0],force[1]*shear.sine,force[1]*shear.cosine};
  next.local_couple_Nm={force[2],force[3]*bend.sine,force[3]*bend.cosine};
  next.failure_criterion=accepted.failure_criterion<1?::fmin(criterion,1):1;
  next.active=accepted.active;
  // Native coupled failure happens AFTER this evaluation's forces/work. The
  // newly failed force cache is retained; OFF zeros the following evaluation.
  if(accepted.active&&criterion>1){next.active=false;next.failure_criterion=1;}
  return ValidHistory(next)?Status::Success:Status::NonfiniteResult;
}
} // namespace tl::fea::type25::detail

#undef TL_TYPE25_HD
