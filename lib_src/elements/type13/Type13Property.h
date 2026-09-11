// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss-derived RKINI3; exact donor inventory in qualification/type13/native.
#pragma once
#include "Type13Units.h"

namespace tl::fea::type13 {
// Pure staged initialization. The selected resolved branch requires four
// five-point curves, A=LSCALE=1, zero damping, H=1, Ileng=1, Ifail=1,
// Ifail2=0, and no sensor/rate-dependent failure. Failure is not evaluated here.
TL_TYPE13_HD inline Status InitializeProperty(const PropertyInput& input,Property& output) {
  detail::UnitFactors units;
  if(!detail::ResolveUnits(input.units,units)||!detail::Positive(input.mass_per_length)||
     !detail::Nonnegative(input.inertia_per_length))return Status::InvalidInput;
  const auto& controls=input.controls;
  if(controls.length_normalized!=1||controls.coupled_failure!=1||controls.force_failure!=0||
     controls.sensor!=0||controls.rate_failure!=0)return Status::UnsupportedScope;
  // Reject all counts before borrowing any curve bytes.
  for(unsigned i=0;i<CurveCount;++i)
    if(input.curves[i].count!=CurvePoints||!input.curves[i].points)return Status::InvalidInput;
  Property next;next.units_=input.units;next.mass_per_length_=input.mass_per_length;
  // Native hm_read_prop13 XIN floor is in the declared working units.
  next.inertia_per_length_=input.inertia_per_length<=1e-20?1e-20:input.inertia_per_length;
  next.added_inertia_per_length_=next.inertia_per_length_-input.inertia_per_length;
  for(unsigned c=0;c<CurveCount;++c) {
    for(unsigned k=0;k<CurvePoints;++k) {
      const auto p=input.curves[c].points[k];
      if(!tl::math::fixed3::Finite(p.x)||!tl::math::fixed3::Finite(p.y)||
         (k&&!(p.x>next.curves_[c].points[k-1].x)))return Status::InvalidInput;
      next.curves_[c].points[k]=p;
    }
  }
  bool used[CurveCount]{};
  for(unsigned c=0;c<ChannelCount;++c) {
    const auto& in=input.channels[c];
    if(in.curve_index>=CurveCount||!detail::Positive(in.stiffness)||
       !tl::math::fixed3::Finite(in.failure_negative)||in.failure_negative>=0||
       !detail::Positive(in.failure_positive)||!detail::Positive(in.failure_weight)||
       !detail::Positive(in.failure_exponent))return Status::InvalidInput;
    if(in.ordinate_scale!=1||in.abscissa_scale!=1||in.damping!=0||in.hysteresis!=1)
      return Status::UnsupportedScope;
    used[in.curve_index]=true;
    auto& channel=next.channels_[c];channel.declaration=in;
    double stiffness=in.stiffness;
    const auto& curve=next.curves_[in.curve_index];
    // RKINI3: preserve source point order and LSCALE*(B2-B1)/(A2-A1).
    for(unsigned k=1;k<CurvePoints;++k) {
      if(!tl::math::fixed3::Finite(curve.points[k].y-curve.points[k-1].y)||
         !tl::math::fixed3::Finite(curve.points[k].x-curve.points[k-1].x))return Status::NonfiniteResult;
      const double slope=in.abscissa_scale*(curve.points[k].y-curve.points[k-1].y)/
                         (curve.points[k].x-curve.points[k-1].x);
      if(!tl::math::fixed3::Finite(slope))return Status::NonfiniteResult;
      stiffness=::fmax(stiffness,slope);
    }
    channel.native_stiffness=stiffness;
    channel.stiffness_si=stiffness*(c<3?units.force:units.rotation_stiffness);
    if(!detail::Positive(channel.stiffness_si))return Status::NonfiniteResult;
  }
  for(unsigned c=0;c<CurveCount;++c)if(!used[c])return Status::InvalidInput;
  next.initialized_=true;output=next;return Status::Success;
}
} // namespace tl::fea::type13
