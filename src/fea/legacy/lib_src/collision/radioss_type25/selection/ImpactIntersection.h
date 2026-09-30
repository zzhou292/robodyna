// SPDX-License-Identifier: AGPL-3.0-or-later
// Original INTERSECA_25 / INTERSECB_25 / INTERSECV0_25 local arithmetic.
// The complete scalar caller owns current/DT1-backtracked native coordinates.
#pragma once
#include "BoundaryValues.h"
namespace tlfea::contact::radioss_type25::selection::detail {
struct ImpactCrossings {
  int primary = 0, opposite = 0, recontact = 0;
};
TL_MATH_HOST_DEVICE inline Vector InterpolateNative(Vector current,Vector previous,double fraction) {
  return v::Add(current,v::Scale(v::Subtract(previous,current),fraction));
}
TL_MATH_HOST_DEVICE inline bool BarycentricIntersection(Vector first,Vector second,
    Vector center,Vector secondary,double lower,double upper) {
  const auto a=v::Subtract(first,center),b=v::Subtract(second,center);
  const auto point_a=v::Subtract(first,secondary),point_b=v::Subtract(second,secondary);
  const auto point_center=v::Subtract(center,secondary);
  const auto normal=v::Cross(a,b);
  const auto crossed_first=v::Cross(point_center,point_a);
  const auto crossed_second=v::Cross(point_b,point_center);
  const double inverse=1./g::Max(native_constant::em30,v::Dot(normal,normal));
  const double lb=v::Dot(normal,crossed_second)*inverse;
  const double lc=v::Dot(normal,crossed_first)*inverse;
  const double la=1.-lb-lc;
  return lb>=lower&&lb<=upper&&lc>=lower&&lc<=upper&&la>=lower&&la<=upper;
}
TL_MATH_HOST_DEVICE inline bool CrossedAtNativeFraction(const Vector* current,
    const Vector* previous,Vector secondary,Vector old_secondary,
    double volume,double old_volume,unsigned sector,bool triangle) {
  double fraction;
  if(::fabs(old_volume)<g::em20)fraction=1.;
  else if(::fabs(volume)<g::em20)fraction=0.;
  else fraction=volume/(volume-old_volume);
  const auto first=InterpolateNative(current[sector],previous[sector],fraction);
  const unsigned next=(sector+1)%4;
  const auto second=InterpolateNative(current[next],previous[next],fraction);
  const auto center=InterpolateNative(current[4],previous[4],fraction);
  const auto point=InterpolateNative(secondary,old_secondary,fraction);
  const double tolerance=triangle?g::epseg:g::em04;
  return BarycentricIntersection(first,second,center,point,-tolerance,1.+tolerance);
}
TL_MATH_HOST_DEVICE inline ImpactCrossings FindNativeCrossings(const Vector* current,
    const Vector* previous,Vector secondary,Vector old_secondary,
    const double* volume,const double* old_volume,bool triangle,
    bool has_opposite,int initial_contact_flag) {
  ImpactCrossings result;const unsigned count=triangle?1:4;
  for(unsigned i=0;i<count;++i) {
    // Preserve the original product test on two-sided pairs; replacing it by
    // a sign-only test would change the native underflow behavior.
    const bool possible=has_opposite?old_volume[i]*volume[i]<=0.:
        old_volume[i]<=0.&&volume[i]>=0.;
    if(possible&&CrossedAtNativeFraction(current,previous,secondary,old_secondary,
        volume[i],old_volume[i],i,triangle)) {
      if(old_volume[i]<=0.&&volume[i]>=0.)result.primary=1;
      else result.opposite=1;
    }
  }
  if(!has_opposite&&initial_contact_flag>=0&&result.primary==0) {
    for(unsigned i=0;i<count;++i)if(old_volume[i]>0.&&volume[i]>=0.) {
      // The caller passes ZEROM/UNP to INTERSECV0 even for T3: this is the
      // narrow EM4 bound, not INTERSECA/B's relaxed T3 EPS bound.
      if(BarycentricIntersection(previous[i],previous[(i+1)%4],previous[4],
          old_secondary,-g::em04,1.+g::em04))result.primary=1;
    }
    result.recontact=result.primary;
  }
  return result;
}
} // namespace tlfea::contact::radioss_type25::selection::detail
