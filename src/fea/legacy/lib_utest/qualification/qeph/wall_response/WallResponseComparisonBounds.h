#pragma once
#include "../response/ResponseBounds.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::qeph::wall_response::comparison_detail {
namespace owning=tlfea::contact::q4_bounds;
using Interval=owning::Interval;

// The fields are recorded values with absolute numerical uncertainty. This
// bounds their difference without turning certificate overlap into equality.
inline bool Difference(double a,double a_error,double b,double b_error,
                       double scale,Interval& output) {
  if(!std::isfinite(a_error)||a_error<0||!std::isfinite(b_error)||b_error<0)
    return false;
  Interval center,uncertainty,signed_difference;
  if(!owning::Difference(a,b,&center)||
     !owning::Add({a_error,a_error},{b_error,b_error},&uncertainty)||
     !owning::AddScalar(center.lower,-uncertainty.upper,false,&signed_difference.lower)||
     !owning::AddScalar(center.upper,uncertainty.upper,true,&signed_difference.upper))
    return false;
  const double lo=signed_difference.lower,hi=signed_difference.upper;
  const Interval magnitude{lo>=0?lo:(hi<=0?-hi:0),std::max(std::abs(lo),std::abs(hi))};
  Interval result;
  if(!owning::DividePositive(magnitude,scale,&result)) return false;
  output=result; return true;
}
inline bool Normalize(Interval value,double scale,Interval& output) {
  Interval result;
  if(!owning::DividePositive(value,scale,&result)) return false;
  output=result; return true;
}
inline bool Refines(Interval fine,Interval coarse,double floor) {
  return owning::Nonnegative(fine)&&owning::Nonnegative(coarse)&&
      std::isfinite(floor)&&floor>=0&&
      tl::qualification::qeph::response::bounds::Refines(fine.upper,coarse.lower,floor);
}
} // namespace tl::qualification::qeph::wall_response::comparison_detail
