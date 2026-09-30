#pragma once
// Qualification-only reuse of the existing binary64 enclosure arithmetic.
// This does not add contact mechanics to the response experiment.
#include "lib_src/collision/Q4ContactBounds.h"
#include <algorithm>
#include <limits>

namespace tl::qualification::qeph::response::bounds {
namespace owning=tlfea::contact::q4_bounds;
using Interval=owning::Interval;
inline bool NormalizedDifference(double a,double b,double scale,Interval& result) {
  Interval signed_value;
  if(!owning::Difference(a,b,&signed_value)) return false;
  const double low=signed_value.lower,high=signed_value.upper;
  const Interval magnitude{low>=0?low:(high<=0?-high:0),std::max(std::abs(low),std::abs(high))};
  return owning::DividePositive(magnitude,scale,&result);
}
inline double RatioUpper(double numerator,double denominator) {
  Interval result;
  return owning::DividePositive({numerator,numerator},denominator,&result)?result.upper:std::numeric_limits<double>::max();
}
inline bool Refines(double fine_upper,double coarse_lower,double floor) {
  double scaled=0,rhs=0;
  return owning::MultiplyScalar(.75,coarse_lower,false,&scaled)&&owning::AddScalar(scaled,floor,false,&rhs)&&fine_upper<=rhs;
}
inline double Product(double a,double b,bool upper) {
  double out=0; return owning::MultiplyScalar(a,b,upper,&out)?out:(upper?std::numeric_limits<double>::max():0.);
}
} // namespace tl::qualification::qeph::response::bounds
