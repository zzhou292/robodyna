#pragma once
#include "WallResponseData.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace tl::qualification::qeph::wall_response::detail {
namespace b=contact::q4_bounds;
constexpr double Roundoff=256*std::numeric_limits<double>::epsilon();
inline bool Fail(std::string& error,const char* why) { error=why; return false; }
inline bool Valid(Interval a) { return std::isfinite(a.lower)&&std::isfinite(a.upper)&&a.lower<=a.upper; }
inline Interval Magnitude(Interval a) {
  return {a.lower>=0?a.lower:(a.upper<=0?-a.upper:0),std::max(std::abs(a.lower),std::abs(a.upper))};
}
inline bool Enclose(double value,double error,Interval& out) {
  return std::isfinite(value)&&std::isfinite(error)&&error>=0&&
    b::Add({value,value},{-error,error},&out);
}
inline bool Radius(double value,Interval range,double& out) {
  Interval difference;
  if(!Valid(range)||!b::Add(range,{-value,-value},&difference)) return false;
  out=Magnitude(difference).upper; return std::isfinite(out);
}
inline bool Square(Interval a,Interval& out) {
  const auto x=Magnitude(a);
  return b::MultiplyScalar(x.lower,x.lower,false,&out.lower)&&b::MultiplyScalar(x.upper,x.upper,true,&out.upper);
}
inline bool DivideSigned(Interval a,double positive,Interval& out) {
  if(!Valid(a))return false;
  Interval result,negative,nonnegative;
  if(a.lower>=0) return b::DividePositive(a,positive,&out);
  if(a.upper<=0) {
    if(!b::DividePositive({-a.upper,-a.lower},positive,&negative))return false;
    out={-negative.upper,-negative.lower}; return true;
  }
  if(!b::DividePositive({0,-a.lower},positive,&negative)||!b::DividePositive({0,a.upper},positive,&nonnegative))return false;
  result={-negative.upper,nonnegative.upper}; out=result; return true;
}
inline bool Difference(Interval a,Interval z,double scale,Interval& out) {
  Interval difference;
  return b::Add(a,{-z.upper,-z.lower},&difference)&&b::DividePositive(Magnitude(difference),scale,&out);
}
inline void Maximum(Interval& a,Interval x) { a.lower=std::max(a.lower,x.lower); a.upper=std::max(a.upper,x.upper); }
// Pure numeric reconstruction shared by observation and retained-sample check.
bool Reconstruct(const Model&,const Config&,Sample&,std::string&);
bool Analytic(const Model&,const Config&,Sample&,std::string&);
bool SameObservation(const Sample&,const Sample&) noexcept;
} // namespace tl::qualification::qeph::wall_response::detail
