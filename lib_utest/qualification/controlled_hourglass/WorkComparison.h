// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <array>
#include <cmath>
#include <limits>
namespace controlled_test {
// Test-only forward-error envelope; production keeps its native signed sum.
struct WorkErrorBound {
  long double modal_drift_j=0,arithmetic_j=0,total_j=0;
};
inline long double Upper(long double value) {
  return value==0?0:std::nextafter(value,std::numeric_limits<long double>::infinity());
}
inline long double UpperAdd(long double a,long double b) {return Upper(a+b);}
inline long double UpperProduct(long double a,long double b) {
  return a==0||b==0?0:std::nextafter(a*b,std::numeric_limits<long double>::infinity());
}
inline WorkErrorBound SignedWorkBound(const std::array<double,63>& actual,
    const std::array<double,63>& native,double dt_s) {
  static_assert(std::numeric_limits<double>::is_iec559&&std::numeric_limits<double>::digits==53);
  static_assert(std::numeric_limits<long double>::digits>std::numeric_limits<double>::digits);
  static_assert(std::numeric_limits<long double>::max_exponent>=2*std::numeric_limits<double>::max_exponent);
  static_assert(std::numeric_limits<long double>::min_exponent<=2*std::numeric_limits<double>::min_exponent);
  long double drift=0,products=0;
  for(unsigned k:{2u,0u,1u})for(unsigned h=0;h<4;++h) {
    const unsigned i=4*k+h;
    const long double fa=actual[51+i],fn=native[51+i],ra=actual[39+i],rn=native[39+i];
    const long double df=Upper(std::abs(fa-fn)),dr=Upper(std::abs(ra-rn));
    const long double term=UpperAdd(UpperAdd(UpperProduct(std::abs(fn),dr),
      UpperProduct(std::abs(rn),df)),UpperProduct(df,dr));
    drift=UpperAdd(drift,term);
    products=UpperAdd(products,UpperAdd(UpperProduct(std::abs(fa),std::abs(ra)),
      UpperProduct(std::abs(fn),std::abs(rn))));
  }
  // A term traverses one product, at most eleven adds, and one dt product.
  // 13*u and 1-13*u are exactly representable; only the quotient needs rounding up.
  constexpr long double u=std::numeric_limits<double>::epsilon()/2;
  const long double gamma13=Upper((13*u)/(1-13*u)),dt=std::abs(static_cast<long double>(dt_s));
  WorkErrorBound bound;bound.modal_drift_j=UpperProduct(dt,drift);
  bound.arithmetic_j=UpperProduct(UpperProduct(gamma13,dt),products);
  bound.total_j=UpperAdd(bound.modal_drift_j,bound.arithmetic_j);return bound;
}
inline bool NormalOrZero(double value) {return value==0||std::isnormal(value);}
inline bool NativeOrderedWork(const std::array<double,63>& values,double dt,double& work) {
  if(!NormalOrZero(dt))return false;
  double power=0;bool first=true;
  for(unsigned k:{2u,0u,1u})for(unsigned h=0;h<4;++h) {
    const unsigned i=4*k+h;const double force=values[51+i],rate=values[39+i];
    if(!NormalOrZero(force)||!NormalOrZero(rate))return false;
    const double term=force*rate;
    if(!NormalOrZero(term)||(term==0&&force!=0&&rate!=0))return false;
    power=first?term:power+term;first=false;if(!NormalOrZero(power))return false;
  }
  work=dt*power;
  return NormalOrZero(work)&&!(work==0&&dt!=0&&power!=0);
}
inline bool SignedWorkMatches(const std::array<double,63>& actual,
    const std::array<double,63>& native,double dt) {
  double actual_replay=0,native_replay=0;
  if(!NativeOrderedWork(actual,dt,actual_replay)||!NativeOrderedWork(native,dt,native_replay)||
     actual[38]!=actual_replay||native[38]!=native_replay)return false;
  const auto bound=SignedWorkBound(actual,native,dt);
  // Round the measured work discrepancy upwards too; a borderline result fails safely.
  const long double error=Upper(std::abs(static_cast<long double>(actual[38])-native[38]));
  return std::isfinite(bound.total_j)&&error<=bound.total_j;
}
} // namespace controlled_test
