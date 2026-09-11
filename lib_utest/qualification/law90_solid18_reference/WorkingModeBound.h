// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
namespace law90_reference_test {
// Working-unit comparison only. Each higher mode is an unscaled signed sum
// of eight local coordinates, so cancellation error is governed by its input
// coordinates, not by the (possibly zero) resulting mode magnitude.
inline long double HigherModeWorkingBound(
    const std::array<double,ReferenceCount>& a,
    const std::array<double,ReferenceCount>& b,unsigned mode,unsigned axis) {
  // Exact signs of the four S8ZDERIC3 higher modes. Independent long-double
  // sums observe the two already-checked local-coordinate packets.
  constexpr int signs[4][8]{{1,1,-1,-1,-1,-1,1,1},
      {1,-1,-1,1,-1,1,1,-1},{1,-1,1,-1,1,-1,1,-1},
      {-1,1,-1,1,1,-1,1,-1}};
  long double ideal_a=0,ideal_b=0,sum_a=0,sum_b=0;
  for(unsigned n=0;n<8;++n) {
    const long double x=a[9+3*n+axis],y=b[9+3*n+axis];
    ideal_a+=signs[mode][n]*x;ideal_b+=signs[mode][n]*y;
    sum_a+=std::abs(x);sum_b+=std::abs(y);
  }
  constexpr long double u=std::numeric_limits<double>::epsilon()/2;
  constexpr long double ul=std::numeric_limits<long double>::epsilon()/2;
  constexpr long double gamma7=7*u/(1-7*u);
  constexpr long double gamma8l=8*ul/(1-8*ul);
  // A: seven binary64 additions. B: seven additions in mm, each displayed
  // local coordinate multiplied to SI, and a separate final mode SI multiply.
  // |scaled_native_coordinate| <= |displayed_SI_coordinate|/(1-u).
  const long double native_rounding=gamma7*sum_a+(gamma7+u)/(1-u)*sum_b+
      u/(1-u)*std::abs(static_cast<long double>(b[42+3*mode+axis]));
  const long double witness_rounding=gamma8l*(sum_a+sum_b)+
      ul/(1-ul)*(std::abs(ideal_a)+std::abs(ideal_b));
  return std::abs(ideal_a-ideal_b)+native_rounding+witness_rounding;
}
} // namespace law90_reference_test
