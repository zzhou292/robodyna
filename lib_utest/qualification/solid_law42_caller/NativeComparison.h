// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace law42_caller_test {
inline double NativeTolerance(unsigned field,const std::array<double,33>& expected) {
  double scale=std::max(std::abs(expected[field]),1e-12);
  if(field<6 || (field>=9 && field<17)) {
    // Spectral stress rotation sums signed principal-stress contributions.
    // A zero shear component therefore uses the stress tensor's norm, as in
    // the qualified point comparison, rather than its cancellation residual.
    for(unsigned k=0;k<6;++k) scale=std::max(scale,std::abs(expected[k]));
    scale=std::max({scale,std::abs(expected[15]),std::abs(expected[16])});
  }
  return 3e-10*scale;
}
inline bool NativeValuesAgree(const std::array<double,33>& actual,
                              const std::array<double,33>& expected) {
  for(unsigned k=0;k<33;++k)
    if(!std::isfinite(actual[k]) || !std::isfinite(expected[k]) ||
       std::abs(actual[k]-expected[k])>NativeTolerance(k,expected)) return false;
  return true;
}
}
