// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../solid_law42_caller/NativeComparison.h"

namespace heph_test {
inline double GroupScale(const std::array<double,187>& x,unsigned first,unsigned last) {
  double scale=0;
  for(unsigned k=first;k<last;++k) scale=std::max(scale,std::abs(x[k]));
  return scale;
}
inline double ForceTolerance(unsigned k,const std::array<double,187>& x) {
  if(k>=151 && k<184) {
    std::array<double,33> material;
    std::copy_n(x.begin()+151,33,material.begin());
    return law42_caller_test::NativeTolerance(k-151,material);
  }
  double scale=std::max(std::abs(x[k]),1e-20);
  if(k<6) scale=std::max(scale,GroupScale(x,0,6));
  if(k>=9 && k<21) scale=std::max(scale,GroupScale(x,9,21));
  if(k>=22 && k<46) scale=std::max(scale,GroupScale(x,22,46));
  if(k>=46 && k<55) scale=1;
  if(k>=55 && k<79) scale=std::max(scale,GroupScale(x,55,79));
  if(k>=79 && k<103) scale=std::max(scale,GroupScale(x,79,103));
  if(k>=105 && k<117) scale=std::max(scale,GroupScale(x,105,117));
  if(k>=117 && k<133) scale=std::max(scale,1.0);
  if(k>=136 && k<145) scale=std::max(scale,GroupScale(x,136,145));
  if(k>=145 && k<151) scale=std::max(scale,GroupScale(x,145,151));
  return 3e-10*scale;
}
} // namespace heph_test
