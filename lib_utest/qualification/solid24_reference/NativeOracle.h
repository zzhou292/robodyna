// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
extern "C" void solid24_reference_native(const double*,const double*,double*,int*,int*);
namespace solid24_test {
struct NativeResult {std::array<double,44> values{};std::array<int,8> permutation{};int status=-1;};
inline NativeResult Native(const s::ReferenceInput& input) {
  std::array<double,24> x{};
  for(unsigned n=0;n<8;++n) {x[3*n]=input.position_m[n].x;x[3*n+1]=input.position_m[n].y;x[3*n+2]=input.position_m[n].z;}
  NativeResult result;
  solid24_reference_native(x.data(),&input.density_kg_m3,result.values.data(),result.permutation.data(),&result.status);
  return result;
}
}  // namespace solid24_test
