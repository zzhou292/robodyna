// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
extern "C" void solid6z_reference_native(const double*,const double*,const int*,double*,int*,int*);
namespace solid6z_test {
struct NativeResult {
  std::array<double,47> values{};
  std::array<int,6> permutation{};
  int status=-1;
};
inline NativeResult Native(const s::ReferenceInput& input,int imas=0) {
  std::array<double,18> x{};
  for (unsigned n=0;n<6;++n) {
    x[3*n]=input.position_m[n].x;
    x[3*n+1]=input.position_m[n].y;
    x[3*n+2]=input.position_m[n].z;
  }
  NativeResult result;
  solid6z_reference_native(x.data(),&input.density_kg_m3,&imas,
                           result.values.data(),result.permutation.data(),&result.status);
  return result;
}
}  // namespace solid6z_test
