// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "JacobianSupport.h"
#include "NativeOracle.h"
extern "C" void solid24_reference_global_native(const double*,const double*,double*,int*,int*);
namespace solid24_test {
struct GlobalNativeResult {
  std::array<double,54> values{};
  std::array<int,8> permutation{};
  int status=-1;
  std::array<double,10> jacobian() const {
    std::array<double,10> out{}; std::copy(values.begin()+44,values.end(),out.begin()); return out;
  }
  std::array<double,44> legacy() const {
    std::array<double,44> out{}; std::copy(values.begin(),values.begin()+44,out.begin()); return out;
  }
};
// Native packet arguments are in the supplied working coordinate/density units.
// It independently applies literal EM20; no production floor is passed to it.
inline GlobalNativeResult GlobalNativeRaw(const s::ReferenceInput& input) {
  double x[24];
  for (unsigned n=0;n<8;++n) {
    x[3*n]=input.position_m[n].x; x[3*n+1]=input.position_m[n].y; x[3*n+2]=input.position_m[n].z;
  }
  GlobalNativeResult out;
  solid24_reference_global_native(x,&input.density_kg_m3,out.values.data(),out.permutation.data(),&out.status);
  return out;
}
inline std::array<double,10> WorkingJacobianToSI(std::array<double,10> value,s::WorkingLengthUnit unit) {
  const double scale=unit==s::WorkingLengthUnit::Metre?1.0:.001;
  for (unsigned n=0;n<9;++n) value[n]/=scale;
  value[9]*=scale*scale*scale;
  return value;
}
}  // namespace solid24_test
