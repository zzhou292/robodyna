// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
extern "C" void law90_reference_native(const double*,const double*,double*,int*,int*,int*);
extern "C" void law90_current_native(const double*,const double*,const double*,const double*,
                                     const double*,double*,int*);
namespace law90_reference_test {
namespace {
void Coordinates(const s::Vec3 (&in)[8],double (&out)[24]) {
  for(unsigned n=0;n<8;++n) {
    out[3*n]=in[n].x;
    out[3*n+1]=in[n].y;
    out[3*n+2]=in[n].z;
  }
}
}
NativeReference ReferenceOracle(const s::ReferenceInput& input) {
  NativeReference result;
  double x[24];
  Coordinates(input.position_m,x);
  law90_reference_native(x,&input.density_kg_m3,result.values.data(),
      result.source_slot,result.allocation,&result.status);
  return result;
}
NativeCurrent CurrentOracle(const s::ReferenceInput& input,const t::KinematicsInput& now) {
  NativeCurrent result;
  double x0[24],x[24],v[24];
  Coordinates(input.position_m,x0);
  Coordinates(now.position_m,x);
  Coordinates(now.velocity_m_s,v);
  law90_current_native(x0,&input.density_kg_m3,x,v,&now.dt_s,result.values.data(),&result.status);
  return result;
}
}
