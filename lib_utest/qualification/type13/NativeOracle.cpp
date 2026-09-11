// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
extern "C" {
void type13_native_slope(const double*,const double*,const double*,double*);
void type13_native_mass(const double*,const double*,const double*,double*);
void type13_native_reference(const double*,const double*,double*,int*);
}
namespace type13_test {
double NativeSlope(const t::CurvePoint (&curve)[5],double stiffness,double scale) {
  double values[10];for(unsigned k=0;k<5;++k){values[2*k]=curve[k].x;values[2*k+1]=curve[k].y;}
  double output;type13_native_slope(values,&stiffness,&scale,&output);return output;
}
std::array<double,3> NativeMass(double mass,double inertia,double length) {
  std::array<double,3> output;type13_native_mass(&mass,&inertia,&length,output.data());return output;
}
NativeFrame NativeReference(const t::ReferenceInput& input) {
  double x[9];for(unsigned k=0;k<3;++k){x[3*k]=input.position[k].x;x[3*k+1]=input.position[k].y;x[3*k+2]=input.position[k].z;}
  const double skew[6]={input.skew_x.x,input.skew_x.y,input.skew_x.z,input.skew_y.x,input.skew_y.y,input.skew_y.z};
  double result[4];int branch;type13_native_reference(x,skew,result,&branch);
  return {{result[0],result[1],result[2]},result[3],branch};
}
}
