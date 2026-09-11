// SPDX-License-Identifier: MIT
#include "NativeOracle.h"
extern "C" {
void qeph_q1_startup(const double*,const double*,double*);
void qbat_native_coefficients(const double*,const double*,const double*,const double*,double*);
void qbat_native_geometry(const double*,double*,int*);
}
namespace qbat_test {
std::array<double,40> NativeReference(const qb::ReferenceInput& input) {
  std::array<double,40> result{};
  const auto& q=input.quadrilateral;
  double position[12];
  for(unsigned i=0;i<4;++i) {
    position[3*i]=q.position[i].x;
    position[3*i+1]=q.position[i].y;
    position[3*i+2]=q.position[i].z;
  }
  const double mass_input[]{q.density,q.thickness,q.young_modulus};
  qeph_q1_startup(position,mass_input,result.data());
  const double material[]{q.density,q.young_modulus,q.poisson_ratio,q.thickness,input.initial_a11_pa};
  const double viscosity[]{input.options.membrane_viscosity,input.options.numerical_viscosity};
  qbat_native_coefficients(result.data()+18,result.data()+9,material,viscosity,result.data()+34);
  return result;
}
std::array<double,84> NativeGeometry(const qb::CurrentInput& input,int& flat) {
  std::array<double,84> result{};
  double position[12];
  for(unsigned i=0;i<4;++i) {
    position[3*i]=input.position_m[i].x;
    position[3*i+1]=input.position_m[i].y;
    position[3*i+2]=input.position_m[i].z;
  }
  qbat_native_geometry(position,result.data(),&flat);
  return result;
}
} // namespace qbat_test
