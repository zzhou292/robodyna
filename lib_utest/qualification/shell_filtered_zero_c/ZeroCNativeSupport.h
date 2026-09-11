#pragma once
#include "ZeroCFixture.h"
#include "lib_utest/qualification/native/law44/NativePoint.h"
#include <algorithm>
#include <cmath>

extern "C" void filtered_zero_c_point(int,int,const double*,double,double,double,double,
    double,double,const double*,const double*,const double*,double,double,double,double,double*,double*);
namespace zero_c_test {
namespace native = tl::qualification::law44;
struct NativePacket { std::array<double,13> values{}; double hardening=0; };
inline NativePacket NativeAdvance(bool analytic,const History& accepted,const Input& in,
    double time,double layer=1.,double thickness=1.) {
  const std::array<double,8> curve{0,0,X[0],Y[0],X[1],Y[1],X[2],Y[2]};
  std::array<double,6> base;
  std::copy_n(accepted.stress,5,base.begin());
  base[5]=accepted.plastic_strain;
  const std::array<double,5> rate{0.,1.,in.total_strain_rate_per_s,
      native::NativeFilterCoefficient(10000,in.dt),accepted.filtered_rate_per_s};
  NativePacket result;
  filtered_zero_c_point(analytic?1:0,3,curve.data(),70e9,.22,2500,in.transverse_shear_modulus,
      analytic?30e6:0.,analytic?1e9:0.,base.data(),in.strain_increment,rate.data(),layer,
      thickness,in.element_active?1.:0.,time,result.values.data(),&result.hardening);
  return result;
}
inline History NativeHistory(const NativePacket& packet) {
  History h;
  std::copy_n(packet.values.begin(),5,h.stress);
  h.plastic_strain=packet.values[5];
  h.filtered_rate_per_s=packet.values[12];
  return h;
}
inline void Near(double actual,double expected) {
  EXPECT_TRUE(std::isfinite(actual));
  EXPECT_TRUE(std::isfinite(expected));
  EXPECT_NEAR(actual,expected,2.e-12*std::max(1.,std::abs(expected)));
}
inline void Compare(const Result& actual,const NativePacket& expected,const History& before,
    double layer=1.,double thickness=1.) {
  for (unsigned i=0;i<5;++i) Near(actual.history.stress[i],expected.values[i]);
  Near(actual.history.plastic_strain,expected.values[5]);
  Near(actual.plastic_increment,expected.values[6]);
  Near(actual.tangent_ratio,expected.values[7]);
  Near(actual.yield_before_pa,expected.values[9]);
  Near(actual.history.filtered_rate_per_s,expected.values[12]);
  const double actual_thickness=(thickness+actual.elastic_thickness_strain*layer)+
      actual.plastic_thickness_strain*layer;
  EXPECT_NEAR(actual_thickness,expected.values[8],2.e-12*std::abs(expected.values[8]));
  const auto vm=[](const double* stress) {
    const long double x=stress[0],y=stress[1],xy=stress[2];
    return std::sqrt(x*x+y*y-x*y+3*xy*xy);
  };
  Near(actual.equivalent_stress_pa,static_cast<double>(vm(expected.values.data())));
  const double work=static_cast<double>(.5L*(vm(before.stress)+vm(expected.values.data()))*
      (expected.values[5]-before.plastic_strain));
  Near(actual.plastic_work_density,work);
}
} // namespace zero_c_test
