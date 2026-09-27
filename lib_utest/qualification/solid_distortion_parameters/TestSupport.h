// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid_common/distortion/Parameters.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
extern "C" void ic1_native_slots(const double*,double*);
extern "C" void ic1_native_parameter_probe(const double*,const double*,const double*,double*,int*);
namespace distortion_test {
namespace d=tl::fea::solid_common::distortion;
namespace m=tl::material::law42;
struct Case {m::Parameters material;d::Input input;};
inline Case Base(double nu=.463) {
  Case value;
  const auto status=m::Prepare(1.17e6,nu,1100,1e12,value.material);
  EXPECT_EQ(status,m::Status::Ok);
  value.input.density_kg_m3=1100;value.input.material_sound_speed_m_s=720;
  value.input.current_volume_m3=6.4e-5;
  return value;
}
inline std::array<double,6> SlotValues(const m::MechanicalSlots& slots) {
  return {slots.pm20_young_pa,slots.pm21_poisson_ratio,slots.pm22_gs_pa,
          slots.pm32_pa,slots.pm100_reader_bulk_pa,slots.pm107_control_pa};
}
inline std::array<double,6> NativeSlots(const Case& value) {
  const double p[]{value.material.mu_pa,value.material.poisson_ratio,
                   value.material.density_kg_m3,value.material.tension_cutoff_pa};
  std::array<double,6> result{};ic1_native_slots(p,result.data());return result;
}
inline d::Parameters Native(const Case& value) {
  const double p[]{value.material.mu_pa,value.material.poisson_ratio,
                   value.material.density_kg_m3,value.material.tension_cutoff_pa};
  const double kin[]{value.input.density_kg_m3,value.input.material_sound_speed_m_s,
                     value.input.current_volume_m3};
  double values[4]{};int flag=-1;
  ic1_native_parameter_probe(p,value.input.cauchy_stress_pa,kin,values,&flag);
  // FQMAX is not returned by this pre-existing probe. The source test pins
  // its unconditional EP02 assignment; do not call this field observed parity.
  return {values[2],values[1],values[0],values[3],100.0,flag};
}
inline std::array<double,5> Values(const d::Parameters& value) {
  return {value.length_m,value.damping_n_s_m2,value.control_stiffness_n_m,
          value.damping_coefficient,value.quadratic_limit};
}
inline void Compare(const d::Parameters& actual,const d::Parameters& expected) {
  const auto a=Values(actual),b=Values(expected);
  for(unsigned i=0;i<a.size();++i){SCOPED_TRACE(i);EXPECT_NEAR(a[i],b[i],1e-13+2e-12*std::abs(b[i]));}
  EXPECT_EQ(actual.buckling_flag,expected.buckling_flag);
}
inline d::Parameters Check(const Case& value) {
  d::Parameters result;
  EXPECT_EQ(d::PrepareParameters(value.material,value.input,result),d::Status::Success);
  Compare(result,Native(value));return result;
}
inline double NativeC1(const Case& value) {
  const auto slots=NativeSlots(value);
  return std::fmax(slots[3],slots[4])+(((1.0+3.0/10.0)+3.0/100.0)+3.0/1000.0)*slots[2];
}
inline std::vector<Case> Cases() {
  std::vector<Case> result;
  const double native_point4=static_cast<double>(.4f);
  for(double nu:{0.0,.2,.4,native_point4,std::nextafter(native_point4,1.0),.463,d::MaximumPoissonRatio}) {
    auto value=Base(nu);result.push_back(value);
    value.input.cauchy_stress_pa[3]=NativeC1(value)*1e-4;
    result.push_back(value); // Interior F_ES ramp, shared CPU/CUDA coverage.
    value.input.cauchy_stress_pa[3]=0;
    value.input.cauchy_stress_pa[0]=-2.0*NativeC1(value);
    value.input.cauchy_stress_pa[1]=.3*NativeC1(value);
    value.input.cauchy_stress_pa[4]=-.1*NativeC1(value);
    value.input.density_kg_m3=780;value.input.material_sound_speed_m_s=320;
    value.input.current_volume_m3=3.8e-8;result.push_back(value);
  }
  return result;
}
inline d::Parameters Sentinel(){return {11,13,17,19,23,29};}
inline void Same(const d::Parameters& actual,const d::Parameters& expected) {
  EXPECT_EQ(Values(actual),Values(expected));EXPECT_EQ(actual.buckling_flag,expected.buckling_flag);
}
} // namespace distortion_test
