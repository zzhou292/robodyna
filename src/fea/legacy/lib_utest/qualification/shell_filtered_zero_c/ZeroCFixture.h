#pragma once
#include "lib_src/elements/sections/ShellLayeredJ2.h"
#include <array>
#include <cstring>
#include <gtest/gtest.h>

namespace zero_c_test {
namespace mat = tl::material;
namespace sec = tl::fea::sections;
using Policy = mat::ShellPlasticityRatePolicy;
using Status = mat::TabulatedShellPlasticityStatus;
using Parameters = mat::TabulatedShellPlasticityParameters;
using History = mat::TabulatedShellPlasticityHistory;
using Input = mat::TabulatedShellPlasticityInput;
using Result = mat::TabulatedShellPlasticityResult;
inline constexpr double X[3]{0,.02,1};
inline constexpr double Y[3]{30e6,50e6,200e6};
inline mat::TabulatedShellPlasticityRate Rate() { return {true,0,1,10000,Policy::FilteredZeroC}; }
inline Parameters Prepare(bool analytic) {
  Parameters p;
  const auto status = analytic ? mat::PrepareLinearLaw44ShellPlasticity(
      70e9,.22,2500,{30e6,1e9},Rate(),p) : mat::PrepareTabulatedShellPlasticity(
      70e9,.22,2500,{X,Y,3},Rate(),p);
  EXPECT_EQ(status,Status::Ok);
  return p;
}
inline Input Increment(unsigned step) {
  Input in;
  in.transverse_shear_modulus = 70e9/(2*(1+.22))*5/6;
  in.dt = step == 3 ? .001 : 1.e-7;
  in.total_strain_rate_per_s = step < 6 ? 1300. : 0.;
  in.strain_increment[0] = step < 6 ? .001 : -1.e-5;
  in.strain_increment[1] = -.17*in.strain_increment[0];
  in.strain_increment[2] = .12*in.strain_increment[0];
  in.strain_increment[3] = .05*in.strain_increment[0];
  in.strain_increment[4] = -.03*in.strain_increment[0];
  in.element_active = step < 10;
  return in;
}
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> bytes;
  std::memcpy(bytes.data(),&value,sizeof value);
  return bytes;
}
} // namespace zero_c_test
