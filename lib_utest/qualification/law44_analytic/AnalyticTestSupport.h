#pragma once
#include "lib_src/materials/TabulatedShellPlasticity.h"
#include <array>
#include <cstring>
#include <gtest/gtest.h>

namespace analytic_test {
namespace mat=tl::material;
using Parameters=mat::TabulatedShellPlasticityParameters;
using History=mat::TabulatedShellPlasticityHistory;
using Input=mat::TabulatedShellPlasticityInput;
using Result=mat::TabulatedShellPlasticityResult;
using Status=mat::TabulatedShellPlasticityStatus;
using Kind=mat::ShellPlasticityHardeningKind;
constexpr mat::TabulatedShellPlasticityRate Rate{true,8000,8,10000};
constexpr double Dt=0x1p-20;
// Literal original source stress triples (N/mm²), independent of production
// prepared B. MID2000357's FAIL0.25 is NOT admitted by this coefficient fixture.
struct Source { double e,sigy,etan,rho; unsigned mid; };
constexpr Source Sources[]{{1000,20,10,1415,2000003},{3000,60,30,1170,2000355},{6000,60,60,1170,2000357}};
inline Parameters Prepare(Source s=Sources[0]) {
  Parameters p;
  EXPECT_EQ(mat::PrepareLinearLaw44ShellPlasticity(s.e*1e6,.3,s.rho,{s.sigy*1e6,s.etan*1e6},Rate,p),Status::Ok);
  return p;
}
inline Input Increment(const Parameters& p) {
  Input in; in.transverse_shear_modulus=(5./6.)*p.shear_modulus;
  in.dt=Dt; in.total_strain_rate_per_s=100;
  in.strain_increment[0]=.05; return in;
}
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& x) {
  std::array<unsigned char,sizeof(T)> result{}; std::memcpy(result.data(),&x,sizeof x); return result;
}
} // namespace analytic_test
