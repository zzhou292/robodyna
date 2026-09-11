// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Values.h"
#include "native/NativeCaller.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>

namespace {
namespace law=tl::material::law42;

TEST(Law42CallerNative, IndependentMaterialViscosityAndSharedEnergyRecurrence) {
  law::Parameters p;
  ASSERT_EQ(law::Prepare(24e6,.463,1980,1e26,p),law::Status::Ok);
  const double parameters[]{p.mu_pa,p.poisson_ratio,p.density_kg_m3,p.tension_cutoff_pa};
  law::CallerHistory history;
  history.density_kg_m3=1980;
  history.internal_energy_density_j_m3=125;
  std::array<double,9> native_history{};
  native_history[6]=1980;native_history[7]=125;
  for(unsigned interval=1;interval<=32;++interval) {
    SCOPED_TRACE(interval);
    const double a=.04*std::sin(double(interval)*.23);
    law::CallerInput input;
    input.displacement_gradient[0]=a;
    input.displacement_gradient[1]=.7*a;
    input.displacement_gradient[4]=-.2*a;
    input.displacement_gradient[5]=-.3*a;
    input.displacement_gradient[8]=.4*a;
    input.engineering_rate_per_s[0]=1e4*a;
    input.engineering_rate_per_s[3]=-.3e4*a;
    input.engineering_rate_per_s[4]=.5e4*a;
    input.dt_s=1e-6;
    input.storage_volume_m3=2e-7;
    input.current_volume_m3=input.storage_volume_m3*(1+a)*(1-.2*a)*(1+.4*a);
    input.characteristic_length_m=.003;
    const double step[]{input.dt_s,input.current_volume_m3,input.storage_volume_m3,
                        input.characteristic_length_m};
    std::array<double,33> native;
    int status=-1;
    law42_solid_caller_native(parameters,native_history.data(),input.displacement_gradient,
                             input.engineering_rate_per_s,step,native.data(),&status);
    ASSERT_EQ(status,0);
    law::CallerResult result;
    ASSERT_EQ(law::UpdateCaller(p,history,input,result),law::Status::Ok);
    const auto actual=law42_caller_test::Values(result);
    ASSERT_EQ(actual.size(),native.size());
    for(unsigned k=0;k<native.size();++k) {
      SCOPED_TRACE(k);
      const double tolerance=3e-10*std::max(std::abs(native[k]),1e-12);
      EXPECT_NEAR(actual[k],native[k],tolerance);
    }
    history=result.history;
    std::copy_n(native.begin(),9,native_history.begin());
  }
}

TEST(Law42CallerNative, NativeInitialSlotIsTwicePrintedShearAndCutoffIsExplicit) {
  const double parameters[]{24e6,.463,1980,1e26};
  double modulus=0;
  law42_solid_initial_modulus_native(parameters,&modulus);
  EXPECT_EQ(modulus,48e6);
  EXPECT_NE(modulus,parameters[0]);
  const double base[]{0,0,0,0,0,0,1980,0,0};
  const double gradient[]{.02,0,0,0,0,0,0,0,0};
  const double rate[6]{};
  const double step[]{1e-6,1.02e-6,1e-6,.01};
  double values[33];int status=-1;
  const double cutoff[]{24e6,.463,1980,1};
  law42_solid_caller_native(cutoff,base,gradient,rate,step,values,&status);
  EXPECT_EQ(status,1);
}
}
