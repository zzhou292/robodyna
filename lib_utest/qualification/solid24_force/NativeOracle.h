// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <gtest/gtest.h>
#include "TestSupport.h"
#include "../solid24_reference/JacobianNative.h"
#include "../solid_law42_caller/NativeComparison.h"

extern "C" void heph_force_native(const double*,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,double*,int*);
namespace heph_test {
struct NativeHistory {
  solid24_test::GlobalNativeResult reference;
  std::array<double,24> initial{};
  std::array<double,21> values{};
};
inline NativeHistory InitializeNative(const s::ReferenceInput& input) {
  NativeHistory history;
  history.reference=solid24_test::GlobalNativeRaw(input);
  if(history.reference.status!=0) throw std::runtime_error("native HEPH reference failed");
  for(unsigned n=0;n<8;++n) {
    const auto& x=input.position_m[history.reference.permutation[n]];
    history.initial[3*n]=x.x;history.initial[3*n+1]=x.y;history.initial[3*n+2]=x.z;
  }
  history.values[6]=input.density_kg_m3;
  return history;
}
struct NativeTrial {
  std::array<double,187> values{};
  int status=-1;
};
inline NativeTrial NativeStep(const NativeHistory& history,const s::PrescribedInterval& interval,
                              const s::Material& material) {
  std::array<double,24> x{},v{};
  for(unsigned n=0;n<8;++n) {
    const auto index=history.reference.permutation[n];
    const auto& position=interval.position_m[index];
    const auto& velocity=interval.velocity_m_s[index];
    x[3*n]=position.x;x[3*n+1]=position.y;x[3*n+2]=position.z;
    v[3*n]=velocity.x;v[3*n+1]=velocity.y;v[3*n+2]=velocity.z;
  }
  const double p[]{material.mu_pa,material.poisson_ratio,material.density_kg_m3,
                   material.tension_cutoff_pa};
  const double time[]{interval.base_time_s,interval.dt_s};
  auto jac=history.reference.jacobian();
  const double volume=history.reference.values[33];
  NativeTrial next;
  heph_force_native(p,history.initial.data(),x.data(),v.data(),jac.data(),&volume,
                    history.values.data(),time,next.values.data(),&next.status);
  // Geometry stays in native slot order; only the assembled RHS is source ordered.
  const auto native_force=next.values;
  for(unsigned n=0;n<8;++n) for(unsigned k=0;k<3;++k)
    next.values[22+3*history.reference.permutation[n]+k]=native_force[22+3*n+k];
  return next;
}
inline void AcceptNative(const NativeTrial& trial,NativeHistory& history) {
  if(trial.status!=0)throw std::runtime_error("unavailable native HEPH trial");
  std::copy_n(trial.values.begin(),21,history.values.begin());
}
inline double GroupScale(const std::array<double,187>& x,unsigned first,unsigned last) {
  double scale=0;
  for(unsigned k=first;k<last;++k) scale=std::max(scale,std::abs(x[k]));
  return scale;
}
inline double ForceTolerance(unsigned k,const std::array<double,187>& x) {
  if(k>=151 && k<184) {
    std::array<double,33> material;
    std::copy_n(x.begin()+151,33,material.begin());
    return law42_caller_test::NativeTolerance(k-151,material);
  }
  double scale=std::max(std::abs(x[k]),1e-20);
  if(k<6) scale=std::max(scale,GroupScale(x,0,6));
  if(k>=9 && k<21) scale=std::max(scale,GroupScale(x,9,21));
  if(k>=22 && k<46) scale=std::max(scale,GroupScale(x,22,46));
  if(k>=46 && k<55) scale=1;
  if(k>=55 && k<79) scale=std::max(scale,GroupScale(x,55,79));
  if(k>=79 && k<103) scale=std::max(scale,GroupScale(x,79,103));
  if(k>=105 && k<117) scale=std::max(scale,GroupScale(x,105,117));
  if(k>=117 && k<133) scale=std::max(scale,1.0);
  if(k>=136 && k<145) scale=std::max(scale,GroupScale(x,136,145));
  if(k>=145 && k<151) scale=std::max(scale,GroupScale(x,145,151));
  return 3e-10*scale;
}
inline void Compare(const s::ForceTrial& actual,const NativeTrial& expected) {
  ASSERT_EQ(expected.status,0);
  const auto values=Values(actual);
  ASSERT_EQ(values.size(),expected.values.size());
  for(unsigned k=0;k<values.size();++k) {
    SCOPED_TRACE(k);
    ASSERT_TRUE(std::isfinite(values[k]));ASSERT_TRUE(std::isfinite(expected.values[k]));
    EXPECT_NEAR(values[k],expected.values[k],ForceTolerance(k,expected.values));
  }
}
}
