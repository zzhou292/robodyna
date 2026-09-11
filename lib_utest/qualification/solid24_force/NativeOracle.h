// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <gtest/gtest.h>
#include "TestSupport.h"
#include "../solid24_reference/JacobianNative.h"
#include "NativeComparison.h"

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
