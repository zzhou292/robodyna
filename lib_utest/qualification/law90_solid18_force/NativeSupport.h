// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include "native/NativeForce.h"
#include "lib_utest/qualification/law90_point/NativeSupport.h"
#include <algorithm>
#include <gtest/gtest.h>
#include <iomanip>
namespace law90_force_test {
struct NativeCaller {
  std::array<double,37> values{};
  double history[20]{};
  int cursor[3]{},status=-1;
  explicit NativeCaller(double density){history[16]=density;}
};
inline void AdvanceNative(const double* prepared,law::CurveView curve,
    const law::CallerInput& input,NativeCaller& state) {
  const int n=static_cast<int>(curve.count);
  const double step[5]{input.endpoint_time_s,input.dt_s,input.current_volume_m3,
    input.storage_volume_m3,input.characteristic_length_m};
  int next[3]{};
  law90_solid_caller_native(prepared,curve.compression_strain,curve.stress_pa,&n,
      state.history,state.cursor,input.selected_b_minus_identity,input.engineering_rate_per_s,
      step,state.values.data(),next,&state.status);
  if(!state.status){std::copy_n(state.values.data(),20,state.history);std::copy_n(next,3,state.cursor);}
}
struct NativeForce {
  std::array<double,357> values{};
  double history[160]{};
  int cursor[24]{},status=-1;
  explicit NativeForce(double density){for(unsigned ip=0;ip<8;++ip)history[20*ip+16]=density;}
};
inline void AdvanceNative(const double* prepared,law::CurveView curve,const s::ReferenceInput& input,
    const s::PrescribedInterval& interval,bool initial,NativeForce& state) {
  double x0[24],x[24],v[24];
  for(unsigned i=0;i<8;++i) {
    const auto a=input.position_m[i],b=interval.position_endpoint_m[i],c=interval.velocity_midpoint_m_s[i];
    x0[3*i]=a.x;x0[3*i+1]=a.y;x0[3*i+2]=a.z;
    x[3*i]=b.x;x[3*i+1]=b.y;x[3*i+2]=b.z;
    v[3*i]=c.x;v[3*i+1]=c.y;v[3*i+2]=c.z;
  }
  const double step[2]{initial ? 0 : interval.base_time_s+interval.dt_s,initial ? 0 : interval.dt_s};
  const int n=static_cast<int>(curve.count);int next[24]{};
  law90_solid_force_native(prepared,curve.compression_strain,curve.stress_pa,&n,x0,
    &input.density_kg_m3,x,v,step,state.history,state.cursor,state.values.data(),next,&state.status);
  if(!state.status){
    for(unsigned ip=0;ip<8;++ip)std::copy_n(state.values.data()+40*ip,20,state.history+20*ip);
    std::copy_n(next,24,state.cursor);
  }
}
inline std::array<double,357> ForceValues(const f::ForceTrial& trial) {
  std::array<double,357> out{};
  for(unsigned ip=0;ip<8;++ip) {
    const auto& p=trial.point[ip];const auto caller=CallerValues(p.material);
    std::copy(caller.begin(),caller.end(),out.begin()+40*ip);
    out[40*ip+37]=p.current_volume_m3;out[40*ip+38]=p.storage_volume_m3;
    out[40*ip+39]=p.characteristic_length_m;
  }
  const auto& h=trial.proposed_history.data().global;
  std::copy_n(h.stress_pa,6,out.data()+320);
  out[326]=h.density_kg_m3;out[327]=h.internal_energy_density_j_m3;
  out[328]=h.bulk_pressure_pa;out[329]=h.scalar_rate_per_s;
  out[330]=trial.diagnostics.minimum_unscaled_dt_s;out[331]=trial.diagnostics.raw_stiffness_n_m;
  out[332]=trial.diagnostics.internal_work_increment_j;
  for(unsigned n=0;n<8;++n){const auto p=trial.rhs_force_n[n];out[333+3*n]=p.x;out[334+3*n]=p.y;out[335+3*n]=p.z;}
  return out;
}
// Same-unit comparisons preserve dimensions: each stress/energy tensor has its
// own norm; dimensionless histories, volume, length and time retain scalar scales.
inline double CallerScale(const double* expected,unsigned k) {
  double scale=std::max(1.,std::abs(expected[k]));
  if(k==0 || (k>=10&&k<16) || (k>=20&&k<26)) {
    scale=std::max(scale,std::abs(expected[0]));
    for(unsigned j=10;j<16;++j)scale=std::max(scale,std::abs(expected[j]));
  }
  if(k==1||k==3||k==8)for(unsigned j:{1u,3u,8u})scale=std::max(scale,std::abs(expected[j]));
  // These SI values are small by dimension, not cancellation-sensitive tensors.
  if(k==31||k==32)scale=std::max(1e-20,std::abs(expected[32]));
  if(k==35)scale=std::max(1e-20,std::abs(expected[k]));
  return scale;
}
inline ::testing::AssertionResult CheckField(double actual,double expected,double scale,unsigned index) {
  const double bound=2e-10*scale;
  if(!std::isfinite(actual)||!std::isfinite(expected)||std::abs(actual-expected)>bound)
    return ::testing::AssertionFailure()<<"index "<<index<<std::setprecision(17)
      <<" actual "<<actual<<" native "<<expected<<" difference "<<actual-expected<<" bound "<<bound;
  return ::testing::AssertionSuccess();
}
inline ::testing::AssertionResult CallerAgreement(const std::array<double,37>& actual,const double* expected) {
  for(unsigned k=0;k<37;++k){const auto result=CheckField(actual[k],expected[k],CallerScale(expected,k),k);if(!result)return result;}
  return ::testing::AssertionSuccess();
}
inline ::testing::AssertionResult ForceAgreement(const std::array<double,357>& actual,const NativeForce& native) {
  for(unsigned ip=0;ip<8;++ip)for(unsigned j=0;j<40;++j) {
    const unsigned k=40*ip+j;const auto* point=native.values.data()+40*ip;
    const double scale=j<37 ? CallerScale(point,j) : std::max(1e-20,std::abs(point[j]));
    const auto result=CheckField(actual[k],point[j],scale,k);if(!result)return result;
  }
  double stress=1,force=1;
  for(unsigned k=320;k<326;++k)stress=std::max(stress,std::abs(native.values[k]));
  for(unsigned k=333;k<357;++k)force=std::max(force,std::abs(native.values[k]));
  for(unsigned k=320;k<357;++k) {
    double scale=std::max(1.,std::abs(native.values[k]));
    if(k<326)scale=stress;if(k==330)scale=std::max(1e-20,std::abs(native.values[k]));if(k>=333)scale=force;
    const auto result=CheckField(actual[k],native.values[k],scale,k);if(!result)return result;
  }
  return ::testing::AssertionSuccess();
}
inline void CheckCursors(const f::ForceTrial& trial,const NativeForce& native) {
  for(unsigned ip=0;ip<8;++ip)for(unsigned j=0;j<3;++j)
    ASSERT_EQ(trial.proposed_history.data().point[ip].point.cursor[j],static_cast<unsigned>(native.cursor[3*ip+j]))<<ip<<':'<<j;
}
}
