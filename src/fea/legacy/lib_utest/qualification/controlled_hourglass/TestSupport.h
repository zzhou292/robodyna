// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid_common/controlled_hourglass/Response.h"
#include <gtest/gtest.h>
#include "WorkComparison.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
extern "C" void controlled_hourglass_native(const double*,const double*,const double*,const double*,const double*,double*);
namespace controlled_test {
namespace c=tl::fea::solid_common::controlled_hourglass;
namespace b=tl::fea::solid_common;
struct Case { c::Input input;c::State state; };
inline Case Base() {
  Case x;x.input.mu_pa=24e6;x.input.poisson_ratio=.463;x.input.density_kg_m3=1980;
  x.input.material_sound_speed_m_s=450;x.input.dt_s=1e-6;
  x.input.current_volume_m3=.02*.03*.04;x.input.reference_volume_m3=x.input.current_volume_m3;
  x.input.raw_stiffness_n_m=3e5;return x;
}
inline Case Moving() {
  auto x=Base();x.input.internal_energy_density_j_m3=124.25;
  for(unsigned n=0;n<8;++n) {
    x.input.local_velocity_m_s[n]={.23*(n+1),.37*(int(n)%3-1),-.19*(int(n)%5-2)};
    x.input.incoming_local_force_n[n]={.12*n,-.27*n,.51*n};
  }
  for(unsigned row=0;row<4;++row)for(unsigned h=0;h<3;++h)
    x.input.projection[row][h]=.013*(int(row)-1)*(int(h)-1)+.007*(row+h);
  for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h)
    x.state.force_n[k][h]=.23*(1+4*k+h)*(k%2?-1:1);
  return x;
}
inline std::array<double,63> Values(const c::Result& r) {
  std::array<double,63> a{};unsigned i=0;
  for(const auto& row:r.proposed_state.force_n)for(double v:row)a[i++]=v;
  for(const auto& v:r.local_force_n){a[i++]=v.x;a[i++]=v.y;a[i++]=v.z;}
  a[i++]=r.internal_energy_density_j_m3;a[i++]=r.raw_stiffness_n_m;a[i++]=r.work_j;
  for(const auto& row:r.modal_velocity_m_s)for(double v:row)a[i++]=v;
  for(const auto& row:r.modal_force_n)for(double v:row)a[i++]=v;
  return a;
}
inline std::array<double,63> Native(const Case& x) {
  const auto& in=x.input;const double p[]{in.mu_pa,in.poisson_ratio,in.density_kg_m3,
    in.material_sound_speed_m_s,in.dt_s,in.current_volume_m3,in.reference_volume_m3,
    in.internal_energy_density_j_m3,in.raw_stiffness_n_m};
  double v[24],f[24],projection[12],history[12];
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k){v[3*n+k]=b::Component(in.local_velocity_m_s[n],k);f[3*n+k]=b::Component(in.incoming_local_force_n[n],k);}
  for(unsigned n=0;n<4;++n)for(unsigned h=0;h<3;++h)projection[3*n+h]=in.projection[n][h];
  for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h)history[4*k+h]=x.state.force_n[k][h];
  std::array<double,63> result{};
  controlled_hourglass_native(p,v,projection,history,f,result.data());return result;
}
inline void Compare(const c::Result& actual,const std::array<double,63>& expected,double dt_s) {
  const auto values=Values(actual);const unsigned boundaries[]{0,12,36,37,38,39,51,63};
  for(unsigned g=0;g<7;++g){double scale=0;for(unsigned i=boundaries[g];i<boundaries[g+1];++i)scale=std::max(scale,std::abs(expected[i]));
    if(g==4){const auto bound=SignedWorkBound(values,expected,dt_s);
      EXPECT_TRUE(SignedWorkMatches(values,expected,dt_s))<<"work error="<<std::abs(values[38]-expected[38])
        <<" measured modal drift bound="<<bound.modal_drift_j<<" arithmetic bound="<<bound.arithmetic_j;
      continue;}
    for(unsigned i=boundaries[g];i<boundaries[g+1];++i){SCOPED_TRACE(i);ASSERT_TRUE(std::isfinite(values[i]));ASSERT_TRUE(std::isfinite(expected[i]));
      EXPECT_LE(std::abs(values[i]-expected[i]),128*std::numeric_limits<double>::epsilon()*scale);}}
}
inline c::Result Check(const Case& x) {
  c::Result result;EXPECT_EQ(c::EvaluateLaw42(x.input,x.state,result),c::Status::Success);
  Compare(result,Native(x),x.input.dt_s);return result;
}
inline void Accept(Case& x,const c::Result& y) {
  x.state=y.proposed_state;x.input.internal_energy_density_j_m3=y.internal_energy_density_j_m3;
}
inline void AcceptNative(Case& x,const std::array<double,63>& y) {
  for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h)x.state.force_n[k][h]=y[4*k+h];
  x.input.internal_energy_density_j_m3=y[36];
}
inline std::vector<Case> BasisCases() {
  std::vector<Case> cases{Base(),Moving()};
  for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h){auto x=Base();x.state.force_n[k][h]=1.25;cases.push_back(x);}
  for(unsigned n=0;n<4;++n)for(unsigned h=0;h<3;++h){auto x=Moving();for(auto& row:x.input.projection)for(double& p:row)p=0;x.input.projection[n][h]=.19;cases.push_back(x);}
  return cases;
}
} // namespace controlled_test
