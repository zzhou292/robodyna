// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid_common/distortion/UnitResponse.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
extern "C" void ic1_native_parameter_probe(const double*,const double*,const double*,double*,int*);
extern "C" void distortion_force_native(int,const double*,const int*,const double*,const double*,const double*,const double*,double*,int*);
namespace distortion_force_test {
namespace d=tl::fea::solid_common::distortion;
namespace b=tl::fea::solid_common;
namespace m=tl::material::law42;
struct Case {m::Parameters material;d::Input parameters;d::ForceInput input;d::UnitScale units{1,1,1};};
inline Case Base(bool buckled=false) {
  Case c;EXPECT_EQ(m::Prepare(24e6,.463,1980,1e26,c.material),m::Status::Ok);
  const b::Vec3 x[]{{0,0,0},{.02,0,0},{.02,.03,0},{0,.03,0},{0,0,.04},{.02,0,.04},{.02,.03,.04},{0,.03,.04}};
  for(unsigned n=0;n<8;++n){c.input.position_m[n]=x[n];c.input.incoming_force_n[n]={.1*n,-.2*n,.3*n};}
  c.input.dt_s=2e-7;c.input.raw_stiffness_n_m=200;c.input.distortion_energy_j=1.25;
  c.parameters.density_kg_m3=1980;c.parameters.material_sound_speed_m_s=650;
  if(buckled)c.parameters.cauchy_stress_pa[0]=-1e10;
  c.parameters.current_volume_m3=b::SignedCenterVolume(c.input.position_m);return c;
}
inline d::PreparedForceValues Prepare(Case c) {
  c.parameters.current_volume_m3=b::SignedCenterVolume(c.input.position_m);
  d::PreparedForceValues out;
  EXPECT_EQ(d::PrepareForceValues(c.material,c.parameters,c.input,c.units,out),d::Status::Success);return out;
}
inline std::array<double,27> Values(const d::ForceResult& r) {
  std::array<double,27> out{};
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k)out[3*n+k]=b::Component(r.force_n[n],k);
  out[24]=r.raw_stiffness_n_m;out[25]=r.distortion_energy_j;out[26]=r.distortion_work_increment_j;return out;
}
inline std::vector<d::ForceResult> Native(const std::vector<Case>& cases) {
  EXPECT_LE(cases.size(),2u);EXPECT_GT(cases.size(),0u);
  double p[10]{},x[48]{},v[48]{},base[52]{},result[52]{},zero_result[52]{};int flags[2]{},contacts[2]{};
  double force_units[2]{},energy_units[2]{},stiffness_units[2]{};
  for(unsigned row=0;row<cases.size();++row) {
    const auto& c=cases[row];const double length=c.units.length_m,mass=c.units.mass_kg,time=c.units.time_s;
    const double volume=length*length*length,pressure=mass/(length*time*time);
    force_units[row]=mass*length/(time*time);energy_units[row]=force_units[row]*length;stiffness_units[row]=mass/(time*time);
    const double material[]{c.material.mu_pa/pressure,c.material.poisson_ratio,c.material.density_kg_m3/(mass/volume),c.material.tension_cutoff_pa/pressure};
    double stress[6];for(unsigned k=0;k<6;++k)stress[k]=c.parameters.cauchy_stress_pa[k]/pressure;
    const double kin[]{c.parameters.density_kg_m3/(mass/volume),c.parameters.material_sound_speed_m_s/(length/time),b::SignedCenterVolume(c.input.position_m)/volume};
    ic1_native_parameter_probe(material,stress,kin,p+5*row,flags+row);p[5*row+4]=100;
    for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k) {
      x[24*row+3*n+k]=b::Component(c.input.position_m[n],k)/length;
      v[24*row+3*n+k]=b::Component(c.input.velocity_m_s[n],k)/(length/time);
      base[26*row+3*n+k]=b::Component(c.input.incoming_force_n[n],k)/force_units[row];
    }
    base[26*row+24]=c.input.raw_stiffness_n_m/stiffness_units[row];base[26*row+25]=c.input.distortion_energy_j/energy_units[row];
  }
  const double step=cases[0].input.dt_s/cases[0].units.time_s;
  distortion_force_native(static_cast<int>(cases.size()),p,flags,x,v,base,&step,result,contacts);
  for(unsigned row=0;row<cases.size();++row)base[26*row+25]=0;
  int ignored[2];distortion_force_native(static_cast<int>(cases.size()),p,flags,x,v,base,&step,zero_result,ignored);
  std::vector<d::ForceResult> out(cases.size());
  for(unsigned row=0;row<cases.size();++row) {
    auto& r=out[row];for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k)
      b::SetComponent(r.force_n[n],k,result[26*row+3*n+k]*force_units[row]);
    r.raw_stiffness_n_m=result[26*row+24]*stiffness_units[row];r.distortion_energy_j=result[26*row+25]*energy_units[row];
    r.distortion_work_increment_j=zero_result[26*row+25]*energy_units[row];
    r.center_contacts=contacts[0];r.corner_contacts=contacts[1];
  }
  return out;
}
inline void Compare(const d::ForceResult& actual,const d::ForceResult& expected) {
  const auto a=Values(actual),e=Values(expected);
  for(unsigned k=0;k<a.size();++k){SCOPED_TRACE(k);ASSERT_TRUE(std::isfinite(e[k]));EXPECT_NEAR(a[k],e[k],1e-8+3e-11*std::abs(e[k]));}
  EXPECT_EQ(actual.center_contacts,expected.center_contacts);EXPECT_EQ(actual.corner_contacts,expected.corner_contacts);
}
inline bool Batch(const d::PreparedForceValues& values) {
  d::DampingActivity activity;EXPECT_EQ(d::ClassifyDamping(values,activity),d::Status::Success);return activity.triggers_native_batch!=0;
}
inline d::ForceResult Check(const Case& c) {
  const auto values=Prepare(c);d::ForceResult result;
  EXPECT_EQ(d::EvaluateForce(values,Batch(values),result),d::Status::Success);
  Compare(result,Native({c})[0]);return result;
}
inline Case Folded(double fraction=1.005) {
  auto c=Base(true);c.input.position_m[0]={.01,.015,.04*fraction};
  c.input.velocity_m_s[0]={0,0,.004};c.input.velocity_m_s[1]={0,0,-.002};return c;
}
inline std::vector<Case> Cases() {
  std::vector<Case> out;
  for(auto units:{d::UnitScale{1,1,1},d::UnitScale{.001,1000,1}}) {
    auto base=Base();base.units=units;out.push_back(base);
    for(auto& v:base.input.velocity_m_s)v={1,-2,.5};out.push_back(base);
    for(bool stress:{false,true})for(double mean:{0.,.01,.1,.125}) {
      auto c=Base(stress);c.units=units;
      for(unsigned n=0;n<8;++n)c.input.velocity_m_s[n]={0,0,(n%2?1.:-1.)+mean};out.push_back(c);
    }
    for(double fraction:{.99,1.,1.0005,1.005,1.01,1.1}) {
      auto c=Folded(fraction);c.units=units;out.push_back(c);
      auto unflagged=c;for(double& stress:unflagged.parameters.cauchy_stress_pa)stress=0;
      for(auto& velocity:unflagged.input.velocity_m_s)velocity={};out.push_back(unflagged);
      for(auto& x:c.input.position_m)x={x.y,x.z,x.x};
      for(auto& v:c.input.velocity_m_s)v={v.y,v.z,v.x};out.push_back(c);
    }
  }
  return out;
}
inline void Same(const d::ForceResult& a,const d::ForceResult& b) {
  EXPECT_EQ(Values(a),Values(b));EXPECT_EQ(a.damping_applied,b.damping_applied);
  EXPECT_EQ(a.center_contacts,b.center_contacts);EXPECT_EQ(a.corner_contacts,b.corner_contacts);
}
} // namespace distortion_force_test
