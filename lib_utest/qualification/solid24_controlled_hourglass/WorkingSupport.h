// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include "lib_src/elements/solid24/controlled_hourglass/UnitResponse.h"
namespace h24_test {
inline Case MillimetreCase(s::ReferenceInput input=solid24_test::Brick()) {
  input.profile.working_length=s::WorkingLengthUnit::Millimetre;return Base(input);
}
inline c::WorkingReference WorkingReference(const Case& x) {
  c::WorkingReference r;const auto u=x.reference.input().profile.working_length==s::WorkingLengthUnit::Millimetre?
    c::UnitScale{.001,1000,1}:c::UnitScale{1,1,1};
  if(c::PrepareWorkingReference(x.reference,x.material,u,r)!=s::ForceStatus::Success)throw std::runtime_error("working reference failed");return r;
}
// Independent wire conversion is dimension-labelled and does not call the
// production unit adapter. Complete Fortran routines run in mm/Mg/s values.
inline native::History MillimetreNativeHistory(const Case& x) {
  auto source=x.reference.input();source.density_kg_m3*=1e-12;
  for(auto& p:source.position_m){p.x*=1000;p.y*=1000;p.z*=1000;}
  native::History h(source);
  for(unsigned k=0;k<6;++k)h.values[k]=x.accepted.material.stress_pa[k]*1e-6;
  h.values[6]=x.accepted.material.density_kg_m3*1e-12;
  h.values[7]=x.accepted.material.internal_energy_density_j_m3*1e-6;h.values[8]=x.accepted.material.bulk_pressure_pa*1e-6;
  for(unsigned k=0;k<3;++k)for(unsigned j=0;j<4;++j)h.values[9+4*k+j]=x.accepted.controlled_hourglass.force_n[k][j];
  return h;
}
struct MillimetreNativeTrial {NativeTrial physical;std::array<double,22> carried{};std::array<double,63> raw_work{};};
inline MillimetreNativeTrial MillimetreNative(const native::History& history,const Case& x) {
  auto interval=x.interval;for(unsigned n=0;n<8;++n){auto& p=interval.position_m[n];p.x*=1000;p.y*=1000;p.z*=1000;
    auto& v=interval.velocity_m_s[n];v.x*=1000;v.y*=1000;v.z*=1000;}
  auto material=x.material;material.mu_pa*=1e-6;material.bulk_pa*=1e-6;material.density_kg_m3*=1e-12;material.tension_cutoff_pa*=1e-6;
  MillimetreNativeTrial r;auto& p=r.physical;p.full=native::Step(history,interval,material);
  h24_adapter_observations(p.snapshot.data(),p.calls.data(),&p.valid);
  auto& v=p.full.values;std::copy_n(v.begin(),22,r.carried.begin());
  r.raw_work[38]=v[90];std::copy_n(p.snapshot.begin()+177,24,r.raw_work.begin()+39);
  for(unsigned i=0;i<6;++i)v[i]*=1e6;v[6]*=1e12;v[7]*=1e6;v[8]*=1e6;v[21]*=.001;
  auto* m=v.data()+46;
  for(unsigned i=0;i<6;++i)m[i]*=1e6;m[6]*=1e12;m[7]*=1e6;m[8]*=1e6;
  for(unsigned i=9;i<=16;++i)m[i]*=1e6;m[19]*=.001;m[21]*=1e6;
  m[28]*=1e-9;m[29]*=1e-9;m[30]*=.001;m[32]*=1000;
  for(unsigned i=79;i<90;++i)v[i]*=1000;v[90]*=.001;v[91]*=.001;
  for(double& a:p.full.distortion_sigma)a*=1e6;
  p.full.distortion_parameters[0]*=1000;p.full.distortion_parameters[1]*=1e6;
  p.full.distortion_parameters[2]*=.001;p.full.distortion_parameters[4]*=1e-9;
  auto& g=p.snapshot;
  for(unsigned i=9;i<57;++i)g[i]*=.001;g[57]*=1e-9;g[58]*=.001;
  for(unsigned i=59;i<71;++i)g[i]*=1000;for(unsigned i=87;i<90;++i)g[i]*=.001;
  for(unsigned i=177;i<189;++i)g[i]*=.001;
  return r;
}
inline std::array<double,63> NativeWitness(const c::NativeModalWork& w) {
  std::array<double,63>a{};a[38]=w.work;
  for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h){a[39+4*k+h]=w.rate[k][h];a[51+4*k+h]=w.force[k][h];}
  return a;
}
inline void CompareWorking(const Case& x,const c::WorkingResult& a,const MillimetreNativeTrial& n) {
  Trial values;values.geometry=a.geometry;values.material=a.material;values.stage=a.stage;
  Compare(x,values,n.physical,false);
  EXPECT_TRUE(controlled_test::SignedWorkMatches(NativeWitness(a.native_modal_work),n.raw_work,x.interval.dt_s));
  EXPECT_EQ(a.native_modal_work.units.length_m,.001);EXPECT_EQ(a.native_modal_work.units.mass_kg,1000);EXPECT_EQ(a.native_modal_work.units.time_s,1);
  EXPECT_EQ(a.stage.hourglass.work_j,a.native_modal_work.work*.001);
}
inline std::vector<double> Values(const c::WorkingResult& r) {
  Trial t;t.geometry=r.geometry;t.material=r.material;t.stage=r.stage;auto a=Values(t);
  const auto raw=NativeWitness(r.native_modal_work);a.insert(a.end(),raw.begin(),raw.end());
  a.push_back(r.native_modal_work.units.length_m);a.push_back(r.native_modal_work.units.mass_kg);a.push_back(r.native_modal_work.units.time_s);return a;
}
} // namespace h24_test
