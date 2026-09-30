// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../solid24_controlled_hourglass/WorkingSupport.h"
#include "lib_src/elements/solid24/controlled_distortion/Stage.h"
namespace h24_full_test {
namespace q=h24_test;namespace s=tl::fea::solid24;namespace c=s::controlled_distortion;namespace b=tl::fea::solid_common;
inline q::Case Case(bool mm){return mm?q::MillimetreCase():q::Base();}
inline c::Reference Reference(const q::Case& x){c::Reference r;const auto u=x.reference.input().profile.working_length==s::WorkingLengthUnit::Millimetre?c::distortion::UnitScale{.001,1000,1}:c::distortion::UnitScale{1,1,1};
 if(c::PrepareReference(x.reference,x.material,u,r)!=s::ForceStatus::Success)throw std::runtime_error("controlled reference");return r;}
struct NativeState {
 bool mm;q::native::History state;
 explicit NativeState(const q::Case& x):mm(x.reference.input().profile.working_length==s::WorkingLengthUnit::Millimetre),state(mm?q::MillimetreNativeHistory(x):q::NativeHistory(x)){}
 q::MillimetreNativeTrial Step(const q::Case& x)const{
  if(mm)return q::MillimetreNative(state,x);
  q::MillimetreNativeTrial out;out.physical=q::Native(state,x);std::copy_n(out.physical.full.values.begin(),22,out.carried.begin());
  out.raw_work[38]=out.physical.full.values[90];std::copy_n(out.physical.snapshot.begin()+177,24,out.raw_work.begin()+39);return out;
 }
 void Accept(const q::MillimetreNativeTrial& n){state.values=n.carried;}
};
inline std::array<double,21> History(const c::History& h){std::array<double,21>a{};unsigned i=0;const auto& v=h.native_values();
 for(double x:v.material.stress_pa)a[i++]=x;a[i++]=v.material.density_kg_m3;a[i++]=v.material.internal_energy_density_j_m3;a[i++]=v.material.bulk_pressure_pa;
 for(const auto& row:v.controlled_hourglass.force_n)for(double x:row)a[i++]=x;return a;
}
inline std::vector<double> Values(const c::Result& r){const auto h=History(r.proposed_history);std::vector<double>a(h.begin(),h.end());a.push_back(r.proposed_history.native_distortion_energy());
 a.push_back(r.proposed_history.stamp().time_s);a.push_back(double(r.proposed_history.stamp().sample_index));
 for(const auto& f:r.rhs_force_n){a.push_back(f.x);a.push_back(f.y);a.push_back(f.z);}
 const double v[]{r.material_raw_stiffness_n_m,r.hourglass_raw_stiffness_n_m,r.nodal_raw_stiffness_n_m,r.minimum_unscaled_dt_s,r.material_work_increment_j,r.hourglass_work_increment_j,r.distortion_energy_j,r.distortion_work_increment_j,r.internal_energy_density_j_m3,double(r.damping_applied),double(r.center_contacts),double(r.corner_contacts)};
 a.insert(a.end(),std::begin(v),std::end(v));return a;
}
inline void Compare(const q::Case& x,const c::Scratch& scratch,const c::Result& result,const q::MillimetreNativeTrial& n){
 q::native::Ready(n.physical.full);ASSERT_FALSE(::testing::Test::HasFailure());
 const auto state=History(result.proposed_history);const unsigned edges[]{0,6,7,8,9,21};
 for(unsigned j=0;j<5;++j){double scale=1e-20;for(unsigned i=edges[j];i<edges[j+1];++i)scale=std::max(scale,std::abs(n.carried[i]));for(unsigned i=edges[j];i<edges[j+1];++i)EXPECT_NEAR(state[i],n.carried[i],3e-10*scale);}
 EXPECT_NEAR(result.proposed_history.native_distortion_energy(),n.carried[21],3e-10*std::max(1e-20,std::abs(n.carried[21])));
 double force_scale=1e-20;for(unsigned i=22;i<46;++i)force_scale=std::max(force_scale,std::abs(n.physical.full.values[i]));
 for(unsigned node=0;node<8;++node)for(unsigned k=0;k<3;++k)EXPECT_NEAR(b::Component(result.rhs_force_n[node],k),n.physical.full.values[22+3*node+k],3e-10*force_scale);
 const auto& v=n.physical.full.values;const double actual[]{result.material_raw_stiffness_n_m,result.hourglass_raw_stiffness_n_m,result.nodal_raw_stiffness_n_m,
  result.minimum_unscaled_dt_s,result.material_work_increment_j,result.hourglass_work_increment_j,result.distortion_energy_j,result.distortion_work_increment_j,result.internal_energy_density_j_m3};
 const double expected[]{v[79],v[80],v[81],v[77],v[76],v[90],v[21],v[91],v[7]};
 for(unsigned i=0;i<9;++i){SCOPED_TRACE(i);EXPECT_NEAR(actual[i],expected[i],3e-10*std::max(1e-20,std::abs(expected[i])));}
 const auto raw=q::HourValues(scratch.prefix.stage.hourglass);EXPECT_TRUE(controlled_test::SignedWorkMatches(raw,n.raw_work,x.interval.dt_s));
 EXPECT_EQ(result.proposed_history.stamp().time_s,v[92]);EXPECT_EQ(result.proposed_history.stamp().sample_index,x.interval.sample_index);
 // No SI roundtrip: next history is exactly the numerical prefix state.
 for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h)EXPECT_EQ(result.proposed_history.native_values().controlled_hourglass.force_n[k][h],scratch.prefix.stage.proposed_values.controlled_hourglass.force_n[k][h]);
 EXPECT_EQ(result.proposed_history.native_values().material.internal_energy_density_j_m3,scratch.prefix.stage.proposed_values.material.internal_energy_density_j_m3);
 for(unsigned node=0;node<8;++node)EXPECT_DOUBLE_EQ(v[82+node],.25*v[81]);
}
inline c::Result Initial(q::Case& x,const c::Reference& reference,NativeState& native){
 x.interval.base_time_s=0;x.interval.dt_s=0;x.interval.sample_index=0;
 c::Scratch scratch;c::Result result;EXPECT_EQ(c::PrepareInitial(reference,{},scratch),s::ForceStatus::Success);
 EXPECT_EQ(c::Complete(scratch,scratch.activity.triggers_native_batch!=0,result),s::ForceStatus::Success);
 auto n=native.Step(x);Compare(x,scratch,result,n);native.Accept(n);x.interval.dt_s=1e-6;x.interval.sample_index=1;return result;
}
} // namespace h24_full_test
