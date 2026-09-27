// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../solid24_icontrol/NativeSupport.h"
#include "../controlled_hourglass/WorkComparison.h"
#include "lib_src/elements/solid24/Solid24ForceMaterial.h"
#include "lib_src/elements/solid24/controlled_hourglass/BeforeDistortion.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <vector>
extern "C" void h24_adapter_observations(double*,int*,int*);
namespace h24_test {
namespace s=tl::fea::solid24;
namespace c=s::controlled_hourglass;
namespace b=tl::fea::solid_common;
namespace native=solid24_icontrol_test;
struct Case {
  s::Reference reference;s::Material material;s::PrescribedInterval interval;
  c::HistoryValues accepted;bool initialization=false;
};
struct Trial {
  s::ForceGeometry geometry;tl::material::law42::CallerResult material;
  c::BeforeDistortionResult stage;
};
TL_BRICK_HD inline s::ForceStatus Evaluate(const Case& x,Trial& output) {
  if(!x.reference.prepared()||!x.reference.reference_jacobian())return s::ForceStatus::UnsupportedProfile;
  Trial next;auto status=s::force_detail::CurrentKinematics(x.reference,x.interval,next.geometry);
  if(status!=s::ForceStatus::Success)return status;
  status=s::force_detail::EvaluateMaterial(x.reference,x.material,next.geometry,x.interval.dt_s,
      x.accepted.material,x.initialization,next.material);
  if(status!=s::ForceStatus::Success)return status;
  status=c::EvaluateBeforeDistortion(x.reference,x.material,next.geometry,next.material,x.interval.dt_s,
      x.accepted.controlled_hourglass,next.stage);
  if(status!=s::ForceStatus::Success)return status;
  output=next;return s::ForceStatus::Success;
}
inline Case Base(s::ReferenceInput input=solid24_test::Brick()) {
  Case x;x.reference=heph_test::Reference(input);x.material=heph_test::Material(input.density_kg_m3);
  x.accepted.material.density_kg_m3=x.material.density_kg_m3;
  x.interval.dt_s=1e-6;x.interval.sample_index=1;
  for(unsigned n=0;n<8;++n)x.interval.position_m[n]=input.position_m[n];
  return x;
}
inline void Move(Case& x,double phase=1) {
  for(unsigned n=0;n<8;++n) {
    const auto p=x.reference.input().position_m[n];
    x.interval.position_m[n]={p.x*(1+.03*phase)+.04*phase*p.y,p.y*(1-.02*phase),p.z*(1+.01*phase)};
    x.interval.velocity_m_s[n]={phase*.13*(int(n)%3-1),phase*.21*(n%2?1:-1),phase*.17*(int(n)%4-1)};
  }
}
inline std::array<double,105> GeometryValues(const s::ForceGeometry& g) {
  std::array<double,105> a{};unsigned i=0;
  for(double v:g.current.frame.v)a[i++]=v;
  for(const auto& v:g.current.local_position_m){a[i++]=v.x;a[i++]=v.y;a[i++]=v.z;}
  for(const auto& v:g.local_velocity_m_s){a[i++]=v.x;a[i++]=v.y;a[i++]=v.z;}
  a[i++]=g.current.volume_m3;a[i++]=g.current.characteristic_length_m;
  for(const auto& row:g.derivative_per_m)for(double v:row)a[i++]=v;
  for(const auto& row:g.hourglass_projection)for(double v:row)a[i++]=v;
  for(double v:g.jacobian_diagonal_m)a[i++]=v;
  for(double v:g.material_displacement_gradient)a[i++]=v;
  for(double v:g.engineering_rate_per_s)a[i++]=v;
  return a;
}
inline std::array<double,63> HourValues(const c::hg::Result& r) {
  std::array<double,63>a{};unsigned i=0;
  for(const auto& row:r.proposed_state.force_n)for(double v:row)a[i++]=v;
  for(const auto& v:r.local_force_n){a[i++]=v.x;a[i++]=v.y;a[i++]=v.z;}
  a[i++]=r.internal_energy_density_j_m3;a[i++]=r.raw_stiffness_n_m;a[i++]=r.work_j;
  for(const auto& row:r.modal_velocity_m_s)for(double v:row)a[i++]=v;
  for(const auto& row:r.modal_force_n)for(double v:row)a[i++]=v;
  return a;
}
inline std::vector<double> Values(const Trial& r) {
  const auto g=GeometryValues(r.geometry);std::vector<double>a(g.begin(),g.end());
  const auto m=law42_caller_test::Values(r.material);a.insert(a.end(),m.begin(),m.end());
  const auto h=HourValues(r.stage.hourglass);a.insert(a.end(),h.begin(),h.end());
  for(const auto* array:{r.stage.local_force_after_material_n,r.stage.world_native_force_before_distortion_n,r.stage.world_force_before_distortion_n})
    for(unsigned n=0;n<8;++n){a.push_back(array[n].x);a.push_back(array[n].y);a.push_back(array[n].z);}
  for(double v:r.stage.proposed_values.material.stress_pa)a.push_back(v);
  a.push_back(r.stage.proposed_values.material.density_kg_m3);a.push_back(r.stage.proposed_values.material.internal_energy_density_j_m3);
  a.push_back(r.stage.proposed_values.material.bulk_pressure_pa);
  for(const auto& row:r.stage.proposed_values.controlled_hourglass.force_n)for(double v:row)a.push_back(v);
  a.push_back(r.stage.raw_stiffness_before_distortion_n_m);return a;
}
struct NativeTrial {native::Trial full;std::array<double,201> snapshot{};std::array<int,5> calls{};int valid=0;};
inline NativeTrial Native(const native::History& history,const Case& x) {
  NativeTrial r;r.full=native::Step(history,x.interval,x.material);
  h24_adapter_observations(r.snapshot.data(),r.calls.data(),&r.valid);return r;
}
inline native::History NativeHistory(const Case& x) {
  native::History h(x.reference.input());
  for(unsigned i=0;i<6;++i)h.values[i]=x.accepted.material.stress_pa[i];
  h.values[6]=x.accepted.material.density_kg_m3;h.values[7]=x.accepted.material.internal_energy_density_j_m3;
  h.values[8]=x.accepted.material.bulk_pressure_pa;
  for(unsigned k=0;k<3;++k)for(unsigned j=0;j<4;++j)h.values[9+4*k+j]=x.accepted.controlled_hourglass.force_n[k][j];
  return h;
}
inline void Compare(const Case& x,const Trial& a,const NativeTrial& n,bool check_work=true) {
  native::Ready(n.full);ASSERT_FALSE(::testing::Test::HasFailure());ASSERT_EQ(n.valid,1);
  for(int v:n.calls)ASSERT_EQ(v,1);
  const auto g=GeometryValues(a.geometry);std::array<double,187> legacy{};
  std::copy_n(n.snapshot.begin(),105,legacy.begin()+46);
  for(unsigned i=0;i<105;++i){SCOPED_TRACE(i);EXPECT_NEAR(g[i],n.snapshot[i],heph_test::ForceTolerance(i+46,legacy));}
  const auto m=law42_caller_test::Values(a.material);std::array<double,33> nm{};
  std::copy_n(n.full.values.begin()+46,33,nm.begin());
  for(unsigned i=0;i<33;++i){SCOPED_TRACE(i);EXPECT_NEAR(m[i],nm[i],law42_caller_test::NativeTolerance(i,nm));}
  const auto h=HourValues(a.stage.hourglass);std::array<double,63> nh{};
  std::copy_n(n.full.values.begin()+9,12,nh.begin());std::copy_n(n.snapshot.begin()+105,24,nh.begin()+12);
  nh[36]=n.full.values[7];nh[37]=n.full.values[80];nh[38]=n.full.values[90];
  std::copy_n(n.snapshot.begin()+177,24,nh.begin()+39);
  // End-to-end geometry/material propagation uses the existing HEPH stage
  // contract; the independent raw controlled leaf retains its 128-epsilon gates.
  const unsigned edges[]{0,12,36,37,38,39,51,63};
  for(unsigned j=0;j<7;++j) {
    if(j==4)continue;
    double scale=1e-20;for(unsigned i=edges[j];i<edges[j+1];++i)scale=std::max(scale,std::abs(nh[i]));
    for(unsigned i=edges[j];i<edges[j+1];++i){SCOPED_TRACE(i);ASSERT_TRUE(std::isfinite(h[i]));EXPECT_NEAR(h[i],nh[i],3e-10*scale);}
  }
  if(check_work)EXPECT_TRUE(controlled_test::SignedWorkMatches(h,nh,x.interval.dt_s));
  for(unsigned stage=0;stage<2;++stage) {
    const auto* f=stage?a.stage.world_native_force_before_distortion_n:a.stage.local_force_after_material_n;
    double scale=1e-20;for(unsigned i=0;i<24;++i)scale=std::max(scale,std::abs(n.snapshot[129+24*stage+i]));
    for(unsigned i=0;i<24;++i)EXPECT_NEAR(b::Component(f[i/3],i%3),n.snapshot[129+24*stage+i],3e-10*scale);
  }
  for(unsigned slot=0;slot<8;++slot)for(unsigned k=0;k<3;++k)
    EXPECT_EQ(b::Component(a.stage.world_force_before_distortion_n[x.reference.source_slot(slot)],k),
      b::Component(a.stage.world_native_force_before_distortion_n[slot],k));
  EXPECT_EQ(a.stage.proposed_values.material.internal_energy_density_j_m3,a.stage.hourglass.internal_energy_density_j_m3);
  EXPECT_EQ(a.stage.raw_stiffness_before_distortion_n_m,a.stage.hourglass.raw_stiffness_n_m);
}
inline Trial Check(const Case& x) {
  Trial r;EXPECT_EQ(Evaluate(x,r),s::ForceStatus::Success);Compare(x,r,Native(NativeHistory(x),x));return r;
}
inline void Accept(Case& x,const Trial& r) {
  x.accepted=r.stage.proposed_values;x.interval.base_time_s+=x.interval.dt_s;++x.interval.sample_index;
}
inline std::vector<Case> Cases() {
  std::vector<Case> values{Base()};auto moving=Base();Move(moving);values.push_back(moving);
  auto warped=solid24_test::Brick();warped.position_m[6].x+=.002;warped.position_m[3].z+=.001;
  auto w=Base(warped);Move(w);values.push_back(w);
  auto reverse=solid24_test::Brick();for(unsigned n=0;n<4;++n){std::swap(reverse.position_m[n],reverse.position_m[n+4]);std::swap(reverse.source_node_id[n],reverse.source_node_id[n+4]);}
  auto r=Base(reverse);Move(r);values.push_back(r);
  for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h){auto x=Base();x.accepted.controlled_hourglass.force_n[k][h]=.125*(1+4*k+h);values.push_back(x);}
  return values;
}
} // namespace h24_test
