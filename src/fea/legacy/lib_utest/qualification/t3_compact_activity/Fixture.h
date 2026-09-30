// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/t3/mapped/ActivityQuery.h"
#include "lib_src/elements/t3/T3Startup.h"
#include "lib_src/materials/TabulatedShellPlasticity.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <vector>
namespace t3_compact_test {
namespace fe=tl::fea;namespace t=fe::t3;namespace m=t::mapped;
namespace sp=fe::shell_batch_plasticity_detail;
using Law=fe::ShellSectionLaw;using Policy=fe::ShellFailurePolicy;
// Prescribed retained packets for validation, not a second force solver.
struct Fixture {
  std::vector<t::T3BatchElement> elements;
  std::vector<t::ForceTrial> forces;
  std::vector<Law> laws;
  std::vector<Policy> policies;
  std::vector<fe::sections::PointParameters> parameters;
  std::vector<fe::ShellBatchOnePointSectionState> points;
  std::vector<fe::ShellBatchSectionState> plastic;
  std::vector<fe::sections::ShellLayeredLaw1History> elastic;
  std::vector<fe::ShellBatchFailureState> failure;
  std::vector<fe::ShellBatchLayeredSection> sections;
  std::size_t missing_law=SIZE_MAX,missing_parameter=SIZE_MAX,missing_failure=SIZE_MAX;
  std::array<double,3> curve_x{{0,.1,.5}},curve_y{{1e6,2e6,3e6}};
  double time=0,force_time=0;
  std::uint64_t epoch=0,force_epoch=0;
  bool has_point=true,execution=true;
  explicit Fixture(std::size_t count=129,unsigned step=0):elements(count),forces(count),laws(count),policies(count),
      parameters(count),points(count),plastic(count),elastic(count),failure(count),sections(count) {
    epoch=force_epoch=step;time=force_time=step*0x1p-15;
    t::ReferenceInput input;input.node_ids[0]=1;input.node_ids[1]=2;input.node_ids[2]=3;
    input.position[0]={0,0,0};input.position[1]={1,0,0};input.position[2]={.2,.8,0};
    input.density=1000;input.young_modulus=250e6;input.poisson_ratio=.35;input.thickness=.0005;
    const Law cycle[]{Law::Law44Nip1,Law::LayeredLaw1Nip3,Law::LayeredLaw44Nip3,Law::RigidSkin,Law::GlobalLaw1Npt0};
    for(std::size_t i=0;i<count;++i) {
      laws[i]=cycle[i%5];policies[i]=laws[i]==Law::Law44Nip1||laws[i]==Law::LayeredLaw44Nip3?Policy::ConstantAllPoints:Policy::None;
      EXPECT_EQ(t::InitializeReference(input,elements[i].reference),t::Status::kSuccess);
      EXPECT_EQ(t::InitializeHistory(elements[i].reference,{time,epoch},forces[i].proposed_history),t::Status::kSuccess);
      const tl::material::TabulatedShellPlasticityRate rate{true,0,1,10000,tl::material::ShellPlasticityRatePolicy::FilteredZeroC};
      EXPECT_EQ(tl::material::PrepareLinearLaw44ShellPlasticity(250e6,.35,1000,{10e6,1e6},rate,parameters[i]),
          tl::material::TabulatedShellPlasticityStatus::Ok);
      points[i].point.reported_thickness_m=input.thickness;
      if(laws[i]==Law::LayeredLaw44Nip3) failure[i]=fe::ShellBatchFailureState::Constant();
      if(step&&laws[i]!=Law::RigidSkin) {
        forces[i].kinematics.area=elements[i].reference.area;
        forces[i].kinematics.sample_index=epoch;
        forces[i].kinematics.base_time=(step-1)*0x1p-15;
        forces[i].kinematics.dt=time-forces[i].kinematics.base_time;
      }
    }
  }
  void Table(bool native_continuation,bool rate_enabled) {
    tl::material::TabulatedShellPlasticityRate rate;
    if(rate_enabled)rate={true,0,1,10000,tl::material::ShellPlasticityRatePolicy::FilteredZeroC};
    for(auto& p:parameters)EXPECT_EQ(tl::material::PrepareTabulatedShellPlasticity(250e6,.35,1000,
        {curve_x.data(),curve_y.data(),3},rate,native_continuation?
        tl::material::ShellPlasticityCurveContinuation::NativeLastSegment:
        tl::material::ShellPlasticityCurveContinuation::StrictDomain,p),tl::material::TabulatedShellPlasticityStatus::Ok);
  }
  std::size_t size() const{return laws.size();}
  std::size_t Find(Law law,bool last=false) const {
    for(std::size_t i=0;i<size();++i){const auto p=last?size()-1-i:i;if(laws[p]==law)return p;}
    return SIZE_MAX;
  }
};
inline void RawBool(bool& value,unsigned char encoded){std::memcpy(&value,&encoded,1);}
inline void Mutate(Fixture& f,unsigned mutation) {
  const auto point=f.Find(Law::Law44Nip1),plastic=f.Find(Law::LayeredLaw44Nip3),elastic=f.Find(Law::LayeredLaw1Nip3);
  const auto global=f.Find(Law::GlobalLaw1Npt0),skin=f.Find(Law::RigidSkin);
  const double nan=std::numeric_limits<double>::quiet_NaN();
  switch(mutation) {
    case 0:break;
    case 1:RawBool(f.points[point].point.failure.history.point_active,2);break;
    case 2:f.points[point].point.saved.plastic_strain=.1;break;
    case 3:f.points[point].point.reported_thickness_m=-1;break;
    case 4:f.elastic[elastic].point[0].stress[0]=nan;break;
    case 5:f.plastic[plastic].history.point[0].stress[0]=nan;break;
    case 6:RawBool(f.failure[plastic].active,2);break;
    case 7:f.failure[plastic]=fe::ShellBatchFailureState::Tab1();break;
    case 8:f.failure[plastic].constant_points()[0].failure_time_s=f.time+1;break;
    case 9:const_cast<t::HistoryValues&>(f.forces[point].proposed_history.data()).active=257;break;
    case 10:f.forces[plastic].internal_force[2].x=nan;break;
    case 11:{auto h=f.forces[point].proposed_history.data();EXPECT_EQ(t::PrepareFailurePrescribedHistory(f.elements[point].reference,h,{f.time+1,f.epoch},f.forces[point].proposed_history),t::Status::kSuccess);break;}
    case 12:{auto input=f.elements[point].reference.input;input.node_ids[0]+=100;t::ReferenceData ref;EXPECT_EQ(t::InitializeReference(input,ref),t::Status::kSuccess);auto h=f.forces[point].proposed_history.data();EXPECT_EQ(t::PrepareFailurePrescribedHistory(ref,h,{f.time,f.epoch},f.forces[point].proposed_history),t::Status::kSuccess);break;}
    case 13:const_cast<t::HistoryValues&>(f.forces[point].proposed_history.data()).thickness*=2;break;
    case 14:const_cast<t::HistoryValues&>(f.forces[plastic].proposed_history.data()).active=0;break;
    case 15:f.points[point].point.saved.stress[0]=f.points[point].point.current.history.stress[0]=1000;break;
    case 16:const_cast<t::HistoryValues&>(f.forces[point].proposed_history.data()).bending_stress[0]=1;break;
    case 17:f.laws[point]=static_cast<Law>(255);break;
    case 18:f.plastic[global].history.point[0].stress[0]=nan;break;
    case 19:RawBool(f.points[skin].point.failure.history.point_active,2);break;
    case 20:f.forces[point].internal_force[0].y=-0.;break;
    case 21:RawBool(f.points[point].point.failure.failed_now,2);break;
    case 22:f.has_point=false;break;
    case 23:f.execution=false;break;
  }
}
inline void MakeInactive(Fixture& f,bool tab1=false) {
  const auto point=f.Find(Law::Law44Nip1),plastic=f.Find(Law::LayeredLaw44Nip3);
  auto& one=f.points[point].point.failure.history;one.point_active=false;one.damage=1;one.failure_time_s=f.time;
  const_cast<t::HistoryValues&>(f.forces[point].proposed_history.data()).active=0;
  if(tab1){f.policies[plastic]=Policy::Tab1AnyPoint;f.failure[plastic]=fe::ShellBatchFailureState::Tab1();
    auto& p=f.failure[plastic].tab1_points()[0];p.damage=p.maximum_damage=1;p.failure_time_s=f.time;p.point_active=false;
  } else for(unsigned p=0;p<3;++p){auto& v=f.failure[plastic].constant_points()[p];v.damage=1;v.failure_time_s=f.time;v.point_active=false;}
  f.failure[plastic].active=false;const_cast<t::HistoryValues&>(f.forces[plastic].proposed_history.data()).active=0;
}
inline void RemovePointRole(Fixture& f) {
  f.has_point=false;
  for(std::size_t p=0;p<f.size();++p)if(f.laws[p]==Law::Law44Nip1){f.laws[p]=Law::LayeredLaw44Nip3;f.failure[p]=fe::ShellBatchFailureState::Constant();}
}
inline void Same(const t::BatchReport& a,const t::BatchReport& b) {
  EXPECT_EQ(a.status,b.status);EXPECT_EQ(a.element,b.element);EXPECT_EQ(a.node,b.node);
  EXPECT_EQ(a.element_status,b.element_status);EXPECT_STREQ(a.message,b.message);
}
} // namespace t3_compact_test
