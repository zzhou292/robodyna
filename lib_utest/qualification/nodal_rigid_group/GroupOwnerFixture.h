#pragma once
#include "lib_utest/qualification/nodal/NodalTemporalFixture.h"
#include "lib_src/constraints/NodalRigidGroupModel.h"
#include "lib_src/solvers/ExplicitNodalRigidStep.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "GroupStepTestSupport.h"

namespace rigid_owner_test {
namespace nt=tl_test::nodal_temporal;
namespace fe=tl::fea;
using Code=fe::NodalStatus;
using Vec3=tl::math::Vec3;
constexpr std::uint64_t Source=781,Qualification=982;
using Cuda=nt::NodalTemporalCuda;
struct Fixture {
  nt::Initial input;
  std::array<fe::NodalRigidGroupMember,8> members{};
  std::array<fe::NodalRigidGroupInput,2> groups{};
  fe::NodalRigidGroupModel model;
  std::size_t count;
  explicit Fixture(std::size_t group_count=2):count(group_count) {
    input.n=4*count+1; input.h=1./1024;
    const auto packet=rigid_step_test::Fixture();
    for(std::size_t g=0;g<count;++g) {
      groups[g]={300+g,400+g,members.data()+4*g,4};
      for(unsigned m=0;m<4;++m) {
        const auto node=4*g+m; auto p=packet.member[m].position; p.z+=g;
        members[node]={100+node,node,p,2,.001,.0004,.0006};
        input.inverse[node]=.5; input.inverse_inertia[node]=1000;
        const double x[]{p.x,p.y,p.z},v[]{.3,-.2,.1};
        for(unsigned a=0;a<3;++a) { input.x[3*node+a]=x[a]; input.v[3*node+a]=v[a]; }
      }
    }
    input.x[3*(input.n-1)]=2;
    input.v[3*(input.n-1)]=-.125;
    const auto result=model.Initialize({Source,input.n,groups.data(),count,{1000,.001}});
    EXPECT_TRUE(result)<<result.message;
  }
  fe::NodalReport Initialize(fe::FENodalState& owner,std::size_t budget=fe::MaxTranslationDeviceBytes,
      fe::NodalTemporalScheme scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart) const {
    fe::NodalStateConfig config; config.node_count=input.n; config.fixed_dt=input.h;
    config.max_device_bytes=budget; config.temporal_scheme=scheme;
    return owner.Initialize(config,{input.x.data(),input.v.data(),input.omega.data(),input.n,input.q.data()},
      input.inverse.data(),{input.fixed.data(),input.rotation_fixed.data(),input.inverse_inertia.data()},model);
  }
  nt::Loads Load(unsigned step=0) const {
    nt::Loads load; const auto packet=rigid_step_test::Fixture();
    for(std::size_t g=0;g<count;++g) for(unsigned i=0;i<4;++i) for(unsigned a=0;a<3;++a) {
      const auto node=4*g+i;
      const auto sign=step<16?1.:(step<32?-.625:0.);
      load.force[3*node+a]=sign*rigid_test::Get(packet.member[i].force,a);
      load.couple[3*node+a]=sign*rigid_test::Get(packet.member[i].couple,a);
    }
    load.force[3*(input.n-1)]=4; load.couple[3*(input.n-1)+2]=1;
    return load;
  }
};
struct Groups {
  std::array<fe::NodalRigidGroupSnapshot,2> values{};
  fe::NodalStamp stamp{};
  fe::NodalRigidGroupSnapshotBuffer buffer() { return {values.data(),values.size()}; }
};
inline bool Read(fe::FENodalState& owner,Groups& out) {
  const auto report=owner.CopyAcceptedRigidGroups(out.buffer(),&out.stamp);
  EXPECT_EQ(report.status,Code::Ok)<<report.message; return report.status==Code::Ok;
}
inline void SameGroups(const Groups& a,const Groups& b,bool stamp=true) {
  for(unsigned i=0;i<a.values.size();++i) {
    const auto& x=a.values[i]; const auto& y=b.values[i];
    EXPECT_EQ(x.source_group_id,y.source_group_id); EXPECT_EQ(x.source_node_set_id,y.source_node_set_id);
    const Vec3 xv[]{x.state.center,x.state.velocity,x.state.omega},yv[]{y.state.center,y.state.velocity,y.state.omega};
    for(unsigned v=0;v<3;++v) for(unsigned axis=0;axis<3;++axis)
      EXPECT_EQ(rigid_test::Get(xv[v],axis),rigid_test::Get(yv[v],axis));
    for(unsigned j=0;j<9;++j) EXPECT_EQ(x.state.principal_axes.v[j],y.state.principal_axes.v[j]);
  }
  if(stamp) EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,b.stamp));
}
inline fe::NodalStaggeredHistoryAdmission Admission(const fe::FENodalState& owner,const fe::NodalAssemblyView& view) {
  return {view.owner_id,view.accepted.base_epoch,view.attempt,owner.accepted().fixed_dt,.2,Qualification};
}
inline bool Prepare(fe::FENodalState& owner,const nt::Loads& loads,fe::NodalTrialToken& token,fe::NodalAssemblyView& view) {
  if(!nt::BeginLoad(owner,loads,token,view)) return false;
  auto r=owner.SealAssembly(token); EXPECT_EQ(r.status,Code::Ok)<<r.message;
  if(r.status!=Code::Ok) return false;
  r=fe::AdvanceStaggeredRigidGroups(owner,token,Admission(owner,view)); EXPECT_EQ(r.status,Code::Ok)<<r.message;
  return r.status==Code::Ok;
}
inline bool Accept(fe::FENodalState& owner,const fe::NodalTrialToken& token,const fe::NodalAssemblyView& view) {
  auto r=fe::CompleteNodalValidation(owner,token,{view.owner_id,view.accepted.base_epoch,view.attempt,Qualification,true});
  EXPECT_EQ(r.status,Code::Ok)<<r.message; if(r.status!=Code::Ok) return false;
  r=owner.Commit(token); EXPECT_EQ(r.status,Code::Ok)<<r.message; return r.status==Code::Ok;
}
inline bool Step(fe::FENodalState& owner,const nt::Loads& loads) {
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  return Prepare(owner,loads,token,view)&&Accept(owner,token,view);
}
inline void SameOwners(fe::FENodalState& a,fe::FENodalState& b) {
  nt::Snapshot x,y; ASSERT_TRUE(nt::Read(a,x)); ASSERT_TRUE(nt::Read(b,y));
  x.stamp.owner_id=y.stamp.owner_id; nt::SameState(x,y);
  Groups gx,gy; ASSERT_TRUE(Read(a,gx)); ASSERT_TRUE(Read(b,gy)); SameGroups(gx,gy,false);
}
} // namespace rigid_owner_test
