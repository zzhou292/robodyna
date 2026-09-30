#pragma once
#include "VehicleRigidFixture.h"
#include "lib_src/solvers/ExplicitNodalRigidStep.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "lib_src/constraints/NodalRigidKineticObservation.h"
#include <cuda_runtime.h>
#include <cstring>
namespace vehicle_rigid_owner_test {
using namespace vehicle_rigid_test;
using Code=fe::NodalStatus;
inline constexpr std::uint64_t Qualification=78124;
class Cuda:public ::testing::Test {
  void SetUp() override{int n=0;ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess);ASSERT_GT(n,0);}
};
struct Snapshot {
  std::vector<double> x,v,w,q,reaction,couple;fe::NodalStamp stamp;
  explicit Snapshot(std::size_t n):x(3*n),v(3*n),w(3*n),q(4*n),reaction(3*n),couple(3*n){}
  fe::NodalSnapshotBuffer buffer(){return {x.data(),v.data(),x.size()/3,q.data(),w.data(),reaction.data(),couple.data()};}
};
struct Fixture {
  SourceFixture source;std::size_t n;double h=1./1024;
  std::vector<double> x,v,w,q,inverse,inverse_j;std::vector<std::uint8_t> fixed;
  explicit Fixture(std::size_t maximum_groups=0,std::size_t nodes=VehicleNodes):source(false,maximum_groups,nodes),n(nodes),
    x(3*n),v(3*n),w(3*n),q(4*n),inverse(n,.5),inverse_j(n,1000),fixed(n) {
    EXPECT_TRUE(source.model.Initialize(source.Input(n)));
    for(std::size_t i=0;i<n;++i){v[3*i]=.125;q[4*i]=1;}
    for(const auto& m:source.members)for(unsigned a=0;a<3;++a)x[3*m.global_node+a]=rigid_test::Get(m.position,a);
  }
  fe::NodalStateConfig Config(bool capture=false) const {
    fe::NodalStateConfig c;c.node_count=n;c.max_nodes=fe::MaxActiveNodalStateNodes;
    c.fixed_dt=h;c.max_device_bytes=fe::MaxActiveNodalStateDeviceBytes;
    c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    c.capture_force_stage_accelerations=capture;c.rigid_limits=fe::NodalRigidOwnerLimits::Vehicle();return c;
  }
  fe::NodalReport Initialize(fe::FENodalState& owner,const fe::NodalStateConfig& c) const {
    return owner.Initialize(c,{x.data(),v.data(),w.data(),n,q.data()},inverse.data(),
      {fixed.data(),fixed.data(),inverse_j.data()},source.model);
  }
};
inline bool Read(fe::FENodalState& owner,Snapshot& s) {
  const auto r=owner.CopyAccepted(s.buffer(),&s.stamp);EXPECT_EQ(r.status,Code::Ok)<<r.message;return r.status==Code::Ok;
}
inline void Same(const Snapshot& a,const Snapshot& b,bool identity=true) {
  EXPECT_EQ(a.x,b.x);EXPECT_EQ(a.v,b.v);EXPECT_EQ(a.w,b.w);EXPECT_EQ(a.q,b.q);
  EXPECT_EQ(a.reaction,b.reaction);EXPECT_EQ(a.couple,b.couple);
  if(identity)EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,b.stamp));
  else {EXPECT_EQ(a.stamp.epoch,b.stamp.epoch);EXPECT_EQ(a.stamp.time,b.stamp.time);}
}
// Test-only prescribed force source. Source members use sparse indices including
// the final global node; no per-step host/device buffer allocation is needed.
static __global__ void Load(fe::NodalAssemblyView view,std::uint32_t members,bool failure) {
  const auto n=view.accepted.node_count;
  for(std::uint32_t i=0;i<members;++i) {
    const auto node=i+1==members?n-1:i;
    view.forces.force_x[node]=failure&&i+2>=members?1e308:2;
  }
  view.forces.force_x[n-2]=4;
}
inline bool Begin(fe::FENodalState& owner,std::size_t members,fe::NodalTrialToken& token,
    fe::NodalAssemblyView& view,bool fail=false) {
  auto r=owner.BeginTrial(&token,&view);EXPECT_EQ(r.status,Code::Ok);if(r.status!=Code::Ok)return false;
  Load<<<1,1,0,view.stream>>>(view,static_cast<std::uint32_t>(members),fail);
  EXPECT_EQ(cudaPeekAtLastError(),cudaSuccess);r=owner.SealAssembly(token);EXPECT_EQ(r.status,Code::Ok);return r.status==Code::Ok;
}
inline fe::NodalStaggeredHistoryAdmission Admission(fe::FENodalState& o,const fe::NodalAssemblyView& v) {
  return {v.owner_id,v.accepted.base_epoch,v.attempt,o.accepted().fixed_dt,.2,Qualification};
}
inline bool Prepare(fe::FENodalState& o,const fe::NodalTrialToken& t,const fe::NodalAssemblyView& v) {
  const auto r=fe::AdvanceStaggeredRigidGroups(o,t,Admission(o,v));EXPECT_EQ(r.status,Code::Ok)<<r.message;return r.status==Code::Ok;
}
inline bool Commit(fe::FENodalState& o,const fe::NodalTrialToken& t,const fe::NodalAssemblyView& v) {
  auto r=fe::CompleteNodalValidation(o,t,{v.owner_id,v.accepted.base_epoch,v.attempt,Qualification,true});
  EXPECT_EQ(r.status,Code::Ok);if(r.status!=Code::Ok)return false;r=o.Commit(t);EXPECT_EQ(r.status,Code::Ok);return r.status==Code::Ok;
}
inline void SameGroups(const std::vector<fe::NodalRigidGroupSnapshot>& a,const std::vector<fe::NodalRigidGroupSnapshot>& b) {
  ASSERT_EQ(a.size(),b.size());for(std::size_t g=0;g<a.size();++g){
    EXPECT_EQ(a[g].source_group_id,b[g].source_group_id);EXPECT_EQ(a[g].source_node_set_id,b[g].source_node_set_id);
    EXPECT_EQ(std::memcmp(&a[g].state,&b[g].state,sizeof(a[g].state)),0);}
}
} // namespace vehicle_rigid_owner_test
