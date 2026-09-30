// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "lib_src/solvers/ExplicitNodalRigidStep.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <cuda_runtime.h>

namespace rigid_assembly_owner_test {
class Cuda : public ::testing::Test {
  void SetUp() override {
    int count=0;
    ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
    ASSERT_GT(count,0);
    ASSERT_EQ(cudaSetDevice(0),cudaSuccess);
  }
};
struct Snapshot {
  std::size_t n;
  std::vector<double> values;
  std::array<fe::NodalRigidGroupSnapshot,2> groups;
  fe::NodalStamp stamp;
  explicit Snapshot(std::size_t nodes):n(nodes),values(19*n,-71.) {}
  fe::NodalSnapshotBuffer Buffer() {
    return {values.data(),values.data()+3*n,n,values.data()+9*n,values.data()+6*n,
      values.data()+13*n,values.data()+16*n};
  }
  void Read(fe::FENodalState& owner) {
    ASSERT_EQ(owner.CopyAccepted(Buffer(),&stamp).status,Code::Ok);
    fe::NodalStamp group_stamp;
    ASSERT_EQ(owner.CopyAcceptedRigidGroups({groups.data(),groups.size()},&group_stamp).status,Code::Ok);
    EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,group_stamp));
  }
};
inline void Same(const Snapshot& a,const Snapshot& b) {
  EXPECT_EQ(a.values,b.values);
  EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,b.stamp));
  for (unsigned g=0;g<2;++g) {
    EXPECT_EQ(a.groups[g].source_kind,b.groups[g].source_kind);
    EXPECT_EQ(a.groups[g].source_group_id,b.groups[g].source_group_id);
    std::array<double,18> x,y;
    r::WriteGroupState(x.data(),a.groups[g].state);
    r::WriteGroupState(y.data(),b.groups[g].state);
    EXPECT_EQ(x,y);
  }
}
inline fe::NodalReport Initialize(fe::FENodalState& owner,Fixture& f,bool use_cin=false,bool capture=false) {
  f.DependentInverses(use_cin);
  auto config=f.Config();
  config.capture_force_stage_accelerations=capture;
  const auto cin=f.Cin();
  return owner.Initialize(config,f.Kinematics(),f.im.data(),f.Dofs(),f.binding,use_cin?&cin:nullptr);
}
inline std::vector<double> Loads(const Fixture& f,unsigned step=0) {
  const auto n=f.m.size();
  std::vector<double> out(6*n);
  const auto sign=step<16?1.:(step<32?-.5:.25);
  for (const auto& member:f.binding.members()) {
    const auto i=member.domain_node;
    for (unsigned axis=0;axis<3;++axis) {
      out[axis*n+i]=sign*(axis+1)*1e-5;
      out[(axis+3)*n+i]=sign*(axis+1)*1e-8;
    }
  }
  out[f.ordinary]=f.m[f.ordinary]*.125;
  return out;
}
inline void Begin(fe::FENodalState& owner,const Fixture& f,const std::vector<double>& loads,
    bool use_cin,fe::NodalTrialToken& token,fe::NodalAssemblyView& view) {
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,Code::Ok);
  double* components[]{view.forces.force_x,view.forces.force_y,view.forces.force_z,
    view.forces.couple_x,view.forces.couple_y,view.forces.couple_z};
  for (unsigned axis=0;axis<6;++axis)
    ASSERT_EQ(cudaMemcpyAsync(components[axis],loads.data()+axis*f.m.size(),f.m.size()*sizeof(double),
      cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  if (use_cin) {
    fe::NodalCinAssemblyView cin;
    ASSERT_EQ(owner.BorrowCinAssembly(token,&cin).status,Code::Ok);
    ASSERT_EQ(cudaMemsetAsync(cin.witness_activity,1,cin.witness_count,view.stream),cudaSuccess);
  }
  ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
}
inline fe::NodalReport Advance(fe::FENodalState& owner,const fe::NodalTrialToken& token,
    const fe::NodalAssemblyView& view,bool use_cin,double angle=.2) {
  if (use_cin) return fe::AdvanceStaggeredCin(owner,token,
    {view.owner_id,view.accepted.base_epoch,view.attempt,Qualification,H,angle,true});
  return fe::AdvanceStaggeredRigidGroups(owner,token,
    {view.owner_id,view.accepted.base_epoch,view.attempt,H,angle,Qualification});
}
inline void Commit(fe::FENodalState& owner,const fe::NodalTrialToken& token,const fe::NodalAssemblyView& view) {
  ASSERT_EQ(fe::CompleteNodalValidation(owner,token,
    {view.owner_id,view.accepted.base_epoch,view.attempt,Qualification,true}).status,Code::Ok);
  ASSERT_EQ(owner.Commit(token).status,Code::Ok);
}
} // namespace rigid_assembly_owner_test
