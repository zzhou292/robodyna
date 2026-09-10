#include "VehicleRigidOwnerFixture.h"
namespace vehicle_rigid_owner_test {
TEST_F(Cuda,VehicleCountAndByteCapsRejectBeforeBorrowedArraysAndCleanStartupRetries) {
  Fixture f;fe::FENodalState owner;const auto config=f.Config();
  const auto* values=reinterpret_cast<const double*>(1);const auto* masks=reinterpret_cast<const std::uint8_t*>(1);
  const fe::HostNodalKinematicsView poisoned{values,values,values,f.n,values};
  for(unsigned fault=0;fault<5;++fault) {auto c=config;
    if(fault==0)c.rigid_limits={};if(fault==1)c.rigid_limits.max_groups=758;
    if(fault==2)c.rigid_limits.max_members=7538;if(fault==3)c.rigid_limits.max_host_bytes=1;
    if(fault==4)c.max_device_bytes=1;
    EXPECT_EQ(owner.Initialize(c,poisoned,values,{masks,masks,values},f.source.model).status,Code::ResourceLimit);
    EXPECT_EQ(owner.allocations().device_allocations,0u);EXPECT_EQ(owner.rigid_groups().group_count,0u);
  }
  const auto valid=f.inverse_j.back();f.inverse_j.back()=valid*2;
  const auto rejected=f.Initialize(owner,config);EXPECT_EQ(rejected.status,Code::InvalidInput);EXPECT_EQ(rejected.node,f.n-1);
  EXPECT_EQ(owner.allocations().device_allocations,0u);f.inverse_j.back()=valid;
  ASSERT_EQ(f.Initialize(owner,config).status,Code::Ok);EXPECT_EQ(owner.rigid_groups().group_count,759u);
}
TEST_F(Cuda,LastGroupFailureAndLateValidationDiscardAllPriorTrialWritesWithExactRetry) {
  Fixture f;fe::FENodalState owner,control;ASSERT_EQ(f.Initialize(owner,f.Config(true)).status,Code::Ok);
  ASSERT_EQ(f.Initialize(control,f.Config(true)).status,Code::Ok);
  const auto g=f.source.model.group_count(),m=f.source.model.member_count();
  Snapshot before(f.n),after(f.n),expected(f.n);std::vector<fe::NodalRigidGroupSnapshot> old_groups(g),new_groups(g);
  for(unsigned epoch=0;epoch<2;++epoch) {
    ASSERT_TRUE(Read(owner,before));fe::NodalStamp prior,current;
    ASSERT_EQ(owner.CopyAcceptedRigidGroups({old_groups.data(),g},&prior).status,Code::Ok);
    fe::NodalTrialToken token;fe::NodalAssemblyView view;ASSERT_TRUE(Begin(owner,m,token,view,true));
    const auto rejected=fe::AdvanceStaggeredRigidGroups(owner,token,Admission(owner,view));
    EXPECT_EQ(rejected.status,Code::InvalidOutput);EXPECT_EQ(rejected.node,f.n-1);
    EXPECT_NE(owner.Commit(token).status,Code::Ok);owner.Discard();
    ASSERT_TRUE(Begin(owner,m,token,view));ASSERT_TRUE(Prepare(owner,token,view));
    EXPECT_EQ(owner.Commit(token).status,Code::MissingCandidateValidation);
    EXPECT_NE(fe::CompleteNodalValidation(owner,token,{view.owner_id,view.accepted.base_epoch,view.attempt,Qualification,false}).status,Code::Ok);
    owner.Discard();ASSERT_TRUE(Read(owner,after));Same(before,after);
    ASSERT_EQ(owner.CopyAcceptedRigidGroups({new_groups.data(),g},&current).status,Code::Ok);
    EXPECT_TRUE(fe::trial_identity::SameStamp(prior,current));SameGroups(old_groups,new_groups);
    ASSERT_TRUE(Begin(owner,m,token,view));ASSERT_TRUE(Prepare(owner,token,view));ASSERT_TRUE(Commit(owner,token,view));
    ASSERT_TRUE(Begin(control,m,token,view));ASSERT_TRUE(Prepare(control,token,view));ASSERT_TRUE(Commit(control,token,view));
    ASSERT_TRUE(Read(owner,after));ASSERT_TRUE(Read(control,expected));Same(after,expected,false);
    ASSERT_EQ(owner.CopyAcceptedRigidGroups({old_groups.data(),g},&prior).status,Code::Ok);
    ASSERT_EQ(control.CopyAcceptedRigidGroups({new_groups.data(),g},&current).status,Code::Ok);SameGroups(old_groups,new_groups);
  }
}
} // namespace vehicle_rigid_owner_test
