// SPDX-License-Identifier: AGPL-3.0-or-later
#include "GroupOwnerFixture.h"

namespace rigid_owner_test {
class NonRigidNodeQuery:public Cuda {};
TEST_F(NonRigidNodeQuery,ActualMembershipRejectsEachGroupAndAdmitsOrdinaryNode) {
  Fixture f;fe::FENodalState owner;ASSERT_EQ(f.Initialize(owner).status,Code::Ok);
  const auto stamp=owner.accepted();const auto allocation=owner.allocations();
  const std::size_t ordinary=f.input.n-1;
  EXPECT_EQ(owner.ValidateNonRigidNodes(&ordinary,1).status,Code::Ok);
  for(std::size_t node=0;node<ordinary;++node) {
    const std::size_t query[2]={ordinary,node};const auto report=owner.ValidateNonRigidNodes(query,2);
    EXPECT_EQ(report.status,Code::InvalidInput);EXPECT_EQ(report.node,node);
  }
  EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,owner.accepted()));
  EXPECT_EQ(allocation.device_bytes,owner.allocations().device_bytes);
  EXPECT_EQ(allocation.device_allocations,owner.allocations().device_allocations);
}
TEST_F(NonRigidNodeQuery,BoundedQueriesDoNotConsumeOrAlterAnActualTrial) {
  Fixture f;fe::FENodalState owner;ASSERT_EQ(f.Initialize(owner).status,Code::Ok);
  fe::NodalTrialToken token;fe::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,Code::Ok);
  EXPECT_EQ(owner.ValidateNonRigidNodes(reinterpret_cast<const std::size_t*>(1),SIZE_MAX).status,Code::ResourceLimit);
  EXPECT_EQ(owner.ValidateNonRigidNodes(nullptr,1).status,Code::InvalidInput);
  const std::size_t out_of_range=f.input.n;
  EXPECT_EQ(owner.ValidateNonRigidNodes(&out_of_range,1).status,Code::InvalidInput);
  const std::size_t member=0,ordinary=f.input.n-1;
  EXPECT_EQ(owner.ValidateNonRigidNodes(&member,1).status,Code::InvalidInput);
  EXPECT_EQ(owner.ValidateNonRigidNodes(&ordinary,1).status,Code::Ok);
  EXPECT_EQ(owner.ValidateAcceptedAssemblySources(view).status,Code::Ok);
  EXPECT_EQ(owner.SealAssembly(token).status,Code::Ok);
  owner.Discard();
}
TEST_F(NonRigidNodeQuery,UninitializedAndUngroupedOwnersKeepTheirOwnScope) {
  fe::FENodalState owner;const std::size_t nodes[2]={0,1};
  EXPECT_EQ(owner.ValidateNonRigidNodes(nodes,2).status,Code::NotInitialized);
  nt::Initial input;input.n=2;ASSERT_EQ(input.Initialize(owner).status,Code::Ok);
  EXPECT_EQ(owner.ValidateNonRigidNodes(nodes,2).status,Code::Ok);
  EXPECT_EQ(owner.ValidateNonRigidNodes(nodes,0).status,Code::InvalidInput);
}
} // namespace rigid_owner_test
