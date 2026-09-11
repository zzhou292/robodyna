// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include <algorithm>

namespace rigid_assembly_owner_test {
namespace {
fe::NodalRigidAssemblyBinding ForeignPlain(const Fixture& fixture, bool reverse) {
  const auto& group=fixture.plain.groups()[0];
  std::vector<fe::NodalRigidGroupMember> members;
  for (std::size_t i=0;i<group.member_count;++i)
    members.push_back(fixture.plain.members()[group.member_offset+i]);
  if (reverse) std::reverse(members.begin(),members.end());
  const fe::NodalRigidGroupInput input{group.source_group_id,
      group.source_node_set_id+(reverse?0:1),members.data(),members.size()};
  fe::NodalRigidGroupModel plain;
  EXPECT_TRUE(plain.Initialize({fixture.plain.source_instance_id(),fixture.domain.node_count(),
      &input,1,fixture.plain.source_units()}));
  fe::NodalRigidAssemblyBinding result;
  EXPECT_TRUE(result.Initialize(fixture.parts,&plain));
  return result;
}
}
TEST_F(Cuda, RigidAssemblyIdentityChecksCompleteSourceBeyondMatchingCounts) {
  Fixture fixture;
  fe::FENodalState owner;
  EXPECT_EQ(owner.ValidateRigidAssemblyBinding(fixture.binding).status,Code::NotInitialized);
  ASSERT_EQ(Initialize(owner,fixture).status,Code::Ok);
  fe::NodalRigidAssemblyBinding empty;
  EXPECT_EQ(owner.ValidateRigidAssemblyBinding(empty).status,Code::InvalidInput);
  ASSERT_EQ(owner.ValidateRigidAssemblyBinding(fixture.binding).status,Code::Ok);
  const auto copied=fixture.binding;
  EXPECT_EQ(owner.ValidateRigidAssemblyBinding(copied).status,Code::Ok);
  // Both sources have identical declared source IDs and complete counts.
  // One differs only in set identity; the other preserves IDs but changes the
  // original two-member traversal. Neither may masquerade as the actual owner.
  const auto changed_set=ForeignPlain(fixture,false);
  const auto changed_order=ForeignPlain(fixture,true);
  EXPECT_EQ(owner.ValidateRigidAssemblyBinding(changed_set).status,Code::InvalidInput);
  EXPECT_EQ(owner.ValidateRigidAssemblyBinding(changed_order).status,Code::InvalidInput);
  EXPECT_EQ(owner.ValidateRigidAssemblyBinding(fixture.binding).status,Code::Ok);
  EXPECT_EQ(owner.accepted().epoch,0u);
}
TEST_F(Cuda, RigidAssemblyIdentitySurvivesNativeMotionCinTransferAndDiscard) {
  Fixture fixture;
  fe::FENodalState owner;
  ASSERT_EQ(Initialize(owner,fixture,true,true).status,Code::Ok);
  const auto allocation=owner.allocations();
  for (unsigned step=0;step<3;++step) {
    SCOPED_TRACE(step);
    ASSERT_EQ(owner.ValidateRigidAssemblyBinding(fixture.binding).status,Code::Ok);
    fe::NodalTrialToken token;
    fe::NodalAssemblyView view;
    Begin(owner,fixture,Loads(fixture,step),true,token,view);
    ASSERT_EQ(Advance(owner,token,view,true).status,Code::Ok);
    EXPECT_EQ(owner.ValidateRigidAssemblyBinding(fixture.binding).status,Code::Ok);
    if (step==1) {
      owner.Discard();
      EXPECT_EQ(owner.ValidateRigidAssemblyBinding(fixture.binding).status,Code::Ok);
      Begin(owner,fixture,Loads(fixture,step),true,token,view);
      ASSERT_EQ(Advance(owner,token,view,true).status,Code::Ok);
    }
    Commit(owner,token,view);
    EXPECT_EQ(owner.accepted().epoch,step+1);
  }
  EXPECT_EQ(owner.ValidateRigidAssemblyBinding(fixture.binding).status,Code::Ok);
  EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
}
} // namespace rigid_assembly_owner_test
