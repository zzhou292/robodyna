// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace rigid_binding_test {
namespace {
struct EmptySource {
  qbat_binding_test::Fixture input;
  fe::ShellBatchBinding shells;
  fe::NodalNodeDomain domain;
  fe::ShellNodeMap mapping;
  fe::NodalCoefficientLedger ledger;
  EmptySource() {
    EXPECT_EQ(shells.InitializeFormulations(input.Input()).status,fe::ShellBindingStatus::Success);
    std::vector<fe::NodalDomainNode> nodes;
    for(std::size_t i=0;i<shells.node_count();++i)nodes.push_back({shells.nodes()[i].source_id,shells.nodes()[i].position});
    EXPECT_TRUE(domain.Initialize({901,nodes.data(),nodes.size()}));
    EXPECT_TRUE(mapping.Initialize(shells,domain));EXPECT_TRUE(ledger.Initialize({&mapping}));
  }
};
}
TEST(RigidAssemblyBinding, ExplicitEmptyKeepsCompleteLedgerWithoutInventedPartOrMembers) {
  EmptySource f;fe::NodalRigidAssemblyBinding binding;
  ASSERT_TRUE(binding.InitializeEmpty(f.ledger));EXPECT_TRUE(binding.explicitly_empty());
  EXPECT_TRUE(binding.prepared());EXPECT_EQ(binding.parts(),nullptr);
  EXPECT_EQ(binding.groups().size(),0);EXPECT_EQ(binding.groups().data(),nullptr);
  EXPECT_EQ(binding.members().size(),0);EXPECT_EQ(binding.members().data(),nullptr);
  EXPECT_TRUE(binding.coefficients()->Matches(f.ledger));EXPECT_TRUE(binding.domain()->SharesStorage(f.domain));
  EXPECT_EQ(binding.FindMember(0),nullptr);EXPECT_EQ(binding.FindMember(SIZE_MAX),nullptr);
  EXPECT_EQ(binding.plain_source_instance_id(),0);
  EXPECT_EQ(binding.InitializeEmpty(f.ledger).status,S::AlreadyInitialized);
  auto copy=binding;EXPECT_TRUE(copy.explicitly_empty());EXPECT_EQ(copy.coefficients(),binding.coefficients());
}
TEST(RigidAssemblyBinding, EmptyRejectsMissingLedgerAndUncoveredSourceThenRetries) {
  EmptySource good;fe::NodalCoefficientLedger absent;fe::NodalRigidAssemblyBinding binding;
  EXPECT_EQ(binding.InitializeEmpty(absent).status,S::InvalidInput);EXPECT_FALSE(binding.prepared());
  coefficient_test::Fixture extra;auto domain=extra.Domain();auto mapping=extra.Map(domain);
  fe::NodalCoefficientLedger uncovered;ASSERT_TRUE(uncovered.Initialize({&mapping}));
  ASSERT_GT(uncovered.scope().uncovered_nodes,0u);
  EXPECT_EQ(binding.InitializeEmpty(uncovered).status,S::InvalidInput);EXPECT_FALSE(binding.prepared());
  ASSERT_TRUE(binding.InitializeEmpty(good.ledger));
}
TEST(RigidAssemblyBinding, EmptyExactHostBudgetAndZeroGroupCapacityRetainBacking) {
  fe::NodalRigidAssemblyBinding retained;
  {
    EmptySource f;ASSERT_TRUE(retained.InitializeEmpty(f.ledger));
    fe::RigidBindingLimits cap;cap.max_groups=cap.max_members=cap.max_members_per_group=0;
    cap.max_host_bytes=retained.owned_payload_bytes()-1;fe::NodalRigidAssemblyBinding retry;
    EXPECT_EQ(retry.InitializeEmpty(f.ledger,cap).status,S::ResourceLimit);EXPECT_FALSE(retry.prepared());
    ++cap.max_host_bytes;ASSERT_TRUE(retry.InitializeEmpty(f.ledger,cap));
    EXPECT_EQ(retry.startup_payload_bytes(),cap.max_host_bytes);
    fe::NodalRigidAssemblyBinding short_nodes;cap.max_nodes=f.domain.node_count()-1;
    EXPECT_EQ(short_nodes.InitializeEmpty(f.ledger,cap).status,S::ResourceLimit);
  }
  EXPECT_TRUE(retained.domain()->prepared());EXPECT_TRUE(retained.coefficients()->prepared());
  EXPECT_GT(retained.coefficients()->totals().mass,0.);
}
TEST(RigidAssemblyBinding, LegacyNonemptyCannotBeRelabeledEmpty) {
  Fixture f;auto parts=f.Parts();auto plain=f.Plain();fe::NodalRigidAssemblyBinding binding;
  ASSERT_TRUE(binding.Initialize(parts,plain.get()));EXPECT_FALSE(binding.explicitly_empty());
  EXPECT_EQ(binding.InitializeEmpty(f.solid.ledger).status,S::AlreadyInitialized);
  EXPECT_EQ(binding.groups().size(),2u);EXPECT_NE(binding.parts(),nullptr);
}
} // namespace rigid_binding_test
