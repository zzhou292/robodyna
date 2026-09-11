#include "Fixture.h"
#include "../qbat_binding/Fixture.h"
#include <algorithm>
#include <type_traits>

namespace nodal_domain_test {
static_assert(std::is_nothrow_copy_constructible_v<fe::ShellNodeMap>);
static_assert(!std::is_copy_assignable_v<fe::ShellNodeMap>);

TEST(ShellNodeMap, IdentityInterleavedExtrasAndExactNativeBindingPreservation) {
  const auto binding=Binding(); auto nodes=Nodes(binding);
  auto identity_domain=Domain(nodes); fe::ShellNodeMap identity;
  ASSERT_EQ(identity.Initialize(binding,identity_domain).status,S::Success);
  EXPECT_TRUE(identity.identity_map());
  for(std::size_t i=0;i<nodes.size();++i) EXPECT_EQ(identity.owner_index(i),i);
  nodes.insert(nodes.begin(),{777,{99,98,97}});
  nodes.insert(nodes.begin()+3,{778,{0,0,0}});
  auto interleaved=Domain(nodes); fe::ShellNodeMap map;
  ASSERT_EQ(map.Initialize(binding,interleaved).status,S::Success);
  const std::size_t expected[]{1,2,4,5,6};
  EXPECT_FALSE(map.identity_map()); EXPECT_EQ(map.owner_node_count(),7); EXPECT_EQ(map.shell_node_count(),5);
  for(std::size_t i=0;i<5;++i) {
    EXPECT_EQ(map.owner_index(i),expected[i]);
    EXPECT_EQ(map.mapping()[i],expected[i]);
    shell_binding_test::Exact(map.shells()->nodes()[i].native,binding.nodes()[i].native);
  }
  EXPECT_EQ(map.owner_index(5),SIZE_MAX); EXPECT_EQ(map.owner_index(SIZE_MAX),SIZE_MAX);
  EXPECT_EQ(map.shells()->inventory(),binding.inventory());
  EXPECT_TRUE(map.Matches(binding,interleaved)); EXPECT_FALSE(map.Matches(identity));
  EXPECT_FALSE(map.Matches(binding,identity_domain));
  // The original strict local shell binding still requires complete coverage.
  auto uncovered=shell_binding_test::Edge(); ++uncovered.node_count;
  fe::ShellBatchBinding rejected;
  EXPECT_NE(rejected.Initialize(uncovered).status,fe::ShellBindingStatus::Success);
}

TEST(ShellNodeMap, MissingLastNodeAndSignedZeroRejectAtomicallyThenRetry) {
  const auto binding=Binding(); auto nodes=Nodes(binding);
  fe::ShellNodeMap map; const auto before=Bytes(map);
  nodes.back().source_id+=1;
  auto missing=Domain(nodes); auto report=map.Initialize(binding,missing);
  EXPECT_EQ(report.status,S::MissingSource); EXPECT_EQ(report.node,4); EXPECT_EQ(Bytes(map),before);
  nodes=Nodes(binding); nodes.back().position.z=-0.;
  auto wrong_zero=Domain(nodes); report=map.Initialize(binding,wrong_zero);
  EXPECT_EQ(report.status,S::PositionMismatch); EXPECT_EQ(report.node,4); EXPECT_EQ(Bytes(map),before);
  nodes.back().position.z=std::nextafter(0.,1.);
  auto wrong_bit=Domain(nodes); report=map.Initialize(binding,wrong_bit);
  EXPECT_EQ(report.status,S::PositionMismatch); EXPECT_EQ(report.node,4); EXPECT_EQ(Bytes(map),before);
  auto domain=Domain(Nodes(binding));
  ASSERT_EQ(map.Initialize(binding,domain).status,S::Success);
  fe::ShellNodeMap clean; ASSERT_EQ(clean.Initialize(binding,domain).status,S::Success);
  EXPECT_TRUE(map.Matches(clean));
}

TEST(ShellNodeMap, LifetimeCompleteInputIdentityAndBudget) {
  const auto binding=Binding(); auto domain=Domain(Nodes(binding));
  fe::ShellNodeMap map; ASSERT_EQ(map.Initialize(binding,domain).status,S::Success);
  const auto bytes=map.owned_payload_bytes(); EXPECT_EQ(bytes,map.startup_payload_bytes());
  EXPECT_GT(bytes,binding.host_bytes()+domain.owned_payload_bytes());
  fe::ShellNodeMap retry; const auto empty=Bytes(retry);
  EXPECT_EQ(retry.Initialize(binding,domain,{5,bytes-1}).status,S::ResourceLimit);
  EXPECT_EQ(retry.Initialize(binding,domain,{4,bytes}).status,S::ResourceLimit);
  EXPECT_EQ(Bytes(retry),empty);
  ASSERT_EQ(retry.Initialize(binding,domain,{5,bytes}).status,S::Success);
  EXPECT_TRUE(retry.Matches(map));
  auto changed_input=shell_binding_test::Edge(); changed_input.qeph.density*=2;
  fe::ShellBatchBinding changed;
  ASSERT_EQ(changed.Initialize(changed_input).status,fe::ShellBindingStatus::Success);
  EXPECT_FALSE(map.Matches(changed,domain)); // Same NID/positions, different original material coefficient.
  const auto make=[] {
    auto b=Binding(); auto d=Domain(Nodes(b)); fe::ShellNodeMap result;
    EXPECT_EQ(result.Initialize(b,d).status,S::Success); return result;
  };
  const auto surviving=make();
  EXPECT_TRUE(map.Matches(surviving)); EXPECT_EQ(surviving.owner_index(4),4);
  fe::ShellNodeMap copied(surviving),moved(std::move(copied));
  EXPECT_TRUE(copied.prepared()); EXPECT_TRUE(moved.Matches(map));
  const auto saved=Bytes(retry); fe::ShellBatchBinding unprepared; fe::NodalNodeDomain no_domain;
  EXPECT_EQ(retry.Initialize(unprepared,no_domain).status,S::AlreadyInitialized);
  EXPECT_EQ(Bytes(retry),saved);
  fe::ShellNodeMap invalid;
  EXPECT_EQ(invalid.Initialize(unprepared,domain).status,S::InvalidInput);
  EXPECT_EQ(invalid.Initialize(binding,no_domain).status,S::InvalidInput);
  EXPECT_EQ(invalid.shells(),nullptr); EXPECT_EQ(invalid.domain(),nullptr);
}

TEST(ShellNodeMap, CompleteQepht3QbatLayersAndPlacementRemainDistinct) {
  qbat_binding_test::Fixture source;
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(source.Input()).status,fe::ShellBindingStatus::Success);
  auto nodes=Nodes(binding); std::reverse(nodes.begin(),nodes.end());
  auto domain=Domain(nodes); fe::ShellNodeMap map;
  ASSERT_EQ(map.Initialize(binding,domain).status,S::Success);
  for(std::size_t i=0;i<nodes.size();++i) EXPECT_EQ(map.owner_index(i),nodes.size()-1-i);
  ASSERT_NE(map.shells(),nullptr);
  EXPECT_EQ(map.shells()->qeph_count(),2); EXPECT_EQ(map.shells()->t3_count(),1);
  EXPECT_EQ(map.shells()->qbat_count(),1);
  qbat_binding_test::Reduction(*map.shells());
  source.q[0].reference.placement=fe::ShellReferencePlacement::Centered;
  fe::ShellBatchBinding placement_changed;
  ASSERT_EQ(placement_changed.InitializeFormulations(source.Input()).status,fe::ShellBindingStatus::Success);
  EXPECT_FALSE(map.Matches(placement_changed,domain));
  source.q[0].reference.placement=fe::ShellReferencePlacement::TopReferencePlane;
  source.b.source_parent_id+=1000;
  fe::ShellBatchBinding parent_changed;
  ASSERT_EQ(parent_changed.InitializeFormulations(source.Input()).status,fe::ShellBindingStatus::Success);
  EXPECT_FALSE(map.Matches(parent_changed,domain));
}
} // namespace nodal_domain_test
