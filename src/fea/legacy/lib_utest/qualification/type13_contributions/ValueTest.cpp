#include "Fixture.h"
#include <type_traits>

namespace type13_contribution_test {
static_assert(std::is_nothrow_copy_constructible_v<fe::Type13NodeContributions>);
static_assert(!std::is_copy_assignable_v<fe::Type13NodeContributions>);

TEST(Type13Contributions, SharedEndpointsRetainModelOrderAndNumericalFloorOnce) {
  Fixture f; f.connections[0].source_id=900; f.declaration.input.inertia_per_length=0;
  const auto model=Model(f); const auto domain=Domain(Nodes(model));
  fe::Type13NodeContributions adapter;
  ASSERT_TRUE(adapter.Initialize(model,domain)); Check(adapter,model);
  EXPECT_EQ(adapter.phase(),fe::Type13ContributionPhase::PreparedEndpointSI);
  EXPECT_EQ(adapter.order(),fe::Type13ContributionOrder::ModelThenEndpoint);
  EXPECT_GT(adapter.records()[0].value.source_element_id,adapter.records()[2].value.source_element_id);
  EXPECT_EQ(adapter.records()[1].value.global_node,adapter.records()[2].value.global_node);
  for(const auto& row:adapter.records()) {
    EXPECT_NE(row.value.source_node_id,99u);
    EXPECT_GT(row.value.coefficients.isotropic_inertia_kg_m2,0);
    EXPECT_EQ(Bytes(row.value.coefficients.isotropic_inertia_kg_m2),Bytes(row.value.coefficients.added_inertia_kg_m2));
  }
  EXPECT_EQ(adapter.model()->property_declaration(0)->input.inertia_per_length,0);
}

TEST(Type13Contributions, InterleavedExtrasAndMappedOrientationRemainSeparate) {
  Fixture f; f.nodes[0].global_node=2; f.nodes[1].global_node=0; f.nodes[2].global_node=3;
  f.nodes[3].global_node=1; // N3 is physically present through a different role.
  auto input=f.Input(); input.global_node_count=5;
  t::Model model; ASSERT_TRUE(model.Initialize(input));
  const auto domain=Domain(Nodes(model)); fe::Type13NodeContributions adapter;
  ASSERT_TRUE(adapter.Initialize(model,domain)); Check(adapter,model);
  for(const auto& row:adapter.records()) EXPECT_NE(row.value.global_node,1u);
  EXPECT_EQ(adapter.model()->nodes()[3].global_node,1u);
  EXPECT_EQ(adapter.domain()->node_count(),5);
}

TEST(Type13Contributions, MappedOrUnindexedOrientationChecksOneWaySignedZero) {
  Fixture f; f.nodes[3].position_native={1000.1234,-0.,700.4321}; f.nodes[3].global_node=3;
  auto input=f.Input(); input.global_node_count=4;
  t::Model model; ASSERT_TRUE(model.Initialize(input));
  auto nodes=Nodes(model); auto exact=Domain(nodes); fe::Type13NodeContributions adapter;
  const auto before=Bytes(adapter); nodes[3].position.y=0.;
  auto wrong_zero=Domain(nodes); auto report=adapter.Initialize(model,wrong_zero);
  EXPECT_EQ(report.status,S::PositionMismatch); EXPECT_EQ(report.node,3); EXPECT_EQ(Bytes(adapter),before);
  nodes=Nodes(model); nodes[3].position.x=std::nextafter(nodes[3].position.x,2.);
  auto wrong_position=Domain(nodes); EXPECT_EQ(adapter.Initialize(model,wrong_position).status,S::PositionMismatch);
  ASSERT_TRUE(adapter.Initialize(model,exact)); Check(adapter,model);
  f.nodes[3].global_node=SIZE_MAX;
  t::Model unindexed; ASSERT_TRUE(unindexed.Initialize(input));
  fe::Type13NodeContributions reference_only;
  ASSERT_TRUE(reference_only.Initialize(unindexed,exact)); Check(reference_only,unindexed);
  EXPECT_EQ(reference_only.model()->nodes()[3].global_node,SIZE_MAX);
  fe::Type13NodeContributions bad;
  EXPECT_EQ(bad.Initialize(unindexed,wrong_zero).status,S::PositionMismatch);
}

TEST(Type13Contributions, LastEndpointIdentityPositionScopeAndRetry) {
  const Fixture f; const auto model=Model(f); auto nodes=Nodes(model);
  fe::Type13NodeContributions adapter; const auto before=Bytes(adapter);
  nodes[2].source_id+=100;
  auto bad_id=Domain(nodes); auto report=adapter.Initialize(model,bad_id);
  EXPECT_EQ(report.status,S::MissingSource); EXPECT_EQ(report.node,2); EXPECT_EQ(Bytes(adapter),before);
  nodes=Nodes(model); nodes[2].position.z=std::nextafter(nodes[2].position.z,1.);
  auto bad_position=Domain(nodes); report=adapter.Initialize(model,bad_position);
  EXPECT_EQ(report.status,S::PositionMismatch); EXPECT_EQ(report.node,2); EXPECT_EQ(Bytes(adapter),before);
  auto other_source=Domain(Nodes(model),8);
  EXPECT_EQ(adapter.Initialize(model,other_source).status,S::InvalidInput);
  nodes=Nodes(model); nodes.push_back({999,{0,0,0}}); auto longer=Domain(nodes);
  EXPECT_EQ(adapter.Initialize(model,longer).status,S::InvalidInput);
  nodes=Nodes(model); std::swap(nodes[0],nodes[2]); auto wrong_index=Domain(nodes);
  EXPECT_EQ(adapter.Initialize(model,wrong_index).status,S::MissingSource);
  const auto domain=Domain(Nodes(model)); ASSERT_TRUE(adapter.Initialize(model,domain)); Check(adapter,model);
  fe::Type13NodeContributions clean; ASSERT_TRUE(clean.Initialize(model,domain)); EXPECT_TRUE(adapter.Matches(clean));
}

TEST(Type13Contributions, CompleteRetainedBudgetLifetimeAndExactModelIdentity) {
  Fixture f; const auto model=Model(f); const auto domain=Domain(Nodes(model));
  fe::Type13NodeContributions adapter; ASSERT_TRUE(adapter.Initialize(model,domain));
  const auto bytes=adapter.owned_payload_bytes(); EXPECT_EQ(adapter.startup_payload_bytes(),bytes);
  EXPECT_GT(bytes,model.owned_payload_bytes()+domain.owned_payload_bytes());
  fe::Type13ContributionLimits limits; limits.max_host_bytes=bytes-1;
  fe::Type13NodeContributions retry; const auto before=Bytes(retry);
  EXPECT_EQ(retry.Initialize(model,domain,limits).status,S::ResourceLimit); EXPECT_EQ(Bytes(retry),before);
  ++limits.max_host_bytes; limits.max_connections=1;
  EXPECT_EQ(retry.Initialize(model,domain,limits).status,S::ResourceLimit);
  limits.max_connections=2; ASSERT_TRUE(retry.Initialize(model,domain,limits)); EXPECT_TRUE(retry.Matches(adapter));
  const auto surviving=[] {
    Fixture local; const auto m=Model(local); const auto d=Domain(Nodes(m));
    fe::Type13NodeContributions result; EXPECT_TRUE(result.Initialize(m,d)); return result;
  }();
  EXPECT_TRUE(surviving.Matches(adapter)); Check(surviving,*surviving.model());
  fe::Type13NodeContributions copy(surviving),moved(std::move(copy));
  EXPECT_TRUE(copy.prepared()); EXPECT_TRUE(moved.Matches(adapter));
  f.nodes[3].source_id+=100; const auto changed=Model(f);
  EXPECT_FALSE(adapter.Matches(changed,domain));
  const auto saved=Bytes(retry); t::Model empty; fe::NodalNodeDomain empty_domain;
  EXPECT_EQ(retry.Initialize(empty,empty_domain).status,S::AlreadyInitialized); EXPECT_EQ(Bytes(retry),saved);
  fe::Type13NodeContributions unprepared;
  EXPECT_EQ(unprepared.Initialize(empty,domain).status,S::InvalidInput);
  EXPECT_EQ(unprepared.Initialize(model,empty_domain).status,S::InvalidInput);
  EXPECT_EQ(unprepared.records().size(),0);
}
} // namespace type13_contribution_test
