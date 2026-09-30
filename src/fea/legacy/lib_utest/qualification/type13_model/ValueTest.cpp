// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <cmath>
#include <limits>

namespace type13_model_test {
TEST(Type13Model, OwnsOriginalInputsCurvesAndIndependentCopiedLifetime) {
  t::Model model;
  {
    Fixture f;
    ASSERT_TRUE(model.Initialize(f.Input()));
    EXPECT_NE(model.property_declaration(0)->input.curves[0].points,f.material.points[0]);
    f.material.points[0][0].y=1234;
    f.nodes[0].position_native.x=9;
  }
  const auto copy=model;
  EXPECT_TRUE(copy.SharesStorage(model));
  Fixture same;t::Model independent;
  ASSERT_TRUE(independent.Initialize(same.Input()));
  EXPECT_TRUE(model.Matches(independent));
  EXPECT_FALSE(model.SharesStorage(independent));
  EXPECT_EQ(copy.nodes()[0].position_native.x,1200);
  EXPECT_EQ(copy.connection_count(),2u);
  EXPECT_EQ(copy.node_count(),4u);
  EXPECT_EQ(copy.property_count(),1u);
  EXPECT_NE(copy.property(0),nullptr);
  EXPECT_EQ(copy.property(1),nullptr);
  EXPECT_EQ(copy.startup(2),nullptr);
  EXPECT_EQ(model.Initialize(same.Input()).status,t::ModelStatus::AlreadyInitialized);
}
TEST(Type13Model, SharedEndpointContributesOnceForEachSourceElementWithoutN3Mass) {
  Fixture f;t::Model model;ASSERT_TRUE(model.Initialize(f.Input()));
  t::EndpointContribution a,b;
  ASSERT_TRUE(model.Endpoint(0,1,a));ASSERT_TRUE(model.Endpoint(1,0,b));
  EXPECT_EQ(a.source_node_id,20u);EXPECT_EQ(b.source_node_id,20u);
  EXPECT_EQ(a.global_node,b.global_node);
  EXPECT_EQ(a.source_element_id,100u);EXPECT_EQ(b.source_element_id,200u);
  EXPECT_EQ(a.source_property_id,50u);
  EXPECT_GT(a.coefficients.mass_kg,0);EXPECT_GT(b.coefficients.mass_kg,0);
  EXPECT_NE(a.coefficients.mass_kg,b.coefficients.mass_kg);
  EXPECT_EQ(model.nodes()[3].global_node,SIZE_MAX);
  const auto saved=a;
  EXPECT_FALSE(model.Endpoint(0,2,a));EXPECT_FALSE(model.Endpoint(2,0,a));
  EXPECT_EQ(a.source_element_id,saved.source_element_id);
  EXPECT_EQ(a.coefficients.mass_kg,saved.coefficients.mass_kg);
}
TEST(Type13Model, LateCollapsedReferencePreservesEmptyModelAndAllowsExactRetry) {
  Fixture f;t::Model model;
  const auto saved=f.nodes[2];f.nodes[2].position_native=f.nodes[1].position_native;
  const auto rejected=model.Initialize(f.Input());
  EXPECT_EQ(rejected.status,t::ModelStatus::NumericalFailure);
  EXPECT_EQ(rejected.kind,t::ModelEntry::Connection);EXPECT_EQ(rejected.entry,1u);
  EXPECT_EQ(rejected.native_status,t::Status::DegenerateGeometry);
  EXPECT_FALSE(model.prepared());EXPECT_EQ(model.nodes(),nullptr);
  EXPECT_EQ(model.owned_payload_bytes(),sizeof(t::Model));
  f.nodes[2]=saved;ASSERT_TRUE(model.Initialize(f.Input()));
  EXPECT_EQ(model.connections()[1].source_id,200u);
}
TEST(Type13Model, ExplicitOwnerIdentityRejectsAliasesAndReferenceOnlyEndpoints) {
  Fixture f;t::Model duplicate_node;
  f.nodes[2].source_id=20;
  EXPECT_EQ(duplicate_node.Initialize(f.Input()).status,t::ModelStatus::DuplicateIdentity);
  f.nodes[2].source_id=30;f.nodes[2].global_node=1;
  EXPECT_EQ(duplicate_node.Initialize(f.Input()).status,t::ModelStatus::DuplicateIdentity);
  f.nodes[2].global_node=2;
  f.connections.push_back({300,0,{3,2,0}});
  const auto absent=duplicate_node.Initialize(f.Input());
  EXPECT_EQ(absent.status,t::ModelStatus::InvalidInput);EXPECT_EQ(absent.entry,2u);
  f.nodes[3].global_node=3;auto input=f.Input();input.global_node_count=4;
  ASSERT_TRUE(duplicate_node.Initialize(input));
  t::EndpointContribution endpoint;ASSERT_TRUE(duplicate_node.Endpoint(2,0,endpoint));
  EXPECT_EQ(endpoint.source_node_id,99u);EXPECT_EQ(endpoint.global_node,3u);
}
TEST(Type13Model, ExactStartupBudgetAndCountRangePreflightBeforeBorrowedReads) {
  Fixture f;t::Model reference;ASSERT_TRUE(reference.Initialize(f.Input()));
  t::ModelLimits limits;limits.max_host_bytes=reference.startup_payload_bytes()-1;
  t::Model model;auto input=f.Input();
  input.nodes=reinterpret_cast<const t::ModelNode*>(1);
  EXPECT_EQ(model.Initialize(input,limits).status,t::ModelStatus::ResourceLimit);
  limits.max_host_bytes++;
  EXPECT_EQ(model.Initialize(input,limits).status,t::ModelStatus::InvalidInput);
  ASSERT_TRUE(model.Initialize(f.Input(),limits));
  EXPECT_TRUE(model.Matches(reference));
  t::Model invalid;input=f.Input();input.connection_count=SIZE_MAX;
  input.connections=reinterpret_cast<const t::ModelConnection*>(1);
  EXPECT_EQ(invalid.Initialize(input).status,t::ModelStatus::ResourceLimit);
  f.declaration.input.curves[3].points=reinterpret_cast<const t::CurvePoint*>(UINTPTR_MAX-7);
  EXPECT_EQ(invalid.Initialize(f.Input()).status,t::ModelStatus::InvalidInput);
}
TEST(Type13Model, CompleteDeclarationsRejectLateUnusedAndDuplicateSourceEntries) {
  Fixture f;t::Model model;
  f.connections[1].source_id=f.connections[0].source_id;
  auto result=model.Initialize(f.Input());
  EXPECT_EQ(result.status,t::ModelStatus::DuplicateIdentity);EXPECT_EQ(result.entry,1u);
  f.connections[1].source_id=200;
  f.nodes.push_back({999,SIZE_MAX,{1,2,3}});
  result=model.Initialize(f.Input());
  EXPECT_EQ(result.status,t::ModelStatus::InvalidInput);EXPECT_EQ(result.kind,t::ModelEntry::Node);
  EXPECT_EQ(result.entry,4u);f.nodes.pop_back();
  t::ModelPropertyInput properties[]{f.declaration,{51,f.declaration.input}};
  auto input=f.Input();input.properties=properties;input.property_count=2;
  result=model.Initialize(input);
  EXPECT_EQ(result.status,t::ModelStatus::InvalidInput);EXPECT_EQ(result.kind,t::ModelEntry::Property);
  properties[1].source_id=50;
  EXPECT_EQ(model.Initialize(input).status,t::ModelStatus::DuplicateIdentity);
  properties[1].source_id=51;properties[1].input.channels[5].hysteresis=2;
  result=model.Initialize(input);
  EXPECT_EQ(result.native_status,t::Status::UnsupportedScope);EXPECT_EQ(result.entry,1u);
  ASSERT_TRUE(model.Initialize(f.Input()));
}
TEST(Type13Model, RawInertiaAndOrientationIdentityRemainDistinctAfterNativeFlooring) {
  Fixture a,b;
  a.declaration.input.inertia_per_length=0;b.declaration.input.inertia_per_length=1e-40;
  t::Model first,second;ASSERT_TRUE(first.Initialize(a.Input()));ASSERT_TRUE(second.Initialize(b.Input()));
  EXPECT_EQ(first.property(0)->inertia_per_length(),second.property(0)->inertia_per_length());
  EXPECT_FALSE(first.Matches(second));
  EXPECT_EQ(first.property_declaration(0)->input.inertia_per_length,0);
  EXPECT_EQ(second.property_declaration(0)->input.inertia_per_length,1e-40);
  EXPECT_GT(first.startup(0)->endpoint.added_inertia_kg_m2,0);
  EXPECT_EQ(first.startup(0)->endpoint.isotropic_inertia_kg_m2,first.startup(0)->endpoint.added_inertia_kg_m2);
  Fixture c,d;d.nodes[3].source_id=1000;
  t::Model third,fourth;ASSERT_TRUE(third.Initialize(c.Input()));ASSERT_TRUE(fourth.Initialize(d.Input()));
  EXPECT_FALSE(third.Matches(fourth));
}
} // namespace type13_model_test
