// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../type13/NativeOracle.h"
#include "OriginalSource.h"

namespace type13_model_test {
TEST(Type13ModelOriginal, Every4442OriginalBeamKeepsIndependentNativeFrameAndEndpointCoefficients) {
  Source source;t::Model model;
  ASSERT_EQ(original::Nodes[0].id,2000001u);
  ASSERT_TRUE(model.Initialize(source.Input()));
  ASSERT_EQ(model.node_count(),7494u);ASSERT_EQ(model.connection_count(),4442u);
  EXPECT_EQ(model.global_node_count(),7493u);
  long double mass=0;std::vector<bool> owner_nodes(7493,false);
  for(std::size_t e=0;e<model.connection_count();++e) {
    SCOPED_TRACE(e);
    const auto& c=model.connections()[e];const auto& original_beam=original::Beams[e];
    EXPECT_EQ(c.source_id,original_beam.id);
    t::ReferenceInput input;
    for(unsigned local=0;local<3;++local) {
      EXPECT_EQ(c.node[local],original_beam.nodes[local]);
      const auto& n=original::Nodes[original_beam.nodes[local]];
      input.position[local]={n.native[0],n.native[1],n.native[2]};
    }
    const auto reference=type13_test::NativeReference(input);
    const auto native_mass=type13_test::NativeMass(source.property.input.mass_per_length,
        source.property.input.inertia_per_length,reference.length);
    const auto* startup=model.startup(e);ASSERT_NE(startup,nullptr);
    EXPECT_EQ(startup->reference.length_native,reference.length);
    EXPECT_EQ(int(startup->reference.branch),reference.branch);
    for(unsigned axis=0;axis<3;++axis)EXPECT_EQ(startup->reference.axes.v[3*axis+1],reference.y[axis]);
    for(unsigned local=0;local<2;++local) {
      t::EndpointContribution endpoint;ASSERT_TRUE(model.Endpoint(e,local,endpoint));
      const auto& node=original::Nodes[original_beam.nodes[local]];
      ASSERT_LT(endpoint.global_node,owner_nodes.size());owner_nodes[endpoint.global_node]=true;
      EXPECT_EQ(endpoint.source_node_id,node.id);EXPECT_NE(endpoint.source_node_id,2000001u);
      EXPECT_EQ(endpoint.source_element_id,original_beam.id);EXPECT_EQ(endpoint.source_property_id,2000486u);
      // Same exact native/source conversion comparisons as existing startup gate.
      EXPECT_EQ(endpoint.coefficients.mass_kg,native_mass[0]*1000);
      EXPECT_EQ(endpoint.coefficients.isotropic_inertia_kg_m2,native_mass[1]*.001);
      EXPECT_EQ(endpoint.coefficients.added_inertia_kg_m2,0);
      mass+=endpoint.coefficients.mass_kg;
    }
  }
  for(bool used:owner_nodes)EXPECT_TRUE(used);
  EXPECT_GT(mass,1.61L);EXPECT_LT(mass,1.62L);
  EXPECT_EQ(model.nodes()[0].global_node,SIZE_MAX);
  RecordProperty("owned_payload_bytes",std::to_string(model.owned_payload_bytes()));
  RecordProperty("startup_payload_bytes",std::to_string(model.startup_payload_bytes()));
}
TEST(Type13ModelOriginal, FinalOriginalConnectionFailureAndExactBudgetRetryPreserveCompleteScope) {
  Source source;t::Model reference;ASSERT_TRUE(reference.Initialize(source.Input()));
  t::ModelLimits limits;limits.max_host_bytes=reference.startup_payload_bytes();
  const auto old=source.connections.back();source.connections.back().node[1]=SIZE_MAX;
  t::Model retry;const auto rejected=retry.Initialize(source.Input(),limits);
  EXPECT_EQ(rejected.status,t::ModelStatus::InvalidInput);EXPECT_EQ(rejected.entry,4441u);
  EXPECT_FALSE(retry.prepared());source.connections.back()=old;
  --limits.max_host_bytes;EXPECT_EQ(retry.Initialize(source.Input(),limits).status,t::ModelStatus::ResourceLimit);
  ++limits.max_host_bytes;ASSERT_TRUE(retry.Initialize(source.Input(),limits));
  EXPECT_TRUE(retry.Matches(reference));EXPECT_FALSE(retry.SharesStorage(reference));
}
} // namespace type13_model_test
