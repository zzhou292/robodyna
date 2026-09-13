// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <cstring>
#include <limits>

namespace active_use_test {
TEST(SelfContactActiveUses, CheckedCountsRejectOverflowAndPreserveOutput) {
  c::SelfContactActiveUseCounts output;
  output.parents = 12345;
  EXPECT_FALSE(c::CountSelfContactActiveUses(
      {std::numeric_limits<std::size_t>::max(),1,1,1,2},&output));
  EXPECT_EQ(output.parents,12345u);
  EXPECT_FALSE(c::CountSelfContactActiveUses({1,1,1,1,3},&output));
  EXPECT_EQ(output.parents,12345u);
  ASSERT_TRUE(c::CountSelfContactActiveUses({3,3,5,6,2},&output));
  EXPECT_EQ(output.parents,6u);
  EXPECT_EQ(output.facets,9u*16u);
  EXPECT_EQ(output.vertex_uses,3u*25u+3u*15u);
  EXPECT_EQ(output.edge_uses,3u*56u+3u*30u);
}

TEST(SelfContactActiveUses, EveryExactCountCapAndOneByteShortRejectThenRetry) {
  Fixture fixture(2);
  const auto plan = c::SelfContactActiveUseBinding::Preflight(fixture.facets);
  ASSERT_EQ(plan.report.status,Code::Ok);
  auto expect_cap = [&](auto reduce) {
    c::SelfContactActiveUseLimits limits;
    reduce(limits);
    c::SelfContactActiveUseBinding rejected;
    EXPECT_EQ(rejected.Initialize(fixture.facets,{},limits).status,Code::ResourceLimit);
    EXPECT_FALSE(rejected.prepared());
  };
  expect_cap([&](auto& x){x.max_parents=plan.forecast.parents-1;});
  expect_cap([&](auto& x){x.max_facets=plan.forecast.facets-1;});
  expect_cap([&](auto& x){x.max_vertices=plan.forecast.vertices-1;});
  expect_cap([&](auto& x){x.max_edges=plan.forecast.edges-1;});
  expect_cap([&](auto& x){x.max_vertex_uses=plan.forecast.vertex_uses-1;});
  expect_cap([&](auto& x){x.max_edge_uses=plan.forecast.edge_uses-1;});
  expect_cap([&](auto& x){x.max_nodes=plan.forecast.node_roles-1;});
  c::SelfContactActiveUseLimits bytes;
  bytes.max_host_bytes=plan.forecast.startup_payload_bytes-1;
  c::SelfContactActiveUseBinding binding;
  EXPECT_EQ(binding.Initialize(fixture.facets,{},bytes).status,Code::ResourceLimit);
  ++bytes.max_host_bytes;
  ASSERT_EQ(binding.Initialize(fixture.facets,{},bytes).status,Code::Ok);
  EXPECT_EQ(binding.forecast().arena_bytes,plan.forecast.arena_bytes);
  EXPECT_EQ(binding.forecast().startup_payload_bytes,plan.forecast.startup_payload_bytes);
  EXPECT_EQ(binding.Initialize(fixture.facets).status,Code::AlreadyInitialized);

  Fixture distinct(2,false,true);
  Tie tie(distinct);
  const c::SelfContactActiveUseSource tied_source{nullptr,tie.Source()};
  c::SelfContactActiveUseLimits cin_rows;
  cin_rows.max_cin_rows=0;
  c::SelfContactActiveUseBinding rejected_rows;
  EXPECT_EQ(rejected_rows.Initialize(distinct.facets,tied_source,cin_rows).status,
      Code::ResourceLimit);
  c::SelfContactActiveUseLimits cin_witnesses;
  cin_witnesses.max_cin_witnesses=0;
  c::SelfContactActiveUseBinding rejected_witnesses;
  EXPECT_EQ(rejected_witnesses.Initialize(distinct.facets,tied_source,
      cin_witnesses).status,Code::ResourceLimit);
  c::SelfContactActiveUseBinding exact_cin;
  c::SelfContactActiveUseLimits exact_limits;
  exact_limits.max_cin_rows=1;
  exact_limits.max_cin_witnesses=1;
  auto late_range=tie.ranges;
  late_range[0].count=2;
  auto late=tie.Source();
  late.ranges=late_range.data();
  EXPECT_EQ(exact_cin.Initialize(distinct.facets,{nullptr,late},exact_limits).status,
      Code::IdentityMismatch);
  EXPECT_FALSE(exact_cin.prepared());
  EXPECT_EQ(exact_cin.Initialize(distinct.facets,tied_source,exact_limits).status,
      Code::Ok);
}

TEST(SelfContactActiveUses, PermutedSourceSelectionHasIdenticalCanonicalInventory) {
  Fixture first(2,false,false,false), second(2,false,false,true);
  c::SelfContactActiveUseBinding a,b;
  ASSERT_EQ(a.Initialize(first.facets).status,Code::Ok);
  ASSERT_EQ(b.Initialize(second.facets).status,Code::Ok);
  ASSERT_EQ(a.forecast().parents,b.forecast().parents);
  ASSERT_EQ(a.forecast().vertices,b.forecast().vertices);
  ASSERT_EQ(a.forecast().edges,b.forecast().edges);
  ASSERT_EQ(a.forecast().vertex_uses,b.forecast().vertex_uses);
  for (std::size_t i=0;i<a.parents().size();++i) {
    EXPECT_EQ(a.parents()[i].source.source_parent_id,
        b.parents()[i].source.source_parent_id);
    EXPECT_EQ(a.parents()[i].reference_area_m2.value,
        b.parents()[i].reference_area_m2.value);
  }
  for (std::size_t i=0;i<a.vertices().size();++i) {
    EXPECT_TRUE(c::SameFacetVertexKey(a.vertices()[i].key,b.vertices()[i].key));
    EXPECT_EQ(a.vertices()[i].use_count,b.vertices()[i].use_count);
  }
  for (std::size_t i=0;i<a.edges().size();++i) {
    EXPECT_TRUE(c::SameFacetEdgeKey(a.edges()[i].key,b.edges()[i].key));
    EXPECT_EQ(a.edges()[i].use_count,b.edges()[i].use_count);
  }
  for (std::size_t i=0;i<a.vertex_uses().size();++i) {
    EXPECT_TRUE(c::SameFacetVertexKey(a.vertex_uses()[i].key,
        b.vertex_uses()[i].key));
    EXPECT_EQ(a.parents()[a.vertex_uses()[i].parent].source.source_parent_id,
        b.parents()[b.vertex_uses()[i].parent].source.source_parent_id);
    EXPECT_EQ(a.vertex_uses()[i].directed_vf_area_m2.value,
        b.vertex_uses()[i].directed_vf_area_m2.value);
  }
  for (std::size_t i=0;i<a.facet_uses().size();++i) {
    const auto& x=a.facet_uses()[i];
    const auto& y=b.facet_uses()[i];
    EXPECT_EQ(a.parents()[x.parent].source.source_parent_id,
        b.parents()[y.parent].source.source_parent_id);
    EXPECT_EQ(x.local_facet,y.local_facet);
    for (unsigned slot=0;slot<3;++slot) {
      EXPECT_EQ(x.vertex_features[slot],y.vertex_features[slot]);
      EXPECT_EQ(x.edge_features[slot],y.edge_features[slot]);
      EXPECT_EQ(x.vertex_uses[slot],y.vertex_uses[slot]);
      EXPECT_EQ(x.edge_uses[slot],y.edge_uses[slot]);
    }
  }
  for (std::size_t i=0;i<a.edge_uses().size();++i) {
    const auto& x=a.edge_uses()[i];
    const auto& y=b.edge_uses()[i];
    EXPECT_TRUE(c::SameFacetEdgeKey(x.key,y.key));
    EXPECT_EQ(a.parents()[x.parent].source.source_parent_id,
        b.parents()[y.parent].source.source_parent_id);
    EXPECT_EQ(x.feature,y.feature);
    EXPECT_EQ(x.facet_valence,y.facet_valence);
    for (unsigned endpoint=0;endpoint<2;++endpoint) {
      EXPECT_EQ(x.endpoints[endpoint].count,y.endpoints[endpoint].count);
      EXPECT_EQ(x.endpoint_support[endpoint].status,
          y.endpoint_support[endpoint].status);
      for (unsigned slot=0;slot<x.endpoints[endpoint].count;++slot) {
        EXPECT_EQ(x.endpoints[endpoint].nodes[slot],
            y.endpoints[endpoint].nodes[slot]);
        EXPECT_EQ(x.endpoints[endpoint].weights[slot],
            y.endpoints[endpoint].weights[slot]);
      }
    }
  }
}

TEST(SelfContactActiveUses, QueryAliasAddressAndLateIndexFailuresPreserveOutput) {
  Fixture fixture(2);
  c::SelfContactActiveUseBinding uses;
  ASSERT_EQ(uses.Initialize(fixture.facets).status,Code::Ok);
  auto active=fixture.Active(uses);
  c::SelfContactResolvedVertexUse output;
  output.reference_half_thickness_m=321;
  const auto before=output;
  EXPECT_EQ(uses.ResolveVertexUse(SIZE_MAX,
      {active.data(),active.data(),active.size()},&output).status,Code::InvalidInput);
  EXPECT_EQ(std::memcmp(&output,&before,sizeof(output)),0);
  auto* overflow=reinterpret_cast<const std::uint8_t*>(
      UINTPTR_MAX-active.size()+1);
  EXPECT_EQ(uses.ResolveVertexUse(0,{overflow,overflow,active.size()},&output).status,
      Code::InvalidInput);
  EXPECT_EQ(std::memcmp(&output,&before,sizeof(output)),0);
  auto* alias=reinterpret_cast<c::SelfContactResolvedVertexUse*>(
      const_cast<c::SelfContactFacetVertexUse*>(uses.vertex_uses().data()));
  EXPECT_EQ(uses.ResolveVertexUse(0,{active.data(),active.data(),active.size()},alias).status,
      Code::InvalidInput);
  auto* activity_alias=reinterpret_cast<c::SelfContactResolvedVertexUse*>(
      active.data());
  EXPECT_EQ(uses.ResolveVertexUse(0,{active.data(),active.data(),active.size()},
      activity_alias).status,Code::InvalidInput);
  ASSERT_EQ(uses.ResolveVertexUse(0,
      {active.data(),active.data(),active.size()},&output).status,Code::Ok);
  EXPECT_TRUE(output.active);
}
} // namespace active_use_test
