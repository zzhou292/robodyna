// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <climits>
#include <memory>

namespace facet_test {
TEST(FixedContactFacets, CompleteMixedParentsRetainNativeSlotSourceRoleAndThickness) {
  Fixture fixture;
  for (unsigned level = 0; level <= 2; ++level) {
    ct::FixedContactFacetBinding binding;
    ASSERT_EQ(binding.Initialize(fixture.surface, {{}, level}).status, S::Ok);
    const unsigned n = 1u << level;
    EXPECT_EQ(binding.forecast().facets, 9u * n * n);
    EXPECT_LT(binding.forecast().template_arena_bytes, 4096u);
    for (std::size_t parent = 0; parent < fixture.surface.parents().size(); ++parent) {
      const auto& source = fixture.surface.parents()[parent];
      for (unsigned local = 0; local < binding.facet_count(parent); ++local) {
        ct::FixedContactFacet facet;
        ASSERT_EQ(binding.Describe(parent, local, &facet).status, S::Ok);
        EXPECT_EQ(facet.source.source_parent_id, source.source.source_parent_id);
        EXPECT_EQ(facet.source.source_part_id, source.source.source_part_id);
        EXPECT_EQ(facet.source.family, source.source.family);
        EXPECT_EQ(facet.source.family_index, source.source.family_index);
        EXPECT_EQ(facet.source_instance_id, 77u);
        EXPECT_EQ(facet.law, source.law);
        EXPECT_EQ(facet.material_points, source.material_points);
        EXPECT_EQ(facet.reference_half_thickness_m, source.reference_half_thickness_m);
        for (const auto& point : facet.vertices) {
          EXPECT_EQ(ct::ValidateWeightedSurfacePoint(point, fixture.source.domain.node_count()), ct::Status::kOk);
          for (unsigned i = 0; i < point.count; ++i)
            EXPECT_EQ(point.nodes[i], source.arity == 4 ? source.q4.nodes[i] : source.t3.nodes[i]);
        }
      }
    }
  }
}

TEST(FixedContactFacets, SharedQ4T3BoundaryReversesWindingWithoutWeldingDistinctSourceLayers) {
  Fixture fixture(true);
  ct::FixedContactFacetBinding binding;
  ASSERT_EQ(binding.Initialize(fixture.surface, {{}, 2}).status, S::Ok);
  auto edges = [&](std::uint64_t eid) {
    std::vector<ct::FacetEdgeKey> result;
    const auto parent = fixture.Parent(eid);
    for (unsigned local = 0; local < binding.facet_count(parent); ++local) {
      ct::FixedContactFacet facet;
      EXPECT_EQ(binding.Describe(parent, local, &facet).status, S::Ok);
      for (const auto& edge : facet.edge_keys) {
        auto on_shared = [](const ct::FacetVertexKey& vertex) {
          return vertex.kind == ct::FacetVertexKind::SourceVertex ? vertex.first == 11 || vertex.first == 12 :
              vertex.kind == ct::FacetVertexKind::SourceEdge && vertex.first == 11 && vertex.second == 12;
        };
        if (edge.parent_boundary && on_shared(edge.endpoints[0]) && on_shared(edge.endpoints[1])) result.push_back(edge);
      }
    }
    return result;
  };
  const auto quad = edges(100), triangle = edges(200);
  ASSERT_EQ(quad.size(), 4u);
  ASSERT_EQ(triangle.size(), 4u);
  for (const auto& edge : quad)
    EXPECT_EQ(std::count_if(triangle.begin(), triangle.end(), [&](const auto& other) {
      return ct::SameFacetEdgeKey(edge, other);
    }), 1);
  EXPECT_TRUE(edges(101).empty()); // Same coordinates, different original NIDs20..23.
  ct::FixedContactFacet first, second;
  ASSERT_EQ(binding.Describe(fixture.Parent(100), 0, &first).status, S::Ok);
  ASSERT_EQ(binding.Describe(fixture.Parent(100), 1, &second).status, S::Ok);
  unsigned internal_matches = 0;
  for (const auto& a : first.edge_keys) for (const auto& b : second.edge_keys)
    if (ct::SameFacetEdgeKey(a, b)) {
      ++internal_matches;
      EXPECT_FALSE(a.parent_boundary);
      EXPECT_EQ(a.parent_eid, 100u);
    }
  EXPECT_EQ(internal_matches, 1u);
}

TEST(FixedContactFacets, CompleteCapsRetentionAndRejectedOutputsPreserveThenRetry) {
  auto fixture = std::make_unique<Fixture>();
  ct::FixedContactFacetBinding binding;
  const auto plan = ct::FixedContactFacetBinding::Preflight(fixture->surface, {{}, 2});
  ASSERT_EQ(plan.report.status, S::Ok);
  ct::FixedContactFacetLimits cap;
  cap.max_host_bytes = plan.forecast.startup_payload_bytes - 1;
  EXPECT_EQ(binding.Initialize(fixture->surface, {{}, 2}, cap).status, S::ResourceLimit);
  EXPECT_FALSE(binding.prepared());
  cap.max_host_bytes += 1;
  cap.max_facets = plan.forecast.facets - 1;
  EXPECT_EQ(binding.Initialize(fixture->surface, {{}, 2}, cap).status, S::ResourceLimit);
  cap.max_facets += 1;
  EXPECT_EQ(binding.Initialize(fixture->surface, {{}, 3}, cap).status, S::InvalidInput);
  ASSERT_EQ(binding.Initialize(fixture->surface, {{}, 2}, cap).status, S::Ok);
  EXPECT_TRUE(binding.surface()->SharesStorage(fixture->surface));
  EXPECT_FALSE(binding.OutputDisjoint(fixture->surface.parents().data(), sizeof(ct::SelfContactSurfaceParent)));
  EXPECT_EQ(binding.Describe(0, 0, reinterpret_cast<ct::FixedContactFacet*>(
      const_cast<ct::SelfContactSurfaceParent*>(fixture->surface.parents().data()))).status, S::InvalidInput);
  ct::FixedContactFacet result;
  result.source.source_parent_id = 12345;
  EXPECT_EQ(binding.Describe(SIZE_MAX, 0, &result).status, S::OutOfRange);
  EXPECT_EQ(result.source.source_parent_id, 12345u);
  EXPECT_EQ(binding.Describe(0, UINT_MAX, &result).status, S::OutOfRange);
  EXPECT_EQ(binding.Describe(0, 0, nullptr).status, S::InvalidInput);
  const auto copy = binding;
  fixture.reset();
  EXPECT_TRUE(copy.SharesStorage(binding));
  EXPECT_EQ(copy.Describe(0, 0, &result).status, S::Ok);
  EXPECT_EQ(result.source.source_parent_id, 103u);
  EXPECT_EQ(binding.Initialize(*copy.surface()).status, S::AlreadyInitialized);
}
} // namespace facet_test
