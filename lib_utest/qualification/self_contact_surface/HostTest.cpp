// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <algorithm>
#include <memory>

namespace self_contact_test {
TEST(SelfContactSurface, ExactNativeParentsThicknessAndScrambledDomain) {
  Fixture f;
  const auto selection = f.Selection();
  ct::SelfContactSurfaceBinding surface;
  ASSERT_EQ(surface.Initialize(f.physical, Input(selection)).status, S::Ok);
  ASSERT_EQ(surface.parents().size(), 6u);
  EXPECT_EQ(surface.vertices().size(), 5u);
  EXPECT_EQ(surface.edges().size(), 6u);
  EXPECT_EQ(surface.vertex_uses().size(), 21u);
  EXPECT_EQ(surface.edge_uses().size(), 21u);
  for (std::size_t p = 0; p < selection.size(); ++p) {
    const auto& parent = surface.parents()[p];
    EXPECT_EQ(parent.source.source_parent_id, selection[p].source_parent_id);
    EXPECT_EQ(parent.source.source_part_id, selection[p].source_part_id);
    EXPECT_EQ(parent.source.family_index, selection[p].family_index);
    const bool triangle = parent.source.family == fe::ShellBindingFamily::T3;
    EXPECT_EQ(parent.arity, triangle ? 3u : 4u);
    unsigned expected_points = 0;
    ASSERT_TRUE(f.catalog.MaterialPointCount(parent.source.family, parent.source.family_index, &expected_points));
    EXPECT_EQ(parent.material_points, expected_points);
    const double thickness = triangle ? f.shells.t3_reference(parent.source.family_index).input.thickness :
        (parent.source.family == fe::ShellBindingFamily::Qbat ?
         f.shells.qbat_reference(parent.source.family_index).quadrilateral().input.thickness :
         f.shells.qeph_reference(parent.source.family_index).input.thickness);
    EXPECT_EQ(qbat_binding_test::Bits(parent.reference_half_thickness_m), qbat_binding_test::Bits(.5 * thickness));
    EXPECT_EQ(parent.t3.half_thickness, 0);
    EXPECT_EQ(parent.q4.half_thickness, 0);
    for (unsigned local = 0; local < parent.arity; ++local) {
      const auto node = triangle ? parent.t3.nodes[local] : parent.q4.nodes[local];
      const auto& vertex = surface.vertices()[parent.vertices[local]];
      EXPECT_EQ(vertex.domain_node, node);
      EXPECT_EQ(vertex.source_node_id, f.domain.nodes()[node].source_id);
    }
  }
  std::uint64_t last = 0;
  for (auto face : surface.faces()) {
    EXPECT_GT(surface.parents()[face].source.source_parent_id, last);
    last = surface.parents()[face].source.source_parent_id;
  }
}
TEST(SelfContactSurface, CoincidentCoordinatesWithDifferentSourceNodesNeverMerge) {
  Fixture f(true, 77, true);
  const auto rows = f.Selection();
  ct::SelfContactSurfaceBinding surface;
  ASSERT_EQ(surface.Initialize(f.physical, Input(rows)).status, S::Ok);
  EXPECT_EQ(surface.vertices().size(), 9u);
  EXPECT_EQ(surface.edges().size(), 10u);
  const auto& first = surface.parents()[2];
  const auto& second = surface.parents()[4];
  for (unsigned i = 0; i < 4; ++i) {
    EXPECT_NE(first.vertices[i], second.vertices[i]);
    EXPECT_NE(first.edges[i], second.edges[i]);
    EXPECT_EQ(surface.VertexInFace(first.vertices[i], 4), ct::TopologicalIncidence::Disjoint);
  }
}
TEST(SelfContactSurface, CanonicalFeaturesPreserveLayersAndRemoteEdges) {
  Fixture f;
  auto rows = f.Selection();
  ct::SelfContactSurfaceBinding first;
  ASSERT_EQ(first.Initialize(f.physical, Input(rows)).status, S::Ok);
  std::reverse(rows.begin(), rows.end());
  ct::SelfContactSurfaceBinding reversed;
  ASSERT_EQ(reversed.Initialize(f.physical, Input(rows)).status, S::Ok);
  ASSERT_EQ(first.parents().size(), reversed.parents().size());
  ASSERT_EQ(first.edges().size(), reversed.edges().size());
  for (std::size_t i = 0; i < first.edges().size(); ++i) {
    EXPECT_EQ(first.edges()[i].source_node_ids[0], reversed.edges()[i].source_node_ids[0]);
    EXPECT_EQ(first.edges()[i].source_node_ids[1], reversed.edges()[i].source_node_ids[1]);
    EXPECT_EQ(first.edges()[i].use_count, reversed.edges()[i].use_count);
  }
  const auto& quad = first.parents()[0]; // QBAT, same nodes as two QEPH faces.
  EXPECT_NE(quad.source.source_parent_id, first.parents()[2].source.source_parent_id);
  EXPECT_EQ(quad.vertices[0], first.parents()[2].vertices[0]);
  EXPECT_EQ(quad.edges[0], first.parents()[2].edges[0]);
  EXPECT_NE(qbat_binding_test::Bits(quad.reference_half_thickness_m),
      qbat_binding_test::Bits(first.parents()[2].reference_half_thickness_m));
  // Canonical edge identity retains all incident uses. Radius/area/activity is
  // a future per-use policy; canonicalization does not select the first layer.
  const auto& edge = first.edges()[quad.edges[0]];
  unsigned thin = 0, thick = 0;
  for (std::size_t i = edge.use_offset; i < edge.use_offset + edge.use_count; ++i) {
    const auto h = first.parents()[first.edge_uses()[i].parent].reference_half_thickness_m;
    thin += h == .00025;
    thick += h == .001;
  }
  EXPECT_EQ(thin, 1u);
  EXPECT_EQ(thick, 2u);
  EXPECT_EQ(first.VertexInFace(quad.vertices[0], 2), ct::TopologicalIncidence::Incident);
  EXPECT_EQ(first.EdgesShareVertex(quad.edges[0], quad.edges[2]), ct::TopologicalIncidence::Disjoint);
  EXPECT_EQ(first.EdgesShareVertex(quad.edges[0], quad.edges[1]), ct::TopologicalIncidence::Incident);
  EXPECT_EQ(first.VertexInFace(SIZE_MAX, 0), ct::TopologicalIncidence::Invalid);
  EXPECT_EQ(first.EdgesShareVertex(0, SIZE_MAX), ct::TopologicalIncidence::Invalid);
  // The fifth source node is absent from every physical quad, despite connected
  // triangle faces. We do not exclude whole faces because another feature touches.
  const auto remote = first.parents()[1].vertices[1];
  EXPECT_EQ(first.VertexInFace(remote, 0), ct::TopologicalIncidence::Disjoint);
}
TEST(SelfContactSurface, LateIdentityDuplicateAndReferencePlaneFailuresPreserveRetry) {
  Fixture f;
  const auto good = f.Selection();
  for (unsigned field = 0; field < 5; ++field) {
    auto rows = good;
    auto& last = rows.back();
    if (field == 0) ++last.source_parent_id;
    if (field == 1) ++last.source_part_id;
    if (field == 2) last.family = fe::ShellBindingFamily::None;
    if (field == 3) last.family_index = SIZE_MAX;
    if (field == 4) last.catalog_row = SIZE_MAX;
    ct::SelfContactSurfaceBinding value;
    const auto failed = value.Initialize(f.physical, Input(rows));
    EXPECT_EQ(failed.status, S::IdentityMismatch);
    EXPECT_EQ(failed.selection, rows.size() - 1);
    EXPECT_FALSE(value.prepared());
    ASSERT_EQ(value.Initialize(f.physical, Input(good)).status, S::Ok);
  }
  auto duplicate = good;
  duplicate.back() = duplicate.front();
  ct::SelfContactSurfaceBinding value;
  EXPECT_EQ(value.Initialize(f.physical, Input(duplicate)).status, S::DuplicateParent);
  EXPECT_FALSE(value.prepared());
  Fixture offset(false);
  const auto offset_rows = offset.Selection();
  EXPECT_EQ(value.Initialize(offset.physical, Input(offset_rows)).status, S::UnsupportedReferencePlane);
  EXPECT_FALSE(value.prepared());
  ASSERT_EQ(value.Initialize(f.physical, Input(good)).status, S::Ok);
}
TEST(SelfContactSurface, InclusiveBytesFeatureCapsAndOverflowBeforeBorrowedReads) {
  Fixture f;
  const auto rows = f.Selection();
  const auto preflight = ct::SelfContactSurfaceBinding::Preflight(f.physical, Input(rows));
  ASSERT_EQ(preflight.report.status, S::Ok);
  auto limits = ct::SelfContactSurfaceLimits{};
  limits.max_host_bytes = preflight.forecast.startup_payload_bytes - 1;
  ct::SelfContactSurfaceBinding value;
  EXPECT_EQ(value.Initialize(f.physical, Input(rows), limits).status, S::ResourceLimit);
  EXPECT_FALSE(value.prepared());
  ++limits.max_host_bytes;
  ASSERT_EQ(value.Initialize(f.physical, Input(rows), limits).status, S::Ok);
  EXPECT_EQ(value.forecast().arena_bytes, preflight.forecast.arena_bytes);
  EXPECT_EQ(value.forecast().retained_source_bytes, f.physical.owned_payload_bytes() - sizeof(f.physical));
  for (bool vertex : {false, true}) {
    auto small = ct::SelfContactSurfaceLimits{};
    if (vertex) small.max_vertices = 4;
    else small.max_edges = 5;
    ct::SelfContactSurfaceBinding retry;
    EXPECT_EQ(retry.Initialize(f.physical, Input(rows), small).status, S::ResourceLimit);
    EXPECT_FALSE(retry.prepared());
    ASSERT_EQ(retry.Initialize(f.physical, Input(rows)).status, S::Ok);
  }
  ct::SelfContactSurfaceInput enormous{reinterpret_cast<const ct::SelfContactParentSelection*>(1), SIZE_MAX};
  EXPECT_EQ(ct::SelfContactSurfaceBinding::Preflight(f.physical, enormous).report.status, S::ResourceLimit);
  enormous = {reinterpret_cast<const ct::SelfContactParentSelection*>(UINTPTR_MAX - 1), 1};
  EXPECT_EQ(ct::SelfContactSurfaceBinding::Preflight(f.physical, enormous).report.status, S::InvalidInput);
}
TEST(SelfContactSurface, LifetimeExactPhysicalIdentityAndCompleteOutputRanges) {
  auto fixture = std::make_unique<Fixture>();
  auto rows = fixture->Selection();
  ct::SelfContactSurfaceBinding surface;
  ASSERT_EQ(surface.Initialize(fixture->physical, Input(rows)).status, S::Ok);
  EXPECT_TRUE(surface.MatchesPhysical(fixture->physical));
  ct::SelfContactSurfaceBinding copy(surface);
  EXPECT_TRUE(surface.SharesStorage(copy));
  rows.clear();
  fixture.reset();
  ASSERT_EQ(surface.parents().size(), 6u);
  double output = 0;
  EXPECT_TRUE(surface.OutputDisjoint(&output, sizeof(output)));
  EXPECT_FALSE(surface.OutputDisjoint(surface.parents().data(), sizeof(ct::SelfContactSurfaceParent)));
  EXPECT_FALSE(surface.OutputDisjoint(surface.vertex_uses().data(), sizeof(ct::SelfContactVertexUse)));
  EXPECT_FALSE(surface.OutputDisjoint(surface.edge_uses().data(), sizeof(ct::SelfContactEdgeUse)));
  EXPECT_FALSE(surface.OutputDisjoint(surface.faces().data(), sizeof(std::uint32_t)));
  EXPECT_FALSE(surface.OutputDisjoint(surface.physical()->domain()->nodes().data(), sizeof(fe::NodalDomainNode)));
  EXPECT_FALSE(surface.OutputDisjoint(surface.physical()->catalog()->parent(5), sizeof(fe::ShellPlasticityParentInput)));
  EXPECT_FALSE(surface.OutputDisjoint(reinterpret_cast<const void*>(UINTPTR_MAX - 1), 4));
  Fixture foreign(true, 78);
  EXPECT_FALSE(surface.MatchesPhysical(foreign.physical));
  ct::SelfContactSurfaceBinding destination;
  const ct::SelfContactSurfaceInput alias{reinterpret_cast<const ct::SelfContactParentSelection*>(&destination), 1};
  EXPECT_EQ(destination.Initialize(foreign.physical, alias).status, S::InvalidInput);
  EXPECT_FALSE(destination.prepared());
}
} // namespace self_contact_test
