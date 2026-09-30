// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <chrono>
#include <iostream>
#include <type_traits>

namespace active_use_test {
namespace {
static_assert(!std::is_copy_constructible_v<c::FixedContactFacetReadCursor>);
static_assert(!std::is_copy_assignable_v<c::FixedContactFacetReadCursor>);
static_assert(!std::is_move_constructible_v<c::FixedContactFacetReadCursor>);
static_assert(!std::is_move_assignable_v<c::FixedContactFacetReadCursor>);

struct CoincidentLayerScale {
  qbat_catalog_test::Fixture seed;
  std::vector<fe::ShellT3BindingInput> triangles;
  fe::ShellQbatBindingInput qbat;
  fe::ShellPlasticityMaterialInput material;
  fe::ShellPlasticitySectionInput section;
  std::vector<fe::ShellPlasticityParentInput> parents;
  std::vector<fe::ShellFailureParentInput> failures;
  fe::ShellBatchBinding shells;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  fe::NodalNodeDomain domain;
  fe::ShellNodeMap map;
  fe::NodalCoefficientLedger ledger;
  fe::ShellPhysicalBinding physical;
  c::SelfContactSurfaceBinding surface;
  c::FixedContactFacetBinding facets;
  std::vector<c::SelfContactParentSelection> selection;

  explicit CoincidentLayerScale(std::size_t layer_count)
      : triangles(layer_count), parents(layer_count+1),
        failures(layer_count+1) {
    qbat = seed.geometry.b;
    qbat.reference.quadrilateral.placement =
        fe::ShellReferencePlacement::Centered;
    for (std::size_t i = 0; i < layer_count; ++i) {
      auto& triangle = triangles[i];
      triangle = seed.triangles[2];
      triangle.source_parent_id = 1000000+layer_count-i;
      for (unsigned node = 0; node < 3; ++node) {
        triangle.nodes[node] = node;
        triangle.reference.node_ids[node] =
            qbat.reference.quadrilateral.node_ids[node];
        triangle.reference.position[node] =
            qbat.reference.quadrilateral.position[node];
      }
    }
    material = qbat_catalog_test::Midlayer();
    section = qbat_catalog_test::MidlayerSection();
    parents[0] = {fe::ShellBindingFamily::Qbat, 0,
        qbat.source_parent_id, material.material_id,
        material.material_id, section.section_id};
    for (std::size_t i = 0; i < layer_count; ++i)
      parents[i+1] = {fe::ShellBindingFamily::T3, i,
          triangles[i].source_parent_id, material.material_id,
          material.material_id, section.section_id};
    for (std::size_t i = 0; i < parents.size(); ++i)
      failures[i] = qbat_catalog_test::Failure(parents[i]);

    const fe::ShellFormulationCollectionInput geometry{
        {nullptr, triangles.data(), 0, triangles.size(), 4}, &qbat, 1};
    EXPECT_EQ(shells.InitializeFormulations(geometry,
        fe::ShellHostBindingLimits::Vehicle()).status,
        fe::ShellBindingStatus::Success);
    const fe::ShellBatchPlasticityBindingInput plasticity{
        nullptr, &material, &section, parents.data(),
        0, 1, 1, parents.size()};
    EXPECT_EQ(catalog.InitializeFormulationCatalog(shells, plasticity,
        fe::ShellPlasticityCatalogLimits::Vehicle()).status,
        fe::ShellPlasticityBindingStatus::Success);
    EXPECT_EQ(failure.Initialize(catalog, failures.data(), failures.size(),
        fe::ShellBatchFailureLimits::Vehicle()).status,
        fe::ShellPlasticityBindingStatus::Success);
    std::vector<fe::NodalDomainNode> nodes;
    for (const auto& node : shells.active_nodes())
      nodes.push_back({node.source_id, node.position});
    EXPECT_TRUE(domain.Initialize({77, nodes.data(), nodes.size()},
        fe::NodalDomainLimits::Vehicle()));
    EXPECT_TRUE(map.Initialize(shells, domain,
        fe::ShellNodeMapLimits::Vehicle()));
    EXPECT_TRUE(ledger.Initialize({&map, nullptr, nullptr},
        fe::CoefficientLimits::Vehicle()));
    EXPECT_TRUE(physical.Initialize({&shells, &catalog, &failure, nullptr},
        ledger, fe::ShellPhysicalBindingLimits::Vehicle()));
    for (std::size_t row = parents.size(); row-- > 0;) {
      const auto& parent = *catalog.parent(row);
      selection.push_back({row, parent.family, parent.family_index,
          parent.source_parent_id, parent.source_part_id});
    }
    EXPECT_EQ(surface.Initialize(physical,
        {selection.data(), selection.size()},
        c::SelfContactSurfaceLimits::Vehicle()).status,
        c::SelfContactSurfaceStatus::Ok);
    EXPECT_EQ(facets.Initialize(surface, {{}, 0},
        c::FixedContactFacetLimits::Vehicle()).status,
        c::FixedContactFacetStatus::Ok);
  }
};

struct ScaleSample {
  std::size_t layers = 0;
  std::int64_t microseconds = 0;
};

void ExpectSameFacet(const c::FixedContactFacet& a,
    const c::FixedContactFacet& b) {
  EXPECT_EQ(a.source.family, b.source.family);
  EXPECT_EQ(a.source.family_index, b.source.family_index);
  EXPECT_EQ(a.source.source_parent_id, b.source.source_parent_id);
  EXPECT_EQ(a.source.source_part_id, b.source.source_part_id);
  EXPECT_EQ(a.source.material_id, b.source.material_id);
  EXPECT_EQ(a.source.section_id, b.source.section_id);
  EXPECT_EQ(a.law, b.law);
  EXPECT_EQ(a.source_instance_id, b.source_instance_id);
  EXPECT_EQ(a.parent_index, b.parent_index);
  EXPECT_EQ(a.level, b.level);
  EXPECT_EQ(a.local_facet, b.local_facet);
  EXPECT_EQ(a.material_points, b.material_points);
  EXPECT_EQ(a.reference_half_thickness_m, b.reference_half_thickness_m);
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    EXPECT_TRUE(c::SameFacetVertexKey(
        a.vertex_keys[vertex], b.vertex_keys[vertex]));
    EXPECT_TRUE(c::SameFacetEdgeKey(
        a.edge_keys[vertex], b.edge_keys[vertex]));
    EXPECT_EQ(a.vertices[vertex].count, b.vertices[vertex].count);
    for (unsigned slot = 0; slot < 4; ++slot) {
      EXPECT_EQ(a.vertices[vertex].nodes[slot],
          b.vertices[vertex].nodes[slot]);
      EXPECT_EQ(a.vertices[vertex].weights[slot],
          b.vertices[vertex].weights[slot]);
    }
  }
}

ScaleSample MeasureCoincidentLayers(std::size_t layers) {
  CoincidentLayerScale fixture(layers);
  const auto plan = c::SelfContactActiveUseBinding::Preflight(
      fixture.facets, {}, c::SelfContactActiveUseLimits::Vehicle());
  EXPECT_EQ(plan.report.status, Code::Ok);
  EXPECT_EQ(plan.forecast.parents, layers+1);
  EXPECT_EQ(plan.forecast.facets, layers+2);
  EXPECT_EQ(plan.forecast.vertices, 4u);
  EXPECT_EQ(plan.forecast.edges, 6u);
  EXPECT_EQ(plan.forecast.vertex_uses, 3*layers+4);
  EXPECT_EQ(plan.forecast.edge_uses, 3*layers+5);
  EXPECT_EQ(plan.forecast.startup_index_bytes,
      sizeof(std::uint32_t)*
          (plan.forecast.vertex_uses+plan.forecast.edge_uses));
  c::SelfContactActiveUseBinding binding;
  const auto begin = std::chrono::steady_clock::now();
  const auto report = binding.Initialize(
      fixture.facets, {}, c::SelfContactActiveUseLimits::Vehicle());
  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now()-begin).count();
  EXPECT_EQ(report.status, Code::Ok);
  EXPECT_EQ(binding.forecast().startup_index_bytes,
      plan.forecast.startup_index_bytes);
  std::cout << "active-use-scale layers=" << layers
            << " vertex_uses=" << plan.forecast.vertex_uses
            << " edge_uses=" << plan.forecast.edge_uses
            << " startup_index_bytes=" << plan.forecast.startup_index_bytes
            << " initialize_us=" << elapsed << '\n';
  return {layers, elapsed};
}
} // namespace

TEST(SelfContactActiveUses, AuthenticatedFacetCursorMatchesCheckedDescriptor) {
  CoincidentLayerScale fixture(8192);
  c::FixedContactFacet checked;
  const c::FixedContactFacet* cursor_output = nullptr;
  std::size_t checked_count = 0, cursor_count = 0;
  const auto checked_begin = std::chrono::steady_clock::now();
  for (std::size_t parent = 0;
       parent < fixture.surface.parents().size(); ++parent)
    for (unsigned local = 0;
         local < fixture.facets.facet_count(parent); ++local) {
      ASSERT_EQ(fixture.facets.Describe(
          parent, local, &checked).status,
          c::FixedContactFacetStatus::Ok);
      ++checked_count;
    }
  const auto checked_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now()-checked_begin).count();
  c::FixedContactFacetReadCursor cursor;
  const auto cursor_begin = std::chrono::steady_clock::now();
  ASSERT_EQ(cursor.Initialize(fixture.facets).status,
      c::FixedContactFacetStatus::Ok);
  for (std::size_t parent = 0;
       parent < fixture.surface.parents().size(); ++parent)
    for (unsigned local = 0;
         local < fixture.facets.facet_count(parent); ++local) {
      const auto descriptor = cursor.Describe(parent, local);
      ASSERT_EQ(descriptor.report.status,
          c::FixedContactFacetStatus::Ok);
      ASSERT_NE(descriptor.facet, nullptr);
      cursor_output = descriptor.facet;
      ++cursor_count;
    }
  const auto cursor_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now()-cursor_begin).count();
  EXPECT_EQ(checked_count, cursor_count);
  ASSERT_NE(cursor_output, nullptr);
  ExpectSameFacet(checked, *cursor_output);
  const auto invalid = cursor.Describe(SIZE_MAX, 0);
  EXPECT_EQ(invalid.report.status,
      c::FixedContactFacetStatus::OutOfRange);
  EXPECT_EQ(invalid.facet, nullptr);
  const auto retried = cursor.Describe(0, 0);
  EXPECT_EQ(retried.report.status, c::FixedContactFacetStatus::Ok);
  EXPECT_NE(retried.facet, nullptr);
  EXPECT_EQ(cursor.Initialize(fixture.facets).status,
      c::FixedContactFacetStatus::AlreadyInitialized);
  c::FixedContactFacetBinding absent;
  c::FixedContactFacetReadCursor retry;
  EXPECT_EQ(retry.Initialize(absent).status,
      c::FixedContactFacetStatus::InvalidInput);
  const auto unavailable = retry.Describe(0, 0);
  EXPECT_EQ(unavailable.report.status,
      c::FixedContactFacetStatus::InvalidInput);
  EXPECT_EQ(unavailable.facet, nullptr);
  EXPECT_EQ(retry.Initialize(fixture.facets).status,
      c::FixedContactFacetStatus::Ok);
  EXPECT_LT(cursor_us, checked_us);
  std::cout << "facet-descriptor-scale parents="
            << fixture.surface.parents().size()
            << " descriptors=" << checked_count
            << " checked_us=" << checked_us
            << " cursor_us=" << cursor_us << '\n';
}

TEST(SelfContactActiveUses, FacetCursorRetainsSourceAndExpiresViews) {
  c::FixedContactFacetReadCursor cursor;
  {
    Fixture fixture(2);
    ASSERT_EQ(cursor.Initialize(fixture.facets).status,
        c::FixedContactFacetStatus::Ok);
    const auto first = cursor.Describe(0, 0);
    ASSERT_EQ(first.report.status, c::FixedContactFacetStatus::Ok);
    ASSERT_NE(first.facet, nullptr);
    const auto* borrowed = first.facet;
    const auto second = cursor.Describe(1, 0);
    ASSERT_EQ(second.report.status, c::FixedContactFacetStatus::Ok);
    ASSERT_NE(second.facet, nullptr);
    EXPECT_EQ(second.facet, borrowed);
  }
  const auto retained = cursor.Describe(0, 0);
  EXPECT_EQ(retained.report.status, c::FixedContactFacetStatus::Ok);
  EXPECT_NE(retained.facet, nullptr);
}

TEST(SelfContactActiveUses, CoincidentLayerStartupScalesWithoutFeatureUseScans) {
  const auto small = MeasureCoincidentLayers(16384);
  const auto medium = MeasureCoincidentLayers(32768);
  const auto large = MeasureCoincidentLayers(65536);
  EXPECT_LT(small.microseconds, medium.microseconds*2+50000);
  EXPECT_LT(medium.microseconds, large.microseconds*2+50000);
  // A 4x roster admits N log N index sorting, but rejects a return to the
  // eliminated quadratic parent lookup through each canonical feature range.
  EXPECT_LT(large.microseconds, small.microseconds*8+50000);
}
} // namespace active_use_test
