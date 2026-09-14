// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include "../qbat_catalog/Fixture.h"
#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"

#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

namespace current_regularity_test {
namespace {

namespace fe = tl::fea;

// The measured vehicle census has 315,963 Q4 and 21,129 T3 parents.  This
// bounded fixture keeps the same 15:1 representative mix and deliberately
// shares four nodes so setup memory does not hide parent/facet scaling.
struct RepresentativeScale {
  qbat_catalog_test::Fixture seed;
  std::vector<fe::ShellQephBindingInput> quads;
  std::vector<fe::ShellT3BindingInput> triangles;
  fe::ShellQbatBindingInput qbat;
  fe::ShellPlasticityCurveInput curve;
  std::array<fe::ShellPlasticityMaterialInput, 2> materials;
  std::array<fe::ShellPlasticitySectionInput, 2> sections;
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
  c::SelfContactActiveUseBinding uses;
  std::vector<c::SelfContactParentSelection> selection;
  std::vector<double> positions;
  std::vector<std::uint8_t> base;
  std::vector<std::uint8_t> current;

  explicit RepresentativeScale(std::size_t parent_count)
      : quads(parent_count-parent_count/16-1),
        triangles(parent_count/16), parents(parent_count),
        failures(parent_count) {
    EXPECT_EQ(parent_count % 16, 0u);
    for (auto& q : seed.geometry.q)
      q.reference.placement = fe::ShellReferencePlacement::Centered;
    for (auto& t : seed.triangles)
      t.reference.placement = fe::ShellReferencePlacement::Centered;
    qbat = seed.geometry.b;
    qbat.reference.quadrilateral.placement =
        fe::ShellReferencePlacement::Centered;
    curve = seed.curve;
    materials = {{seed.materials[0], qbat_catalog_test::Midlayer()}};
    sections = {{seed.sections[0], qbat_catalog_test::MidlayerSection()}};
    parents[0] = {fe::ShellBindingFamily::Qbat, 0,
        qbat.source_parent_id, materials[1].material_id,
        materials[1].material_id, sections[1].section_id};
    for (std::size_t i = 0; i < quads.size(); ++i) {
      quads[i] = seed.geometry.q[0];
      quads[i].source_parent_id = 1000000+i;
      parents[i+1] = {fe::ShellBindingFamily::Qeph, i,
          quads[i].source_parent_id, materials[0].material_id,
          materials[0].material_id, sections[0].section_id};
    }
    for (std::size_t i = 0; i < triangles.size(); ++i) {
      triangles[i] = seed.triangles[2];
      triangles[i].source_parent_id = 2000000+i;
      for (unsigned node = 0; node < 3; ++node) {
        triangles[i].nodes[node] = node;
        triangles[i].reference.node_ids[node] =
            seed.geometry.q[0].reference.node_ids[node];
        triangles[i].reference.position[node] =
            seed.geometry.q[0].reference.position[node];
      }
      const auto p = quads.size()+i+1;
      parents[p] = {fe::ShellBindingFamily::T3, i,
          triangles[i].source_parent_id, materials[1].material_id,
          materials[1].material_id, sections[1].section_id};
    }
    for (std::size_t i = 0; i < parents.size(); ++i)
      failures[i] = qbat_catalog_test::Failure(parents[i]);

    const fe::ShellFormulationCollectionInput geometry{{
        quads.data(), triangles.data(), quads.size(), triangles.size(), 4},
        &qbat, 1};
    EXPECT_EQ(shells.InitializeFormulations(
        geometry, fe::ShellHostBindingLimits::Vehicle()).status,
        fe::ShellBindingStatus::Success);
    const fe::ShellBatchPlasticityBindingInput plasticity{
        &curve, materials.data(), sections.data(), parents.data(),
        1, materials.size(), sections.size(), parents.size()};
    const auto catalog_report = catalog.InitializeFormulationCatalog(
        shells, plasticity, fe::ShellPlasticityCatalogLimits::Vehicle());
    EXPECT_EQ(catalog_report.status,
        fe::ShellPlasticityBindingStatus::Success) << catalog_report.message;
    if (catalog_report.status !=
        fe::ShellPlasticityBindingStatus::Success) return;
    const auto failure_report = failure.Initialize(
        catalog, failures.data(), failures.size(),
        fe::ShellBatchFailureLimits::Vehicle());
    EXPECT_EQ(failure_report.status,
        fe::ShellPlasticityBindingStatus::Success) << failure_report.message;
    if (failure_report.status !=
        fe::ShellPlasticityBindingStatus::Success) return;
    std::vector<fe::NodalDomainNode> nodes;
    for (const auto& node : shells.active_nodes())
      nodes.push_back({node.source_id, node.position});
    EXPECT_TRUE(domain.Initialize(
        {77, nodes.data(), nodes.size()},
        fe::NodalDomainLimits::Vehicle()));
    EXPECT_TRUE(map.Initialize(
        shells, domain, fe::ShellNodeMapLimits::Vehicle()));
    EXPECT_TRUE(ledger.Initialize(
        {&map, nullptr, nullptr}, fe::CoefficientLimits::Vehicle()));
    EXPECT_TRUE(physical.Initialize(
        {&shells, &catalog, &failure, nullptr}, ledger,
        fe::ShellPhysicalBindingLimits::Vehicle()));
    for (std::size_t row = parents.size(); row-- > 0;) {
      const auto& parent = *catalog.parent(row);
      selection.push_back({row, parent.family, parent.family_index,
          parent.source_parent_id, parent.source_part_id});
    }
    EXPECT_EQ(surface.Initialize(
        physical, {selection.data(), selection.size()},
        c::SelfContactSurfaceLimits::Vehicle()).status,
        c::SelfContactSurfaceStatus::Ok);
    EXPECT_EQ(facets.Initialize(
        surface, {{}, 0}, c::FixedContactFacetLimits::Vehicle()).status,
        c::FixedContactFacetStatus::Ok);
    EXPECT_EQ(uses.Initialize(
        facets, {}, c::SelfContactActiveUseLimits::Vehicle()).status,
        c::SelfContactActiveUseStatus::Ok);
    positions.resize(3*domain.node_count());
    for (std::size_t n = 0; n < domain.node_count(); ++n) {
      positions[3*n] = domain.nodes()[n].position.x;
      positions[3*n+1] = domain.nodes()[n].position.y;
      positions[3*n+2] = domain.nodes()[n].position.z;
    }
    base.assign(parents.size(), 1);
    current = base;
  }

  c::VectorView Positions() const {
    return {positions.data(), static_cast<std::uint32_t>(domain.node_count()),
            3, 1};
  }
  c::SelfContactActivityView Activity() const {
    return {base.data(), current.data(), base.size()};
  }
};

struct ScaleSample {
  std::size_t parents = 0;
  std::size_t facets = 0;
  std::size_t arena_bytes = 0;
  std::int64_t initialize_us = 0;
  std::int64_t certify_us = 0;
};

ScaleSample Measure(std::size_t parents) {
  RepresentativeScale fixture(parents);
  const auto plan = c::SelfContactCurrentRegularity::Preflight(
      fixture.uses, c::SelfContactCurrentRegularityLimits::Vehicle());
  EXPECT_EQ(plan.report.status, Status::Ok);
  EXPECT_EQ(plan.forecast.parents, parents);
  EXPECT_EQ(plan.forecast.facets, parents*31/16);

  c::SelfContactCurrentRegularity regularity;
  const auto initialize_begin = std::chrono::steady_clock::now();
  const auto initialized = regularity.Initialize(
      fixture.uses, c::SelfContactCurrentRegularityLimits::Vehicle());
  const auto initialize_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now()-initialize_begin).count();
  EXPECT_EQ(initialized.status, Status::Ok);

  c::SelfContactCurrentRegularityReceipt receipt;
  const auto certify_begin = std::chrono::steady_clock::now();
  const auto certified = regularity.Certify(
      fixture.Positions(), fixture.Activity(), &receipt);
  const auto certify_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now()-certify_begin).count();
  EXPECT_EQ(certified.status, Status::Ok);
  const auto view = regularity.results();
  EXPECT_TRUE(view.complete);
  EXPECT_EQ(view.summary.certified_parents, parents);
  EXPECT_EQ(view.summary.facets_evaluated, plan.forecast.facets);
  std::cout << "current-regularity-scale parents=" << parents
            << " q4=" << parents*15/16
            << " t3=" << parents/16
            << " facets=" << plan.forecast.facets
            << " arena_bytes=" << plan.forecast.arena_bytes
            << " initialize_us=" << initialize_us
            << " certify_us=" << certify_us << '\n';
  return {parents, plan.forecast.facets, plan.forecast.arena_bytes,
          initialize_us, certify_us};
}

}  // namespace

TEST(SelfContactCurrentRegularityScaling,
    RepresentativeQ4T3CensusHasBoundedLinearStartupAndQuery) {
  constexpr std::size_t vehicle_parents = 337092;
  constexpr std::size_t vehicle_facets = 653055;
  constexpr std::size_t vehicle_q4 = vehicle_facets-vehicle_parents;
  constexpr std::size_t vehicle_t3 = 2*vehicle_parents-vehicle_facets;
  static_assert(vehicle_q4+vehicle_t3 == vehicle_parents);
  static_assert(2*vehicle_q4+vehicle_t3 == vehicle_facets);

  const auto small = Measure(512);
  const auto medium = Measure(1024);
  const auto large = Measure(2048);
  ASSERT_GE(small.arena_bytes,
      2*small.parents*sizeof(c::SelfContactCurrentParentResult));
  const auto template_bytes =
      small.arena_bytes -
      2*small.parents*sizeof(c::SelfContactCurrentParentResult);
  const auto vehicle_arena_bytes =
      2*vehicle_parents*sizeof(c::SelfContactCurrentParentResult) +
      template_bytes;
  std::cout << "current-regularity-v5-forecast parents="
            << vehicle_parents << " q4=" << vehicle_q4
            << " t3=" << vehicle_t3 << " facets=" << vehicle_facets
            << " arena_bytes=" << vehicle_arena_bytes
            << " template_bytes=" << template_bytes << '\n';
  const auto total = [](ScaleSample sample) {
    return sample.initialize_us+sample.certify_us;
  };
  EXPECT_LT(total(medium), total(small)*3+5000);
  EXPECT_LT(total(large), total(medium)*3+5000);
  // A 4x census must remain below an 8x envelope.  The former checked
  // Describe path traversed the complete retained source for every facet and
  // grew quadratically.
  EXPECT_LT(total(large), total(small)*8+5000);
}

TEST(SelfContactCurrentRegularityScaling,
    CompactFacetPathMatchesIndependentDescriptorBaselineExactly) {
  Fixture fixture(2, true);
  fixture.InitializeRegularity();
  c::SelfContactCurrentRegularityReceipt receipt;
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(), fixture.Activity(), &receipt).status, Status::Ok);
  const auto view = fixture.regularity.results();
  ASSERT_TRUE(view.complete);
  for (std::size_t p = 0; p < view.count; ++p) {
    const auto& result = view.data[p];
    c::FacetApproximationBound approximation;
    ASSERT_EQ(fixture.source.facets.Approximation(
        result.surface_parent, fixture.Positions(), &approximation),
        c::Status::kOk);
    EXPECT_EQ(std::memcmp(
        &result.approximation, &approximation,
        sizeof(approximation)), 0);
    bool have = false;
    c::SelfContactCurrentFacetWitness minimum_area;
    c::SelfContactCurrentFacetWitness minimum_quality;
    c::SelfContactCurrentFacetWitness minimum_directed;
    std::uint32_t area_local = UINT32_MAX;
    std::uint32_t quality_local = UINT32_MAX;
    std::uint32_t directed_local = UINT32_MAX;
    for (std::uint32_t local = 0; local < result.facet_count; ++local) {
      c::FixedContactFacet descriptor;
      ASSERT_EQ(fixture.source.facets.Describe(
          result.surface_parent, local, &descriptor).status,
          c::FixedContactFacetStatus::Ok);
      c::CurrentFixedTriangle triangle;
      ASSERT_EQ(c::EvaluateCurrentFixedTriangle(
          descriptor, fixture.Positions(), &triangle), c::Status::kOk);
      c::SelfContactCurrentFacetWitness witness;
      ASSERT_EQ(c::EvaluateCurrentFacetRegularity(
          triangle.vertices, result.chart_direction, &witness),
          c::SelfContactCurrentFacetStatus::Ok);
      if (!have || witness.double_area_m2.lower <
                       minimum_area.double_area_m2.lower) {
        minimum_area = witness;
        area_local = local;
      }
      if (!have || witness.scaled_jacobian_quality <
                       minimum_quality.scaled_jacobian_quality) {
        minimum_quality = witness;
        quality_local = local;
      }
      if (!have || witness.directed_chart_measure_m2.lower <
                       minimum_directed.directed_chart_measure_m2.lower) {
        minimum_directed = witness;
        directed_local = local;
      }
      have = true;
    }
    ASSERT_TRUE(have);
    EXPECT_EQ(std::memcmp(
        &result.minimum_area_witness, &minimum_area,
        sizeof(minimum_area)), 0);
    EXPECT_EQ(result.minimum_area_local_facet, area_local);
    EXPECT_EQ(result.minimum_scaled_jacobian_quality,
              minimum_quality.scaled_jacobian_quality);
    EXPECT_EQ(result.minimum_quality_local_facet, quality_local);
    EXPECT_EQ(std::memcmp(
        &result.minimum_directed_chart_measure_m2,
        &minimum_directed.directed_chart_measure_m2,
        sizeof(result.minimum_directed_chart_measure_m2)), 0);
    EXPECT_EQ(result.minimum_directed_local_facet, directed_local);
  }
}

}  // namespace current_regularity_test
