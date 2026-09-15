// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include <chrono>
#include <cstring>
#include <iostream>

namespace active_use_test {
namespace {
constexpr std::size_t UnrelatedCinRows = 10000;
constexpr std::size_t CinRows = UnrelatedCinRows + 1;
constexpr std::size_t MediumQueryCoupon = 100000;
constexpr std::size_t OracleSamples = 128;

struct CinScaleFixture {
  static constexpr std::size_t unrelated_triangles =
      (UnrelatedCinRows+2)/3;
  qbat_catalog_test::Fixture seed;
  fe::ShellQbatBindingInput qbat;
  std::vector<fe::ShellT3BindingInput> triangles;
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
  std::array<c::SelfContactParentSelection, 2> selection;

  explicit CinScaleFixture(unsigned level)
      : triangles(unrelated_triangles+1),
        parents(triangles.size()+1), failures(parents.size()) {
    qbat = seed.geometry.b;
    qbat.source_parent_id = 100;
    qbat.reference.quadrilateral.placement =
        fe::ShellReferencePlacement::Centered;
    const tl::math::Vec3 positions[]{
        {.01, .003, .00025}, {.03, .003, .00025},
        {.02, .006, .00025}};
    for (std::size_t t = 0; t < triangles.size(); ++t) {
      auto& triangle = triangles[t];
      triangle = seed.triangles[2];
      triangle.source_parent_id = t ? 1000+t : 101;
      triangle.reference.placement =
          fe::ShellReferencePlacement::Centered;
      for (unsigned n = 0; n < 3; ++n) {
        triangle.nodes[n] = 4+3*t+n;
        triangle.reference.node_ids[n] = t ?
            10000+3*(t-1)+n : 20+n;
        triangle.reference.position[n] = positions[n];
      }
    }
    material = qbat_catalog_test::Midlayer();
    section = qbat_catalog_test::MidlayerSection();
    parents[0] = {fe::ShellBindingFamily::Qbat, 0, 100, 1000,
        material.material_id, section.section_id};
    for (std::size_t t = 0; t < triangles.size(); ++t)
      parents[t+1] = {fe::ShellBindingFamily::T3, t,
          triangles[t].source_parent_id, 1000,
          material.material_id, section.section_id};
    for (std::size_t p = 0; p < parents.size(); ++p)
      failures[p] = qbat_catalog_test::Failure(parents[p]);

    const fe::ShellFormulationCollectionInput geometry{
        {nullptr, triangles.data(), 0, triangles.size(),
         4+3*triangles.size()}, &qbat, 1};
    EXPECT_EQ(shells.InitializeFormulations(
        geometry, fe::ShellHostBindingLimits::Vehicle()).status,
        fe::ShellBindingStatus::Success);
    const fe::ShellBatchPlasticityBindingInput plasticity{
        nullptr, &material, &section, parents.data(),
        0, 1, 1, parents.size()};
    EXPECT_EQ(catalog.InitializeFormulationCatalog(
        shells, plasticity,
        fe::ShellPlasticityCatalogLimits::Vehicle()).status,
        fe::ShellPlasticityBindingStatus::Success);
    EXPECT_EQ(failure.Initialize(
        catalog, failures.data(), failures.size(),
        fe::ShellBatchFailureLimits::Vehicle()).status,
        fe::ShellPlasticityBindingStatus::Success);
    std::vector<fe::NodalDomainNode> nodes;
    nodes.reserve(shells.node_count());
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
    for (unsigned p = 0; p < selection.size(); ++p) {
      const auto row = p;
      const auto& parent = *catalog.parent(row);
      selection[p] = {row, parent.family, parent.family_index,
          parent.source_parent_id, parent.source_part_id};
    }
    EXPECT_EQ(surface.Initialize(
        physical, {selection.data(), selection.size()},
        c::SelfContactSurfaceLimits::Vehicle()).status,
        c::SelfContactSurfaceStatus::Ok);
    EXPECT_EQ(facets.Initialize(
        surface, {{}, level},
        c::FixedContactFacetLimits::Vehicle()).status,
        c::FixedContactFacetStatus::Ok);
  }

  std::size_t Parent(
      std::uint64_t eid,
      const c::SelfContactActiveUseBinding& uses) const {
    for (std::size_t p = 0; p < uses.parents().size(); ++p)
      if (uses.parents()[p].source.source_parent_id == eid)
        return p;
    return SIZE_MAX;
  }
  std::size_t VertexUse(
      std::uint64_t eid,
      const c::SelfContactActiveUseBinding& uses,
      std::uint64_t source_nid) const {
    const auto parent = Parent(eid, uses);
    for (std::size_t i = 0; i < uses.vertex_uses().size(); ++i) {
      const auto& use = uses.vertex_uses()[i];
      if (use.parent == parent &&
          use.key.kind == c::FacetVertexKind::SourceVertex &&
          use.key.first == source_nid)
        return i;
    }
    return SIZE_MAX;
  }
  std::size_t RemoteFacet(
      std::uint64_t eid, std::uint32_t vertex_feature,
      const c::SelfContactActiveUseBinding& uses) const {
    const auto parent = Parent(eid, uses);
    const auto& use = uses.parents()[parent];
    for (std::size_t f = use.facet_offset;
         f < std::size_t(use.facet_offset)+use.facet_count; ++f) {
      bool incident = false;
      for (const auto feature : uses.facet_uses()[f].vertex_features)
        incident = incident || feature == vertex_feature;
      if (!incident) return f;
    }
    return SIZE_MAX;
  }
  c::WeightedSurfacePoint FacePoint(
      std::size_t facet_index,
      const c::SelfContactActiveUseBinding& uses) const {
    const auto& use = uses.facet_uses()[facet_index];
    const auto& parent = uses.parents()[use.parent];
    c::FixedContactFacet facet;
    EXPECT_EQ(facets.Describe(
        parent.surface_parent, use.local_facet, &facet).status,
        c::FixedContactFacetStatus::Ok);
    constexpr std::array<double, 3> barycentric{
        {1./3, 1./3, 1./3}};
    c::WeightedSurfacePoint point;
    EXPECT_EQ(c::ComposeFacetPoint(
        facet, barycentric.data(), domain.node_count(),
        &point), c::Status::kOk);
    return point;
  }
  std::vector<std::uint8_t> Active(
      const c::SelfContactActiveUseBinding& uses) const {
    return std::vector<std::uint8_t>(uses.parents().size(), 1);
  }
};

struct LargeTie {
  std::array<std::int32_t, 8192> decode{};
  std::vector<tied::KinChkSlave> slaves;
  tied::PostKinChkResult post;
  std::vector<tied::CinAttachmentDeclaration> declarations;
  tied::TiedCinAttachmentModel model;
  std::vector<cin::WitnessRange> ranges;
  std::vector<cin::ActiveWitness> witnesses;

  explicit LargeTie(const CinScaleFixture& fixture)
      : slaves(CinRows), declarations(CinRows),
        ranges(CinRows), witnesses(CinRows) {
    for (std::size_t i = 0; i < decode.size(); ++i)
      decode[i] = (i&2) != 0;
    for (std::size_t row = 0; row < CinRows; ++row) {
      const auto secondary =
          row == UnrelatedCinRows ? UINT64_C(20) : UINT64_C(10000)+row;
      slaves[row] = {
          static_cast<std::uint32_t>(secondary), 0, {2, 7, 7, 0, 0}};
      auto& declaration = declarations[row];
      declaration.original_nsv_row = static_cast<std::uint32_t>(row+1);
      declaration.ordered_master_rank = 19;
      declaration.master_source = {
          tied::CinMasterSourceKind::DeclaredShellElement, 100, 1000};
      declaration.topology = tied::CinMasterTopology::Quad;
      declaration.secondary_source_id = secondary;
      declaration.reference_positions[0] =
          fixture.domain.nodes()[fixture.domain.Find(secondary)].position;
      for (unsigned i = 0; i < 4; ++i) {
        declaration.master_source_ids[i] = 10+i;
        declaration.reference_positions[i+1] =
            fixture.domain.nodes()[fixture.domain.Find(10+i)].position;
      }
      ranges[row] = {
          static_cast<std::uint32_t>(row), 1};
      auto& witness = witnesses[row];
      witness.source_element_id = 100;
      witness.native_parent_index = 0;
      witness.family = cin::WitnessFamily::ShellQuad;
      for (unsigned i = 0; i < 4; ++i)
        witness.nodes[i] =
            static_cast<std::uint32_t>(fixture.domain.Find(10+i));
    }
    const tied::KinChkInput kin{
        tied::KinChkProfile::NoWallRbeOrCyclic,
        tied::ClassificationPhase::InterfaceTaggedBeforeKinChk,
        77, 881, {slaves.data(), slaves.size()},
        {decode.data(), decode.size()}};
    EXPECT_TRUE(tied::PostKinChk(
        kin, &post, tied::KinChkLimits{}));
    EXPECT_TRUE(tied::PrepareCinAttachments(
        post, fixture.domain,
        {declarations.data(), declarations.size()}, &model,
        tied::CinAttachmentLimits{}));
  }

  c::SelfContactCinWitnessSource Source() const {
    return {&model, ranges.data(), witnesses.data(),
        ranges.size(), witnesses.size()};
  }
};

bool RowContains(
    const tied::CinAttachmentRow& row, std::uint32_t node) {
  for (const auto candidate : row.master_domain_nodes)
    if (candidate == node) return true;
  return false;
}

bool WitnessContains(
    const cin::ActiveWitness& witness, std::uint32_t node) {
  for (const auto candidate : witness.nodes)
    if (candidate == node) return true;
  return false;
}

bool SlowWitnessAuthenticates(
    c::SelfContactCinWitnessSource source, std::size_t row_index,
    const c::SelfContactParentUse& parent) {
  const auto range = source.ranges[row_index];
  for (std::size_t i = range.offset;
       i < std::size_t(range.offset)+range.count; ++i) {
    if (i >= source.witness_count) return false;
    const auto& witness = source.witnesses[i];
    const bool triangle = parent.arity == 3;
    if (witness.source_element_id !=
            parent.source.source_parent_id ||
        witness.native_parent_index != parent.source.family_index ||
        witness.family != (triangle ?
            cin::WitnessFamily::ShellTriangle :
            cin::WitnessFamily::ShellQuad))
      continue;
    bool complete = true;
    for (unsigned n = 0; n < parent.arity; ++n)
      complete = complete &&
          WitnessContains(witness, parent.nodes[n]);
    if (complete) return true;
  }
  return false;
}

c::SelfContactTiedStatus SlowDirectionalTied(
    c::SelfContactCinWitnessSource source,
    const c::WeightedSurfacePoint& secondary,
    const c::SelfContactParentUse& master_parent,
    const c::WeightedSurfacePoint& master,
    std::uint64_t* row_visits) {
  if (!source.range_count)
    return c::SelfContactTiedStatus::NotRelated;
  const auto rows = source.model->rows();
  bool any_local = false, complete = true;
  for (unsigned slot = 0; slot < secondary.count; ++slot) {
    if (secondary.weights[slot] == 0) continue;
    const auto node = secondary.nodes[slot];
    bool matched = false, local = false;
    for (std::size_t r = 0; r < source.range_count; ++r) {
      ++*row_visits;
      const auto& row = rows.data[r];
      if (row.secondary_domain_node != node ||
          row.master_source.element_id !=
              master_parent.source.source_parent_id)
        continue;
      local = true;
      bool contains = true;
      for (unsigned m = 0; m < master.count; ++m)
        if (master.weights[m] != 0 &&
            !RowContains(row, master.nodes[m]))
          contains = false;
      if (contains &&
          SlowWitnessAuthenticates(source, r, master_parent))
        matched = true;
    }
    any_local = any_local || local;
    complete = complete && matched;
  }
  if (!any_local) return c::SelfContactTiedStatus::NotRelated;
  return complete ?
      c::SelfContactTiedStatus::
          CompleteLocalSupportNeedsRuntimeActivity :
      c::SelfContactTiedStatus::
          PartialOrUnauthenticatedLocalSupportNotExcluded;
}

c::SelfContactTiedStatus SlowTied(
    c::SelfContactCinWitnessSource source,
    const c::SelfContactParentUse& first_parent,
    const c::WeightedSurfacePoint& first,
    const c::SelfContactParentUse& second_parent,
    const c::WeightedSurfacePoint& second,
    std::uint64_t* row_visits) {
  const auto a = SlowDirectionalTied(
      source, first, second_parent, second, row_visits);
  const auto b = SlowDirectionalTied(
      source, second, first_parent, first, row_visits);
  if (a == c::SelfContactTiedStatus::
          CompleteLocalSupportNeedsRuntimeActivity ||
      b == c::SelfContactTiedStatus::
          CompleteLocalSupportNeedsRuntimeActivity)
    return c::SelfContactTiedStatus::
        CompleteLocalSupportNeedsRuntimeActivity;
  if (a == c::SelfContactTiedStatus::
          PartialOrUnauthenticatedLocalSupportNotExcluded ||
      b == c::SelfContactTiedStatus::
          PartialOrUnauthenticatedLocalSupportNotExcluded)
    return c::SelfContactTiedStatus::
        PartialOrUnauthenticatedLocalSupportNotExcluded;
  return c::SelfContactTiedStatus::NotRelated;
}

std::size_t NonzeroSlots(const c::WeightedSurfacePoint& point) {
  std::size_t count = 0;
  for (unsigned i = 0; i < point.count; ++i)
    count += point.weights[i] != 0;
  return count;
}

c::SelfContactPairStatus SlowCommonStatus(
    c::SelfContactPairClassification& pair) {
  const auto unsupported =
      c::SelfContactSupportStatus::UnsupportedCinSecondary;
  if (!pair.active[0] || !pair.active[1])
    return c::SelfContactPairStatus::InactiveParent;
  if (pair.local_incidence) {
    pair.excluded = true;
    return c::SelfContactPairStatus::ExcludedLocalIncidence;
  }
  if (pair.parent[0] == pair.parent[1])
    return c::SelfContactPairStatus::
        SameParentNeedsCurrentRegularity;
  if (pair.endpoint_support[0].status == unsupported ||
      pair.endpoint_support[1].status == unsupported)
    return c::SelfContactPairStatus::UnsupportedCinSecondary;
  if (pair.endpoint_support[0].status ==
          c::SelfContactSupportStatus::CompleteRigidGroup &&
      pair.endpoint_support[1].status ==
          c::SelfContactSupportStatus::CompleteRigidGroup &&
      pair.endpoint_support[0].complete_rigid_group ==
          pair.endpoint_support[1].complete_rigid_group) {
    pair.excluded = true;
    return c::SelfContactPairStatus::ExcludedSameRigidGroup;
  }
  if (pair.tied == c::SelfContactTiedStatus::
          CompleteLocalSupportNeedsRuntimeActivity)
    return c::SelfContactPairStatus::
        UnresolvedTiedSupportNotExcluded;
  return c::SelfContactPairStatus::AdmittedVertexFace;
}

void SlowVertexFaceOracle(
    const c::SelfContactActiveUseBinding& uses,
    std::size_t vertex_index, std::size_t facet_index,
    const c::WeightedSurfacePoint& face_point,
    c::SelfContactActivityView activity,
    c::SelfContactTiedStatus tied_status,
    c::SelfContactPairClassification* output) {
  auto& pair = *output;
  const auto& vertex = uses.vertex_uses()[vertex_index];
  const auto& facet = uses.facet_uses()[facet_index];
  const auto& first_parent = uses.parents()[vertex.parent];
  const auto& second_parent = uses.parents()[facet.parent];
  pair.binding_identity = uses.identity();
  pair.activity_base_identity = activity.base;
  pair.activity_current_identity = activity.current;
  pair.activity_parent_count = activity.parent_count;
  pair.kind = c::SelfContactPairKind::VertexFace;
  pair.parent[0] = vertex.parent;
  pair.parent[1] = facet.parent;
  pair.feature[0] = vertex.feature;
  pair.feature[1] = static_cast<std::uint32_t>(facet_index);
  pair.active[0] = activity.current[vertex.parent] != 0;
  pair.active[1] = activity.current[facet.parent] != 0;
  if (pair.active[0])
    pair.reference_half_thickness_m[0] =
        first_parent.reference_half_thickness_m;
  if (pair.active[1])
    pair.reference_half_thickness_m[1] =
        second_parent.reference_half_thickness_m;
  pair.endpoint_support[0] = vertex.support;
  EXPECT_EQ(uses.ClassifySupport(
      face_point, pair.endpoint_support+1).status, Code::Ok);
  for (const auto feature : facet.vertex_features)
    if (feature == vertex.feature)
      pair.local_incidence = true;
  pair.tied = tied_status;
  if (pair.active[0] && pair.active[1])
    pair.candidate_directed_area_m2 =
        vertex.directed_vf_area_m2;
  pair.status = SlowCommonStatus(pair);
  if (pair.status ==
      c::SelfContactPairStatus::AdmittedVertexFace)
    pair.admitted_force_area_m2 =
        pair.candidate_directed_area_m2;
}
} // namespace

TEST(SelfContactActiveUses,
    CinSecondaryIndexMatchesSlowOracleAtMediumQueryCoupon) {
  CinScaleFixture fixture(2);
  c::SelfContactActiveUseBinding uses;
  c::SelfContactActiveUseForecast forecast;
  const tied::CinAttachmentRow* source_rows = nullptr;
  tied::CinAttachmentRow late_row;
  cin::WitnessRange late_range;
  cin::ActiveWitness late_witness;
  {
    LargeTie tie(fixture);
    const c::SelfContactActiveUseSource source{
        nullptr, tie.Source()};
    auto limits = c::SelfContactActiveUseLimits::Vehicle();
    limits.max_host_bytes = 512u << 20;
    const auto plan = c::SelfContactActiveUseBinding::Preflight(
        fixture.facets, source, limits);
    ASSERT_EQ(plan.report.status, Code::Ok);
    ASSERT_LE(plan.forecast.startup_payload_bytes,
        limits.max_host_bytes);
    EXPECT_EQ(plan.forecast.cin_rows, CinRows);
    EXPECT_EQ(plan.forecast.node_roles,
        fixture.domain.node_count());
    EXPECT_EQ(plan.forecast.cin_index_bytes,
        2*sizeof(std::uint32_t)*plan.forecast.node_roles +
        sizeof(std::uint32_t)*plan.forecast.cin_rows);
    auto short_limits = limits;
    short_limits.max_host_bytes =
        plan.forecast.startup_payload_bytes-1;
    c::SelfContactActiveUseBinding short_binding;
    EXPECT_EQ(short_binding.Initialize(
        fixture.facets, source, short_limits).status,
        Code::ResourceLimit);
    limits.max_host_bytes = plan.forecast.startup_payload_bytes;
    ASSERT_EQ(uses.Initialize(
        fixture.facets, source, limits).status, Code::Ok);
    forecast = uses.forecast();
    source_rows = tie.model.rows().data;
    late_row = source_rows[UnrelatedCinRows];
    late_range = tie.ranges[UnrelatedCinRows];
    late_witness = tie.witnesses[UnrelatedCinRows];
  }

  ASSERT_EQ(uses.cin().range_count, CinRows);
  ASSERT_EQ(uses.cin().witness_count, CinRows);
  EXPECT_EQ(uses.cin().model->rows().data, source_rows);
  EXPECT_EQ(std::memcmp(
      uses.cin().model->rows().data+UnrelatedCinRows,
      &late_row, sizeof(late_row)), 0);
  EXPECT_EQ(std::memcmp(
      uses.cin().ranges+UnrelatedCinRows,
      &late_range, sizeof(late_range)), 0);
  EXPECT_EQ(std::memcmp(
      uses.cin().witnesses+UnrelatedCinRows,
      &late_witness, sizeof(late_witness)), 0);
  EXPECT_EQ(uses.forecast().cin_index_bytes,
      forecast.cin_index_bytes);

  const auto secondary = fixture.VertexUse(101, uses, 20);
  ASSERT_NE(secondary, SIZE_MAX);
  const auto master_face = fixture.RemoteFacet(
      100, uses.vertex_uses()[secondary].feature, uses);
  ASSERT_NE(master_face, SIZE_MAX);
  const auto master_point =
      fixture.FacePoint(master_face, uses);
  const auto active = fixture.Active(uses);
  const c::SelfContactActivityView activity{
      active.data(), active.data(), active.size()};
  const auto& secondary_use = uses.vertex_uses()[secondary];
  const auto& secondary_parent =
      uses.parents()[secondary_use.parent];
  const auto& master_facet = uses.facet_uses()[master_face];
  const auto& master_parent =
      uses.parents()[master_facet.parent];

  std::uint64_t oracle_visits = 0;
  c::SelfContactTiedStatus oracle =
      c::SelfContactTiedStatus::NotRelated;
  const auto oracle_begin = std::chrono::steady_clock::now();
  bool oracle_stable = true;
  for (std::size_t query = 0; query < OracleSamples; ++query) {
    const auto status = SlowTied(
        uses.cin(), secondary_parent, secondary_use.point,
        master_parent, master_point, &oracle_visits);
    if (!query) oracle = status;
    else oracle_stable = oracle_stable && status == oracle;
  }
  const auto oracle_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now()-oracle_begin).count();
  ASSERT_TRUE(oracle_stable);
  ASSERT_EQ(oracle,
      c::SelfContactTiedStatus::
          CompleteLocalSupportNeedsRuntimeActivity);
  EXPECT_EQ(oracle_visits,
      OracleSamples*CinRows*
          (NonzeroSlots(secondary_use.point) +
           NonzeroSlots(master_point)));

  c::SelfContactPairClassification oracle_pair;
  std::memset(&oracle_pair, 0, sizeof(oracle_pair));
  SlowVertexFaceOracle(
      uses, secondary, master_face, master_point,
      activity, oracle, &oracle_pair);
  c::SelfContactPairClassification expected;
  std::memset(&expected, 0, sizeof(expected));
  ASSERT_EQ(uses.ClassifyVertexFace(
      secondary, master_face, master_point, activity,
      &expected).status, Code::Ok);
  ASSERT_EQ(expected.tied, oracle);
  ASSERT_EQ(expected.status, oracle_pair.status);
  ASSERT_EQ(expected.endpoint_support[0].status,
      oracle_pair.endpoint_support[0].status);
  ASSERT_EQ(expected.endpoint_support[1].status,
      oracle_pair.endpoint_support[1].status);
  ASSERT_EQ(expected.excluded, oracle_pair.excluded);
  bool exact = true;
  bool reports_ok = true;
  const auto indexed_begin = std::chrono::steady_clock::now();
  for (std::size_t query = 0;
       query < MediumQueryCoupon; ++query) {
    c::SelfContactPairClassification actual;
    std::memset(&actual, 0, sizeof(actual));
    reports_ok = reports_ok &&
        uses.ClassifyVertexFace(
            secondary, master_face, master_point, activity,
            &actual).status == Code::Ok;
    if (exact && std::memcmp(
            &actual, &expected, sizeof(actual)) != 0) {
      const auto* actual_bytes =
          reinterpret_cast<const unsigned char*>(&actual);
      const auto* expected_bytes =
          reinterpret_cast<const unsigned char*>(&expected);
      std::size_t mismatch = 0;
      while (mismatch < sizeof(actual) &&
          actual_bytes[mismatch] == expected_bytes[mismatch])
        ++mismatch;
      std::cout << "cin-tied-oracle-mismatch byte=" << mismatch
                << " actual=" << unsigned(actual_bytes[mismatch])
                << " expected=" << unsigned(expected_bytes[mismatch])
                << " actual_status=" << unsigned(actual.status)
                << " expected_status=" << unsigned(expected.status)
                << " actual_tied=" << unsigned(actual.tied)
                << " expected_tied=" << unsigned(expected.tied)
                << '\n';
      exact = false;
    }
    exact = exact && actual.tied == oracle;
  }
  const auto indexed_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now()-indexed_begin).count();
  EXPECT_TRUE(reports_ok);
  EXPECT_TRUE(exact);
  const auto projected_slow_us =
      oracle_us*static_cast<std::int64_t>(MediumQueryCoupon)/
      static_cast<std::int64_t>(OracleSamples);
  const auto projected_slow_row_visits =
      oracle_visits*MediumQueryCoupon/OracleSamples;
  EXPECT_GT(projected_slow_row_visits, MediumQueryCoupon);
  EXPECT_GT(uses.forecast().cin_index_bytes, 0u);
  std::cout << "cin-tied-query-index rows=" << CinRows
            << " queries=" << MediumQueryCoupon
            << " slow_sample_us=" << oracle_us
            << " slow_projected_us=" << projected_slow_us
            << " indexed_us=" << indexed_us
            << " slow_row_visits=" << oracle_visits
            << " slow_projected_row_visits="
            << projected_slow_row_visits << '\n';
}
} // namespace active_use_test
