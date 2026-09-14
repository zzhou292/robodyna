// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../physical_publication/OwnerFixture.h"
#include "lib_src/collision/SelfContactForceAssembly.h"
#include "lib_src/collision/FixedContactFacetValues.h"
#include "lib_src/collision/fixed_triangle_features/Geometry.h"
#include "lib_src/collision/penalty_pair/Values.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <type_traits>
#include <vector>

namespace self_contact_force_cuda_test {
namespace c = tlfea::contact;
namespace fe = tl::fea;
namespace p = physical_publication_test;

bool Good(c::SelfContactForceReport report) {
  EXPECT_EQ(report.status, c::SelfContactForceStatus::Ok)
      << report.message << " event=" << report.event
      << " source=" << report.source_order << " node=" << report.node;
  return report.status == c::SelfContactForceStatus::Ok;
}

struct Fixture {
  explicit Fixture(bool surface_rigid = false,
      p::ContactConstraintLayout constraints =
          p::ContactConstraintLayout::Legacy)
      : rig(surface_rigid, 2.5,
            constraints == p::ContactConstraintLayout::SurfaceCinSecondary,
            constraints) {
    qbat_catalog_test::Fixture declaration;
    for (unsigned row = 0; row < 2; ++row) {
      declaration.materials[row].curve_id = 0;
      declaration.materials[row].hardening =
          tl::material::ShellPlasticityHardeningKind::LinearLaw44;
      declaration.materials[row].linear = {10e6, 0};
      declaration.materials[row].rate = declaration.materials[2].rate;
    }
    const fe::ShellPlasticityParentInput parents[]{
        {fe::ShellBindingFamily::Qeph, 0, 100, 1000, 1000, 1000},
        {fe::ShellBindingFamily::Qeph, 1, 101, 1001, 1001, 1001},
        {fe::ShellBindingFamily::T3, 0, 102, 2000524, 2000524, 2000524},
        {fe::ShellBindingFamily::Qbat, 0, 103, 2000524, 2000524, 2000524}};
    const auto catalog_report = execution_catalog.InitializeExecutionCatalog(
        rig.fixture.source.shells,
        {nullptr, declaration.materials.data(), declaration.sections.data(),
         parents, 0, 3, 3, 4});
    EXPECT_EQ(catalog_report.status,
        fe::ShellPlasticityBindingStatus::Success)
        << catalog_report.message;
    if (catalog_report.status != fe::ShellPlasticityBindingStatus::Success)
      return;
    fe::ShellFailureParentInput failures[4];
    for (unsigned row = 0; row < 4; ++row)
      failures[row] = qbat_catalog_test::Failure(parents[row]);
    failures[2].constant.failure_strain = 2.5;
    for (unsigned row = 0; row < 2; ++row) {
      failures[row].policy = fe::ShellFailurePolicy::Tab1AnyPoint;
      failures[row].constant = {};
      failures[row].tab1.table = {{-1, 0, 1}, 1};
    }
    const auto failure_report = execution_failure.InitializeExecution(
        execution_catalog, failures, 4);
    EXPECT_EQ(failure_report.status,
        fe::ShellPlasticityBindingStatus::Success)
        << failure_report.message;
    if (failure_report.status != fe::ShellPlasticityBindingStatus::Success)
      return;
    const auto execution_report = execution.Initialize(
        execution_catalog, rig.fixture.ledger,
        rig.fixture.rigid);
    EXPECT_EQ(execution_report.status,
        fe::ShellPlasticityBindingStatus::Success)
        << execution_report.message << " entry=" << execution_report.entry;
    EXPECT_TRUE(physical.InitializeExecution(
        {&rig.fixture.source.shells, &execution_catalog,
         &execution_failure, nullptr},
        rig.fixture.ledger, execution));
    for (std::size_t row = 0;
         row < execution_catalog.parent_count(); ++row) {
      const auto& parent = *execution_catalog.parent(row);
      // The physical fixture deliberately keeps two unsupported top/bottom
      // QEPH layers. This self-contact profile selects only its centered
      // native T3 and QBAT parents; no offset is silently reinterpreted.
      if (parent.family != fe::ShellBindingFamily::T3 &&
          parent.family != fe::ShellBindingFamily::Qbat)
        continue;
      selection.push_back({
          row, parent.family, parent.family_index,
          parent.source_parent_id, parent.source_part_id});
    }
    EXPECT_EQ(surface.Initialize(
        physical, {selection.data(), selection.size()}).status,
        c::SelfContactSurfaceStatus::Ok);
    EXPECT_EQ(facets.Initialize(surface, {{}, 1}).status,
        c::FixedContactFacetStatus::Ok);
    const auto cin = rig.fixture.WitnessSource();
    c::SelfContactActiveUseSource source;
    source.rigid = &rig.fixture.rigid;
    source.cin = {
        cin.model, cin.ranges, cin.witnesses,
        cin.range_count, cin.witness_count};
    EXPECT_EQ(uses.Initialize(facets, source).status,
        c::SelfContactActiveUseStatus::Ok);
    activity.assign(uses.parents().size(), 1);
  }

  p::Rig rig;
  fe::ShellBatchPlasticityBinding execution_catalog;
  fe::ShellBatchFailureBinding execution_failure;
  fe::ShellExecutionBinding execution;
  fe::ShellPhysicalBinding physical;
  c::SelfContactSurfaceBinding surface;
  c::FixedContactFacetBinding facets;
  c::SelfContactActiveUseBinding uses;
  c::SelfContactForceAssembly force;
  std::vector<c::SelfContactParentSelection> selection;
  std::vector<std::uint8_t> activity;
  c::SelfContactForceConfig config;

  bool Initialize(c::SelfContactForceLimits limits = {},
                  double stiffness_per_area_n_m3 = 2e9) {
    if (!rig.Initialize()) return false;
    config.owner = rig.owner.accepted();
    config.stiffness_per_area_n_m3 = stiffness_per_area_n_m3;
    config.event_capacity = 64;
    config.configuration_id = p::Configuration;
    config.qualification_id = p::Qualification;
    return Good(force.Initialize(config, uses, rig.owner, limits));
  }

  c::WeightedSurfacePoint FacePoint(std::size_t facet_index) {
    const auto& use = uses.facet_uses()[facet_index];
    const auto& parent = uses.parents()[use.parent];
    c::FixedContactFacet facet;
    EXPECT_EQ(facets.Describe(
        parent.surface_parent, use.local_facet, &facet).status,
        c::FixedContactFacetStatus::Ok);
    const double barycentric[]{.25, .25, .5};
    c::WeightedSurfacePoint point;
    EXPECT_EQ(c::ComposeFacetPoint(
        facet, barycentric,
        rig.fixture.domain.node_count(), &point), c::Status::kOk);
    return point;
  }

  c::SelfContactActivityView Activity() const {
    return {activity.data(), activity.data(), activity.size()};
  }

  std::vector<c::SelfContactForceEvent> Events(
      std::size_t maximum = 8) {
    std::vector<c::SelfContactForceEvent> result;
    const auto state = Activity();
    // A resolved T3 edge-midpoint against a QBAT map 0.2 mm inboard supplies
    // one deterministic active mechanics control. Candidate-geometry
    // authentication is intentionally the next slice; this fixture exercises
    // only the already-resolved weighted maps consumed here.
    std::size_t active_vertex = SIZE_MAX, active_facet = SIZE_MAX;
    for (std::size_t vertex = 0;
         vertex < uses.vertex_uses().size(); ++vertex) {
      const auto& use = uses.vertex_uses()[vertex];
      const auto& parent = uses.parents()[use.parent];
      if (parent.source.source_parent_id == 102 &&
          use.key.kind == c::FacetVertexKind::SourceEdge &&
          use.key.first == 11 && use.key.second == 12) {
        active_vertex = vertex;
        break;
      }
    }
    if (active_vertex != SIZE_MAX) {
      const auto feature = uses.vertex_uses()[active_vertex].feature;
      for (std::size_t facet = 0;
           facet < uses.facet_uses().size(); ++facet) {
        const auto& use = uses.facet_uses()[facet];
        const auto& parent = uses.parents()[use.parent];
        bool incident = false;
        for (const auto vertex : use.vertex_features)
          incident = incident || vertex == feature;
        if (parent.source.source_parent_id == 103 && !incident) {
          active_facet = facet;
          break;
        }
      }
    }
    if (active_facet != SIZE_MAX) {
      const auto& parent =
          uses.parents()[uses.facet_uses()[active_facet].parent];
      c::WeightedSurfacePoint point;
      point.count = parent.arity;
      constexpr double inward = .005;
      for (unsigned slot = 0; slot < parent.arity; ++slot) {
        point.nodes[slot] = parent.nodes[slot];
        const auto x = rig.fixture.domain.nodes()[parent.nodes[slot]].position.x;
        point.weights[slot] = x < .02
            ? .5 * inward : .5 * (1 - inward);
      }
      c::SelfContactPairClassification classification;
      if (uses.ClassifyVertexFace(
              active_vertex, active_facet, point, state,
              &classification).status ==
              c::SelfContactActiveUseStatus::Ok &&
          classification.status ==
              c::SelfContactPairStatus::AdmittedVertexFace) {
        c::SelfContactForceEvent event;
        event.source_order = 900;
        event.vertex_use = static_cast<std::uint32_t>(active_vertex);
        event.facet_use = static_cast<std::uint32_t>(active_facet);
        event.endpoints[0] =
            uses.vertex_uses()[active_vertex].point;
        event.endpoints[1] = point;
        event.classification = classification;
        event.feature.vertex_face.vertex =
            uses.vertex_uses()[active_vertex].key;
        event.feature.vertex_face.target.SetFace({
            rig.fixture.domain.source_instance_id(),
            parent.source.source_parent_id, 1,
            uses.facet_uses()[active_facet].local_facet});
        result.push_back(event);
      }
    }
    for (std::size_t vertex = 0;
         vertex < uses.vertex_uses().size() &&
             result.size() < maximum; ++vertex) {
      for (std::size_t facet = 0;
           facet < uses.facet_uses().size() &&
               result.size() < maximum; ++facet) {
        const auto point = FacePoint(facet);
        c::SelfContactPairClassification classification;
        if (uses.ClassifyVertexFace(
                vertex, facet, point, state,
                &classification).status !=
                c::SelfContactActiveUseStatus::Ok ||
            classification.status !=
                c::SelfContactPairStatus::AdmittedVertexFace)
          continue;
        c::SelfContactForceEvent event;
        event.source_order = 1000 + result.size();
        event.vertex_use = static_cast<std::uint32_t>(vertex);
        event.facet_use = static_cast<std::uint32_t>(facet);
        event.endpoints[0] = uses.vertex_uses()[vertex].point;
        event.endpoints[1] = point;
        event.classification = classification;
        event.feature.vertex_face.vertex =
            uses.vertex_uses()[vertex].key;
        const auto& facet_use = uses.facet_uses()[facet];
        const auto& parent = uses.parents()[facet_use.parent];
        event.feature.vertex_face.target.SetFace({
            rig.fixture.domain.source_instance_id(),
            parent.source.source_parent_id, 1,
            facet_use.local_facet});
        bool duplicate = false;
        for (const auto& prior : result)
          duplicate = duplicate ||
              !c::fixed_triangle_features::Compare(
                  prior.feature, event.feature);
        if (!duplicate) result.push_back(event);
      }
    }
    return result;
  }

  void Discard() {
    rig.owner.Discard();
    rig.publication.DiscardTrial();
    rig.qeph.DiscardTrial();
    rig.t3.DiscardTrial();
    rig.qbat.DiscardTrial();
    rig.welds.DiscardTrial();
    rig.beams.DiscardTrial();
    rig.solids.DiscardTrial();
    force.DiscardTrial();
  }

  bool Commit(const fe::NodalTrialToken& token,
              const fe::NodalAssemblyView& assembly) {
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics candidates, common;
    if (!rig.Advance(token, assembly, prepared) ||
        !rig.Evaluate(token, prepared, candidates) ||
        !p::Good(rig.publication.PreparePhysical(
            rig.owner, token,
            {&candidates.qeph, &candidates.t3, &candidates.qbat,
             &candidates.type25, &candidates.type13,
             &candidates.solids},
            &common)))
      return false;
    return p::Good(rig.publication.CommitPhysical(
        rig.owner, token, common,
        {prepared.owner_id, prepared.kinematics.base_epoch,
         prepared.attempt, p::Qualification, true}));
  }
};

struct AssemblySnapshot {
  std::size_t nodes = 0;
  std::vector<double> values;
  explicit AssemblySnapshot(std::size_t count)
      : nodes(count), values(8 * count) {}
  bool Read(const fe::NodalAssemblyView& view,
            const fe::NodalCinAssemblyView& cin) {
    double* arrays[]{
        view.forces.force_x, view.forces.force_y, view.forces.force_z,
        view.forces.couple_x, view.forces.couple_y,
        view.forces.couple_z, cin.translational_stiffness,
        cin.rotational_stiffness};
    for (unsigned channel = 0; channel < 8; ++channel)
      if (cudaMemcpyAsync(
              values.data() + channel * nodes, arrays[channel],
              nodes * sizeof(double), cudaMemcpyDeviceToHost,
              view.stream) != cudaSuccess)
        return false;
    return cudaStreamSynchronize(view.stream) == cudaSuccess;
  }
  c::Vec3 Vector(unsigned channel, std::size_t node) const {
    return {values[channel * nodes + node],
            values[(channel + 1) * nodes + node],
            values[(channel + 2) * nodes + node]};
  }
};

TEST(SelfContactForceCuda,
     ExactForecastCapsSourceAuthorityAliasesAndStableAllocation) {
  Fixture f;
  ASSERT_TRUE(f.rig.Initialize());
  f.config.owner = f.rig.owner.accepted();
  f.config.stiffness_per_area_n_m3 = 2e9;
  f.config.event_capacity = 64;
  f.config.configuration_id = p::Configuration;
  f.config.qualification_id = p::Qualification;
  const auto forecast =
      c::SelfContactForceAssembly::Forecast(f.config, f.uses);
  ASSERT_EQ(forecast.report.status, c::SelfContactForceStatus::Ok);
  EXPECT_EQ(forecast.forecast.device_allocations, 1u);
  EXPECT_EQ(forecast.forecast.incidence_capacity,
            8 * f.config.event_capacity);
  ASSERT_GE(f.uses.forecast().owned_payload_bytes,
            sizeof(c::SelfContactActiveUseBinding));
  EXPECT_EQ(forecast.forecast.retained_active_use_bytes,
            f.uses.forecast().owned_payload_bytes -
                sizeof(c::SelfContactActiveUseBinding));
  auto limit = c::SelfContactForceLimits{};
  limit.max_device_bytes = forecast.forecast.device_bytes - 1;
  EXPECT_EQ(f.force.Initialize(
      f.config, f.uses, f.rig.owner, limit).status,
      c::SelfContactForceStatus::ResourceLimit);
  EXPECT_EQ(f.force.allocations().device_allocations, 0u);
  limit = {};
  limit.max_startup_host_bytes =
      forecast.forecast.startup_host_bytes - 1;
  EXPECT_EQ(f.force.Initialize(
      f.config, f.uses, f.rig.owner, limit).status,
      c::SelfContactForceStatus::ResourceLimit);
  ASSERT_TRUE(Good(f.force.Initialize(
      f.config, f.uses, f.rig.owner)));
  EXPECT_EQ(f.force.allocations().device_bytes,
            forecast.forecast.device_bytes);
}

TEST(SelfContactForceCuda,
     EmptyBatchPreservesEveryForceAndCinChannelExactly) {
  Fixture f;
  ASSERT_TRUE(f.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(f.rig.Begin(token, assembly));
  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(p::Good(f.rig.owner.BorrowCinAssembly(token, &cin)));
  AssemblySnapshot before(f.rig.fixture.domain.node_count()), after(before.nodes);
  ASSERT_TRUE(before.Read(assembly, cin));
  c::SelfContactForceAssemblyReceipt receipt;
  ASSERT_TRUE(Good(f.force.AssembleAccepted(
      f.rig.owner, token, assembly, f.Activity(), {nullptr, 0},
      &receipt)));
  ASSERT_TRUE(receipt.prepared());
  EXPECT_EQ(receipt.diagnostics().event_count, 0u);
  EXPECT_EQ(receipt.diagnostics().active_count, 0u);
  EXPECT_EQ(receipt.diagnostics().first_source_order, UINT64_MAX);
  EXPECT_EQ(receipt.diagnostics().last_source_order, UINT64_MAX);
  ASSERT_TRUE(after.Read(assembly, cin));
  EXPECT_EQ(after.values, before.values);
  f.Discard();
}

TEST(SelfContactForceCuda,
     ExactOwnerViewRejectsForgedSourcesDestinationsStreamAndToken) {
  Fixture f;
  ASSERT_TRUE(f.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(f.rig.Begin(token, assembly));
  EXPECT_EQ(f.rig.owner.AuthenticateAssemblyView(token, assembly).status,
            fe::NodalStatus::Ok);
  for (unsigned kind = 0; kind < 9; ++kind) {
    SCOPED_TRACE(kind);
    auto forged = assembly;
    if (kind == 0)
      forged.accepted.position_xyz = assembly.forces.force_x;
    if (kind == 1)
      forged.mass.inverse_mass = assembly.forces.force_x;
    if (kind == 2)
      forged.forces.force_x =
          const_cast<double*>(assembly.accepted.position_xyz);
    if (kind == 3)
      forged.bounds = reinterpret_cast<fe::stability::RowBounds*>(
          forged.forces.force_x);
    if (kind == 4)
      forged.result = reinterpret_cast<fe::NodalAssemblyResult*>(
          forged.forces.force_x);
    if (kind == 5) forged.stream = nullptr;
    if (kind == 6) ++forged.forces.node_count;
    if (kind == 7) ++forged.forces.base_epoch;
    if (kind == 8) ++forged.attempt;
    EXPECT_EQ(
        f.rig.owner.AuthenticateAssemblyView(token, forged).status,
        fe::NodalStatus::StaleTrial);
    c::SelfContactForceAssemblyReceipt receipt;
    const auto report = f.force.AssembleAccepted(
        f.rig.owner, token, forged, f.Activity(), {nullptr, 0},
        &receipt);
    EXPECT_EQ(report.status, c::SelfContactForceStatus::OwnerFailure);
    EXPECT_FALSE(receipt.prepared());
  }
  fe::NodalTrialToken forged_token;
  EXPECT_EQ(f.rig.owner.AuthenticateAssemblyView(
      forged_token, assembly).status, fe::NodalStatus::StaleTrial);
  c::SelfContactForceAssemblyReceipt receipt;
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, forged_token, assembly, f.Activity(), {nullptr, 0},
      &receipt).status, c::SelfContactForceStatus::OwnerFailure);

  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(p::Good(f.rig.owner.BorrowCinAssembly(token, &cin)));
  EXPECT_FALSE(f.rig.owner.AssemblyRangeDisjoint(
      token, assembly, cin.translational_stiffness, sizeof(double)));
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, token, assembly, f.Activity(),
      {reinterpret_cast<const c::SelfContactForceEvent*>(
           cin.translational_stiffness), 1},
      &receipt).status, c::SelfContactForceStatus::InvalidInput);
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, token, assembly, f.Activity(), {nullptr, 0},
      reinterpret_cast<c::SelfContactForceAssemblyReceipt*>(
          cin.rotational_stiffness)).status,
      c::SelfContactForceStatus::InvalidInput);
  ASSERT_TRUE(Good(f.force.AssembleAccepted(
      f.rig.owner, token, assembly, f.Activity(), {nullptr, 0},
      &receipt)));
  f.Discard();
}

TEST(SelfContactForceCuda,
     DiagnosticReceiptCannotAuthenticateAfterAssemblerAddressReuse) {
  Fixture f;
  ASSERT_TRUE(f.Initialize());
  using Storage = std::aligned_storage_t<
      sizeof(c::SelfContactForceAssembly),
      alignof(c::SelfContactForceAssembly)>;
  Storage storage;
  auto* first = new (&storage) c::SelfContactForceAssembly;
  ASSERT_TRUE(Good(first->Initialize(
      f.config, f.uses, f.rig.owner)));
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(f.rig.Begin(token, assembly));
  c::SelfContactForceAssemblyReceipt receipt;
  ASSERT_TRUE(Good(first->AssembleAccepted(
      f.rig.owner, token, assembly, f.Activity(), {nullptr, 0},
      &receipt)));
  EXPECT_TRUE(first->Authenticates(receipt));
  first->~SelfContactForceAssembly();
  f.rig.owner.Discard();
  EXPECT_TRUE(receipt.prepared());
  auto* replacement = new (&storage) c::SelfContactForceAssembly;
  ASSERT_TRUE(Good(replacement->Initialize(
      f.config, f.uses, f.rig.owner)));
  EXPECT_FALSE(replacement->Authenticates(receipt));
  replacement->~SelfContactForceAssembly();
}

TEST(SelfContactForceCuda,
     OrdinaryInitialHalfKickAndIntervalPreserveCouplesStirAndBalance) {
  Fixture f;
  ASSERT_TRUE(f.Initialize());
  const auto events = f.Events();
  ASSERT_FALSE(events.empty());
  auto ordered = events;
  std::vector<c::SelfContactForceIncidence> incidence(8 * events.size());
  std::vector<c::SelfContactForceNodeIncidence> node_ranges(
      f.rig.fixture.domain.node_count());
  c::SelfContactForceIncidenceSummary incidence_summary;
  ASSERT_EQ(c::BuildSelfContactForceIncidence(
      ordered.data(), ordered.size(),
      f.rig.fixture.domain.node_count(), incidence.data(),
      incidence.size(), node_ranges.data(), node_ranges.size(),
      &incidence_summary).status, c::SelfContactForceStatus::Ok);
  EXPECT_TRUE(std::any_of(
      node_ranges.begin(),
      node_ranges.begin() + incidence_summary.touched_nodes,
      [](const auto& node) { return node.count > 1; }));
  const auto allocation = f.force.allocations();
  for (unsigned interval = 0; interval < 2; ++interval) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(f.rig.Begin(token, assembly));
    fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(p::Good(f.rig.owner.BorrowCinAssembly(token, &cin)));
    AssemblySnapshot before(f.rig.fixture.domain.node_count()), after(before.nodes);
    ASSERT_TRUE(before.Read(assembly, cin));
    c::SelfContactForceAssemblyReceipt receipt;
    ASSERT_TRUE(Good(f.force.AssembleAccepted(
        f.rig.owner, token, assembly,
        f.Activity(),
        {events.data(), events.size()}, &receipt)));
    ASSERT_TRUE(receipt.prepared());
    const auto& diagnostics = receipt.diagnostics();
    EXPECT_EQ(diagnostics.event_count, events.size());
    EXPECT_GT(diagnostics.active_count, 0u);
    EXPECT_GT(diagnostics.maximum_force_norm_n, 0);
    EXPECT_GT(diagnostics.maximum_sti_diagonal_n_m, 0);
    EXPECT_EQ(diagnostics.owner_id, assembly.owner_id);
    EXPECT_EQ(diagnostics.base_epoch, interval);
    EXPECT_EQ(diagnostics.attempt, assembly.attempt);
    EXPECT_EQ(diagnostics.active_use_identity, f.uses.identity());
    EXPECT_GE(diagnostics.potential_j, 0);
    EXPECT_NEAR(diagnostics.equal_opposite_residual_n.x, 0, 1e-12);
    EXPECT_NEAR(diagnostics.equal_opposite_residual_n.y, 0, 1e-12);
    EXPECT_NEAR(diagnostics.equal_opposite_residual_n.z, 0, 1e-12);
    ASSERT_TRUE(after.Read(assembly, cin));
    for (unsigned channel : {3u, 4u, 5u, 7u})
      for (std::size_t node = 0; node < before.nodes; ++node)
        EXPECT_EQ(std::memcmp(
            &before.values[channel * before.nodes + node],
            &after.values[channel * before.nodes + node],
            sizeof(double)), 0);
    std::vector<double> kinematics(6 * before.nodes);
    ASSERT_EQ(cudaMemcpyAsync(
        kinematics.data(), assembly.accepted.position_xyz,
        3 * before.nodes * sizeof(double), cudaMemcpyDeviceToHost,
        assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(
        kinematics.data() + 3 * before.nodes,
        assembly.accepted.velocity_xyz,
        3 * before.nodes * sizeof(double), cudaMemcpyDeviceToHost,
        assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream), cudaSuccess);
    c::Vec3 resultant, moment;
    double virtual_power = 0;
    for (std::size_t node = 0; node < before.nodes; ++node) {
      const c::Vec3 increment{
          after.values[node] - before.values[node],
          after.values[before.nodes + node] -
              before.values[before.nodes + node],
          after.values[2 * before.nodes + node] -
              before.values[2 * before.nodes + node]};
      const c::Vec3 x{
          kinematics[3 * node], kinematics[3 * node + 1],
          kinematics[3 * node + 2]};
      const c::Vec3 velocity{
          kinematics[3 * before.nodes + 3 * node],
          kinematics[3 * before.nodes + 3 * node + 1],
          kinematics[3 * before.nodes + 3 * node + 2]};
      resultant = c::Add(resultant, increment);
      moment = c::Add(moment, {
          x.y * increment.z - x.z * increment.y,
          x.z * increment.x - x.x * increment.z,
          x.x * increment.y - x.y * increment.x});
      virtual_power += c::Dot(increment, velocity);
    }
    double endpoint_virtual_power = 0;
    for (const auto& event : events) {
      c::SurfacePenaltyInput input;
      input.positions = {
          kinematics.data(), static_cast<std::uint32_t>(before.nodes), 3, 1};
      input.velocities = {
          kinematics.data() + 3 * before.nodes,
          static_cast<std::uint32_t>(before.nodes), 3, 1};
      ASSERT_TRUE(c::RepresentedSelfContactStiffness(
          f.config.stiffness_per_area_n_m3,
          event.classification.admitted_force_area_m2,
          &input.stiffness_n_m));
      for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
        auto& target = endpoint ? input.b : input.a;
        target.point = event.endpoints[endpoint];
        target.reference_half_thickness_m =
            event.classification.reference_half_thickness_m[endpoint];
        for (unsigned slot = 0; slot < target.point.count; ++slot)
          target.translation_fixed_bits[slot] =
              f.rig.fixture.fixed[target.point.nodes[slot]];
      }
      c::SurfacePenaltyPacket packet;
      ASSERT_EQ(c::EvaluateSurfacePenaltyPair(input, &packet),
                c::SurfacePenaltyStatus::Ok);
      endpoint_virtual_power +=
          c::Dot(packet.force_a_n, packet.a.velocity) +
          c::Dot(packet.force_b_n, packet.b.velocity);
    }
    const double tolerance =
        1e-10 * (1 + diagnostics.maximum_force_norm_n);
    EXPECT_NEAR(resultant.x,
                diagnostics.equal_opposite_residual_n.x, tolerance);
    EXPECT_NEAR(resultant.y,
                diagnostics.equal_opposite_residual_n.y, tolerance);
    EXPECT_NEAR(resultant.z,
                diagnostics.equal_opposite_residual_n.z, tolerance);
    EXPECT_NEAR(moment.x, diagnostics.global_moment_n_m.x, tolerance);
    EXPECT_NEAR(moment.y, diagnostics.global_moment_n_m.y, tolerance);
    EXPECT_NEAR(moment.z, diagnostics.global_moment_n_m.z, tolerance);
    EXPECT_NEAR(virtual_power, endpoint_virtual_power, tolerance);
    EXPECT_EQ(f.force.allocations().device_bytes, allocation.device_bytes);
    EXPECT_EQ(f.force.allocations().device_allocations,
              allocation.device_allocations);
    ASSERT_TRUE(f.Commit(token, assembly));
    EXPECT_EQ(p::Bits(f.rig.owner.accepted().reaction_kick_dt),
              p::Bits(interval ? p::H : .5 * p::H));
  }
}

TEST(SelfContactForceCuda,
     EventPermutationPreservesCanonicalAssemblyAndAllocationExactly) {
  Fixture f;
  ASSERT_TRUE(f.Initialize());
  const auto ordered=f.Events(32);
  ASSERT_GT(ordered.size(),1u);
  auto reversed=ordered;
  std::reverse(reversed.begin(),reversed.end());
  const auto allocation=f.force.allocations();
  std::vector<double> reference;
  c::SelfContactForceDiagnostics first_diagnostics;
  for (unsigned pass=0;pass<2;++pass) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(f.rig.Begin(token,assembly));
    fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(p::Good(f.rig.owner.BorrowCinAssembly(token,&cin)));
    AssemblySnapshot before(f.rig.fixture.domain.node_count()),after(before.nodes);
    ASSERT_TRUE(before.Read(assembly,cin));
    const auto& events=pass ? reversed : ordered;
    c::SelfContactForceAssemblyReceipt receipt;
    ASSERT_TRUE(Good(f.force.AssembleAccepted(
        f.rig.owner,token,assembly,f.Activity(),
        {events.data(),events.size()},&receipt)));
    ASSERT_TRUE(after.Read(assembly,cin));
    std::vector<double> increment(after.values.size());
    for (std::size_t i=0;i<increment.size();++i)
      increment[i]=after.values[i]-before.values[i];
    if (!pass) {
      reference=increment;
      first_diagnostics=receipt.diagnostics();
    } else {
      EXPECT_EQ(increment,reference);
      EXPECT_EQ(receipt.diagnostics().first_source_order,
                first_diagnostics.first_source_order);
      EXPECT_EQ(receipt.diagnostics().last_source_order,
                first_diagnostics.last_source_order);
      EXPECT_EQ(receipt.diagnostics().event_count,
                first_diagnostics.event_count);
      EXPECT_EQ(receipt.diagnostics().active_count,
                first_diagnostics.active_count);
    }
    EXPECT_EQ(f.force.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(f.force.allocations().device_allocations,
              allocation.device_allocations);
    f.Discard();
  }
}

TEST(SelfContactForceCuda,
     DuplicateStaleForeignFinalEventNaNAndNodeSumOverflowRollbackRetry) {
  Fixture f;
  ASSERT_TRUE(f.Initialize({}, 1e308));
  auto events = f.Events();
  ASSERT_GE(events.size(), 2u);
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(f.rig.Begin(token, assembly));
  c::SelfContactForceAssemblyReceipt receipt;
  auto duplicate = events;
  duplicate[1] = duplicate[0];
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, token, assembly,
      f.Activity(),
      {duplicate.data(), duplicate.size()}, &receipt).status,
      c::SelfContactForceStatus::DuplicateEvent);
  EXPECT_FALSE(receipt.prepared());
  auto foreign = events;
  foreign.back().classification.binding_identity = nullptr;
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, token, assembly,
      f.Activity(),
      {foreign.data(), foreign.size()}, &receipt).status,
      c::SelfContactForceStatus::IdentityMismatch);
  auto same_body = events;
  same_body.back().classification.status =
      c::SelfContactPairStatus::ExcludedSameRigidGroup;
  same_body.back().classification.excluded = true;
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, token, assembly,
      f.Activity(),
      {same_body.data(), same_body.size()}, &receipt).status,
      c::SelfContactForceStatus::IdentityMismatch);
  auto secondary = events;
  secondary.back().classification.status =
      c::SelfContactPairStatus::UnsupportedCinSecondary;
  secondary.back().classification.endpoint_support[1].status =
      c::SelfContactSupportStatus::UnsupportedCinSecondary;
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, token, assembly,
      f.Activity(),
      {secondary.data(), secondary.size()}, &receipt).status,
      c::SelfContactForceStatus::IdentityMismatch);
  auto tied = events;
  tied.back().classification.tied =
      c::SelfContactTiedStatus::CompleteLocalSupportNeedsRuntimeActivity;
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, token, assembly, f.Activity(),
      {tied.data(), tied.size()}, &receipt).status,
      c::SelfContactForceStatus::IdentityMismatch);
  auto area = events;
  area.back().classification.admitted_force_area_m2.value =
      std::nextafter(
          area.back().classification.admitted_force_area_m2.value,
          HUGE_VAL);
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, token, assembly, f.Activity(),
      {area.data(), area.size()}, &receipt).status,
      c::SelfContactForceStatus::IdentityMismatch);
  auto endpoint = events;
  endpoint.back().endpoints[0] = endpoint.back().endpoints[1];
  EXPECT_NE(f.force.AssembleAccepted(
      f.rig.owner, token, assembly, f.Activity(),
      {endpoint.data(), endpoint.size()}, &receipt).status,
      c::SelfContactForceStatus::Ok);
  auto foreign_activity = f.activity;
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, token, assembly,
      {foreign_activity.data(), foreign_activity.data(),
       foreign_activity.size()},
      {events.data(), events.size()}, &receipt).status,
      c::SelfContactForceStatus::IdentityMismatch);
  const auto parent = events.back().classification.parent[0];
  f.activity[parent] = 0;
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, token, assembly,
      f.Activity(),
      {events.data(), events.size()}, &receipt).status,
      c::SelfContactForceStatus::IdentityMismatch);
  f.activity[parent] = 1;

  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(p::Good(f.rig.owner.BorrowCinAssembly(token, &cin)));
  AssemblySnapshot before(f.rig.fixture.domain.node_count()), after(before.nodes);
  std::array<c::SelfContactForceEvent, 1> final_event{{events.back()}};
  const auto corrupt_node = final_event[0].endpoints[1].nodes[0];
  double original_position = 0;
  ASSERT_EQ(cudaMemcpyAsync(
      &original_position,
      assembly.accepted.position_xyz + 3 * corrupt_node,
      sizeof(original_position), cudaMemcpyDeviceToHost,
      assembly.stream), cudaSuccess);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpyAsync(
      const_cast<double*>(assembly.accepted.position_xyz) +
          3 * corrupt_node,
      &nan, sizeof(nan), cudaMemcpyHostToDevice,
      assembly.stream), cudaSuccess);
  ASSERT_TRUE(before.Read(assembly, cin));
  const auto failed_event = f.force.AssembleAccepted(
      f.rig.owner, token, assembly,
      f.Activity(),
      {final_event.data(), final_event.size()}, &receipt);
  EXPECT_EQ(failed_event.status, c::SelfContactForceStatus::EventFailure);
  EXPECT_EQ(failed_event.event, 0u);
  EXPECT_FALSE(receipt.prepared());
  ASSERT_TRUE(after.Read(assembly, cin));
  EXPECT_EQ(before.values, after.values);
  ASSERT_EQ(cudaMemcpyAsync(
      const_cast<double*>(assembly.accepted.position_xyz) +
          3 * corrupt_node,
      &original_position, sizeof(original_position),
      cudaMemcpyHostToDevice, assembly.stream), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(assembly.stream), cudaSuccess);
  f.Discard();

  ASSERT_TRUE(f.rig.Begin(token, assembly));
  ASSERT_TRUE(p::Good(f.rig.owner.BorrowCinAssembly(token, &cin)));
  ASSERT_TRUE(before.Read(assembly, cin));
  ASSERT_TRUE(Good(f.force.AssembleAccepted(
      f.rig.owner, token, assembly,
      f.Activity(),
      {events.data(), events.size()}, &receipt)));
  ASSERT_TRUE(after.Read(assembly, cin));
  struct OverflowSeed {
    unsigned channel;
    std::size_t node;
    double value;
  };
  std::vector<OverflowSeed> overflow_seeds;
  for (unsigned channel = 0; channel < 3; ++channel)
    for (std::size_t node = 0; node < before.nodes; ++node) {
      const double increment =
          after.values[channel * before.nodes + node] -
          before.values[channel * before.nodes + node];
      if (increment != 0)
        overflow_seeds.push_back(
            {channel, node,
             std::copysign(std::numeric_limits<double>::max(),
                           increment)});
    }
  ASSERT_FALSE(overflow_seeds.empty());
  const auto prior_attempt = receipt.diagnostics().attempt;
  f.Discard();

  ASSERT_TRUE(f.rig.Begin(token, assembly));
  ASSERT_TRUE(p::Good(f.rig.owner.BorrowCinAssembly(token, &cin)));
  double* force_channels[]{
      assembly.forces.force_x, assembly.forces.force_y,
      assembly.forces.force_z};
  for (const auto& seed : overflow_seeds)
    ASSERT_EQ(cudaMemcpyAsync(
        force_channels[seed.channel] + seed.node, &seed.value,
        sizeof(seed.value), cudaMemcpyHostToDevice,
        assembly.stream), cudaSuccess);
  ASSERT_TRUE(before.Read(assembly, cin));
  EXPECT_EQ(f.force.AssembleAccepted(
      f.rig.owner, token, assembly,
      f.Activity(),
      {events.data(), events.size()}, &receipt).status,
      c::SelfContactForceStatus::AssemblyFailure);
  ASSERT_TRUE(after.Read(assembly, cin));
  EXPECT_EQ(before.values, after.values);
  EXPECT_EQ(receipt.diagnostics().attempt, prior_attempt);
  f.Discard();

  ASSERT_TRUE(f.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(f.force.AssembleAccepted(
      f.rig.owner, token, assembly,
      f.Activity(),
      {events.data(), events.size()}, &receipt)));
  EXPECT_TRUE(receipt.prepared());
  f.Discard();
}

TEST(SelfContactForceCuda,
     PartPlainAndCinMasterSupportsRemainOwnerTransferredNotCallerBits) {
  Fixture f(true);
  ASSERT_TRUE(f.Initialize());
  const auto events = f.Events(32);
  ASSERT_FALSE(events.empty());
  bool cin_master = false, mixed_rigid = false;
  for (const auto& event : events) {
    for (const auto& support : event.classification.endpoint_support) {
      cin_master = cin_master ||
          support.status == c::SelfContactSupportStatus::AdmittedCinMaster;
      mixed_rigid = mixed_rigid ||
          support.status ==
              c::SelfContactSupportStatus::AdmittedPartialOrMixedRigid;
    }
  }
  EXPECT_TRUE(cin_master);
  EXPECT_TRUE(mixed_rigid);
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(f.rig.Begin(token, assembly));
  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(p::Good(f.rig.owner.BorrowCinAssembly(token, &cin)));
  const auto master = f.rig.fixture.domain.Find(10);
  double before_sti = 0, after_sti = 0;
  ASSERT_EQ(cudaMemcpyAsync(
      &before_sti, cin.translational_stiffness + master,
      sizeof(before_sti), cudaMemcpyDeviceToHost,
      assembly.stream), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(assembly.stream), cudaSuccess);
  c::SelfContactForceAssemblyReceipt receipt;
  ASSERT_TRUE(Good(f.force.AssembleAccepted(
      f.rig.owner, token, assembly,
      f.Activity(),
      {events.data(), events.size()}, &receipt)));
  ASSERT_EQ(cudaMemcpyAsync(
      &after_sti, cin.translational_stiffness + master,
      sizeof(after_sti), cudaMemcpyDeviceToHost,
      assembly.stream), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(assembly.stream), cudaSuccess);
  EXPECT_GT(after_sti, before_sti);
  // The existing actual owner operation performs native CIN transfer after
  // this contact contribution; no secondary endpoint was admitted above.
  fe::NodalPreparedView prepared;
  ASSERT_TRUE(f.rig.Advance(token, assembly, prepared));
  EXPECT_EQ(prepared.attempt, receipt.diagnostics().attempt);
  f.Discard();
}

TEST(SelfContactForceCuda,
     ActualCinMasterGetsDenseForceMomentAndStiWhileSecondaryGetsNone) {
  Fixture f(false,p::ContactConstraintLayout::SurfaceCinSecondary);
  ASSERT_TRUE(f.Initialize());
  const auto events=f.Events(32);
  auto selected=events.end();
  unsigned master_endpoint=2;
  for (auto event=events.begin();event!=events.end();++event)
    for (unsigned endpoint=0;endpoint<2;++endpoint)
      if (event->classification.endpoint_support[endpoint].status ==
          c::SelfContactSupportStatus::AdmittedCinMaster) {
        selected=event;
        master_endpoint=endpoint;
        break;
      }
  ASSERT_NE(selected,events.end());
  ASSERT_LT(master_endpoint,2u);
  const auto secondary=f.rig.fixture.domain.Find(14);
  ASSERT_NE(secondary,SIZE_MAX);
  for (const auto& event:events)
    for (const auto& point:event.endpoints)
      for (unsigned slot=0;slot<point.count;++slot)
        EXPECT_TRUE(point.nodes[slot]!=secondary || point.weights[slot]==0);

  std::size_t secondary_vertex=SIZE_MAX,master_facet=SIZE_MAX;
  for (std::size_t vertex=0;vertex<f.uses.vertex_uses().size();++vertex)
    if (f.uses.vertex_uses()[vertex].key.kind ==
            c::FacetVertexKind::SourceVertex &&
        f.uses.vertex_uses()[vertex].key.first==14) {
      secondary_vertex=vertex;
      break;
    }
  for (std::size_t facet=0;facet<f.uses.facet_uses().size();++facet)
    if (f.uses.parents()[f.uses.facet_uses()[facet].parent].
            source.source_parent_id==103) {
      master_facet=facet;
      break;
    }
  ASSERT_NE(secondary_vertex,SIZE_MAX);
  ASSERT_NE(master_facet,SIZE_MAX);
  auto master_point=f.FacePoint(master_facet);
  c::SelfContactPairClassification rejected;
  ASSERT_EQ(f.uses.ClassifyVertexFace(secondary_vertex,master_facet,
      master_point,f.Activity(),&rejected).status,
      c::SelfContactActiveUseStatus::Ok);
  EXPECT_EQ(rejected.endpoint_support[0].status,
      c::SelfContactSupportStatus::UnsupportedCinSecondary);
  EXPECT_EQ(rejected.status,c::SelfContactPairStatus::UnsupportedCinSecondary);
  EXPECT_EQ(rejected.admitted_force_area_m2.value,0);

  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(f.rig.Begin(token,assembly));
  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(p::Good(f.rig.owner.BorrowCinAssembly(token,&cin)));
  AssemblySnapshot before(f.rig.fixture.domain.node_count()),after(before.nodes);
  ASSERT_TRUE(before.Read(assembly,cin));
  std::vector<double> kinematics(6*before.nodes);
  ASSERT_EQ(cudaMemcpyAsync(kinematics.data(),assembly.accepted.position_xyz,
      3*before.nodes*sizeof(double),cudaMemcpyDeviceToHost,assembly.stream),
      cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(kinematics.data()+3*before.nodes,
      assembly.accepted.velocity_xyz,3*before.nodes*sizeof(double),
      cudaMemcpyDeviceToHost,assembly.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);

  const auto weighted=[&](const c::WeightedSurfacePoint& point) {
    c::Vec3 value;
    for (unsigned slot=0;slot<point.count;++slot) {
      const auto node=point.nodes[slot];
      value.x+=point.weights[slot]*kinematics[3*node];
      value.y+=point.weights[slot]*kinematics[3*node+1];
      value.z+=point.weights[slot]*kinematics[3*node+2];
    }
    return value;
  };
  const auto a=weighted(selected->endpoints[0]);
  const auto b=weighted(selected->endpoints[1]);
  const c::Vec3 separation{a.x-b.x,a.y-b.y,a.z-b.z};
  const double distance=std::hypot(
      std::hypot(separation.x,separation.y),separation.z);
  ASSERT_GT(distance,0);
  const c::Vec3 normal{
      separation.x/distance,separation.y/distance,separation.z/distance};
  const double gap=(distance-
      selected->classification.reference_half_thickness_m[0])-
      selected->classification.reference_half_thickness_m[1];
  ASSERT_LT(gap,0);
  const double stiffness=std::nextafter(
      f.config.stiffness_per_area_n_m3*
          selected->classification.admitted_force_area_m2.value,HUGE_VAL);
  const double magnitude=stiffness*(-gap);
  const c::Vec3 endpoint_force[]{
      {magnitude*normal.x,magnitude*normal.y,magnitude*normal.z},
      {-magnitude*normal.x,-magnitude*normal.y,-magnitude*normal.z}};
  std::vector<c::Vec3> expected_force(before.nodes);
  std::vector<long double> jacobian_norm(before.nodes);
  long double norm_sum=0;
  for (unsigned endpoint=0;endpoint<2;++endpoint) {
    const auto& point=selected->endpoints[endpoint];
    for (unsigned slot=0;slot<point.count;++slot) {
      const auto node=point.nodes[slot];
      const double weight=point.weights[slot];
      expected_force[node]=c::Add(expected_force[node],
          c::Scale(endpoint_force[endpoint],weight));
      const long double x=weight*normal.x;
      const long double y=weight*normal.y;
      const long double z=weight*normal.z;
      jacobian_norm[node]=std::sqrt(x*x+y*y+z*z);
      norm_sum+=jacobian_norm[node];
    }
  }

  c::SelfContactForceAssemblyReceipt receipt;
  ASSERT_TRUE(Good(f.force.AssembleAccepted(
      f.rig.owner,token,assembly,f.Activity(),{&*selected,1},&receipt)));
  ASSERT_EQ(receipt.diagnostics().attempt,assembly.attempt);
  ASSERT_TRUE(after.Read(assembly,cin));
  c::Vec3 actual_master_resultant,expected_master_resultant;
  c::Vec3 actual_master_moment,expected_master_moment;
  long double actual_master_sti=0,expected_master_sti=0;
  for (std::size_t node=0;node<before.nodes;++node) {
    const auto increment=c::Subtract(after.Vector(0,node),
                                     before.Vector(0,node));
    const double force_scale=1+receipt.diagnostics().maximum_force_norm_n;
    EXPECT_NEAR(increment.x,expected_force[node].x,2e-12*force_scale);
    EXPECT_NEAR(increment.y,expected_force[node].y,2e-12*force_scale);
    EXPECT_NEAR(increment.z,expected_force[node].z,2e-12*force_scale);
    const long double exact_sti=
        static_cast<long double>(stiffness)*jacobian_norm[node]*norm_sum;
    const double sti_increment=
        after.values[6*before.nodes+node]-
        before.values[6*before.nodes+node];
    const long double sti_roundoff=64*DBL_EPSILON*(
        std::fabs(after.values[6*before.nodes+node])+
        std::fabs(before.values[6*before.nodes+node])+
        std::fabs(static_cast<double>(exact_sti)))+DBL_TRUE_MIN;
    EXPECT_GE(static_cast<long double>(sti_increment)+sti_roundoff,
              exact_sti);
    EXPECT_LE(static_cast<long double>(sti_increment),
              exact_sti+sti_roundoff);
    bool master=false;
    for (unsigned slot=0;
         slot<selected->endpoints[master_endpoint].count;++slot)
      master=master ||
          selected->endpoints[master_endpoint].nodes[slot]==node;
    if (master) {
      const c::Vec3 x{kinematics[3*node],kinematics[3*node+1],
                      kinematics[3*node+2]};
      actual_master_resultant=c::Add(actual_master_resultant,increment);
      expected_master_resultant=c::Add(
          expected_master_resultant,expected_force[node]);
      const auto actual_moment=c::Vec3{
          x.y*increment.z-x.z*increment.y,
          x.z*increment.x-x.x*increment.z,
          x.x*increment.y-x.y*increment.x};
      const auto expected_moment=c::Vec3{
          x.y*expected_force[node].z-x.z*expected_force[node].y,
          x.z*expected_force[node].x-x.x*expected_force[node].z,
          x.x*expected_force[node].y-x.y*expected_force[node].x};
      actual_master_moment=c::Add(actual_master_moment,actual_moment);
      expected_master_moment=c::Add(expected_master_moment,expected_moment);
      actual_master_sti+=sti_increment;
      expected_master_sti+=exact_sti;
    }
  }
  const double scale=1+receipt.diagnostics().maximum_force_norm_n;
  EXPECT_NEAR(actual_master_resultant.x,expected_master_resultant.x,
              2e-12*scale);
  EXPECT_NEAR(actual_master_resultant.y,expected_master_resultant.y,
              2e-12*scale);
  EXPECT_NEAR(actual_master_resultant.z,expected_master_resultant.z,
              2e-12*scale);
  EXPECT_NEAR(actual_master_moment.x,expected_master_moment.x,2e-12*scale);
  EXPECT_NEAR(actual_master_moment.y,expected_master_moment.y,2e-12*scale);
  EXPECT_NEAR(actual_master_moment.z,expected_master_moment.z,2e-12*scale);
  const long double master_sti_roundoff=
      128*DBL_EPSILON*(std::fabs(actual_master_sti)+
                       std::fabs(expected_master_sti))+DBL_TRUE_MIN;
  EXPECT_GE(actual_master_sti+master_sti_roundoff,expected_master_sti);
  EXPECT_LE(actual_master_sti,expected_master_sti+master_sti_roundoff);
  EXPECT_EQ(after.Vector(0,secondary).x,before.Vector(0,secondary).x);
  EXPECT_EQ(after.Vector(0,secondary).y,before.Vector(0,secondary).y);
  EXPECT_EQ(after.Vector(0,secondary).z,before.Vector(0,secondary).z);
  EXPECT_EQ(after.values[6*before.nodes+secondary],
            before.values[6*before.nodes+secondary]);
  ASSERT_TRUE(f.Commit(token,assembly));
  EXPECT_EQ(f.rig.owner.accepted().epoch,1u);
}

TEST(SelfContactForceCuda,
     ActualOwnerPartialAndFullyFixedMasksKeepFullReactionChannels) {
  for (const std::uint8_t mask : {std::uint8_t{1}, std::uint8_t{7}}) {
    Fixture f;
    auto events = f.Events(32);
    const auto fixed_node = f.rig.fixture.domain.Find(14);
    events.erase(std::remove_if(events.begin(), events.end(),
        [&](const auto& event) {
          for (const auto& point : event.endpoints)
            for (unsigned slot = 0; slot < point.count; ++slot)
              if (point.nodes[slot] == fixed_node) return false;
          return true;
        }), events.end());
    ASSERT_FALSE(events.empty());
    f.rig.fixture.fixed[fixed_node] = mask;
    if (mask == 7) f.rig.fixture.im[fixed_node] = 0;
    fe::FENodalState owner;
    const auto cin_startup = f.rig.fixture.CinStartup();
    ASSERT_TRUE(p::Good(owner.Initialize(
        f.rig.fixture.OwnerConfig(),
        {f.rig.fixture.x.data(), f.rig.fixture.v.data(),
         f.rig.fixture.w.data(), f.rig.fixture.domain.node_count(),
         f.rig.fixture.q.data()},
        f.rig.fixture.im.data(),
        {f.rig.fixture.fixed.data(),
         f.rig.fixture.rotation_fixed.data(),
         f.rig.fixture.ij.data(), f.rig.fixture.present.data()},
        f.rig.fixture.rigid, &cin_startup)));
    c::SelfContactForceAssembly force;
    c::SelfContactForceConfig config;
    config.owner = owner.accepted();
    config.stiffness_per_area_n_m3 = 2e9;
    config.event_capacity = 64;
    config.configuration_id = p::Configuration;
    config.qualification_id = p::Qualification;
    ASSERT_TRUE(Good(force.Initialize(config, f.uses, owner)));
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(p::Good(owner.BeginTrial(&token, &assembly)));
    std::uint8_t actual_mask = 0;
    ASSERT_EQ(cudaMemcpyAsync(
        &actual_mask, assembly.translation_fixed_bits + fixed_node,
        sizeof(actual_mask), cudaMemcpyDeviceToHost,
        assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream), cudaSuccess);
    ASSERT_EQ(actual_mask, mask);
    c::SelfContactForceAssemblyReceipt receipt;
    ASSERT_TRUE(Good(force.AssembleAccepted(
        owner, token, assembly,
        f.Activity(),
        {events.data(), events.size()}, &receipt)));
    EXPECT_TRUE(receipt.diagnostics().valid);
    // Force XYZ remain full reaction channels. Only the represented STI
    // majorant projects the fixed world components inside the pair primitive.
    double assembled_force[3]{};
    double* channels[]{
        assembly.forces.force_x,assembly.forces.force_y,
        assembly.forces.force_z};
    for (unsigned axis=0;axis<3;++axis)
      ASSERT_EQ(cudaMemcpyAsync(assembled_force+axis,
          channels[axis]+fixed_node,sizeof(double),cudaMemcpyDeviceToHost,
          assembly.stream),cudaSuccess);
    fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(p::Good(owner.BorrowCinAssembly(token,&cin)));
    ASSERT_EQ(cudaMemsetAsync(cin.witness_activity,1,cin.witness_count,
                             assembly.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
    ASSERT_TRUE(p::Good(owner.SealAssembly(token)));
    ASSERT_TRUE(p::Good(fe::AdvanceStaggeredCin(owner,token,
        {assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,
         p::Qualification,p::H,.2,true})));
    fe::NodalPreparedView prepared;
    ASSERT_TRUE(p::Good(owner.BorrowPrepared(token,&prepared)));
    ASSERT_EQ(prepared.attempt,receipt.diagnostics().attempt);
    ASSERT_TRUE(p::Good(fe::CompleteNodalValidation(owner,token,
        {prepared.owner_id,prepared.kinematics.base_epoch,prepared.attempt,
         p::Qualification,true})));
    ASSERT_TRUE(p::Good(owner.Commit(token)));
    std::vector<double> accepted_x(3*f.rig.fixture.domain.node_count());
    std::vector<double> accepted_v(accepted_x.size());
    std::vector<double> reactions(accepted_x.size());
    fe::NodalStamp stamp;
    ASSERT_TRUE(p::Good(owner.CopyAccepted(
        {accepted_x.data(),accepted_v.data(),
         f.rig.fixture.domain.node_count(),nullptr,nullptr,reactions.data(),
         nullptr},&stamp)));
    for (unsigned axis=0;axis<3;++axis) {
      const double expected=(mask&(1u<<axis)) ? -assembled_force[axis] : 0;
      EXPECT_NEAR(reactions[3*fixed_node+axis],expected,
                  2e-12*(1+std::abs(expected)));
    }
    EXPECT_EQ(stamp.reaction_kick_dt,.5*p::H);
  }
}

}  // namespace self_contact_force_cuda_test
