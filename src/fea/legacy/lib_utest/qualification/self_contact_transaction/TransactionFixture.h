// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
// Shared native test fixture. Owner startup, mechanics, contact and commit all
// remain the existing qualified operations; callers supply only test loads.
#include "../physical_publication/OwnerFixture.h"
#include "lib_src/collision/SelfContactTransaction.h"
#include "lib_src/collision/FixedContactFacetValues.h"
#include "lib_src/collision/SurfaceContactGeometry.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <vector>

namespace self_contact_transaction_cuda_test {
namespace c = tlfea::contact;
namespace fe = tl::fea;
namespace sct = tlfea::contact::self_contact_transaction;
namespace p = physical_publication_test;

inline bool Good(c::SelfContactTransactionReport report) {
  EXPECT_EQ(report.status, c::SelfContactTransactionStatus::Ok)
      << report.message << " candidate=" << report.candidate
      << " pair=" << report.pair
      << " discovery_task=" << report.discovery_task
      << " discovery_reason="
      << static_cast<unsigned>(report.discovery_reason)
      << " crossing_reason="
      << static_cast<unsigned>(report.crossing_reason);
  return report.status == c::SelfContactTransactionStatus::Ok;
}

inline void CheckInteriorEeGeometry(const p::Rig& rig) {
  const auto& geometry = rig.fixture.source.shell_input;
  const auto point = [](const tl::math::Vec3& value) {
    return c::Vec3{value.x, value.y, value.z};
  };
  c::SegmentGeometry segments[2];
  segments[0].vertices[0] = point(geometry.t.reference.position[0]);
  segments[0].vertices[1] = point(geometry.t.reference.position[1]);
  segments[1].vertices[0] = point(geometry.q[0].reference.position[3]);
  segments[1].vertices[1] = point(geometry.q[0].reference.position[0]);
  c::SegmentPairGeometry closest;
  ASSERT_EQ(c::ClosestPointsBetweenSegments(
      segments[0], segments[1], &closest), c::Status::kOk);
  EXPECT_GT(closest.parameter_a, 0);
  EXPECT_LT(closest.parameter_a, 1);
  EXPECT_GT(closest.parameter_b, 0);
  EXPECT_LT(closest.parameter_b, 1);
  EXPECT_NEAR(closest.distance, .00025, 1e-15);
}

struct Fixture {
  explicit Fixture(bool single = false, bool crossing = false,
                   double t3_failure = 2.5,
                   p::ContactConstraintLayout constraints =
                       p::ContactConstraintLayout::Legacy,
                   unsigned facet_level = 0,
                   unsigned input_permutation = 0,
                   double adjacent_apex_x = .05,
                   fe::ShellBatchStartup declared_startup = {},
                   bool separate_adjacent_contact = false)
      : rig(false, t3_failure, !single, constraints,
            !single && constraints == p::ContactConstraintLayout::Legacy,
            adjacent_apex_x, declared_startup, separate_adjacent_contact),
        single_parent(single), pass_through(crossing),
        t3_failure(t3_failure), facet_level(facet_level),
        input_permutation(input_permutation) {
    if (!single &&
        constraints == p::ContactConstraintLayout::Legacy)
      CheckInteriorEeGeometry(rig);
    if (pass_through) {
      rig.external_force_source_node = 14;
      const auto apex = rig.fixture.domain.Find(14);
      const double mass =
          rig.fixture.ledger.nodes()[apex].coefficients.mass;
      rig.external_force_z_n =
          -2 * mass * .00075 / (p::H * p::H);
    }
  }
  p::Rig rig;
  fe::ShellBatchPlasticityBinding execution_catalog;
  fe::ShellBatchFailureBinding execution_failure;
  fe::ShellExecutionBinding execution;
  fe::ShellPhysicalBinding physical;
  c::SelfContactSurfaceBinding surface;
  c::FixedContactFacetBinding facets;
  c::SelfContactActiveUseBinding uses;
  c::SelfContactTransaction transaction;
  std::vector<c::SelfContactParentSelection> selection;
  c::SelfContactTransactionConfig config;
  cudaStream_t owner_stream = nullptr;
  bool single_parent = false;
  bool pass_through = false;
  double t3_failure = 2.5;
  unsigned facet_level = 0;
  unsigned input_permutation = 0;
  bool authority_prepared = false;

  bool PrepareExecutionAuthority() {
    qbat_catalog_test::Fixture declaration;
    std::array<fe::ShellPlasticityMaterialInput,4> materials;
    std::array<fe::ShellPlasticitySectionInput,4> sections;
    std::copy(declaration.materials.begin(), declaration.materials.end(),
              materials.begin());
    std::copy(declaration.sections.begin(), declaration.sections.end(),
              sections.begin());
    for (unsigned row = 0; row < 2; ++row) {
      materials[row].curve_id = 0;
      materials[row].hardening =
          tl::material::ShellPlasticityHardeningKind::LinearLaw44;
      materials[row].linear = {10e6, 0};
      materials[row].rate = materials[2].rate;
    }
    const bool rigid_contact =
        rig.fixture.contact_constraints ==
            p::ContactConstraintLayout::SameMergedParts ||
        rig.fixture.contact_constraints ==
            p::ContactConstraintLayout::MergedPartAndPlain;
    const bool same_merged_parts =
        rig.fixture.contact_constraints ==
            p::ContactConstraintLayout::SameMergedParts;
    if (rigid_contact) {
      materials[3] = materials[2];
      materials[3].material_id = 3000;
      materials[3].curve_id = 0;
      materials[3].hardening =
          tl::material::ShellPlasticityHardeningKind::Tabulated;
      materials[3].rate = {};
      materials[3].linear = {};
      materials[3].law = fe::ShellSectionLaw::RigidSkin;
      sections[3] = sections[2];
      sections[3].section_id = 3000;
      sections[3].through_thickness_points = 0;
      sections[3].formulation =
          fe::ShellSectionFormulation::Nonconstitutive;
    }
    std::array<fe::ShellPlasticityParentInput,5> parents{{
        {fe::ShellBindingFamily::Qeph, 0, 100, 1000, 1000, 1000},
        {fe::ShellBindingFamily::Qeph, 1, 101, 1001, 1001, 1001},
        {fe::ShellBindingFamily::T3, 0, 102, 2000524, 2000524, 2000524},
        {fe::ShellBindingFamily::Qbat, 0, 103, 2000524, 2000524, 2000524},
        {fe::ShellBindingFamily::T3, 1, 104, 2000524, 2000524, 2000524}}};
    if (rigid_contact) {
      parents[2].source_part_id = 3000;
      parents[2].material_id = 3000;
      parents[2].section_id = 3000;
      if (same_merged_parts) {
        parents[4].source_part_id = 3001;
        parents[4].material_id = 3000;
        parents[4].section_id = 3000;
      }
    }
    const std::size_t parent_count=rigid_contact ? parents.size() : 4;
    const auto catalog_report =
        execution_catalog.InitializeExecutionCatalog(
            rig.fixture.source.shells,
            {nullptr, materials.data(), sections.data(), parents.data(),
             0, rigid_contact ? 4u : 3u,
             rigid_contact ? 4u : 3u, parent_count});
    EXPECT_EQ(catalog_report.status,
              fe::ShellPlasticityBindingStatus::Success)
        << catalog_report.message << " entry=" << catalog_report.entry
        << " family=" << static_cast<int>(catalog_report.family);
    if (catalog_report.status !=
        fe::ShellPlasticityBindingStatus::Success)
      return false;

    fe::ShellFailureParentInput failures[5];
    for (std::size_t row=0;row<parent_count;++row)
      failures[row] = qbat_catalog_test::Failure(parents[row]);
    if (rigid_contact) {
      failures[2] = {};
      failures[2].source = parents[2];
      if (same_merged_parts) {
        failures[4] = {};
        failures[4].source = parents[4];
      }
      for (const unsigned row : {0u,1u}) {
        failures[row].policy = fe::ShellFailurePolicy::Tab1AnyPoint;
        failures[row].constant = {};
        failures[row].tab1.table = {{-1, 0, 1}, 1};
      }
    } else {
      failures[2].constant.failure_strain = t3_failure;
      for (unsigned row = 0; row < 2; ++row) {
        failures[row].policy = fe::ShellFailurePolicy::Tab1AnyPoint;
        failures[row].constant = {};
        failures[row].tab1.table = {{-1, 0, 1}, 1};
      }
    }
    const auto failure_report = execution_failure.InitializeExecution(
        execution_catalog, failures, parent_count);
    EXPECT_EQ(failure_report.status,
              fe::ShellPlasticityBindingStatus::Success)
        << failure_report.message;
    if (failure_report.status !=
        fe::ShellPlasticityBindingStatus::Success)
      return false;

    const auto execution_report = execution.Initialize(
        execution_catalog, rig.fixture.ledger, rig.fixture.rigid);
    EXPECT_EQ(execution_report.status,
              fe::ShellPlasticityBindingStatus::Success)
        << execution_report.message << " entry=" << execution_report.entry;
    if (execution_report.status !=
        fe::ShellPlasticityBindingStatus::Success)
      return false;
    const auto physical_report = physical.InitializeExecution(
        {&rig.fixture.source.shells, &execution_catalog,
         &execution_failure, nullptr},
        rig.fixture.ledger, execution);
    if (!p::Good(physical_report)) return false;

    for (std::size_t row = 0;
         row < execution_catalog.parent_count(); ++row) {
      const auto& parent = *execution_catalog.parent(row);
      if (rigid_contact) {
        if (parent.family != fe::ShellBindingFamily::T3)
          continue;
      } else if (parent.family != fe::ShellBindingFamily::T3 &&
                 parent.family != fe::ShellBindingFamily::Qbat) {
        continue;
      }
      if (single_parent && !selection.empty()) continue;
      selection.push_back({
          row, parent.family, parent.family_index,
          parent.source_parent_id, parent.source_part_id});
    }
    if (selection.size() > 1) {
      const auto shift = input_permutation % selection.size();
      std::rotate(selection.begin(), selection.begin() + shift,
                  selection.end());
      if (input_permutation & 1)
        std::reverse(selection.begin(), selection.end());
    }
    const auto surface_report = surface.Initialize(
        physical, {selection.data(), selection.size()});
    EXPECT_EQ(surface_report.status, c::SelfContactSurfaceStatus::Ok)
        << surface_report.message;
    if (surface_report.status != c::SelfContactSurfaceStatus::Ok)
      return false;
    const auto facet_report =
        facets.Initialize(surface, {{}, facet_level});
    EXPECT_EQ(facet_report.status, c::FixedContactFacetStatus::Ok)
        << facet_report.message;
    if (facet_report.status != c::FixedContactFacetStatus::Ok)
      return false;
    const auto cin = rig.fixture.WitnessSource();
    c::SelfContactActiveUseSource source;
    source.rigid = &rig.fixture.rigid;
    source.cin = {
        cin.model, cin.ranges, cin.witnesses,
        cin.range_count, cin.witness_count};
    const auto use_report = uses.Initialize(facets, source);
    EXPECT_EQ(use_report.status, c::SelfContactActiveUseStatus::Ok)
        << use_report.message << " parent=" << use_report.parent
        << " feature=" << use_report.feature;
    if (use_report.status != c::SelfContactActiveUseStatus::Ok)
      return false;
    authority_prepared = true;
    return true;
  }

  bool InitializeInfrastructure() {
    if (!authority_prepared && !PrepareExecutionAuthority()) return false;
    // Owner, q/t/qbat/PART/plain/CIN mapped participants and publication all
    // consume the same execution-authenticated physical authority retained by
    // surface/facets/uses.
    return rig.InitializeAgainst(physical);
  }

  bool Initialize(c::SelfContactTransactionLimits limits = {}) {
    if (!InitializeInfrastructure()) return false;
    if (rig.owner.BorrowOwnerStream(&owner_stream).status !=
        fe::NodalStatus::Ok)
      return false;
    config.force.owner = rig.owner.accepted();
    config.force.startup = rig.fixture.Identity().startup;
    config.force.stiffness_per_area_n_m3 = 2e9;
    config.force.event_capacity = 512;
    config.force.configuration_id = p::Configuration;
    config.force.qualification_id = p::Qualification;
    config.source_id = p::SelfContactSource;
    const auto plan = c::SelfContactTransaction::Forecast(
        config, uses, rig.fixture.Identity(), limits);
    if (!Good(plan.report)) return false;
    if (!Good(transaction.Initialize(
            config, uses, rig.owner, rig.publication,
            physical, rig.Participants(),
            rig.fixture.Identity(), owner_stream, limits)))
      return false;
    fe::ShellPhysicalScratchRoster roster{
        {}, transaction.roster_entry()};
    if (!p::Good(
            rig.publication.ConfigurePhysicalScratchParticipation(
                rig.owner, physical, rig.Participants(),
                rig.fixture.Identity(), roster,
                limits.participation)))
      return false;
    return true;
  }

  std::size_t Parent(std::uint64_t eid) const {
    for (std::size_t parent = 0;
         parent < uses.parents().size(); ++parent)
      if (uses.parents()[parent].source.source_parent_id == eid)
        return parent;
    return SIZE_MAX;
  }

  std::uint32_t ProbeNode() {
    const auto t3 = Parent(102);
    const auto qbat = Parent(103);
    if (t3 == SIZE_MAX || qbat == SIZE_MAX) return UINT32_MAX;
    std::size_t vertex_use = SIZE_MAX;
    for (std::size_t vertex = 0;
         vertex < uses.vertex_uses().size(); ++vertex) {
      const auto& use = uses.vertex_uses()[vertex];
      if (use.parent == t3 &&
          use.key.kind == c::FacetVertexKind::SourceVertex &&
          use.key.first == 14) {
        vertex_use = vertex;
        break;
      }
    }
    if (vertex_use == SIZE_MAX) return UINT32_MAX;
    const auto feature = uses.vertex_uses()[vertex_use].feature;
    for (std::size_t facet_index = 0;
         facet_index < uses.facet_uses().size(); ++facet_index) {
      const auto& use = uses.facet_uses()[facet_index];
      if (use.parent != qbat) continue;
      bool incident = false;
      for (const auto vertex : use.vertex_features)
        incident = incident || vertex == feature;
      if (incident) continue;
      return uses.vertex_uses()[vertex_use].point.nodes[0];
    }
    return UINT32_MAX;
  }

  bool DriveT3Removal() {
    const auto apex = rig.fixture.domain.Find(14);
    if (apex == SIZE_MAX) return false;
    const double mass =
        rig.fixture.ledger.nodes()[apex].coefficients.mass;
    if (!(mass > 0)) return false;
    rig.external_force_source_node = 14;
    rig.external_force_z_n =
        -2 * mass * .01 / (p::H * p::H);
    return true;
  }

  bool Prepare(const fe::NodalTrialToken& token,
               const fe::NodalAssemblyView& assembly,
               fe::NodalPreparedView& prepared,
               fe::ShellPhysicalDiagnostics& common) {
    fe::ShellPhysicalDiagnostics candidates;
    return rig.Advance(token, assembly, prepared) &&
        rig.Evaluate(token, prepared, candidates) &&
        p::Good(rig.publication.PreparePhysical(
            rig.owner, token,
            {&candidates.qeph, &candidates.t3, &candidates.qbat,
             &candidates.type25, &candidates.type13,
             &candidates.solids},
            &common));
  }

  bool Commit(const fe::NodalTrialToken& token,
              const fe::NodalPreparedView& prepared,
              const fe::ShellPhysicalDiagnostics& common,
              const c::SelfContactTransactionReceipt& receipt) {
    return p::Good(
               rig.publication.SealPhysicalScratchParticipation(
                   rig.owner, token, receipt.scratch_receipts())) &&
        p::Good(rig.publication.CommitPhysical(
            rig.owner, token, common,
            {prepared.owner_id,
             prepared.kinematics.base_epoch,
             prepared.attempt, p::Qualification, true}));
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
    transaction.DiscardTrial();
  }
};

} // namespace self_contact_transaction_cuda_test
