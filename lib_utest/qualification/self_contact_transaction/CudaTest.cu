// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../physical_publication/OwnerFixture.h"
#include "lib_src/collision/SelfContactTransaction.h"
#include "lib_src/collision/FixedContactFacetValues.h"
#include "lib_src/collision/SurfaceContactGeometry.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace self_contact_transaction_cuda_test {
namespace c = tlfea::contact;
namespace fe = tl::fea;
namespace p = physical_publication_test;

bool Good(c::SelfContactTransactionReport report) {
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

void CheckInteriorEeGeometry(const p::Rig& rig) {
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

struct AssemblyFields {
  std::size_t nodes=0;
  std::vector<double> values;
  explicit AssemblyFields(std::size_t count) : nodes(count),values(8*count) {}
  bool Read(const fe::NodalAssemblyView& view,
            const fe::NodalCinAssemblyView& cin) {
    double* source[]{
        view.forces.force_x,view.forces.force_y,view.forces.force_z,
        view.forces.couple_x,view.forces.couple_y,view.forces.couple_z,
        cin.translational_stiffness,cin.rotational_stiffness};
    for (unsigned channel=0;channel<8;++channel)
      if (cudaMemcpyAsync(values.data()+channel*nodes,source[channel],
              nodes*sizeof(double),cudaMemcpyDeviceToHost,view.stream) !=
          cudaSuccess)
        return false;
    return cudaStreamSynchronize(view.stream) == cudaSuccess;
  }
  c::Vec3 Vector(unsigned channel,std::size_t node) const {
    return {values[channel*nodes+node],
            values[(channel+1)*nodes+node],
            values[(channel+2)*nodes+node]};
  }
};

c::Vec3 Cross(c::Vec3 a,c::Vec3 b) {
  return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
double Norm(c::Vec3 a) {
  return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
}
void Near(c::Vec3 actual,c::Vec3 expected,double scale=1) {
  const double tolerance=2e-12*std::max({1.,scale,Norm(expected)});
  EXPECT_NEAR(actual.x,expected.x,tolerance);
  EXPECT_NEAR(actual.y,expected.y,tolerance);
  EXPECT_NEAR(actual.z,expected.z,tolerance);
}
c::Vec3 DenseAngularAcceleration(const fe::RigidBindingGroup& body,
                                 c::Vec3 moment) {
  c::Vec3 result;
  const auto& axes=body.principal.axes;
  const double inverse[]{
      1/body.principal.inertia.x,1/body.principal.inertia.y,
      1/body.principal.inertia.z};
  for (unsigned axis=0;axis<3;++axis) {
    const c::Vec3 column{
        axes.v[axis],axes.v[3+axis],axes.v[6+axis]};
    const double local=column.x*moment.x+column.y*moment.y+
        column.z*moment.z;
    result=c::Add(result,c::Scale(column,inverse[axis]*local));
  }
  return result;
}

struct Fixture {
  explicit Fixture(bool single = false, bool crossing = false,
                   double t3_failure = 2.5,
                   p::ContactConstraintLayout constraints =
                       p::ContactConstraintLayout::Legacy,
                   unsigned facet_level = 0)
      : rig(false, t3_failure, !single, constraints,
            !single && constraints == p::ContactConstraintLayout::Legacy),
        single_parent(single), pass_through(crossing),
        t3_failure(t3_failure), facet_level(facet_level) {
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

TEST(SelfContactTransactionCuda,
     ExactForecastCapMinusOneAndRosterEntryAreStable) {
  Fixture fixture;
  ASSERT_TRUE(fixture.InitializeInfrastructure());
  fixture.config.force.owner = fixture.rig.owner.accepted();
  fixture.config.force.stiffness_per_area_n_m3 = 2e9;
  fixture.config.force.event_capacity = 512;
  fixture.config.force.configuration_id = p::Configuration;
  fixture.config.force.qualification_id = p::Qualification;
  fixture.config.source_id = p::SelfContactSource;
  auto limits = c::SelfContactTransactionLimits{};
  const auto exact = c::SelfContactTransaction::Forecast(
      fixture.config, fixture.uses, fixture.rig.fixture.Identity(),
      limits);
  ASSERT_TRUE(Good(exact.report));
  EXPECT_EQ(exact.forecast.shared_backing_discount_bytes,
            exact.forecast.broadphase.retained_source_bytes +
                exact.forecast.force.retained_active_use_bytes);
  limits.max_host_bytes = exact.forecast.owned_host_bytes - 1;
  EXPECT_EQ(c::SelfContactTransaction::Forecast(
      fixture.config, fixture.uses, fixture.rig.fixture.Identity(),
      limits).report.status, c::SelfContactTransactionStatus::ResourceLimit);
  ++limits.max_host_bytes;
  limits.activity.max_host_bytes =
      exact.forecast.activity.owned_host_bytes - 1;
  EXPECT_EQ(c::SelfContactTransaction::Forecast(
      fixture.config, fixture.uses, fixture.rig.fixture.Identity(),
      limits).report.status, c::SelfContactTransactionStatus::ResourceLimit);
  ++limits.activity.max_host_bytes;
  limits.max_device_bytes = exact.forecast.device_bytes - 1;
  EXPECT_EQ(c::SelfContactTransaction::Forecast(
      fixture.config, fixture.uses, fixture.rig.fixture.Identity(),
      limits).report.status, c::SelfContactTransactionStatus::ResourceLimit);
  ++limits.max_device_bytes;
  limits.max_startup_host_bytes =
      exact.forecast.startup_host_bytes - 1;
  EXPECT_EQ(c::SelfContactTransaction::Forecast(
      fixture.config, fixture.uses, fixture.rig.fixture.Identity(),
      limits).report.status, c::SelfContactTransactionStatus::ResourceLimit);
  ++limits.max_startup_host_bytes;
  cudaStream_t owner_stream = nullptr;
  ASSERT_EQ(fixture.rig.owner.BorrowOwnerStream(&owner_stream).status,
            fe::NodalStatus::Ok);
  EXPECT_EQ(fixture.rig.owner.ValidateOwnerStream(nullptr).status,
            fe::NodalStatus::InvalidInput);
  EXPECT_EQ(fixture.rig.owner.ValidateOwnerStream(
                cudaStreamLegacy).status,
            fe::NodalStatus::InvalidInput);
  EXPECT_EQ(fixture.rig.owner.ValidateOwnerStream(owner_stream).status,
            fe::NodalStatus::Ok);
  ASSERT_TRUE(Good(fixture.transaction.Initialize(
      fixture.config, fixture.uses, fixture.rig.owner,
      fixture.rig.publication, fixture.physical,
      fixture.rig.Participants(), fixture.rig.fixture.Identity(),
      owner_stream, limits)));
  const auto entry = fixture.transaction.roster_entry();
  EXPECT_NE(entry.issuer, nullptr);
  EXPECT_EQ(entry.source_id, p::SelfContactSource);
  EXPECT_EQ(fixture.transaction.allocations().device.device_allocations, 2u);
  EXPECT_EQ(fixture.transaction.allocations().device.device_bytes,
            exact.forecast.device_bytes);
  EXPECT_EQ(fixture.transaction.allocations().activity.host_bytes,
            exact.forecast.activity.arena_bytes);
}

TEST(SelfContactTransactionCuda,
     SingleParentZeroPairZeroEventStillParticipatesAndCommits) {
  Fixture fixture(true);
  ASSERT_TRUE(fixture.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  EXPECT_EQ(accepted.broadphase_pairs(), 0u);
  EXPECT_EQ(accepted.facet_pairs(), 0u);
  EXPECT_EQ(accepted.diagnostics().event_count, 0u);

  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  c::SelfContactTransactionReceipt receipt;
  ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
      fixture.rig.owner, token, common, prepared, accepted, &receipt)));
  EXPECT_EQ(receipt.broadphase_pairs(), 0u);
  EXPECT_EQ(receipt.facet_pairs(), 0u);
  EXPECT_EQ(receipt.policy_outcomes(), 0u);
  const auto policy = fixture.transaction.policy_outcomes();
  EXPECT_TRUE(policy.complete);
  EXPECT_EQ(policy.count, 0u);
  EXPECT_EQ(policy.data, nullptr);
  ASSERT_TRUE(fixture.Commit(token, prepared, common, receipt));
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 1u);
}

TEST(SelfContactTransactionCuda,
     AcceptedInteriorEeForceCandidateRetryAndRollbackKeepForceSti) {
  // Level one keeps the physical fixture small while producing one mixed
  // candidate stream: exact contact pairs and strict swept-box separations.
  Fixture fixture(false, false, 2.5,
                  p::ContactConstraintLayout::Legacy, 1);
  ASSERT_TRUE(fixture.Initialize());
  const auto node = fixture.ProbeNode();
  ASSERT_NE(node, UINT32_MAX);
  const auto allocation = fixture.transaction.allocations();
  {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactTransactionReceipt unchanged;
    EXPECT_NE(fixture.transaction.SealCandidate(
        fixture.rig.owner, token, {}, {}, {}, &unchanged).status,
        c::SelfContactTransactionStatus::Ok);
    EXPECT_FALSE(unchanged.valid());
    EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
  }
  {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    EXPECT_EQ(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly,
        reinterpret_cast<c::SelfContactAcceptedAssemblyReceipt*>(
            &assembly)).status,
        c::SelfContactTransactionStatus::InvalidInput);
    EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
  }
  for (unsigned interval = 0; interval < 2; ++interval) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(p::Good(fixture.rig.owner.BorrowCinAssembly(token, &cin)));
    double before_sti = 0, after_sti = 0;
    ASSERT_EQ(cudaMemcpyAsync(
        &before_sti, cin.translational_stiffness + node,
        sizeof(before_sti), cudaMemcpyDeviceToHost,
        assembly.stream), cudaSuccess);
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    EXPECT_GT(accepted.broadphase_pairs(), 0u);
    EXPECT_GT(accepted.facet_pairs(), 0u);
    EXPECT_GT(accepted.diagnostics().event_count, 0u);
    EXPECT_GT(accepted.diagnostics().vertex_face_event_count,0u);
    EXPECT_GT(
        accepted.diagnostics().boundary_vertex_edge_event_count,0u);
    EXPECT_GT(accepted.diagnostics().edge_edge_event_count,0u);
    EXPECT_EQ(accepted.diagnostics().vertex_face_event_count+
              accepted.diagnostics().edge_edge_event_count,
              accepted.diagnostics().event_count);
    EXPECT_GT(accepted.diagnostics().active_count, 0u);
    EXPECT_EQ(accepted.diagnostics().first_source_order, 0u);
    EXPECT_EQ(accepted.diagnostics().last_source_order,
              accepted.diagnostics().event_count - 1);
    EXPECT_GT(
        accepted.diagnostics().maximum_represented_stiffness_n_m, 0);
    ASSERT_EQ(cudaMemcpyAsync(
        &after_sti, cin.translational_stiffness + node,
        sizeof(after_sti), cudaMemcpyDeviceToHost,
        assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream), cudaSuccess);
    EXPECT_GT(accepted.diagnostics().maximum_force_norm_n, 0);
    EXPECT_GT(accepted.diagnostics().maximum_sti_diagonal_n_m, 0);
    EXPECT_GE(after_sti, before_sti);

    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    if (interval == 0) {
      EXPECT_EQ(fixture.rig.publication.CommitPhysical(
          fixture.rig.owner, token, common,
          {prepared.owner_id, prepared.kinematics.base_epoch,
           prepared.attempt, p::Qualification, true}).status,
          fe::ShellPublicationStatus::ParticipationFailure);
      EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);

      ASSERT_TRUE(fixture.rig.Begin(token, assembly));
      ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
          fixture.rig.owner, token, assembly, &accepted)));
      ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
      c::SelfContactTransactionReceipt unchanged;
      EXPECT_NE(fixture.transaction.SealCandidate(
          fixture.rig.owner, token, common, {}, accepted,
          &unchanged).status,
          c::SelfContactTransactionStatus::Ok);
      EXPECT_FALSE(unchanged.valid());
      EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);

      ASSERT_TRUE(fixture.rig.Begin(token, assembly));
      ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
          fixture.rig.owner, token, assembly, &accepted)));
      ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    }
    c::SelfContactTransactionReceipt receipt;
    ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
        fixture.rig.owner, token, common, prepared, accepted, &receipt)));
    ASSERT_TRUE(receipt.valid());
    const auto& policy = receipt.policy_summary();
    EXPECT_TRUE(policy.complete);
    EXPECT_GT(policy.motion_certified_linear_separated,0u);
    EXPECT_LE(policy.axis_certified_linear_separated,
              policy.motion_certified_linear_separated);
    EXPECT_LE(policy.edge_axis_certified_linear_separated,
              policy.axis_certified_linear_separated);
    EXPECT_LE(policy.vertex_edge_axis_separated,
              policy.axis_certified_linear_separated -
                  policy.edge_axis_certified_linear_separated);
    EXPECT_LE(policy.vertex_vertex_axis_separated,
              policy.axis_certified_linear_separated -
                  policy.edge_axis_certified_linear_separated -
                  policy.vertex_edge_axis_separated);
    EXPECT_GT(policy.exact_crossing_pairs,0u);
    EXPECT_GT(policy.exact_crossing_work,0u);
    EXPECT_LT(policy.exact_crossing_pairs,policy.outcomes);
    EXPECT_EQ(
        policy.outcomes,
        policy.motion_certified_linear_separated +
            policy.motion_excluded_same_rigid_group +
            policy.exact_crossing_pairs);
    ASSERT_TRUE(fixture.Commit(token, prepared, common, receipt));
    EXPECT_EQ(fixture.rig.owner.accepted().epoch,
              static_cast<std::uint64_t>(interval + 1));
    EXPECT_EQ(p::Bits(fixture.rig.owner.accepted().reaction_kick_dt),
              p::Bits(interval ? p::H : .5 * p::H));
    EXPECT_EQ(fixture.transaction.allocations().device.device_bytes,
              allocation.device.device_bytes);
    EXPECT_EQ(fixture.transaction.allocations().device.device_allocations,
              allocation.device.device_allocations);
    EXPECT_EQ(fixture.transaction.allocations().activity.host_bytes,
              allocation.activity.host_bytes);
  }
}

TEST(SelfContactTransactionCuda,
     ActualMergedRigidBodyExcludesDiscoveredVfBeforeForceOrSti) {
  Fixture fixture(false,false,2.5,
      p::ContactConstraintLayout::SameMergedParts);
  ASSERT_TRUE(fixture.Initialize());
  ASSERT_EQ(fixture.rig.fixture.rigid.groups().size(),1u);
  EXPECT_EQ(fixture.rig.fixture.rigid.groups()[0].source_kind,
            fe::RigidBindingSourceKind::Part);
  EXPECT_EQ(fixture.rig.fixture.topology.part_count(),2u);

  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(fixture.rig.Begin(token,assembly));
  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(p::Good(fixture.rig.owner.BorrowCinAssembly(token,&cin)));
  AssemblyFields before(fixture.rig.fixture.domain.node_count()),after(before.nodes);
  ASSERT_TRUE(before.Read(assembly,cin));
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner,token,assembly,&accepted)));
  EXPECT_GT(accepted.broadphase_pairs(),0u);
  EXPECT_GT(accepted.facet_pairs(),0u);
  EXPECT_EQ(accepted.discovered_features(),0u);
  EXPECT_EQ(accepted.diagnostics().event_count,0u);
  EXPECT_EQ(accepted.diagnostics().active_count,0u);
  EXPECT_EQ(accepted.diagnostics().maximum_force_norm_n,0);
  EXPECT_EQ(accepted.diagnostics().maximum_sti_diagonal_n_m,0);
  ASSERT_TRUE(after.Read(assembly,cin));
  EXPECT_EQ(after.values,before.values);

  fixture.Discard();
  EXPECT_EQ(fixture.rig.owner.accepted().epoch,0u);
}

TEST(SelfContactTransactionCuda,
     ActualMergedPartAndPlainBodiesUseMergedWrenchesBeforeInverseResponse) {
  Fixture fixture(false,false,2.5,
      p::ContactConstraintLayout::MergedPartAndPlain);
  ASSERT_TRUE(fixture.Initialize());
  const auto& binding=fixture.rig.fixture.rigid;
  ASSERT_EQ(binding.groups().size(),2u);
  ASSERT_EQ(fixture.rig.fixture.topology.part_count(),2u);
  EXPECT_EQ(binding.groups()[0].source_kind,fe::RigidBindingSourceKind::Part);
  EXPECT_EQ(binding.groups()[1].source_kind,
            fe::RigidBindingSourceKind::NodalGroup);
  EXPECT_EQ(binding.groups()[0].source_id,binding.groups()[1].source_id);
  ASSERT_NE(fixture.Parent(102),SIZE_MAX);
  ASSERT_NE(fixture.Parent(104),SIZE_MAX);

  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(fixture.rig.Begin(token,assembly));
  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(p::Good(fixture.rig.owner.BorrowCinAssembly(token,&cin)));
  AssemblyFields before(fixture.rig.fixture.domain.node_count()),after(before.nodes);
  ASSERT_TRUE(before.Read(assembly,cin));
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner,token,assembly,&accepted)));
  ASSERT_GT(accepted.diagnostics().event_count,0u);
  ASSERT_GT(accepted.diagnostics().active_count,0u);
  ASSERT_TRUE(after.Read(assembly,cin));

  std::vector<double> positions(3*after.nodes);
  ASSERT_EQ(cudaMemcpyAsync(positions.data(),assembly.accepted.position_xyz,
      positions.size()*sizeof(double),cudaMemcpyDeviceToHost,assembly.stream),
      cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
  std::array<c::Vec3,2> expected_acceleration{},expected_angular{};
  c::Vec3 contact_resultant,contact_moment;
  std::array<std::size_t,2> contacted_nodes{};
  std::array<c::Vec3,2> endpoint_inverse_sum{},merged_contact_acceleration{};
  for (std::size_t g=0;g<binding.groups().size();++g) {
    const auto& group=binding.groups()[g];
    c::Vec3 force,moment,contact_force;
    for (std::size_t slot=0;slot<group.member_count;++slot) {
      const auto& member=
          binding.members()[group.member_offset+slot];
      const auto node=member.domain_node;
      const c::Vec3 x{positions[3*node],positions[3*node+1],
                      positions[3*node+2]};
      const auto node_force=after.Vector(0,node);
      const auto node_couple=after.Vector(3,node);
      force=c::Add(force,node_force);
      moment=c::Add(moment,c::Add(node_couple,
          Cross(c::Subtract(x,{group.center.x,group.center.y,group.center.z}),
                node_force)));
      const auto increment=c::Subtract(node_force,before.Vector(0,node));
      contact_force=c::Add(contact_force,increment);
      contact_resultant=c::Add(contact_resultant,increment);
      contact_moment=c::Add(contact_moment,Cross(x,increment));
      if (Norm(increment)>0) {
        ASSERT_GT(member.mass_kg,0);
        ++contacted_nodes[g];
        endpoint_inverse_sum[g]=c::Add(endpoint_inverse_sum[g],
            c::Scale(increment,1/member.mass_kg));
      }
    }
    expected_acceleration[g]=c::Scale(force,1/group.mass_kg);
    expected_angular[g]=DenseAngularAcceleration(group,moment);
    merged_contact_acceleration[g]=c::Scale(contact_force,1/group.mass_kg);
  }
  // The fully rigid T3 face contributes multiple weighted PART nodes. The
  // opposite ordinary T3 contributes its contacting vertex to the partial
  // plain group; its third node remains ordinary, so this is not an
  // unsupported complete plain-rigid shell skin.
  EXPECT_GE(contacted_nodes[0],2u);
  EXPECT_GT(Norm(merged_contact_acceleration[0]),0);
  EXPECT_GT(Norm(c::Subtract(endpoint_inverse_sum[0],
                         merged_contact_acceleration[0])),
            1e-6*Norm(merged_contact_acceleration[0]));
  const double contact_scale=accepted.diagnostics().maximum_force_norm_n;
  Near(contact_resultant,accepted.diagnostics().equal_opposite_residual_n,
       contact_scale);
  Near(contact_moment,accepted.diagnostics().global_moment_n_m,
       contact_scale);
  Near(contact_resultant,{},contact_scale);

  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.Prepare(token,assembly,prepared,common));
  std::vector<double> force_stage(6*after.nodes);
  std::array<fe::NodalRigidGroupAccelerationSnapshot,2> actual;
  fe::NodalPreparedView captured;
  ASSERT_EQ(fixture.rig.owner.CopyPreparedForceStage(token,
      {force_stage.data(),force_stage.data()+3*after.nodes,after.nodes,
       actual.data(),actual.size()},&captured).status,fe::NodalStatus::Ok);
  EXPECT_EQ(captured.attempt,prepared.attempt);
  for (std::size_t g=0;g<actual.size();++g) {
    EXPECT_EQ(actual[g].source_kind,binding.groups()[g].source_kind);
    EXPECT_EQ(actual[g].source_group_id,binding.groups()[g].source_id);
    Near({actual[g].acceleration.x,actual[g].acceleration.y,
          actual[g].acceleration.z},expected_acceleration[g]);
    Near({actual[g].angular_acceleration.x,actual[g].angular_acceleration.y,
          actual[g].angular_acceleration.z},expected_angular[g]);
  }
  c::SelfContactTransactionReceipt unsupported;
  const auto rigid_interval=fixture.transaction.SealCandidate(
      fixture.rig.owner,token,common,prepared,accepted,&unsupported);
  EXPECT_EQ(rigid_interval.status,
            c::SelfContactTransactionStatus::UnsupportedMotion);
  EXPECT_FALSE(unsupported.valid());
  fixture.Discard();
  EXPECT_EQ(fixture.rig.owner.accepted().epoch,0u);
}

TEST(SelfContactTransactionCuda,
     ActualT3RemovalFiltersCandidateAndLongInactiveRetryCommits) {
  Fixture fixture(false, false, 1.e-12);
  ASSERT_TRUE(fixture.DriveT3Removal());
  ASSERT_TRUE(fixture.Initialize());
  p::Snapshot before, after;
  ASSERT_TRUE(fixture.rig.Read(before));

  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  auto forged = common;
  ++forged.t3.attempt;
  c::SelfContactTransactionReceipt unchanged;
  const auto rejected = fixture.transaction.SealCandidate(
      fixture.rig.owner, token, forged, prepared, accepted, &unchanged);
  EXPECT_EQ(rejected.status,
            c::SelfContactTransactionStatus::ActivityFailure);
  EXPECT_FALSE(unchanged.valid());
  EXPECT_FALSE(accepted.valid());
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
  ASSERT_TRUE(fixture.rig.Read(after));
  p::Exact(before, after);

  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  std::uint8_t prepared_t3_activity = 1;
  ASSERT_TRUE(p::Good(fixture.rig.t3.CopyPreparedParentActivity(
      fixture.rig.owner, token, common.t3,
      &prepared_t3_activity, 1)));
  ASSERT_EQ(prepared_t3_activity, 0u);
  c::SelfContactTransactionReceipt removal;
  ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
      fixture.rig.owner, token, common, prepared, accepted, &removal)));
  EXPECT_FALSE(accepted.valid());
  EXPECT_EQ(removal.active_parents(), 1u);
  EXPECT_EQ(removal.removing_parents(), 1u);
  EXPECT_EQ(removal.skipped_parents(), 0u);
  EXPECT_EQ(removal.facet_pairs(), 0u);
  EXPECT_EQ(removal.policy_outcomes(), 0u);
  const auto expired = accepted;
  ASSERT_TRUE(fixture.Commit(token, prepared, common, removal));
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 1u);

  c::SelfContactAcceptedAssemblyReceipt inactive;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &inactive)));
  EXPECT_EQ(inactive.facet_pairs(), 0u);
  EXPECT_EQ(inactive.diagnostics().event_count, 0u);
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  EXPECT_NE(fixture.transaction.SealCandidate(
      fixture.rig.owner, token, common, prepared, expired,
      &unchanged).status, c::SelfContactTransactionStatus::Ok);
  EXPECT_FALSE(unchanged.valid());
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 1u);

  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &inactive)));
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  c::SelfContactTransactionReceipt long_inactive;
  ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
      fixture.rig.owner, token, common, prepared, inactive,
      &long_inactive)));
  EXPECT_EQ(long_inactive.active_parents(), 1u);
  EXPECT_EQ(long_inactive.removing_parents(), 0u);
  EXPECT_EQ(long_inactive.skipped_parents(), 1u);
  EXPECT_EQ(long_inactive.facet_pairs(), 0u);
  EXPECT_EQ(long_inactive.policy_outcomes(), 0u);
  ASSERT_TRUE(fixture.Commit(
      token, prepared, common, long_inactive));
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 2u);
}

TEST(SelfContactTransactionCuda,
     ExactPassThroughUnresolvedReasonRollsBackAndRetriesExactly) {
  Fixture fixture(false, true);
  auto limits = c::SelfContactTransactionLimits{};
  limits.crossing.max_depth = 4;
  limits.crossing.max_work_per_pair = 31;
  limits.crossing.max_total_work = 31 * 64;
  ASSERT_TRUE(fixture.Initialize(limits));
  p::Snapshot before, after;
  ASSERT_TRUE(fixture.rig.Read(before));

  std::size_t failed_pair = SIZE_MAX;
  for (unsigned retry = 0; retry < 2; ++retry) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    ASSERT_GT(accepted.diagnostics().active_count, 0u);

    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(
        token, assembly, prepared, common));
    c::SelfContactTransactionReceipt unchanged;
    const auto report = fixture.transaction.SealCandidate(
        fixture.rig.owner, token, common, prepared, accepted, &unchanged);
    EXPECT_EQ(report.status,
              c::SelfContactTransactionStatus::UnresolvedCandidate);
    EXPECT_EQ(report.crossing_reason,
              c::RepresentedIntervalReason::WorkExhausted);
    EXPECT_FALSE(unchanged.valid());
    EXPECT_NE(report.pair, SIZE_MAX);
    if (!retry)
      failed_pair = report.pair;
    else
      EXPECT_EQ(report.pair, failed_pair);
    EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
    ASSERT_TRUE(fixture.rig.Read(after));
    p::Exact(before, after);
  }
}

}  // namespace self_contact_transaction_cuda_test
