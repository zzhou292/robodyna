// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../physical_publication/OwnerFixture.h"
#include "lib_src/collision/SelfContactTransaction.h"
#include "lib_src/collision/FixedContactFacetValues.h"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace self_contact_transaction_cuda_test {
namespace c = tlfea::contact;
namespace fe = tl::fea;
namespace p = physical_publication_test;

bool Good(c::SelfContactTransactionReport report) {
  EXPECT_EQ(report.status, c::SelfContactTransactionStatus::Ok)
      << report.message << " candidate=" << report.candidate
      << " pair=" << report.pair
      << " crossing_reason="
      << static_cast<unsigned>(report.crossing_reason);
  return report.status == c::SelfContactTransactionStatus::Ok;
}

struct Fixture {
  explicit Fixture(bool single = false, bool crossing = false)
      : rig(false, 2.5, !single),
        single_parent(single), pass_through(crossing) {}
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
  bool authority_prepared = false;

  bool PrepareExecutionAuthority() {
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
    const auto catalog_report =
        execution_catalog.InitializeExecutionCatalog(
            rig.fixture.source.shells,
            {nullptr, declaration.materials.data(),
             declaration.sections.data(), parents, 0, 3, 3, 4});
    EXPECT_EQ(catalog_report.status,
              fe::ShellPlasticityBindingStatus::Success)
        << catalog_report.message;
    if (catalog_report.status !=
        fe::ShellPlasticityBindingStatus::Success)
      return false;

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
      if (parent.family != fe::ShellBindingFamily::T3 &&
          parent.family != fe::ShellBindingFamily::Qbat)
        continue;
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
    const auto facet_report = facets.Initialize(surface, {{}, 1});
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
  limits.max_host_bytes = exact.forecast.owned_host_bytes - 1;
  EXPECT_EQ(c::SelfContactTransaction::Forecast(
      fixture.config, fixture.uses, fixture.rig.fixture.Identity(),
      limits).report.status, c::SelfContactTransactionStatus::ResourceLimit);
  ++limits.max_host_bytes;
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
  EXPECT_EQ(fixture.transaction.allocations().device_allocations, 2u);
  EXPECT_EQ(fixture.transaction.allocations().device_bytes,
            exact.forecast.device_bytes);
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
      fixture.rig.owner, token, prepared, accepted, &receipt)));
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
     MandatoryReceiptRollbackRetryHalfKickAndOrdinaryKeepNonzeroForceSti) {
  Fixture fixture;
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
        fixture.rig.owner, token, {}, {}, &unchanged).status,
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
          fixture.rig.owner, token, {}, accepted,
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
        fixture.rig.owner, token, prepared, accepted, &receipt)));
    ASSERT_TRUE(receipt.valid());
    ASSERT_TRUE(fixture.Commit(token, prepared, common, receipt));
    EXPECT_EQ(fixture.rig.owner.accepted().epoch,
              static_cast<std::uint64_t>(interval + 1));
    EXPECT_EQ(p::Bits(fixture.rig.owner.accepted().reaction_kick_dt),
              p::Bits(interval ? p::H : .5 * p::H));
    EXPECT_EQ(fixture.transaction.allocations().device_bytes,
              allocation.device_bytes);
    EXPECT_EQ(fixture.transaction.allocations().device_allocations,
              allocation.device_allocations);
  }
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
        fixture.rig.owner, token, prepared, accepted, &unchanged);
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
