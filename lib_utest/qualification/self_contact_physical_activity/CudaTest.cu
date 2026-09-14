// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../physical_publication/OwnerFixture.h"
#include "../qbat_resident/ResidentFixture.h"
#include "lib_src/collision/FixedContactFacetBinding.h"
#include "lib_src/collision/SelfContactPhysicalActivity.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace self_contact_physical_activity_cuda_test {
namespace c = tlfea::contact;
namespace fe = tl::fea;
namespace p = physical_publication_test;

bool Good(const c::SelfContactPhysicalActivityReport& report) {
  EXPECT_EQ(report.status, c::SelfContactPhysicalActivityStatus::Ok)
      << report.message << " parent=" << report.parent
      << " family-index=" << report.family_index;
  return report.status == c::SelfContactPhysicalActivityStatus::Ok;
}

struct Fixture {
  explicit Fixture(double t3_failure = 2.5,
                   bool t3_only = false)
      : rig(false, t3_failure), t3_only(t3_only),
        t3_failure(t3_failure) {}

  p::Rig rig;
  fe::ShellBatchPlasticityBinding execution_catalog;
  fe::ShellBatchFailureBinding execution_failure;
  fe::ShellExecutionBinding execution;
  fe::ShellPhysicalBinding physical;
  c::SelfContactSurfaceBinding surface;
  c::FixedContactFacetBinding facets;
  c::SelfContactActiveUseBinding uses;
  c::SelfContactPhysicalActivity authority;
  std::vector<c::SelfContactParentSelection> selection;
  bool t3_only = false;
  double t3_failure = 2.5;

  bool InitializeExecutionAuthority() {
    qbat_catalog_test::Fixture declaration;
    for (unsigned row = 0; row < 2; ++row) {
      declaration.materials[row].curve_id = 0;
      declaration.materials[row].hardening =
          tl::material::ShellPlasticityHardeningKind::LinearLaw44;
      declaration.materials[row].linear = {10e6, 0};
      declaration.materials[row].rate =
          declaration.materials[2].rate;
    }
    const fe::ShellPlasticityParentInput parents[]{
        {fe::ShellBindingFamily::Qeph, 0, 100,
         1000, 1000, 1000},
        {fe::ShellBindingFamily::Qeph, 1, 101,
         1001, 1001, 1001},
        {fe::ShellBindingFamily::T3, 0, 102,
         2000524, 2000524, 2000524},
        {fe::ShellBindingFamily::Qbat, 0, 103,
         2000524, 2000524, 2000524}};
    auto catalog = execution_catalog.InitializeExecutionCatalog(
        rig.fixture.source.shells,
        {nullptr, declaration.materials.data(),
         declaration.sections.data(), parents, 0, 3, 3, 4});
    EXPECT_EQ(catalog.status,
              fe::ShellPlasticityBindingStatus::Success)
        << catalog.message;
    if (catalog.status !=
        fe::ShellPlasticityBindingStatus::Success)
      return false;

    fe::ShellFailureParentInput failures[4];
    for (unsigned row = 0; row < 4; ++row)
      failures[row] = qbat_catalog_test::Failure(parents[row]);
    failures[2].constant.failure_strain = t3_failure;
    for (unsigned row = 0; row < 2; ++row) {
      failures[row].policy =
          fe::ShellFailurePolicy::Tab1AnyPoint;
      failures[row].constant = {};
      failures[row].tab1.table = {{-1, 0, 1}, 1};
    }
    const auto failure = execution_failure.InitializeExecution(
        execution_catalog, failures, 4);
    EXPECT_EQ(failure.status,
              fe::ShellPlasticityBindingStatus::Success)
        << failure.message;
    if (failure.status !=
        fe::ShellPlasticityBindingStatus::Success)
      return false;

    const auto executed = execution.Initialize(
        execution_catalog, rig.fixture.ledger,
        rig.fixture.rigid);
    EXPECT_EQ(executed.status,
              fe::ShellPlasticityBindingStatus::Success)
        << executed.message;
    if (executed.status !=
        fe::ShellPlasticityBindingStatus::Success)
      return false;
    if (!p::Good(physical.InitializeExecution(
            {&rig.fixture.source.shells, &execution_catalog,
             &execution_failure, nullptr},
            rig.fixture.ledger, execution)))
      return false;

    for (std::size_t row = 0;
         row < execution_catalog.parent_count(); ++row) {
      const auto& parent = *execution_catalog.parent(row);
      if (parent.family != fe::ShellBindingFamily::T3 &&
          parent.family != fe::ShellBindingFamily::Qbat)
        continue;
      if (t3_only && parent.family != fe::ShellBindingFamily::T3)
        continue;
      selection.push_back({
          row, parent.family, parent.family_index,
          parent.source_parent_id, parent.source_part_id});
    }
    const auto selected = surface.Initialize(
        physical, {selection.data(), selection.size()});
    EXPECT_EQ(selected.status, c::SelfContactSurfaceStatus::Ok)
        << selected.message;
    if (selected.status != c::SelfContactSurfaceStatus::Ok)
      return false;
    const auto fixed = facets.Initialize(surface);
    EXPECT_EQ(fixed.status, c::FixedContactFacetStatus::Ok)
        << fixed.message;
    if (fixed.status != c::FixedContactFacetStatus::Ok)
      return false;
    const auto cin = rig.fixture.WitnessSource();
    c::SelfContactActiveUseSource source;
    source.rigid = &rig.fixture.rigid;
    source.cin = {
        cin.model, cin.ranges, cin.witnesses,
        cin.range_count, cin.witness_count};
    const auto active = uses.Initialize(facets, source);
    EXPECT_EQ(active.status, c::SelfContactActiveUseStatus::Ok)
        << active.message;
    return active.status == c::SelfContactActiveUseStatus::Ok;
  }

  bool Initialize(
      c::SelfContactPhysicalActivityLimits limits = {}) {
    if (!InitializeExecutionAuthority() ||
        !rig.InitializeAgainst(physical))
      return false;
    const auto forecast = c::SelfContactPhysicalActivity::Forecast(
        uses, physical, limits);
    if (!Good(forecast.report)) return false;
    return Good(authority.Initialize(
        uses, rig.owner, rig.publication, physical,
        rig.Participants(), rig.fixture.Identity(), limits));
  }

  bool Begin(fe::NodalTrialToken& token,
             fe::NodalAssemblyView& assembly,
             c::SelfContactAcceptedActivityReceipt& receipt) {
    return rig.Begin(token, assembly) &&
        Good(authority.CaptureAccepted(
            rig.owner, token, assembly, &receipt));
  }

  bool Prepare(
      const fe::NodalTrialToken& token,
      const fe::NodalAssemblyView& assembly,
      const c::SelfContactAcceptedActivityReceipt& accepted,
      fe::NodalPreparedView& prepared,
      fe::ShellPhysicalDiagnostics& common,
      c::SelfContactPreparedActivityReceipt& receipt) {
    fe::ShellPhysicalDiagnostics candidates;
    return rig.Advance(token, assembly, prepared) &&
        rig.Evaluate(token, prepared, candidates) &&
        p::Good(rig.publication.PreparePhysical(
            rig.owner, token,
            {&candidates.qeph, &candidates.t3,
             &candidates.qbat, &candidates.type25,
             &candidates.type13, &candidates.solids},
            &common)) &&
        Good(authority.CapturePrepared(
            rig.owner, token, common, prepared,
            accepted, &receipt));
  }

  bool Commit(const fe::NodalTrialToken& token,
              const fe::NodalPreparedView& prepared,
              const fe::ShellPhysicalDiagnostics& common) {
    return p::Good(rig.publication.CommitPhysical(
        rig.owner, token, common,
        {prepared.owner_id,
         prepared.kinematics.base_epoch,
         prepared.attempt, p::Qualification, true}));
  }

  void Discard() {
    authority.DiscardTrial();
    rig.owner.Discard();
    rig.publication.DiscardTrial();
  }

  std::size_t Parent(
      fe::ShellBindingFamily family,
      std::size_t family_index = 0) const {
    const auto parents = uses.parents();
    for (std::size_t parent = 0;
         parent < parents.size(); ++parent)
      if (parents[parent].source.family == family &&
          parents[parent].source.family_index == family_index)
        return parent;
    return SIZE_MAX;
  }

  bool DriveT3Removal() {
    const auto apex = rig.fixture.domain.Find(14);
    if (apex == SIZE_MAX) return false;
    const double mass =
        rig.fixture.ledger.nodes()[apex].coefficients.mass;
    if (!(mass > 0)) return false;
    rig.external_force_source_node = 14;
    rig.external_force_z_n =
        -2 * mass * .00075 / (p::H * p::H);
    return true;
  }

  bool MatchesAccepted(
      const c::SelfContactAcceptedActivityReceipt& receipt) {
    const auto& shells = *physical.shells();
    std::vector<std::uint8_t> q(shells.qeph_count());
    std::vector<std::uint8_t> t(shells.t3_count());
    std::vector<std::uint8_t> b(shells.qbat_count());
    fe::qeph::BatchDiagnostics qd;
    fe::t3::BatchDiagnostics td;
    fe::qbat::BatchDiagnostics bd;
    if (rig.qeph.CopyAcceptedParentActivity(
            rig.owner.accepted(), q.data(), q.size(), &qd).status !=
            fe::qeph::BatchStatus::Success ||
        rig.t3.CopyAcceptedParentActivity(
            rig.owner.accepted(), t.data(), t.size(), &td).status !=
            fe::t3::BatchStatus::Success ||
        rig.qbat.CopyAcceptedParentActivity(
            rig.owner.accepted(), b.data(), b.size(), &bd).status !=
            fe::qbat::BatchStatus::Success)
      return false;
    const auto activity = receipt.activity();
    const auto parents = uses.parents();
    if (activity.parent_count != parents.size()) return false;
    for (std::size_t parent = 0; parent < parents.size(); ++parent) {
      const auto& source = parents[parent].source;
      const std::uint8_t actual =
          source.family == fe::ShellBindingFamily::Qeph
              ? q[source.family_index]
              : source.family == fe::ShellBindingFamily::T3
                    ? t[source.family_index]
                    : b[source.family_index];
      if (activity.base[parent] != actual ||
          activity.current[parent] != actual)
        return false;
    }
    return true;
  }
};

TEST(SelfContactPhysicalActivityCuda,
     ExactCapAssemblyParticipantsAndReceiptLifetime) {
  Fixture fixture;
  ASSERT_TRUE(fixture.InitializeExecutionAuthority());
  ASSERT_TRUE(fixture.rig.InitializeAgainst(fixture.physical));
  auto limits = c::SelfContactPhysicalActivityLimits{};
  const auto exact = c::SelfContactPhysicalActivity::Forecast(
      fixture.uses, fixture.physical, limits);
  ASSERT_TRUE(Good(exact.report));
  limits.max_host_bytes = exact.forecast.owned_host_bytes - 1;
  EXPECT_EQ(c::SelfContactPhysicalActivity::Forecast(
      fixture.uses, fixture.physical, limits).report.status,
      c::SelfContactPhysicalActivityStatus::ResourceLimit);
  ++limits.max_host_bytes;
  limits.max_startup_host_bytes =
      exact.forecast.startup_host_bytes - 1;
  EXPECT_EQ(c::SelfContactPhysicalActivity::Forecast(
      fixture.uses, fixture.physical, limits).report.status,
      c::SelfContactPhysicalActivityStatus::ResourceLimit);
  ++limits.max_startup_host_bytes;

  fe::qbat::Batch foreign;
  auto forged_participants = fixture.rig.Participants();
  forged_participants.qbat = &foreign;
  c::SelfContactPhysicalActivity rejected;
  EXPECT_EQ(rejected.Initialize(
      fixture.uses, fixture.rig.owner,
      fixture.rig.publication, fixture.physical,
      forged_participants, fixture.rig.fixture.Identity(),
      limits).status,
      c::SelfContactPhysicalActivityStatus::PublicationFailure);
  ASSERT_TRUE(Good(fixture.authority.Initialize(
      fixture.uses, fixture.rig.owner,
      fixture.rig.publication, fixture.physical,
      fixture.rig.Participants(),
      fixture.rig.fixture.Identity(), limits)));

  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(p::Good(
      fixture.rig.owner.BeginTrial(&token, &assembly)));
  c::SelfContactAcceptedActivityReceipt incomplete;
  EXPECT_EQ(fixture.authority.CaptureAccepted(
      fixture.rig.owner, token, assembly,
      &incomplete).status,
      c::SelfContactPhysicalActivityStatus::AssemblyIncomplete);
  EXPECT_FALSE(incomplete.valid());
  fixture.Discard();

  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  fe::NodalTrialToken forged_token;
  EXPECT_NE(fixture.authority.CaptureAccepted(
      fixture.rig.owner, forged_token, assembly,
      &incomplete).status,
      c::SelfContactPhysicalActivityStatus::Ok);
  EXPECT_FALSE(incomplete.valid());
  fixture.Discard();

  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  const auto attempt = assembly.attempt;
  EXPECT_EQ(fixture.authority.CaptureAccepted(
      fixture.rig.owner, token, assembly,
      reinterpret_cast<c::SelfContactAcceptedActivityReceipt*>(
          &assembly)).status,
      c::SelfContactPhysicalActivityStatus::InvalidInput);
  EXPECT_EQ(assembly.attempt, attempt);
  fixture.Discard();

  c::SelfContactAcceptedActivityReceipt accepted;
  ASSERT_TRUE(fixture.Begin(token, assembly, accepted));
  ASSERT_TRUE(accepted.valid());
  ASSERT_TRUE(fixture.MatchesAccepted(accepted));
  const auto view = accepted.activity();
  ASSERT_EQ(view.parent_count, fixture.uses.parents().size());
  for (std::size_t parent = 0; parent < view.parent_count;
       ++parent)
    EXPECT_EQ(view.base[parent], 1);
  const std::vector<std::uint8_t> retained(
      view.base, view.base + view.parent_count);
  const auto allocations = fixture.authority.allocations();

  auto forged_view = assembly;
  ++forged_view.attempt;
  c::SelfContactAcceptedActivityReceipt unchanged;
  EXPECT_NE(fixture.authority.CaptureAccepted(
      fixture.rig.owner, token, forged_view,
      &unchanged).status,
      c::SelfContactPhysicalActivityStatus::Ok);
  EXPECT_FALSE(accepted.valid());
  EXPECT_FALSE(unchanged.valid());
  EXPECT_TRUE(std::equal(
      retained.begin(), retained.end(), view.base));
  EXPECT_EQ(fixture.authority.allocations().host_bytes,
            allocations.host_bytes);
  fixture.Discard();
}

TEST(SelfContactPhysicalActivityCuda,
     ForgedCandidateLateReadbackRollbackAndExactRetry) {
  Fixture fixture;
  ASSERT_TRUE(fixture.Initialize());
  const auto allocation = fixture.authority.allocations();
  const auto stamp = fixture.rig.owner.accepted();

  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  c::SelfContactAcceptedActivityReceipt accepted;
  ASSERT_TRUE(fixture.Begin(token, assembly, accepted));
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics candidates, common;
  ASSERT_TRUE(fixture.rig.Advance(token, assembly, prepared));
  ASSERT_TRUE(fixture.rig.Evaluate(
      token, prepared, candidates));
  ASSERT_TRUE(p::Good(
      fixture.rig.publication.PreparePhysical(
          fixture.rig.owner, token,
          {&candidates.qeph, &candidates.t3,
           &candidates.qbat, &candidates.type25,
           &candidates.type13, &candidates.solids},
          &common)));
  auto forged = common;
  ++forged.qbat.attempt;
  c::SelfContactPreparedActivityReceipt rejected;
  EXPECT_NE(fixture.authority.CapturePrepared(
      fixture.rig.owner, token, forged, prepared,
      accepted, &rejected).status,
      c::SelfContactPhysicalActivityStatus::Ok);
  EXPECT_FALSE(accepted.valid());
  EXPECT_FALSE(rejected.valid());
  EXPECT_TRUE(fe::trial_identity::SameStamp(
      fixture.rig.owner.accepted(), stamp));
  fixture.Discard();

  ASSERT_TRUE(fixture.Begin(token, assembly, accepted));
  ASSERT_TRUE(fixture.rig.Advance(token, assembly, prepared));
  ASSERT_TRUE(fixture.rig.Evaluate(
      token, prepared, candidates));
  ASSERT_TRUE(p::Good(
      fixture.rig.publication.PreparePhysical(
          fixture.rig.owner, token,
          {&candidates.qeph, &candidates.t3,
           &candidates.qbat, &candidates.type25,
           &candidates.type13, &candidates.solids},
          &common)));
  qbat_resident_test::Arm(
      qbat_resident_test::ReadFault::LateNonfinite,
      fixture.physical.shells()->qbat_count());
  EXPECT_EQ(fixture.authority.CapturePrepared(
      fixture.rig.owner, token, common, prepared,
      accepted, &rejected).status,
      c::SelfContactPhysicalActivityStatus::QbatFailure);
  EXPECT_FALSE(accepted.valid());
  EXPECT_TRUE(fe::trial_identity::SameStamp(
      fixture.rig.owner.accepted(), stamp));
  fixture.Discard();

  ASSERT_TRUE(fixture.Begin(token, assembly, accepted));
  c::SelfContactPreparedActivityReceipt activity;
  ASSERT_TRUE(fixture.Prepare(
      token, assembly, accepted, prepared, common, activity));
  ASSERT_TRUE(activity.valid());
  EXPECT_FALSE(accepted.valid());
  const auto actual = activity.activity();
  for (std::size_t parent = 0;
       parent < actual.parent_count; ++parent)
    EXPECT_LE(actual.current[parent], actual.base[parent]);
  ASSERT_TRUE(fixture.Commit(token, prepared, common));
  EXPECT_FALSE(activity.valid());
  EXPECT_EQ(fixture.authority.allocations().host_bytes,
            allocation.host_bytes);
  EXPECT_EQ(fixture.authority.allocations().host_allocations,
            allocation.host_allocations);
}

TEST(SelfContactPhysicalActivityCuda,
     ActualMixedRemovalLastParentAndLongInactive) {
  Fixture mixed(1.e-12);
  ASSERT_TRUE(mixed.DriveT3Removal());
  ASSERT_TRUE(mixed.Initialize());
  bool removed = false;
  bool long_inactive = false;
  for (unsigned step = 0; step < 64 && !long_inactive; ++step) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    c::SelfContactAcceptedActivityReceipt accepted;
    ASSERT_TRUE(mixed.Begin(token, assembly, accepted));
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    c::SelfContactPreparedActivityReceipt current;
    ASSERT_TRUE(mixed.Prepare(
        token, assembly, accepted, prepared, common, current));
    const auto activity = current.activity();
    const auto t3 = mixed.Parent(fe::ShellBindingFamily::T3);
    ASSERT_NE(t3, SIZE_MAX);
    if (activity.base[t3] == 1 && activity.current[t3] == 0) {
      removed = true;
      bool other_active = false;
      for (std::size_t parent = 0;
           parent < activity.parent_count; ++parent)
        other_active = other_active ||
            (parent != t3 && activity.current[parent] == 1);
      EXPECT_TRUE(other_active);
    }
    if (activity.base[t3] == 0 && activity.current[t3] == 0)
      long_inactive = true;
    ASSERT_TRUE(mixed.Commit(token, prepared, common));
  }
  EXPECT_TRUE(removed);
  EXPECT_TRUE(long_inactive);

  Fixture last(1.e-12, true);
  ASSERT_TRUE(last.DriveT3Removal());
  ASSERT_TRUE(last.Initialize());
  bool last_removed = false;
  for (unsigned step = 0; step < 64 && !last_removed; ++step) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    c::SelfContactAcceptedActivityReceipt accepted;
    ASSERT_TRUE(last.Begin(token, assembly, accepted));
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    c::SelfContactPreparedActivityReceipt current;
    ASSERT_TRUE(last.Prepare(
        token, assembly, accepted, prepared, common, current));
    const auto activity = current.activity();
    ASSERT_EQ(activity.parent_count, 1u);
    last_removed =
        activity.base[0] == 1 && activity.current[0] == 0;
    ASSERT_TRUE(last.Commit(token, prepared, common));
  }
  EXPECT_TRUE(last_removed);

  c::SelfContactPreparedActivityReceipt expired;
  {
    Fixture lifetime;
    ASSERT_TRUE(lifetime.Initialize());
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    c::SelfContactAcceptedActivityReceipt accepted;
    ASSERT_TRUE(lifetime.Begin(token, assembly, accepted));
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(lifetime.Prepare(
        token, assembly, accepted, prepared, common, expired));
    ASSERT_TRUE(expired.valid());
  }
  EXPECT_FALSE(expired.valid());
  EXPECT_EQ(expired.activity().base, nullptr);
}

}  // namespace self_contact_physical_activity_cuda_test
