// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/self_contact_transaction/CandidateFailureCapture.h"
#include "TransactionReportAssertions.h"

namespace self_contact_transaction_cuda_test {
namespace failure_capture_test {

using sct::test::ExpectTransactionReport;
inline void ExactReport(const c::SelfContactTransactionReport& a,
                        const c::SelfContactTransactionReport& b) { ExpectTransactionReport(a, b); }

struct Observation {
  c::SelfContactTransaction* transaction = nullptr;
  fe::NodalStamp expected_accepted;
  fe::NodalPreparedView expected_prepared;
  std::size_t calls = 0;
  bool live = false, correct_phase = false, correct_pair = false;
  bool nonlocal_intersection = false, caught = false;
  sct::CandidateFailureCapture retained;
  c::FixedTriangleFeatureCandidate feature;
  sct::AcceptedFeaturePolicyEvidence policy;
  c::SelfContactTransactionReport policy_report;
  std::size_t policy_count = 0;

  static void Capture(void* context,
                      const sct::CandidateFailureCapture& value) noexcept {
    auto& out = *static_cast<Observation*>(context);
    ++out.calls;
    try {
      out.retained = value;
      out.live = value.activity.valid() && value.motion.complete &&
          value.accepted_events.complete && value.transaction == out.transaction &&
          value.accepted_assembly && !value.accepted_assembly->valid();
      out.correct_phase = fe::trial_identity::SameStamp(
          out.expected_accepted, value.accepted) &&
          fe::trial_identity::SamePrepared(out.expected_prepared, value.prepared);
      if (!out.live || value.facets.first >= value.motion.facet_count ||
          value.facets.second >= value.motion.facet_count)
        return;
      const auto& first = value.motion.prepared_triangles[value.facets.first];
      const auto& second = value.motion.prepared_triangles[value.facets.second];
      out.correct_pair = c::fixed_triangle_features::Compare(first.key, second.key) < 0 &&
          c::fixed_triangle_features::Compare(
              first.key, value.report.offending_motion[0].facet) == 0 &&
          c::fixed_triangle_features::Compare(
              second.key, value.report.offending_motion[1].facet) == 0;
      c::FixedTriangleIntersection intersection;
      bool intersects = false;
      out.nonlocal_intersection = c::fixed_triangle_features::ClassifyPairIntersection(
          first, second, &intersection, &intersects) == c::FixedTriangleDiscoveryStatus::Ok &&
          intersects && c::RequiresIntersectionAdmission(intersection);
      if (!value.accepted_events.count) return;
      out.feature = value.accepted_events.data[0].discovery;
      out.policy_report = sct::QualificationAccess::ClassifyAcceptedFeaturePolicies(
          *out.transaction, value.activity, {&out.feature, 1, true},
          &out.policy, 1, &out.policy_count);
    } catch (...) {
      out.caught = true;
    }
  }
  sct::CandidateFailureObserver observer() noexcept {
    return {this, sizeof(*this), Capture};
  }
};

}  // namespace failure_capture_test

TEST(SelfContactTransactionCuda,
     CandidateFailureObserverSeesLiveSourceAndPreservesReportAndRollback) {
  Fixture fixture(false, true);
  auto limits = c::SelfContactTransactionLimits{};
  limits.crossing.max_depth = 4;
  limits.crossing.max_work_per_pair = 31;
  limits.crossing.max_total_work = 31 * 64;
  ASSERT_TRUE(fixture.Initialize(limits));
  p::Snapshot before, after;
  ASSERT_TRUE(fixture.rig.Read(before));
  c::SelfContactTransactionReport baseline;
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    failure_capture_test::Observation observation;
    observation.transaction = &fixture.transaction;
    observation.expected_accepted = fixture.rig.owner.accepted();
    observation.expected_prepared = prepared;
    c::SelfContactTransactionReceipt receipt;
    const auto report = attempt
        ? sct::QualificationAccess::SealCandidateWithFailureObserver(
              fixture.transaction, fixture.rig.owner, token, common, prepared,
              accepted, &receipt, observation.observer())
        : fixture.transaction.SealCandidate(
              fixture.rig.owner, token, common, prepared, accepted, &receipt);
    EXPECT_EQ(report.status, c::SelfContactTransactionStatus::CandidateRejected);
    EXPECT_FALSE(receipt.valid());
    EXPECT_FALSE(accepted.valid());
    EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
    if (!attempt) baseline = report;
    else {
      failure_capture_test::ExactReport(report, baseline);
      EXPECT_EQ(observation.calls, 1u);
      EXPECT_FALSE(observation.caught);
      EXPECT_TRUE(observation.live);
      EXPECT_TRUE(observation.correct_phase);
      EXPECT_TRUE(observation.correct_pair);
      EXPECT_TRUE(observation.nonlocal_intersection);
      EXPECT_FALSE(observation.retained.has_nonlinear_baseline);
      failure_capture_test::ExactReport(report, observation.retained.report);
      EXPECT_EQ(observation.policy_report.status, c::SelfContactTransactionStatus::Ok);
      EXPECT_EQ(observation.policy_count, 1u);
      EXPECT_TRUE(observation.policy.classification_complete);
      EXPECT_GT(observation.policy.ledger_key_matches, 0u);
      EXPECT_FALSE(observation.retained.activity.valid());
      EXPECT_FALSE(observation.retained.activity.activity_summary().complete);
      EXPECT_EQ(sct::QualificationAccess::ClassifyAcceptedFeaturePolicies(
          fixture.transaction, observation.retained.activity,
          {&observation.feature, 1, true}, &observation.policy, 1,
          &observation.policy_count).status, c::SelfContactTransactionStatus::IdentityMismatch);
    }
    // Observe/replay must not repair, suppress or change automatic rollback.
    ASSERT_TRUE(fixture.rig.Read(after));
    p::Exact(before, after);
  }
}

TEST(SelfContactTransactionCuda,
     CandidateFailureObserverAfterTwoCommitsPreservesLatestAcceptedState) {
  Fixture fixture(false, true);
  const auto load_node = fixture.rig.external_force_source_node;
  const auto load_force = fixture.rig.external_force_z_n;
  fixture.rig.external_force_source_node = 0;
  ASSERT_TRUE(fixture.Initialize());
  for (unsigned interval = 0; interval < 2; ++interval) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    c::SelfContactTransactionReceipt receipt;
    ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
        fixture.rig.owner, token, common, prepared, accepted, &receipt)));
    ASSERT_TRUE(fixture.Commit(token, prepared, common, receipt));
    ASSERT_EQ(fixture.rig.owner.accepted().epoch, interval + 1u);
  }
  p::Snapshot before, after;
  ASSERT_TRUE(fixture.rig.Read(before));
  ASSERT_EQ(before.stamp.epoch, 2u);
  EXPECT_EQ(p::Bits(before.stamp.time), p::Bits(2 * p::H));
  // Activate the existing pass-through coupon load only after two actual
  // commits. No coordinates, velocities, stamps or proof outcomes are forged.
  fixture.rig.external_force_source_node = load_node;
  fixture.rig.external_force_z_n = load_force;
  c::SelfContactTransactionReport baseline;
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    failure_capture_test::Observation observation;
    observation.transaction = &fixture.transaction;
    observation.expected_accepted = before.stamp;
    observation.expected_prepared = prepared;
    c::SelfContactTransactionReceipt receipt;
    const auto report = attempt
        ? sct::QualificationAccess::SealCandidateWithFailureObserver(
              fixture.transaction, fixture.rig.owner, token, common, prepared,
              accepted, &receipt, observation.observer())
        : fixture.transaction.SealCandidate(
              fixture.rig.owner, token, common, prepared, accepted, &receipt);
    ASSERT_EQ(report.status, c::SelfContactTransactionStatus::CandidateRejected)
        << report.message;
    EXPECT_FALSE(receipt.valid());
    EXPECT_FALSE(accepted.valid());
    if (!attempt) baseline = report;
    else {
      failure_capture_test::ExactReport(report, baseline);
      failure_capture_test::ExactReport(report, observation.retained.report);
      EXPECT_EQ(observation.calls, 1u);
      EXPECT_FALSE(observation.caught);
      EXPECT_TRUE(observation.live);
      EXPECT_TRUE(observation.correct_phase);
      EXPECT_TRUE(observation.correct_pair);
      EXPECT_TRUE(observation.nonlocal_intersection);
      EXPECT_FALSE(observation.retained.has_nonlinear_baseline);
      EXPECT_EQ(observation.retained.accepted.epoch, 2u);
      EXPECT_EQ(observation.retained.prepared.owner_id, before.stamp.owner_id);
      EXPECT_EQ(observation.policy_report.status, c::SelfContactTransactionStatus::Ok);
      EXPECT_EQ(observation.policy_count, 1u);
      EXPECT_FALSE(observation.retained.activity.valid());
      EXPECT_EQ(sct::QualificationAccess::ClassifyAcceptedFeaturePolicies(
          fixture.transaction, observation.retained.activity,
          {&observation.feature, 1, true}, &observation.policy, 1,
          &observation.policy_count).status, c::SelfContactTransactionStatus::IdentityMismatch);
    }
    ASSERT_TRUE(fixture.rig.Read(after));
    p::Exact(before, after);
  }
}

TEST(SelfContactTransactionCuda,
     CandidateFailureObserverDoesNotRunOnSuccessOrUnauthenticatedFailure) {
  Fixture fixture(true);
  ASSERT_TRUE(fixture.Initialize());
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    if (attempt) ++common.t3.attempt;
    failure_capture_test::Observation observation;
    c::SelfContactTransactionReceipt receipt;
    const auto report = sct::QualificationAccess::SealCandidateWithFailureObserver(
        fixture.transaction, fixture.rig.owner, token, common, prepared,
        accepted, &receipt, observation.observer());
    EXPECT_EQ(report.status, attempt ? c::SelfContactTransactionStatus::ActivityFailure
                                    : c::SelfContactTransactionStatus::Ok);
    EXPECT_EQ(receipt.valid(), !attempt);
    EXPECT_EQ(observation.calls, 0u);
    fixture.Discard();
  }
}

TEST(SelfContactTransactionCuda,
     CandidateFailureObserverRejectsAliasedAndOverflowedContextBeforeWrites) {
  Fixture fixture(true);
  ASSERT_TRUE(fixture.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  failure_capture_test::Observation observation;
  c::SelfContactTransactionReceipt receipt;
  const auto check = [&](sct::CandidateFailureObserver observer) {
    EXPECT_EQ(sct::QualificationAccess::SealCandidateWithFailureObserver(
        fixture.transaction, fixture.rig.owner, token, common, prepared,
        accepted, &receipt, observer).status, c::SelfContactTransactionStatus::InvalidInput);
    EXPECT_EQ(observation.calls, 0u);
    EXPECT_TRUE(accepted.valid());
    EXPECT_FALSE(receipt.valid());
  };
  auto observer = observation.observer();
  observer.capture = nullptr; check(observer);
  observer = observation.observer(); observer.context_bytes = SIZE_MAX; check(observer);
  observer = observation.observer(); observer.context = &receipt;
  observer.context_bytes = sizeof(receipt); check(observer);
  observer.context = &common; observer.context_bytes = sizeof(common); check(observer);
  observer.context = &fixture.transaction;
  observer.context_bytes = sizeof(fixture.transaction); check(observer);
  fixture.Discard();
}

}  // namespace self_contact_transaction_cuda_test
