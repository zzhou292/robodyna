// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/self_contact_transaction/CandidateFailureCapture.h"

namespace self_contact_transaction_cuda_test {
namespace failure_capture_test {

void ExactReport(const c::SelfContactTransactionReport& a,
                 const c::SelfContactTransactionReport& b) {
#define FIELD(name) EXPECT_EQ(a.name, b.name)
  FIELD(status); FIELD(candidate); FIELD(pair); FIELD(count_kind);
  FIELD(force_status); FIELD(activity_status); FIELD(broadphase_status);
  FIELD(regularity_status); FIELD(discovery_status); FIELD(discovery_task);
  FIELD(discovery_reason); FIELD(crossing_status); FIELD(crossing_reason);
  FIELD(nonlinear_subdivision_work); FIELD(nonlinear_subdivision_depth);
  FIELD(nonlinear_subdivision_work_exhausted);
  FIELD(nonlinear_subdivision_depth_exhausted);
  FIELD(publication_status); FIELD(owner_status);
#undef FIELD
  EXPECT_STREQ(a.message, b.message);
  EXPECT_EQ(p::Bits(a.offending_feature_distance_m),
            p::Bits(b.offending_feature_distance_m));
  const auto vector = [](c::Vec3 first, c::Vec3 second) {
    EXPECT_EQ(p::Bits(first.x), p::Bits(second.x));
    EXPECT_EQ(p::Bits(first.y), p::Bits(second.y));
    EXPECT_EQ(p::Bits(first.z), p::Bits(second.z));
  };
  for (unsigned side = 0; side < 2; ++side) {
    const auto& first = a.offending_motion[side];
    const auto& second = b.offending_motion[side];
    EXPECT_EQ(c::fixed_triangle_features::Compare(first.facet, second.facet), 0);
    EXPECT_EQ(first.active_parent, second.active_parent);
    EXPECT_EQ(first.motion, second.motion);
    EXPECT_EQ(first.rigid_group_count, second.rigid_group_count);
    for (unsigned group = 0; group < 4; ++group) {
      const auto& x = first.rigid_groups[group];
      const auto& y = second.rigid_groups[group];
      EXPECT_EQ(x.binding_group, y.binding_group);
      EXPECT_EQ(x.source_kind, y.source_kind);
      EXPECT_EQ(x.source_group_id, y.source_group_id);
      EXPECT_EQ(x.source_node_set_id, y.source_node_set_id);
    }
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      vector(a.offending_quadratic_lower[side][vertex],
             b.offending_quadratic_lower[side][vertex]);
      vector(a.offending_quadratic_upper[side][vertex],
             b.offending_quadratic_upper[side][vertex]);
    }
    vector(a.offending_swept_bounds[side].lower,
           b.offending_swept_bounds[side].lower);
    vector(a.offending_swept_bounds[side].upper,
           b.offending_swept_bounds[side].upper);
    EXPECT_EQ(p::Bits(a.offending_half_thickness_m[side]),
              p::Bits(b.offending_half_thickness_m[side]));
    EXPECT_EQ(p::Bits(a.offending_edge_parameters[side]),
              p::Bits(b.offending_edge_parameters[side]));
  }
}

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
