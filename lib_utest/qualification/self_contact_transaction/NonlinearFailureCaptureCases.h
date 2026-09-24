// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <new>
#include <stdexcept>

namespace self_contact_transaction_cuda_test {
namespace nonlinear_failure_capture_test {
struct Observation : failure_capture_test::Observation {
  std::size_t facet_capacity = 2;
  bool simulate_allocation_failure = false;
  bool copied = false, copy_failed = false, has_curvature = false;
  std::array<c::CurrentFixedTriangle, 2> base, prepared;
  std::array<sct::FacetQuadraticCoefficients, 2> quadratic;
  std::array<double, 2> half_thickness{};

  static void Capture(void* context, const sct::CandidateFailureCapture& value) noexcept {
    auto& out = *static_cast<Observation*>(context);
    failure_capture_test::Observation::Capture(
        static_cast<failure_capture_test::Observation*>(&out), value);
    try {
      if (out.simulate_allocation_failure) throw std::bad_alloc{};
      if (out.facet_capacity < 2) throw std::length_error("bounded facet capture capacity");
      if (!out.live || !out.correct_pair || !out.correct_phase)
        throw std::runtime_error("unauthenticated nonlinear capture");
      const std::uint32_t facets[]{value.facets.first, value.facets.second};
      std::array<c::CurrentFixedTriangle, 2> base, prepared;
      std::array<sct::FacetQuadraticCoefficients, 2> quadratic;
      std::array<double, 2> thickness{};
      bool curvature = false;
      for (unsigned side = 0; side < 2; ++side) {
        base[side] = value.motion.accepted_triangles[facets[side]];
        prepared[side] = value.motion.prepared_triangles[facets[side]];
        quadratic[side] = value.motion.quadratic[facets[side]];
        thickness[side] = value.motion.descriptors[facets[side]].reference_half_thickness_m;
        for (const auto& vertex : quadratic[side].q)
          for (const auto& component : vertex)
            curvature = curvature || component.lower != 0 || component.upper != 0;
      }
      out.base = base; out.prepared = prepared; out.quadratic = quadratic;
      out.half_thickness = thickness; out.has_curvature = curvature; out.copied = true;
    } catch (...) {
      out.copy_failed = true; // Diagnostic failure cannot replace the physical rejection.
    }
  }
  sct::CandidateFailureObserver observer() noexcept { return {this, sizeof(*this), Capture}; }
};
}  // namespace nonlinear_failure_capture_test

TEST(SelfContactTransactionCuda, NonlinearTerminalObserverKeepsActualCurvatureRejectionAndRollback) {
  // Reuse the actual PART/plain/CIN owner fixture. A real external load at a
  // rigid member produces curvature; coordinates, velocities and stamps are
  // never supplied by this test. The deliberately small proof budget makes
  // the terminal nonlinear coverage rejection inexpensive and reproducible.
  Fixture fixture(false, false, 2.5, p::ContactConstraintLayout::MergedPartAndPlain);
  fixture.rig.external_force_source_node = 14;
  fixture.rig.external_force_z_n = 1000;
  c::SelfContactTransactionLimits limits;
  limits.max_nonlinear_subdivision_work_per_pair = 1;
  limits.max_nonlinear_subdivision_depth = 0;
  ASSERT_TRUE(fixture.Initialize(limits));
  p::Snapshot before, after;
  ASSERT_TRUE(fixture.rig.Read(before));
  c::SelfContactTransactionReport baseline;
  for (unsigned mode = 0; mode < 4; ++mode) {
    SCOPED_TRACE(mode); // plain / bounded capture / short capacity / caught allocation failure
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    ASSERT_GT(accepted.diagnostics().active_count, 0u);
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    ASSERT_EQ(prepared.rigid_member_trajectory,
              fe::NodalRigidMemberTrajectory::EndpointCorrectedSecondOrderDriftV1);
    nonlinear_failure_capture_test::Observation observation;
    observation.transaction = &fixture.transaction;
    observation.expected_accepted = before.stamp;
    observation.expected_prepared = prepared;
    observation.facet_capacity = mode == 2 ? 1 : 2;
    observation.simulate_allocation_failure = mode == 3;
    c::SelfContactTransactionReceipt receipt;
    const auto report = mode == 0
        ? fixture.transaction.SealCandidate(fixture.rig.owner, token, common, prepared, accepted, &receipt)
        : sct::QualificationAccess::SealCandidateWithFailureObserver(
              fixture.transaction, fixture.rig.owner, token, common, prepared,
              accepted, &receipt, observation.observer());
    ASSERT_EQ(report.status, c::SelfContactTransactionStatus::UnsupportedMotion) << report.message;
    EXPECT_STREQ(report.message, "Quadratic subdivision and ledger coverage remain unresolved");
    EXPECT_EQ(report.crossing_reason, c::RepresentedIntervalReason::UnsupportedMotion);
    EXPECT_EQ(report.nonlinear_subdivision_work, 1u);
    EXPECT_EQ(report.nonlinear_subdivision_depth, 0u);
    EXPECT_TRUE(report.nonlinear_subdivision_work_exhausted);
    EXPECT_TRUE(report.nonlinear_subdivision_depth_exhausted);
    EXPECT_FALSE(receipt.valid()); EXPECT_FALSE(accepted.valid());
    if (!mode) baseline = report;
    else {
      failure_capture_test::ExactReport(report, baseline);
      failure_capture_test::ExactReport(report, observation.retained.report);
      ASSERT_EQ(observation.calls, 1u);
      EXPECT_FALSE(observation.caught);
      EXPECT_TRUE(observation.live && observation.correct_phase && observation.correct_pair);
      ASSERT_TRUE(observation.retained.has_nonlinear_baseline);
      const auto& nonlinear = observation.retained.nonlinear_baseline;
      EXPECT_EQ(nonlinear.status, sct::NonlinearSeparationStatus::WorkExhausted);
      EXPECT_EQ(nonlinear.work, report.nonlinear_subdivision_work);
      EXPECT_EQ(nonlinear.deepest, report.nonlinear_subdivision_depth);
      EXPECT_TRUE(nonlinear.work_exhausted && nonlinear.depth_exhausted);
      EXPECT_EQ(observation.copied, mode == 1);
      EXPECT_EQ(observation.copy_failed, mode != 1);
      if (mode == 1) {
        EXPECT_TRUE(observation.has_curvature);
        for (unsigned side = 0; side < 2; ++side) {
          EXPECT_TRUE(observation.quadratic[side].complete);
          EXPECT_GT(observation.half_thickness[side], 0);
          EXPECT_EQ(c::fixed_triangle_features::Compare(observation.base[side].key,
                                                       observation.prepared[side].key), 0);
          EXPECT_EQ(c::fixed_triangle_features::Compare(observation.prepared[side].key,
                                                       report.offending_motion[side].facet), 0);
        }
      }
      EXPECT_FALSE(observation.retained.activity.valid());
      EXPECT_EQ(sct::QualificationAccess::ClassifyAcceptedFeaturePolicies(
          fixture.transaction, observation.retained.activity, {&observation.feature, 1, true},
          &observation.policy, 1, &observation.policy_count).status,
          c::SelfContactTransactionStatus::IdentityMismatch);
    }
    ASSERT_TRUE(fixture.rig.Read(after));
    p::Exact(before, after);
  }
}
}  // namespace self_contact_transaction_cuda_test
