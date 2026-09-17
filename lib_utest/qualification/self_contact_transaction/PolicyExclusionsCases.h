// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Included after ValueTest.cpp's native geometry and accepted-owner helpers.
namespace policy_exclusion_test {
using Result = sct::NonlinearSeparationResult;
using Status = sct::NonlinearSeparationStatus;

void ExpectFeature(const c::RepresentedFeaturePathKey& actual,
                   const c::RepresentedFeaturePathKey& expected) {
  EXPECT_EQ(actual.kind, expected.kind);
  EXPECT_EQ(c::fixed_triangle_features::Compare(actual.vertex, expected.vertex), 0);
  EXPECT_EQ(sct::Compare(actual.face, expected.face), 0);
  for (unsigned side = 0; side < 2; ++side)
    EXPECT_EQ(c::fixed_triangle_features::Compare(actual.edges[side], expected.edges[side]), 0);
}

void ExpectResult(const Result& actual, const Result& expected) {
#define POLICY_FIELD(name) EXPECT_EQ(actual.name, expected.name)
  POLICY_FIELD(status); POLICY_FIELD(work); POLICY_FIELD(deepest);
  POLICY_FIELD(separated_cells); POLICY_FIELD(covered_cells); POLICY_FIELD(closed_covered_cells);
  POLICY_FIELD(accepted_certificate); POLICY_FIELD(accepted_source_order);
  ExpectFeature(actual.feature, expected.feature);
  ExpectFeature(actual.intersection_feature, expected.intersection_feature);
  POLICY_FIELD(intersection_time_numerator); POLICY_FIELD(intersection_time_depth);
  POLICY_FIELD(has_intersection); POLICY_FIELD(proof_digest);
  POLICY_FIELD(work_exhausted); POLICY_FIELD(depth_exhausted);
  POLICY_FIELD(unresolved_path); POLICY_FIELD(unresolved_depth); POLICY_FIELD(has_unresolved_cell);
  ExpectFeature(actual.transition_feature, expected.transition_feature);
  POLICY_FIELD(transition_time_lower_numerator); POLICY_FIELD(transition_time_depth);
  POLICY_FIELD(transition_time_exact); POLICY_FIELD(transition_zero_geometry_separated);
  POLICY_FIELD(has_contact_transition); POLICY_FIELD(excluded_rigid_group);
#undef POLICY_FIELD
}

struct Probe {
  std::size_t calls = 0;
  std::size_t count = 1;
  sct::AcceptedFeatureExclusionCertificate exclusion;
  c::SelfContactTransactionReport failure;

  static c::SelfContactTransactionReport Prepare(
      void* context, sct::AcceptedFeatureExclusionCertificate* output,
      std::size_t capacity, std::size_t* written) noexcept {
    auto& probe = *static_cast<Probe*>(context);
    ++probe.calls;
    if (probe.failure.status != c::SelfContactTransactionStatus::Ok)
      return probe.failure;
    if (capacity && probe.count) output[0] = probe.exclusion;
    *written = probe.count;
    return {};
  }
  sct::PolicyExclusionSource Source() {
    return {this, &Prepare, 1, {}};
  }
};

struct SharedEdge {
  c::CurrentFixedTriangle first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {2, 0, 0}, {0, 1, 0}}});
  c::CurrentFixedTriangle second = Triangle(
      20, {1, 2, 4}, {{{0, 0, 0}, {2, 0, 0}, {0, -1, 0}}});
  sct::AcceptedFeatureExclusionCertificate Exclusion() const {
    const auto discovered = DiscoverPreparedPair(first, second);
    return {FirstEdgeEdge(discovered), 7};
  }
  Result Run(const sct::AcceptedFeatureExclusionCertificate* eager,
             std::size_t count, sct::PolicyExclusionSource* deferred) const {
    return sct::CertifyQuadraticFacetPolicyCoverage(
        first, first, Quadratic(0), 1,
        second, second, Quadratic(0), 1, 1,
        nullptr, 0, eager, count, 255, 8, deferred);
  }
};
}  // namespace policy_exclusion_test

TEST(SelfContactPolicyExclusions, EagerAndDeferredSameRigidResultsMatchEveryField) {
  using namespace policy_exclusion_test;
  const SharedEdge pair;
  Probe probe; probe.exclusion = pair.Exclusion();
  const auto eager = pair.Run(&probe.exclusion, 1, nullptr);
  ASSERT_EQ(eager.status, Status::CertifiedExactExclusion);
  ASSERT_EQ(eager.excluded_rigid_group, 7u);
  auto deferred = probe.Source();
  const auto lazy = pair.Run(nullptr, 0, &deferred);
  EXPECT_EQ(probe.calls, 1u);
  EXPECT_EQ(deferred.report.status, c::SelfContactTransactionStatus::Ok);
  ExpectResult(lazy, eager);
}

TEST(SelfContactPolicyExclusions, SuccessfulOrTerminalEarlierPhasesNeverPrepareExclusions) {
  using namespace policy_exclusion_test;
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  const auto second_at = [](double height) {
    return Triangle(20, {4, 5, 6},
        {{{0, 0, height}, {2, 0, height}, {0, 2, height}}});
  };
  const auto contact = second_at(.15);
  const auto geometry = DiscoverPreparedPair(first, contact);
  const auto accepted = AcceptedCertificate(FirstEdgeEdge(geometry));
  const auto check = [&](const c::CurrentFixedTriangle& second,
                         const sct::AcceptedEventCertificate* owner,
                         double duration, std::size_t work, Status expected) {
    Probe probe; auto source = probe.Source();
    const auto result = sct::CertifyQuadraticFacetPolicyCoverage(
        first, first, Quadratic(0), .1, second, second, Quadratic(0), .1,
        duration, owner, owner ? 1 : 0, nullptr, 0, work, 8, &source);
    EXPECT_EQ(result.status, expected);
    EXPECT_EQ(probe.calls, 0u);
  };
  check(second_at(2), nullptr, 1, 255, Status::CertifiedSeparated);
  check(contact, &accepted, 1, 255, Status::CertifiedAcceptedCoverage);
  check(contact, nullptr, 0, 255, Status::InvalidInput);
  check(contact, nullptr, 1, 1, Status::WorkExhausted);
  const auto local = Triangle(
      20, {1, 4, 5}, {{{0, 0, 0}, {-2, 0, 0}, {0, -2, 0}}});
  check(local, nullptr, 1, 255, Status::CertifiedLocalIntersection);
}

TEST(SelfContactPolicyExclusions, UnresolvedWithoutExclusionsCallsOnceAndPreservesLedger) {
  using namespace policy_exclusion_test;
  const SharedEdge pair;
  Probe probe; probe.count = 0;
  auto source = probe.Source();
  source.capacity = 0;
  const auto eager = pair.Run(nullptr, 0, nullptr);
  ASSERT_EQ(eager.status, Status::MissingAcceptedOwner);
  const auto deferred = pair.Run(nullptr, 0, &source);
  EXPECT_EQ(probe.calls, 1u);
  EXPECT_EQ(source.report.status, c::SelfContactTransactionStatus::Ok);
  ExpectResult(deferred, eager);
}

TEST(SelfContactPolicyExclusions, CallbackFailureRetainsTypedDiagnostic) {
  using namespace policy_exclusion_test;
  const SharedEdge pair;
  Probe probe;
  probe.failure.status = c::SelfContactTransactionStatus::ResourceLimit;
  probe.failure.candidate = 17;
  probe.failure.pair = 23;
  probe.failure.discovery_task = 6;
  probe.failure.discovery_status = c::FixedTriangleDiscoveryStatus::InvalidInput;
  probe.failure.crossing_reason = c::RepresentedIntervalReason::WorkExhausted;
  probe.failure.nonlinear_subdivision_work = 91;
  probe.failure.nonlinear_subdivision_depth = 4;
  probe.failure.message = "fixture exclusion preparation failure";
  auto source = probe.Source();
  const auto result = pair.Run(nullptr, 0, &source);
  EXPECT_EQ(result.status, Status::InvalidInput);
  EXPECT_EQ(probe.calls, 1u);
  EXPECT_EQ(source.report.status, probe.failure.status);
  EXPECT_EQ(source.report.candidate, probe.failure.candidate);
  EXPECT_EQ(source.report.pair, probe.failure.pair);
  EXPECT_EQ(source.report.discovery_task, probe.failure.discovery_task);
  EXPECT_EQ(source.report.discovery_status, probe.failure.discovery_status);
  EXPECT_EQ(source.report.crossing_reason, probe.failure.crossing_reason);
  EXPECT_EQ(source.report.nonlinear_subdivision_work, probe.failure.nonlinear_subdivision_work);
  EXPECT_EQ(source.report.nonlinear_subdivision_depth, probe.failure.nonlinear_subdivision_depth);
  EXPECT_STREQ(source.report.message, probe.failure.message);
}

TEST(SelfContactPolicyExclusions, InvalidDescriptorsAndMixedSourcesRejectBeforeCallback) {
  using namespace policy_exclusion_test;
  const SharedEdge pair;
  Probe probe; probe.exclusion = pair.Exclusion();
  for (unsigned mutation = 0; mutation < 5; ++mutation) {
    SCOPED_TRACE(mutation);
    auto source = probe.Source();
    if (mutation == 0) source.context = nullptr;
    if (mutation == 1) source.prepare = nullptr;
    if (mutation == 2) source.capacity = 65;
    const auto* eager = mutation >= 3 ? &probe.exclusion : nullptr;
    const auto count = mutation == 4 ? 1u : 0u;
    EXPECT_EQ(pair.Run(eager, count, &source).status, Status::InvalidInput);
    EXPECT_EQ(source.report.status, c::SelfContactTransactionStatus::InvalidInput);
    EXPECT_EQ(probe.calls, 0u);
  }
}

TEST(SelfContactPolicyExclusions, OversizedPreparedCountFailsBeforeCertificateConsumption) {
  using namespace policy_exclusion_test;
  const SharedEdge pair;
  Probe probe; probe.exclusion = pair.Exclusion(); probe.count = 2;
  auto source = probe.Source();
  const auto result = pair.Run(nullptr, 0, &source);
  EXPECT_EQ(probe.calls, 1u);
  EXPECT_EQ(result.status, Status::InvalidInput);
  EXPECT_EQ(source.report.status, c::SelfContactTransactionStatus::ResourceLimit);
  EXPECT_STREQ(source.report.message, "Deferred policy exclusion count exceeds its bounded storage");
}
