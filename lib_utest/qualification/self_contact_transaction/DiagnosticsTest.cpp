// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/self_contact_transaction/Diagnostics.h"
#include "../represented_interval_crossing/Fixture.h"

#include <array>
#include <stdexcept>

namespace {
namespace c = tlfea::contact;
namespace sct = c::self_contact_transaction;
using Stage = c::SelfContactDiagnosticStage;
struct Clock {
  std::array<std::uint64_t, 16> ticks{};
  std::size_t calls = 0, fail_on = SIZE_MAX;
  static bool Read(void* context, std::uint64_t* output) noexcept {
    auto& clock = *static_cast<Clock*>(context);
    const auto index = clock.calls++;
    errno = ENOSPC;
    *output = clock.ticks[index % clock.ticks.size()];
    return index != clock.fail_on;
  }
  sct::DiagnosticClock reader() { return {Read, this}; }
};
const auto& Counter(const c::SelfContactAttemptDiagnostics& value, Stage stage) {
  return value.stages[static_cast<std::size_t>(stage)];
}
TEST(SelfContactDiagnostics, DisabledNeverReadsClockOrCountsAndPreservesExceptions) {
  Clock clock;
  c::SelfContactAttemptDiagnostics value;
  errno = EDOM;
  EXPECT_THROW({
    sct::DiagnosticAttempt attempt(value, false, 1, 2, 3, clock.reader());
    attempt.Authenticate();
    attempt.Stage(Stage::Discovery);
    attempt.Discovery({});
    attempt.Crossing({}, 0, 1);
    throw std::runtime_error("original");
  }, std::runtime_error);
  EXPECT_EQ(errno, EDOM);
  EXPECT_EQ(clock.calls, 0u);
  EXPECT_FALSE(value.enabled);
  EXPECT_FALSE(value.entered);
  EXPECT_FALSE(value.finished);
  EXPECT_EQ(value.discovery.calls, 0u);
  EXPECT_EQ(value.native_batches, 0u);
}
TEST(SelfContactDiagnostics, DisjointScopesKeepStampSuccessAndPerInstanceSnapshots) {
  Clock clock; clock.ticks = {10, 30, 40, 80, 90, 150};
  c::SelfContactAttemptDiagnostics value;
  {
    sct::DiagnosticAttempt attempt(value, true, 91, 4, 8, clock.reader());
    attempt.Authenticate();
    attempt.Stage(Stage::Discovery);
    attempt.Stage(Stage::Policy);
    attempt.Success();
  }
  EXPECT_TRUE(value.finished && value.succeeded && value.authenticated);
  EXPECT_TRUE(value.counts_complete);
  EXPECT_EQ(value.owner_id, 91u); EXPECT_EQ(value.base_epoch, 4u);
  EXPECT_EQ(value.attempt, 8u);
  EXPECT_EQ(Counter(value, Stage::Setup).wall_ns, 20u);
  EXPECT_EQ(Counter(value, Stage::Discovery).wall_ns, 40u);
  EXPECT_EQ(Counter(value, Stage::Policy).wall_ns, 60u);
  EXPECT_EQ(Counter(value, Stage::Policy).failures, 0u);
  const auto frozen = value;
  {
    sct::DiagnosticAttempt retry(value, true, 91, 4, 9, clock.reader());
    retry.Success();
  }
  EXPECT_EQ(value.attempt, 9u);
  EXPECT_EQ(value.discovery.calls, 0u);
  EXPECT_EQ(Counter(value, Stage::Discovery).calls, 0u);
  EXPECT_EQ(frozen.attempt, 8u);
  EXPECT_EQ(Counter(frozen, Stage::Discovery).wall_ns, 40u);
}
TEST(SelfContactDiagnostics, FailedAndBackwardClocksAreUnavailableAndKeepOperationErrno) {
  Clock clock; clock.ticks = {10, 20, 70, 60}; clock.fail_on = 1;
  c::SelfContactAttemptDiagnostics value;
  errno = EDOM;
  {
    sct::DiagnosticAttempt attempt(value, true, 1, 0, 1, clock.reader());
    EXPECT_EQ(errno, EDOM);
    attempt.Stage(Stage::Policy);
    errno = EAGAIN;
    attempt.Success();
  }
  EXPECT_EQ(errno, EAGAIN);
  EXPECT_TRUE(value.succeeded && value.counts_complete);
  EXPECT_EQ(value.clock_failures, 1u);
  EXPECT_EQ(value.backward_samples, 1u);
  for (const auto stage : {Stage::Setup, Stage::Policy}) {
    EXPECT_EQ(Counter(value, stage).calls, 1u);
    EXPECT_EQ(Counter(value, stage).valid_samples, 0u);
    EXPECT_EQ(Counter(value, stage).wall_ns, 0u);
  }
}
TEST(SelfContactDiagnostics, FailedScopeSurvivesUnwindAndDoesNotClaimCompleteCensus) {
  Clock clock; clock.ticks = {1, 2, 3, 4};
  c::SelfContactAttemptDiagnostics value;
  EXPECT_THROW({
    sct::DiagnosticAttempt attempt(value, true, 1, 0, 7, clock.reader());
    attempt.Authenticate();
    attempt.Stage(Stage::Policy);
    errno = EIO;
    throw std::runtime_error("mechanics");
  }, std::runtime_error);
  EXPECT_EQ(errno, EIO);
  EXPECT_TRUE(value.finished && value.authenticated);
  EXPECT_FALSE(value.succeeded || value.counts_complete);
  EXPECT_EQ(Counter(value, Stage::Setup).failures, 0u);
  EXPECT_EQ(Counter(value, Stage::Policy).failures, 1u);
}
TEST(SelfContactDiagnostics, DiscoveryAggregatesReturnedSizesAndSaturatesExplicitly) {
  Clock clock; clock.ticks = {0, UINT64_MAX, 0, 1};
  c::SelfContactAttemptDiagnostics value;
  {
    sct::DiagnosticAttempt attempt(value, true, 1, 0, 1, clock.reader());
    c::FixedTriangleDiscoveryReport report;
    report.triangle_references = 8; report.triangles = 3;
    report.raw_feature_candidates = 60; report.feature_candidates = 17;
    report.raw_intersections = 4; report.intersections = 2;
    report.potential_tasks = 60; report.local_masked_tasks = 8;
    report.exact_executed_tasks = 52;
    attempt.Discovery(report); attempt.Discovery(report);
    value.discovery.vertices = UINT64_MAX;
    report.vertices = 1;
    report.status = c::FixedTriangleDiscoveryStatus::ResourceLimit;
    attempt.Discovery(report);
    attempt.Stage(Stage::Setup); // Same slot accumulation also saturates wall_ns.
    attempt.Success();
  }
  EXPECT_EQ(value.discovery.calls, 3u);
  EXPECT_EQ(value.discovery.failures, 1u);
  EXPECT_EQ(value.discovery.triangle_references, 24u);
  EXPECT_EQ(value.discovery.raw_feature_candidates, 180u);
  EXPECT_EQ(value.discovery.feature_candidates, 51u);
  EXPECT_EQ(value.discovery.raw_intersections, 12u);
  EXPECT_EQ(value.discovery.intersections, 6u);
  EXPECT_EQ(value.discovery.vertices, UINT64_MAX);
  EXPECT_EQ(Counter(value, Stage::Setup).wall_ns, UINT64_MAX);
  EXPECT_TRUE(value.counter_saturated && value.succeeded);
  EXPECT_FALSE(value.counts_complete);
}
TEST(SelfContactDiagnostics, NativeCountsDistinguishInvokedFailedSliceFromPreflightFailure) {
  c::SelfContactAttemptDiagnostics value;
  {
    sct::DiagnosticAttempt attempt(value, true, 1, 0, 1);
    sct::CrossingBatchReport report;
    report.status = c::RepresentedIntervalStatus::ResourceLimit;
    attempt.Crossing(report, 10, 3); // No native invocation.
    report.native_called = true;
    report.completed_pairs = 6; report.completed_batches = 2;
    report.batch_offset = 6; report.prior_work = 12;
    report.native_report.work = 2;
    attempt.Crossing(report, 10, 3); // Invoked 3 + 3 + 3, final slice failed.
  }
  EXPECT_EQ(value.native_batches, 3u);
  EXPECT_EQ(value.native_submitted_pairs, 9u);
  EXPECT_EQ(value.native_work, 14u);
  EXPECT_FALSE(value.counts_complete);
}
TEST(SelfContactDiagnostics, ActualNativeBatchReportsCountExactlyOnceIncludingEmptySlice) {
  namespace native = represented_interval_test;
  const std::array<c::RepresentedTrianglePath, 4> paths{{
      native::Static(10, native::BaseTriangle()),
      native::Static(20, native::BaseTriangle(1)),
      native::Static(30, native::BaseTriangle(2)),
      native::Static(40, native::BaseTriangle(3))}};
  const c::RepresentedTrianglePair pairs[]{{0, 1}, {0, 2}, {0, 3}};
  std::array<c::RepresentedIntervalResult, 3> scratch;
  c::RepresentedIntervalLimits limits;
  limits.max_paths = 4; limits.max_input_pairs = 3; limits.max_results = 3;
  auto owner = native::Owner(limits);
  c::SelfContactAttemptDiagnostics value;
  std::size_t expected_work = 0;
  {
    sct::DiagnosticAttempt attempt(value, true, 1, 0, 1);
    const auto report = sct::CertifyCrossingBatches(
        owner, paths.data(), paths.size(), pairs, 3, 2, scratch.data(), 3);
    ASSERT_EQ(report.status, c::RepresentedIntervalStatus::Ok);
    for (const auto& result : scratch) expected_work += result.work;
    attempt.Crossing(report, 3, 2);
    const auto empty = sct::CertifyCrossingBatches(
        owner, paths.data(), paths.size(), nullptr, 0, 2, scratch.data(), 3);
    ASSERT_EQ(empty.status, c::RepresentedIntervalStatus::Ok);
    attempt.Crossing(empty, 0, 2);
    attempt.Success();
  }
  EXPECT_EQ(value.native_batches, 3u);
  EXPECT_EQ(value.native_submitted_pairs, 3u);
  EXPECT_EQ(value.native_work, expected_work);
  EXPECT_TRUE(value.counts_complete);
}
}  // namespace
