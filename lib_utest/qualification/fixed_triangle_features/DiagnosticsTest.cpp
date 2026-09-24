// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "exact_storage/DiscoverySerialization.h"
#include "lib_src/collision/fixed_triangle_features/Diagnostics.h"
#include <gtest/gtest.h>
#include <array>
#include <cerrno>

namespace {
namespace c = tlfea::contact;
namespace ft = fixed_triangle_test;
namespace measure = c::fixed_triangle_features;
using Stage = c::FixedTriangleDiscoveryStage;
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
  c::diagnostic::Clock reader() { return {Read, this}; }
};
const auto& Counter(const c::FixedTriangleDiscoveryDiagnostics& value, Stage stage) {
  return value.timing.stages[static_cast<std::size_t>(stage)];
}
std::string Record(const c::FixedTriangleDiscoveryReport& report,
                   const c::FixedTriangleFeatureDiscovery& owner) {
  std::ostringstream out;
  exact_storage_discovery::Report(out, report);
  return out.str() + exact_storage_discovery::Publication(owner);
}
TEST(FixedTriangleDiagnostics, DisabledNeverReadsClockOrChangesReport) {
  Clock clock;
  c::FixedTriangleDiscoveryDiagnostics snapshot;
  c::FixedTriangleDiscoveryReport report;
  report.status = c::FixedTriangleDiscoveryStatus::OutOfRange;
  report.input_pair = 19;
  errno = EDOM;
  {
    measure::DiscoveryTiming timing(snapshot, false, report, clock.reader());
    timing.Stage(Stage::Geometry);
  }
  EXPECT_EQ(clock.calls, 0u); EXPECT_EQ(errno, EDOM);
  EXPECT_FALSE(snapshot.enabled || snapshot.finished);
  EXPECT_EQ(report.input_pair, 19u);
  EXPECT_EQ(report.status, c::FixedTriangleDiscoveryStatus::OutOfRange);
}
TEST(FixedTriangleDiagnostics, ClockFailureBackwardsAndErrnoAreDiagnosticOnly) {
  Clock clock; clock.ticks = {10, 20, 70, 60, 80, 110}; clock.fail_on = 1;
  c::FixedTriangleDiscoveryDiagnostics snapshot;
  c::FixedTriangleDiscoveryReport report;
  errno = EDOM;
  {
    measure::DiscoveryTiming timing(snapshot, true, report, clock.reader());
    timing.Stage(Stage::InputSort);
    timing.Stage(Stage::Geometry);
    report.status = c::FixedTriangleDiscoveryStatus::NonFiniteResult;
  }
  EXPECT_EQ(errno, EDOM);
  EXPECT_EQ(snapshot.timing.clock_failures, 1u);
  EXPECT_EQ(snapshot.timing.backward_samples, 1u);
  EXPECT_TRUE(snapshot.enabled && snapshot.finished);
  EXPECT_FALSE(snapshot.succeeded);
  EXPECT_EQ(Counter(snapshot, Stage::InputLedger).valid_samples, 0u);
  EXPECT_EQ(Counter(snapshot, Stage::InputSort).valid_samples, 0u);
  EXPECT_EQ(Counter(snapshot, Stage::Geometry).wall_ns, 30u);
  EXPECT_EQ(Counter(snapshot, Stage::Geometry).failures, 1u);
}
TEST(FixedTriangleDiagnostics, RepeatedScopesSaturateAndResetPerCall) {
  Clock clock; clock.ticks = {0, UINT64_MAX, 0, 9};
  c::FixedTriangleDiscoveryDiagnostics snapshot;
  c::FixedTriangleDiscoveryReport report;
  {
    measure::DiscoveryTiming timing(snapshot, true, report, clock.reader());
    timing.Stage(Stage::InputLedger);
  }
  EXPECT_TRUE(snapshot.succeeded && snapshot.timing.counter_saturated);
  EXPECT_EQ(Counter(snapshot, Stage::InputLedger).calls, 2u);
  EXPECT_EQ(Counter(snapshot, Stage::InputLedger).wall_ns, UINT64_MAX);
  EXPECT_EQ(Counter(snapshot, Stage::InputLedger).maximum_ns, UINT64_MAX);
  {
    measure::DiscoveryTiming timing(snapshot, false, report, clock.reader());
  }
  EXPECT_FALSE(snapshot.enabled || snapshot.finished || snapshot.timing.counter_saturated);
  EXPECT_EQ(Counter(snapshot, Stage::InputLedger).calls, 0u);
}
TEST(FixedTriangleDiagnostics, CompleteOutputsMatchAndFailureRetainsPublication) {
  const c::Vec3 a[3]{{0,0,0},{2,0,0},{0,2,0}};
  const c::Vec3 b[3]{{0,0,0},{-2,0,1},{0,-2,1}};
  const std::uint64_t ai[3]{1,2,3}, bi[3]{1,4,5};
  std::array<c::CurrentFixedTriangle,2> triangles{{
      ft::Triangle(100,0,a,ai),ft::Triangle(200,0,b,bi)}};
  const c::FixedTrianglePair pair{0,1}, bad_pair{0,2};
  c::FixedTriangleFeatureTaskMask mask;
  ASSERT_EQ(c::BuildFixedTriangleFeatureTaskMask(triangles[0],triangles[1],&mask),
            c::FixedTriangleDiscoveryStatus::Ok);
  for (unsigned workers : {1u,4u}) {
    auto limits = ft::Limits(1); limits.worker_count = workers;
    c::FixedTriangleFeatureDiscovery off,on;
    ft::Initialize(&off,limits);
    limits.enable_diagnostics = true; ft::Initialize(&on,limits);
    // Enablement changes no allocation or physical capacity.
    EXPECT_EQ(off.forecast().owned_host_bytes,on.forecast().owned_host_bytes);
    for (bool masked : {false,true}) {
      const auto query = [&](c::FixedTriangleFeatureDiscovery& owner) {
        return masked ? owner.DiscoverMasked(triangles.data(),2,&pair,1,&mask)
                      : owner.Discover(triangles.data(),2,&pair,1);
      };
      const auto off_report=query(off), on_report=query(on);
      ASSERT_EQ(on_report.status,c::FixedTriangleDiscoveryStatus::Ok);
      EXPECT_EQ(Record(off_report,off),Record(on_report,on));
      const auto snapshot=on.diagnostics();
      EXPECT_TRUE(snapshot.enabled && snapshot.finished && snapshot.succeeded);
      EXPECT_FALSE(off.diagnostics().enabled);
      EXPECT_EQ(Counter(snapshot,Stage::InputSort).calls,3u);
      EXPECT_EQ(Counter(snapshot,Stage::OutputSort).calls,2u);
      EXPECT_EQ(Counter(snapshot,Stage::Geometry).calls,1u);
      EXPECT_EQ(Counter(snapshot,Stage::Publication).calls,1u);
      for(const auto& counter:snapshot.timing.stages)
        EXPECT_EQ(counter.calls,counter.valid_samples);
      const auto preserved=exact_storage_discovery::Publication(on);
      const auto off_bad=off.Discover(triangles.data(),2,&bad_pair,1);
      const auto on_bad=on.Discover(triangles.data(),2,&bad_pair,1);
      EXPECT_EQ(Record(off_bad,off),Record(on_bad,on));
      EXPECT_EQ(preserved,exact_storage_discovery::Publication(on));
      const auto failed=on.diagnostics();
      EXPECT_TRUE(failed.enabled && failed.finished); EXPECT_FALSE(failed.succeeded);
      EXPECT_EQ(Counter(failed,Stage::InputLedger).failures,1u);
      EXPECT_EQ(Counter(failed,Stage::Geometry).calls,0u);
      EXPECT_EQ(Counter(failed,Stage::Publication).calls,0u);
      // The retry replaces measured prefix, without retaining failed counters.
      EXPECT_EQ(Record(query(off),off),Record(query(on),on));
      EXPECT_TRUE(on.diagnostics().succeeded);
    }
  }
}
TEST(FixedTriangleDiagnostics, EmptyCallPublishesEmptyAndUninitializedStaysAbsent) {
  c::FixedTriangleFeatureDiscovery owner;
  EXPECT_FALSE(owner.diagnostics().enabled);
  EXPECT_EQ(owner.Discover(nullptr,0,nullptr,0).status,
            c::FixedTriangleDiscoveryStatus::NotInitialized);
  EXPECT_FALSE(owner.diagnostics().finished);
  auto limits=ft::Limits(1);limits.enable_diagnostics=true;ft::Initialize(&owner,limits);
  ASSERT_EQ(owner.Discover(nullptr,0,nullptr,0).status,c::FixedTriangleDiscoveryStatus::Ok);
  const auto snapshot=owner.diagnostics();
  EXPECT_TRUE(snapshot.succeeded && snapshot.finished && snapshot.enabled);
  EXPECT_EQ(Counter(snapshot,Stage::InputLedger).calls,1u);
  EXPECT_EQ(Counter(snapshot,Stage::Publication).calls,1u);
  EXPECT_EQ(Counter(snapshot,Stage::Geometry).calls,0u);
  EXPECT_TRUE(owner.features().complete);
}
} // namespace
