#include "Fixture.h"
#include "lib_utils/BoundedArena.h"
namespace wall_interval_test {
TEST(MappedWallIntervalHost, RoundedLeavesAndNewCertificatesEncloseIndependentTruthAcrossGrains) {
  for (unsigned count : {1u, 127u, 128u, 129u, 263u, 33001u}) {
    SCOPED_TRACE(count);
    Fixture f(count);
    CheckEndpointEnclosure(f);
    for (unsigned seeds = 0; seeds < 2; ++seeds) {
      f.view.kick_dt = seeds ? .0005 : .001;
      f.view.proposed_time = .001; // Drift geometry spans the full interval in both cases.
      f.seed.kick_work = seeds ? .125 : 0;
      f.seed.drift_work = seeds ? -.25 : 0;
      auto next = f.seed;
      ASSERT_TRUE(m::ApplyInterval(f.Reduce(), f.Storage(), f.view, next));
      ASSERT_TRUE(f.Staged());
      CheckRoundedWork(f, f.Storage().result.diagnostics);
      CheckPhysicalTruth(f.Storage(), f.view);
      const auto first = f.Storage().result.diagnostics;
      ASSERT_TRUE(f.Staged());
      ExactDiagnostics(first, f.Storage().result.diagnostics);
      EXPECT_EQ(first.owner_id, f.seed.owner_id);
      EXPECT_EQ(first.attempt, f.seed.attempt);
    }
  }
}
TEST(MappedWallIntervalHost, SerialEndpointEnclosureRejectsAmbiguousCancellationAndScaledOverflow) {
  const double tiny = std::numeric_limits<double>::denorm_min();
  for (double tail : {0., tiny, ::nextafter(tiny, INFINITY), ::nextafter(DBL_MIN, 0.), DBL_MIN}) {
    Fixture f(3);
    const double moments[]{1, -1, tail};
    for (unsigned i = 0; i < 3; ++i) {
      f.base.nodes[i].force = {1, 1, 1, 0};
      f.base.nodes[i].force_world = {};
      f.base.nodes[i].wall_point = {0, 0, moments[i]};
    }
    f.view.kick_dt = .5;
    CheckEndpointEnclosure(f);
    FallbackMatches(f);
  }
  Fixture large(3);
  for (auto& n : large.base.nodes) { n.force_world = {}; n.wall_point.z = 1e200; }
  large.view.kick_dt = 1e200;
  FallbackMatches(large);
  EXPECT_EQ(large.Storage().control.status, Code::NonFiniteArithmetic);
  // All exactly-zero moment leaves are a proved zero serial sum, even with a
  // subnormal positive kick duration. There is no cancellation to infer.
  Fixture zero(1);
  zero.base.nodes[0].wall_point = {};
  zero.base.nodes[0].force_world = {};
  zero.base_velocity.assign(3, 0);
  zero.velocity.assign(3, 0);
  zero.addition[0] = 0;
  zero.Storage().base.diagnostics.wall_moment = {};
  zero.Storage().base.diagnostics.wall_reaction = {};
  zero.Storage().base.diagnostics.resultant = {};
  zero.view.kick_dt = tiny;
  auto out = zero.seed;
  ASSERT_TRUE(m::ApplyInterval(zero.Reduce(), zero.Storage(), zero.view, out));
  ASSERT_TRUE(zero.Staged());
}
TEST(MappedWallIntervalHost, PostLoopSeedsAndUnusualLegacyDurationsReplayWithoutNewAdmission) {
  for (unsigned fault = 0; fault < 10; ++fault) {
    SCOPED_TRACE(fault);
    Fixture f;
    if (fault == 0) f.seed.kick_work = DBL_MAX/4;
    if (fault == 1) f.seed.drift_work = NAN;
    if (fault == 2) f.seed.potential.error = -1;
    if (fault == 3) f.Storage().base.diagnostics.potential.error = -1;
    if (fault == 4) f.Storage().base.diagnostics.wall_moment.y = DBL_MAX/4;
    if (fault == 5) f.Storage().base.diagnostics.resultant.upper = DBL_MAX;
    if (fault == 6) f.seed.potential.value = DBL_MAX/4;
    if (fault == 7) f.view.kick_dt = -.001;
    if (fault == 8) f.view.kick_dt = NAN;
    if (fault == 9) f.Storage().base.diagnostics.wall_reaction.x = DBL_MAX/4;
    FallbackMatches(f);
  }
  for (double duration : {0., -0.}) {
    Fixture zero;
    zero.view.kick_dt = duration;
    ASSERT_TRUE(zero.Staged());
    CheckPhysicalTruth(zero.Storage(), zero.view);
    check::Exact(zero.view.kick_dt, duration);
  }
}
TEST(MappedWallIntervalHost, ConsumedOnlyReadSetAndFirstCompactFailureKeepPartialDiagnostics) {
  Fixture f;
  for (unsigned i = 0; i < f.base.nodes.size(); ++i) {
    f.base.positions[3*i+1] = NAN;
    f.velocity[3*i+2] = NAN;
    f.current.nodes[i].force_world = {NAN, NAN, NAN};
  }
  // Current forces, unused xyz components and absent rotation arrays must not
  // become interval inputs. Base force and x kinematics remain authoritative.
  ASSERT_TRUE(f.Staged());
  CheckPhysicalTruth(f.Storage(), f.view);
  f.base.nodes[7].force_world.x = 0;
  f.velocity[3*f.base.nodes[7].node] = NAN;
  f.velocity[3*f.base.nodes[200].node] = INFINITY;
  f.seed.wall_kick_moment_error = {7, 8, 9};
  f.seed.quadratic_work_upper = 11;
  FallbackMatches(f);
  EXPECT_EQ(f.Storage().control.status, Code::NonFiniteArithmetic);
  EXPECT_EQ(f.Storage().control.node, f.base.nodes[7].node);
  EXPECT_EQ(f.Storage().result.diagnostics.wall_kick_moment_error.x, 7);
  f.velocity[3*f.base.nodes[7].node] = .001;
  f.velocity[3*f.base.nodes[200].node] = .001;
  ASSERT_TRUE(f.Staged());
}
TEST(MappedWallIntervalHost, SeparateTypedTailMetadataExactCapAndLateRetry) {
  EXPECT_EQ(sizeof(m::Layout), 10*sizeof(tl::util::ArenaRegion)+sizeof(std::size_t));
  EXPECT_EQ(sizeof(m::Sidecar), 10*sizeof(void*)+sizeof(std::size_t));
  m::Layout layout;
  ASSERT_TRUE(m::MakeLayout(7, 263, 2, SIZE_MAX, layout));
  EXPECT_EQ(layout.interval.count, 3u);
  EXPECT_EQ(layout.interval.bytes, 3*sizeof(m::IntervalSummary));
  EXPECT_GE(layout.interval.offset, layout.observer.offset+layout.observer.bytes);
  EXPECT_EQ(layout.bytes, layout.interval.offset+layout.interval.bytes);
  const auto good = layout;
  EXPECT_FALSE(m::MakeLayout(7, 263, 2, good.bytes-1, layout));
  EXPECT_EQ(layout.bytes, good.bytes);
  EXPECT_EQ(layout.interval.offset, good.interval.offset);
  ASSERT_TRUE(m::MakeLayout(7, 263, 2, good.bytes, layout));
  std::vector<std::max_align_t> arena((layout.bytes+sizeof(std::max_align_t)-1)/sizeof(std::max_align_t));
  const auto side = m::Bind(arena.data(), layout);
  EXPECT_EQ(reinterpret_cast<unsigned char*>(side.interval)-reinterpret_cast<unsigned char*>(arena.data()),
      static_cast<std::ptrdiff_t>(layout.interval.offset));
  ASSERT_TRUE(m::MakeLayout(c::MaxVehicleNodalWallDeviceParents, c::MaxVehicleNodalWallDeviceNodes,
      1024, SIZE_MAX, layout));
  EXPECT_EQ(layout.interval.bytes, 32768u);
  EXPECT_EQ(layout.interval.count, 256u);
}
} // namespace wall_interval_test
