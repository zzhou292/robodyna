// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "Oracle.h"
#include "ResultAssertions.h"
#include "lib_src/collision/represented_interval_crossing/FixedPolicyQualification.h"
#include "lib_src/collision/represented_interval_crossing/native/CellKernel.h"
#include "lib_src/collision/represented_interval_crossing/native/FixedIntegerPolicy.h"
#if defined(__x86_64__) || defined(__i386__)
#include <xmmintrin.h>
#endif

namespace represented_interval_test {
namespace fixed_policy_test {
namespace numeric = ct::represented_interval_crossing::native;
namespace policy = ct::represented_interval_crossing;
using Paths = std::array<ct::RepresentedTrianglePath, 2>;
using Fixed = numeric::CellKernel<512, numeric::FixedIntegerPolicy<512>>;
using Boost = numeric::CellKernel<512>;

std::array<ct::Vec3, 3> Positive(double z = 2) {
  auto points = BaseTriangle(z);
  for (auto& point : points) { point.x += 2; point.y += 2; }
  return points;
}
policy::FixedPolicyComparison Compare(const Paths& paths, ct::RepresentedIntervalLimits limits = {}) {
  const auto result = policy::CompareFixedIntegerPolicy(paths[0], paths[1], limits);
  EXPECT_EQ(result.status, S::Ok);
  SameResult(result.current, result.original);
  EXPECT_EQ(result.fixed_executed, result.domain.eligible);
  EXPECT_FALSE(result.arithmetic_failed);
  EXPECT_LE(result.fixed_scratch_bytes, 8192u);
  return result;
}
template <class Kernel>
ct::RepresentedIntervalResult Direct(Kernel& kernel, typename Kernel::ExactScratch& scratch,
    const Paths& paths, ct::RepresentedIntervalLimits limits = {},
    numeric::NormalCounters* counters = nullptr) {
  // Internal numerical fault/instance testing only. Production fixed admission
  // is exercised separately by CompareFixedIntegerPolicy above.
  numeric::Cell dfs[53];
  return kernel.CertifyPair(paths[0], paths[1], limits,
      {{paths[0].key, paths[1].key}}, dfs, limits.max_depth + 1, &scratch, counters);
}
}  // namespace fixed_policy_test

TEST(RepresentedFixedPolicy, AdmittedStaticAndMovingGeometryPreservesAllWideFields) {
  using namespace fixed_policy_test;
  const auto first = Static(10, Positive(), 100);
  for (const auto& second : {Static(20, Positive(), 1), Static(20, Positive(3)),
      Static(20, {{{2,2,2},{1,2,3},{2,1,1}}}),
      Path(20, Positive(3), Positive(1)), Path(20, Positive(3), Positive(1.5))}) {
    for (unsigned depth : {0u, 20u, 52u}) for (std::size_t work : {1u, 17u}) {
      ct::RepresentedIntervalLimits limits; limits.max_depth = depth; limits.max_work_per_pair = work;
      const auto result = Compare({first, second}, limits);
      ASSERT_TRUE(result.fixed_executed);
      if (result.current.classification == C::CertifiedCrossingContact)
        EXPECT_TRUE(ExactOracleAt(first, second, result.current.witness_time_numerator,
            std::uint64_t{1} << result.current.witness_time_depth).intersects);
    }
  }
}

TEST(RepresentedFixedPolicy, EdgeOnlyWitnessAndDegeneracyRetainTheOriginalPriority) {
  using namespace fixed_policy_test;
  const auto first = Static(10, {{{4,6,2},{2,3,2},{6,3,2}}});
  const auto second = Static(20, {{{4,2,2},{2,5,2},{6,5,2}}});
  const auto edge = Compare({first, second});
  ASSERT_TRUE(edge.fixed_executed);
  EXPECT_EQ(edge.current.feature.kind, K::EdgeEdge);
  const auto oracle = ExactOracleAt(first, second, 0, 1);
  EXPECT_TRUE(oracle.valid && oracle.intersects && oracle.edge_edge && !oracle.vertex_face);
  const std::array<ct::Vec3, 3> collapsed{{{2,2,2},{2,2,2},{2,2,2}}};
  for (const auto& path : {Static(20, collapsed), Path(20, Positive(), collapsed),
                           Path(20, collapsed, Positive())})
    Compare({Static(10, Positive()), path});
}

TEST(RepresentedFixedPolicy, DomainBoundaryAndUnsupportedOrWideInputsRouteBeforeFixedExecution) {
  using namespace fixed_policy_test;
  for (unsigned depth : {0u, 20u, 52u}) for (unsigned bound : {125u, 126u}) {
    auto points = Positive(); points[0].z = std::ldexp(1., 55 + int(depth + 1) - int(bound));
    ct::RepresentedIntervalLimits limits; limits.max_depth = depth;
    const auto result = Compare({Static(10, points), Static(20, points)}, limits);
    EXPECT_EQ(result.domain.projection.coordinate_bits, bound);
    EXPECT_EQ(result.fixed_executed, bound == 125);
  }
  const auto mixed = MixedExponentPaths();
  EXPECT_FALSE(Compare({mixed[0], mixed[1]}).fixed_executed);
  EXPECT_TRUE(Compare({Static(10, BaseTriangle()), Static(20, BaseTriangle())}).fixed_executed);
  auto genuine_wide = Positive();
  genuine_wide[0].z = std::ldexp(1., -900);
  EXPECT_FALSE(Compare({Static(10, genuine_wide), Static(20, Positive())}).fixed_executed);
  auto unsupported = Static(20, Positive()); unsupported.motion = ct::RepresentedMotion::RigidArc;
  const auto result = Compare({Static(10, Positive()), unsupported});
  EXPECT_FALSE(result.fixed_executed);
  EXPECT_EQ(result.current.reason, R::UnsupportedMotion);
}

TEST(RepresentedFixedPolicy, ScalesPermutationsAndSignedCoordinatesPreserveCanonicalResults) {
  using namespace fixed_policy_test;
  for (int exponent : {-1070, -300, 0, 700, 1020}) {
    auto points = Positive();
    for (auto& point : points) {
      point.x = std::ldexp(point.x, exponent);
      point.y = std::ldexp(-point.y, exponent);
      point.z = std::ldexp(point.z, exponent);
    }
    const Paths paths{Static(10, points, 100), Static(20, points, 1)};
    const auto expected = Compare(paths);
    ASSERT_TRUE(expected.fixed_executed);
    const std::array<std::array<unsigned, 3>, 6> orders{{
        {{0,1,2}},{{0,2,1}},{{1,0,2}},{{1,2,0}},{{2,0,1}},{{2,1,0}}}};
    for (const auto& a : orders) for (const auto& b : orders)
      SameResult(Compare({Permute(paths[1], b), Permute(paths[0], a)}).current, expected.original);
  }
}

TEST(RepresentedFixedPolicy, StickyErrorSurvivesSignAndZeroAssignmentAndOnlyPairEntryResetsIt) {
  using namespace fixed_policy_test;
  numeric::ArithmeticContext context;
  numeric::FixedIntegerPolicy<512> integers(context);
  using Core = numeric::FixedIntegerPolicy<512>::Core;
  auto value = Core::ShiftLeft(Core::FromU64(1), 512);
  ASSERT_TRUE(value.overflow);
  EXPECT_EQ(integers.Sign(value), 0);
  EXPECT_FALSE(context.valid());
  integers.Assign(value, 0);
  EXPECT_TRUE(integers.IsZero(value));
  EXPECT_FALSE(context.valid());
  Fixed kernel(context); Fixed::ExactScratch scratch;
  const Paths paths{Static(10, Positive()), Static(20, Positive())};
  const auto result = Direct(kernel, scratch, paths);
  EXPECT_TRUE(context.valid());
  // This coupon calls the numerical kernel directly. Compare against the raw
  // Boost result too; StoreResult separately normalizes unused publication fields.
  numeric::ArithmeticContext boost_context;
  Boost boost(boost_context); Boost::ExactScratch boost_scratch;
  SameResult(result, Direct(boost, boost_scratch, paths));
}

TEST(RepresentedFixedPolicy, OverflowBoundariesMatchCheckedBoostWorkZeroAndWorkOne) {
  using namespace fixed_policy_test;
  // Deliberately bypass production admission only in this numeric policy test.
  // Both checked512 backends must fail closed; the public adapter routes these
  // actual out-of-domain paths to the unchanged16384 CPU implementation.
  for (bool moving : {false, true}) {
    auto points = Positive(); points[1].z = std::ldexp(1., moving ? -400 : -900);
    const Paths paths{Static(10, points), moving ? Path(20, Positive(3), Positive(1))
                                               : Static(20, Positive(3))};
    ASSERT_FALSE(Compare(paths).domain.eligible);
    numeric::ArithmeticContext boost_context, fixed_context;
    Boost boost(boost_context); Fixed fixed(fixed_context);
    Boost::ExactScratch boost_scratch; Fixed::ExactScratch fixed_scratch;
    const auto original = Direct(boost, boost_scratch, paths);
    const auto result = Direct(fixed, fixed_scratch, paths);
    SameResult(result, original);
    EXPECT_FALSE(boost_context.valid()); EXPECT_FALSE(fixed_context.valid());
    EXPECT_EQ(result.classification, C::Unresolved);
    EXPECT_EQ(result.reason, R::ExactArithmeticRange);
    EXPECT_EQ(result.work, moving ? 1u : 0u);
    if (!moving) EXPECT_FALSE(fixed_scratch.ready_a[0]);
    // Reuse both instances/scratch on a valid pair after failure.
    const Paths retry{Static(10, Positive()), Static(20, Positive())};
    SameResult(Direct(fixed, fixed_scratch, retry), Direct(boost, boost_scratch, retry));
    EXPECT_TRUE(boost_context.valid()); EXPECT_TRUE(fixed_context.valid());
  }
}

TEST(RepresentedFixedPolicy, CommonTranslationOverflowStopsBeforeAnyCellOrWorkAdmission) {
  using namespace fixed_policy_test;
  auto start = Positive(); start[0].x = std::ldexp(1., -900);
  const Paths paths{Path(10, start, Positive()), Static(20, Positive(3))};
  ASSERT_FALSE(Compare(paths).domain.eligible);
  numeric::ArithmeticContext boost_context, fixed_context;
  Boost boost(boost_context); Fixed fixed(fixed_context);
  Boost::ExactScratch boost_scratch; Fixed::ExactScratch fixed_scratch;
  numeric::NormalCounters original_counts, current_counts;
  const auto original = Direct(boost, boost_scratch, paths, {}, &original_counts);
  const auto current = Direct(fixed, fixed_scratch, paths, {}, &current_counts);
  SameResult(current, original);
  EXPECT_EQ(current.classification, C::Unresolved);
  EXPECT_EQ(current.reason, R::ExactArithmeticRange);
  EXPECT_EQ(current.work, 0u);
  EXPECT_EQ(original_counts.evaluated_cells, 0u);
  EXPECT_EQ(current_counts.evaluated_cells, 0u);
  EXPECT_FALSE(boost_context.valid()); EXPECT_FALSE(fixed_context.valid());
}

TEST(RepresentedFixedPolicy, DazFtzDoesNotAlterIntegerPolicyOrItsHostEnvironment) {
#if defined(__x86_64__) || defined(__i386__)
  using namespace fixed_policy_test;
  auto points = Positive();
  for (auto& point : points) {
    point.x = std::ldexp(point.x, -1070); point.y = std::ldexp(point.y, -1070);
    point.z = std::ldexp(point.z, -1070);
  }
  const Paths paths{Static(10, points), Static(20, points)};
  const auto saved = _mm_getcsr();
  for (unsigned mode : {1u << 6, 1u << 15, (1u << 6) | (1u << 15)}) {
    _mm_setcsr(saved | mode);
    const auto result = policy::CompareFixedIntegerPolicy(paths[0], paths[1], {});
    const auto observed = _mm_getcsr(); _mm_setcsr(saved);
    ASSERT_EQ(result.status, S::Ok); ASSERT_TRUE(result.fixed_executed);
    EXPECT_FALSE(result.arithmetic_failed); SameResult(result.current, result.original);
    EXPECT_EQ(observed & ~0x3fu, (saved | mode) & ~0x3fu);
  }
#else
  GTEST_SKIP() << "FTZ/DAZ qualification requires MXCSR";
#endif
}

TEST(RepresentedFixedPolicy, MalformedIdentityCoordinatesAndLimitsNeverReachTheFixedBackend) {
  using namespace fixed_policy_test;
  const auto a = Static(10, Positive()); auto b = Static(20, Positive());
  b.vertices[2].endpoint[1].z = NAN;
  const auto invalid = policy::CompareFixedIntegerPolicy(a, b, {});
  EXPECT_EQ(invalid.status, S::InvalidInput); EXPECT_FALSE(invalid.fixed_executed);
  b = Static(20, Positive()); b.vertices[0].key = a.vertices[0].key;
  b.vertices[0].endpoint[1].x += 1; b = Permute(b, {0,1,2});
  const auto inconsistent = policy::CompareFixedIntegerPolicy(a, b, {});
  EXPECT_EQ(inconsistent.status, S::IdentityMismatch); EXPECT_FALSE(inconsistent.fixed_executed);
  ct::RepresentedIntervalLimits limits; limits.max_depth = 53;
  EXPECT_EQ(policy::CompareFixedIntegerPolicy(a, Static(20, Positive()), limits).status, S::InvalidInput);
}
}  // namespace represented_interval_test
