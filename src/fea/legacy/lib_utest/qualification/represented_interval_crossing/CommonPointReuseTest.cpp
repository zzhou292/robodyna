// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "Oracle.h"
#include "ResultAssertions.h"
#include "lib_src/collision/represented_interval_crossing/CommonPointReuseQualification.h"
#if defined(__x86_64__) || defined(__i386__)
#include <xmmintrin.h>
#endif

namespace represented_interval_test {
namespace common = ct::represented_interval_crossing;
using CommonPaths = std::array<ct::RepresentedTrianglePath, 2>;
common::CommonPointReuseComparison CompareCommon(
    const CommonPaths& paths, ct::RepresentedIntervalLimits limits = {}) {
  const auto result = common::CompareCommonPointReuse(paths[0], paths[1], limits);
  EXPECT_EQ(result.status, S::Ok);
  SameResult(result.current.result, result.original.result);
  EXPECT_FALSE(result.original.counters.saturated || result.current.counters.saturated);
  EXPECT_EQ(result.original.counters.endpoint_queries, 0u);
  EXPECT_EQ(result.original.counters.static_sat_bypasses, 0u);
  EXPECT_LE(result.current.counters.point_comparisons, 9 * result.current.counters.endpoint_queries);
  return result;
}

TEST(RepresentedCommonPointReuse, DistinctSourceCoincidenceKeepsCoplanarityAndCanonicalWitness) {
  const auto a = Static(10, BaseTriangle(), 100);
  const std::array<ct::Vec3, 3> transverse{{{0, 0, 0}, {-2, 0, 1}, {0, -2, -1}}};
  for (const auto& points : {BaseTriangle(), transverse}) {
    const auto b = Static(20, points, 1);
    const auto compared = CompareCommon({a, b});
    ASSERT_TRUE(compared.domain.eligible);
    EXPECT_EQ(compared.current.counters.static_sat_bypasses, 1u);
    EXPECT_EQ(compared.current.result.work, 1u);
    EXPECT_EQ(compared.current.result.witness_time_numerator, 0u);
    EXPECT_EQ(compared.current.result.witness_time_depth, 0u);
    const auto exact = ExactOracleAt(a, b, 0, 1);
    EXPECT_TRUE(exact.valid && exact.intersects && exact.vertex_face);
    EXPECT_EQ(ct::BaseIntersectionGeometry(compared.current.result.geometry),
              exact.coplanar ? G::Coplanar : G::Transverse);
    auto owner = Owner(); SameResult(One(owner, {a, b}), compared.original.result);
  }
  const auto coincident = CompareCommon({a, Static(20, BaseTriangle(), 1)});
  EXPECT_EQ(coincident.current.result.feature.vertex.first, 1u);
  EXPECT_EQ(coincident.current.result.feature.face.parent_eid, 10u);
}

TEST(RepresentedCommonPointReuse, SourceSharedAndSignedZeroPointsUseNumericEquality) {
  const auto a = Static(10, BaseTriangle());
  auto b = Static(20, {{{0, 0, 0}, {-2, 0, 1}, {0, -2, -1}}});
  b.vertices[0].key = a.vertices[0].key;
  b = Permute(b, {0, 1, 2});
  EXPECT_EQ(CompareCommon({a, b}).current.counters.static_sat_bypasses, 1u);
  auto signed_points = BaseTriangle();
  for (auto& point : signed_points) if (point.z == 0) point.z = -0.;
  const auto signed_compared = CompareCommon({a, Static(20, signed_points)});
  EXPECT_EQ(signed_compared.current.counters.static_sat_bypasses, 1u);
  // Contradictory shared source trajectories still fail before the shortcut.
  b.vertices[0].endpoint[1].x = 1;
  const auto rejected = common::CompareCommonPointReuse(a, b, {});
  EXPECT_EQ(rejected.status, S::IdentityMismatch);
  EXPECT_EQ(rejected.current.counters.endpoint_queries, 0u);
}

TEST(RepresentedCommonPointReuse, NearAndCrossTimeCoincidenceCannotBypassStaticSeparation) {
  const auto first = Static(10, BaseTriangle());
  const auto nearby = Static(20, BaseTriangle(std::numeric_limits<double>::denorm_min()));
  const auto separated = CompareCommon({first, nearby});
  EXPECT_EQ(separated.current.result.classification, C::CertifiedSeparated);
  EXPECT_EQ(separated.current.counters.static_sat_bypasses, 0u);
  // The first lower endpoint equals the second upper endpoint, at different
  // times. Both triangles share velocity and remain separated everywhere.
  const auto cross_time = CompareCommon({Path(10, BaseTriangle(), BaseTriangle(1)),
                                        Path(20, BaseTriangle(-1), BaseTriangle())});
  EXPECT_EQ(cross_time.current.result.classification, C::CertifiedSeparated);
  EXPECT_EQ(cross_time.current.counters.static_sat_bypasses, 0u);
  const std::array<ct::Vec3, 3> a{{{0, 2, 0}, {-2, -1, 0}, {2, -1, 0}}};
  const std::array<ct::Vec3, 3> b{{{0, -2, 0}, {-2, 1, 0}, {2, 1, 0}}};
  const auto edges = CompareCommon({Static(10, a), Static(20, b)});
  EXPECT_EQ(edges.current.result.feature.kind, K::EdgeEdge);
  EXPECT_EQ(edges.current.counters.static_sat_bypasses, 0u);
}

TEST(RepresentedCommonPointReuse, AmbientDazAndFtzCannotTurnSubnormalGapsIntoCommonPoints) {
#if defined(__x86_64__) || defined(__i386__)
  // Build the exact coordinate bits before changing the environment. No test
  // framework or floating arithmetic is invoked until the original CSR returns.
  const auto first = Static(10, BaseTriangle());
  const auto second = Static(20, BaseTriangle(std::numeric_limits<double>::denorm_min()));
  auto zeros = BaseTriangle(); for (auto& point : zeros) point.z = -0.;
  const auto signed_zero = Static(20, zeros);
  const auto saved = _mm_getcsr();
  for (unsigned mode : {1u << 6, 1u << 15, (1u << 6) | (1u << 15)}) {
    _mm_setcsr(saved | mode);
    const auto gap = common::CompareCommonPointReuse(first, second, {});
    const auto touch = common::CompareCommonPointReuse(first, signed_zero, {});
    const auto observed = _mm_getcsr();
    _mm_setcsr(saved);
    // Legacy finite-input validation may raise sticky status flags; only
    // control bits are promised unchanged by the existing native API.
    EXPECT_EQ(observed & ~0x3fu, (saved | mode) & ~0x3fu);
    ASSERT_EQ(gap.status, S::Ok); ASSERT_TRUE(gap.domain.eligible);
    SameResult(gap.current.result, gap.original.result);
    EXPECT_EQ(gap.current.result.classification, C::CertifiedSeparated);
    EXPECT_EQ(gap.current.counters.static_sat_bypasses, 0u);
    ASSERT_EQ(touch.status, S::Ok);
    SameResult(touch.current.result, touch.original.result);
    EXPECT_EQ(touch.current.counters.static_sat_bypasses, 1u);
  }
#else
  GTEST_SKIP() << "FTZ/DAZ qualification requires an x86 MXCSR";
#endif
}

TEST(RepresentedCommonPointReuse, EndpointAndInteriorSamplesRetainOriginalWitnessOrder) {
  const auto first = Static(10, BaseTriangle());
  ct::RepresentedIntervalLimits limits; limits.max_depth = 0; limits.max_work_per_pair = 1;
  const auto end = CompareCommon({first, Path(20, BaseTriangle(1), BaseTriangle())}, limits);
  EXPECT_EQ(end.current.counters.static_sat_bypasses, 1u);
  EXPECT_EQ(end.current.result.witness_time_numerator, 1u);
  EXPECT_EQ(end.current.result.witness_time_depth, 0u);
  const auto middle = CompareCommon({first, Path(20, BaseTriangle(1), BaseTriangle(-1))}, limits);
  EXPECT_EQ(middle.current.counters.static_sat_bypasses, 0u);
  EXPECT_EQ(middle.current.result.witness_time_numerator, 1u);
  EXPECT_EQ(middle.current.result.witness_time_depth, 1u);
  for (unsigned depth : {0u, 20u, 52u}) {
    limits.max_depth = depth;
    const auto no_sample = CompareCommon({first, Path(20, BaseTriangle(1), BaseTriangle(-2))}, limits);
    EXPECT_EQ(no_sample.current.result.classification, C::Unresolved);
    EXPECT_EQ(no_sample.current.result.reason, R::WorkExhausted);
    EXPECT_EQ(no_sample.current.counters.static_sat_bypasses, 0u);
  }
}

TEST(RepresentedCommonPointReuse, DegeneracyAndLaterCollapsedSamplesKeepOriginalPriority) {
  const std::array<ct::Vec3, 3> collapsed{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
  const auto first = Static(10, BaseTriangle());
  const auto later = CompareCommon({first, Path(20, BaseTriangle(), collapsed)});
  EXPECT_EQ(later.current.counters.static_sat_bypasses, 1u);
  EXPECT_EQ(later.current.result.witness_time_numerator, 0u);
  const auto initial = CompareCommon({first, Path(20, collapsed, BaseTriangle())});
  EXPECT_EQ(initial.current.counters.static_sat_bypasses, 0u);
  EXPECT_EQ(initial.current.result.witness_time_depth, 1u);
  const auto all = CompareCommon({first, Static(20, collapsed)});
  EXPECT_EQ(all.current.result.classification, C::Unresolved);
  EXPECT_EQ(all.current.result.reason, R::DegenerateGeometry);
  EXPECT_EQ(all.current.counters.static_sat_bypasses, 0u);
}

TEST(RepresentedCommonPointReuse, ArithmeticDomainBoundaryAndWideExtremesKeepOriginalTraversal) {
  ct::RepresentedIntervalLimits limits; limits.max_depth = 20;
  const auto settings = CompareCommon({Static(10, BaseTriangle()), Static(20, BaseTriangle())}, limits).domain;
  ASSERT_TRUE(settings.supported);
  const auto bound = ((settings.karatsuba_cutoff - 1) * settings.limb_bits - 3) / 2;
  const int exponent = static_cast<int>(bound) - 1076 - static_cast<int>(limits.max_depth + 1);
  for (unsigned offset : {0u, 1u}) {
    auto vertices = BaseTriangle();
    const auto scale = std::ldexp(1., exponent + int(offset));
    for (auto& point : vertices) { point.x *= scale; point.y *= scale; }
    const auto compared = CompareCommon({Static(10, vertices), Static(20, vertices)}, limits);
    EXPECT_EQ(compared.domain.coordinate_bits, bound + offset);
    EXPECT_EQ(compared.domain.eligible, offset == 0);
    EXPECT_EQ(compared.current.counters.static_sat_bypasses, offset ? 0u : 1u);
  }
  const auto mixed = MixedExponentPaths();
  const auto wide = CompareCommon({mixed[0], mixed[1]}, limits);
  EXPECT_FALSE(wide.domain.eligible);
  EXPECT_EQ(wide.current.counters.endpoint_queries, 0u);
  auto unsupported = Static(20, BaseTriangle()); unsupported.motion = ct::RepresentedMotion::RigidArc;
  const auto arc = CompareCommon({Static(10, BaseTriangle()), unsupported});
  EXPECT_EQ(arc.current.result.reason, R::UnsupportedMotion);
  EXPECT_EQ(arc.current.counters.endpoint_queries, 0u);
}

TEST(RepresentedCommonPointReuse, VertexAndPairPermutationsRetainAllCanonicalFields) {
  const CommonPaths paths{Static(10, BaseTriangle(), 100), Static(20, BaseTriangle(), 1)};
  const auto expected = CompareCommon(paths).original.result;
  const std::array<std::array<unsigned, 3>, 6> orders{{
      {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}, {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};
  for (const auto& a : orders) for (const auto& b : orders) {
    const auto compared = CompareCommon({Permute(paths[1], b), Permute(paths[0], a)});
    SameResult(compared.current.result, expected);
    EXPECT_EQ(compared.current.counters.static_sat_bypasses, 1u);
  }
}

TEST(RepresentedCommonPointReuse, CompleteReportsPublicationWorkAdmissionAndRetryAreUnchanged) {
  const std::vector<ct::RepresentedTrianglePath> paths{Static(10, BaseTriangle()),
      Static(20, BaseTriangle()), Static(30, BaseTriangle(1))};
  const ct::RepresentedTrianglePair pairs[]{{0, 1}, {0, 2}};
  const auto first = CompareCommon({paths[0], paths[1]}).original.result;
  const auto second = CompareCommon({paths[0], paths[2]}).original.result;
  for (unsigned workers : {1u, 4u}) for (std::size_t cap : {1u, 2u}) {
    ct::RepresentedIntervalLimits limits; limits.worker_count = workers; limits.max_total_work = cap;
    auto owner = Owner(limits); SameResult(One(owner, {paths[0], paths[1]}), first);
    const auto before = owner.results();
    for (unsigned repeat = 0; repeat < 3; ++repeat) {
      const auto report = owner.Certify(paths.data(), paths.size(), pairs, 2);
      ct::RepresentedIntervalReport expected;
      expected.input_paths = paths.size(); expected.input_pairs = expected.unique_pairs = 2;
      expected.work = first.work; expected.certified_crossing_contact = 1;
      if (cap == 1) {
        expected.status = S::ResourceLimit; expected.message = "resource limit";
        expected.input_pair = 1; expected.total_work_limit = 1; expected.rejected_pair_work = second.work;
        SameNativeReport(report, expected);
        ASSERT_EQ(owner.results().data, before.data); ASSERT_EQ(owner.results().count, 1u);
        SameResult(owner.results().data[0], first);
      } else {
        expected.work += second.work; expected.certified_separated = 1;
        SameNativeReport(report, expected);
        ASSERT_EQ(owner.results().count, 2u);
        SameResult(owner.results().data[0], first); SameResult(owner.results().data[1], second);
      }
    }
    SameResult(One(owner, {paths[0], paths[1]}), first);
  }
}

TEST(RepresentedCommonPointReuse, MalformedGeometryAndLimitsNeverAcquireShortcutAuthority) {
  const auto a = Static(10, BaseTriangle());
  auto b = Static(20, BaseTriangle()); b.vertices[2].endpoint[1].z = NAN;
  const auto invalid = common::CompareCommonPointReuse(a, b, {});
  EXPECT_EQ(invalid.status, S::InvalidInput);
  EXPECT_EQ(invalid.current.counters.endpoint_queries, 0u);
  ct::RepresentedIntervalLimits limits; limits.max_depth = 53;
  EXPECT_EQ(common::CompareCommonPointReuse(a, Static(20, BaseTriangle()), limits).status, S::InvalidInput);
  limits.max_depth = 0; limits.max_work_per_pair = 0;
  EXPECT_EQ(common::CompareCommonPointReuse(a, Static(20, BaseTriangle()), limits).status, S::InvalidInput);
  auto owner = Owner(); One(owner, {a, Static(20, BaseTriangle())});
  const auto before = owner.results(); const auto saved = before.data[0];
  const ct::RepresentedTrianglePath invalid_paths[]{a, b}; const ct::RepresentedTrianglePair pair{0, 1};
  EXPECT_EQ(owner.Certify(invalid_paths, 2, &pair, 1).status, S::InvalidInput);
  ASSERT_EQ(owner.results().data, before.data); ASSERT_EQ(owner.results().count, before.count);
  SameResult(owner.results().data[0], saved);
  SameResult(One(owner, {a, Static(20, BaseTriangle())}), saved);
}
}  // namespace represented_interval_test
