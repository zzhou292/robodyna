// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "Oracle.h"
#include "ResultAssertions.h"
#include "lib_src/collision/represented_interval_crossing/NativeStorageQualification.h"
#if defined(__x86_64__) || defined(__i386__)
#include <xmmintrin.h>
#endif

namespace represented_interval_test {
namespace storage = ct::represented_interval_crossing;
using StoragePaths = std::array<ct::RepresentedTrianglePath, 2>;
std::array<ct::Vec3, 3> PositiveTriangle(double z = 2) {
  return {{{2, 2, z}, {4, 2, z}, {2, 4, z}}};
}
storage::NativeStorageComparison CompareStorage(const StoragePaths& paths,
                                                ct::RepresentedIntervalLimits limits = {}) {
  const auto result = storage::CompareNativeStorage(paths[0], paths[1], limits);
  EXPECT_EQ(result.status, S::Ok);
  SameResult(result.current.result, result.original.result);
  EXPECT_EQ(result.original.counters.wide_pairs, 1u);
  EXPECT_EQ(result.original.counters.narrow_pairs, 0u);
  EXPECT_EQ(result.current.counters.narrow_pairs, result.domain.eligible ? 1u : 0u);
  EXPECT_EQ(result.current.counters.wide_pairs, result.domain.eligible ? 0u : 1u);
  EXPECT_FALSE(result.original.counters.saturated || result.current.counters.saturated);
  EXPECT_LE(result.narrow_scratch_bytes, 8192u);
  EXPECT_LT(result.narrow_scratch_bytes, result.wide_scratch_bytes);
  return result;
}

TEST(RepresentedNativeStorage, NarrowClosedContactSeparationAndCanonicalFeaturesMatchWide) {
  const auto first = Static(10, PositiveTriangle(), 100);
  for (const auto& second : {Static(20, PositiveTriangle(), 1),
                            Static(20, PositiveTriangle(3), 1),
                            Static(20, {{{2,2,2}, {1,2,3}, {2,1,1}}}, 1)}) {
    const auto result = CompareStorage({first, second});
    ASSERT_TRUE(result.domain.eligible);
    EXPECT_LE(result.domain.maximum_coefficient_bits, 509u);
    EXPECT_LE(result.domain.maximum_product_limbs, 8u);
    auto owner = Owner(); SameResult(One(owner, {first, second}), result.original.result);
    const auto oracle = ExactOracleAt(first, second, 0, 1);
    EXPECT_TRUE(oracle.valid);
    EXPECT_EQ(result.current.result.classification == C::CertifiedCrossingContact, oracle.intersects);
  }
}

TEST(RepresentedNativeStorage, DegreeFourEdgeWitnessAndCoplanarSatRemainExact) {
  const std::array<ct::Vec3, 3> a{{{4,6,2}, {2,3,2}, {6,3,2}}};
  const std::array<ct::Vec3, 3> b{{{4,2,2}, {2,5,2}, {6,5,2}}};
  const auto result = CompareStorage({Static(10, a), Static(20, b)});
  ASSERT_TRUE(result.domain.eligible);
  EXPECT_EQ(result.current.result.feature.kind, K::EdgeEdge);
  const auto oracle = ExactOracleAt(Static(10, a), Static(20, b), 0, 1);
  EXPECT_TRUE(oracle.valid && oracle.intersects && oracle.edge_edge && !oracle.vertex_face);
  const auto separated = CompareStorage({Static(10, PositiveTriangle()),
      Static(20, {{{3.5,3.5,2}, {5.5,3.5,2}, {3.5,5.5,2}}})});
  EXPECT_EQ(separated.current.result.classification, C::CertifiedSeparated);
}

TEST(RepresentedNativeStorage, EntireNonuniformCellPathKeepsWitnessWorkAndDegeneracyPriority) {
  const auto first = Static(10, PositiveTriangle());
  const std::array<ct::Vec3, 3> collapsed{{{2,2,2}, {2,2,2}, {2,2,2}}};
  for (const auto& second : {Path(20, PositiveTriangle(3), PositiveTriangle(1)),
                            Path(20, PositiveTriangle(3), PositiveTriangle(1.5)),
                            Path(20, PositiveTriangle(), collapsed),
                            Path(20, collapsed, PositiveTriangle()),
                            Static(20, collapsed)}) {
    for (unsigned depth : {0u, 4u, 20u, 52u}) for (std::size_t work : {1u, 17u}) {
      ct::RepresentedIntervalLimits limits; limits.max_depth = depth; limits.max_work_per_pair = work;
      const auto result = CompareStorage({first, second}, limits);
      ASSERT_TRUE(result.domain.eligible);
      if (result.current.result.classification == C::CertifiedCrossingContact) {
        const auto& value = result.current.result;
        EXPECT_TRUE(ExactOracleAt(first, second, value.witness_time_numerator,
            std::uint64_t{1} << value.witness_time_depth).intersects);
      }
    }
  }
}

TEST(RepresentedNativeStorage, ThresholdAndTimeDepthSelectStorageWithoutChangingProjectionPolicy) {
  for (unsigned depth : {0u, 20u, 52u}) for (unsigned target : {125u, 126u}) {
    // Maximum encoded exponent comes from4 (stored exponent -50). A small
    // positive nonzero z gives the requested full-path B exactly.
    const int small_exponent = 55 + int(depth + 1) - int(target);
    auto vertices = PositiveTriangle(); vertices[0].z = std::ldexp(1., small_exponent);
    ct::RepresentedIntervalLimits limits; limits.max_depth = depth;
    const auto result = CompareStorage({Static(10, vertices), Static(20, vertices)}, limits);
    EXPECT_EQ(result.domain.projection.coordinate_bits, target);
    EXPECT_TRUE(result.domain.projection.eligible);
    EXPECT_EQ(result.domain.eligible, target == 125);
    EXPECT_EQ(result.current.result.geometry, G::ExactCommonTranslationCoplanar);
    EXPECT_EQ(result.current.result.work, 1u);
  }
}

TEST(RepresentedNativeStorage, UniformExtremeScalesCancellationAndWideExponentMixturesPreserveResults) {
  for (int exponent : {-1070, -1000, -300, 0, 700, 1020}) {
    auto first = PositiveTriangle(), second = PositiveTriangle(3);
    for (auto* triangle : {&first, &second}) for (auto& point : *triangle) {
      point.x = std::ldexp(point.x, exponent);
      point.y = std::ldexp(point.y, exponent);
      point.z = std::ldexp(point.z, exponent);
    }
    const auto result = CompareStorage({Static(10, first), Static(20, second)});
    ASSERT_TRUE(result.domain.eligible);
    EXPECT_EQ(result.current.result.classification, C::CertifiedSeparated);
    const auto contact = CompareStorage({Static(10, first), Static(20, first)});
    EXPECT_EQ(contact.current.result.classification, C::CertifiedCrossingContact);
  }
  const auto mixed = MixedExponentPaths();
  const auto wide = CompareStorage({mixed[0], mixed[1]});
  EXPECT_FALSE(wide.domain.eligible);
  EXPECT_EQ(wide.current.counters.wide_pairs, 1u);
  const auto zeros = CompareStorage({Static(10, BaseTriangle()), Static(20, BaseTriangle())});
  EXPECT_FALSE(zeros.domain.eligible); // Zero exponent remains deliberately conservative.
  EXPECT_TRUE(zeros.domain.projection.eligible);
}

TEST(RepresentedNativeStorage, SourceAndVertexPermutationsRetainAllResultFields) {
  const StoragePaths paths{Static(10, PositiveTriangle(), 100), Static(20, PositiveTriangle(), 1)};
  const auto expected = CompareStorage(paths).original.result;
  const std::array<std::array<unsigned, 3>, 6> orders{{
      {{0,1,2}}, {{0,2,1}}, {{1,0,2}}, {{1,2,0}}, {{2,0,1}}, {{2,1,0}}}};
  for (const auto& first : orders) for (const auto& second : orders)
    SameResult(CompareStorage({Permute(paths[1], second), Permute(paths[0], first)}).current.result, expected);
}

TEST(RepresentedNativeStorage, DazFtzCannotChangeNarrowSubnormalResultsOrWideSignedZeros) {
#if defined(__x86_64__) || defined(__i386__)
  auto first = PositiveTriangle(), second = PositiveTriangle(3);
  for (auto* triangle : {&first, &second}) for (auto& point : *triangle) {
    point.x = std::ldexp(point.x, -1070); point.y = std::ldexp(point.y, -1070); point.z = std::ldexp(point.z, -1070);
  }
  const StoragePaths tiny{Static(10, first), Static(20, second)};
  auto zeros = BaseTriangle(); for (auto& point : zeros) point.z = -0.;
  const StoragePaths signed_zero{Static(10, BaseTriangle()), Static(20, zeros)};
  const auto saved = _mm_getcsr();
  for (unsigned mode : {1u << 6, 1u << 15, (1u << 6) | (1u << 15)}) {
    _mm_setcsr(saved | mode);
    const auto narrow = storage::CompareNativeStorage(tiny[0], tiny[1], {});
    const auto wide = storage::CompareNativeStorage(signed_zero[0], signed_zero[1], {});
    const auto observed = _mm_getcsr(); _mm_setcsr(saved);
    EXPECT_EQ(observed & ~0x3fu, (saved | mode) & ~0x3fu);
    ASSERT_EQ(narrow.status, S::Ok); EXPECT_TRUE(narrow.domain.eligible);
    SameResult(narrow.current.result, narrow.original.result);
    ASSERT_EQ(wide.status, S::Ok); EXPECT_FALSE(wide.domain.eligible);
    SameResult(wide.current.result, wide.original.result);
  }
#else
  GTEST_SKIP() << "FTZ/DAZ qualification requires MXCSR";
#endif
}

TEST(RepresentedNativeStorage, WorkersReuseWideArenaAndPreserveCompleteReportsCapsAndRetries) {
  const std::vector<ct::RepresentedTrianglePath> paths{Static(10, PositiveTriangle()),
      Static(20, PositiveTriangle()), Static(30, PositiveTriangle(3)),
      Static(40, BaseTriangle()), Static(50, BaseTriangle(1))};
  const ct::RepresentedTrianglePair pairs[]{{0,1}, {0,2}, {3,4}};
  std::vector<ct::RepresentedIntervalResult> expected;
  for (const auto pair : pairs) expected.push_back(CompareStorage({paths[pair.first], paths[pair.second]}).original.result);
  const auto bytes = CompareStorage({paths[0], paths[1]}).wide_scratch_bytes;
  for (unsigned workers : {1u,4u}) for (std::size_t cap : {1u,3u}) {
    ct::RepresentedIntervalLimits limits; limits.worker_count = workers; limits.max_total_work = cap;
    auto owner = Owner(limits);
    EXPECT_EQ(owner.forecast().exact_scratch_bytes, workers * bytes);
    SameResult(One(owner, {paths[0],paths[1]}), expected[0]);
    for (unsigned repeat = 0; repeat < 3; ++repeat) {
      const auto before = owner.results();
      const auto report = owner.Certify(paths.data(), paths.size(), pairs, 3);
      ct::RepresentedIntervalReport wanted;
      wanted.input_paths = paths.size(); wanted.input_pairs = wanted.unique_pairs = 3;
      wanted.certified_crossing_contact = 1; wanted.work = expected[0].work;
      if (cap == 1) {
        wanted.status = S::ResourceLimit; wanted.message = "resource limit";
        wanted.input_pair = 1; wanted.total_work_limit = 1; wanted.rejected_pair_work = expected[1].work;
        SameNativeReport(report, wanted);
        ASSERT_EQ(owner.results().data, before.data); ASSERT_EQ(owner.results().count, 1u);
        SameResult(owner.results().data[0], expected[0]);
      } else {
        wanted.certified_separated = 2; wanted.work += expected[1].work + expected[2].work;
        SameNativeReport(report, wanted); ASSERT_EQ(owner.results().count, 3u);
        for (unsigned i = 0; i < 3; ++i) SameResult(owner.results().data[i], expected[i]);
      }
    }
    SameResult(One(owner, {paths[0],paths[1]}), expected[0]);
  }
}

TEST(RepresentedNativeStorage, InvalidInputsCannotChooseBackendAndPreservedPublicationRemainsUsable) {
  const auto a = Static(10, PositiveTriangle()); auto b = Static(20, PositiveTriangle());
  b.vertices[2].endpoint[1].z = NAN;
  const auto invalid = storage::CompareNativeStorage(a, b, {});
  EXPECT_EQ(invalid.status, S::InvalidInput);
  EXPECT_EQ(invalid.current.counters.narrow_pairs + invalid.current.counters.wide_pairs, 0u);
  ct::RepresentedIntervalLimits limits; limits.max_depth = 53;
  EXPECT_EQ(storage::CompareNativeStorage(a, Static(20, PositiveTriangle()), limits).status, S::InvalidInput);
  limits.max_depth = 0; limits.max_work_per_pair = 0;
  EXPECT_EQ(storage::CompareNativeStorage(a, Static(20, PositiveTriangle()), limits).status, S::InvalidInput);
  auto owner = Owner(); One(owner, {a,Static(20, PositiveTriangle())});
  const auto before = owner.results(); const auto saved = before.data[0];
  const ct::RepresentedTrianglePath malformed[]{a,b}; const ct::RepresentedTrianglePair pair{0,1};
  EXPECT_EQ(owner.Certify(malformed, 2, &pair, 1).status, S::InvalidInput);
  ASSERT_EQ(owner.results().data, before.data); SameResult(owner.results().data[0], saved);
  SameResult(One(owner, {a,Static(20, PositiveTriangle())}), saved);
}
}  // namespace represented_interval_test
