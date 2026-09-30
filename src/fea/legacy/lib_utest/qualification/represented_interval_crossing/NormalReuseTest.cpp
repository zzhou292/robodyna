// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "ResultAssertions.h"
#include "lib_src/collision/represented_interval_crossing/NormalReuseQualification.h"
#include "lib_src/collision/represented_interval_crossing/Batch.h"

namespace represented_interval_test {
namespace qualification = ct::represented_interval_crossing;

qualification::NormalReuseComparison ComparedNormals(
    const ct::RepresentedTrianglePath& first,
    const ct::RepresentedTrianglePath& second,
    ct::RepresentedIntervalLimits limits = {}) {
  const auto comparison = qualification::CompareNormalReuse(first, second, limits);
  EXPECT_EQ(comparison.status, S::Ok);
  SameResult(comparison.memoized.result, comparison.recomputed.result);
  EXPECT_FALSE(comparison.memoized.counters.saturated);
  EXPECT_FALSE(comparison.recomputed.counters.saturated);
  EXPECT_EQ(comparison.memoized.counters.evaluated_cells,
            comparison.recomputed.counters.evaluated_cells);
  EXPECT_LE(comparison.memoized.counters.normal_evaluations,
            6 * comparison.memoized.counters.evaluated_cells);
  EXPECT_LE(comparison.memoized.counters.normal_evaluations,
            comparison.recomputed.counters.normal_evaluations);
  return comparison;
}

TEST(RepresentedNormalReuse, StaticSeparationAndContactRemoveRepeatedNormalArithmetic) {
  const auto first = Static(10, BaseTriangle());
  const auto contact = Static(20, BaseTriangle());
  const auto separated = Static(20, BaseTriangle(1));
  auto owner = Owner();
  for (const auto& second : {contact, separated}) {
    const auto result = ComparedNormals(first, second);
    SameResult(One(owner, {first, second}), result.recomputed.result);
    EXPECT_EQ(result.memoized.counters.evaluated_cells, 1u);
    EXPECT_EQ(result.memoized.counters.normal_evaluations, 6u);
    EXPECT_GT(result.memoized.counters.cache_hits, 0u);
    EXPECT_LT(result.memoized.counters.normal_evaluations,
              result.recomputed.counters.normal_evaluations);
  }
  const auto result = ComparedNormals(first, separated);
  EXPECT_EQ(result.recomputed.counters.normal_evaluations, 24u);
}

TEST(RepresentedNormalReuse, DegenerateShortCircuitStillAllowsTheOriginalLaterWitness) {
  const std::array<ct::Vec3, 3> collapsed{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
  const auto first = Path(10, collapsed, BaseTriangle());
  const auto second = Static(20, BaseTriangle());
  const auto result = ComparedNormals(first, second);
  ASSERT_EQ(result.memoized.result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(result.memoized.result.witness_time_numerator, 1u);
  EXPECT_EQ(result.memoized.result.witness_time_depth, 1u);
  // Only A(lower), A(mid), B(mid) are requested. Eager six-normal filling
  // changes the original short-circuit execution and would fail this count.
  EXPECT_EQ(result.memoized.counters.normal_evaluations, 3u);
  auto owner = Owner();
  SameResult(One(owner, {first, second}), result.recomputed.result);
}

TEST(RepresentedNormalReuse, DeepCellsCapsAndPermutationMatchTheOriginalExactExecutor) {
  const auto first = Static(10, BaseTriangle());
  const auto second = Path(20, BaseTriangle(1), BaseTriangle(1 - std::ldexp(1.0, 40)));
  for (unsigned depth : {0u, 8u, 40u, 52u})
    for (std::size_t work : {1u, 64u}) {
      ct::RepresentedIntervalLimits limits;
      limits.max_depth = depth; limits.max_work_per_pair = work;
      const auto result = ComparedNormals(first, second, limits);
      const auto permuted = ComparedNormals(Permute(first, {2, 0, 1}),
          Permute(second, {1, 2, 0}), limits);
      SameResult(result.memoized.result, permuted.memoized.result);
    }
}

TEST(RepresentedNormalReuse, ExtremeFiniteExponentsAndInvalidInputKeepFailureBoundaries) {
  const auto base = BaseTriangle();
  for (const double scale : {std::numeric_limits<double>::denorm_min(),
                            std::ldexp(1.0, -1000), 1.0, std::ldexp(1.0, 1000)}) {
    auto a = base, b = base;
    for (auto& point : a) { point.x *= scale; point.y *= scale; }
    for (auto& point : b) { point.x *= scale; point.y *= scale; point.z = scale; }
    ComparedNormals(Static(10, a), Static(20, b));
    b[2].x = -scale;
    ComparedNormals(Static(10, a), Path(20, b, a));
  }
  auto invalid = Static(20, base);
  invalid.vertices[0].endpoint[1].z = NAN;
  EXPECT_EQ(qualification::CompareNormalReuse(Static(10, base), invalid, {}).status,
            S::InvalidInput);
  ct::RepresentedIntervalLimits invalid_limits;
  invalid_limits.max_depth = 53;
  EXPECT_EQ(qualification::CompareNormalReuse(Static(10, base), Static(20, base), invalid_limits).status,
            S::InvalidInput);
  invalid_limits.max_depth = 0; invalid_limits.max_work_per_pair = 0;
  EXPECT_EQ(qualification::CompareNormalReuse(Static(10, base), Static(20, base), invalid_limits).status,
            S::InvalidInput);
}

TEST(RepresentedNormalReuse, MixedHugeAndDenormalCoordinatesPreserveCheckedArithmetic) {
  const auto paths = MixedExponentPaths();
  ct::RepresentedIntervalLimits limits; limits.max_depth = 52;
  const auto result = ComparedNormals(paths[0], paths[1], limits);
  EXPECT_EQ(result.memoized.result.classification, C::CertifiedSeparated);
  EXPECT_EQ(result.memoized.result.reason, R::None);
}

TEST(RepresentedNormalReuse, ResetAcrossPairsCallsAndWorkersPreservesPublications) {
  const auto first = Static(10, BaseTriangle());
  const std::array<ct::Vec3, 3> collapsed{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
  const std::vector<ct::RepresentedTrianglePath> paths{
      first, Static(20, BaseTriangle()), Static(30, BaseTriangle(1)),
      Path(40, BaseTriangle(1), BaseTriangle(-3)), Static(50, collapsed)};
  const std::vector<ct::RepresentedTrianglePair> pairs{{0, 1}, {0, 2}, {0, 3}, {0, 4}};
  std::vector<ct::RepresentedIntervalResult> expected;
  for (std::size_t i = 1; i < paths.size(); ++i)
    expected.push_back(ComparedNormals(first, paths[i]).recomputed.result);
  for (unsigned workers : {1u, 4u}) {
    ct::RepresentedIntervalLimits limits; limits.worker_count = workers;
    auto owner = Owner(limits);
    for (unsigned repetition = 0; repetition < 3; ++repetition) {
      auto reordered = pairs;
      if (repetition & 1) std::reverse(reordered.begin(), reordered.end());
      const auto report = owner.Certify(paths.data(), paths.size(), reordered.data(), reordered.size());
      ASSERT_EQ(report.status, S::Ok);
      std::size_t expected_work = 0, separated = 0, crossing = 0, unresolved = 0;
      for (const auto& row : expected) {
        expected_work += row.work;
        separated += row.classification == C::CertifiedSeparated;
        crossing += row.classification == C::CertifiedCrossingContact;
        unresolved += row.classification == C::Unresolved;
      }
      EXPECT_EQ(report.work, expected_work);
      EXPECT_EQ(report.certified_separated, separated);
      EXPECT_EQ(report.certified_crossing_contact, crossing);
      EXPECT_EQ(report.unresolved, unresolved);
      ASSERT_EQ(owner.results().count, expected.size());
      for (std::size_t i = 0; i < expected.size(); ++i)
        SameResult(owner.results().data[i], expected[i]);
      auto invalid = paths; invalid.back().vertices[0].endpoint[0].x = NAN;
      EXPECT_EQ(owner.Certify(invalid.data(), invalid.size(), pairs.data(), pairs.size()).status,
                S::InvalidInput);
      ASSERT_EQ(owner.results().count, expected.size());
      for (std::size_t i = 0; i < expected.size(); ++i)
        SameResult(owner.results().data[i], expected[i]);
    }
  }
}

TEST(RepresentedNormalReuse, ScratchForecastAndExactHostCapIncludeAllCachedNormals) {
  const auto probe = ComparedNormals(Static(10, BaseTriangle()), Static(20, BaseTriangle(1)));
  ASSERT_GT(probe.normal_storage_bytes, 0u);
  EXPECT_GT(probe.worker_exact_scratch_bytes, probe.normal_storage_bytes);
  for (unsigned workers : {1u, 4u}) {
    ct::RepresentedIntervalLimits limits; limits.worker_count = workers;
    limits.max_depth = 52;
    const auto plan = ct::RepresentedIntervalCrossing::Preflight(limits);
    ASSERT_EQ(plan.report.status, S::Ok);
    EXPECT_EQ(plan.forecast.exact_scratch_bytes, workers * probe.worker_exact_scratch_bytes);
    limits.max_host_bytes = plan.forecast.owned_host_bytes - 1;
    ct::RepresentedIntervalCrossing owner;
    EXPECT_EQ(owner.Initialize(limits).status, S::ResourceLimit);
    ++limits.max_host_bytes;
    ASSERT_EQ(owner.Initialize(limits).status, S::Ok);
    SameResult(One(owner, {Static(10, BaseTriangle()), Static(20, BaseTriangle(1))}),
               probe.recomputed.result);
  }
}

TEST(RepresentedNormalReuse, BatchSlicesAliasAndCapacityFailureKeepCanonicalResults) {
  const std::vector<ct::RepresentedTrianglePath> paths{
      Static(10, BaseTriangle()), Static(20, BaseTriangle()),
      Static(30, BaseTriangle(1)), Path(40, BaseTriangle(1), BaseTriangle(-3))};
  const std::vector<ct::RepresentedTrianglePair> pairs{{0, 1}, {0, 2}, {0, 3}};
  std::vector<ct::RepresentedIntervalResult> expected;
  for (std::size_t i = 1; i < paths.size(); ++i)
    expected.push_back(ComparedNormals(paths[0], paths[i]).recomputed.result);
  for (std::size_t slice : {1u, 2u, 3u}) {
    auto owner = Owner();
    std::vector<ct::RepresentedIntervalResult> scratch(pairs.size());
    const auto result = qualification::BatchAccess::Certify(owner,
        paths.data(), paths.size(), pairs.data(), pairs.size(), slice,
        scratch.data(), scratch.size());
    ASSERT_EQ(result.status, S::Ok);
    ASSERT_TRUE(result.results.complete);
    for (std::size_t i = 0; i < scratch.size(); ++i) SameResult(scratch[i], expected[i]);
    const auto prior = std::vector<ct::RepresentedIntervalResult>(
        owner.results().data, owner.results().data + owner.results().count);
    auto* alias = const_cast<ct::RepresentedIntervalResult*>(owner.results().data);
    const auto aliased = qualification::BatchAccess::Certify(owner,
        paths.data(), paths.size(), pairs.data(), 1, 1, alias, 1);
    EXPECT_EQ(aliased.status, S::InvalidInput);
    ASSERT_EQ(owner.results().count, prior.size());
    for (std::size_t i = 0; i < prior.size(); ++i) SameResult(owner.results().data[i], prior[i]);
    const auto denied = qualification::BatchAccess::Certify(owner,
        paths.data(), paths.size(), pairs.data(), pairs.size(), slice,
        scratch.data(), scratch.size() - 1);
    EXPECT_EQ(denied.status, S::ResourceLimit);
    ASSERT_EQ(owner.results().count, prior.size());
    for (std::size_t i = 0; i < prior.size(); ++i) SameResult(owner.results().data[i], prior[i]);
  }
}

}  // namespace represented_interval_test
