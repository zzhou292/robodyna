// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace represented_interval_test {
namespace {

std::array<ct::Vec3, 3> DiagonalSeparated(double shift = 0) {
  return {{{1.5 + shift, 1.5, 0},
           {3.5 + shift, 1.5, 0},
           {1.5 + shift, 3.5, 0}}};
}

}  // namespace

TEST(RepresentedIntervalCrossing,
     PerPairWorkExhaustionPublishesExplicitUnresolvedRecord) {
  ct::RepresentedIntervalLimits limits;
  limits.max_work_per_pair = 3;
  limits.max_depth = 20;
  auto owner = Owner(limits);
  const auto result =
      One(owner, {Static(10, BaseTriangle()),
                  Path(20, DiagonalSeparated(),
                       DiagonalSeparated(.125))});
  EXPECT_EQ(result.classification, C::Unresolved);
  EXPECT_EQ(result.reason, R::WorkExhausted);
  EXPECT_EQ(result.work, 3u);
}

TEST(RepresentedIntervalCrossing,
     CheckedTotalWorkExhaustionPreservesPublicationAndAllowsRetry) {
  ct::RepresentedIntervalLimits limits;
  limits.max_work_per_pair = 3;
  limits.max_total_work = 5;
  auto owner = Owner(limits);
  std::vector<ct::RepresentedTrianglePath> initial{
      Static(10, BaseTriangle()), Static(20, BaseTriangle(1))};
  const auto accepted = One(owner, initial);
  ASSERT_EQ(accepted.classification, C::CertifiedSeparated);
  const auto before_view = owner.results();
  const auto before = Bytes(before_view.data, before_view.count);

  std::vector<ct::RepresentedTrianglePath> expensive{
      Static(10, BaseTriangle()),
      Path(20, DiagonalSeparated(), DiagonalSeparated(.125)),
      Path(30, DiagonalSeparated(.125), DiagonalSeparated(.25))};
  const ct::RepresentedTrianglePair pairs[]{{0, 1}, {0, 2}};
  const auto failed =
      owner.Certify(expensive.data(), expensive.size(), pairs, 2);
  EXPECT_EQ(failed.status, S::ResourceLimit);
  EXPECT_EQ(failed.input_pair, 1u);
  const auto after = owner.results();
  EXPECT_TRUE(after.complete);
  EXPECT_EQ(after.count, before_view.count);
  EXPECT_EQ(Bytes(after.data, after.count), before);

  const auto retry = One(owner, initial);
  EXPECT_EQ(retry.classification, C::CertifiedSeparated);
}

TEST(RepresentedIntervalCrossing,
     ResultCapMinusOneFailureIsAtomicAndSubsetRetrySucceeds) {
  ct::RepresentedIntervalLimits limits;
  limits.max_results = 1;
  auto owner = Owner(limits);
  std::vector<ct::RepresentedTrianglePath> paths{
      Static(10, BaseTriangle()), Static(20, BaseTriangle(1)),
      Static(30, BaseTriangle(2))};
  const auto accepted = One(owner, {paths[0], paths[1]});
  ASSERT_EQ(accepted.classification, C::CertifiedSeparated);
  const auto before_view = owner.results();
  const auto before = Bytes(before_view.data, before_view.count);

  const ct::RepresentedTrianglePair too_many[]{{0, 1}, {0, 2}};
  auto report =
      owner.Certify(paths.data(), paths.size(), too_many, 2);
  EXPECT_EQ(report.status, S::ResourceLimit);
  EXPECT_EQ(report.unique_pairs, 2u);
  auto current = owner.results();
  EXPECT_EQ(Bytes(current.data, current.count), before);

  const ct::RepresentedTrianglePair retry_pair{0, 2};
  report = owner.Certify(paths.data(), paths.size(), &retry_pair, 1);
  ASSERT_EQ(report.status, S::Ok);
  current = owner.results();
  ASSERT_EQ(current.count, 1u);
  EXPECT_EQ(current.data[0].key.paths[1].parent_eid, 30u);
}

TEST(RepresentedIntervalCrossing,
     HostByteCapMinusOneRejectsWithoutInitializationThenRetriesExactly) {
  ct::RepresentedIntervalLimits limits;
  limits.max_depth = 52;
  const auto plan = ct::RepresentedIntervalCrossing::Preflight(limits);
  ASSERT_EQ(plan.report.status, S::Ok);
  ASSERT_GT(plan.forecast.owned_host_bytes, 0u);
  EXPECT_EQ(plan.forecast.vertex_ledger_capacity, 3 * limits.max_paths);
  EXPECT_EQ(plan.forecast.dfs_frame_capacity, limits.max_depth + 1);
  EXPECT_GT(plan.forecast.vertex_ledger_bytes, 0u);
  EXPECT_GT(plan.forecast.dfs_frame_bytes, 0u);
  EXPECT_GT(plan.forecast.exact_scratch_bytes, 0u);
  EXPECT_GT(plan.forecast.owned_host_bytes,
            plan.forecast.path_index_bytes + plan.forecast.pair_bytes +
                plan.forecast.result_bytes +
                plan.forecast.vertex_ledger_bytes +
                plan.forecast.dfs_frame_bytes +
                plan.forecast.exact_scratch_bytes);
  limits.max_host_bytes = plan.forecast.owned_host_bytes - 1;
  ct::RepresentedIntervalCrossing owner;
  EXPECT_EQ(owner.Initialize(limits).status, S::ResourceLimit);
  EXPECT_FALSE(owner.initialized());
  ++limits.max_host_bytes;
  ASSERT_EQ(owner.Initialize(limits).status, S::Ok);
  EXPECT_EQ(owner.forecast().owned_host_bytes, limits.max_host_bytes);
  const auto result =
      One(owner, {Static(10, BaseTriangle()),
                  Static(20, BaseTriangle(1))});
  EXPECT_EQ(result.classification, C::CertifiedSeparated);
}

TEST(RepresentedIntervalCrossing,
     LateInvalidInputAndDuplicateIdentityPreserveCompleteOutput) {
  auto owner = Owner();
  std::vector<ct::RepresentedTrianglePath> paths{
      Static(10, BaseTriangle()), Static(20, BaseTriangle(1))};
  ASSERT_EQ(One(owner, paths).classification, C::CertifiedSeparated);
  const auto prior = owner.results();
  const auto bytes = Bytes(prior.data, prior.count);

  auto invalid = paths;
  invalid.back().vertices[2].endpoint[1].z =
      std::numeric_limits<double>::quiet_NaN();
  const ct::RepresentedTrianglePair pair{0, 1};
  auto report =
      owner.Certify(invalid.data(), invalid.size(), &pair, 1);
  EXPECT_EQ(report.status, S::InvalidInput);
  EXPECT_EQ(report.input_path, 1u);
  EXPECT_EQ(Bytes(owner.results().data, owner.results().count), bytes);

  auto duplicate = paths;
  duplicate[1].key = duplicate[0].key;
  report = owner.Certify(duplicate.data(), duplicate.size(), &pair, 1);
  EXPECT_EQ(report.status, S::IdentityMismatch);
  EXPECT_EQ(Bytes(owner.results().data, owner.results().count), bytes);
}

}  // namespace represented_interval_test
