// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include <gtest/gtest.h>

#include <limits>

namespace {
namespace ft = fixed_triangle_test;
namespace ct = tlfea::contact;

void DisjointTriangles(ct::CurrentFixedTriangle* triangles) {
  const ct::Vec3 pa[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const ct::Vec3 pb[3]{{0, 0, 1}, {2, 0, 1}, {0, 2, 1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  triangles[0] = ft::Triangle(100, 0, pa, ia);
  triangles[1] = ft::Triangle(200, 0, pb, ib);
}

TEST(FixedTriangleCapacity, ExactFeatureCapPassesAndMinusOneRejectsNoPrefix) {
  ct::CurrentFixedTriangle triangles[2];
  DisjointTriangles(triangles);
  const ct::FixedTrianglePair pair[1]{{0, 1}};

  ct::FixedTriangleFeatureDiscovery exact;
  ft::Initialize(&exact, ft::Limits(1, 15, 15, 1, 1));
  auto report = exact.Discover(triangles, 2, pair, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 15u);
  EXPECT_EQ(exact.features().count, 15u);

  ct::FixedTriangleFeatureDiscovery short_output;
  ft::Initialize(&short_output, ft::Limits(1, 15, 14, 1, 1));
  report = short_output.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::ResourceLimit);
  EXPECT_EQ(report.feature_tasks, 15u);
  EXPECT_EQ(report.raw_feature_candidates, 15u);
  EXPECT_EQ(report.feature_candidates, 15u);
  EXPECT_FALSE(short_output.features().complete);
  EXPECT_EQ(short_output.features().count, 0u);

  // Capacity failure is recoverable and cannot leak the rejected prefix.  A
  // locally adjacent retry still executes all 15 tasks and publishes its nine
  // nonincident candidates.
  const ct::Vec3 adjacent_points[3]{{0, 0, 0}, {-1, 0, 1},
                                     {0, -1, 1}};
  const std::uint64_t adjacent_ids[3]{1, 12, 13};
  triangles[1] = ft::Triangle(200, 0, adjacent_points, adjacent_ids);
  report = short_output.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 15u);
  EXPECT_TRUE(short_output.features().complete);
  EXPECT_EQ(short_output.features().count, 9u);
}

TEST(FixedTriangleCapacity, RawTaskCapacityCountsDuplicatesBeforeDeduplication) {
  ct::CurrentFixedTriangle triangles[2];
  DisjointTriangles(triangles);
  const ct::FixedTrianglePair duplicates[2]{{0, 1}, {1, 0}};

  ct::FixedTriangleFeatureDiscovery exact;
  ft::Initialize(&exact, ft::Limits(2, 30, 15, 2, 1));
  auto report = exact.Discover(triangles, 2, duplicates, 2);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 30u);
  EXPECT_EQ(report.raw_feature_candidates, 30u);
  EXPECT_EQ(report.feature_candidates, 15u);

  ct::FixedTriangleFeatureDiscovery short_staging;
  ft::Initialize(&short_staging, ft::Limits(2, 29, 15, 2, 1));
  report = short_staging.Discover(triangles, 2, duplicates, 2);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::ResourceLimit);
  // Topology counting proves the raw cap failure before geometry work.
  EXPECT_EQ(report.feature_tasks, 0u);
  EXPECT_EQ(report.raw_feature_candidates, 30u);
  EXPECT_FALSE(short_staging.features().complete);

  report = short_staging.Discover(triangles, 2, duplicates, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 15u);
  EXPECT_EQ(short_staging.features().count, 15u);
}

TEST(FixedTriangleCapacity,
     ExactIntersectionCapPassesAndMinusOneRejectsThenRetries) {
  const ct::Vec3 pa[3]{{-2, -2, 0}, {2, -2, 0}, {0, 2, 0}};
  const ct::Vec3 pb[3]{{0, -0.5, -1}, {0, 0.5, 1}, {0, 1, -1}};
  const ct::Vec3 separated[3]{{0, 0, 2}, {1, 0, 2}, {0, 1, 2}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, pa, ia), ft::Triangle(200, 0, pb, ib)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};

  ct::FixedTriangleFeatureDiscovery exact;
  ft::Initialize(&exact, ft::Limits(1, 15, 15, 1, 1));
  auto report = exact.Discover(triangles, 2, pair, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.raw_intersections, 1u);
  EXPECT_EQ(report.intersections, 1u);

  ct::FixedTriangleFeatureDiscovery short_output;
  ft::Initialize(&short_output, ft::Limits(1, 15, 15, 1, 0));
  report = short_output.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::ResourceLimit);
  EXPECT_EQ(report.feature_tasks, 15u);
  EXPECT_EQ(report.raw_intersections, 1u);
  EXPECT_EQ(report.intersections, 1u);
  EXPECT_FALSE(short_output.features().complete);
  EXPECT_FALSE(short_output.intersections().complete);

  triangles[1] = ft::Triangle(200, 0, separated, ib);
  report = short_output.Discover(triangles, 2, pair, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 15u);
  EXPECT_EQ(short_output.features().count, 15u);
  EXPECT_EQ(short_output.intersections().count, 0u);
}

TEST(FixedTriangleCapacity,
     RawIntersectionStagingMinusOneCountsAllThenRetries) {
  const ct::Vec3 pa[3]{{-2, -2, 0}, {2, -2, 0}, {0, 2, 0}};
  const ct::Vec3 pb[3]{{0, -0.5, -1}, {0, 0.5, 1}, {0, 1, -1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  const ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, pa, ia), ft::Triangle(200, 0, pb, ib)};
  const ct::FixedTrianglePair duplicates[2]{{0, 1}, {1, 0}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery, ft::Limits(2, 30, 15, 1, 1));

  auto report = discovery.Discover(triangles, 2, duplicates, 2);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::ResourceLimit);
  EXPECT_EQ(report.feature_tasks, 30u);
  EXPECT_EQ(report.raw_intersections, 2u);
  EXPECT_FALSE(discovery.features().complete);
  EXPECT_FALSE(discovery.intersections().complete);

  report = discovery.Discover(triangles, 2, duplicates, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 15u);
  EXPECT_EQ(report.raw_intersections, 1u);
  EXPECT_EQ(discovery.intersections().count, 1u);
}

TEST(FixedTriangleCapacity, ForecastOneByteShortAndCountOverflowReject) {
  EXPECT_EQ(ct::FixedTriangleFeatureDiscovery::Preflight().report.status,
            ct::FixedTriangleDiscoveryStatus::Ok);
  auto limits = ft::Limits(4);
  const auto exact = ct::FixedTriangleFeatureDiscovery::Preflight(limits);
  ASSERT_EQ(exact.report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(exact.forecast.triangle_ledger_capacity,
            limits.max_triangle_references);
  EXPECT_EQ(exact.forecast.vertex_ledger_capacity,
            limits.max_vertex_references);
  EXPECT_EQ(exact.forecast.edge_ledger_capacity,
            limits.max_edge_references);
  EXPECT_EQ(exact.forecast.feature_publication_capacity,
            limits.max_feature_candidates);
  EXPECT_EQ(exact.forecast.pair_intersection_capacity,
            limits.max_input_pairs);
  ASSERT_GT(exact.forecast.owned_host_bytes, 0u);
  limits.max_host_bytes = exact.forecast.owned_host_bytes - 1;
  EXPECT_EQ(ct::FixedTriangleFeatureDiscovery::Preflight(limits).report.status,
            ct::FixedTriangleDiscoveryStatus::ResourceLimit);

  limits = ft::Limits(1);
  limits.max_input_pairs =
      std::numeric_limits<std::size_t>::max() / 15 + 1;
  EXPECT_EQ(ct::FixedTriangleFeatureDiscovery::Preflight(limits).report.status,
            ct::FixedTriangleDiscoveryStatus::InvalidInput);

  limits = ft::Limits(1);
  limits.max_triangle_references =
      std::numeric_limits<std::size_t>::max();
  EXPECT_EQ(ct::FixedTriangleFeatureDiscovery::Preflight(limits).report.status,
            ct::FixedTriangleDiscoveryStatus::ResourceLimit);
}

TEST(FixedTriangleCapacity,
     TriangleVertexAndEdgeLedgerCapsRejectBeforeGeometryAndRetry) {
  ct::CurrentFixedTriangle triangles[2];
  DisjointTriangles(triangles);
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  const ct::FixedTrianglePair same[1]{{0, 0}};

  auto limits = ft::Limits(1);
  limits.max_triangle_references = 1;
  ct::FixedTriangleFeatureDiscovery triangle_short;
  ft::Initialize(&triangle_short, limits);
  auto report = triangle_short.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::ResourceLimit);
  EXPECT_EQ(report.triangle_references, 2u);
  EXPECT_EQ(report.feature_tasks, 0u);

  limits = ft::Limits(1);
  limits.max_vertex_references = 5;
  ct::FixedTriangleFeatureDiscovery vertex_short;
  ft::Initialize(&vertex_short, limits);
  report = vertex_short.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::ResourceLimit);
  EXPECT_EQ(report.triangles, 2u);
  EXPECT_EQ(report.vertex_references, 6u);
  EXPECT_EQ(report.feature_tasks, 0u);
  report = vertex_short.Discover(triangles, 2, same, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.triangles, 1u);
  EXPECT_EQ(report.vertex_references, 3u);
  EXPECT_EQ(report.feature_tasks, 15u);

  limits = ft::Limits(1);
  limits.max_edge_references = 5;
  ct::FixedTriangleFeatureDiscovery edge_short;
  ft::Initialize(&edge_short, limits);
  report = edge_short.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::ResourceLimit);
  EXPECT_EQ(report.edge_references, 6u);
  EXPECT_EQ(report.feature_tasks, 0u);
}

TEST(FixedTriangleCapacity, LateInvalidPairPreservesPreviousPublication) {
  ct::CurrentFixedTriangle triangles[2];
  DisjointTriangles(triangles);
  const ct::FixedTrianglePair good[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  ASSERT_EQ(discovery.Discover(triangles, 2, good, 1).status,
            ct::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_TRUE(discovery.features().complete);
  const auto previous_count = discovery.features().count;
  const auto previous_first = discovery.features().data[0];

  const ct::FixedTrianglePair bad[2]{{0, 1}, {0, 2}};
  const auto report = discovery.Discover(triangles, 2, bad, 2);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::OutOfRange);
  EXPECT_EQ(report.input_pair, 1u);
  ASSERT_TRUE(discovery.features().complete);
  ASSERT_EQ(discovery.features().count, previous_count);
  EXPECT_TRUE(ft::Same(discovery.features().data[0].key,
                       previous_first.key));
  EXPECT_TRUE(discovery.intersections().complete);
}

TEST(FixedTriangleCapacity,
     OwnedPublicationAndControlAliasesRejectAndPreservePublication) {
  ct::CurrentFixedTriangle triangles[2];
  DisjointTriangles(triangles);
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  ASSERT_EQ(discovery.Discover(triangles, 2, pair, 1).status,
            ct::FixedTriangleDiscoveryStatus::Ok);
  const auto previous = discovery.features();
  ASSERT_TRUE(previous.complete);
  ASSERT_GT(previous.count, 0u);
  const auto first_key = previous.data[0].key;

  auto report = discovery.Discover(
      reinterpret_cast<const ct::CurrentFixedTriangle*>(previous.data),
      1, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::InvalidInput);
  ASSERT_TRUE(discovery.features().complete);
  EXPECT_TRUE(ft::Same(discovery.features().data[0].key, first_key));

  const auto rebound = discovery.features();
  report = discovery.Discover(
      triangles, 2,
      reinterpret_cast<const ct::FixedTrianglePair*>(rebound.data), 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::InvalidInput);
  ASSERT_TRUE(discovery.features().complete);
  EXPECT_TRUE(ft::Same(discovery.features().data[0].key, first_key));

  report = discovery.Discover(
      reinterpret_cast<const ct::CurrentFixedTriangle*>(&discovery),
      1, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::InvalidInput);
  ASSERT_TRUE(discovery.features().complete);
  EXPECT_TRUE(ft::Same(discovery.features().data[0].key, first_key));
}

}  // namespace
