// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "lib_src/collision/fixed_triangle_features/Geometry.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace {
namespace ft = fixed_triangle_test;
namespace ct = tlfea::contact;
namespace geo = tlfea::contact::fixed_triangle_features;

struct Observation {
  std::array<unsigned char, sizeof(ct::FixedTriangleDiscoveryReport)> report;
  std::vector<unsigned char> features;
  std::vector<unsigned char> intersections;
  bool features_complete = false;
  bool intersections_complete = false;

  bool operator==(const Observation& other) const {
    return report == other.report && features == other.features &&
           intersections == other.intersections &&
           features_complete == other.features_complete &&
           intersections_complete == other.intersections_complete;
  }
};

Observation Observe(const ct::FixedTriangleDiscoveryReport& report,
                    const ct::FixedTriangleFeatureDiscovery& discovery) {
  Observation result;
  std::memcpy(result.report.data(), &report, sizeof(report));
  const auto features = discovery.features();
  result.features_complete = features.complete;
  result.features.resize(features.count * sizeof(*features.data));
  if (!result.features.empty())
    std::memcpy(result.features.data(), features.data,
                result.features.size());
  const auto intersections = discovery.intersections();
  result.intersections_complete = intersections.complete;
  result.intersections.resize(
      intersections.count * sizeof(*intersections.data));
  if (!result.intersections.empty())
    std::memcpy(result.intersections.data(), intersections.data,
                result.intersections.size());
  return result;
}

ct::FixedTriangleFeatureLimits WorkerLimits(std::size_t pairs,
                                            unsigned workers) {
  auto result = ft::Limits(pairs);
  result.worker_count = workers;
  return result;
}

std::array<ct::CurrentFixedTriangle, 4> Geometry() {
  const ct::Vec3 a[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const ct::Vec3 b[3]{{0, 0, 1}, {2, 0, 1}, {0, 2, 1}};
  const ct::Vec3 c[3]{{0, 0, 0}, {-2, 0, 1}, {0, -2, 1}};
  const ct::Vec3 d[3]{{1, -1, -1}, {1, 1, 1}, {1, 2, -1}};
  const std::uint64_t ai[3]{1, 2, 3};
  const std::uint64_t bi[3]{11, 12, 13};
  const std::uint64_t ci[3]{1, 22, 23};
  const std::uint64_t di[3]{31, 32, 33};
  return {ft::Triangle(100, 0, a, ai),
          ft::Triangle(200, 0, b, bi),
          ft::Triangle(300, 0, c, ci),
          ft::Triangle(400, 0, d, di)};
}

Observation EvaluateObservation(
    unsigned workers,
    const ct::CurrentFixedTriangle* triangles,
    std::size_t triangle_count,
    const ct::FixedTrianglePair* pairs,
    std::size_t pair_count,
    const ct::FixedTriangleFeatureTaskMask* masks,
    std::size_t feature_cap = SIZE_MAX) {
  auto limits = WorkerLimits(pair_count, workers);
  if (feature_cap != SIZE_MAX)
    limits.max_feature_candidates = feature_cap;
  ct::FixedTriangleFeatureDiscovery discovery;
  EXPECT_EQ(discovery.Initialize(limits).status,
            ct::FixedTriangleDiscoveryStatus::Ok);
  const auto report = masks
      ? discovery.DiscoverMasked(
            triangles, triangle_count, pairs, pair_count, masks)
      : discovery.Discover(
            triangles, triangle_count, pairs, pair_count);
  return Observe(report, discovery);
}

TEST(FixedTriangleParallel,
     WorkerCountsMatchAcrossMasksCapsPermutationsAndRetries) {
  const auto triangles = Geometry();
  const std::array<ct::FixedTrianglePair, 8> pairs{{
      {0, 1}, {1, 0}, {0, 2}, {2, 0},
      {0, 3}, {3, 0}, {1, 3}, {3, 1}}};
  std::array<ct::FixedTriangleFeatureTaskMask, pairs.size()> masks;
  for (std::size_t i = 0; i < pairs.size(); ++i)
    masks[i] = geo::PairLocalFeatureTaskMask(
        triangles[pairs[i].first], triangles[pairs[i].second]);

  const auto serial = EvaluateObservation(
      1, triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), nullptr);
  EXPECT_EQ(serial, EvaluateObservation(
      2, triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), nullptr));
  EXPECT_EQ(serial, EvaluateObservation(
      4, triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), nullptr));

  const auto serial_masked = EvaluateObservation(
      1, triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), masks.data());
  EXPECT_EQ(serial_masked, EvaluateObservation(
      2, triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), masks.data()));
  EXPECT_EQ(serial_masked, EvaluateObservation(
      4, triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), masks.data()));

  auto reversed_pairs = pairs;
  auto reversed_masks = masks;
  std::reverse(reversed_pairs.begin(), reversed_pairs.end());
  std::reverse(reversed_masks.begin(), reversed_masks.end());
  EXPECT_EQ(serial_masked.features,
            EvaluateObservation(
                4, triangles.data(), triangles.size(),
                reversed_pairs.data(), reversed_pairs.size(),
                reversed_masks.data()).features);

  const auto capped = EvaluateObservation(
      1, triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), masks.data(), 1);
  EXPECT_EQ(capped, EvaluateObservation(
      4, triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), masks.data(), 1));

  auto limits = WorkerLimits(pairs.size(), 4);
  ct::FixedTriangleFeatureDiscovery repeated;
  ASSERT_EQ(repeated.Initialize(limits).status,
            ct::FixedTriangleDiscoveryStatus::Ok);
  const auto first_report = repeated.DiscoverMasked(
      triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), masks.data());
  const auto first = Observe(first_report, repeated);
  const auto second_report = repeated.DiscoverMasked(
      triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), masks.data());
  EXPECT_EQ(first, Observe(second_report, repeated));
}

TEST(FixedTriangleParallel,
     EarliestExactFailureMatchesSerialAndPreservesPublication) {
  const auto regular = Geometry();
  const double smallest = std::nextafter(0.0, 1.0);
  const ct::Vec3 exact_edge{smallest, -1, 1};
  const ct::Vec3 failure_source[3]{
      exact_edge, {exact_edge.x + 4, exact_edge.y, exact_edge.z},
      {exact_edge.x, exact_edge.y + 4, exact_edge.z}};
  const ct::Vec3 failure_target[3]{
      {0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const std::uint64_t source_ids[3]{50, 51, 52};
  const std::uint64_t target_ids[3]{61, 62, 63};
  std::array<ct::CurrentFixedTriangle, 6> triangles{
      regular[0], regular[1], regular[2], regular[3],
      ft::Triangle(500, 0, failure_source, source_ids),
      ft::Triangle(600, 0, failure_target, target_ids)};
  const std::array<ct::FixedTrianglePair, 4> pairs{{
      {0, 1}, {4, 5}, {2, 3}, {5, 4}}};

  const auto serial = EvaluateObservation(
      1, triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), nullptr);
  EXPECT_EQ(serial, EvaluateObservation(
      2, triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), nullptr));
  EXPECT_EQ(serial, EvaluateObservation(
      4, triangles.data(), triangles.size(),
      pairs.data(), pairs.size(), nullptr));
  ct::FixedTriangleDiscoveryReport report;
  std::memcpy(&report, serial.report.data(), sizeof(report));
  EXPECT_EQ(report.status,
            ct::FixedTriangleDiscoveryStatus::NonFiniteResult);
  EXPECT_EQ(report.input_pair, 1u);
  EXPECT_EQ(report.input_task, 0u);
  EXPECT_EQ(report.feature_tasks, 16u);

  for (unsigned workers : {1u, 2u, 4u}) {
    auto limits = WorkerLimits(pairs.size(), workers);
    ct::FixedTriangleFeatureDiscovery discovery;
    ASSERT_EQ(discovery.Initialize(limits).status,
              ct::FixedTriangleDiscoveryStatus::Ok);
    const ct::FixedTrianglePair good[1]{{0, 1}};
    ASSERT_EQ(discovery.Discover(
                  triangles.data(), triangles.size(), good, 1).status,
              ct::FixedTriangleDiscoveryStatus::Ok);
    const auto before = Observe(
        discovery.Discover(
            triangles.data(), triangles.size(), good, 1),
        discovery);
    const auto failed = discovery.Discover(
        triangles.data(), triangles.size(), pairs.data(), pairs.size());
    EXPECT_EQ(failed.input_pair, 1u);
    const auto after = Observe(failed, discovery);
    EXPECT_EQ(after.features, before.features);
    EXPECT_EQ(after.intersections, before.intersections);
  }
}

TEST(FixedTriangleParallel,
     PreflightBoundsMetadataStacksAndRepeatedDestruction) {
  auto one = WorkerLimits(8, 1);
  auto four = WorkerLimits(8, 4);
  const auto one_plan =
      ct::FixedTriangleFeatureDiscovery::Preflight(one);
  const auto four_plan =
      ct::FixedTriangleFeatureDiscovery::Preflight(four);
  ASSERT_EQ(one_plan.report.status,
            ct::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_EQ(four_plan.report.status,
            ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(four_plan.forecast.worker_count, 4u);
  EXPECT_EQ(four_plan.forecast.pair_status_capacity, 8u);
  EXPECT_GT(four_plan.forecast.pair_status_bytes, 0u);
  EXPECT_GT(four_plan.forecast.worker_metadata_bytes,
            one_plan.forecast.worker_metadata_bytes);
  EXPECT_GT(four_plan.forecast.worker_stack_bytes,
            one_plan.forecast.worker_stack_bytes);
  EXPECT_EQ(four_plan.forecast.startup_host_bytes,
            four_plan.forecast.owned_host_bytes);
  four.max_host_bytes = four_plan.forecast.owned_host_bytes - 1;
  EXPECT_EQ(ct::FixedTriangleFeatureDiscovery::Preflight(four).report.status,
            ct::FixedTriangleDiscoveryStatus::ResourceLimit);
  four = WorkerLimits(8, 0);
  EXPECT_EQ(ct::FixedTriangleFeatureDiscovery::Preflight(four).report.status,
            ct::FixedTriangleDiscoveryStatus::InvalidInput);
  four = WorkerLimits(8, 8);
  EXPECT_EQ(ct::FixedTriangleFeatureDiscovery::Preflight(four).report.status,
            ct::FixedTriangleDiscoveryStatus::Ok);
  four.worker_count = 9;
  EXPECT_EQ(ct::FixedTriangleFeatureDiscovery::Preflight(four).report.status,
            ct::FixedTriangleDiscoveryStatus::InvalidInput);

  const auto triangles = Geometry();
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  for (unsigned iteration = 0; iteration < 12; ++iteration) {
    auto limits = WorkerLimits(1, 1 + iteration % 4);
    ct::FixedTriangleFeatureDiscovery discovery;
    ASSERT_EQ(discovery.Initialize(limits).status,
              ct::FixedTriangleDiscoveryStatus::Ok);
    EXPECT_EQ(discovery.Discover(
                  triangles.data(), triangles.size(), pair, 1).status,
              ct::FixedTriangleDiscoveryStatus::Ok);
  }
}

}  // namespace
