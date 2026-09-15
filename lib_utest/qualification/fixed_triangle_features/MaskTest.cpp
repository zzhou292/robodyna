// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include <gtest/gtest.h>

#include <array>

namespace {
namespace ct = tlfea::contact;
namespace ft = fixed_triangle_test;

static_assert(ct::FixedTriangleVertexFaceTaskSlot(0, 0) == 0);
static_assert(ct::FixedTriangleVertexFaceTaskSlot(1, 0) == 1);
static_assert(ct::FixedTriangleVertexFaceTaskSlot(0, 1) == 2);
static_assert(ct::FixedTriangleVertexFaceTaskSlot(1, 1) == 3);
static_assert(ct::FixedTriangleVertexFaceTaskSlot(0, 2) == 4);
static_assert(ct::FixedTriangleVertexFaceTaskSlot(1, 2) == 5);
static_assert(ct::FixedTriangleEdgeEdgeTaskSlot(0, 0) == 6);
static_assert(ct::FixedTriangleEdgeEdgeTaskSlot(0, 2) == 8);
static_assert(ct::FixedTriangleEdgeEdgeTaskSlot(1, 0) == 9);
static_assert(ct::FixedTriangleEdgeEdgeTaskSlot(1, 2) == 11);
static_assert(ct::FixedTriangleEdgeEdgeTaskSlot(2, 0) == 12);
static_assert(ct::FixedTriangleEdgeEdgeTaskSlot(2, 2) == 14);
static_assert(sizeof(ct::FixedTriangleFeatureTaskMask) ==
              sizeof(std::uint16_t));

ct::FixedTriangleFeatureTaskMask Mask(
    std::initializer_list<unsigned> slots) {
  ct::FixedTriangleFeatureTaskMask result;
  for (const auto slot : slots)
    result.local_tasks |= ct::FixedTriangleFeatureTaskBit(slot);
  return result;
}

TEST(FixedTriangleTaskMask,
     ProductionBuilderCoversAllSlotsCanonicalOrderAndDistinctIds) {
  const ct::Vec3 points[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const std::uint64_t ids[3]{1, 2, 3};
  const auto first = ft::Triangle(100, 0, points, ids);
  const auto second = ft::Triangle(200, 0, points, ids);

  ct::FixedTriangleFeatureTaskMask forward{0x8000u};
  ASSERT_EQ(ct::BuildFixedTriangleFeatureTaskMask(
                first, second, &forward),
            ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(forward.local_tasks, ct::FixedTriangleFeatureTaskBits);
  for (unsigned slot = 0; slot < 15; ++slot)
    EXPECT_NE(forward.local_tasks &
                  ct::FixedTriangleFeatureTaskBit(slot),
              0u) << slot;

  ct::FixedTriangleFeatureTaskMask reverse;
  ASSERT_EQ(ct::BuildFixedTriangleFeatureTaskMask(
                second, first, &reverse),
            ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(reverse.local_tasks, forward.local_tasks);

  const auto other_source = ft::Triangle(100, 0, points, ids, 2);
  ct::FixedTriangleFeatureTaskMask distinct_source{0x8000u};
  ASSERT_EQ(ct::BuildFixedTriangleFeatureTaskMask(
                first, other_source, &distinct_source),
            ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(distinct_source.local_tasks, 0u);

  ct::FixedTriangleFeatureTaskMask unchanged{0x1234u};
  EXPECT_EQ(ct::BuildFixedTriangleFeatureTaskMask(
                first, first, &unchanged),
            ct::FixedTriangleDiscoveryStatus::IdentityMismatch);
  EXPECT_EQ(unchanged.local_tasks, 0x1234u);
  EXPECT_EQ(ct::BuildFixedTriangleFeatureTaskMask(
                first, second, nullptr),
            ct::FixedTriangleDiscoveryStatus::InvalidInput);
}

TEST(FixedTriangleTaskMask,
     AllFifteenSlotsMapAndIntersectionRemainsComplete) {
  const ct::Vec3 points[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const std::uint64_t ids[3]{1, 2, 3};
  const ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, points, ids),
      ft::Triangle(200, 0, points, ids)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};

  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery, ft::Limits(1));
  for (unsigned slot = 0; slot < 15; ++slot) {
    const ct::FixedTriangleFeatureTaskMask mask{
        ct::FixedTriangleFeatureTaskBit(slot)};
    const auto report = discovery.DiscoverMasked(
        triangles, 2, pair, 1, &mask);
    ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok)
        << slot << ": " << report.message;
    EXPECT_EQ(report.potential_tasks, 15u);
    EXPECT_EQ(report.local_masked_tasks, 1u);
    EXPECT_EQ(report.exact_executed_tasks, 14u);
    EXPECT_EQ(report.feature_tasks, 14u);
  }

  const ct::FixedTriangleFeatureTaskMask all{
      ct::FixedTriangleFeatureTaskBits};
  const auto report = discovery.DiscoverMasked(
      triangles, 2, pair, 1, &all);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.local_masked_tasks, 15u);
  EXPECT_EQ(report.exact_executed_tasks, 0u);
  EXPECT_EQ(discovery.features().count, 0u);
  ASSERT_TRUE(discovery.intersections().complete);
  EXPECT_EQ(discovery.intersections().count, 1u);

  const ct::FixedTriangleFeatureTaskMask high{0x8000u};
  const auto invalid = discovery.DiscoverMasked(
      triangles, 2, pair, 1, &high);
  EXPECT_EQ(invalid.status, ct::FixedTriangleDiscoveryStatus::InvalidInput);
  EXPECT_EQ(invalid.input_pair, 0u);
  EXPECT_EQ(invalid.input_task, 15u);
}

TEST(FixedTriangleTaskMask,
     SharedVertexAndSharedEdgePreserveEveryNonlocalObservation) {
  const ct::Vec3 first_points[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const ct::Vec3 vertex_points[3]{{0, 0, 0}, {-2, 0, 1}, {0, -2, 1}};
  const ct::Vec3 edge_points[3]{{2, 0, 0}, {0, 0, 0}, {1, -2, 0}};
  const std::uint64_t first_ids[3]{1, 2, 3};
  const std::uint64_t vertex_ids[3]{1, 12, 13};
  const std::uint64_t edge_ids[3]{2, 1, 13};
  ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, first_points, first_ids),
      ft::Triangle(200, 0, vertex_points, vertex_ids)};
  const ct::FixedTrianglePair forward[1]{{0, 1}};
  const ct::FixedTrianglePair reverse[1]{{1, 0}};

  ct::FixedTriangleFeatureDiscovery baseline;
  ct::FixedTriangleFeatureDiscovery masked;
  ft::Initialize(&baseline, ft::Limits(1));
  ft::Initialize(&masked, ft::Limits(1));

  ASSERT_EQ(baseline.Discover(triangles, 2, forward, 1).status,
            ct::FixedTriangleDiscoveryStatus::Ok);
  const auto vertex_baseline = ft::Tasks(baseline.features());
  const auto shared_vertex = Mask({0, 1, 6, 8, 12, 14});
  auto report = masked.DiscoverMasked(
      triangles, 2, reverse, 1, &shared_vertex);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.local_masked_tasks, 6u);
  EXPECT_EQ(report.exact_executed_tasks, 9u);
  EXPECT_EQ(ft::Tasks(masked.features()), vertex_baseline);

  triangles[1] = ft::Triangle(200, 0, edge_points, edge_ids);
  ASSERT_EQ(baseline.Discover(triangles, 2, forward, 1).status,
            ct::FixedTriangleDiscoveryStatus::Ok);
  const auto edge_baseline = ft::Tasks(baseline.features());
  const auto shared_edge =
      Mask({0, 1, 2, 3, 6, 7, 8, 9, 11, 12, 13});
  report = masked.DiscoverMasked(
      triangles, 2, reverse, 1, &shared_edge);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok)
      << report.message << " task=" << report.input_task;
  EXPECT_EQ(report.local_masked_tasks, 11u);
  EXPECT_EQ(report.exact_executed_tasks, 4u);
  EXPECT_EQ(ft::Tasks(masked.features()), edge_baseline);
}

TEST(FixedTriangleTaskMask,
     PermutationsUseCanonicalSidesAndLocalOrdinals) {
  const ct::Vec3 first_points[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const ct::Vec3 second_points[3]{{-2, 0, 1}, {0, -2, 1}, {0, 0, 0}};
  const std::uint64_t first_ids[3]{1, 2, 3};
  const std::uint64_t second_ids[3]{12, 13, 1};
  const ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(200, 0, second_points, second_ids),
      ft::Triangle(100, 0, first_points, first_ids)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  // Canonical first is array element 1. Its shared vertex is local 0;
  // canonical second is element 0 and its shared vertex is local 2.
  const auto mask = Mask({0, 5, 7, 8, 13, 14});

  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery, ft::Limits(1));
  const auto report = discovery.DiscoverMasked(
      triangles, 2, pair, 1, &mask);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok)
      << report.message << " task=" << report.input_task;
  EXPECT_EQ(report.local_masked_tasks, 6u);
  EXPECT_EQ(report.exact_executed_tasks, 9u);
}

TEST(FixedTriangleTaskMask,
     CoincidentDistinctIdsAndSameParentRemoteFeaturesStayUnmasked) {
  const ct::Vec3 points[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const std::uint64_t first_ids[3]{1, 2, 3};
  const std::uint64_t second_ids[3]{11, 12, 13};
  const ct::CurrentFixedTriangle coincident[2]{
      ft::Triangle(100, 0, points, first_ids),
      ft::Triangle(200, 0, points, second_ids)};
  const ct::CurrentFixedTriangle same_parent[2]{
      ft::Triangle(100, 0, points, first_ids),
      ft::Triangle(100, 1, points, second_ids)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  const ct::FixedTriangleFeatureTaskMask none{};
  const ct::FixedTriangleFeatureTaskMask forged{
      ct::FixedTriangleFeatureTaskBit(0)};

  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery, ft::Limits(1));
  for (const auto* triangles : {coincident, same_parent}) {
    auto report = discovery.DiscoverMasked(
        triangles, 2, pair, 1, &none);
    ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
    EXPECT_EQ(report.local_masked_tasks, 0u);
    EXPECT_EQ(report.exact_executed_tasks, 15u);
    EXPECT_EQ(discovery.features().count, 15u);
    report = discovery.DiscoverMasked(
        triangles, 2, pair, 1, &forged);
    EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::InvalidInput);
    EXPECT_EQ(report.input_task, 0u);
  }
}

TEST(FixedTriangleTaskMask,
     ExecutedNonlocalCapsPassExactlyAndMinusOnePublishesNoPrefix) {
  const ct::Vec3 first_points[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const ct::Vec3 second_points[3]{{0, 0, 0}, {-2, 0, 1}, {0, -2, 1}};
  const std::uint64_t first_ids[3]{1, 2, 3};
  const std::uint64_t second_ids[3]{1, 12, 13};
  const ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, first_points, first_ids),
      ft::Triangle(200, 0, second_points, second_ids)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  const auto local = Mask({0, 1, 6, 8, 12, 14});

  ct::FixedTriangleFeatureDiscovery exact;
  ft::Initialize(&exact, ft::Limits(1, 9, 9, 1, 1));
  auto report = exact.DiscoverMasked(
      triangles, 2, pair, 1, &local);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.raw_feature_candidates, 9u);
  EXPECT_EQ(report.exact_executed_tasks, 9u);
  EXPECT_EQ(exact.features().count, 9u);

  ct::FixedTriangleFeatureDiscovery raw_short;
  ft::Initialize(&raw_short, ft::Limits(1, 8, 8, 1, 1));
  report = raw_short.DiscoverMasked(
      triangles, 2, pair, 1, &local);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::ResourceLimit);
  EXPECT_EQ(report.raw_feature_candidates, 9u);
  EXPECT_EQ(report.exact_executed_tasks, 0u);
  EXPECT_FALSE(raw_short.features().complete);

  ct::FixedTriangleFeatureDiscovery publication_short;
  ft::Initialize(&publication_short, ft::Limits(1, 9, 8, 1, 1));
  report = publication_short.DiscoverMasked(
      triangles, 2, pair, 1, &local);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::ResourceLimit);
  EXPECT_EQ(report.exact_executed_tasks, 9u);
  EXPECT_EQ(report.feature_candidates, 9u);
  EXPECT_FALSE(publication_short.features().complete);
}

}  // namespace
