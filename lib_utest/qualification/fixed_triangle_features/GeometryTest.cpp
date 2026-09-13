// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <limits>

namespace {
namespace ft = fixed_triangle_test;
namespace ct = tlfea::contact;

TEST(FixedTriangleFeatures, CompleteVFBothOrientationsAndEEAreDeterministic) {
  const ct::Vec3 pa[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const ct::Vec3 pb[3]{{0.25, 0.25, 1}, {2.25, 0.25, 1},
                       {0.25, 2.25, 1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, pa, ia), ft::Triangle(200, 0, pb, ib)};
  const ct::FixedTrianglePair forward[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  auto report = discovery.Discover(triangles, 2, forward, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 15u);
  EXPECT_EQ(report.raw_feature_candidates, 15u);
  EXPECT_EQ(report.feature_candidates, 15u);
  ASSERT_TRUE(discovery.features().complete);
  ASSERT_EQ(discovery.features().count, 15u);

  unsigned vf = 0, ee = 0, face_a = 0, face_b = 0;
  for (std::size_t i = 0; i < discovery.features().count; ++i) {
    const auto& value = discovery.features().data[i];
    if (value.kind == ct::FixedTriangleCandidateKind::VertexFace) {
      ++vf;
      face_a += value.face.parent_eid == 100;
      face_b += value.face.parent_eid == 200;
    } else {
      ++ee;
    }
  }
  EXPECT_EQ(vf, 6u);
  EXPECT_EQ(ee, 9u);
  EXPECT_EQ(face_a, 3u);
  EXPECT_EQ(face_b, 3u);
  const auto expected = ft::Tasks(discovery.features());

  std::swap(triangles[0], triangles[1]);
  const ct::FixedTrianglePair reverse[2]{{0, 1}, {1, 0}};
  report = discovery.Discover(triangles, 2, reverse, 2);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 30u);
  EXPECT_EQ(report.raw_feature_candidates, 30u);
  EXPECT_EQ(report.feature_candidates, 15u);
  EXPECT_EQ(ft::Tasks(discovery.features()), expected);
}

TEST(FixedTriangleFeatures, ExclusionsAreOnlyLocalFeatureIncidence) {
  const ct::Vec3 pa[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const ct::Vec3 pb[3]{{0, 0, 0}, {-2, 0, 1}, {0, -2, 1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{1, 12, 13};
  // Same parent EID is deliberate.  Only the shared vertex is locally
  // incident; no whole-parent or same-PID concept is available to this module.
  ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, pa, ia), ft::Triangle(100, 1, pb, ib)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  const auto report = discovery.Discover(triangles, 2, pair, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 15u);
  // Two incident VF tasks and four edge pairs supported at the common vertex
  // are excluded.  All nine remote feature tasks remain.
  EXPECT_EQ(report.raw_feature_candidates, 9u);
  EXPECT_EQ(report.feature_candidates, 9u);
  unsigned vf = 0, ee = 0;
  for (std::size_t i = 0; i < discovery.features().count; ++i) {
    vf += discovery.features().data[i].kind ==
          ct::FixedTriangleCandidateKind::VertexFace;
    ee += discovery.features().data[i].kind ==
          ct::FixedTriangleCandidateKind::EdgeEdge;
  }
  EXPECT_EQ(vf, 4u);
  EXPECT_EQ(ee, 5u);
}

TEST(FixedTriangleFeatures, DegenerateCurrentTriangleRejectsAllPublication) {
  const ct::Vec3 pa[3]{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
  const ct::Vec3 pb[3]{{0, 0, 1}, {1, 0, 1}, {0, 1, 1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  const ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, pa, ia), ft::Triangle(200, 0, pb, ib)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  const auto report = discovery.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status,
            ct::FixedTriangleDiscoveryStatus::DegenerateTriangle);
  EXPECT_EQ(report.input_pair, 0u);
  EXPECT_FALSE(discovery.features().complete);
  EXPECT_FALSE(discovery.intersections().complete);
}

TEST(FixedTriangleFeatures,
     DegeneracyBoundaryIsClosedAndNextafterAboveRetriesSuccessfully) {
  const double boundary = 64 * DBL_EPSILON;
  const ct::Vec3 degenerate[3]{{0, 0, 0}, {1, 0, 0},
                                {1, boundary, 0}};
  const ct::Vec3 regular[3]{
      {0, 0, 0}, {1, 0, 0},
      {1, std::nextafter(boundary,
                         std::numeric_limits<double>::infinity()), 0}};
  const ct::Vec3 other[3]{{0, 0, 1}, {1, 0, 1}, {0, 1, 1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, degenerate, ia),
      ft::Triangle(200, 0, other, ib)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);

  auto report = discovery.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status,
            ct::FixedTriangleDiscoveryStatus::DegenerateTriangle);
  EXPECT_FALSE(discovery.features().complete);
  triangles[0] = ft::Triangle(100, 0, regular, ia);
  report = discovery.Discover(triangles, 2, pair, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 15u);
  EXPECT_EQ(discovery.features().count, 15u);
}

TEST(FixedTriangleFeatures,
     SharedImmutableVertexWithDifferentCurrentValueRejectsBeforeTasks) {
  const ct::Vec3 pa[3]{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  const ct::Vec3 pb[3]{{0, 0, 0.25}, {-1, 0, 1}, {0, -1, 1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{1, 12, 13};
  const ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, pa, ia), ft::Triangle(200, 0, pb, ib)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  const auto report = discovery.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status,
            ct::FixedTriangleDiscoveryStatus::IdentityMismatch);
  EXPECT_EQ(report.feature_tasks, 0u);
  EXPECT_FALSE(discovery.features().complete);
}

}  // namespace
