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
  EXPECT_EQ(report.triangle_references, 2u);
  EXPECT_EQ(report.triangles, 2u);
  EXPECT_EQ(report.vertex_references, 6u);
  EXPECT_EQ(report.vertices, 6u);
  EXPECT_EQ(report.edge_references, 6u);
  EXPECT_EQ(report.edges, 6u);
  EXPECT_EQ(report.raw_feature_candidates, 15u);
  EXPECT_EQ(report.feature_candidates, 15u);
  ASSERT_TRUE(discovery.features().complete);
  ASSERT_EQ(discovery.features().count, 15u);

  unsigned vf = 0, ee = 0, face_a = 0, face_b = 0;
  for (std::size_t i = 0; i < discovery.features().count; ++i) {
    const auto& value = discovery.features().data[i];
    if (value.key.kind == ct::FixedTriangleCandidateKind::VertexFace) {
      ++vf;
      face_a += value.local_features[0] == 3;
      face_b += value.local_features[1] == 3;
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
    vf += discovery.features().data[i].key.kind ==
          ct::FixedTriangleCandidateKind::VertexFace;
    ee += discovery.features().data[i].key.kind ==
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
  EXPECT_EQ(report.input_pair, SIZE_MAX);
  EXPECT_EQ(report.input_task, SIZE_MAX);
  EXPECT_EQ(report.arithmetic_reason,
            ct::FixedTriangleArithmeticReason::TriangleValidation);
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

TEST(FixedTriangleFeatures,
     GlobalLedgerRejectsConflictsOutsideDirectPairContexts) {
  const ct::Vec3 a[3]{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  const ct::Vec3 b[3]{{0, 0, 2}, {1, 0, 2}, {0, 1, 2}};
  const ct::Vec3 c[3]{{0, 0, 0.25}, {-1, 0, 0}, {0, -1, 0}};
  const ct::Vec3 d[3]{{0, 0, 3}, {-1, 0, 3}, {0, -1, 3}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  const std::uint64_t ic[3]{1, 22, 23};
  const std::uint64_t id[3]{31, 32, 33};
  ct::CurrentFixedTriangle triangles[4]{
      ft::Triangle(100, 0, a, ia), ft::Triangle(200, 0, b, ib),
      ft::Triangle(300, 0, c, ic), ft::Triangle(400, 0, d, id)};
  const ct::FixedTrianglePair pairs[2]{{0, 1}, {2, 3}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery, ft::Limits(2));
  auto report = discovery.Discover(triangles, 4, pairs, 2);
  EXPECT_EQ(report.status,
            ct::FixedTriangleDiscoveryStatus::IdentityMismatch);
  EXPECT_EQ(report.feature_tasks, 0u);

  triangles[2] = triangles[0];
  triangles[2].vertices[1].x = 2;
  report = discovery.Discover(triangles, 4, pairs, 2);
  EXPECT_EQ(report.status,
            ct::FixedTriangleDiscoveryStatus::IdentityMismatch);
  EXPECT_EQ(report.feature_tasks, 0u);
}

TEST(FixedTriangleFeatures,
     EdgeLedgerRequiresCanonicalEndpointsAndBoundarySemantics) {
  const ct::Vec3 a[3]{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  const ct::Vec3 b[3]{{0, 0, 1}, {1, 0, 1}, {0, 1, 1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, a, ia), ft::Triangle(200, 0, b, ib)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);

  std::swap(triangles[0].edge_keys[0].endpoints[0],
            triangles[0].edge_keys[0].endpoints[1]);
  auto report = discovery.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::InvalidInput);

  triangles[0] = ft::Triangle(100, 0, a, ia);
  triangles[0].edge_keys[0].parent_eid = 100;
  report = discovery.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::InvalidInput);

  triangles[0] = ft::Triangle(100, 0, a, ia);
  triangles[0].edge_keys[0].parent_boundary = false;
  triangles[0].edge_keys[0].parent_eid = 999;
  report = discovery.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::InvalidInput);
}

TEST(FixedTriangleFeatures,
     CanonicalVertexFaceStratumDeduplicatesProducingFacetSeam) {
  const ct::Vec3 a0[3]{{0, 0, 1}, {3, 0, 1}, {0, 3, 1}};
  const ct::Vec3 a1[3]{{0, 0, 1}, {-3, 0, 1}, {0, -3, 1}};
  const ct::Vec3 face[3]{{-2, -2, 0}, {2, -2, 0}, {0, 2, 0}};
  const std::uint64_t ia0[3]{1, 2, 3};
  const std::uint64_t ia1[3]{1, 4, 5};
  const std::uint64_t ib[3]{11, 12, 13};
  ct::CurrentFixedTriangle triangles[3]{
      ft::Triangle(100, 0, a0, ia0),
      ft::Triangle(200, 0, a1, ia1),
      ft::Triangle(300, 0, face, ib)};
  const ct::FixedTrianglePair pairs[2]{{0, 2}, {1, 2}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery, ft::Limits(2));
  const auto report = discovery.Discover(triangles, 3, pairs, 2);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 30u);
  EXPECT_EQ(report.raw_feature_candidates, 30u);
  EXPECT_EQ(report.feature_candidates, 29u);

  std::size_t matches = 0;
  const ct::FixedTriangleFeatureCandidate* selected = nullptr;
  for (std::size_t i = 0; i < discovery.features().count; ++i) {
    const auto& value = discovery.features().data[i];
    if (value.key.kind == ct::FixedTriangleCandidateKind::VertexFace &&
        ft::Same(value.key.vertex_face.vertex, ft::Vertex(1)) &&
        value.key.vertex_face.target.kind ==
            ct::FixedTriangleStratumKind::Face &&
        value.key.vertex_face.target.face.parent_eid == 300) {
      ++matches;
      selected = &value;
    }
  }
  ASSERT_EQ(matches, 1u);
  ASSERT_NE(selected, nullptr);
  EXPECT_EQ(selected->triangles[0].parent_eid, 100u);
}

TEST(FixedTriangleFeatures,
     CanonicalTargetEdgeAndEdgePairDeduplicateAcrossFacetSeam) {
  const ct::Vec3 source[3]{{0, 0, 1}, {2, 0, 2}, {0, 2, 2}};
  const ct::Vec3 upper[3]{{-1, 0, 0}, {1, 0, 0}, {0, 2, 0}};
  const ct::Vec3 lower[3]{{1, 0, 0}, {-1, 0, 0}, {0, -2, 0}};
  const std::uint64_t source_ids[3]{50, 51, 52};
  const std::uint64_t upper_ids[3]{11, 12, 13};
  const std::uint64_t lower_ids[3]{12, 11, 14};
  const ct::CurrentFixedTriangle triangles[3]{
      ft::Triangle(100, 0, source, source_ids),
      ft::Triangle(200, 0, upper, upper_ids),
      ft::Triangle(300, 0, lower, lower_ids)};
  const ct::FixedTrianglePair pairs[2]{{0, 1}, {0, 2}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery, ft::Limits(2));
  const auto report = discovery.Discover(triangles, 3, pairs, 2);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 30u);

  std::size_t vf_matches = 0;
  std::size_t ee_shared_target = 0;
  for (std::size_t i = 0; i < discovery.features().count; ++i) {
    const auto& value = discovery.features().data[i];
    if (value.key.kind == ct::FixedTriangleCandidateKind::VertexFace &&
        ft::Same(value.key.vertex_face.vertex, ft::Vertex(50)) &&
        value.key.vertex_face.target.kind ==
            ct::FixedTriangleStratumKind::Edge &&
        ft::Same(value.key.vertex_face.target.edge,
                 triangles[1].edge_keys[0]))
      ++vf_matches;
    if (value.key.kind == ct::FixedTriangleCandidateKind::EdgeEdge &&
        (ft::Same(value.key.edge_edge.edges[0],
                  triangles[1].edge_keys[0]) ||
         ft::Same(value.key.edge_edge.edges[1],
                  triangles[1].edge_keys[0])))
      ++ee_shared_target;
  }
  EXPECT_EQ(vf_matches, 1u);
  EXPECT_EQ(ee_shared_target, 3u);
  EXPECT_LT(report.feature_candidates,
            report.raw_feature_candidates);
}

}  // namespace
