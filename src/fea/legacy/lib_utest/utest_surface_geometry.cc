#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

#include "../lib_src/collision/SurfaceContactGeometry.h"

namespace {
namespace sc = tlfea::contact;

static_assert(std::is_trivially_copyable<sc::TriangleGeometry>::value,
              "Geometry records must pass directly to CUDA kernels");
static_assert(std::is_trivially_copyable<sc::SegmentPairGeometry>::value,
              "Geometry results must not own host state");

void ExpectPoint(sc::Vec3 actual, sc::Vec3 expected, double tolerance = 1e-12) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

void ExpectFeature(sc::FeatureKey actual, sc::FeatureKind kind,
                   std::uint64_t first, std::uint64_t second = 0) {
  EXPECT_EQ(actual.kind, kind);
  EXPECT_EQ(actual.first, first);
  EXPECT_EQ(actual.second, second);
}

sc::TriangleGeometry UnitTriangle() {
  return {{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, {10, 11, 12}, 90};
}

TEST(SurfaceGeometrySegment, InteriorAndEndpointsHaveCorrectCoordinates) {
  const sc::SegmentGeometry segment{{{1, 2, 3}, {3, 2, 3}}, {30, 20}};
  sc::SegmentPointGeometry result;
  ASSERT_EQ(sc::ClosestPointOnSegment({1.5, 5, 3}, segment, &result), sc::Status::kOk);
  ExpectPoint(result.point, {1.5, 2, 3});
  EXPECT_DOUBLE_EQ(result.parameter, 0.25);
  EXPECT_DOUBLE_EQ(result.distance, 3);
  ExpectFeature(result.feature, sc::FeatureKind::kEdge, 20, 30);
  ASSERT_EQ(sc::ClosestPointOnSegment({-2, 2, 3}, segment, &result), sc::Status::kOk);
  EXPECT_DOUBLE_EQ(result.parameter, 0);
  ExpectFeature(result.feature, sc::FeatureKind::kVertex, 30);
  ASSERT_EQ(sc::ClosestPointOnSegment({7, 2, 3}, segment, &result), sc::Status::kOk);
  EXPECT_DOUBLE_EQ(result.parameter, 1);
  ExpectFeature(result.feature, sc::FeatureKind::kVertex, 20);
}

TEST(SurfaceGeometrySegment, CollapsedSegmentUsesStableVertexAndFiniteDistance) {
  sc::SegmentGeometry segment{{{1, 2, 3}, {1, 2, 3}}, {30, 20}};
  sc::SegmentPointGeometry result;
  ASSERT_EQ(sc::ClosestPointOnSegment({1, 5, 7}, segment, &result), sc::Status::kOk);
  EXPECT_TRUE(result.degenerate);
  EXPECT_DOUBLE_EQ(result.distance, 5);
  EXPECT_DOUBLE_EQ(result.parameter, 1);
  ExpectFeature(result.feature, sc::FeatureKind::kVertex, 20);
  std::swap(segment.vertices[0], segment.vertices[1]);
  std::swap(segment.vertex_ids[0], segment.vertex_ids[1]);
  ASSERT_EQ(sc::ClosestPointOnSegment({1, 5, 7}, segment, &result), sc::Status::kOk);
  EXPECT_DOUBLE_EQ(result.parameter, 0);
  ExpectFeature(result.feature, sc::FeatureKind::kVertex, 20);
}

TEST(SurfaceGeometryTriangle, InteriorProjectionTracksWindingAndBothSides) {
  auto triangle = UnitTriangle();
  sc::TrianglePointGeometry result;
  for (double z : {-2.0, 0.0, 2.0}) {
    ASSERT_EQ(sc::ClosestPointOnTriangle({0.25, 0.25, z}, triangle, &result), sc::Status::kOk);
    ExpectPoint(result.point, {0.25, 0.25, 0});
    EXPECT_DOUBLE_EQ(result.distance, std::abs(z));
    EXPECT_DOUBLE_EQ(result.signed_plane_distance, z);
    EXPECT_DOUBLE_EQ(result.weights[0], 0.5);
    EXPECT_DOUBLE_EQ(result.weights[1], 0.25);
    EXPECT_DOUBLE_EQ(result.weights[2], 0.25);
    ExpectPoint(result.face_normal, {0, 0, 1});
    ExpectFeature(result.feature, sc::FeatureKind::kFace, 90);
    EXPECT_FALSE(result.degenerate);
  }
  std::swap(triangle.vertices[1], triangle.vertices[2]);
  std::swap(triangle.vertex_ids[1], triangle.vertex_ids[2]);
  ASSERT_EQ(sc::ClosestPointOnTriangle({0.2, 0.3, 2}, triangle, &result), sc::Status::kOk);
  ExpectPoint(result.point, {0.2, 0.3, 0});
  ExpectPoint(result.face_normal, {0, 0, -1});
  EXPECT_DOUBLE_EQ(result.signed_plane_distance, -2);
  EXPECT_NEAR(result.weights[1], 0.3, 1e-14);
  EXPECT_NEAR(result.weights[2], 0.2, 1e-14);
}

TEST(SurfaceGeometryTriangle, OutsideProjectionFindsEachBoundaryFeature) {
  const auto triangle = UnitTriangle();
  struct Case {
    sc::Vec3 query;
    sc::Vec3 closest;
    sc::FeatureKind kind;
    std::uint64_t first;
    std::uint64_t second;
  };
  const Case cases[] = {
      {{-1, -1, 0}, {0, 0, 0}, sc::FeatureKind::kVertex, 10, 0},
      {{2, -1, 0}, {1, 0, 0}, sc::FeatureKind::kVertex, 11, 0},
      {{-1, 2, 0}, {0, 1, 0}, sc::FeatureKind::kVertex, 12, 0},
      {{0.25, -1, 0}, {0.25, 0, 0}, sc::FeatureKind::kEdge, 10, 11},
      {{-1, 0.25, 0}, {0, 0.25, 0}, sc::FeatureKind::kEdge, 10, 12},
      {{1, 1, 0}, {0.5, 0.5, 0}, sc::FeatureKind::kEdge, 11, 12},
  };
  for (const auto& value : cases) {
    sc::TrianglePointGeometry result;
    ASSERT_EQ(sc::ClosestPointOnTriangle(value.query, triangle, &result), sc::Status::kOk);
    ExpectPoint(result.point, value.closest);
    ExpectFeature(result.feature, value.kind, value.first, value.second);
    sc::Vec3 reconstructed;
    for (int i = 0; i < 3; ++i) {
      EXPECT_GE(result.weights[i], 0);
      EXPECT_LE(result.weights[i], 1);
      reconstructed = sc::Add(reconstructed, sc::Scale(triangle.vertices[i], result.weights[i]));
    }
    ExpectPoint(result.point, reconstructed);
    EXPECT_NEAR(result.weights[0] + result.weights[1] + result.weights[2], 1, 1e-14);
  }
}

TEST(SurfaceGeometryTriangle, DegenerateFacesReduceToEdgesOrPointsExplicitly) {
  sc::TriangleGeometry triangle{{{0, 0, 0}, {2, 0, 0}, {1, 0, 0}}, {10, 11, 12}, 90};
  sc::TrianglePointGeometry result;
  ASSERT_EQ(sc::ClosestPointOnTriangle({1.5, 3, 0}, triangle, &result), sc::Status::kOk);
  EXPECT_TRUE(result.degenerate);
  ExpectPoint(result.point, {1.5, 0, 0});
  EXPECT_DOUBLE_EQ(result.distance, 3);
  ExpectPoint(result.face_normal, {});
  triangle.vertices[1] = triangle.vertices[0];
  triangle.vertices[2] = triangle.vertices[0];
  ASSERT_EQ(sc::ClosestPointOnTriangle({0, 3, 4}, triangle, &result), sc::Status::kOk);
  EXPECT_TRUE(result.degenerate);
  EXPECT_DOUBLE_EQ(result.distance, 5);
  ExpectFeature(result.feature, sc::FeatureKind::kVertex, 10);

  // The numerical aspect-ratio threshold is explicit and scale-relative.
  triangle.vertices[1] = {1, 0, 0};
  triangle.vertices[2] = {0.5, 1e-16, 0};
  ASSERT_EQ(sc::ClosestPointOnTriangle({0.5, 0, 1}, triangle, &result), sc::Status::kOk);
  EXPECT_TRUE(result.degenerate);
}

TEST(SurfaceGeometryTriangle, SharedMeshSeamHasCommonKeyAndContinuousDistance) {
  const sc::TriangleGeometry first{{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}, {10, 11, 12}, 80};
  const sc::TriangleGeometry second{{{0, 0, 0}, {1, 1, 0}, {0, 1, 0}}, {10, 12, 13}, 81};
  sc::TrianglePointGeometry a, b;
  ASSERT_EQ(sc::ClosestPointOnTriangle({0.5, 0.5, 0.25}, first, &a), sc::Status::kOk);
  ASSERT_EQ(sc::ClosestPointOnTriangle({0.5, 0.5, 0.25}, second, &b), sc::Status::kOk);
  ExpectFeature(a.feature, sc::FeatureKind::kEdge, 10, 12);
  ExpectFeature(b.feature, sc::FeatureKind::kEdge, 10, 12);
  ExpectPoint(a.point, b.point);
  for (double dx : {-1e-5, -1e-9, 0.0, 1e-9, 1e-5}) {
    const sc::Vec3 query{0.5 + dx, 0.5, 0.25};
    ASSERT_EQ(sc::ClosestPointOnTriangle(query, first, &a), sc::Status::kOk);
    ASSERT_EQ(sc::ClosestPointOnTriangle(query, second, &b), sc::Status::kOk);
    const auto& closest = a.distance < b.distance ? a : b;
    EXPECT_NEAR(closest.distance, 0.25, 1e-14);
    // Select analytically which triangle contains the projection to avoid
    // using indistinguishable rounded distances as a feature-ownership rule.
    ExpectPoint(dx >= 0 ? a.point : b.point, {query.x, query.y, 0});
  }
}

TEST(SurfaceGeometryTriangle, LargeNormalDistanceRetainsInteriorProjection) {
  sc::TrianglePointGeometry result;
  ASSERT_EQ(sc::ClosestPointOnTriangle({0.25, 0.25, 1e12}, UnitTriangle(), &result), sc::Status::kOk);
  ExpectPoint(result.point, {0.25, 0.25, 0});
  ExpectFeature(result.feature, sc::FeatureKind::kFace, 90);
}

TEST(SurfaceGeometryEdges, InteriorSkewSolutionIsSymmetricAndReconstructible) {
  sc::SegmentGeometry a{{{-1, 0, 0}, {1, 0, 0}}, {10, 11}};
  const sc::SegmentGeometry b{{{0, -1, 0.5}, {0, 1, 0.5}}, {20, 21}};
  sc::SegmentPairGeometry result, swapped;
  ASSERT_EQ(sc::ClosestPointsBetweenSegments(a, b, &result), sc::Status::kOk);
  EXPECT_FALSE(result.parallel);
  EXPECT_DOUBLE_EQ(result.parameter_a, 0.5);
  EXPECT_DOUBLE_EQ(result.parameter_b, 0.5);
  EXPECT_DOUBLE_EQ(result.distance, 0.5);
  ExpectPoint(result.point_a, {0, 0, 0});
  ExpectPoint(result.point_b, {0, 0, 0.5});
  ExpectFeature(result.feature_a, sc::FeatureKind::kEdge, 10, 11);
  ExpectFeature(result.feature_b, sc::FeatureKind::kEdge, 20, 21);
  ASSERT_EQ(sc::ClosestPointsBetweenSegments(b, a, &swapped), sc::Status::kOk);
  ExpectPoint(swapped.point_a, result.point_b);
  ExpectPoint(swapped.point_b, result.point_a);
  std::swap(a.vertices[0], a.vertices[1]);
  std::swap(a.vertex_ids[0], a.vertex_ids[1]);
  ASSERT_EQ(sc::ClosestPointsBetweenSegments(a, b, &swapped), sc::Status::kOk);
  ExpectPoint(swapped.point_a, result.point_a);
  ExpectFeature(swapped.feature_a, sc::FeatureKind::kEdge, 10, 11);
}

TEST(SurfaceGeometryEdges, BoundedEndpointsReplaceInvalidInfiniteLineSolution) {
  const sc::SegmentGeometry a{{{0, 0, 0}, {1, 0, 0}}, {10, 11}};
  const sc::SegmentGeometry b{{{2, 1, 0}, {2, 2, 0}}, {20, 21}};
  sc::SegmentPairGeometry result;
  ASSERT_EQ(sc::ClosestPointsBetweenSegments(a, b, &result), sc::Status::kOk);
  EXPECT_DOUBLE_EQ(result.parameter_a, 1);
  EXPECT_DOUBLE_EQ(result.parameter_b, 0);
  ExpectPoint(result.point_a, {1, 0, 0});
  ExpectPoint(result.point_b, {2, 1, 0});
  EXPECT_NEAR(result.distance, std::sqrt(2.0), 1e-14);
  ExpectFeature(result.feature_a, sc::FeatureKind::kVertex, 11);
  ExpectFeature(result.feature_b, sc::FeatureKind::kVertex, 20);
}

TEST(SurfaceGeometryEdges, ParallelOverlappingDisjointAndCollinearCases) {
  const sc::SegmentGeometry a{{{0, 0, 0}, {2, 0, 0}}, {10, 11}};
  sc::SegmentGeometry b{{{1, 1, 0}, {3, 1, 0}}, {20, 21}};
  sc::SegmentPairGeometry result, swapped;
  ASSERT_EQ(sc::ClosestPointsBetweenSegments(a, b, &result), sc::Status::kOk);
  EXPECT_TRUE(result.parallel);
  EXPECT_DOUBLE_EQ(result.distance, 1);
  EXPECT_DOUBLE_EQ(result.point_a.x, result.point_b.x);
  ASSERT_EQ(sc::ClosestPointsBetweenSegments(b, a, &swapped), sc::Status::kOk);
  ExpectPoint(result.point_a, swapped.point_b);
  ExpectPoint(result.point_b, swapped.point_a);
  b.vertices[0] = {3, 1, 0};
  b.vertices[1] = {4, 1, 0};
  ASSERT_EQ(sc::ClosestPointsBetweenSegments(a, b, &result), sc::Status::kOk);
  ExpectPoint(result.point_a, {2, 0, 0});
  ExpectPoint(result.point_b, {3, 1, 0});
  EXPECT_NEAR(result.distance, std::sqrt(2.0), 1e-14);
  b.vertices[0] = {1, 0, 0};
  b.vertices[1] = {3, 0, 0};
  ASSERT_EQ(sc::ClosestPointsBetweenSegments(a, b, &result), sc::Status::kOk);
  EXPECT_DOUBLE_EQ(result.distance, 0);
  ExpectPoint(result.point_a, result.point_b);
}

TEST(SurfaceGeometryEdges, NearlyParallelCrossingAvoidsGramCancellation) {
  const sc::SegmentGeometry a{{{-1, 0, 0}, {1, 0, 0}}, {10, 11}};
  for (double slope : {1e-8, 1e-12, 1e-16}) {
    const sc::SegmentGeometry b{{{-1, slope, 0}, {1, -slope, 0}}, {20, 21}};
    sc::SegmentPairGeometry result;
    ASSERT_EQ(sc::ClosestPointsBetweenSegments(a, b, &result), sc::Status::kOk);
    EXPECT_FALSE(result.parallel);
    EXPECT_NEAR(result.parameter_a, 0.5, 1e-14);
    EXPECT_NEAR(result.parameter_b, 0.5, 1e-14);
    EXPECT_NEAR(result.distance, 0, 1e-20);
  }
}

TEST(SurfaceGeometryEdges, ZeroLengthSegmentsAreExplicitAndFinite) {
  sc::SegmentGeometry a{{{0.5, 0, 0}, {0.5, 0, 0}}, {10, 11}};
  sc::SegmentGeometry b{{{0, 1, 0}, {1, 1, 0}}, {20, 21}};
  sc::SegmentPairGeometry result;
  ASSERT_EQ(sc::ClosestPointsBetweenSegments(a, b, &result), sc::Status::kOk);
  EXPECT_TRUE(result.degenerate_a);
  EXPECT_FALSE(result.degenerate_b);
  EXPECT_DOUBLE_EQ(result.distance, 1);
  ExpectPoint(result.point_b, {0.5, 1, 0});
  b.vertices[0] = b.vertices[1] = {0.5, 3, 4};
  ASSERT_EQ(sc::ClosestPointsBetweenSegments(a, b, &result), sc::Status::kOk);
  EXPECT_TRUE(result.degenerate_b);
  EXPECT_DOUBLE_EQ(result.distance, 5);
  ExpectFeature(result.feature_a, sc::FeatureKind::kVertex, 10);
  ExpectFeature(result.feature_b, sc::FeatureKind::kVertex, 20);
}

TEST(SurfaceGeometryTransforms, ScalingTranslationAndRotationPreserveSolutions) {
  for (double scale : {1e-9, 1.0, 1e9}) {
    for (const sc::Vec3 shift : {sc::Vec3{}, sc::Vec3{3 * scale, -7 * scale, 2 * scale}}) {
      auto transform = [=](sc::Vec3 p) {
        // Cyclic coordinate permutation is a proper rotation.
        return sc::Add(sc::Scale({p.y, p.z, p.x}, scale), shift);
      };
      auto triangle = UnitTriangle();
      for (auto& p : triangle.vertices)
        p = transform(p);
      sc::TrianglePointGeometry point;
      ASSERT_EQ(sc::ClosestPointOnTriangle(transform({0.25, 0.25, 2}), triangle, &point), sc::Status::kOk);
      ExpectPoint(point.point, transform({0.25, 0.25, 0}), scale * 1e-12);
      EXPECT_NEAR(point.distance, 2 * scale, scale * 1e-12);
      EXPECT_NEAR(point.weights[0], 0.5, 1e-12);
      EXPECT_FALSE(point.degenerate);
      const sc::SegmentGeometry a{{transform({-1, 0, 0}), transform({1, 0, 0})}, {10, 11}};
      const sc::SegmentGeometry b{{transform({0, -1, 0.5}), transform({0, 1, 0.5})}, {20, 21}};
      sc::SegmentPairGeometry pair;
      ASSERT_EQ(sc::ClosestPointsBetweenSegments(a, b, &pair), sc::Status::kOk);
      EXPECT_NEAR(pair.distance, 0.5 * scale, scale * 1e-12);
      EXPECT_NEAR(pair.parameter_a, 0.5, 1e-12);
      EXPECT_NEAR(pair.parameter_b, 0.5, 1e-12);
    }
  }
  auto translated = UnitTriangle();
  for (auto& p : translated.vertices)
    p = sc::Add(p, {1e9, -1e9, 1e9});
  sc::TrianglePointGeometry result;
  ASSERT_EQ(sc::ClosestPointOnTriangle({1e9 + 0.25, -1e9 + 0.25, 1e9 + 2}, translated, &result), sc::Status::kOk);
  EXPECT_DOUBLE_EQ(result.distance, 2);
  EXPECT_DOUBLE_EQ(result.weights[0], 0.5);
}

TEST(SurfaceGeometryValidation, NonfiniteAndUnrepresentableGeometryFailsExplicitly) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  sc::TrianglePointGeometry triangle_result;
  EXPECT_EQ(sc::ClosestPointOnTriangle({nan, 0, 0}, UnitTriangle(), &triangle_result), sc::Status::kInvalidArgument);
  auto triangle = UnitTriangle();
  triangle.vertices[1].x = inf;
  EXPECT_EQ(sc::ClosestPointOnTriangle({}, triangle, &triangle_result), sc::Status::kInvalidArgument);
  const sc::SegmentGeometry a{{{0, 0, 0}, {1, 0, 0}}, {10, 11}};
  sc::SegmentGeometry b{{{0, 1, 0}, {1, inf, 0}}, {20, 21}};
  sc::SegmentPairGeometry pair;
  EXPECT_EQ(sc::ClosestPointsBetweenSegments(a, b, &pair), sc::Status::kInvalidArgument);
  b.vertices[0] = {-DBL_MAX, 0, 0};
  b.vertices[1] = {DBL_MAX, 0, 0};
  EXPECT_EQ(sc::ClosestPointsBetweenSegments(a, b, &pair), sc::Status::kNonFiniteResult);
  EXPECT_EQ(sc::ClosestPointsBetweenSegments(a, b, nullptr), sc::Status::kInvalidArgument);
}

}  // namespace
