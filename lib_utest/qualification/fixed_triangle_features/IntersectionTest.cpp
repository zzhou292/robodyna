// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

namespace {
namespace ft = fixed_triangle_test;
namespace ct = tlfea::contact;

ct::FixedTriangleIntersection DiscoverIntersection(
    const ct::CurrentFixedTriangle& a,
    const ct::CurrentFixedTriangle& b) {
  const ct::CurrentFixedTriangle triangles[2]{a, b};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  const auto report = discovery.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(discovery.intersections().count, 1u);
  return discovery.intersections().data[0];
}

ct::CurrentFixedTriangle PermutedTriangle(
    std::uint64_t eid, const ct::Vec3 (&points)[3],
    const std::uint64_t (&ids)[3],
    const std::array<unsigned, 3>& permutation) {
  ct::Vec3 permuted_points[3];
  std::uint64_t permuted_ids[3];
  for (unsigned i = 0; i < 3; ++i) {
    permuted_points[i] = points[permutation[i]];
    permuted_ids[i] = ids[permutation[i]];
  }
  return ft::Triangle(eid, 0, permuted_points, permuted_ids);
}

TEST(FixedTriangleIntersections,
     TransversePiercingIsExplicitWhenAllBoundaryQueriesArePositive) {
  const ct::Vec3 pa[3]{{-2, -2, 0}, {2, -2, 0}, {0, 2, 0}};
  const ct::Vec3 pb[3]{{0, -0.5, -1}, {0, 0.5, 1}, {0, 1, -1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  const ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, pa, ia), ft::Triangle(200, 0, pb, ib)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  const auto report = discovery.Discover(triangles, 2, pair, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_EQ(discovery.intersections().count, 1u);
  EXPECT_EQ(discovery.intersections().data[0].kind,
            ct::FixedTriangleIntersectionKind::Transverse);
  EXPECT_TRUE(
      ct::RequiresIntersectionAdmission(discovery.intersections().data[0]));
  for (std::size_t i = 0; i < discovery.features().count; ++i)
    EXPECT_GT(discovery.features().data[i].distance_m, 0);
}

TEST(FixedTriangleIntersections,
     CoplanarOverlapContainmentAndIndependentTouchAreDistinguished) {
  const ct::Vec3 pa[3]{{0, 0, 0}, {4, 0, 0}, {0, 4, 0}};
  const ct::Vec3 contained[3]{{0.5, 0.5, 0}, {1, 0.5, 0},
                              {0.5, 1, 0}};
  const ct::Vec3 touched[3]{{4, 0, 0}, {5, 0, 0}, {4, 1, 0}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  const std::uint64_t ic[3]{21, 22, 23};
  auto value = DiscoverIntersection(ft::Triangle(100, 0, pa, ia),
                                    ft::Triangle(200, 0, contained, ib));
  EXPECT_EQ(value.kind,
            ct::FixedTriangleIntersectionKind::CoplanarOverlap);
  EXPECT_EQ(value.local_exclusion,
            ct::FixedTriangleLocalExclusion::None);

  value = DiscoverIntersection(ft::Triangle(100, 0, pa, ia),
                               ft::Triangle(300, 0, touched, ic));
  EXPECT_EQ(value.kind, ct::FixedTriangleIntersectionKind::CoplanarTouch);
  EXPECT_EQ(value.local_exclusion,
            ct::FixedTriangleLocalExclusion::None);
}

TEST(FixedTriangleIntersections,
     SharedEdgeTouchIsLocalButCoplanarFoldOverlapIsNot) {
  const ct::Vec3 pa[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const ct::Vec3 adjacent[3]{{2, 0, 0}, {0, 0, 0}, {2, -2, 0}};
  const ct::Vec3 folded[3]{{2, 0, 0}, {0, 0, 0}, {1, 1, 0}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t shared[3]{2, 1, 4};
  auto value = DiscoverIntersection(ft::Triangle(100, 0, pa, ia),
                                    ft::Triangle(200, 0, adjacent, shared));
  EXPECT_EQ(value.kind, ct::FixedTriangleIntersectionKind::CoplanarTouch);
  EXPECT_EQ(value.local_exclusion,
            ct::FixedTriangleLocalExclusion::SharedEdgeOnly);
  EXPECT_FALSE(ct::RequiresIntersectionAdmission(value));

  value = DiscoverIntersection(ft::Triangle(100, 0, pa, ia),
                               ft::Triangle(200, 0, folded, shared));
  EXPECT_EQ(value.kind,
            ct::FixedTriangleIntersectionKind::CoplanarOverlap);
  EXPECT_EQ(value.local_exclusion,
            ct::FixedTriangleLocalExclusion::None);
  EXPECT_TRUE(ct::RequiresIntersectionAdmission(value));
}

TEST(FixedTriangleIntersections,
     SharedCornerDoesNotHideRemoteTransverseIntersection) {
  const ct::Vec3 pa[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const ct::Vec3 pb[3]{{0, 0, 0}, {0.5, 0.5, 1}, {1, 0.5, -1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{1, 12, 13};
  const auto value =
      DiscoverIntersection(ft::Triangle(100, 0, pa, ia),
                           ft::Triangle(200, 0, pb, ib));
  EXPECT_EQ(value.kind, ct::FixedTriangleIntersectionKind::Transverse);
  EXPECT_EQ(value.local_exclusion,
            ct::FixedTriangleLocalExclusion::None);
}

TEST(FixedTriangleIntersections, IdenticalFaceIsTheOnlyWholeFaceExclusion) {
  const ct::Vec3 points[3]{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  const std::uint64_t ids[3]{1, 2, 3};
  const auto triangle = ft::Triangle(100, 0, points, ids);
  const ct::CurrentFixedTriangle triangles[1]{triangle};
  const ct::FixedTrianglePair pair[1]{{0, 0}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  const auto report = discovery.Discover(triangles, 1, pair, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 15u);
  EXPECT_EQ(discovery.features().count, 0u);
  ASSERT_EQ(discovery.intersections().count, 1u);
  EXPECT_EQ(discovery.intersections().data[0].local_exclusion,
            ct::FixedTriangleLocalExclusion::IdenticalFace);
}

TEST(FixedTriangleIntersections,
     CoplanarAndTransverseClassesAreInvariantUnderAllPermutationsAndPairOrder) {
  const std::array<std::array<unsigned, 3>, 6> permutations{{
      {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
      {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}},
  }};
  const ct::Vec3 base[3]{{0, 0, 0}, {4, 0, 0}, {0, 4, 0}};
  const ct::Vec3 contained[3]{{0.5, 0.5, 0}, {1, 0.5, 0},
                              {0.5, 1, 0}};
  const ct::Vec3 touched[3]{{4, 0, 0}, {5, 0, 0}, {4, 1, 0}};
  const ct::Vec3 transverse[3]{{1, 0, -1}, {1, 0, 1},
                               {1, 1, -1}};
  const ct::Vec3 separated[3]{{5, 5, 1}, {6, 5, 1}, {5, 6, 1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  struct Case {
    const ct::Vec3* points;
    bool intersects;
    ct::FixedTriangleIntersectionKind kind;
  };
  const Case cases[]{
      {contained, true, ct::FixedTriangleIntersectionKind::CoplanarOverlap},
      {touched, true, ct::FixedTriangleIntersectionKind::CoplanarTouch},
      {transverse, true, ct::FixedTriangleIntersectionKind::Transverse},
      {separated, false, ct::FixedTriangleIntersectionKind::Transverse},
  };

  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  for (const auto& test : cases) {
    ct::Vec3 other[3]{test.points[0], test.points[1], test.points[2]};
    for (const auto& pa : permutations) {
      for (const auto& pb : permutations) {
        ct::CurrentFixedTriangle triangles[2]{
            PermutedTriangle(100, base, ia, pa),
            PermutedTriangle(200, other, ib, pb)};
        for (unsigned reversed = 0; reversed < 2; ++reversed) {
          const ct::FixedTrianglePair pair[1]{
              {reversed, 1u - reversed}};
          const auto report =
              discovery.Discover(triangles, 2, pair, 1);
          ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
          EXPECT_EQ(report.feature_tasks, 15u);
          ASSERT_EQ(discovery.intersections().count,
                    test.intersects ? 1u : 0u);
          if (test.intersects)
            EXPECT_EQ(discovery.intersections().data[0].kind, test.kind);
        }
      }
    }
  }
}

TEST(FixedTriangleIntersections,
     ExactBoundaryAndAdjacentRepresentableCoordinatesHaveExplicitClasses) {
  const double below = std::nextafter(1.0, 0.0);
  const double above =
      std::nextafter(1.0, std::numeric_limits<double>::infinity());
  const ct::Vec3 base[3]{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  const ct::Vec3 coplanar_overlap[3]{{below, 0, 0}, {2, 0, 0},
                                     {below, 1, 0}};
  const ct::Vec3 coplanar_touch[3]{{1, 0, 0}, {2, 0, 0},
                                   {1, 1, 0}};
  const ct::Vec3 coplanar_separated[3]{{above, 0, 0}, {2, 0, 0},
                                       {above, 1, 0}};
  const ct::Vec3 transverse_inside[3]{{below, 0, -1},
                                      {below, 0, 1},
                                      {below, -1, -1}};
  const ct::Vec3 transverse_touch[3]{{1, 0, -1}, {1, 0, 1},
                                     {1, -1, -1}};
  const ct::Vec3 transverse_outside[3]{{above, 0, -1},
                                       {above, 0, 1},
                                       {above, -1, -1}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};

  struct Case {
    const ct::Vec3* points;
    std::size_t count;
    ct::FixedTriangleIntersectionKind kind;
  };
  const Case cases[]{
      {coplanar_overlap, 1,
       ct::FixedTriangleIntersectionKind::CoplanarOverlap},
      {coplanar_touch, 1,
       ct::FixedTriangleIntersectionKind::CoplanarTouch},
      {coplanar_separated, 0,
       ct::FixedTriangleIntersectionKind::CoplanarTouch},
      {transverse_inside, 1,
       ct::FixedTriangleIntersectionKind::Transverse},
      {transverse_touch, 1,
       ct::FixedTriangleIntersectionKind::Transverse},
      {transverse_outside, 0,
       ct::FixedTriangleIntersectionKind::Transverse},
  };
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  for (const auto& test : cases) {
    ct::Vec3 other[3]{test.points[0], test.points[1], test.points[2]};
    const ct::CurrentFixedTriangle triangles[2]{
        ft::Triangle(100, 0, base, ia),
        ft::Triangle(200, 0, other, ib)};
    const ct::FixedTrianglePair pair[1]{{0, 1}};
    const auto report =
        discovery.Discover(triangles, 2, pair, 1);
    ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_EQ(discovery.intersections().count, test.count);
    if (test.count)
      EXPECT_EQ(discovery.intersections().data[0].kind, test.kind);
  }
}

}  // namespace
