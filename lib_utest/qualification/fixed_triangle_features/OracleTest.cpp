// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include <gtest/gtest.h>

#include <boost/multiprecision/cpp_dec_float.hpp>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

namespace {
namespace ft = fixed_triangle_test;
namespace ct = tlfea::contact;
using Q = boost::multiprecision::cpp_dec_float_100;

struct Q2 {
  Q x, y;
};
struct Q3 {
  Q x, y, z;
};

Q3 Convert(ct::Vec3 p) {
  return {Q(p.x), Q(p.y), Q(p.z)};
}
Q3 operator+(Q3 a, Q3 b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
Q3 operator-(Q3 a, Q3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
Q3 operator*(Q3 a, const Q& b) {
  return {a.x * b, a.y * b, a.z * b};
}
Q Dot(Q3 a, Q3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
Q3 Cross(Q3 a, Q3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}
Q Norm(Q3 a) {
  return sqrt(Dot(a, a));
}

Q PointSegmentDistance(Q3 p, Q3 a, Q3 b) {
  const Q3 edge = b - a;
  Q t = Dot(p - a, edge) / Dot(edge, edge);
  if (t < 0)
    t = 0;
  if (t > 1)
    t = 1;
  return Norm(p - (a + edge * t));
}

Q PointTriangleDistance(Q3 p, const Q3* triangle) {
  const Q3 u = triangle[1] - triangle[0];
  const Q3 v = triangle[2] - triangle[0];
  const Q3 delta = p - triangle[0];
  const Q uu = Dot(u, u);
  const Q uv = Dot(u, v);
  const Q vv = Dot(v, v);
  const Q du = Dot(delta, u);
  const Q dv = Dot(delta, v);
  const Q denominator = uu * vv - uv * uv;
  const Q wb = (du * vv - dv * uv) / denominator;
  const Q wc = (dv * uu - du * uv) / denominator;
  Q best = std::numeric_limits<Q>::max();
  if (wb >= 0 && wc >= 0 && wb + wc <= 1) {
    const Q3 point = triangle[0] + u * wb + v * wc;
    best = Norm(p - point);
  }
  for (unsigned i = 0; i < 3; ++i)
    best = std::min(best, PointSegmentDistance(
                              p, triangle[i], triangle[(i + 1) % 3]));
  return best;
}

Q SegmentDistance(Q3 a0, Q3 a1, Q3 b0, Q3 b1) {
  Q best = std::min(
      std::min(PointSegmentDistance(a0, b0, b1),
               PointSegmentDistance(a1, b0, b1)),
      std::min(PointSegmentDistance(b0, a0, a1),
               PointSegmentDistance(b1, a0, a1)));
  const Q3 u = a1 - a0;
  const Q3 v = b1 - b0;
  const Q3 w = a0 - b0;
  const Q a = Dot(u, u);
  const Q b = Dot(u, v);
  const Q c = Dot(v, v);
  const Q d = Dot(u, w);
  const Q e = Dot(v, w);
  const Q denominator = a * c - b * b;
  if (denominator != 0) {
    const Q s = (b * e - c * d) / denominator;
    const Q t = (a * e - b * d) / denominator;
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
      best = std::min(best, Norm((a0 + u * s) - (b0 + v * t)));
  }
  return best;
}

TEST(FixedTriangleOracle,
     EveryVFAndEEValueMatchesIndependentDecimal100Geometry) {
  const ct::Vec3 pa[3]{{-0.25, 0.125, 0}, {2, 0, 0.125},
                        {0, 2, -0.25}};
  const ct::Vec3 pb[3]{
      {0.125, 0.25, std::nextafter(1.0, 2.0)},
      {2.25, 0.375, 1.125},
      {0.375, 2.125, std::nextafter(0.75, 1.0)}};
  const std::uint64_t ia[3]{1, 2, 3};
  const std::uint64_t ib[3]{11, 12, 13};
  const ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, pa, ia), ft::Triangle(200, 0, pb, ib)};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  const auto report = discovery.Discover(triangles, 2, pair, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_EQ(discovery.features().count, 15u);

  Q3 qa[3], qb[3];
  for (unsigned i = 0; i < 3; ++i) {
    qa[i] = Convert(pa[i]);
    qb[i] = Convert(pb[i]);
  }
  for (std::size_t i = 0; i < discovery.features().count; ++i) {
    const auto& value = discovery.features().data[i];
    Q oracle;
    if (value.kind == ct::FixedTriangleCandidateKind::VertexFace) {
      if (value.local_features[0] < 3)
        oracle = PointTriangleDistance(
            qa[value.local_features[0]], qb);
      else
        oracle = PointTriangleDistance(
            qb[value.local_features[1]], qa);
    } else {
      const unsigned a = value.local_features[0];
      const unsigned b = value.local_features[1];
      oracle = SegmentDistance(qa[a], qa[(a + 1) % 3],
                               qb[b], qb[(b + 1) % 3]);
    }
    const double expected = static_cast<double>(oracle);
    const double tolerance =
        32 * DBL_EPSILON * std::max(1.0, std::fabs(expected));
    EXPECT_NEAR(value.distance_m, expected, tolerance)
        << "kind=" << static_cast<int>(value.kind)
        << " local=" << value.local_features[0] << ","
        << value.local_features[1];
  }
}

TEST(FixedTriangleOracle,
     AllTrianglePermutationsAndPairReversalsMatchDecimal100Geometry) {
  const std::array<std::array<unsigned, 3>, 6> permutations{{
      {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
      {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}},
  }};
  const ct::Vec3 source_a[3]{{-0.25, 0.125, 0},
                              {2, 0, 0.125},
                              {0, 2, -0.25}};
  const ct::Vec3 source_b[3]{
      {0.125, 0.25, std::nextafter(1.0, 2.0)},
      {2.25, 0.375, 1.125},
      {0.375, 2.125, std::nextafter(0.75, 1.0)}};
  const std::uint64_t source_ids_a[3]{1, 2, 3};
  const std::uint64_t source_ids_b[3]{11, 12, 13};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);

  for (const auto& permutation_a : permutations) {
    for (const auto& permutation_b : permutations) {
      ct::Vec3 a[3], b[3];
      std::uint64_t ids_a[3], ids_b[3];
      Q3 qa[3], qb[3];
      for (unsigned i = 0; i < 3; ++i) {
        a[i] = source_a[permutation_a[i]];
        b[i] = source_b[permutation_b[i]];
        ids_a[i] = source_ids_a[permutation_a[i]];
        ids_b[i] = source_ids_b[permutation_b[i]];
        qa[i] = Convert(a[i]);
        qb[i] = Convert(b[i]);
      }
      const ct::CurrentFixedTriangle triangles[2]{
          ft::Triangle(100, 0, a, ids_a),
          ft::Triangle(200, 0, b, ids_b)};
      for (unsigned reversed = 0; reversed < 2; ++reversed) {
        const ct::FixedTrianglePair pair[1]{{reversed, 1u - reversed}};
        const auto report =
            discovery.Discover(triangles, 2, pair, 1);
        ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
        ASSERT_EQ(report.feature_tasks, 15u);
        ASSERT_EQ(discovery.features().count, 15u);
        for (std::size_t i = 0; i < discovery.features().count; ++i) {
          const auto& value = discovery.features().data[i];
          Q oracle;
          if (value.kind ==
              ct::FixedTriangleCandidateKind::VertexFace) {
            oracle = value.local_features[0] < 3
                         ? PointTriangleDistance(
                               qa[value.local_features[0]], qb)
                         : PointTriangleDistance(
                               qb[value.local_features[1]], qa);
          } else {
            const unsigned edge_a = value.local_features[0];
            const unsigned edge_b = value.local_features[1];
            oracle = SegmentDistance(
                qa[edge_a], qa[(edge_a + 1) % 3],
                qb[edge_b], qb[(edge_b + 1) % 3]);
          }
          const double expected = static_cast<double>(oracle);
          const double tolerance =
              32 * DBL_EPSILON *
              std::max(1.0, std::fabs(expected));
          EXPECT_NEAR(value.distance_m, expected, tolerance);
        }
      }
    }
  }
}

Q Orient(Q2 a, Q2 b, Q2 p) {
  return (b.x - a.x) * (p.y - a.y) -
         (b.y - a.y) * (p.x - a.x);
}

// -1 separated, 0 lower-dimensional touch, 1 positive-area overlap.
int CoplanarOracle(const std::array<Q2, 3>& a,
                   const std::array<Q2, 3>& b) {
  bool strict = true;
  for (unsigned owner = 0; owner < 2; ++owner) {
    const auto& source = owner == 0 ? a : b;
    for (unsigned edge = 0; edge < 3; ++edge) {
      const Q dx = source[(edge + 1) % 3].x - source[edge].x;
      const Q dy = source[(edge + 1) % 3].y - source[edge].y;
      const Q2 axis{-dy, dx};
      Q min_a = axis.x * a[0].x + axis.y * a[0].y;
      Q max_a = min_a;
      Q min_b = axis.x * b[0].x + axis.y * b[0].y;
      Q max_b = min_b;
      for (unsigned i = 1; i < 3; ++i) {
        const Q pa = axis.x * a[i].x + axis.y * a[i].y;
        const Q pb = axis.x * b[i].x + axis.y * b[i].y;
        min_a = std::min(min_a, pa);
        max_a = std::max(max_a, pa);
        min_b = std::min(min_b, pb);
        max_b = std::max(max_b, pb);
      }
      const Q overlap =
          std::min(max_a, max_b) - std::max(min_a, min_b);
      if (overlap < 0)
        return -1;
      if (overlap == 0)
        strict = false;
    }
  }
  return strict ? 1 : 0;
}

TEST(FixedTriangleOracle,
     ExhaustiveSmallLatticeIntersectionsMatchDecimal100SAT) {
  std::array<Q2, 9> grid;
  for (unsigned y = 0, n = 0; y < 3; ++y)
    for (unsigned x = 0; x < 3; ++x, ++n)
      grid[n] = {Q(static_cast<int>(x) - 1),
                 Q(static_cast<int>(y) - 1)};

  std::vector<std::array<unsigned, 3>> combinations;
  for (unsigned i = 0; i < 9; ++i)
    for (unsigned j = i + 1; j < 9; ++j)
      for (unsigned k = j + 1; k < 9; ++k)
        if (Orient(grid[i], grid[j], grid[k]) != 0)
          combinations.push_back({i, j, k});
  ASSERT_GT(combinations.size(), 60u);

  std::vector<ct::CurrentFixedTriangle> triangles;
  triangles.reserve(combinations.size());
  for (std::size_t t = 0; t < combinations.size(); ++t) {
    ct::Vec3 points[3];
    std::uint64_t ids[3];
    for (unsigned i = 0; i < 3; ++i) {
      const auto& point = grid[combinations[t][i]];
      points[i] = {static_cast<double>(point.x),
                   static_cast<double>(point.y), 0};
      // Independent source topology despite reused geometric lattice points.
      ids[i] = 1000 + 3 * t + i;
    }
    triangles.push_back(
        ft::Triangle(100 + t, 0, points, ids));
  }
  std::vector<ct::FixedTrianglePair> pairs;
  for (std::size_t i = 0; i < triangles.size(); ++i)
    for (std::size_t j = i + 1; j < triangles.size(); ++j)
      pairs.push_back({static_cast<std::uint32_t>(i),
                       static_cast<std::uint32_t>(j)});

  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery, ft::Limits(pairs.size()));
  const auto report = discovery.Discover(
      triangles.data(), triangles.size(), pairs.data(), pairs.size());
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok)
      << report.message;
  std::map<std::pair<std::uint64_t, std::uint64_t>,
           ct::FixedTriangleIntersectionKind>
      actual;
  for (std::size_t i = 0; i < discovery.intersections().count; ++i) {
    const auto& value = discovery.intersections().data[i];
    EXPECT_EQ(value.local_exclusion,
              ct::FixedTriangleLocalExclusion::None);
    actual[{value.triangles[0].parent_eid,
            value.triangles[1].parent_eid}] = value.kind;
  }

  for (std::size_t i = 0; i < combinations.size(); ++i) {
    std::array<Q2, 3> a;
    for (unsigned p = 0; p < 3; ++p)
      a[p] = grid[combinations[i][p]];
    for (std::size_t j = i + 1; j < combinations.size(); ++j) {
      std::array<Q2, 3> b;
      for (unsigned p = 0; p < 3; ++p)
        b[p] = grid[combinations[j][p]];
      const int oracle = CoplanarOracle(a, b);
      const auto found = actual.find({100 + i, 100 + j});
      if (oracle < 0) {
        EXPECT_EQ(found, actual.end()) << i << "," << j;
      } else {
        ASSERT_NE(found, actual.end()) << i << "," << j;
        EXPECT_EQ(found->second,
                  oracle ? ct::FixedTriangleIntersectionKind::CoplanarOverlap
                         : ct::FixedTriangleIntersectionKind::CoplanarTouch)
            << i << "," << j;
      }
    }
  }
}

}  // namespace
