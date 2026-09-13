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

void ExpectCompleteRecordEqual(
    const ct::FixedTriangleFeatureCandidate& a,
    const ct::FixedTriangleFeatureCandidate& b) {
  EXPECT_TRUE(ft::Same(a.key, b.key));
  for (unsigned i = 0; i < 2; ++i) {
    EXPECT_TRUE(ft::Same(a.triangles[i], b.triangles[i]));
    EXPECT_EQ(a.local_features[i], b.local_features[i]);
    EXPECT_EQ(a.points[i].x, b.points[i].x);
    EXPECT_EQ(a.points[i].y, b.points[i].y);
    EXPECT_EQ(a.points[i].z, b.points[i].z);
    EXPECT_EQ(a.edge_parameters[i], b.edge_parameters[i]);
  }
  for (unsigned i = 0; i < 3; ++i)
    EXPECT_EQ(a.face_weights[i], b.face_weights[i]);
  EXPECT_EQ(a.distance_m, b.distance_m);
}

const ct::FixedTriangleFeatureCandidate* FindEdgePair(
    ct::FixedTriangleFeatureView view, const ct::FacetEdgeKey& first,
    const ct::FacetEdgeKey& second) {
  for (std::size_t i = 0; i < view.count; ++i) {
    const auto& value = view.data[i];
    if (value.key.kind != ct::FixedTriangleCandidateKind::EdgeEdge)
      continue;
    const auto& edges = value.key.edge_edge.edges;
    if ((ft::Same(edges[0], first) && ft::Same(edges[1], second)) ||
        (ft::Same(edges[0], second) && ft::Same(edges[1], first)))
      return &value;
  }
  return nullptr;
}

ct::Vec3 LookupVertex(const ct::CurrentFixedTriangle* triangles,
                      const ct::FacetVertexKey& key) {
  for (unsigned triangle = 0; triangle < 2; ++triangle)
    for (unsigned local = 0; local < 3; ++local)
      if (ft::Same(triangles[triangle].vertex_keys[local], key))
        return triangles[triangle].vertices[local];
  ADD_FAILURE() << "missing ledger vertex";
  return {};
}

void ExpectRecordReconstructsGeometry(
    const ct::FixedTriangleFeatureCandidate& value,
    const ct::CurrentFixedTriangle* triangles) {
  if (value.key.kind == ct::FixedTriangleCandidateKind::VertexFace) {
    const unsigned source = value.local_features[0] < 3 ? 0 : 1;
    const unsigned target = 1 - source;
    const unsigned source_local = value.local_features[source];
    EXPECT_TRUE(ft::Same(value.key.vertex_face.vertex,
                         triangles[source].vertex_keys[source_local]));
    EXPECT_EQ(value.points[0].x,
              triangles[source].vertices[source_local].x);
    EXPECT_EQ(value.points[0].y,
              triangles[source].vertices[source_local].y);
    EXPECT_EQ(value.points[0].z,
              triangles[source].vertices[source_local].z);
    ct::Vec3 reconstructed{};
    double sum = 0;
    for (unsigned i = 0; i < 3; ++i) {
      EXPECT_GE(value.face_weights[i], 0);
      EXPECT_LE(value.face_weights[i], 1);
      sum += value.face_weights[i];
      reconstructed.x +=
          value.face_weights[i] * triangles[target].vertices[i].x;
      reconstructed.y +=
          value.face_weights[i] * triangles[target].vertices[i].y;
      reconstructed.z +=
          value.face_weights[i] * triangles[target].vertices[i].z;
    }
    EXPECT_NEAR(sum, 1, 8 * DBL_EPSILON);
    const double scale = std::max(
        {1.0, std::fabs(value.points[1].x),
         std::fabs(value.points[1].y),
         std::fabs(value.points[1].z)});
    EXPECT_NEAR(value.points[1].x, reconstructed.x,
                16 * DBL_EPSILON * scale);
    EXPECT_NEAR(value.points[1].y, reconstructed.y,
                16 * DBL_EPSILON * scale);
    EXPECT_NEAR(value.points[1].z, reconstructed.z,
                16 * DBL_EPSILON * scale);
    const auto& stratum = value.key.vertex_face.target;
    if (stratum.kind == ct::FixedTriangleStratumKind::Vertex) {
      bool found = false;
      for (unsigned i = 0; i < 3; ++i)
        found = found ||
                (value.face_weights[i] == 1 &&
                 ft::Same(stratum.vertex,
                          triangles[target].vertex_keys[i]));
      EXPECT_TRUE(found);
    } else if (stratum.kind == ct::FixedTriangleStratumKind::Edge) {
      bool found = false;
      for (unsigned i = 0; i < 3; ++i)
        found = found ||
                (value.face_weights[(i + 2) % 3] == 0 &&
                 ft::Same(stratum.edge,
                          triangles[target].edge_keys[i]));
      EXPECT_TRUE(found);
    } else {
      EXPECT_TRUE(ft::Same(stratum.face, triangles[target].key));
      for (double weight : value.face_weights)
        EXPECT_GT(weight, 0);
    }
  } else {
    for (unsigned edge = 0; edge < 2; ++edge) {
      EXPECT_GE(value.edge_parameters[edge], 0);
      EXPECT_LE(value.edge_parameters[edge], 1);
      const auto& key = value.key.edge_edge.edges[edge];
      const ct::Vec3 a = LookupVertex(triangles, key.endpoints[0]);
      const ct::Vec3 b = LookupVertex(triangles, key.endpoints[1]);
      const double t = value.edge_parameters[edge];
      const ct::Vec3 reconstructed{
          a.x + (b.x - a.x) * t,
          a.y + (b.y - a.y) * t,
          a.z + (b.z - a.z) * t};
      const double scale = std::max(
          {1.0, std::fabs(value.points[edge].x),
           std::fabs(value.points[edge].y),
           std::fabs(value.points[edge].z)});
      EXPECT_NEAR(value.points[edge].x, reconstructed.x,
                  8 * DBL_EPSILON * scale);
      EXPECT_NEAR(value.points[edge].y, reconstructed.y,
                  8 * DBL_EPSILON * scale);
      EXPECT_NEAR(value.points[edge].z, reconstructed.z,
                  8 * DBL_EPSILON * scale);
    }
  }
  const double dx = value.points[0].x - value.points[1].x;
  const double dy = value.points[0].y - value.points[1].y;
  const double dz = value.points[0].z - value.points[1].z;
  EXPECT_NEAR(value.distance_m, std::sqrt(dx * dx + dy * dy + dz * dz),
              16 * DBL_EPSILON *
                  std::max(1.0, value.distance_m));
}

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

Q PlaneSide(Q3 point, const Q3* triangle) {
  return Dot(point - triangle[0],
             Cross(triangle[1] - triangle[0],
                   triangle[2] - triangle[0]));
}

bool PointInTriangle3D(Q3 point, const Q3* triangle) {
  const Q3 normal = Cross(triangle[1] - triangle[0],
                          triangle[2] - triangle[0]);
  Q signs[3];
  for (unsigned i = 0; i < 3; ++i)
    signs[i] = Dot(
        Cross(triangle[(i + 1) % 3] - triangle[i],
              point - triangle[i]),
        normal);
  return (signs[0] >= 0 && signs[1] >= 0 && signs[2] >= 0) ||
         (signs[0] <= 0 && signs[1] <= 0 && signs[2] <= 0);
}

bool SegmentTriangleOracle(Q3 first, Q3 second,
                           const Q3* triangle) {
  const Q side_first = PlaneSide(first, triangle);
  const Q side_second = PlaneSide(second, triangle);
  if ((side_first > 0 && side_second > 0) ||
      (side_first < 0 && side_second < 0))
    return false;
  if (side_first == 0)
    return PointInTriangle3D(first, triangle);
  if (side_second == 0)
    return PointInTriangle3D(second, triangle);
  const Q t = side_first / (side_first - side_second);
  return PointInTriangle3D(first + (second - first) * t, triangle);
}

bool TransverseIntersectionOracle(const ct::Vec3 (&a)[3],
                                  const ct::Vec3 (&b)[3]) {
  Q3 qa[3], qb[3];
  for (unsigned i = 0; i < 3; ++i) {
    qa[i] = Convert(a[i]);
    qb[i] = Convert(b[i]);
  }
  for (unsigned i = 0; i < 3; ++i)
    if (SegmentTriangleOracle(
            qa[i], qa[(i + 1) % 3], qb) ||
        SegmentTriangleOracle(
            qb[i], qb[(i + 1) % 3], qa))
      return true;
  return false;
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
    ExpectRecordReconstructsGeometry(value, triangles);
    Q oracle;
    if (value.key.kind == ct::FixedTriangleCandidateKind::VertexFace) {
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
        << "kind=" << static_cast<int>(value.key.kind)
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
  std::vector<ct::FixedTriangleFeatureKey> canonical_keys;

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
      std::vector<ct::FixedTriangleFeatureCandidate> forward;
      for (unsigned reversed = 0; reversed < 2; ++reversed) {
        const ct::FixedTrianglePair pair[1]{{reversed, 1u - reversed}};
        const auto report =
            discovery.Discover(triangles, 2, pair, 1);
        ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
        ASSERT_EQ(report.feature_tasks, 15u);
        ASSERT_EQ(discovery.features().count, 15u);
        if (!reversed) {
          if (canonical_keys.empty()) {
            for (std::size_t i = 0;
                 i < discovery.features().count; ++i)
              canonical_keys.push_back(
                  discovery.features().data[i].key);
          } else {
            ASSERT_EQ(discovery.features().count,
                      canonical_keys.size());
            for (std::size_t i = 0;
                 i < canonical_keys.size(); ++i)
              EXPECT_TRUE(ft::Same(
                  discovery.features().data[i].key,
                  canonical_keys[i]));
          }
        }
        for (std::size_t i = 0; i < discovery.features().count; ++i) {
          const auto& value = discovery.features().data[i];
          ExpectRecordReconstructsGeometry(value, triangles);
          Q oracle;
          if (value.key.kind ==
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
        if (!reversed) {
          forward.assign(discovery.features().data,
                         discovery.features().data +
                             discovery.features().count);
        } else {
          ASSERT_EQ(discovery.features().count, forward.size());
          for (std::size_t i = 0; i < forward.size(); ++i)
            ExpectCompleteRecordEqual(
                discovery.features().data[i], forward[i]);
        }
      }
    }
  }
}

TEST(FixedTriangleOracle,
     ExhaustiveTransverseGridMatchesIndependentDecimal100PlaneClipping) {
  const std::array<std::array<unsigned, 3>, 6> permutations{{
      {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
      {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}},
  }};
  const ct::Vec3 source_a[3]{{-2, -2, 0}, {2, -2, 0}, {0, 2, 0}};
  const std::uint64_t source_ids_a[3]{1, 2, 3};
  const std::uint64_t source_ids_b[3]{11, 12, 13};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);

  for (int x = -3; x <= 3; ++x) {
    for (int y = -3; y <= 3; ++y) {
      const ct::Vec3 source_b[3]{
          {double(x), double(y), -1},
          {double(x), double(y + 1), 1},
          {double(x), double(y + 2), -1}};
      const bool expected =
          TransverseIntersectionOracle(source_a, source_b);
      for (const auto& permutation_a : permutations) {
        for (const auto& permutation_b : permutations) {
          ct::Vec3 a[3], b[3];
          std::uint64_t ids_a[3], ids_b[3];
          for (unsigned i = 0; i < 3; ++i) {
            a[i] = source_a[permutation_a[i]];
            b[i] = source_b[permutation_b[i]];
            ids_a[i] = source_ids_a[permutation_a[i]];
            ids_b[i] = source_ids_b[permutation_b[i]];
          }
          const ct::CurrentFixedTriangle triangles[2]{
              ft::Triangle(100, 0, a, ids_a),
              ft::Triangle(200, 0, b, ids_b)};
          for (unsigned reversed = 0; reversed < 2; ++reversed) {
            const ct::FixedTrianglePair pair[1]{
                {reversed, 1u - reversed}};
            const auto report =
                discovery.Discover(triangles, 2, pair, 1);
            ASSERT_EQ(report.status,
                      ct::FixedTriangleDiscoveryStatus::Ok);
            ASSERT_EQ(discovery.intersections().count,
                      expected ? 1u : 0u)
                << x << "," << y;
            if (expected)
              EXPECT_EQ(discovery.intersections().data[0].kind,
                        ct::FixedTriangleIntersectionKind::Transverse);
          }
        }
      }
    }
  }
}

TEST(FixedTriangleOracle,
     ParallelCollinearAndNearParallelEdgeRecordsSurviveAllReversals) {
  const std::array<std::array<unsigned, 3>, 6> permutations{{
      {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
      {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}},
  }};
  const ct::Vec3 source_a[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const std::uint64_t ids_a_source[3]{1, 2, 3};
  const std::uint64_t ids_b_source[3]{11, 12, 13};
  const std::array<std::array<ct::Vec3, 3>, 3> cases{{
      {{{0, 1, 1}, {2, 1, 1}, {0, 3, 1}}},
      {{{1, 0, 0}, {3, 0, 0}, {3, -2, 0}}},
      {{{0, std::nextafter(0.0, 1.0), 1},
        {2, std::nextafter(0.0, 1.0),
         std::nextafter(1.0, 2.0)},
        {0, 2, 1}}},
  }};
  const auto canonical_a =
      ft::Triangle(100, 0, source_a, ids_a_source);
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);

  for (const auto& source_b : cases) {
    const ct::Vec3 canonical_b_points[3]{
        source_b[0], source_b[1], source_b[2]};
    const auto canonical_b =
        ft::Triangle(200, 0, canonical_b_points, ids_b_source);
    bool have_baseline = false;
    ct::FixedTriangleFeatureCandidate baseline;
    for (const auto& permutation_a : permutations) {
      for (const auto& permutation_b : permutations) {
        ct::Vec3 a[3], b[3];
        std::uint64_t ids_a[3], ids_b[3];
        for (unsigned i = 0; i < 3; ++i) {
          a[i] = source_a[permutation_a[i]];
          b[i] = source_b[permutation_b[i]];
          ids_a[i] = ids_a_source[permutation_a[i]];
          ids_b[i] = ids_b_source[permutation_b[i]];
        }
        const ct::CurrentFixedTriangle triangles[2]{
            ft::Triangle(100, 0, a, ids_a),
            ft::Triangle(200, 0, b, ids_b)};
        for (unsigned reversed = 0; reversed < 2; ++reversed) {
          const ct::FixedTrianglePair pair[1]{
              {reversed, 1u - reversed}};
          ASSERT_EQ(discovery.Discover(triangles, 2, pair, 1).status,
                    ct::FixedTriangleDiscoveryStatus::Ok);
          const auto* value = FindEdgePair(
              discovery.features(), canonical_a.edge_keys[0],
              canonical_b.edge_keys[0]);
          ASSERT_NE(value, nullptr);
          ExpectRecordReconstructsGeometry(*value, triangles);
          const Q expected = SegmentDistance(
              Convert(source_a[0]), Convert(source_a[1]),
              Convert(source_b[0]), Convert(source_b[1]));
          EXPECT_NEAR(value->distance_m,
                      static_cast<double>(expected),
                      32 * DBL_EPSILON *
                          std::max(1.0, value->distance_m));
          if (!have_baseline) {
            baseline = *value;
            have_baseline = true;
          } else {
            EXPECT_TRUE(ft::Same(value->key, baseline.key));
            for (unsigned i = 0; i < 2; ++i) {
              EXPECT_EQ(value->points[i].x, baseline.points[i].x);
              EXPECT_EQ(value->points[i].y, baseline.points[i].y);
              EXPECT_EQ(value->points[i].z, baseline.points[i].z);
              EXPECT_EQ(value->edge_parameters[i],
                        baseline.edge_parameters[i]);
            }
            EXPECT_EQ(value->distance_m, baseline.distance_m);
          }
        }
      }
    }
  }
}

Q Orient(Q2 a, Q2 b, Q2 p) {
  return (b.x - a.x) * (p.y - a.y) -
         (b.y - a.y) * (p.x - a.x);
}

Q2 IntersectionWithLine(Q2 first, Q2 second, Q2 a, Q2 b) {
  const Q side_first = Orient(a, b, first);
  const Q side_second = Orient(a, b, second);
  const Q t = side_first / (side_first - side_second);
  return {first.x + (second.x - first.x) * t,
          first.y + (second.y - first.y) * t};
}

// Independent exact-arithmetic Sutherland-Hodgman clipping oracle.
// -1 separated, 0 lower-dimensional touch, 1 positive-area overlap.
int CoplanarClippingOracle(const std::array<Q2, 3>& a,
                           const std::array<Q2, 3>& b) {
  std::vector<Q2> polygon(a.begin(), a.end());
  const Q winding = Orient(b[0], b[1], b[2]);
  for (unsigned edge = 0; edge < 3 && !polygon.empty(); ++edge) {
    const Q2 clip_a = b[edge];
    const Q2 clip_b = b[(edge + 1) % 3];
    std::vector<Q2> next;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
      const Q2 first = polygon[i];
      const Q2 second = polygon[(i + 1) % polygon.size()];
      const Q side_first = Orient(clip_a, clip_b, first) * winding;
      const Q side_second = Orient(clip_a, clip_b, second) * winding;
      const bool first_inside = side_first >= 0;
      const bool second_inside = side_second >= 0;
      if (first_inside != second_inside)
        next.push_back(IntersectionWithLine(
            first, second, clip_a, clip_b));
      if (second_inside)
        next.push_back(second);
    }
    polygon = std::move(next);
  }
  if (polygon.empty())
    return -1;
  Q twice_area = 0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto& p = polygon[i];
    const auto& q = polygon[(i + 1) % polygon.size()];
    twice_area += p.x * q.y - p.y * q.x;
  }
  return twice_area == 0 ? 0 : 1;
}

TEST(FixedTriangleOracle,
     ExhaustiveSmallLatticeIntersectionsMatchExactDecimal100Clipping) {
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
      const int oracle = CoplanarClippingOracle(a, b);
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
