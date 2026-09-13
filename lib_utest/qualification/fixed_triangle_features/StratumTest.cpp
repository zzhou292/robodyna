// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include "lib_src/collision/SurfaceContactGeometry.h"

#include <boost/multiprecision/cpp_dec_float.hpp>
#include <gtest/gtest.h>

#include <array>
#include <cfloat>
#include <cmath>
#include <vector>

namespace {
namespace ct = tlfea::contact;
namespace ft = fixed_triangle_test;
using Q = boost::multiprecision::cpp_dec_float_100;

struct Q3 {
  Q x, y, z;
};

Q3 Convert(ct::Vec3 value) {
  return {Q(value.x), Q(value.y), Q(value.z)};
}

Q3 Subtract(Q3 a, Q3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Q Dot(Q3 a, Q3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

struct OracleStratum {
  ct::FixedTriangleStratumKind kind;
  unsigned local;
};

// Independent 100-decimal-digit Voronoi control.  All adversarial coordinates
// below are finite binary64 values represented exactly at this precision.
OracleStratum Oracle(ct::Vec3 point, const ct::Vec3 (&triangle)[3]) {
  const Q3 p = Convert(point);
  const Q3 a = Convert(triangle[0]);
  const Q3 b = Convert(triangle[1]);
  const Q3 c = Convert(triangle[2]);
  const Q3 ab = Subtract(b, a);
  const Q3 ac = Subtract(c, a);
  const Q d1 = Dot(ab, Subtract(p, a));
  const Q d2 = Dot(ac, Subtract(p, a));
  const Q d3 = Dot(ab, Subtract(p, b));
  const Q d4 = Dot(ac, Subtract(p, b));
  const Q d5 = Dot(ab, Subtract(p, c));
  const Q d6 = Dot(ac, Subtract(p, c));
  if (d1 <= 0 && d2 <= 0)
    return {ct::FixedTriangleStratumKind::Vertex, 0};
  if (d3 >= 0 && d4 <= d3)
    return {ct::FixedTriangleStratumKind::Vertex, 1};
  if (d1 * d4 - d3 * d2 <= 0 && d1 >= 0 && d3 <= 0)
    return {ct::FixedTriangleStratumKind::Edge, 0};
  if (d6 >= 0 && d5 <= d6)
    return {ct::FixedTriangleStratumKind::Vertex, 2};
  if (d5 * d2 - d1 * d6 <= 0 && d2 >= 0 && d6 <= 0)
    return {ct::FixedTriangleStratumKind::Edge, 2};
  if (d3 * d6 - d5 * d4 <= 0 && d4 >= d3 && d5 >= d6)
    return {ct::FixedTriangleStratumKind::Edge, 1};
  return {ct::FixedTriangleStratumKind::Face, 0};
}

ct::TrianglePointGeometry RoundedClosest(
    ct::Vec3 point, const ct::Vec3 (&triangle)[3]) {
  ct::TriangleGeometry geometry;
  for (unsigned i = 0; i < 3; ++i) {
    geometry.vertices[i] = triangle[i];
    geometry.vertex_ids[i] = i;
  }
  ct::TrianglePointGeometry result;
  EXPECT_EQ(ct::ClosestPointOnTriangle(point, geometry, &result),
            ct::Status::kOk);
  return result;
}

const ct::FixedTriangleFeatureCandidate* FindSourceVertex(
    ct::FixedTriangleFeatureView view,
    const ct::FacetVertexKey& source) {
  for (std::size_t i = 0; i < view.count; ++i) {
    const auto& value = view.data[i];
    if (value.key.kind == ct::FixedTriangleCandidateKind::VertexFace &&
        ft::Same(value.key.vertex_face.vertex, source))
      return &value;
  }
  return nullptr;
}

std::array<double, 3> RebasedWeights(
    const ct::FixedTriangleFeatureCandidate& value,
    const ct::CurrentFixedTriangle& target,
    const std::uint64_t (&source_order)[3]) {
  std::array<double, 3> result{};
  for (unsigned local = 0; local < 3; ++local)
    for (unsigned canonical = 0; canonical < 3; ++canonical)
      if (target.vertex_keys[local].first == source_order[canonical])
        result[canonical] = value.face_weights[local];
  return result;
}

void CheckAllTargetPermutations(
    const ct::Vec3 (&source_points)[3],
    const std::uint64_t (&source_ids)[3],
    const ct::Vec3 (&target_points)[3],
    const std::uint64_t (&target_ids)[3],
    OracleStratum expected) {
  const std::array<std::array<unsigned, 3>, 6> permutations{{
      {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
      {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}},
  }};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  bool have_baseline = false;
  ct::FixedTriangleFeatureKey baseline_key;
  ct::Vec3 baseline_point{};
  std::array<double, 3> baseline_weights{};

  for (const auto& permutation : permutations) {
    ct::Vec3 points[3];
    std::uint64_t ids[3];
    for (unsigned i = 0; i < 3; ++i) {
      points[i] = target_points[permutation[i]];
      ids[i] = target_ids[permutation[i]];
    }
    const ct::CurrentFixedTriangle triangles[2]{
        ft::Triangle(100, 0, source_points, source_ids),
        ft::Triangle(200, 0, points, ids)};
    for (unsigned reversed = 0; reversed < 2; ++reversed) {
      const ct::FixedTrianglePair pair[1]{{reversed, 1u - reversed}};
      const auto report = discovery.Discover(triangles, 2, pair, 1);
      ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
      ASSERT_EQ(report.feature_tasks, 15u);
      const auto* value =
          FindSourceVertex(discovery.features(), ft::Vertex(source_ids[0]));
      ASSERT_NE(value, nullptr);
      const auto& stratum = value->key.vertex_face.target;
      ASSERT_EQ(stratum.kind, expected.kind);
      if (expected.kind == ct::FixedTriangleStratumKind::Vertex)
        EXPECT_TRUE(ft::Same(stratum.vertex,
                            ft::Vertex(target_ids[expected.local])));
      else if (expected.kind == ct::FixedTriangleStratumKind::Edge)
        EXPECT_TRUE(ft::Same(
            stratum.edge,
            ft::Triangle(200, 0, target_points, target_ids)
                .edge_keys[expected.local]));
      else
        EXPECT_TRUE(ft::Same(
            stratum.face,
            ft::Triangle(200, 0, target_points, target_ids).key));
      const auto weights =
          RebasedWeights(*value, triangles[1], target_ids);
      if (expected.kind == ct::FixedTriangleStratumKind::Vertex) {
        for (unsigned i = 0; i < 3; ++i)
          EXPECT_EQ(weights[i], i == expected.local ? 1 : 0);
      } else if (expected.kind == ct::FixedTriangleStratumKind::Edge) {
        EXPECT_GT(value->edge_parameters[0], 0);
        EXPECT_LT(value->edge_parameters[0], 1);
        const auto& edge_key = stratum.edge;
        for (unsigned i = 0; i < 3; ++i) {
          if (target_ids[i] == edge_key.endpoints[0].first)
            EXPECT_EQ(weights[i], 1 - value->edge_parameters[0]);
          else if (target_ids[i] == edge_key.endpoints[1].first)
            EXPECT_EQ(weights[i], value->edge_parameters[0]);
          else
            EXPECT_EQ(weights[i], 0);
        }
      } else {
        for (double weight : weights) {
          EXPECT_GT(weight, 0);
          EXPECT_LT(weight, 1);
        }
      }
      const ct::Vec3 represented = ct::Add(
          ct::Add(ct::Scale(target_points[0], weights[0]),
                  ct::Scale(target_points[1], weights[1])),
          ct::Scale(target_points[2], weights[2]));
      EXPECT_EQ(value->points[1].x, represented.x);
      EXPECT_EQ(value->points[1].y, represented.y);
      EXPECT_EQ(value->points[1].z, represented.z);
      if (!have_baseline) {
        baseline_key = value->key;
        baseline_point = value->points[1];
        baseline_weights = weights;
        have_baseline = true;
      } else {
        EXPECT_TRUE(ft::Same(value->key, baseline_key));
        EXPECT_EQ(value->points[1].x, baseline_point.x);
        EXPECT_EQ(value->points[1].y, baseline_point.y);
        EXPECT_EQ(value->points[1].z, baseline_point.z);
        EXPECT_EQ(weights, baseline_weights);
      }
    }
  }
}

void ExpectUnrepresentableAllPermutations(
    ct::Vec3 query, const ct::Vec3 (&target_points)[3],
    const std::uint64_t (&target_ids)[3]) {
  const std::array<std::array<unsigned, 3>, 6> permutations{{
      {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
      {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}},
  }};
  const ct::Vec3 source_points[3]{
      query, {query.x + 4, query.y, query.z},
      {query.x, query.y + 4, query.z}};
  const std::uint64_t source_ids[3]{50, 51, 52};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);
  for (const auto& permutation : permutations) {
    ct::Vec3 points[3];
    std::uint64_t ids[3];
    for (unsigned i = 0; i < 3; ++i) {
      points[i] = target_points[permutation[i]];
      ids[i] = target_ids[permutation[i]];
    }
    const ct::CurrentFixedTriangle triangles[2]{
        ft::Triangle(100, 0, source_points, source_ids),
        ft::Triangle(200, 0, points, ids)};
    for (unsigned reversed = 0; reversed < 2; ++reversed) {
      const ct::FixedTrianglePair pair[1]{{reversed, 1u - reversed}};
      const auto report = discovery.Discover(triangles, 2, pair, 1);
      EXPECT_EQ(report.status,
                ct::FixedTriangleDiscoveryStatus::NonFiniteResult);
      EXPECT_EQ(report.input_pair, 0u);
      EXPECT_FALSE(discovery.features().complete);
      EXPECT_FALSE(discovery.intersections().complete);
    }
  }
}

TEST(FixedTriangleStratum,
     ExactEdgeIdentityIgnoresRoundedPositiveBoundaryWeight) {
  const ct::Vec3 target[3]{
      {134217730.0, -134217729.0, 67108863.0},
      {134217715.0, -134217718.0, 67108875.0},
      {134217737.0, -134217739.0, 67108854.0}};
  const ct::Vec3 query{
      134217883.0000143, -134218126.0000105, 67109458.99998856};
  const ct::Vec3 source[3]{
      query, {query.x + 100, query.y, query.z},
      {query.x, query.y + 100, query.z}};
  const std::uint64_t source_ids[3]{50, 51, 52};
  const std::uint64_t target_ids[3]{1, 2, 3};
  const auto rounded = RoundedClosest(query, target);
  ASSERT_GT(rounded.weights[2], 0);
  ASSERT_LT(rounded.weights[2], 1e-12);
  const auto oracle = Oracle(query, target);
  ASSERT_EQ(oracle.kind, ct::FixedTriangleStratumKind::Edge);
  ASSERT_EQ(oracle.local, 0u);
  CheckAllTargetPermutations(
      source, source_ids, target, target_ids, oracle);
}

TEST(FixedTriangleStratum,
     ExactVertexIdentityIgnoresRoundedSmallNonzeroWeight) {
  const ct::Vec3 target[3]{
      {4294967305.0, -4294967284.0, 2147483632.0},
      {4294967321.0, -4294967293.0, 2147483642.0},
      {4294967323.0, -4294967267.0, 2147483660.0}};
  const ct::Vec3 query{
      4294963947.0, -4294969411.0, 2147487132.0};
  const ct::Vec3 source[3]{
      query, {query.x + 100, query.y, query.z},
      {query.x, query.y + 100, query.z}};
  const std::uint64_t source_ids[3]{50, 51, 52};
  const std::uint64_t target_ids[3]{1, 2, 3};
  const auto rounded = RoundedClosest(query, target);
  ASSERT_GT(rounded.weights[0], 0);
  ASSERT_LT(rounded.weights[0], 1e-12);
  ASSERT_GT(rounded.weights[1], 0);
  ASSERT_LT(rounded.weights[1], 1e-12);
  ASSERT_NE(rounded.weights[2], 1);
  const auto oracle = Oracle(query, target);
  ASSERT_EQ(oracle.kind, ct::FixedTriangleStratumKind::Vertex);
  ASSERT_EQ(oracle.local, 2u);
  CheckAllTargetPermutations(
      source, source_ids, target, target_ids, oracle);
}

TEST(FixedTriangleStratum,
     AdversarialExactEdgeStillDeduplicatesAcrossTargetSeam) {
  const ct::Vec3 first[3]{
      {134217730.0, -134217729.0, 67108863.0},
      {134217715.0, -134217718.0, 67108875.0},
      {134217737.0, -134217739.0, 67108854.0}};
  const ct::Vec3 second[3]{
      first[1], first[0],
      {134217708.0, -134217708.0, 67108884.0}};
  const ct::Vec3 query{
      134217883.0000143, -134218126.0000105, 67109458.99998856};
  const ct::Vec3 source[3]{
      query, {query.x + 100, query.y, query.z},
      {query.x, query.y + 100, query.z}};
  const std::uint64_t source_ids[3]{50, 51, 52};
  const std::uint64_t first_ids[3]{1, 2, 3};
  const std::uint64_t second_ids[3]{2, 1, 4};
  const std::array<std::array<unsigned, 3>, 6> permutations{{
      {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
      {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}},
  }};
  const std::array<std::array<ct::FixedTrianglePair, 2>, 3> pair_orders{{
      {{{0, 1}, {0, 2}}},
      {{{1, 0}, {2, 0}}},
      {{{2, 0}, {1, 0}}},
  }};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery, ft::Limits(2));
  const auto canonical_first =
      ft::Triangle(200, 0, first, first_ids);
  for (const auto& first_permutation : permutations) {
    for (const auto& second_permutation : permutations) {
      ct::Vec3 first_points[3], second_points[3];
      std::uint64_t first_permuted_ids[3], second_permuted_ids[3];
      for (unsigned i = 0; i < 3; ++i) {
        first_points[i] = first[first_permutation[i]];
        first_permuted_ids[i] = first_ids[first_permutation[i]];
        second_points[i] = second[second_permutation[i]];
        second_permuted_ids[i] = second_ids[second_permutation[i]];
      }
      const ct::CurrentFixedTriangle triangles[3]{
          ft::Triangle(100, 0, source, source_ids),
          ft::Triangle(200, 0, first_points, first_permuted_ids),
          ft::Triangle(300, 0, second_points, second_permuted_ids)};
      for (const auto& pairs : pair_orders) {
        const auto report =
            discovery.Discover(triangles, 3, pairs.data(), pairs.size());
        ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
        EXPECT_EQ(report.feature_tasks, 30u);
        EXPECT_LT(report.feature_candidates,
                  report.raw_feature_candidates);
        std::size_t matches = 0;
        for (std::size_t i = 0; i < discovery.features().count; ++i) {
          const auto& value = discovery.features().data[i];
          if (value.key.kind ==
                  ct::FixedTriangleCandidateKind::VertexFace &&
              ft::Same(value.key.vertex_face.vertex, ft::Vertex(50)) &&
              value.key.vertex_face.target.kind ==
                  ct::FixedTriangleStratumKind::Edge &&
              ft::Same(value.key.vertex_face.target.edge,
                       canonical_first.edge_keys[0])) {
            EXPECT_GT(value.edge_parameters[0], 0);
            EXPECT_LT(value.edge_parameters[0], 1);
            ++matches;
          }
        }
        EXPECT_EQ(matches, 1u);
      }
    }
  }
}

TEST(FixedTriangleStratum,
     ClosedBoundaryNextafterAndOutsideVoronoiRegionsStayExact) {
  const ct::Vec3 target[3]{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  const std::uint64_t target_ids[3]{1, 2, 3};
  const std::uint64_t source_ids[3]{50, 51, 52};

  const ct::Vec3 edge_query{0.5, 0, 1};
  const ct::Vec3 edge_source[3]{
      edge_query, {edge_query.x + 2, edge_query.y, edge_query.z},
      {edge_query.x, edge_query.y + 2, edge_query.z}};
  const auto edge = Oracle(edge_query, target);
  ASSERT_EQ(edge.kind, ct::FixedTriangleStratumKind::Edge);
  ASSERT_EQ(edge.local, 0u);
  CheckAllTargetPermutations(
      edge_source, source_ids, target, target_ids, edge);

  const ct::Vec3 face_query{
      0.5, std::nextafter(0.0, 1.0), 1};
  const ct::Vec3 face_source[3]{
      face_query, {face_query.x + 2, face_query.y, face_query.z},
      {face_query.x, face_query.y + 2, face_query.z}};
  const auto face = Oracle(face_query, target);
  ASSERT_EQ(face.kind, ct::FixedTriangleStratumKind::Face);
  CheckAllTargetPermutations(
      face_source, source_ids, target, target_ids, face);

  const ct::Vec3 outside_query{-1, -1, 1};
  const ct::Vec3 outside_source[3]{
      outside_query,
      {outside_query.x + 2, outside_query.y, outside_query.z},
      {outside_query.x, outside_query.y + 2, outside_query.z}};
  const auto outside = Oracle(outside_query, target);
  ASSERT_EQ(outside.kind, ct::FixedTriangleStratumKind::Vertex);
  ASSERT_EQ(outside.local, 0u);
  CheckAllTargetPermutations(
      outside_source, source_ids, target, target_ids, outside);
}

TEST(FixedTriangleStratum,
     UnrepresentableFaceAndEdgeInteriorsRejectAllPermutations) {
  const ct::Vec3 target[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const std::uint64_t target_ids[3]{1, 2, 3};
  const double smallest = std::nextafter(0.0, 1.0);

  const ct::Vec3 exact_face{1, smallest, 1};
  ASSERT_EQ(Oracle(exact_face, target).kind,
            ct::FixedTriangleStratumKind::Face);
  ExpectUnrepresentableAllPermutations(
      exact_face, target, target_ids);

  const ct::Vec3 exact_edge{smallest, -1, 1};
  const auto edge = Oracle(exact_edge, target);
  ASSERT_EQ(edge.kind, ct::FixedTriangleStratumKind::Edge);
  ASSERT_EQ(edge.local, 0u);
  ExpectUnrepresentableAllPermutations(
      exact_edge, target, target_ids);
}

TEST(FixedTriangleStratum,
     UnrepresentableInteriorFailurePreservesPublicationAndRetries) {
  const ct::Vec3 target_points[3]{
      {0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const std::uint64_t target_ids[3]{1, 2, 3};
  const std::uint64_t source_ids[3]{50, 51, 52};
  const ct::FixedTrianglePair pair[1]{{0, 1}};
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery);

  const auto make_source = [](ct::Vec3 query,
                              ct::Vec3 (&points)[3]) {
    points[0] = query;
    points[1] = {query.x + 4, query.y, query.z};
    points[2] = {query.x, query.y + 4, query.z};
  };
  ct::Vec3 source_points[3];
  make_source({1, 0.5, 1}, source_points);
  ct::CurrentFixedTriangle triangles[2]{
      ft::Triangle(100, 0, source_points, source_ids),
      ft::Triangle(200, 0, target_points, target_ids)};
  auto report = discovery.Discover(triangles, 2, pair, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  const auto previous = discovery.features();
  ASSERT_TRUE(previous.complete);
  const auto previous_count = previous.count;
  const auto previous_key = previous.data[0].key;

  make_source(
      {1, std::nextafter(0.0, 1.0), 1}, source_points);
  triangles[0] =
      ft::Triangle(100, 0, source_points, source_ids);
  report = discovery.Discover(triangles, 2, pair, 1);
  EXPECT_EQ(report.status,
            ct::FixedTriangleDiscoveryStatus::NonFiniteResult);
  ASSERT_TRUE(discovery.features().complete);
  ASSERT_EQ(discovery.features().count, previous_count);
  EXPECT_TRUE(ft::Same(discovery.features().data[0].key,
                       previous_key));

  make_source({1, 0, 1}, source_points);
  triangles[0] =
      ft::Triangle(100, 0, source_points, source_ids);
  report = discovery.Discover(triangles, 2, pair, 1);
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  const auto* edge =
      FindSourceVertex(discovery.features(), ft::Vertex(50));
  ASSERT_NE(edge, nullptr);
  ASSERT_EQ(edge->key.vertex_face.target.kind,
            ct::FixedTriangleStratumKind::Edge);
  EXPECT_GT(edge->edge_parameters[0], 0);
  EXPECT_LT(edge->edge_parameters[0], 1);
}

}  // namespace
