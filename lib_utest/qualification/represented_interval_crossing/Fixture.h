// SPDX-License-Identifier: MIT
#pragma once

#include "../../../lib_src/collision/RepresentedIntervalCrossing.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>
#include <limits>
#include <vector>

namespace represented_interval_test {
namespace ct = tlfea::contact;
using C = ct::RepresentedIntervalClassification;
using G = ct::RepresentedIntersectionGeometry;
using K = ct::RepresentedFeatureKind;
using R = ct::RepresentedIntervalReason;
using S = ct::RepresentedIntervalStatus;

inline ct::FacetVertexKey Vertex(std::uint64_t id) {
  ct::FacetVertexKey result;
  result.source_instance_id = 17;
  result.first = id;
  return result;
}

inline bool VertexLess(const ct::FacetVertexKey& a,
                       const ct::FacetVertexKey& b) {
  return std::tie(a.source_instance_id, a.kind, a.first, a.second,
                  a.numerator, a.denominator, a.level, a.grid_i, a.grid_j) <
         std::tie(b.source_instance_id, b.kind, b.first, b.second,
                  b.numerator, b.denominator, b.level, b.grid_i, b.grid_j);
}

inline ct::FacetEdgeKey Edge(ct::FacetVertexKey a,
                             ct::FacetVertexKey b) {
  ct::FacetEdgeKey result;
  result.parent_boundary = true;
  if (VertexLess(b, a))
    std::swap(a, b);
  result.endpoints[0] = a;
  result.endpoints[1] = b;
  return result;
}

inline ct::RepresentedTrianglePath Path(
    std::uint64_t eid, const std::array<ct::Vec3, 3>& first,
    const std::array<ct::Vec3, 3>& second,
    std::uint64_t vertex_base = 0,
    ct::RepresentedMotion motion =
        ct::RepresentedMotion::LinearNodalV1) {
  ct::RepresentedTrianglePath result;
  result.key = {17, eid, 0, 0};
  result.motion = motion;
  if (!vertex_base)
    vertex_base = 10 * eid;
  for (unsigned i = 0; i < 3; ++i) {
    result.vertices[i].key = Vertex(vertex_base + i);
    result.vertices[i].endpoint[0] = first[i];
    result.vertices[i].endpoint[1] = second[i];
  }
  for (unsigned i = 0; i < 3; ++i)
    result.edge_keys[i] =
        Edge(result.vertices[i].key, result.vertices[(i + 1) % 3].key);
  return result;
}

inline ct::RepresentedTrianglePath Static(
    std::uint64_t eid, const std::array<ct::Vec3, 3>& value,
    std::uint64_t vertex_base = 0) {
  return Path(eid, value, value, vertex_base);
}

inline ct::RepresentedTrianglePath Permute(
    const ct::RepresentedTrianglePath& input,
    std::array<unsigned, 3> permutation) {
  auto result = input;
  for (unsigned i = 0; i < 3; ++i)
    result.vertices[i] = input.vertices[permutation[i]];
  for (unsigned i = 0; i < 3; ++i)
    result.edge_keys[i] =
        Edge(result.vertices[i].key, result.vertices[(i + 1) % 3].key);
  return result;
}

inline std::array<ct::Vec3, 3> BaseTriangle(double z = 0) {
  return {{{0, 0, z}, {2, 0, z}, {0, 2, z}}};
}

inline std::array<ct::RepresentedTrianglePath, 2> MixedExponentPaths() {
  const double huge = std::ldexp(1.0, 900);
  const double tiny = std::numeric_limits<double>::denorm_min();
  const std::array<ct::Vec3, 3> base{
      ct::Vec3{0, 0, 0}, {huge, 0, 0}, {0, huge, 0}};
  const std::array<ct::Vec3, 3> first{
      ct::Vec3{huge, 0, tiny}, {2 * huge, 0, tiny}, {huge, huge, tiny}};
  const std::array<ct::Vec3, 3> second{
      ct::Vec3{-huge, 0, tiny}, {0, 0, tiny}, {-huge, huge, tiny}};
  return {Static(10, base), Path(20, first, second)};
}

inline ct::RepresentedIntervalResult One(
    ct::RepresentedIntervalCrossing& owner,
    const std::vector<ct::RepresentedTrianglePath>& paths,
    ct::RepresentedTrianglePair pair = {0, 1}) {
  const auto report = owner.Certify(paths.data(), paths.size(), &pair, 1);
  EXPECT_EQ(report.status, S::Ok);
  const auto view = owner.results();
  EXPECT_TRUE(view.complete);
  EXPECT_EQ(view.count, 1u);
  return view.count == 1 ? view.data[0]
                         : ct::RepresentedIntervalResult{};
}

inline ct::RepresentedIntervalCrossing Owner(
    ct::RepresentedIntervalLimits limits = {}) {
  ct::RepresentedIntervalCrossing result;
  EXPECT_EQ(result.Initialize(limits).status, S::Ok);
  return result;
}

inline std::vector<unsigned char> Bytes(
    const ct::RepresentedIntervalResult* data, std::size_t count) {
  const auto* first = reinterpret_cast<const unsigned char*>(data);
  return {first, first + count * sizeof(*data)};
}

inline void SameFeature(const ct::RepresentedFeaturePathKey& a,
                        const ct::RepresentedFeaturePathKey& b) {
  EXPECT_EQ(a.kind, b.kind);
  if (a.kind == K::VertexFace) {
    EXPECT_EQ(std::memcmp(&a.vertex, &b.vertex, sizeof(a.vertex)), 0);
    EXPECT_EQ(std::memcmp(&a.face, &b.face, sizeof(a.face)), 0);
  }
  if (a.kind == K::EdgeEdge)
    EXPECT_EQ(std::memcmp(a.edges, b.edges, sizeof(a.edges)), 0);
}

}  // namespace represented_interval_test
