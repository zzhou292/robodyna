// SPDX-License-Identifier: MIT
#pragma once

#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"

#include <algorithm>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace fixed_triangle_test {
namespace ct = tlfea::contact;

inline ct::FacetVertexKey Vertex(std::uint64_t id,
                                 std::uint64_t instance = 1) {
  ct::FacetVertexKey result;
  result.source_instance_id = instance;
  result.first = id;
  return result;
}

inline bool VertexLess(const ct::FacetVertexKey& a,
                       const ct::FacetVertexKey& b) {
  return a.first < b.first;
}

inline ct::CurrentFixedTriangle Triangle(
    std::uint64_t eid, unsigned local, const ct::Vec3 (&points)[3],
    const std::uint64_t (&ids)[3], std::uint64_t instance = 1) {
  ct::CurrentFixedTriangle result;
  result.key = {instance, eid, 0, local};
  for (unsigned i = 0; i < 3; ++i) {
    result.vertices[i] = points[i];
    result.vertex_keys[i] = Vertex(ids[i], instance);
  }
  for (unsigned i = 0; i < 3; ++i) {
    const auto a = result.vertex_keys[i];
    const auto b = result.vertex_keys[(i + 1) % 3];
    result.edge_keys[i].parent_boundary = true;
    result.edge_keys[i].endpoints[0] = VertexLess(a, b) ? a : b;
    result.edge_keys[i].endpoints[1] = VertexLess(a, b) ? b : a;
  }
  return result;
}

inline ct::FixedTriangleFeatureLimits Limits(std::size_t pairs,
                                              std::size_t raw_features,
                                              std::size_t features,
                                              std::size_t raw_intersections,
                                              std::size_t intersections) {
  ct::FixedTriangleFeatureLimits result;
  result.max_input_pairs = pairs;
  result.max_triangle_references = 2 * pairs;
  result.max_vertex_references = 6 * pairs;
  result.max_edge_references = 6 * pairs;
  result.max_raw_feature_candidates = raw_features;
  result.max_feature_candidates = features;
  result.max_raw_intersections = raw_intersections;
  result.max_intersections = intersections;
  result.max_host_bytes = 64u << 20;
  return result;
}

inline ct::FixedTriangleFeatureLimits Limits(std::size_t pairs = 32) {
  return Limits(pairs, 15 * pairs, 15 * pairs, pairs, pairs);
}

inline void Initialize(ct::FixedTriangleFeatureDiscovery* result,
                       ct::FixedTriangleFeatureLimits limits = Limits()) {
  const auto report = result->Initialize(limits);
  if (report.status != ct::FixedTriangleDiscoveryStatus::Ok)
    throw std::runtime_error(report.message);
}

inline bool Same(const ct::FixedTriangleKey& a,
                 const ct::FixedTriangleKey& b) {
  return a.source_instance_id == b.source_instance_id &&
         a.parent_eid == b.parent_eid && a.level == b.level &&
         a.local_facet == b.local_facet;
}

inline bool Same(const ct::FacetVertexKey& a,
                 const ct::FacetVertexKey& b) {
  return a.source_instance_id == b.source_instance_id &&
         a.kind == b.kind && a.first == b.first && a.second == b.second &&
         a.numerator == b.numerator && a.denominator == b.denominator &&
         a.level == b.level && a.grid_i == b.grid_i &&
         a.grid_j == b.grid_j;
}

inline bool Same(const ct::FacetEdgeKey& a,
                 const ct::FacetEdgeKey& b) {
  return a.parent_boundary == b.parent_boundary &&
         a.parent_eid == b.parent_eid &&
         Same(a.endpoints[0], b.endpoints[0]) &&
         Same(a.endpoints[1], b.endpoints[1]);
}

inline bool Same(const ct::FixedTriangleStratumKey& a,
                 const ct::FixedTriangleStratumKey& b) {
  if (a.kind != b.kind)
    return false;
  if (a.kind == ct::FixedTriangleStratumKind::Vertex)
    return Same(a.vertex, b.vertex);
  if (a.kind == ct::FixedTriangleStratumKind::Edge)
    return Same(a.edge, b.edge);
  return Same(a.face, b.face);
}

inline bool Same(const ct::FixedTriangleFeatureKey& a,
                 const ct::FixedTriangleFeatureKey& b) {
  if (a.kind != b.kind)
    return false;
  if (a.kind == ct::FixedTriangleCandidateKind::VertexFace)
    return Same(a.vertex_face.vertex, b.vertex_face.vertex) &&
           Same(a.vertex_face.target, b.vertex_face.target);
  return Same(a.edge_edge.edges[0], b.edge_edge.edges[0]) &&
         Same(a.edge_edge.edges[1], b.edge_edge.edges[1]);
}

struct Task {
  std::uint64_t first_eid = 0;
  unsigned first_local = 0;
  std::uint64_t second_eid = 0;
  unsigned second_local = 0;
  ct::FixedTriangleCandidateKind kind =
      ct::FixedTriangleCandidateKind::VertexFace;
  unsigned local_a = 0;
  unsigned local_b = 0;

  bool operator<(const Task& other) const {
    return std::tie(first_eid, first_local, second_eid, second_local, kind,
                    local_a, local_b) <
           std::tie(other.first_eid, other.first_local, other.second_eid,
                    other.second_local, other.kind, other.local_a,
                    other.local_b);
  }
  bool operator==(const Task& other) const {
    return !(*this < other) && !(other < *this);
  }
};

inline Task TaskOf(const ct::FixedTriangleFeatureCandidate& value) {
  return {value.triangles[0].parent_eid,
          value.triangles[0].local_facet,
          value.triangles[1].parent_eid,
          value.triangles[1].local_facet,
          value.key.kind,
          value.local_features[0],
          value.local_features[1]};
}

inline std::vector<Task> Tasks(ct::FixedTriangleFeatureView view) {
  std::vector<Task> result;
  for (std::size_t i = 0; i < view.count; ++i)
    result.push_back(TaskOf(view.data[i]));
  return result;
}

}  // namespace fixed_triangle_test
