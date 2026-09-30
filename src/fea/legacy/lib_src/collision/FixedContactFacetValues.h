// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FixedContactFacetTypes.h"

namespace tlfea::contact {
// Algebra over a previously resolved facet; this authenticates neither source
// IDs nor an owner stage. Original cyclic slots are retained, including zeros.
// Pointee extents and input/output nonoverlap are caller contracts.
TL_SURFACE_HD inline Status ComposeFacetPoint(const FixedContactFacet& facet,
    const double* barycentric, std::uint32_t node_count, WeightedSurfacePoint* output) {
  if (!barycentric || !output) return Status::kInvalidArgument;
  double sum = 0;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    const double weight = barycentric[vertex];
    if (!IsFinite(weight) || weight < 0 || weight > 1) return Status::kInvalidArgument;
    sum += weight;
    const auto status = ValidateWeightedSurfacePoint(facet.vertices[vertex], node_count);
    if (status != Status::kOk) return status;
    if (facet.vertices[vertex].count != facet.vertices[0].count) return Status::kInvalidArgument;
  }
  if (::fabs(sum - 1) > 1e-12) return Status::kInvalidArgument;
  WeightedSurfacePoint next;
  next.count = facet.vertices[0].count;
  for (unsigned node = 0; node < next.count; ++node) {
    next.nodes[node] = facet.vertices[0].nodes[node];
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      if (facet.vertices[vertex].nodes[node] != next.nodes[node]) return Status::kInvalidArgument;
      next.weights[node] += barycentric[vertex] * facet.vertices[vertex].weights[node];
    }
  }
  const auto status = ValidateWeightedSurfacePoint(next, node_count);
  if (status == Status::kOk) *output = next;
  return status;
}

inline bool SameFacetVertexKey(const FacetVertexKey& a, const FacetVertexKey& b) noexcept {
  return a.source_instance_id == b.source_instance_id && a.kind == b.kind &&
      a.first == b.first && a.second == b.second && a.numerator == b.numerator &&
      a.denominator == b.denominator && a.level == b.level && a.grid_i == b.grid_i && a.grid_j == b.grid_j;
}
inline bool SameFacetEdgeKey(const FacetEdgeKey& a, const FacetEdgeKey& b) noexcept {
  return a.parent_boundary == b.parent_boundary && a.parent_eid == b.parent_eid &&
      SameFacetVertexKey(a.endpoints[0], b.endpoints[0]) && SameFacetVertexKey(a.endpoints[1], b.endpoints[1]);
}
} // namespace tlfea::contact
