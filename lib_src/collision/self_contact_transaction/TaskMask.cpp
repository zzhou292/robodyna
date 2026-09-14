// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include <algorithm>

namespace tlfea::contact::self_contact_transaction {
namespace {

using S = SelfContactTransactionStatus;

SelfContactTransactionReport Failure(
    S status, const char* message, std::size_t pair = SIZE_MAX) noexcept {
  SelfContactTransactionReport result;
  result.status = status;
  result.pair = pair;
  result.message = message;
  return result;
}

FixedTriangleKey Key(const FixedContactFacet& facet) noexcept {
  return {facet.source_instance_id, facet.source.source_parent_id,
          facet.level, facet.local_facet};
}

bool Same(const FacetVertexKey& a,
          const FacetVertexKey& b) noexcept {
  return fixed_triangle_features::Compare(a, b) == 0;
}

bool VertexInFacet(const FacetVertexKey& vertex,
                   const FixedContactFacet& facet) noexcept {
  for (const auto& candidate : facet.vertex_keys)
    if (Same(vertex, candidate))
      return true;
  return false;
}

bool EdgesShareEndpoint(const FacetEdgeKey& a,
                        const FacetEdgeKey& b) noexcept {
  for (const auto& first : a.endpoints)
    for (const auto& second : b.endpoints)
      if (Same(first, second))
        return true;
  return false;
}

FixedTriangleFeatureTaskMask LocalMask(
    const FixedContactFacet& input_a,
    const FixedContactFacet& input_b) noexcept {
  const FixedContactFacet* a = &input_a;
  const FixedContactFacet* b = &input_b;
  if (fixed_triangle_features::Compare(Key(*b), Key(*a)) < 0)
    std::swap(a, b);

  FixedTriangleFeatureTaskMask result;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    if (VertexInFacet(a->vertex_keys[vertex], *b))
      result.local_tasks |= FixedTriangleFeatureTaskBit(
          FixedTriangleVertexFaceTaskSlot(0, vertex));
    if (VertexInFacet(b->vertex_keys[vertex], *a))
      result.local_tasks |= FixedTriangleFeatureTaskBit(
          FixedTriangleVertexFaceTaskSlot(1, vertex));
  }
  for (unsigned edge_a = 0; edge_a < 3; ++edge_a)
    for (unsigned edge_b = 0; edge_b < 3; ++edge_b)
      if (EdgesShareEndpoint(a->edge_keys[edge_a],
                             b->edge_keys[edge_b]))
        result.local_tasks |= FixedTriangleFeatureTaskBit(
            FixedTriangleEdgeEdgeTaskSlot(edge_a, edge_b));
  return result;
}

}  // namespace

SelfContactTransactionReport BuildLocalFeatureTaskMasks(
    const FixedContactFacet* facets, std::size_t facet_count,
    const FixedTrianglePair* pairs, std::size_t pair_count,
    FixedTriangleFeatureTaskMask* masks,
    std::size_t mask_capacity) noexcept {
  if (!pair_count)
    return {};
  if (!facets || !facet_count || !pairs || !masks)
    return Failure(
        S::InvalidInput, "Local feature task mask storage is incomplete");
  if (pair_count > mask_capacity)
    return Failure(
        S::ResourceLimit, "Local feature task mask capacity exceeded");

  // Validate the complete chunk before publishing any mask.
  for (std::size_t pair = 0; pair < pair_count; ++pair) {
    if (pairs[pair].first >= facet_count ||
        pairs[pair].second >= facet_count ||
        pairs[pair].first == pairs[pair].second ||
        fixed_triangle_features::Compare(
            Key(facets[pairs[pair].first]),
            Key(facets[pairs[pair].second])) == 0)
      return Failure(
          S::IdentityMismatch,
          "Local feature task mask pair identity is invalid", pair);
  }
  for (std::size_t pair = 0; pair < pair_count; ++pair)
    masks[pair] = LocalMask(
        facets[pairs[pair].first], facets[pairs[pair].second]);
  return {};
}

}  // namespace tlfea::contact::self_contact_transaction
