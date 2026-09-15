// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../FixedTriangleFeatureDiscovery.h"

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

CurrentFixedTriangle Identity(const FixedContactFacet& facet) noexcept {
  CurrentFixedTriangle result;
  result.key = Key(facet);
  for (unsigned local = 0; local < 3; ++local) {
    result.vertex_keys[local] = facet.vertex_keys[local];
    result.edge_keys[local] = facet.edge_keys[local];
  }
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
  for (std::size_t pair = 0; pair < pair_count; ++pair) {
    const auto first = Identity(facets[pairs[pair].first]);
    const auto second = Identity(facets[pairs[pair].second]);
    const auto status = BuildFixedTriangleFeatureTaskMask(
        first, second, masks + pair);
    if (status != FixedTriangleDiscoveryStatus::Ok)
      return Failure(
          S::IdentityMismatch,
          "Local feature task mask pair identity is invalid", pair);
  }
  return {};
}

}  // namespace tlfea::contact::self_contact_transaction
