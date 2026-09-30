// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
namespace tlfea::contact::self_contact_transaction::detail {
// One unchanged validation/compaction fold for the original scalar classifier
// and a current numerical batch view. The classifier is private, not an issuer.
template <class Classifier>
SelfContactTransactionReport FilterAcceptedFacetPairsWith(
    const SelfContactActiveUseBinding& active_use,
    const CurrentFixedTriangle* triangles,
    const MotionSupport* motion, std::size_t facets,
    FixedTrianglePair* pairs, std::size_t* pair_count,
    Classifier ClassifyAcceptedFacetPair) noexcept {
  using S = SelfContactTransactionStatus;
  const auto Failure = [](S status, const char* message,
      std::size_t candidate = SIZE_MAX, std::size_t pair = SIZE_MAX) {
    SelfContactTransactionReport result;
    result.status = status; result.message = message;
    result.candidate = candidate; result.pair = pair;
    return result;
  };

  if (!triangles || !motion || !facets || !pairs || !pair_count)
    return Failure(S::InvalidInput,
        "Accepted facet-pair filter storage is incomplete");
  const auto facet_uses = active_use.facet_uses();
  const auto parents = active_use.parents();
  if (facet_uses.size() != facets)
    return Failure(S::IdentityMismatch,
        "Accepted facet-pair filter inventory is incomplete");
  std::size_t write = 0;
  for (std::size_t pair = 0; pair < *pair_count; ++pair) {
    const auto value = pairs[pair];
    if (value.first >= facets || value.second >= facets ||
        value.first == value.second)
      return Failure(S::IdentityMismatch,
          "Rigid facet-pair filter received an invalid exact pair",
          SIZE_MAX, pair);
    const auto first_parent = facet_uses[value.first].parent;
    const auto second_parent = facet_uses[value.second].parent;
    if (first_parent >= parents.size() ||
        second_parent >= parents.size())
      return Failure(S::IdentityMismatch,
          "Accepted facet pair has no active parent",
          SIZE_MAX, pair);
    const auto filtered = ClassifyAcceptedFacetPair(
        triangles[value.first],
        parents[first_parent].reference_half_thickness_m,
        motion[value.first].complete_rigid_group,
        triangles[value.second],
        parents[second_parent].reference_half_thickness_m,
        motion[value.second].complete_rigid_group);
    if (filtered.status != SelfContactFacetFilterStatus::Ok)
      return Failure(S::IdentityMismatch,
          "Accepted facet filter input is invalid",
          SIZE_MAX, pair);
    if (filtered.category !=
        SelfContactFacetFilterCategory::ExactRemaining)
      continue;
    pairs[write++] = value;
  }
  *pair_count = write;
  return {};

}
}  // namespace tlfea::contact::self_contact_transaction::detail
