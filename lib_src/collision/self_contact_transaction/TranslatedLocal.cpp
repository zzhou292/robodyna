// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TranslatedLocal.h"
#include "Storage.h"

#include <algorithm>

namespace tlfea::contact::self_contact_transaction {

const FixedTriangleIntersection* FindPairIntersection(
    FixedTriangleIntersectionView intersections,
    const RepresentedIntervalPairKey& pair) noexcept {
  if (intersections.count && !intersections.data) return nullptr;
  for (std::size_t index = 0; index < intersections.count; ++index) {
    const auto& intersection = intersections.data[index];
    const auto& a = intersection.triangles[0];
    const auto& b = intersection.triangles[1];
    RepresentedIntervalPairKey key{{
        {a.source_instance_id, a.parent_eid, a.level, a.local_facet},
        {b.source_instance_id, b.parent_eid, b.level, b.local_facet}}};
    if (Compare(key.paths[1], key.paths[0]) < 0)
      std::swap(key.paths[0], key.paths[1]);
    if (Compare(key, pair) == 0) return &intersection;
  }
  return nullptr;
}

TranslatedLocalStatus NormalizeExactTranslatedLocal(
    FixedTriangleIntersectionView intersections,
    RepresentedIntervalResult* result) noexcept {
  if (!result) return TranslatedLocalStatus::InvalidInput;
  if (!HasExactCommonTranslationProof(result->geometry))
    return TranslatedLocalStatus::NotApplicable;
  if (!intersections.complete ||
      (intersections.count && !intersections.data) ||
      Compare(result->key.paths[0], result->key.paths[1]) >= 0 ||
      result->classification !=
          RepresentedIntervalClassification::CertifiedCrossingContact ||
      result->reason != RepresentedIntervalReason::None ||
      result->witness_time_numerator != 0 ||
      result->witness_time_depth != 0 || result->work != 1)
    return TranslatedLocalStatus::InvalidInput;

  const auto* intersection = FindPairIntersection(intersections, result->key);
  if (!intersection || RequiresIntersectionAdmission(*intersection))
    return TranslatedLocalStatus::NotApplicable;
  switch (intersection->local_exclusion) {
    case FixedTriangleLocalExclusion::IdenticalFace:
    case FixedTriangleLocalExclusion::SharedVertexOnly:
    case FixedTriangleLocalExclusion::SharedEdgeOnly:
      break;
    default:
      return TranslatedLocalStatus::InvalidInput;
  }
  result->geometry = RepresentedIntersectionGeometry::CertifiedLocalTopology;
  result->feature = {};
  result->feature.kind = RepresentedFeatureKind::TriangleIntersection;
  result->accepted_event = SIZE_MAX;
  // Preserve the actual native work and exact time-zero witness. No policy
  // subdivision was executed and no accepted VF/EE owner was synthesized.
  return TranslatedLocalStatus::Certified;
}

}  // namespace tlfea::contact::self_contact_transaction
