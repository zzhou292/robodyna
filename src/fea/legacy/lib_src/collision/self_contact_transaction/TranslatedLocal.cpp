// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TranslatedLocal.h"
#include "Storage.h"
#include "SortedIntersections.h"

#include <algorithm>

namespace tlfea::contact::self_contact_transaction {

namespace {
RepresentedIntervalPairKey CanonicalIntersectionKey(const FixedTriangleIntersection& value) noexcept {
  const auto& a = value.triangles[0];
  const auto& b = value.triangles[1];
  RepresentedIntervalPairKey key{{
      {a.source_instance_id, a.parent_eid, a.level, a.local_facet},
      {b.source_instance_id, b.parent_eid, b.level, b.local_facet}}};
  if (Compare(key.paths[1], key.paths[0]) < 0) std::swap(key.paths[0], key.paths[1]);
  return key;
}
void Visit(IntersectionLookupCounts* counts) noexcept {
  if (!counts) return;
  if (counts->rows == SIZE_MAX) counts->saturated = true;
  else ++counts->rows;
}
const FixedTriangleIntersection* FindRaw(FixedTriangleIntersectionView intersections,
    const RepresentedIntervalPairKey& pair, IntersectionLookupCounts* counts) noexcept {
  if (intersections.count && !intersections.data) return nullptr;
  for (std::size_t index = 0; index < intersections.count; ++index) {
    Visit(counts);
    if (Compare(CanonicalIntersectionKey(intersections.data[index]), pair) == 0)
      return intersections.data + index;
  }
  return nullptr;
}
}  // namespace

const FixedTriangleIntersection* FindPairIntersection(
    FixedTriangleIntersectionView intersections,
    const RepresentedIntervalPairKey& pair) noexcept {
  return FindRaw(intersections, pair, nullptr);
}

SortedIntersections::SortedIntersections(const FixedTriangleFeatureDiscovery& source) noexcept
    : SortedIntersections(source.intersections()) {}
SortedIntersections::SortedIntersections(FixedTriangleIntersectionView view) noexcept : view_(view) {
  if (!view.complete || (view.count && !view.data) ||
      view.count > (UINTPTR_MAX - reinterpret_cast<std::uintptr_t>(view.data)) /
                       sizeof(FixedTriangleIntersection)) return;
  for (std::size_t row = 0; row < view.count; ++row) {
    const auto& value = view.data[row];
    if (fixed_triangle_features::Compare(value.triangles[0], value.triangles[1]) >= 0 ||
        (row && !fixed_triangle_features::IntersectionLess(view.data[row - 1], value))) return;
  }
  ordered_ = true;
}
bool SortedIntersections::matches(FixedTriangleIntersectionView view) const noexcept {
  return ordered_ && view.complete && view.data == view_.data && view.count == view_.count;
}
const FixedTriangleIntersection* SortedIntersections::Find(FixedTriangleIntersectionView view,
    const RepresentedIntervalPairKey& pair, IntersectionLookupCounts* counts) const noexcept {
  if (!matches(view)) return FindRaw(view, pair, counts);
  std::size_t lower = 0, upper = view.count;
  while (lower < upper) {
    const auto middle = lower + (upper - lower) / 2;
    Visit(counts);
    if (Compare(CanonicalIntersectionKey(view.data[middle]), pair) < 0) lower = middle + 1;
    else upper = middle;
  }
  if (lower == view.count) return nullptr;
  Visit(counts);
  return Compare(CanonicalIntersectionKey(view.data[lower]), pair) == 0 ? view.data + lower : nullptr;
}
IntersectionLookupComparison CompareIntersectionLookup(FixedTriangleIntersectionView view,
    const RepresentedIntervalPairKey& key, const SortedIntersections& index) noexcept {
  IntersectionLookupComparison result;
  result.ordered = index.matches(view);
  const auto* original = FindRaw(view, key, &result.original_counts);
  const auto* indexed = index.Find(view, key, &result.indexed_counts);
  if (original) result.original = static_cast<std::size_t>(original - view.data);
  if (indexed) result.indexed = static_cast<std::size_t>(indexed - view.data);
  return result;
}

namespace {
TranslatedLocalStatus NormalizeImpl(FixedTriangleIntersectionView intersections,
    RepresentedIntervalResult* result, const SortedIntersections* index) noexcept {
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

  const auto* intersection = index ? index->Find(intersections, result->key)
                                   : FindPairIntersection(intersections, result->key);
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

}  // namespace

TranslatedLocalStatus NormalizeExactTranslatedLocal(FixedTriangleIntersectionView intersections,
    RepresentedIntervalResult* result) noexcept {
  return NormalizeImpl(intersections, result, nullptr);
}
TranslatedLocalStatus NormalizeExactTranslatedLocal(FixedTriangleIntersectionView intersections,
    RepresentedIntervalResult* result, const SortedIntersections& index) noexcept {
  return NormalizeImpl(intersections, result, &index);
}

}  // namespace tlfea::contact::self_contact_transaction
