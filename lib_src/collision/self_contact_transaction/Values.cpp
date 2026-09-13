// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include <tuple>

namespace tlfea::contact::self_contact_transaction {
namespace {

template <class T>
int ScalarCompare(const T& a, const T& b) noexcept {
  return a < b ? -1 : (b < a ? 1 : 0);
}

int Compare(const FacetVertexKey& a,
            const FacetVertexKey& b) noexcept {
  const auto aa =
      std::tie(a.source_instance_id, a.kind, a.first, a.second,
               a.numerator, a.denominator, a.level, a.grid_i,
               a.grid_j);
  const auto bb =
      std::tie(b.source_instance_id, b.kind, b.first, b.second,
               b.numerator, b.denominator, b.level, b.grid_i,
               b.grid_j);
  return aa < bb ? -1 : (bb < aa ? 1 : 0);
}

int Compare(const FacetEdgeKey& a,
            const FacetEdgeKey& b) noexcept {
  int result = ScalarCompare(a.parent_boundary, b.parent_boundary);
  if (!result) result = ScalarCompare(a.parent_eid, b.parent_eid);
  if (!result) result = Compare(a.endpoints[0], b.endpoints[0]);
  if (!result) result = Compare(a.endpoints[1], b.endpoints[1]);
  return result;
}

RepresentedIntervalPairKey PairKey(
    const FixedTriangleKey& a,
    const FixedTriangleKey& b) noexcept {
  RepresentedIntervalPairKey result;
  result.paths[0] =
      {a.source_instance_id, a.parent_eid, a.level, a.local_facet};
  result.paths[1] =
      {b.source_instance_id, b.parent_eid, b.level, b.local_facet};
  if (self_contact_transaction::Compare(
          result.paths[1], result.paths[0]) < 0)
    std::swap(result.paths[0], result.paths[1]);
  return result;
}

bool PairPresent(const RepresentedIntervalPairKey* values,
                 std::size_t count,
                 const RepresentedIntervalPairKey& key) noexcept {
  std::size_t lower = 0, upper = count;
  while (lower < upper) {
    const auto middle = lower + (upper - lower) / 2;
    const int order =
        self_contact_transaction::Compare(values[middle], key);
    if (order < 0) lower = middle + 1;
    else upper = middle;
  }
  return lower < count &&
      self_contact_transaction::Compare(values[lower], key) == 0;
}

const RepresentedIntervalResult* Crossing(
    RepresentedIntervalResultView values,
    const RepresentedIntervalPairKey& key) noexcept {
  std::size_t lower = 0, upper = values.count;
  while (lower < upper) {
    const auto middle = lower + (upper - lower) / 2;
    const int order = self_contact_transaction::Compare(
        values.data[middle].key, key);
    if (order < 0) lower = middle + 1;
    else upper = middle;
  }
  return lower < values.count &&
      self_contact_transaction::Compare(
          values.data[lower].key, key) == 0
      ? values.data + lower : nullptr;
}

bool LocallyExcluded(FixedTriangleIntersectionView values,
                     const RepresentedIntervalPairKey& key) noexcept {
  for (std::size_t i = 0; i < values.count; ++i) {
    const auto& value = values.data[i];
    if (self_contact_transaction::Compare(
            PairKey(value.triangles[0], value.triangles[1]),
            key) == 0)
      return !RequiresIntersectionAdmission(value);
  }
  return false;
}

const SelfContactCrossingDecision* Decision(
    const CandidateValidationInput& input,
    const RepresentedIntervalPairKey& key,
    std::size_t* count) noexcept {
  const SelfContactCrossingDecision* result = nullptr;
  *count = 0;
  for (std::size_t i = 0; i < input.decision_count; ++i) {
    if (self_contact_transaction::Compare(
            input.decisions[i].pair, key) != 0) continue;
    result = input.decisions + i;
    ++*count;
  }
  return result;
}

bool RepresentedByAcceptedVertexFace(
    const RepresentedIntervalResult& crossing,
    const AcceptedEventIdentity* events,
    std::size_t event_count) noexcept {
  if (crossing.feature.kind != RepresentedFeatureKind::VertexFace)
    return false;
  FixedTriangleFeatureKey key;
  key.vertex_face.vertex = crossing.feature.vertex;
  key.vertex_face.target.SetFace({
      crossing.feature.face.source_instance_id,
      crossing.feature.face.parent_eid,
      crossing.feature.face.level,
      crossing.feature.face.local_facet});
  for (std::size_t event = 0; event < event_count; ++event)
    if (Same(events[event].feature, key)) return true;
  return false;
}

SelfContactTransactionReport Failure(
    SelfContactTransactionStatus status, const char* message,
    std::size_t pair = SIZE_MAX) noexcept {
  SelfContactTransactionReport result;
  result.status = status;
  result.pair = pair;
  result.message = message;
  return result;
}

}  // namespace

int Compare(const RepresentedTrianglePathKey& a,
            const RepresentedTrianglePathKey& b) noexcept {
  const auto aa =
      std::tie(a.source_instance_id, a.parent_eid, a.level,
               a.local_facet);
  const auto bb =
      std::tie(b.source_instance_id, b.parent_eid, b.level,
               b.local_facet);
  return aa < bb ? -1 : (bb < aa ? 1 : 0);
}

int Compare(const RepresentedIntervalPairKey& a,
            const RepresentedIntervalPairKey& b) noexcept {
  const int first = Compare(a.paths[0], b.paths[0]);
  return first ? first : Compare(a.paths[1], b.paths[1]);
}

bool Same(const FixedTriangleKey& a,
          const FixedTriangleKey& b) noexcept {
  return fixed_triangle_features::Compare(a, b) == 0;
}

bool Same(const FixedTriangleIntersection& a,
          const FixedTriangleIntersection& b) noexcept {
  return fixed_triangle_features::SameIntersectionPair(a, b) &&
      a.kind == b.kind && a.local_exclusion == b.local_exclusion;
}

bool Same(const FixedTriangleFeatureKey& a,
          const FixedTriangleFeatureKey& b) noexcept {
  return fixed_triangle_features::Compare(a, b) == 0;
}

SelfContactTransactionReport ValidateCandidatePublications(
    const CandidateValidationInput& input) noexcept {
  if (!input.canonical_pairs || !input.pair_count ||
      !input.features.complete || !input.intersections.complete ||
      !input.crossings.complete ||
      (input.features.count && !input.features.data) ||
      (input.intersections.count && !input.intersections.data) ||
      !input.crossings.data ||
      input.crossings.count != input.pair_count ||
      (input.decision_count && !input.decisions) ||
      (input.accepted_event_count && !input.accepted_events))
    return Failure(SelfContactTransactionStatus::InvalidInput,
        "Candidate publications or bounded ranges are incomplete");

  for (std::size_t pair = 0; pair < input.pair_count; ++pair) {
    if ((pair && Compare(input.canonical_pairs[pair - 1],
                         input.canonical_pairs[pair]) >= 0) ||
        Compare(input.crossings.data[pair].key,
                input.canonical_pairs[pair]) != 0)
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Represented interval results differ from the exact candidate roster",
          pair);
    if (input.crossings.data[pair].classification ==
        RepresentedIntervalClassification::Unresolved)
      return Failure(SelfContactTransactionStatus::UnresolvedCandidate,
          "Represented interval candidate remains unresolved", pair);
  }

  for (std::size_t feature = 0;
       feature < input.features.count; ++feature) {
    const auto& value = input.features.data[feature];
    if (!PairPresent(input.canonical_pairs, input.pair_count,
                     PairKey(value.triangles[0],
                             value.triangles[1])))
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Discovered feature belongs to a foreign candidate pair");
  }

  for (std::size_t intersection = 0;
       intersection < input.intersections.count; ++intersection) {
    const auto& value = input.intersections.data[intersection];
    const auto key = PairKey(
        value.triangles[0], value.triangles[1]);
    if (!PairPresent(input.canonical_pairs, input.pair_count, key))
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Discovered intersection belongs to a foreign candidate pair");
    if (RequiresIntersectionAdmission(value)) {
      const auto* result = Crossing(input.crossings, key);
      if (!result || result->classification !=
              RepresentedIntervalClassification::CertifiedCrossingContact)
        return Failure(SelfContactTransactionStatus::IdentityMismatch,
            "Required current intersection lacks its interval crossing",
            intersection);
    }
  }

  std::size_t required_decisions = 0;
  for (std::size_t pair = 0; pair < input.crossings.count; ++pair) {
    const auto& crossing = input.crossings.data[pair];
    if (crossing.classification !=
            RepresentedIntervalClassification::CertifiedCrossingContact ||
        LocallyExcluded(input.intersections, crossing.key))
      continue;
    ++required_decisions;
    std::size_t count = 0;
    const auto* decision = Decision(input, crossing.key, &count);
    if (!decision || count != 1)
      return Failure(SelfContactTransactionStatus::UnresolvedCandidate,
          "Nonlocal crossing lacks one exact policy decision", pair);
    if (decision->disposition ==
        SelfContactCrossingDisposition::RejectCandidate)
      return Failure(SelfContactTransactionStatus::CandidateRejected,
          "Crossing policy rejected the prepared candidate", pair);
    if (decision->disposition !=
            SelfContactCrossingDisposition::
                RepresentedByAcceptedVertexFace ||
        !RepresentedByAcceptedVertexFace(
            crossing, input.accepted_events,
            input.accepted_event_count))
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Crossing is not represented by an accepted VF force event", pair);
  }
  if (input.decision_count != required_decisions)
    return Failure(SelfContactTransactionStatus::IdentityMismatch,
        "Crossing decision roster has missing or foreign entries");
  return {};
}

}  // namespace tlfea::contact::self_contact_transaction
