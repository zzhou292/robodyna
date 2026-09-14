// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include <cmath>
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

std::size_t AcceptedVertexFace(
    const RepresentedIntervalResult& crossing,
    const AcceptedEventCertificate* events,
    std::size_t event_count) noexcept {
  if (crossing.feature.kind != RepresentedFeatureKind::VertexFace)
    return SIZE_MAX;
  FixedTriangleFeatureKey key;
  key.vertex_face.vertex = crossing.feature.vertex;
  key.vertex_face.target.SetFace({
      crossing.feature.face.source_instance_id,
      crossing.feature.face.parent_eid,
      crossing.feature.face.level,
      crossing.feature.face.local_facet});
  for (std::size_t event = 0; event < event_count; ++event) {
    const auto& certificate = events[event];
    const auto& accepted = certificate.event;
    double face_sum = 0;
    bool face_weights_valid = true;
    for (double weight : certificate.discovery.face_weights) {
      face_weights_valid = face_weights_valid &&
          IsFinite(weight) && weight >= 0 && weight <= 1;
      face_sum += weight;
    }
    if (!Same(accepted.feature, key) ||
        !Same(certificate.discovery.key, key) ||
        accepted.source_order != event ||
        ValidateWeightedSurfacePoint(
            accepted.endpoints[0], UINT32_MAX) != Status::kOk ||
        ValidateWeightedSurfacePoint(
            accepted.endpoints[1], UINT32_MAX) != Status::kOk ||
        !face_weights_valid || std::fabs(face_sum - 1) > 1e-12 ||
        !IsFinite(certificate.discovery.representation_error_m) ||
        certificate.discovery.representation_error_m < 0 ||
        accepted.classification.kind !=
            SelfContactPairKind::VertexFace ||
        accepted.classification.status !=
            SelfContactPairStatus::AdmittedVertexFace ||
        accepted.classification.excluded ||
        accepted.classification.local_incidence ||
        !accepted.classification.active[0] ||
        !accepted.classification.active[1] ||
        !PositiveSelfContactArea(
            accepted.classification.admitted_force_area_m2) ||
        accepted.classification.candidate_directed_area_m2.value !=
            accepted.classification.admitted_force_area_m2.value ||
        accepted.classification.candidate_directed_area_m2.lower !=
            accepted.classification.admitted_force_area_m2.lower ||
        accepted.classification.candidate_directed_area_m2.upper !=
            accepted.classification.admitted_force_area_m2.upper ||
        accepted.classification.candidate_directed_area_m2.error !=
            accepted.classification.admitted_force_area_m2.error ||
        ((certificate.discovery.local_features[0] == 3) ==
         (certificate.discovery.local_features[1] == 3)) ||
        certificate.vertex_facet == UINT32_MAX ||
        certificate.target_facet == UINT32_MAX)
      continue;
    return event;
  }
  return SIZE_MAX;
}

SelfContactTransactionReport Failure(
    SelfContactTransactionStatus status, const char* message,
    std::size_t pair = SIZE_MAX,
    RepresentedIntervalReason crossing_reason =
        RepresentedIntervalReason::None) noexcept {
  SelfContactTransactionReport result;
  result.status = status;
  result.pair = pair;
  result.crossing_reason = crossing_reason;
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

bool ExactFacetPair(const FixedTriangleFeatureCandidate& a,
                    const FixedTriangleFeatureCandidate& b) noexcept {
  const auto same = [](const FixedTriangleKey& first,
                       const FixedTriangleKey& second) {
    return fixed_triangle_features::Compare(first, second) == 0;
  };
  return (same(a.triangles[0], b.triangles[0]) &&
          same(a.triangles[1], b.triangles[1])) ||
      (same(a.triangles[0], b.triangles[1]) &&
       same(a.triangles[1], b.triangles[0]));
}

SelfContactTransactionReport ValidateCandidatePublications(
    const CandidateValidationInput& input) noexcept {
  if ((input.pair_count && !input.canonical_pairs) ||
      !input.features.complete || !input.intersections.complete ||
      !input.crossings.complete ||
      (input.features.count && !input.features.data) ||
      (input.intersections.count && !input.intersections.data) ||
      (input.crossings.count && !input.crossings.data) ||
      input.crossings.count != input.pair_count ||
      (input.accepted_event_count && !input.accepted_events) ||
      !input.outcome_count ||
      (input.pair_count &&
       (!input.outcomes ||
        input.outcome_capacity < input.pair_count)))
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
          "Represented interval candidate remains unresolved", pair,
          input.crossings.data[pair].reason);
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
      return Failure(SelfContactTransactionStatus::CandidateRejected,
          "Nonlocal current triangle intersection is rejected",
          intersection);
    }
  }

  for (std::size_t pair = 0; pair < input.crossings.count; ++pair) {
    const auto& crossing = input.crossings.data[pair];
    auto& outcome = input.outcomes[pair];
    outcome = {};
    outcome.pair = crossing.key;
    if (crossing.classification ==
        RepresentedIntervalClassification::CertifiedSeparated) {
      outcome.disposition =
          SelfContactCandidateDisposition::CertifiedSeparated;
      continue;
    }
    if (LocallyExcluded(input.intersections, crossing.key)) {
      outcome.disposition =
          SelfContactCandidateDisposition::ExcludedLocalIntersection;
      continue;
    }
    const auto accepted = AcceptedVertexFace(
        crossing, input.accepted_events,
        input.accepted_event_count);
    if (accepted == SIZE_MAX)
      return Failure(SelfContactTransactionStatus::CandidateRejected,
          crossing.feature.kind == RepresentedFeatureKind::EdgeEdge
              ? "Nonlocal EE crossing has no force-area policy"
              : "Crossing lacks its full accepted VF event certificate",
          pair);
    outcome.disposition =
        SelfContactCandidateDisposition::RepresentedByAcceptedVertexFace;
    outcome.accepted_event = accepted;
    outcome.source_order =
        input.accepted_events[accepted].event.source_order;
  }
  *input.outcome_count = input.pair_count;
  return {};
}

SelfContactTransactionReport ExpandFacetPairs(
    const SelfContactPairKey* keys, std::size_t key_count,
    const std::uint32_t* surface_to_active,
    std::size_t surface_parents,
    const std::uint32_t* parent_facet_offsets,
    std::size_t parents, SelfContactActivityView activity,
    FixedTrianglePair* output,
    std::size_t capacity, std::size_t* output_count) noexcept {
  if ((key_count && !keys) || !surface_to_active ||
      !surface_parents || !parent_facet_offsets || !parents ||
      !activity.base || !activity.current ||
      activity.parent_count != parents ||
      !output || !capacity || !output_count)
    return Failure(SelfContactTransactionStatus::InvalidInput,
        "Facet expansion inputs or fixed storage are invalid");
  if (parent_facet_offsets[0] != 0)
    return Failure(SelfContactTransactionStatus::IdentityMismatch,
        "Parent facet offsets do not begin at zero");
  for (std::size_t parent = 0; parent < parents; ++parent) {
    if (activity.base[parent] > 1 ||
        activity.current[parent] > activity.base[parent])
      return Failure(SelfContactTransactionStatus::ActivityFailure,
          "Facet expansion activity is invalid", parent);
    if (parent_facet_offsets[parent + 1] <=
        parent_facet_offsets[parent])
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Parent facet offsets are not strictly increasing", parent);
  }

  std::size_t required = 0;
  for (std::size_t pair = 0; pair < key_count; ++pair) {
    if (pair && keys[pair - 1] >= keys[pair])
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Broadphase pair keys are not complete canonical order", pair);
    const auto first_surface = FirstSurfaceParent(keys[pair]);
    const auto second_surface = SecondSurfaceParent(keys[pair]);
    if (first_surface >= surface_parents ||
        second_surface >= surface_parents ||
        first_surface >= second_surface)
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Broadphase pair key is not a canonical S0 pair", pair);
    const auto first = surface_to_active[first_surface];
    const auto second = surface_to_active[second_surface];
    if (first >= parents || second >= parents || first == second)
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Broadphase pair cannot map to two selected active-use parents",
          pair);
    if (!activity.current[first] || !activity.current[second])
      continue;
    const std::size_t first_facets =
        parent_facet_offsets[first + 1] -
        parent_facet_offsets[first];
    const std::size_t second_facets =
        parent_facet_offsets[second + 1] -
        parent_facet_offsets[second];
    if (first_facets &&
        second_facets > (capacity - required) / first_facets)
      return Failure(SelfContactTransactionStatus::ResourceLimit,
          "Complete parent-pair facet expansion exceeds capacity", pair);
    required += first_facets * second_facets;
  }

  std::size_t write = 0;
  for (std::size_t pair = 0; pair < key_count; ++pair) {
    const auto first =
        surface_to_active[FirstSurfaceParent(keys[pair])];
    const auto second =
        surface_to_active[SecondSurfaceParent(keys[pair])];
    if (!activity.current[first] || !activity.current[second])
      continue;
    for (std::uint32_t a = parent_facet_offsets[first];
         a < parent_facet_offsets[first + 1]; ++a) {
      for (std::uint32_t b = parent_facet_offsets[second];
           b < parent_facet_offsets[second + 1]; ++b) {
        output[write++] = a < b ? FixedTrianglePair{a, b}
                                : FixedTrianglePair{b, a};
      }
    }
  }
  std::sort(output, output + write,
            [](FixedTrianglePair a, FixedTrianglePair b) {
              return a.first < b.first ||
                  (a.first == b.first && a.second < b.second);
            });
  for (std::size_t pair = 1; pair < write; ++pair)
    if (output[pair - 1].first == output[pair].first &&
        output[pair - 1].second == output[pair].second)
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Complete facet expansion contains a duplicate pair", pair);
  *output_count = write;
  return {};
}

}  // namespace tlfea::contact::self_contact_transaction
