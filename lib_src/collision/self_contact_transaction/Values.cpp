// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include <algorithm>
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

template <class T>
void HashValue(T value, std::uint64_t* hash) noexcept {
  std::uint64_t bits = static_cast<std::uint64_t>(value);
  for (unsigned byte = 0; byte < sizeof(bits); ++byte) {
    *hash ^= static_cast<unsigned char>(bits >> (8 * byte));
    *hash *= 1099511628211ull;
  }
}

void Hash(const FacetVertexKey& key, std::uint64_t* hash) noexcept {
  HashValue(key.source_instance_id, hash);
  HashValue(key.kind, hash);
  HashValue(key.first, hash);
  HashValue(key.second, hash);
  HashValue(key.numerator, hash);
  HashValue(key.denominator, hash);
  HashValue(key.level, hash);
  HashValue(key.grid_i, hash);
  HashValue(key.grid_j, hash);
}

void Hash(const FacetEdgeKey& key, std::uint64_t* hash) noexcept {
  HashValue(key.parent_boundary, hash);
  HashValue(key.parent_eid, hash);
  Hash(key.endpoints[0], hash);
  Hash(key.endpoints[1], hash);
}

void Hash(const FixedTriangleKey& key, std::uint64_t* hash) noexcept {
  HashValue(key.source_instance_id, hash);
  HashValue(key.parent_eid, hash);
  HashValue(key.level, hash);
  HashValue(key.local_facet, hash);
}

void Hash(const FixedTriangleFeatureKey& key,
          std::uint64_t* hash) noexcept {
  HashValue(key.kind, hash);
  if (key.kind == FixedTriangleCandidateKind::VertexFace) {
    Hash(key.vertex_face.vertex, hash);
    HashValue(key.vertex_face.target.kind, hash);
    if (key.vertex_face.target.kind ==
        FixedTriangleStratumKind::Vertex)
      Hash(key.vertex_face.target.vertex, hash);
    else if (key.vertex_face.target.kind ==
             FixedTriangleStratumKind::Edge)
      Hash(key.vertex_face.target.edge, hash);
    else
      Hash(key.vertex_face.target.face, hash);
  } else {
    Hash(key.edge_edge.edges[0], hash);
    Hash(key.edge_edge.edges[1], hash);
  }
}

bool Same(Vec3 a, Vec3 b) noexcept {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool Same(const WeightedSurfacePoint& a,
          const WeightedSurfacePoint& b) noexcept {
  if (a.count != b.count) return false;
  for (unsigned i = 0; i < 4; ++i)
    if (a.nodes[i] != b.nodes[i] ||
        a.weights[i] != b.weights[i])
      return false;
  return true;
}

bool Same(const Q4CertifiedIntegral& a,
          const Q4CertifiedIntegral& b) noexcept {
  return a.value == b.value && a.lower == b.lower &&
      a.upper == b.upper && a.error == b.error;
}

bool Same(const SelfContactSupportClassification& a,
          const SelfContactSupportClassification& b) noexcept {
  return a.status == b.status &&
      a.complete_rigid_group == b.complete_rigid_group &&
      a.nonzero_slots == b.nonzero_slots &&
      a.rigid_slots == b.rigid_slots &&
      a.cin_master_slots == b.cin_master_slots;
}

bool Same(const SelfContactPairClassification& a,
          const SelfContactPairClassification& b) noexcept {
  if (a.binding_identity != b.binding_identity ||
      a.activity_base_identity != b.activity_base_identity ||
      a.activity_current_identity != b.activity_current_identity ||
      a.activity_parent_count != b.activity_parent_count ||
      a.kind != b.kind || a.edge_edge_case != b.edge_edge_case ||
      a.status != b.status || a.tied != b.tied ||
      a.local_incidence != b.local_incidence ||
      a.excluded != b.excluded ||
      !Same(a.candidate_directed_area_m2,
            b.candidate_directed_area_m2) ||
      !Same(a.admitted_force_area_m2,
            b.admitted_force_area_m2))
    return false;
  for (unsigned i = 0; i < 2; ++i)
    if (!Same(a.endpoint_support[i], b.endpoint_support[i]) ||
        a.parent[i] != b.parent[i] ||
        a.feature[i] != b.feature[i] ||
        a.active[i] != b.active[i] ||
        a.reference_half_thickness_m[i] !=
            b.reference_half_thickness_m[i])
      return false;
  return true;
}

bool SameCandidateValue(
    const FixedTriangleFeatureCandidate& a,
    const FixedTriangleFeatureCandidate& b) noexcept {
  if (!self_contact_transaction::Same(a.key, b.key) ||
      a.distance_m != b.distance_m ||
      a.representation_error_m != b.representation_error_m)
    return false;
  for (unsigned i = 0; i < 2; ++i)
    if (!Same(a.points[i], b.points[i]) ||
        a.edge_parameters[i] != b.edge_parameters[i])
      return false;
  if (a.key.kind == FixedTriangleCandidateKind::VertexFace) {
    const unsigned target_a = a.local_features[0] == 3 ? 0 : 1;
    const unsigned target_b = b.local_features[0] == 3 ? 0 : 1;
    if (fixed_triangle_features::Compare(
            a.triangles[target_a],
            b.triangles[target_b]) == 0)
      for (unsigned i = 0; i < 3; ++i)
        if (a.face_weights[i] != b.face_weights[i])
          return false;
  }
  return true;
}

bool SameCertificate(const AcceptedEventCertificate& a,
                     const AcceptedEventCertificate& b) noexcept {
  if (a.kind != b.kind ||
      !self_contact_transaction::Same(
          a.event.feature, b.event.feature) ||
      a.event.vertex_use != b.event.vertex_use ||
      a.event.facet_use != b.event.facet_use ||
      a.event.edge_use[0] != b.event.edge_use[0] ||
      a.event.edge_use[1] != b.event.edge_use[1] ||
      !Same(a.event.endpoints[0], b.event.endpoints[0]) ||
      !Same(a.event.endpoints[1], b.event.endpoints[1]) ||
      !Same(a.event.classification, b.event.classification))
    return false;
  if (a.kind == AcceptedEventCertificateKind::EdgeEdge)
    return true;
  return SameCandidateValue(a.discovery, b.discovery) &&
      a.target_facet == b.target_facet;
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

std::size_t AcceptedFeature(
    const RepresentedIntervalResult& crossing,
    const AcceptedEventCertificate* events,
    std::size_t event_count, bool* edge_edge) noexcept {
  if (!edge_edge ||
      (crossing.feature.kind != RepresentedFeatureKind::VertexFace &&
       crossing.feature.kind != RepresentedFeatureKind::EdgeEdge))
    return SIZE_MAX;
  FixedTriangleFeatureKey key;
  if (crossing.feature.kind == RepresentedFeatureKind::VertexFace) {
    key.vertex_face.vertex = crossing.feature.vertex;
    key.vertex_face.target.SetFace({
        crossing.feature.face.source_instance_id,
        crossing.feature.face.parent_eid,
        crossing.feature.face.level,
        crossing.feature.face.local_facet});
  } else {
    key.SetEdgeEdge();
    key.edge_edge.edges[0] = crossing.feature.edges[0];
    key.edge_edge.edges[1] = crossing.feature.edges[1];
  }
  std::size_t lower = 0, upper = event_count;
  while (lower < upper) {
    const auto middle = lower + (upper - lower) / 2;
    if (fixed_triangle_features::Compare(
            events[middle].event.feature, key) < 0)
      lower = middle + 1;
    else
      upper = middle;
  }
  if (lower == event_count ||
      !self_contact_transaction::Same(
          events[lower].event.feature, key))
    return SIZE_MAX;
  const auto& certificate = events[lower];
  const auto& accepted = certificate.event;
  if (!self_contact_transaction::Same(
          certificate.discovery.key, key) ||
      (crossing.feature.kind == RepresentedFeatureKind::VertexFace &&
       self_contact_transaction::Compare(
           PairKey(certificate.discovery.triangles[0],
                   certificate.discovery.triangles[1]),
           crossing.key) != 0) ||
      accepted.source_order != lower ||
      ValidateWeightedSurfacePoint(
          accepted.endpoints[0], UINT32_MAX) != Status::kOk ||
      ValidateWeightedSurfacePoint(
          accepted.endpoints[1], UINT32_MAX) != Status::kOk ||
      !IsFinite(certificate.discovery.representation_error_m) ||
      certificate.discovery.representation_error_m < 0 ||
      accepted.classification.excluded ||
      accepted.classification.local_incidence ||
      !accepted.classification.active[0] ||
      !accepted.classification.active[1] ||
      !PositiveSelfContactArea(
          accepted.classification.admitted_force_area_m2) ||
      !Same(accepted.classification.candidate_directed_area_m2,
            accepted.classification.admitted_force_area_m2))
    return SIZE_MAX;

  if (crossing.feature.kind == RepresentedFeatureKind::EdgeEdge) {
    const bool strict_interior =
        certificate.discovery.edge_parameters[0] > 0 &&
        certificate.discovery.edge_parameters[0] < 1 &&
        certificate.discovery.edge_parameters[1] > 0 &&
        certificate.discovery.edge_parameters[1] < 1;
    const bool boundary_minimum =
        certificate.discovery.edge_parameters[0] == 0 ||
        certificate.discovery.edge_parameters[0] == 1 ||
        certificate.discovery.edge_parameters[1] == 0 ||
        certificate.discovery.edge_parameters[1] == 1;
    const double gap =
        (certificate.discovery.distance_m -
         accepted.classification.reference_half_thickness_m[0]) -
        accepted.classification.reference_half_thickness_m[1];
    if (certificate.kind != AcceptedEventCertificateKind::EdgeEdge ||
        accepted.feature.kind != FixedTriangleCandidateKind::EdgeEdge ||
        accepted.vertex_use != UINT32_MAX ||
        accepted.facet_use != UINT32_MAX ||
        accepted.edge_use[0] == UINT32_MAX ||
        accepted.edge_use[1] == UINT32_MAX ||
        certificate.vertex_facet != UINT32_MAX ||
        certificate.target_facet != UINT32_MAX ||
        certificate.edge_facet[0] == UINT32_MAX ||
        certificate.edge_facet[1] == UINT32_MAX ||
        accepted.classification.kind != SelfContactPairKind::EdgeEdge ||
        accepted.classification.status !=
            SelfContactPairStatus::AdmittedEdgeEdge ||
        !IsFinite(certificate.discovery.distance_m) ||
        certificate.discovery.distance_m < 0 ||
        !IsFinite(certificate.discovery.edge_parameters[0]) ||
        !IsFinite(certificate.discovery.edge_parameters[1]) ||
        certificate.discovery.edge_parameters[0] < 0 ||
        certificate.discovery.edge_parameters[0] > 1 ||
        certificate.discovery.edge_parameters[1] < 0 ||
        certificate.discovery.edge_parameters[1] > 1 ||
        !(certificate.discovery.distance_m == 0 ||
          strict_interior || boundary_minimum) ||
        !IsFinite(gap) || gap > 0)
      return SIZE_MAX;
    *edge_edge = true;
    return lower;
  }

  double face_sum = 0;
  bool face_weights_valid = true;
  for (double weight : certificate.discovery.face_weights) {
    face_weights_valid = face_weights_valid &&
        IsFinite(weight) && weight >= 0 && weight <= 1;
    face_sum += weight;
  }
  if (certificate.kind != AcceptedEventCertificateKind::VertexFace ||
      accepted.feature.kind != FixedTriangleCandidateKind::VertexFace ||
      accepted.edge_use[0] != UINT32_MAX ||
      accepted.edge_use[1] != UINT32_MAX ||
      !face_weights_valid || std::fabs(face_sum - 1) > 1e-12 ||
      accepted.classification.kind !=
          SelfContactPairKind::VertexFace ||
      accepted.classification.status !=
          SelfContactPairStatus::AdmittedVertexFace ||
      ((certificate.discovery.local_features[0] == 3) ==
       (certificate.discovery.local_features[1] == 3)) ||
      certificate.vertex_facet == UINT32_MAX ||
      certificate.target_facet == UINT32_MAX ||
      certificate.edge_facet[0] != UINT32_MAX ||
      certificate.edge_facet[1] != UINT32_MAX)
    return SIZE_MAX;
  *edge_edge = false;
  return lower;
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

SelfContactTransactionReport ValidateCompleteTriangleIdentities(
    const CurrentFixedTriangle* triangles, std::size_t count,
    const std::uint32_t* vertex_order,
    const std::uint32_t* edge_order) noexcept {
  if (!triangles || !count || !vertex_order || !edge_order ||
      count > UINT32_MAX / 3)
    return Failure(SelfContactTransactionStatus::InvalidInput,
        "Complete triangle identity ledger is absent");
  const auto same_point = [](Vec3 a, Vec3 b) noexcept {
    return a.x == b.x && a.y == b.y && a.z == b.z;
  };
  const auto canonical_endpoint =
      [](const CurrentFixedTriangle& triangle, unsigned edge,
         unsigned endpoint) noexcept {
        const auto& key =
            triangle.edge_keys[edge].endpoints[endpoint];
        for (unsigned vertex = 0; vertex < 3; ++vertex)
          if (fixed_triangle_features::Compare(
                  triangle.vertex_keys[vertex], key) == 0)
            return triangle.vertices[vertex];
        return Vec3{};
      };
  for (std::size_t i = 0; i < 3 * count; ++i) {
    if (vertex_order[i] >= 3 * count ||
        edge_order[i] >= 3 * count)
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Complete triangle identity order is out of range", i);
    if (!i) continue;
    const auto old_vertex = vertex_order[i - 1];
    const auto new_vertex = vertex_order[i];
    const auto& old_triangle = triangles[old_vertex / 3];
    const auto& new_triangle = triangles[new_vertex / 3];
    if (fixed_triangle_features::Compare(
            old_triangle.vertex_keys[old_vertex % 3],
            new_triangle.vertex_keys[new_vertex % 3]) == 0 &&
        !same_point(old_triangle.vertices[old_vertex % 3],
                    new_triangle.vertices[new_vertex % 3]))
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Canonical vertex has inconsistent current coordinates",
          new_vertex);

    const auto old_edge = edge_order[i - 1];
    const auto new_edge = edge_order[i];
    const auto& old_edge_triangle = triangles[old_edge / 3];
    const auto& new_edge_triangle = triangles[new_edge / 3];
    if (fixed_triangle_features::Compare(
            old_edge_triangle.edge_keys[old_edge % 3],
            new_edge_triangle.edge_keys[new_edge % 3]) != 0)
      continue;
    if (!same_point(canonical_endpoint(
                        old_edge_triangle, old_edge % 3, 0),
                    canonical_endpoint(
                        new_edge_triangle, new_edge % 3, 0)) ||
        !same_point(canonical_endpoint(
                        old_edge_triangle, old_edge % 3, 1),
                    canonical_endpoint(
                        new_edge_triangle, new_edge % 3, 1)))
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Canonical edge has inconsistent current coordinates",
          new_edge);
  }
  return {};
}

SelfContactTransactionReport MergeAcceptedEventChunk(
    const AcceptedEventCertificate* input, std::size_t input_count,
    AcceptedEventCertificate* ledger, std::size_t ledger_capacity,
    std::uint32_t* hash_slots, std::size_t hash_capacity,
    std::size_t* ledger_count) noexcept {
  if ((input_count && !input) || !ledger || !ledger_capacity ||
      !hash_slots || !hash_capacity || !ledger_count ||
      *ledger_count > ledger_capacity ||
      ledger_capacity > UINT32_MAX)
    return Failure(SelfContactTransactionStatus::InvalidInput,
                   "Global accepted-event ledger is incomplete");
  for (std::size_t i = 0; i < input_count; ++i) {
    std::uint64_t hash = 1469598103934665603ull;
    Hash(input[i].event.feature, &hash);
    std::size_t slot = hash % hash_capacity;
    bool inserted = false;
    for (std::size_t probe = 0; probe < hash_capacity; ++probe) {
      const auto value = hash_slots[slot];
      if (value == UINT32_MAX) {
        if (*ledger_count == ledger_capacity)
          return Failure(SelfContactTransactionStatus::ResourceLimit,
              "Complete accepted-event ledger exceeds its hard cap",
              *ledger_count);
        ledger[*ledger_count] = input[i];
        hash_slots[slot] =
            static_cast<std::uint32_t>((*ledger_count)++);
        inserted = true;
        break;
      }
      if (value >= *ledger_count)
        return Failure(SelfContactTransactionStatus::IdentityMismatch,
            "Accepted-event hash index is corrupt", slot);
      if (self_contact_transaction::Same(
              ledger[value].event.feature,
              input[i].event.feature)) {
        if (!SameCertificate(ledger[value], input[i]))
          return Failure(SelfContactTransactionStatus::IdentityMismatch,
              "Repeated immutable event identity disagrees", value);
        inserted = true;
        break;
      }
      slot = slot + 1 == hash_capacity ? 0 : slot + 1;
    }
    if (!inserted)
      return Failure(SelfContactTransactionStatus::ResourceLimit,
          "Accepted-event hash ledger has no free slot", i);
  }
  return {};
}

SelfContactTransactionReport FinalizeAcceptedEventLedger(
    AcceptedEventCertificate* certificates, std::size_t count,
    SelfContactForceEvent* events,
    std::size_t force_capacity) noexcept {
  if ((count && (!certificates || !events)))
    return Failure(SelfContactTransactionStatus::InvalidInput,
        "Accepted-event finalization storage is absent");
  if (count > force_capacity) {
    auto report = Failure(SelfContactTransactionStatus::ResourceLimit,
        "Complete accepted VF+EE event set exceeds force capacity");
    report.candidate = count;
    return report;
  }
  std::sort(certificates, certificates + count,
            [](const AcceptedEventCertificate& a,
               const AcceptedEventCertificate& b) {
              return fixed_triangle_features::Compare(
                  a.event.feature, b.event.feature) < 0;
            });
  for (std::size_t i = 0; i < count; ++i) {
    if (i && fixed_triangle_features::Compare(
                 certificates[i - 1].event.feature,
                 certificates[i].event.feature) >= 0)
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Final accepted-event ledger is not unique", i);
    certificates[i].event.source_order = i;
    events[i] = certificates[i].event;
  }
  return {};
}

void FoldPolicyOutcomes(
    const SelfContactCandidatePolicyOutcome* values,
    std::size_t count,
    SelfContactCandidatePolicySummary* summary) noexcept {
  if (!summary || (count && !values)) return;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& value = values[i];
    HashValue(value.pair.paths[0].source_instance_id,
              &summary->digest);
    HashValue(value.pair.paths[0].parent_eid, &summary->digest);
    HashValue(value.pair.paths[0].level, &summary->digest);
    HashValue(value.pair.paths[0].local_facet, &summary->digest);
    HashValue(value.pair.paths[1].source_instance_id,
              &summary->digest);
    HashValue(value.pair.paths[1].parent_eid, &summary->digest);
    HashValue(value.pair.paths[1].level, &summary->digest);
    HashValue(value.pair.paths[1].local_facet, &summary->digest);
    HashValue(value.disposition, &summary->digest);
    HashValue(value.accepted_event, &summary->digest);
    HashValue(value.source_order, &summary->digest);
    ++summary->outcomes;
    if (value.disposition ==
        SelfContactCandidateDisposition::CertifiedSeparated)
      ++summary->certified_separated;
    else if (value.disposition ==
             SelfContactCandidateDisposition::
                 ExcludedSameRigidGroup)
      ++summary->excluded_same_rigid_group;
    else if (value.disposition ==
             SelfContactCandidateDisposition::
                 ExcludedLocalIntersection)
      ++summary->excluded_local_intersection;
    else if (value.disposition ==
             SelfContactCandidateDisposition::
                 RepresentedByAcceptedVertexFace)
      ++summary->represented_by_accepted_vf;
    else
      ++summary->represented_by_accepted_ee;
  }
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
    bool edge_edge = false;
    const auto accepted = AcceptedFeature(
        crossing, input.accepted_events,
        input.accepted_event_count, &edge_edge);
    if (accepted == SIZE_MAX)
      return Failure(SelfContactTransactionStatus::CandidateRejected,
          crossing.feature.kind == RepresentedFeatureKind::EdgeEdge
              ? "EE crossing lacks its exact accepted EE certificate"
              : "Crossing lacks its full accepted VF event certificate",
          pair);
    outcome.disposition = edge_edge
        ? SelfContactCandidateDisposition::RepresentedByAcceptedEdgeEdge
        : SelfContactCandidateDisposition::RepresentedByAcceptedVertexFace;
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
