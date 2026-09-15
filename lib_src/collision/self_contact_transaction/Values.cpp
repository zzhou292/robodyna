// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../SelfContactForceValues.h"

#include <algorithm>
#include <boost/multiprecision/cpp_int.hpp>
#include <cmath>
#include <cstring>
#include <limits>
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

using ExactBackend = boost::multiprecision::cpp_int_backend<
    16384, 16384, boost::multiprecision::signed_magnitude,
    boost::multiprecision::checked, void>;
using ExactInteger =
    boost::multiprecision::number<ExactBackend,
                                  boost::multiprecision::et_off>;

struct Dyadic {
  ExactInteger numerator = 0;
  int exponent = 0;
};

Dyadic Exact(double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "binary64 representation");
  std::memcpy(&bits, &value, sizeof(bits));
  const bool negative = (bits >> 63) != 0;
  const unsigned encoded_exponent =
      static_cast<unsigned>((bits >> 52) & 0x7ffu);
  const std::uint64_t fraction =
      bits & ((std::uint64_t{1} << 52) - 1);
  Dyadic result;
  if (encoded_exponent == 0) {
    result.numerator = fraction;
    result.exponent = -1074;
  } else {
    result.numerator = (std::uint64_t{1} << 52) | fraction;
    result.exponent =
        static_cast<int>(encoded_exponent) - 1023 - 52;
  }
  if (negative)
    result.numerator = -result.numerator;
  return result;
}

Dyadic Add(Dyadic a, Dyadic b) {
  if (a.numerator == 0) return b;
  if (b.numerator == 0) return a;
  const int exponent = std::min(a.exponent, b.exponent);
  const auto shift = [](ExactInteger* value, unsigned amount) {
    const bool negative = *value < 0;
    if (negative) *value = -*value;
    *value <<= amount;
    if (negative) *value = -*value;
  };
  shift(&a.numerator,
        static_cast<unsigned>(a.exponent - exponent));
  shift(&b.numerator,
        static_cast<unsigned>(b.exponent - exponent));
  return {a.numerator + b.numerator, exponent};
}

Dyadic Negate(Dyadic value) {
  value.numerator = -value.numerator;
  return value;
}

Dyadic Subtract(Dyadic a, Dyadic b) {
  return Add(a, Negate(b));
}

Dyadic Multiply(const Dyadic& a, const Dyadic& b) {
  return {a.numerator * b.numerator, a.exponent + b.exponent};
}

Dyadic Absolute(Dyadic value) {
  if (value.numerator < 0)
    value.numerator = -value.numerator;
  return value;
}

int Compare(const Dyadic& a, const Dyadic& b) {
  const auto difference = Subtract(a, b);
  return difference.numerator < 0
      ? -1
      : (difference.numerator > 0 ? 1 : 0);
}

double Component(Vec3 value, unsigned component) noexcept {
  return component == 0 ? value.x
                        : (component == 1 ? value.y : value.z);
}

double NextDown(double value) noexcept {
  return std::nextafter(
      value, -std::numeric_limits<double>::infinity());
}

double NextUp(double value) noexcept {
  return std::nextafter(
      value, std::numeric_limits<double>::infinity());
}

double ResidualL1Upper(
    Vec3 base, Vec3 prepared, Vec3 reference) noexcept {
  double bound = 0;
  for (unsigned component = 0; component < 3; ++component) {
    const double displacement =
        Component(prepared, component) - Component(base, component);
    if (!std::isfinite(displacement))
      return std::numeric_limits<double>::infinity();
    const double lower =
        NextDown(NextDown(displacement) -
                 Component(reference, component));
    const double upper =
        NextUp(NextUp(displacement) -
               Component(reference, component));
    if (!std::isfinite(lower) || !std::isfinite(upper))
      return std::numeric_limits<double>::infinity();
    const double absolute =
        std::max(std::fabs(lower), std::fabs(upper));
    bound = NextUp(bound + absolute);
  }
  return bound;
}

Dyadic ResidualL1(
    Vec3 base, Vec3 prepared, Vec3 reference) {
  Dyadic bound;
  for (unsigned component = 0; component < 3; ++component) {
    const auto displacement = Subtract(
        Exact(Component(prepared, component)),
        Exact(Component(base, component)));
    bound = Add(
        bound,
        Absolute(Subtract(
            displacement,
            Exact(Component(reference, component)))));
  }
  return bound;
}

Dyadic TriangleResidualL1(
    const CurrentFixedTriangle& base,
    const CurrentFixedTriangle& prepared,
    Vec3 reference) {
  Dyadic bound;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    const auto residual = ResidualL1(
        base.vertices[vertex], prepared.vertices[vertex], reference);
    if (Compare(residual, bound) > 0)
      bound = residual;
  }
  return bound;
}

double TriangleResidualL1Upper(
    const CurrentFixedTriangle& base,
    const CurrentFixedTriangle& prepared,
    Vec3 reference) noexcept {
  double bound = 0;
  for (unsigned vertex = 0; vertex < 3; ++vertex)
    bound = std::max(
        bound, ResidualL1Upper(
                   base.vertices[vertex],
                   prepared.vertices[vertex], reference));
  return bound;
}

Dyadic SquaredDistance(Vec3 first, Vec3 second) {
  Dyadic result;
  for (unsigned component = 0; component < 3; ++component) {
    const auto difference = Subtract(
        Exact(Component(first, component)),
        Exact(Component(second, component)));
    result = Add(result, Multiply(difference, difference));
  }
  return result;
}

struct ExactPoint {
  Dyadic component[3];
};

ExactPoint ExactValue(Vec3 value) {
  ExactPoint result;
  result.component[0] = Exact(value.x);
  result.component[1] = Exact(value.y);
  result.component[2] = Exact(value.z);
  return result;
}

ExactPoint Add(ExactPoint first, ExactPoint second) {
  ExactPoint result;
  for (unsigned component = 0; component < 3; ++component)
    result.component[component] = Add(
        first.component[component], second.component[component]);
  return result;
}

ExactPoint Scale(ExactPoint point, const Dyadic& scale) {
  for (auto& component : point.component)
    component = Multiply(component, scale);
  return point;
}

Dyadic SquaredDistance(
    const ExactPoint& first, const ExactPoint& second) {
  Dyadic result;
  for (unsigned component = 0; component < 3; ++component) {
    const auto difference = Subtract(
        first.component[component],
        second.component[component]);
    result = Add(result, Multiply(difference, difference));
  }
  return result;
}

const CurrentFixedTriangle* FeatureTriangle(
    const FixedTriangleKey& key,
    const CurrentFixedTriangle& first,
    const CurrentFixedTriangle& second) noexcept {
  if (fixed_triangle_features::Compare(key, first.key) == 0)
    return &first;
  if (fixed_triangle_features::Compare(key, second.key) == 0)
    return &second;
  return nullptr;
}

const Vec3* VertexValue(
    const CurrentFixedTriangle& triangle,
    const FacetVertexKey& key) noexcept {
  for (unsigned vertex = 0; vertex < 3; ++vertex)
    if (fixed_triangle_features::Compare(
            triangle.vertex_keys[vertex], key) == 0)
      return triangle.vertices + vertex;
  return nullptr;
}

bool ExactFeatureSquaredDistance(
    const FixedTriangleFeatureCandidate& feature,
    const CurrentFixedTriangle& first,
    const CurrentFixedTriangle& second,
    Dyadic* output) {
  if (!output)
    return false;
  const CurrentFixedTriangle* triangles[2]{
      FeatureTriangle(feature.triangles[0], first, second),
      FeatureTriangle(feature.triangles[1], first, second)};
  if (!triangles[0] || !triangles[1])
    return false;
  ExactPoint points[2];
  if (feature.key.kind ==
      FixedTriangleCandidateKind::VertexFace) {
    const bool first_vertex =
        feature.local_features[0] < 3 &&
        feature.local_features[1] == 3;
    const bool second_vertex =
        feature.local_features[1] < 3 &&
        feature.local_features[0] == 3;
    if (first_vertex == second_vertex ||
        feature.key.vertex_face.target.kind !=
            FixedTriangleStratumKind::Face)
      return false;
    const unsigned source = second_vertex ? 1 : 0;
    const unsigned target = 1 - source;
    points[source] = ExactValue(
        triangles[source]->vertices[
            feature.local_features[source]]);
    Dyadic weight_sum;
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      const double weight = feature.face_weights[vertex];
      if (!std::isfinite(weight) || weight < 0 || weight > 1)
        return false;
      weight_sum = Add(weight_sum, Exact(weight));
      points[target] = Add(
          points[target],
          Scale(ExactValue(
                    triangles[target]->vertices[vertex]),
                Exact(weight)));
    }
    if (Compare(weight_sum, Exact(1)) != 0)
      return false;
  } else if (feature.key.kind ==
             FixedTriangleCandidateKind::EdgeEdge) {
    if (feature.local_features[0] >= 3 ||
        feature.local_features[1] >= 3)
      return false;
    const auto& first_edge =
        triangles[0]->edge_keys[feature.local_features[0]];
    const auto& second_edge =
        triangles[1]->edge_keys[feature.local_features[1]];
    const bool first_key_first =
        fixed_triangle_features::Compare(
            first_edge, second_edge) <= 0;
    for (unsigned side = 0; side < 2; ++side) {
      const auto& edge =
          triangles[side]->edge_keys[
              feature.local_features[side]];
      const auto* begin =
          VertexValue(*triangles[side], edge.endpoints[0]);
      const auto* end =
          VertexValue(*triangles[side], edge.endpoints[1]);
      const unsigned parameter =
          (side == 0) == first_key_first ? 0 : 1;
      const double t = feature.edge_parameters[parameter];
      if (!begin || !end || !std::isfinite(t) ||
          t < 0 || t > 1)
        return false;
      points[side] = Add(
          Scale(ExactValue(*begin),
                Subtract(Exact(1), Exact(t))),
          Scale(ExactValue(*end), Exact(t)));
    }
  } else {
    return false;
  }
  *output = SquaredDistance(points[0], points[1]);
  return true;
}

unsigned FeatureTaskSlot(
    const FixedTriangleFeatureCandidate& feature) noexcept {
  if (feature.key.kind ==
      FixedTriangleCandidateKind::VertexFace) {
    const bool first_is_vertex =
        feature.local_features[0] < 3 &&
        feature.local_features[1] == 3;
    const bool second_is_vertex =
        feature.local_features[1] < 3 &&
        feature.local_features[0] == 3;
    if (first_is_vertex == second_is_vertex)
      return 15;
    const unsigned side = second_is_vertex ? 1 : 0;
    return FixedTriangleVertexFaceTaskSlot(
        side, feature.local_features[side]);
  }
  if (feature.key.kind ==
          FixedTriangleCandidateKind::EdgeEdge &&
      feature.local_features[0] < 3 &&
      feature.local_features[1] < 3)
    return FixedTriangleEdgeEdgeTaskSlot(
        feature.local_features[0], feature.local_features[1]);
  return 15;
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

void HashEventIdentity(const SelfContactForceEvent& event,
                       std::uint64_t* hash) noexcept {
  HashSelfContactForceEventIdentity(event, hash);
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
  const bool boundary_vertex_face =
      a.kind == AcceptedEventCertificateKind::VertexFace &&
      b.kind == AcceptedEventCertificateKind::VertexFace &&
      a.event.feature.kind ==
          FixedTriangleCandidateKind::VertexFace &&
      b.event.feature.kind ==
          FixedTriangleCandidateKind::VertexFace &&
      a.event.feature.vertex_face.target.kind !=
          FixedTriangleStratumKind::Face &&
      b.event.feature.vertex_face.target.kind !=
          FixedTriangleStratumKind::Face;
  auto first_classification = a.event.classification;
  auto second_classification = b.event.classification;
  if (boundary_vertex_face)
    second_classification.feature[1] =
        first_classification.feature[1];
  if (a.kind != b.kind ||
      !self_contact_transaction::Same(
          a.event.feature, b.event.feature) ||
      a.event.vertex_use != b.event.vertex_use ||
      (!boundary_vertex_face &&
       a.event.facet_use != b.event.facet_use) ||
      a.event.edge_use[0] != b.event.edge_use[0] ||
      a.event.edge_use[1] != b.event.edge_use[1] ||
      !Same(a.event.endpoints[0], b.event.endpoints[0]) ||
      !Same(a.event.endpoints[1], b.event.endpoints[1]) ||
      !Same(first_classification, second_classification))
    return false;
  if (a.kind == AcceptedEventCertificateKind::EdgeEdge ||
      boundary_vertex_face)
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

bool SameParentOwner(
    const FixedTriangleKey& triangle,
    const RepresentedTrianglePathKey& path) noexcept {
  return triangle.source_instance_id == path.source_instance_id &&
      triangle.parent_eid == path.parent_eid;
}

bool SameParentPair(
    const FixedTriangleFeatureCandidate& discovery,
    const RepresentedIntervalPairKey& crossing) noexcept {
  return
      (SameParentOwner(discovery.triangles[0], crossing.paths[0]) &&
       SameParentOwner(discovery.triangles[1], crossing.paths[1])) ||
      (SameParentOwner(discovery.triangles[0], crossing.paths[1]) &&
       SameParentOwner(discovery.triangles[1], crossing.paths[0]));
}

bool SameVertexFaceOwners(
    const FixedTriangleFeatureCandidate& discovery,
    const FixedTriangleFeatureKey& key,
    const RepresentedIntervalPairKey& crossing) noexcept {
  if (key.vertex_face.target.kind !=
          FixedTriangleStratumKind::Face ||
      ((discovery.local_features[0] == 3) ==
       (discovery.local_features[1] == 3)))
    return false;
  const unsigned target =
      discovery.local_features[0] == 3 ? 0 : 1;
  const auto& target_key = key.vertex_face.target.face;
  if (!self_contact_transaction::Same(
          discovery.triangles[target], target_key))
    return false;
  for (unsigned path = 0; path < 2; ++path)
    if (self_contact_transaction::Compare(
            crossing.paths[path],
            {target_key.source_instance_id, target_key.parent_eid,
             target_key.level, target_key.local_facet}) == 0)
      return SameParentOwner(
          discovery.triangles[1 - target],
          crossing.paths[1 - path]);
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
  for (std::size_t candidate = lower;
       candidate < event_count &&
       self_contact_transaction::Same(
           events[candidate].event.feature, key);
       ++candidate) {
    const auto& certificate = events[candidate];
    const auto& accepted = certificate.event;
    if (!self_contact_transaction::Same(
            certificate.discovery.key, key) ||
        accepted.source_order != candidate ||
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
      continue;

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
      if (!SameParentPair(certificate.discovery, crossing.key) ||
          certificate.kind != AcceptedEventCertificateKind::EdgeEdge ||
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
        continue;
      *edge_edge = true;
      return candidate;
    }

    double face_sum = 0;
    bool face_weights_valid = true;
    for (double weight : certificate.discovery.face_weights) {
      face_weights_valid = face_weights_valid &&
          IsFinite(weight) && weight >= 0 && weight <= 1;
      face_sum += weight;
    }
    if (!SameVertexFaceOwners(
            certificate.discovery, key, crossing.key) ||
        certificate.kind != AcceptedEventCertificateKind::VertexFace ||
        accepted.feature.kind != FixedTriangleCandidateKind::VertexFace ||
        accepted.edge_use[0] != UINT32_MAX ||
        accepted.edge_use[1] != UINT32_MAX ||
        !face_weights_valid || std::fabs(face_sum - 1) > 1e-12 ||
        accepted.classification.kind !=
            SelfContactPairKind::VertexFace ||
        accepted.classification.status !=
            SelfContactPairStatus::AdmittedVertexFace ||
        certificate.vertex_facet == UINT32_MAX ||
        certificate.target_facet == UINT32_MAX ||
        certificate.edge_facet[0] != UINT32_MAX ||
        certificate.edge_facet[1] != UINT32_MAX)
      continue;
    *edge_edge = false;
    return candidate;
  }
  return SIZE_MAX;
}

std::size_t PersistentAcceptedFeature(
    const RepresentedIntervalResult& crossing,
    const AcceptedEventCertificate* events,
    std::size_t event_count) noexcept {
  FixedTriangleFeatureKey key;
  if (crossing.feature.kind ==
      RepresentedFeatureKind::VertexFace) {
    key.vertex_face.vertex = crossing.feature.vertex;
    key.vertex_face.target.SetFace({
        crossing.feature.face.source_instance_id,
        crossing.feature.face.parent_eid,
        crossing.feature.face.level,
        crossing.feature.face.local_facet});
  } else if (crossing.feature.kind ==
             RepresentedFeatureKind::EdgeEdge) {
    key.SetEdgeEdge();
    key.edge_edge.edges[0] = crossing.feature.edges[0];
    key.edge_edge.edges[1] = crossing.feature.edges[1];
  } else {
    return SIZE_MAX;
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
  const auto same_path = [](
      const FixedTriangleKey& triangle,
      const RepresentedTrianglePathKey& path) {
    return self_contact_transaction::Compare(
        path, {triangle.source_instance_id,
               triangle.parent_eid, triangle.level,
               triangle.local_facet}) == 0;
  };
  for (std::size_t candidate = lower;
       candidate < event_count &&
       self_contact_transaction::Same(
           events[candidate].event.feature, key);
       ++candidate) {
    const auto& certificate = events[candidate];
    const auto& accepted = certificate.event;
    bool exact_pair =
        (same_path(certificate.discovery.triangles[0],
                   crossing.key.paths[0]) &&
         same_path(certificate.discovery.triangles[1],
                   crossing.key.paths[1])) ||
        (same_path(certificate.discovery.triangles[0],
                   crossing.key.paths[1]) &&
         same_path(certificate.discovery.triangles[1],
                   crossing.key.paths[0]));
    if (!exact_pair &&
        crossing.feature.kind ==
            RepresentedFeatureKind::EdgeEdge) {
      auto accepted_paths = PairKey(
          certificate.discovery.triangles[0],
          certificate.discovery.triangles[1]);
      const int first_owner =
          self_contact_transaction::Compare(
              accepted_paths.paths[0],
              crossing.key.paths[0]);
      const int second_owner =
          self_contact_transaction::Compare(
              accepted_paths.paths[1],
              crossing.key.paths[1]);
      exact_pair =
          first_owner <= 0 && second_owner <= 0 &&
          (first_owner < 0 || second_owner < 0);
    }
    const bool edge_edge =
        crossing.feature.kind ==
            RepresentedFeatureKind::EdgeEdge &&
        certificate.kind ==
            AcceptedEventCertificateKind::EdgeEdge &&
        accepted.classification.kind ==
            SelfContactPairKind::EdgeEdge &&
        accepted.classification.status ==
            SelfContactPairStatus::AdmittedEdgeEdge;
    const bool vertex_face =
        crossing.feature.kind ==
            RepresentedFeatureKind::VertexFace &&
        certificate.kind ==
            AcceptedEventCertificateKind::VertexFace &&
        accepted.classification.kind ==
            SelfContactPairKind::VertexFace &&
        accepted.classification.status ==
            SelfContactPairStatus::AdmittedVertexFace;
    const double gap =
        (certificate.discovery.distance_m -
         accepted.classification.reference_half_thickness_m[0]) -
        accepted.classification.reference_half_thickness_m[1];
    if (exact_pair && (edge_edge || vertex_face) &&
        self_contact_transaction::Same(
            certificate.discovery.key, key) &&
        accepted.source_order == candidate &&
        !accepted.classification.excluded &&
        !accepted.classification.local_incidence &&
        accepted.classification.active[0] &&
        accepted.classification.active[1] &&
        IsFinite(certificate.discovery.distance_m) &&
        certificate.discovery.distance_m >= 0 &&
        IsFinite(gap) && gap <= 0)
      return candidate;
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

LinearResidualSeparationResult CertifyLinearResidualSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    double first_half_thickness_m,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    double second_half_thickness_m,
    FixedTriangleFeatureView prepared_features,
    FixedTriangleIntersectionView prepared_intersections) noexcept {
  LinearResidualSeparationResult result;
  if (!Same(first_base.key, first_prepared.key) ||
      !Same(second_base.key, second_prepared.key) ||
      !prepared_features.complete ||
      !prepared_intersections.complete ||
      (prepared_features.count && !prepared_features.data) ||
      (prepared_intersections.count &&
       !prepared_intersections.data) ||
      !std::isfinite(first_half_thickness_m) ||
      !(first_half_thickness_m > 0) ||
      !std::isfinite(second_half_thickness_m) ||
      !(second_half_thickness_m > 0))
    return result;

  result.reference_translation = {
      first_prepared.vertices[0].x - first_base.vertices[0].x,
      first_prepared.vertices[0].y - first_base.vertices[0].y,
      first_prepared.vertices[0].z - first_base.vertices[0].z};
  if (!IsFinite(result.reference_translation))
    return result;

  const auto key = PairKey(first_prepared.key, second_prepared.key);
  for (std::size_t i = 0;
       i < prepared_intersections.count; ++i) {
    const auto& intersection = prepared_intersections.data[i];
    if (Compare(PairKey(
                    intersection.triangles[0],
                    intersection.triangles[1]),
                key) == 0) {
      result.status =
          LinearResidualSeparationStatus::PotentialContact;
      return result;
    }
  }

  try {
    const auto first_residual = TriangleResidualL1(
        first_base, first_prepared,
        result.reference_translation);
    const auto second_residual = TriangleResidualL1(
        second_base, second_prepared,
        result.reference_translation);
    result.exact_common_translation =
        first_residual.numerator == 0 &&
        second_residual.numerator == 0;
    result.first_residual_upper_m =
        TriangleResidualL1Upper(
            first_base, first_prepared,
            result.reference_translation);
    result.second_residual_upper_m =
        TriangleResidualL1Upper(
            second_base, second_prepared,
            result.reference_translation);
    if (!std::isfinite(result.first_residual_upper_m) ||
        !std::isfinite(result.second_residual_upper_m))
      return result;

    auto exact_margin = Add(
        Add(Exact(first_half_thickness_m),
            Exact(second_half_thickness_m)),
        Add(first_residual, second_residual));
    std::uint16_t observed = 0;
    double minimum_lower =
        std::numeric_limits<double>::infinity();
    for (std::size_t i = 0;
         i < prepared_features.count; ++i) {
      const auto& feature = prepared_features.data[i];
      if (Compare(PairKey(
                      feature.triangles[0],
                      feature.triangles[1]),
                  key) != 0)
        continue;
      const unsigned slot = FeatureTaskSlot(feature);
      const auto bit = FixedTriangleFeatureTaskBit(slot);
      if (!bit || (observed & bit) ||
          !std::isfinite(feature.distance_m) ||
          feature.distance_m < 0 ||
          !std::isfinite(feature.representation_error_m) ||
          feature.representation_error_m < 0) {
        result.status =
            LinearResidualSeparationStatus::
                IncompleteFeatureRoster;
        return result;
      }
      observed = static_cast<std::uint16_t>(observed | bit);
      const auto margin = Add(
          exact_margin,
          Exact(feature.representation_error_m));
      const auto squared_distance =
          SquaredDistance(feature.points[0], feature.points[1]);
      if (Compare(squared_distance,
                  Multiply(margin, margin)) <= 0) {
        result.status =
            LinearResidualSeparationStatus::PotentialContact;
        return result;
      }
      minimum_lower = std::min(
          minimum_lower,
          NextDown(feature.distance_m -
                   feature.representation_error_m));
    }
    if (observed != FixedTriangleFeatureTaskBits) {
      result.status =
          LinearResidualSeparationStatus::
              IncompleteFeatureRoster;
      return result;
    }
    result.prepared_distance_lower_m = minimum_lower;
    const double residual_upper = NextUp(
        result.first_residual_upper_m +
        result.second_residual_upper_m);
    result.strict_gap_lower_m = NextDown(
        NextDown(
            NextDown(minimum_lower -
                     first_half_thickness_m) -
            second_half_thickness_m) -
        residual_upper);
    result.status =
        LinearResidualSeparationStatus::CertifiedSeparated;
    return result;
  } catch (...) {
    return result;
  }
}

PersistentLinearContactResult CertifyPersistentLinearContact(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    double first_half_thickness_m,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    double second_half_thickness_m,
    FixedTriangleFeatureView prepared_features,
    const AcceptedEventCertificate* accepted_certificates,
    std::size_t accepted_certificate_count) noexcept {
  PersistentLinearContactResult result;
  if (!Same(first_base.key, first_prepared.key) ||
      !Same(second_base.key, second_prepared.key) ||
      !prepared_features.complete ||
      (prepared_features.count && !prepared_features.data) ||
      (accepted_certificate_count && !accepted_certificates) ||
      !std::isfinite(first_half_thickness_m) ||
      !(first_half_thickness_m > 0) ||
      !std::isfinite(second_half_thickness_m) ||
      !(second_half_thickness_m > 0))
    return result;
  result.reference_translation = {
      first_prepared.vertices[0].x - first_base.vertices[0].x,
      first_prepared.vertices[0].y - first_base.vertices[0].y,
      first_prepared.vertices[0].z - first_base.vertices[0].z};
  if (!IsFinite(result.reference_translation))
    return result;

  try {
    const auto first_residual = TriangleResidualL1(
        first_base, first_prepared,
        result.reference_translation);
    const auto second_residual = TriangleResidualL1(
        second_base, second_prepared,
        result.reference_translation);
    result.exact_common_translation =
        first_residual.numerator == 0 &&
        second_residual.numerator == 0;
    result.first_residual_upper_m =
        TriangleResidualL1Upper(
            first_base, first_prepared,
            result.reference_translation);
    result.second_residual_upper_m =
        TriangleResidualL1Upper(
            second_base, second_prepared,
            result.reference_translation);
    if (!std::isfinite(result.first_residual_upper_m) ||
        !std::isfinite(result.second_residual_upper_m))
      return result;
    const auto thickness = Add(
        Exact(first_half_thickness_m),
        Exact(second_half_thickness_m));
    const auto residual =
        Add(first_residual, second_residual);
    const auto pair =
        PairKey(first_prepared.key, second_prepared.key);
    bool saw_exact_pair = false;
    for (std::size_t feature_index = 0;
         feature_index < prepared_features.count;
         ++feature_index) {
      const auto& feature =
          prepared_features.data[feature_index];
      if (Compare(PairKey(feature.triangles[0],
                          feature.triangles[1]),
                  pair) != 0)
        continue;
      saw_exact_pair = true;
      if (!std::isfinite(feature.distance_m) ||
          feature.distance_m < 0 ||
          !std::isfinite(feature.representation_error_m) ||
          feature.representation_error_m < 0)
        return result;
      if (feature.key.kind ==
              FixedTriangleCandidateKind::VertexFace &&
          feature.key.vertex_face.target.kind !=
              FixedTriangleStratumKind::Face)
        continue;
      Dyadic squared_distance;
      if (!ExactFeatureSquaredDistance(
              feature, first_prepared, second_prepared,
              &squared_distance))
        continue;
      const auto available = Subtract(
          Subtract(thickness, residual),
          Exact(feature.representation_error_m));
      if (available.numerator <= 0 ||
          Compare(squared_distance,
                  Multiply(available, available)) >= 0)
        continue;
      ++result.bounded_feature_count;

      std::size_t lower = 0;
      std::size_t upper = accepted_certificate_count;
      while (lower < upper) {
        const auto middle = lower + (upper - lower) / 2;
        if (fixed_triangle_features::Compare(
                accepted_certificates[middle].event.feature,
                feature.key) < 0)
          lower = middle + 1;
        else
          upper = middle;
      }
      for (std::size_t i = lower;
           i < accepted_certificate_count &&
           Same(accepted_certificates[i].event.feature,
                feature.key);
           ++i)
        if (Same(accepted_certificates[i].discovery.key,
                 feature.key) &&
            ExactFacetPair(
                accepted_certificates[i].discovery,
                feature))
          ++result.exact_accepted_candidate_count;

      RepresentedIntervalResult publication;
      publication.key = pair;
      if (feature.key.kind ==
          FixedTriangleCandidateKind::VertexFace) {
        publication.feature.kind =
            RepresentedFeatureKind::VertexFace;
        publication.feature.vertex =
            feature.key.vertex_face.vertex;
        const auto& face =
            feature.key.vertex_face.target.face;
        publication.feature.face = {
            face.source_instance_id, face.parent_eid,
            face.level, face.local_facet};
      } else {
        publication.feature.kind =
            RepresentedFeatureKind::EdgeEdge;
        publication.feature.edges[0] =
            feature.key.edge_edge.edges[0];
        publication.feature.edges[1] =
            feature.key.edge_edge.edges[1];
      }
      const std::size_t accepted = PersistentAcceptedFeature(
          publication, accepted_certificates,
          accepted_certificate_count);
      Dyadic accepted_squared_distance;
      if (accepted == SIZE_MAX)
        continue;
      ++result.full_accepted_candidate_count;
      if (ExactFacetPair(
              accepted_certificates[accepted].discovery,
              feature) &&
          !ExactFeatureSquaredDistance(
              accepted_certificates[accepted].discovery,
              first_base, second_base,
              &accepted_squared_distance))
        continue;
      if (result.status ==
              PersistentLinearContactStatus::CertifiedContact &&
          fixed_triangle_features::Compare(
              accepted_certificates[
                  result.accepted_certificate].discovery.key,
              feature.key) <= 0)
        continue;

      result.feature = publication.feature;
      result.accepted_certificate = accepted;
      result.prepared_distance_upper_m = NextUp(
          feature.distance_m +
          feature.representation_error_m);
      const double residual_upper = NextUp(
          result.first_residual_upper_m +
          result.second_residual_upper_m);
      result.strict_thickness_margin_lower_m = NextDown(
          NextDown(
              first_half_thickness_m +
              second_half_thickness_m) -
          NextUp(result.prepared_distance_upper_m +
                 residual_upper));
      result.status =
          PersistentLinearContactStatus::CertifiedContact;
    }
    if (result.status ==
        PersistentLinearContactStatus::CertifiedContact)
      return result;
    result.status = saw_exact_pair
        ? PersistentLinearContactStatus::PotentialChange
        : PersistentLinearContactStatus::
              IncompleteFeatureRoster;
    return result;
  } catch (...) {
    return result;
  }
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

SelfContactTransactionReport MergeAcceptedEventIdentityChunk(
    const SelfContactForceEventIdentity* input,
    std::size_t input_count,
    SelfContactForceEventIdentity* census,
    std::size_t census_capacity,
    std::uint32_t* hash_slots, std::size_t hash_capacity,
    std::size_t* census_count) noexcept {
  if ((input_count && !input) || !census || !census_capacity ||
      !hash_slots || !hash_capacity || !census_count ||
      *census_count > census_capacity ||
      census_capacity > UINT32_MAX ||
      hash_capacity < census_capacity)
    return Failure(SelfContactTransactionStatus::InvalidInput,
                   "Accepted-event identity census is incomplete");
  for (std::size_t i = 0; i < input_count; ++i) {
    std::uint64_t hash = 1469598103934665603ull;
    HashSelfContactForceEventIdentity(input[i], &hash);
    std::size_t slot = hash % hash_capacity;
    bool inserted = false;
    // Hashes select probe order only. Every occupied slot is resolved by the
    // complete canonical feature and both ordered owner ordinals.
    for (std::size_t probe = 0; probe < hash_capacity; ++probe) {
      const auto value = hash_slots[slot];
      if (value == UINT32_MAX) {
        if (*census_count == census_capacity) {
          auto report = Failure(
              SelfContactTransactionStatus::ResourceLimit,
              "Accepted-event identity census exceeds its hard cap");
          report.candidate = census_capacity + 1;
          report.count_kind =
              SelfContactTransactionCountKind::
                  AcceptedEventsLowerBound;
          return report;
        }
        census[*census_count] = input[i];
        hash_slots[slot] =
            static_cast<std::uint32_t>((*census_count)++);
        inserted = true;
        break;
      }
      if (value >= *census_count)
        return Failure(SelfContactTransactionStatus::IdentityMismatch,
            "Accepted-event identity hash index is corrupt", slot);
      if (SameSelfContactForceEventIdentity(
              census[value], input[i])) {
        inserted = true;
        break;
      }
      slot = slot + 1 == hash_capacity ? 0 : slot + 1;
    }
    if (!inserted) {
      auto report = Failure(
          SelfContactTransactionStatus::ResourceLimit,
          "Accepted-event identity hash census has no free slot");
      report.candidate = *census_count + 1;
      report.count_kind =
          SelfContactTransactionCountKind::
              AcceptedEventsLowerBound;
      return report;
    }
  }
  return {};
}

SelfContactTransactionReport
CanonicalizeAcceptedEventIdentityCensus(
    SelfContactForceEventIdentity* census,
    std::size_t count) noexcept {
  if (count && !census)
    return Failure(SelfContactTransactionStatus::InvalidInput,
        "Accepted-event identity census is absent");
  if (count > 1)
    std::sort(census, census + count,
              [](const SelfContactForceEventIdentity& a,
                 const SelfContactForceEventIdentity& b) {
                return CompareSelfContactForceEventIdentity(a, b) < 0;
              });
  for (std::size_t i = 1; i < count; ++i)
    if (CompareSelfContactForceEventIdentity(
            census[i - 1], census[i]) >= 0)
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Canonical accepted-event identity census is not unique", i);
  return {};
}

SelfContactTransactionReport VerifyAcceptedEventIdentityChunk(
    const AcceptedEventCertificate* input, std::size_t input_count,
    const SelfContactForceEventIdentity* census,
    std::size_t census_count) noexcept {
  if ((input_count && !input) || (census_count && !census))
    return Failure(SelfContactTransactionStatus::InvalidInput,
        "Accepted-event identity verification storage is absent");
  for (std::size_t i = 0; i < input_count; ++i) {
    const auto identity =
        SelfContactForceEventIdentityOf(input[i].event);
    std::size_t lower = 0, upper = census_count;
    while (lower < upper) {
      const auto middle = lower + (upper - lower) / 2;
      if (CompareSelfContactForceEventIdentity(
              census[middle], identity) < 0)
        lower = middle + 1;
      else
        upper = middle;
    }
    if (lower == census_count ||
        !SameSelfContactForceEventIdentity(
            census[lower], identity))
      return Failure(SelfContactTransactionStatus::IdentityMismatch,
          "Verification pass produced an uncensused event identity", i);
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
    HashEventIdentity(input[i].event, &hash);
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
      if (SameSelfContactForceEventIdentity(
              ledger[value].event, input[i].event)) {
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
    report.count_kind =
        SelfContactTransactionCountKind::ExactAcceptedEvents;
    return report;
  }
  std::sort(certificates, certificates + count,
            [](const AcceptedEventCertificate& a,
               const AcceptedEventCertificate& b) {
              return CompareSelfContactForceEventIdentity(
                  a.event, b.event) < 0;
            });
  for (std::size_t i = 0; i < count; ++i) {
    if (i && CompareSelfContactForceEventIdentity(
                 certificates[i - 1].event,
                 certificates[i].event) >= 0)
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
            RepresentedIntervalClassification::Unresolved &&
        !(input.crossings.data[pair].reason ==
              RepresentedIntervalReason::UnsupportedMotion &&
          LocallyExcluded(
              input.intersections, input.canonical_pairs[pair])))
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
    std::size_t accepted = SIZE_MAX;
    if (crossing.geometry ==
        RepresentedIntersectionGeometry::
            PersistentPhysicalContact) {
      accepted = PersistentAcceptedFeature(
          crossing, input.accepted_events,
          input.accepted_event_count);
      edge_edge =
          crossing.feature.kind ==
          RepresentedFeatureKind::EdgeEdge;
    } else {
      accepted = AcceptedFeature(
          crossing, input.accepted_events,
          input.accepted_event_count, &edge_edge);
    }
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
