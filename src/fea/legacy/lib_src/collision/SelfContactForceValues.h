// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "SelfContactForceTypes.h"
#include "Q4ContactBounds.h"
#include "fixed_triangle_features/Geometry.h"

namespace tlfea::contact {

namespace self_contact_force_identity_detail {

inline int Compare(
    const FixedTriangleFeatureKey& a_feature,
    const std::uint32_t (&a_parent)[2],
    const FixedTriangleFeatureKey& b_feature,
    const std::uint32_t (&b_parent)[2]) noexcept {
  const int feature =
      fixed_triangle_features::Compare(a_feature, b_feature);
  if (feature) return feature;
  for (unsigned side = 0; side < 2; ++side) {
    if (a_parent[side] < b_parent[side]) return -1;
    if (b_parent[side] < a_parent[side]) return 1;
  }
  return 0;
}

}  // namespace self_contact_force_identity_detail

inline SelfContactForceEventIdentity SelfContactForceEventIdentityOf(
    const SelfContactForceEvent& event) noexcept {
  SelfContactForceEventIdentity result;
  result.feature = event.feature;
  result.parent[0] = event.classification.parent[0];
  result.parent[1] = event.classification.parent[1];
  return result;
}

inline int CompareSelfContactForceEventIdentity(
    const SelfContactForceEventIdentity& a,
    const SelfContactForceEventIdentity& b) noexcept {
  return self_contact_force_identity_detail::Compare(
      a.feature, a.parent, b.feature, b.parent);
}

inline bool SameSelfContactForceEventIdentity(
    const SelfContactForceEventIdentity& a,
    const SelfContactForceEventIdentity& b) noexcept {
  return CompareSelfContactForceEventIdentity(a, b) == 0;
}

inline int CompareSelfContactForceEventIdentity(
    const SelfContactForceEvent& a,
    const SelfContactForceEvent& b) noexcept {
  return self_contact_force_identity_detail::Compare(
      a.feature, a.classification.parent,
      b.feature, b.classification.parent);
}

inline bool SameSelfContactForceEventIdentity(
    const SelfContactForceEvent& a,
    const SelfContactForceEvent& b) noexcept {
  return CompareSelfContactForceEventIdentity(a, b) == 0;
}

namespace self_contact_force_identity_detail {

template <class T>
inline void HashValue(T value, std::uint64_t* hash) noexcept {
  const std::uint64_t bits = static_cast<std::uint64_t>(value);
  for (unsigned byte = 0; byte < sizeof(bits); ++byte) {
    *hash ^= static_cast<unsigned char>(bits >> (8 * byte));
    *hash *= 1099511628211ull;
  }
}

inline void Hash(const FacetVertexKey& key,
                 std::uint64_t* hash) noexcept {
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

inline void Hash(const FacetEdgeKey& key,
                 std::uint64_t* hash) noexcept {
  HashValue(key.parent_boundary, hash);
  HashValue(key.parent_eid, hash);
  Hash(key.endpoints[0], hash);
  Hash(key.endpoints[1], hash);
}

inline void Hash(const FixedTriangleKey& key,
                 std::uint64_t* hash) noexcept {
  HashValue(key.source_instance_id, hash);
  HashValue(key.parent_eid, hash);
  HashValue(key.level, hash);
  HashValue(key.local_facet, hash);
}

inline void Hash(const FixedTriangleFeatureKey& key,
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

}  // namespace self_contact_force_identity_detail

inline void HashSelfContactForceEventIdentity(
    const SelfContactForceEventIdentity& identity,
    std::uint64_t* hash) noexcept {
  if (!hash) return;
  self_contact_force_identity_detail::Hash(identity.feature, hash);
  self_contact_force_identity_detail::HashValue(
      identity.parent[0], hash);
  self_contact_force_identity_detail::HashValue(
      identity.parent[1], hash);
}

inline void HashSelfContactForceEventIdentity(
    const SelfContactForceEvent& event,
    std::uint64_t* hash) noexcept {
  if (!hash) return;
  self_contact_force_identity_detail::Hash(event.feature, hash);
  self_contact_force_identity_detail::HashValue(
      event.classification.parent[0], hash);
  self_contact_force_identity_detail::HashValue(
      event.classification.parent[1], hash);
}

TL_SURFACE_HD inline bool PositiveSelfContactArea(
    Q4CertifiedIntegral area) noexcept {
  Q4CertifiedIntegral checked;
  return IsFinite(area.error) && area.error >= 0 &&
      area.value > 0 && area.lower > 0 &&
      q4_bounds::Certify(area.value, {area.lower, area.upper}, &checked) &&
      checked.error <= area.error;
}

// The represented force coefficient is the outward binary64 product of the
// explicit pressure stiffness and the active-use certified directed area.
// A lost positive product and overflow both reject.
TL_SURFACE_HD inline bool RepresentedSelfContactStiffness(
    double stiffness_per_area_n_m3, Q4CertifiedIntegral area,
    double* stiffness_n_m) noexcept {
  if (!stiffness_n_m || !IsFinite(stiffness_per_area_n_m3) ||
      stiffness_per_area_n_m3 <= 0 || !PositiveSelfContactArea(area))
    return false;
  double next = 0;
  if (!mass_detail::UpperProduct(stiffness_per_area_n_m3, area.value, &next) ||
      !IsFinite(next) || next <= 0)
    return false;
  *stiffness_n_m = next;
  return true;
}

// Scratch-only host canonicalization. An empty event batch publishes an exact
// zero summary without requiring event/incidence/node storage. Nonempty events
// sort by immutable feature plus ordered active-parent ownership, then source
// order. Incidence sorts by physical node and canonical event ordinal; one
// event contributes at most once to one physical node even when its two
// endpoint maps share that node.
SelfContactForceReport BuildSelfContactForceIncidence(
    SelfContactForceEvent* events, std::size_t event_count,
    std::uint32_t node_count,
    SelfContactForceIncidence* incidences, std::size_t incidence_capacity,
    SelfContactForceNodeIncidence* nodes, std::size_t node_capacity,
    SelfContactForceIncidenceSummary* summary) noexcept;

}  // namespace tlfea::contact
