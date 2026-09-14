// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../fixed_triangle_features/Geometry.h"

namespace tlfea::contact::self_contact_force {
namespace {

using S = SelfContactForceStatus;

SelfContactForceReport Invalid(const SelfContactForceEvent& event,
                               std::size_t index,
                               const char* message,
                               SelfContactForceStatus status =
                                   S::IdentityMismatch) noexcept {
  return {status, index, event.source_order, UINT32_MAX,
          SurfacePenaltyStatus::InvalidInput, tl::fea::NodalStatus::Ok,
          message};
}

bool SameBits(double a, double b) noexcept {
  return tl::fea::shell_startup_detail::SameBits(a, b);
}

bool MapMatchesParent(const SelfContactParentUse& parent,
                      const WeightedSurfacePoint& point) noexcept {
  if (point.count != parent.arity) return false;
  for (unsigned i = 0; i < parent.arity; ++i)
    if (point.nodes[i] != parent.nodes[i]) return false;
  return true;
}

bool TargetBelongsToFacet(const SelfContactActiveUseBinding& binding,
                          const SelfContactPairClassification& pair,
                          const FixedTriangleStratumKey& target) noexcept {
  const auto& use = binding.facet_uses()[pair.feature[1]];
  const auto& parent = binding.parents()[use.parent];
  FixedContactFacet facet;
  if (binding.facets()->Describe(parent.surface_parent, use.local_facet,
                                 &facet).status !=
      FixedContactFacetStatus::Ok)
    return false;
  if (target.kind == FixedTriangleStratumKind::Vertex) {
    for (const auto& key : facet.vertex_keys)
      if (!fixed_triangle_features::Compare(target.vertex, key)) return true;
    return false;
  }
  if (target.kind == FixedTriangleStratumKind::Edge) {
    for (const auto& key : facet.edge_keys)
      if (!fixed_triangle_features::Compare(target.edge, key)) return true;
    return false;
  }
  const FixedTriangleKey expected{
      binding.facets()->surface()->physical()->domain()->source_instance_id(),
      parent.source.source_parent_id, facet.level, facet.local_facet};
  return target.kind == FixedTriangleStratumKind::Face &&
      !fixed_triangle_features::Compare(target.face, expected);
}

bool SameClassification(
    const SelfContactPairClassification& a,
    const SelfContactPairClassification& b) noexcept {
  if (a.binding_identity != b.binding_identity ||
      a.activity_base_identity != b.activity_base_identity ||
      a.activity_current_identity != b.activity_current_identity ||
      a.activity_parent_count != b.activity_parent_count ||
      a.kind != b.kind || a.edge_edge_case != b.edge_edge_case ||
      a.status != b.status || a.tied != b.tied ||
      a.local_incidence != b.local_incidence ||
      a.excluded != b.excluded ||
      !SameSupport(a.endpoint_support[0], b.endpoint_support[0]) ||
      !SameSupport(a.endpoint_support[1], b.endpoint_support[1]))
    return false;
  for (unsigned endpoint = 0; endpoint < 2; ++endpoint)
    if (a.parent[endpoint] != b.parent[endpoint] ||
        a.feature[endpoint] != b.feature[endpoint] ||
        a.active[endpoint] != b.active[endpoint] ||
        !SameBits(a.reference_half_thickness_m[endpoint],
                  b.reference_half_thickness_m[endpoint]))
      return false;
  return SameCertificate(a.candidate_directed_area_m2,
                         b.candidate_directed_area_m2) &&
      SameCertificate(a.admitted_force_area_m2,
                      b.admitted_force_area_m2);
}

}  // namespace

bool SamePoint(const WeightedSurfacePoint& a,
               const WeightedSurfacePoint& b) noexcept {
  if (a.count != b.count) return false;
  for (unsigned i = 0; i < 4; ++i)
    if (a.nodes[i] != b.nodes[i] ||
        !tl::fea::shell_startup_detail::SameBits(
            a.weights[i], b.weights[i]))
      return false;
  return true;
}

bool SameSupport(const SelfContactSupportClassification& a,
                 const SelfContactSupportClassification& b) noexcept {
  return a.status == b.status &&
      a.complete_rigid_group == b.complete_rigid_group &&
      a.nonzero_slots == b.nonzero_slots &&
      a.rigid_slots == b.rigid_slots &&
      a.cin_master_slots == b.cin_master_slots;
}

bool SameCertificate(Q4CertifiedIntegral a,
                     Q4CertifiedIntegral b) noexcept {
  return SameBits(a.value, b.value) && SameBits(a.lower, b.lower) &&
      SameBits(a.upper, b.upper) && SameBits(a.error, b.error);
}

SelfContactForceReport ValidateEvent(
    const SelfContactActiveUseBinding& binding,
    const SelfContactForceEvent& event,
    SelfContactActivityView activity,
    std::size_t canonical_event) noexcept {
  const auto& pair = event.classification;
  const auto parents = binding.parents();
  const auto facets = binding.facet_uses();
  if (event.feature.kind == FixedTriangleCandidateKind::EdgeEdge) {
    if (event.vertex_use != UINT32_MAX ||
        event.facet_use != UINT32_MAX ||
        event.edge_use[0] >= binding.edge_uses().size() ||
        event.edge_use[1] >= binding.edge_uses().size() ||
        fixed_triangle_features::Compare(
            event.feature.edge_edge.edges[0],
            event.feature.edge_edge.edges[1]) >= 0)
      return Invalid(event, canonical_event,
                     "EE event ordinals or canonical edge order are invalid");

    SelfContactPairClassification regenerated;
    const auto classified = binding.ClassifyEdgeEdge(
        event.edge_use[0], event.endpoints[0],
        event.edge_use[1], event.endpoints[1],
        pair.edge_edge_case, activity, &regenerated);
    if (classified.status != SelfContactActiveUseStatus::Ok)
      return Invalid(event, canonical_event,
                     "EE event cannot be regenerated from supplied activity",
                     S::StaleAttempt);
    if (!SameClassification(regenerated, pair))
      return Invalid(event, canonical_event,
                     "Caller EE classification differs from exact regeneration");
    if (!binding.Authenticates(regenerated) ||
        regenerated.kind != SelfContactPairKind::EdgeEdge ||
        regenerated.status != SelfContactPairStatus::AdmittedEdgeEdge ||
        regenerated.excluded || regenerated.local_incidence ||
        !regenerated.active[0] || !regenerated.active[1] ||
        regenerated.parent[0] == regenerated.parent[1] ||
        (regenerated.edge_edge_case !=
             SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum &&
         regenerated.edge_edge_case !=
             SelfContactEdgeEdgeCase::BoundaryVertexEdgeMinimum &&
         regenerated.edge_edge_case !=
             SelfContactEdgeEdgeCase::ZeroDistance))
      return Invalid(event, canonical_event,
                     "Regenerated event is not an admitted symmetric EE pair");

    const auto edge_uses = binding.edge_uses();
    const auto edge_features = binding.edges();
    for (unsigned side = 0; side < 2; ++side) {
      if (pair.parent[side] >= parents.size() ||
          pair.feature[side] >= edge_features.size())
        return Invalid(event, canonical_event,
                       "EE parent or feature ordinal is out of range");
      const auto& parent = parents[pair.parent[side]];
      const auto& use = edge_uses[event.edge_use[side]];
      const auto& feature = edge_features[pair.feature[side]];
      if (!MapMatchesParent(parent, event.endpoints[side]) ||
          !SameBits(pair.reference_half_thickness_m[side],
                    parent.reference_half_thickness_m) ||
          use.parent != pair.parent[side] ||
          use.feature != pair.feature[side] ||
          event.edge_use[side] < feature.use_offset ||
          event.edge_use[side] >=
              std::size_t(feature.use_offset) + feature.use_count ||
          fixed_triangle_features::Compare(
              use.key, event.feature.edge_edge.edges[side]) != 0)
        return Invalid(event, canonical_event,
                       "EE map, provenance or retained edge use differs");
    }
    if (!SameCertificate(pair.candidate_directed_area_m2,
                         pair.admitted_force_area_m2) ||
        !PositiveSelfContactArea(pair.admitted_force_area_m2))
      return Invalid(event, canonical_event,
                     "EE symmetric directed edge-point area is unauthenticated");
    return {};
  }

  if (event.feature.kind != FixedTriangleCandidateKind::VertexFace ||
      event.vertex_use >= binding.vertex_uses().size() ||
      event.facet_use >= facets.size() ||
      event.edge_use[0] != UINT32_MAX ||
      event.edge_use[1] != UINT32_MAX)
    return Invalid(event, canonical_event,
                   "Event kind or exact active-use ordinals are invalid");

  SelfContactPairClassification regenerated;
  const auto classified = binding.ClassifyVertexFace(
      event.vertex_use, event.facet_use, event.endpoints[1],
      activity, &regenerated);
  if (classified.status != SelfContactActiveUseStatus::Ok)
    return Invalid(event, canonical_event,
                   "Event cannot be regenerated from the supplied activity",
                   S::StaleAttempt);
  if (!SameClassification(regenerated, pair))
    return Invalid(event, canonical_event,
                   "Caller classification differs from exact regeneration");
  if (!binding.Authenticates(regenerated) ||
      regenerated.status != SelfContactPairStatus::AdmittedVertexFace ||
      regenerated.excluded || regenerated.local_incidence ||
      !regenerated.active[0] || !regenerated.active[1] ||
      regenerated.parent[0] == regenerated.parent[1] ||
      regenerated.endpoint_support[0].status ==
          SelfContactSupportStatus::UnsupportedCinSecondary ||
      regenerated.endpoint_support[1].status ==
          SelfContactSupportStatus::UnsupportedCinSecondary)
    return Invalid(event, canonical_event,
                   "Regenerated event is not an admitted directed VF pair");

  const auto& first_parent = parents[pair.parent[0]];
  const auto& second_parent = parents[pair.parent[1]];
  if (!MapMatchesParent(first_parent, event.endpoints[0]) ||
      !MapMatchesParent(second_parent, event.endpoints[1]) ||
      !SameBits(pair.reference_half_thickness_m[0],
                first_parent.reference_half_thickness_m) ||
      !SameBits(pair.reference_half_thickness_m[1],
                second_parent.reference_half_thickness_m))
    return Invalid(event, canonical_event,
                   "Event maps or thickness differ from retained parents");

  const auto& vertex = binding.vertex_uses()[event.vertex_use];
  if (vertex.parent != pair.parent[0] ||
      vertex.feature != pair.feature[0] ||
      facets[event.facet_use].parent != pair.parent[1] ||
      pair.feature[1] != event.facet_use)
    return Invalid(event, canonical_event,
                   "Regenerated event ordinals differ from retained uses");
  const auto& feature = binding.vertices()[pair.feature[0]];
  if (event.vertex_use < feature.use_offset ||
      event.vertex_use >=
          std::size_t(feature.use_offset) + feature.use_count ||
      !SamePoint(vertex.point, event.endpoints[0]) ||
      fixed_triangle_features::Compare(
          vertex.key, event.feature.vertex_face.vertex) ||
      !SameCertificate(vertex.directed_vf_area_m2,
                       pair.candidate_directed_area_m2) ||
      !SameCertificate(pair.candidate_directed_area_m2,
                       pair.admitted_force_area_m2) ||
      !PositiveSelfContactArea(pair.admitted_force_area_m2))
    return Invalid(event, canonical_event,
                   "Event vertex map, feature or directed area is unauthenticated");

  // Immutable facet data authenticates target membership and provenance. The
  // exact discovered target stratum still requires the discovery receipt that
  // intentionally lives outside this accepted-state force contributor.
  if (!TargetBelongsToFacet(binding, pair,
                            event.feature.vertex_face.target))
    return Invalid(event, canonical_event,
                   "Event target stratum is foreign to its retained facet");
  return {};
}

}  // namespace tlfea::contact::self_contact_force
