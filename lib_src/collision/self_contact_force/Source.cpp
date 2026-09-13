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
    std::size_t canonical_event) noexcept {
  const auto& pair = event.classification;
  const auto parents = binding.parents();
  const auto facets = binding.facet_uses();
  if (event.feature.kind != FixedTriangleCandidateKind::VertexFace ||
      !binding.Authenticates(pair) ||
      pair.binding_identity != binding.identity() ||
      pair.kind != SelfContactPairKind::VertexFace ||
      pair.status != SelfContactPairStatus::AdmittedVertexFace ||
      pair.excluded || pair.local_incidence ||
      !pair.active[0] || !pair.active[1] ||
      pair.parent[0] == pair.parent[1] ||
      pair.parent[0] >= parents.size() || pair.parent[1] >= parents.size() ||
      pair.feature[0] >= binding.vertices().size() ||
      pair.feature[1] >= facets.size() ||
      facets[pair.feature[1]].parent != pair.parent[1] ||
      pair.endpoint_support[0].status ==
          SelfContactSupportStatus::UnsupportedCinSecondary ||
      pair.endpoint_support[1].status ==
          SelfContactSupportStatus::UnsupportedCinSecondary)
    return Invalid(event, canonical_event,
                   "Event is not an authenticated admitted directed VF pair");
  if (!pair.activity_base_identity || !pair.activity_current_identity ||
      pair.activity_parent_count != parents.size() ||
      pair.activity_base_identity[pair.parent[0]] != 1 ||
      pair.activity_base_identity[pair.parent[1]] != 1 ||
      pair.activity_current_identity[pair.parent[0]] != 1 ||
      pair.activity_current_identity[pair.parent[1]] != 1)
    return Invalid(event, canonical_event,
                   "Event active-use activity identity is stale",
                   S::StaleAttempt);

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

  const SelfContactFacetVertexUse* vertex = nullptr;
  const auto& feature = binding.vertices()[pair.feature[0]];
  for (std::size_t i = feature.use_offset;
       i < std::size_t(feature.use_offset) + feature.use_count; ++i) {
    if (i >= binding.vertex_uses().size())
      return Invalid(event, canonical_event,
          "Retained vertex-use range is invalid");
    const auto& candidate = binding.vertex_uses()[i];
    if (candidate.parent == pair.parent[0] &&
        candidate.feature == pair.feature[0]) {
      if (vertex) return Invalid(event, canonical_event,
          "Retained vertex-use identity is ambiguous");
      vertex = &candidate;
    }
  }
  if (!vertex || !SamePoint(vertex->point, event.endpoints[0]) ||
      fixed_triangle_features::Compare(
          vertex->key, event.feature.vertex_face.vertex) ||
      !SameCertificate(vertex->directed_vf_area_m2,
                       pair.candidate_directed_area_m2) ||
      !SameCertificate(pair.candidate_directed_area_m2,
                       pair.admitted_force_area_m2) ||
      !PositiveSelfContactArea(pair.admitted_force_area_m2))
    return Invalid(event, canonical_event,
                   "Event vertex map, feature or directed area is unauthenticated");

  SelfContactSupportClassification first_support, second_support;
  auto report = binding.ClassifySupport(event.endpoints[0], &first_support);
  if (report.status != SelfContactActiveUseStatus::Ok)
    return Invalid(event, canonical_event,
                   "Event first support cannot be authenticated");
  report = binding.ClassifySupport(event.endpoints[1], &second_support);
  if (report.status != SelfContactActiveUseStatus::Ok ||
      !SameSupport(first_support, pair.endpoint_support[0]) ||
      !SameSupport(second_support, pair.endpoint_support[1]) ||
      second_support.status ==
          SelfContactSupportStatus::UnsupportedCinSecondary)
    return Invalid(event, canonical_event,
                   "Event support classification differs from retained source");
  if (!TargetBelongsToFacet(binding, pair,
                            event.feature.vertex_face.target))
    return Invalid(event, canonical_event,
                   "Event target stratum is foreign to its retained facet");
  return {};
}

}  // namespace tlfea::contact::self_contact_force
