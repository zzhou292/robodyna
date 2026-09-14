// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tlfea::contact::self_contact_transaction {
namespace {

bool Product(std::size_t a, std::size_t b, std::size_t* output) noexcept {
  if (!output || (a && b > SIZE_MAX / a)) return false;
  *output = a * b;
  return true;
}

}  // namespace

PairMotionAction ClassifyCandidatePairMotion(
    const MotionSupport& first,
    const SelfContactSweptParentBounds& first_bounds,
    const MotionSupport& second,
    const SelfContactSweptParentBounds& second_bounds) noexcept {
  if (first.motion == SelfContactFacetMotion::CompleteRigidGroup &&
      second.motion == SelfContactFacetMotion::CompleteRigidGroup &&
      first.complete_rigid_group != UINT32_MAX &&
      first.complete_rigid_group == second.complete_rigid_group)
    return PairMotionAction::ExcludedSameRigidGroup;
  const bool separated =
      first_bounds.upper.x < second_bounds.lower.x ||
      second_bounds.upper.x < first_bounds.lower.x ||
      first_bounds.upper.y < second_bounds.lower.y ||
      second_bounds.upper.y < first_bounds.lower.y ||
      first_bounds.upper.z < second_bounds.lower.z ||
      second_bounds.upper.z < first_bounds.lower.z;
  if (first.motion == SelfContactFacetMotion::LinearNodalV1 &&
      second.motion == SelfContactFacetMotion::LinearNodalV1)
    return separated ? PairMotionAction::CertifiedLinearSeparation
                     : PairMotionAction::LinearNodalV1;
  return separated ? PairMotionAction::CertifiedRigidArcSeparation
                   : PairMotionAction::UnsupportedRigidArc;
}

bool MakeLayout(std::size_t nodes, std::size_t surface_parents,
                std::size_t parents, std::size_t facets,
                std::size_t rigid_groups,
                std::size_t broadphase_pair_capacity,
                std::size_t pair_chunk_capacity,
                std::size_t event_capacity,
                std::size_t event_ledger_capacity,
                std::size_t event_hash_capacity,
                std::size_t policy_outcome_capacity,
                std::size_t max_bytes,
                Layout& output) noexcept {
  if (!nodes || !surface_parents || !parents || !facets ||
      !broadphase_pair_capacity || !pair_chunk_capacity ||
      !event_capacity || !event_ledger_capacity ||
      !event_hash_capacity || event_capacity > event_ledger_capacity ||
      event_ledger_capacity > UINT32_MAX ||
      nodes > UINT32_MAX ||
      surface_parents > UINT32_MAX || parents >= UINT32_MAX ||
      facets > UINT32_MAX || pair_chunk_capacity > UINT32_MAX)
    return false;
  std::size_t vector_values = 0;
  std::size_t identity_references = 0;
  std::size_t chunk_paths = 0;
  std::size_t chunk_events = 0;
  if (!Product(nodes, 3, &vector_values) ||
      !Product(facets, 3, &identity_references) ||
      !Product(pair_chunk_capacity, 2, &chunk_paths) ||
      !Product(pair_chunk_capacity, 15, &chunk_events) ||
      identity_references > UINT32_MAX)
    return false;
  tl::util::BoundedArenaLayout builder(max_bytes);
  Layout next;
  if (!builder.Append<double>(vector_values, next.accepted_positions) ||
      !builder.Append<double>(vector_values, next.accepted_velocities) ||
      !builder.Append<double>(vector_values, next.prepared_positions) ||
      !builder.Append<double>(vector_values, next.prepared_velocities) ||
      !builder.Append<tl::fea::NodalRigidGroupSnapshot>(
          rigid_groups, next.accepted_rigid_groups) ||
      !builder.Append<tl::fea::NodalRigidGroupSnapshot>(
          rigid_groups, next.prepared_rigid_groups) ||
      !builder.Append<std::uint32_t>(
          nodes, next.node_rigid_groups) ||
      !builder.Append<std::uint32_t>(
          surface_parents, next.surface_to_active) ||
      !builder.Append<std::uint32_t>(
          parents + 1, next.parent_facet_offsets) ||
      !builder.Append<FixedContactFacet>(
          facets, next.facet_descriptors) ||
      !builder.Append<MotionSupport>(
          parents, next.parent_motion) ||
      !builder.Append<MotionSupport>(
          facets, next.facet_motion) ||
      !builder.Append<std::uint32_t>(facets, next.triangle_order) ||
      !builder.Append<std::uint32_t>(
          identity_references, next.vertex_identity_order) ||
      !builder.Append<std::uint32_t>(
          identity_references, next.edge_identity_order) ||
      !builder.Append<CurrentFixedTriangle>(
          facets, next.accepted_triangles) ||
      !builder.Append<CurrentFixedTriangle>(
          facets, next.prepared_triangles) ||
      !builder.Append<SelfContactPairKey>(
          broadphase_pair_capacity, next.broadphase_pairs) ||
      !builder.Append<FacetPairCursor>(
          broadphase_pair_capacity, next.facet_pair_cursors) ||
      !builder.Append<std::uint32_t>(
          broadphase_pair_capacity, next.facet_pair_heap) ||
      !builder.Append<FixedTrianglePair>(
          pair_chunk_capacity, next.facet_pair_chunk) ||
      !builder.Append<FixedTriangleFeatureTaskMask>(
          pair_chunk_capacity, next.chunk_feature_task_masks) ||
      !builder.Append<RepresentedTrianglePath>(
          chunk_paths, next.chunk_paths) ||
      !builder.Append<RepresentedTrianglePair>(
          pair_chunk_capacity, next.chunk_represented_pairs) ||
      !builder.Append<RepresentedIntervalPairKey>(
          pair_chunk_capacity, next.chunk_canonical_pairs) ||
      !builder.Append<RepresentedIntervalPairKey>(
          pair_chunk_capacity, next.chunk_raw_canonical_pairs) ||
      !builder.Append<PairMotionAction>(
          pair_chunk_capacity, next.chunk_motion_actions) ||
      !builder.Append<RepresentedIntervalResult>(
          pair_chunk_capacity, next.chunk_crossings) ||
      !builder.Append<SelfContactCandidatePolicyOutcome>(
          pair_chunk_capacity, next.chunk_validated_outcomes) ||
      !builder.Append<SelfContactForceEvent>(
          chunk_events, next.chunk_events) ||
      !builder.Append<AcceptedEventCertificate>(
          chunk_events, next.chunk_certificates) ||
      !builder.Append<SelfContactSweptParentBounds>(
          surface_parents, next.swept_parent_bounds) ||
      !builder.Append<SelfContactSweptParentBounds>(
          facets, next.swept_facet_bounds) ||
      !builder.Append<SelfContactForceEvent>(
          event_capacity, next.accepted_events) ||
      !builder.Append<AcceptedEventCertificate>(
          event_ledger_capacity, next.accepted_certificates) ||
      !builder.Append<std::uint32_t>(
          event_hash_capacity, next.accepted_event_hash) ||
      !builder.Append<SelfContactCandidatePolicyOutcome>(
          pair_chunk_capacity, next.chunk_policy_outcomes) ||
      !builder.Append<SelfContactCandidatePolicyOutcome>(
          policy_outcome_capacity, next.policy_outcomes))
    return false;
  next.bytes = builder.bytes();
  output = next;
  return true;
}

Buffers Bind(void* base, const Layout& layout) noexcept {
  using tl::util::ArenaPointer;
  return {
      ArenaPointer<double>(base, layout.accepted_positions),
      ArenaPointer<double>(base, layout.accepted_velocities),
      ArenaPointer<double>(base, layout.prepared_positions),
      ArenaPointer<double>(base, layout.prepared_velocities),
      ArenaPointer<tl::fea::NodalRigidGroupSnapshot>(
          base, layout.accepted_rigid_groups),
      ArenaPointer<tl::fea::NodalRigidGroupSnapshot>(
          base, layout.prepared_rigid_groups),
      ArenaPointer<std::uint32_t>(base, layout.node_rigid_groups),
      ArenaPointer<std::uint32_t>(base, layout.surface_to_active),
      ArenaPointer<std::uint32_t>(base, layout.parent_facet_offsets),
      ArenaPointer<FixedContactFacet>(base, layout.facet_descriptors),
      ArenaPointer<MotionSupport>(base, layout.parent_motion),
      ArenaPointer<MotionSupport>(base, layout.facet_motion),
      ArenaPointer<std::uint32_t>(base, layout.triangle_order),
      ArenaPointer<std::uint32_t>(base, layout.vertex_identity_order),
      ArenaPointer<std::uint32_t>(base, layout.edge_identity_order),
      ArenaPointer<CurrentFixedTriangle>(base, layout.accepted_triangles),
      ArenaPointer<CurrentFixedTriangle>(base, layout.prepared_triangles),
      ArenaPointer<SelfContactPairKey>(
          base, layout.broadphase_pairs),
      ArenaPointer<FacetPairCursor>(base, layout.facet_pair_cursors),
      ArenaPointer<std::uint32_t>(base, layout.facet_pair_heap),
      ArenaPointer<FixedTrianglePair>(base, layout.facet_pair_chunk),
      ArenaPointer<FixedTriangleFeatureTaskMask>(
          base, layout.chunk_feature_task_masks),
      ArenaPointer<RepresentedTrianglePath>(base, layout.chunk_paths),
      ArenaPointer<RepresentedTrianglePair>(
          base, layout.chunk_represented_pairs),
      ArenaPointer<RepresentedIntervalPairKey>(
          base, layout.chunk_canonical_pairs),
      ArenaPointer<RepresentedIntervalPairKey>(
          base, layout.chunk_raw_canonical_pairs),
      ArenaPointer<PairMotionAction>(
          base, layout.chunk_motion_actions),
      ArenaPointer<RepresentedIntervalResult>(
          base, layout.chunk_crossings),
      ArenaPointer<SelfContactCandidatePolicyOutcome>(
          base, layout.chunk_validated_outcomes),
      ArenaPointer<SelfContactForceEvent>(base, layout.chunk_events),
      ArenaPointer<AcceptedEventCertificate>(
          base, layout.chunk_certificates),
      ArenaPointer<SelfContactSweptParentBounds>(
          base, layout.swept_parent_bounds),
      ArenaPointer<SelfContactSweptParentBounds>(
          base, layout.swept_facet_bounds),
      ArenaPointer<SelfContactForceEvent>(base, layout.accepted_events),
      ArenaPointer<AcceptedEventCertificate>(
          base, layout.accepted_certificates),
      ArenaPointer<std::uint32_t>(base, layout.accepted_event_hash),
      ArenaPointer<SelfContactCandidatePolicyOutcome>(
          base, layout.chunk_policy_outcomes),
      ArenaPointer<SelfContactCandidatePolicyOutcome>(
          base, layout.policy_outcomes)};
}

}  // namespace tlfea::contact::self_contact_transaction
