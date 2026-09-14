// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tlfea::contact::self_contact_transaction {
namespace {

bool Product(std::size_t a, std::size_t b, std::size_t* output) noexcept {
  if (!output || (a && b > SIZE_MAX / a)) return false;
  *output = a * b;
  return true;
}

}  // namespace

bool MakeLayout(std::size_t nodes, std::size_t surface_parents,
                std::size_t parents, std::size_t facets,
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
      !Product(pair_chunk_capacity, 6, &chunk_events) ||
      identity_references > UINT32_MAX)
    return false;
  tl::util::BoundedArenaLayout builder(max_bytes);
  Layout next;
  if (!builder.Append<double>(vector_values, next.accepted_positions) ||
      !builder.Append<double>(vector_values, next.accepted_velocities) ||
      !builder.Append<double>(vector_values, next.prepared_positions) ||
      !builder.Append<double>(vector_values, next.prepared_velocities) ||
      !builder.Append<std::uint32_t>(
          surface_parents, next.surface_to_active) ||
      !builder.Append<std::uint32_t>(
          parents + 1, next.parent_facet_offsets) ||
      !builder.Append<FixedContactFacet>(
          facets, next.facet_descriptors) ||
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
      !builder.Append<RepresentedTrianglePath>(
          chunk_paths, next.chunk_paths) ||
      !builder.Append<RepresentedTrianglePair>(
          pair_chunk_capacity, next.chunk_represented_pairs) ||
      !builder.Append<RepresentedIntervalPairKey>(
          pair_chunk_capacity, next.chunk_canonical_pairs) ||
      !builder.Append<SelfContactForceEvent>(
          chunk_events, next.chunk_events) ||
      !builder.Append<AcceptedEventCertificate>(
          chunk_events, next.chunk_certificates) ||
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
      ArenaPointer<std::uint32_t>(base, layout.surface_to_active),
      ArenaPointer<std::uint32_t>(base, layout.parent_facet_offsets),
      ArenaPointer<FixedContactFacet>(base, layout.facet_descriptors),
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
      ArenaPointer<RepresentedTrianglePath>(base, layout.chunk_paths),
      ArenaPointer<RepresentedTrianglePair>(
          base, layout.chunk_represented_pairs),
      ArenaPointer<RepresentedIntervalPairKey>(
          base, layout.chunk_canonical_pairs),
      ArenaPointer<SelfContactForceEvent>(base, layout.chunk_events),
      ArenaPointer<AcceptedEventCertificate>(
          base, layout.chunk_certificates),
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
