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
                std::size_t pair_capacity,
                std::size_t event_capacity, std::size_t max_bytes,
                Layout& output) noexcept {
  if (!nodes || !surface_parents || !parents || !facets ||
      !broadphase_pair_capacity || !pair_capacity ||
      !event_capacity || nodes > UINT32_MAX ||
      surface_parents > UINT32_MAX || parents >= UINT32_MAX ||
      facets > UINT32_MAX || pair_capacity > UINT32_MAX)
    return false;
  std::size_t vector_values = 0;
  if (!Product(nodes, 3, &vector_values)) return false;
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
      !builder.Append<CurrentFixedTriangle>(
          facets, next.accepted_triangles) ||
      !builder.Append<CurrentFixedTriangle>(
          facets, next.prepared_triangles) ||
      !builder.Append<RepresentedTrianglePath>(
          facets, next.represented_paths) ||
      !builder.Append<SelfContactPairKey>(
          broadphase_pair_capacity, next.accepted_broadphase_pairs) ||
      !builder.Append<SelfContactPairKey>(
          broadphase_pair_capacity, next.candidate_broadphase_pairs) ||
      !builder.Append<FixedTrianglePair>(
          pair_capacity, next.accepted_facet_pairs) ||
      !builder.Append<FixedTrianglePair>(
          pair_capacity, next.candidate_facet_pairs) ||
      !builder.Append<RepresentedTrianglePair>(
          pair_capacity, next.represented_pairs) ||
      !builder.Append<RepresentedIntervalPairKey>(
          pair_capacity, next.canonical_pairs) ||
      !builder.Append<SelfContactForceEvent>(
          event_capacity, next.accepted_events) ||
      !builder.Append<AcceptedEventCertificate>(
          event_capacity, next.accepted_certificates) ||
      !builder.Append<SelfContactCandidatePolicyOutcome>(
          pair_capacity, next.policy_outcomes))
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
      ArenaPointer<CurrentFixedTriangle>(base, layout.accepted_triangles),
      ArenaPointer<CurrentFixedTriangle>(base, layout.prepared_triangles),
      ArenaPointer<RepresentedTrianglePath>(base, layout.represented_paths),
      ArenaPointer<SelfContactPairKey>(
          base, layout.accepted_broadphase_pairs),
      ArenaPointer<SelfContactPairKey>(
          base, layout.candidate_broadphase_pairs),
      ArenaPointer<FixedTrianglePair>(base, layout.accepted_facet_pairs),
      ArenaPointer<FixedTrianglePair>(base, layout.candidate_facet_pairs),
      ArenaPointer<RepresentedTrianglePair>(base, layout.represented_pairs),
      ArenaPointer<RepresentedIntervalPairKey>(base, layout.canonical_pairs),
      ArenaPointer<SelfContactForceEvent>(base, layout.accepted_events),
      ArenaPointer<AcceptedEventCertificate>(
          base, layout.accepted_certificates),
      ArenaPointer<SelfContactCandidatePolicyOutcome>(
          base, layout.policy_outcomes)};
}

}  // namespace tlfea::contact::self_contact_transaction
