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

bool MakeLayout(std::size_t nodes, std::size_t parents,
                std::size_t triangle_capacity, std::size_t pair_capacity,
                std::size_t event_capacity, std::size_t max_bytes,
                Layout& output) noexcept {
  if (!nodes || !parents || !triangle_capacity || !pair_capacity ||
      !event_capacity || nodes > UINT32_MAX ||
      triangle_capacity > UINT32_MAX || pair_capacity > UINT32_MAX)
    return false;
  std::size_t vector_values = 0;
  if (!Product(nodes, 3, &vector_values)) return false;
  tl::util::BoundedArenaLayout builder(max_bytes);
  Layout next;
  if (!builder.Append<double>(vector_values, next.accepted_positions) ||
      !builder.Append<double>(vector_values, next.accepted_velocities) ||
      !builder.Append<double>(vector_values, next.prepared_positions) ||
      !builder.Append<double>(vector_values, next.prepared_velocities) ||
      !builder.Append<std::uint8_t>(parents, next.activity) ||
      !builder.Append<CurrentFixedTriangle>(
          triangle_capacity, next.current_triangles) ||
      !builder.Append<RepresentedTrianglePath>(
          triangle_capacity, next.represented_paths) ||
      !builder.Append<RepresentedTrianglePair>(
          pair_capacity, next.represented_pairs) ||
      !builder.Append<RepresentedIntervalPairKey>(
          pair_capacity, next.canonical_pairs) ||
      !builder.Append<AcceptedEventIdentity>(
          event_capacity, next.accepted_events))
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
      ArenaPointer<std::uint8_t>(base, layout.activity),
      ArenaPointer<CurrentFixedTriangle>(base, layout.current_triangles),
      ArenaPointer<RepresentedTrianglePath>(base, layout.represented_paths),
      ArenaPointer<RepresentedTrianglePair>(base, layout.represented_pairs),
      ArenaPointer<RepresentedIntervalPairKey>(base, layout.canonical_pairs),
      ArenaPointer<AcceptedEventIdentity>(base, layout.accepted_events)};
}

}  // namespace tlfea::contact::self_contact_transaction
