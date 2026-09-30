// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstdint>

namespace tl::util {
// Topology-neutral flat incidence. Callers retain their own source, role and
// repeated-slot policy. Every encoded occurrence stays in parent/slot order.
template<unsigned Slots, class NodeAt>
bool BuildOrderedNodeIncidence(std::size_t parents, std::size_t nodes, NodeAt node_at,
    std::uint32_t* offsets, std::size_t offset_count,
    std::uint32_t* incidence, std::size_t incidence_count) noexcept {
  static_assert(Slots > 0, "An incidence needs source slots");
  if (!parents || parents > UINT32_MAX / Slots || !nodes || nodes >= UINT32_MAX ||
      offset_count != nodes + 1 || incidence_count != Slots * parents ||
      !offsets || !incidence) return false;
  for (std::size_t parent = 0; parent < parents; ++parent)
    for (unsigned slot = 0; slot < Slots; ++slot)
      if (node_at(parent, slot) >= nodes) return false;
  for (std::size_t node = 0; node <= nodes; ++node) offsets[node] = 0;
  for (std::size_t parent = 0; parent < parents; ++parent)
    for (unsigned slot = 0; slot < Slots; ++slot)
      ++offsets[node_at(parent, slot) + 1];
  for (std::size_t node = 1; node <= nodes; ++node) offsets[node] += offsets[node - 1];
  for (std::size_t parent = 0; parent < parents; ++parent)
    for (unsigned slot = 0; slot < Slots; ++slot)
      incidence[offsets[node_at(parent, slot)]++] =
          static_cast<std::uint32_t>(Slots * parent + slot);
  for (std::size_t node = nodes; node > 0; --node) offsets[node] = offsets[node - 1];
  offsets[0] = 0;
  return true;
}
} // namespace tl::util
