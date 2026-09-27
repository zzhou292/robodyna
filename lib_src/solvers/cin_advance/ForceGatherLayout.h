// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceGatherTypes.h"
#include "lib_utils/BoundedArena.h"

namespace tl::fea::cin_advance::force_gather {
struct Layout {
  util::ArenaRegion nodes, offsets, incidence, values, summary;
  util::ArenaRegion host_nodes, host_offsets, host_incidence, dense_offsets;
  std::size_t capacity = 0, device_bytes = 0, host_bytes = 0, temporary_bytes = 0;
  bool Initialize(std::size_t base, std::size_t node_count, std::size_t rows,
      std::size_t device_cap, std::size_t host_cap) noexcept {
    if (!node_count || node_count >= UINT32_MAX || !rows || rows > UINT32_MAX / 4) return false;
    Layout next;
    next.capacity = node_count < 4 * rows ? node_count : 4 * rows;
    util::BoundedArenaLayout device(device_cap), host(host_cap), temporary(host_cap);
    util::ArenaRegion prefix;
    if (!device.Append<std::byte>(base, prefix) ||
        !device.Append<std::uint32_t>(next.capacity, next.nodes) ||
        !device.Append<std::uint32_t>(next.capacity + 1, next.offsets) ||
        !device.Append<std::uint32_t>(4 * rows, next.incidence) ||
        !device.Append<Master>(next.capacity, next.values) ||
        !device.Append<Summary>(1, next.summary) ||
        !host.Append<std::uint32_t>(next.capacity, next.host_nodes) ||
        !host.Append<std::uint32_t>(next.capacity + 1, next.host_offsets) ||
        !host.Append<std::uint32_t>(4 * rows, next.host_incidence) ||
        !temporary.Append<std::uint32_t>(node_count + 1, next.dense_offsets)) return false;
    next.device_bytes = device.bytes();
    next.host_bytes = host.bytes();
    next.temporary_bytes = temporary.bytes();
    *this = next;
    return true;
  }
};
} // namespace tl::fea::cin_advance::force_gather
