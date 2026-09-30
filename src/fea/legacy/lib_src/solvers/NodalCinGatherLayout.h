// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalCinLayout.h"
#include "NodalStateLayout.h"

namespace tl::fea::nodal_detail {
// Optional implementation storage, selected only after the existing required
// owner layout fits. Both forecast and initialization call this same builder.
// Insufficient gather capacity leaves the required layout byte-for-byte intact.
inline bool SelectCinGatherLayout(CinLayout& cin, StateLayout& owner,
    const NodalCinLimits& limits, std::size_t owner_device_cap,
    std::size_t rigid_host_bytes, std::size_t owner_control_bytes) noexcept {
  if (cin.gather.device_bytes || owner.cin.offset > owner_device_cap ||
      cin.optional_device_bytes < cin.device_bytes || cin.host_bytes > limits.max_host_bytes) return false;
  const auto tails = cin.optional_device_bytes - cin.device_bytes;
  if (tails > limits.max_device_bytes) return false;
  const auto optional_cap = limits.max_device_bytes - tails;
  const auto complete_cap = owner_device_cap - owner.cin.offset;
  const auto device_cap = optional_cap < complete_cap ? optional_cap : complete_cap;
  cin_advance::force_gather::Layout gathered;
  if (!gathered.Initialize(cin.device_bytes, cin.nodes, cin.attachments, device_cap,
      limits.max_host_bytes - cin.host_bytes)) return false;
  util::BoundedArenaLayout host(limits.max_host_bytes), device(owner_device_cap);
  util::ArenaRegion unused, region;
  if (!host.Append<std::byte>(cin.host_bytes, unused) ||
      !host.Append<std::byte>(gathered.host_bytes, unused) ||
      !host.Append<std::byte>(gathered.temporary_bytes, unused) ||
      !CinOwnerHostFits(host.bytes(), rigid_host_bytes, owner.accepted.count,
          owner.fixed.count, owner_control_bytes, limits.max_host_bytes) ||
      !device.Append<std::byte>(owner.cin.offset, unused) ||
      !device.Append<std::byte>(gathered.device_bytes, region)) return false;
  cin.gather = gathered;
  cin.device_bytes = gathered.device_bytes;
  cin.optional_device_bytes = gathered.device_bytes + tails;
  cin.host_bytes = host.bytes();
  owner.cin = region;
  owner.bytes = device.bytes();
  return true;
}
} // namespace tl::fea::nodal_detail
