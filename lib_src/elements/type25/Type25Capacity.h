// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>

namespace tl::fea::type25 {
enum class CapacityProfile { Legacy,Vehicle };
struct CapacityBounds {
  std::size_t connections,properties,nodes;
  std::size_t model_host_bytes,batch_device_bytes,batch_host_bytes,combined_host_bytes;
};
inline constexpr CapacityBounds LegacyCapacity{1024,64,2048,
  16u*1024u*1024u,16u*1024u*1024u,16u*1024u*1024u,16u*1024u*1024u};
inline constexpr CapacityBounds VehicleCapacity{4096,64,524288,
  16u*1024u*1024u,32u*1024u*1024u,2ull*1024u*1024u*1024u,2ull*1024u*1024u*1024u};
inline constexpr bool ValidProfile(CapacityProfile p) noexcept {
  return p==CapacityProfile::Legacy||p==CapacityProfile::Vehicle;
}
// Callers reject unknown profiles before consulting their bounds.
inline constexpr CapacityBounds Bounds(CapacityProfile p) noexcept {
  return p==CapacityProfile::Vehicle?VehicleCapacity:LegacyCapacity;
}
} // namespace tl::fea::type25
