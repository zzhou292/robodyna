#include "Layout.h"
#include <algorithm>
#include <climits>

namespace tlfea::contact::self_contact_broadphase {
using S = SelfContactBroadphaseStatus;
SelfContactBroadphaseReport CheckSource(const SelfContactSurfaceBinding& source,
    SelfContactBroadphaseLimits limits) noexcept {
  if (!source.prepared() || !source.physical() || !source.physical()->domain())
    return {S::InvalidInput, "Prepared immutable S0 surface required"};
  if (!limits.max_parents || limits.max_parents > 524288 || !limits.max_nodes ||
      limits.max_nodes > 524288 || !limits.max_pairs || limits.max_pairs > INT_MAX ||
      !limits.max_device_bytes || limits.max_device_bytes > (2ull << 30) ||
      !limits.max_host_bytes || limits.max_host_bytes > (2ull << 30) ||
      source.parents().size() > limits.max_parents ||
      source.physical()->domain()->node_count() > limits.max_nodes)
    return {S::ResourceLimit, "Broadphase counts or full workspace limits exceed scope"};
  return {};
}
SelfContactBroadphaseReport MakeLayout(const SelfContactSurfaceBinding& source,
    SelfContactBroadphaseLimits limits, ScratchRequirements temp,
    std::size_t implementation_bytes, Layout& output) noexcept {
  const auto checked = CheckSource(source, limits);
  if (checked.status != S::Ok) return checked;
  Layout next;
  auto& f = next.forecast;
  f.parents = source.parents().size();
  f.nodes = source.physical()->domain()->node_count();
  f.pair_capacity = limits.max_pairs;
  f.sort_temp_bytes = temp.sort; f.scan_temp_bytes = temp.scan; f.pair_sort_temp_bytes = temp.pair_sort;
  tl::util::BoundedArenaLayout device(limits.max_device_bytes);
  if (!device.Append<Parent>(f.parents, next.parents) ||
      !device.Append<AABB>(f.parents, next.boxes) || !device.Append<AABB>(f.parents, next.sorted_boxes) ||
      !device.Append<double>(f.parents, next.keys) || !device.Append<double>(f.parents, next.sorted_keys) ||
      !device.Append<int>(f.parents, next.indices) || !device.Append<int>(f.parents, next.sorted_indices) ||
      !device.Append<unsigned long long>(f.parents + 1, next.counts) ||
      !device.Append<unsigned long long>(f.parents + 1, next.offsets) ||
      !device.Append<SelfContactPairKey>(f.pair_capacity, next.pair_keys) ||
      !device.Append<SelfContactPairKey>(f.pair_capacity, next.sorted_pair_keys) ||
      !device.Append<Control>(1, next.control))
    return {S::ResourceLimit, "Complete broadphase arrays exceed device cap"};
  // CUB temporary phases do not overlap. Keep its base at cudaMalloc alignment,
  // even though the preceding typed arena records need only natural alignment.
  tl::util::ArenaRegion padding;
  const auto remainder = device.bytes() % 256;
  if (!device.Append<std::byte>(remainder ? 256 - remainder : 0, padding) ||
      !device.Append<std::byte>(std::max({temp.sort, temp.scan, temp.pair_sort}), next.cub_temp))
    return {S::ResourceLimit, "CUB sort/scan workspace exceeds device cap"};
  f.device_bytes = device.bytes();
  const auto retained = source.forecast().owned_payload_bytes;
  if (retained < sizeof(source)) return {S::ResourceLimit, "Invalid retained source footprint"};
  f.retained_source_bytes = retained - sizeof(source);
  tl::util::BoundedArenaLayout host(limits.max_host_bytes);
  tl::util::ArenaRegion ignored;
  // Impl includes the retained source handle, forecast/layout and host scalar
  // readback. Startup/query descriptors are accounted by their represented types.
  if (!host.Append<std::byte>(sizeof(SelfContactBroadphase) + implementation_bytes, ignored) ||
      !host.Append<std::byte>(f.retained_source_bytes, ignored))
    return {S::ResourceLimit, "Retained broadphase and source exceed host cap"};
  f.owned_host_bytes = host.bytes();
  f.startup_scratch_bytes = next.parents.bytes + QueryStagingBytes;
  if (!host.Append<std::byte>(f.startup_scratch_bytes, ignored))
    return {S::ResourceLimit, "Parent upload/query staging exceeds host cap"};
  f.startup_host_bytes = host.bytes();
  output = next;
  return {};
}
bool CopyParents(const SelfContactSurfaceBinding& source, Parent* out, std::size_t count) noexcept {
  if (!source.prepared() || !out || count != source.parents().size()) return false;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& p = source.parents()[i];
    if ((p.arity != 3 && p.arity != 4) || !IsFinite(p.reference_half_thickness_m) ||
        p.reference_half_thickness_m <= 0) return false;
    Parent next;
    next.arity = p.arity; next.half_thickness = p.reference_half_thickness_m;
    for (unsigned local = 0; local < p.arity; ++local) {
      next.nodes[local] = p.arity == 3 ? p.t3.nodes[local] : p.q4.nodes[local];
      if (next.nodes[local] >= source.physical()->domain()->node_count()) return false;
    }
    out[i] = next;
  }
  return true;
}
} // namespace tlfea::contact::self_contact_broadphase
