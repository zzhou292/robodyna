// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <algorithm>

namespace tl::constraints::tied_shell::driver_detail {
namespace {
bool Add(std::size_t& total, std::size_t count, std::size_t width, std::size_t cap) noexcept {
  if (!width || total > cap || count > (cap - total) / width) return false;
  total += count * width;
  return true;
}
}
std::size_t FixedDeviceBytes(const SearchDriverInput& in, Scratch scratch) noexcept {
  const auto e = in.master_count + in.secondary_count;
  return 3 * in.node_count * sizeof(double) + e * (4 * sizeof(int) + sizeof(int) +
      2 * sizeof(AABB) + 2 * sizeof(double) + 2 * sizeof(int) + sizeof(double)) +
      sizeof(Broadphase) + 2 * sizeof(int) + scratch.sort;
}
SearchDriverReport Budget(const SearchDriverInput& in, const SearchDriverLimits& limits,
    Scratch scratch, SearchDriverBudget& output) noexcept {
  const auto checked = CheckCounts(in, limits);
  if (!checked) return checked;
  const auto e = in.master_count + in.secondary_count;
  std::size_t device = FixedDeviceBytes(in, {0, 0});
  if (!Add(device, scratch.sort, 1, limits.max_device_bytes) ||
      !Add(device, 2 * (e + 1), sizeof(unsigned long long), limits.max_device_bytes) ||
      !Add(device, scratch.scan, 1, limits.max_device_bytes))
    return Error(SearchDriverStatus::ResourceLimit, "Tied search fixed device payload exceeds admission");
  std::size_t host = sizeof(Broadphase) + sizeof(SearchDriverResult) + sizeof(SearchDriverBudget);
  // Eigen staging and Broadphase's retained host copies coexist. Input/source
  // arrays and a caller's previous result are outside this internal payload.
  if (!Add(host, 6 * in.node_count, sizeof(double), limits.max_host_bytes) ||
      !Add(host, 10 * e, sizeof(int), limits.max_host_bytes) ||
      !Add(host, in.node_count, sizeof(unsigned char), limits.max_host_bytes) ||
      !Add(host, in.master_count, sizeof(NativeSearchBounds), limits.max_host_bytes) ||
      !Add(host, e, sizeof(double), limits.max_host_bytes) ||
      !Add(host, in.secondary_count, sizeof(SearchDriverRow), limits.max_host_bytes))
    return Error(SearchDriverStatus::ResourceLimit, "Tied search fixed host payload exceeds admission");
  const auto capacity = std::min({limits.max_pairs,
      (limits.max_host_bytes - host) / sizeof(CollisionPair),
      (limits.max_device_bytes - device) / sizeof(CollisionPair)});
  if (!capacity)
    return Error(SearchDriverStatus::ResourceLimit, "Tied search has no admitted candidate storage");
  host += capacity * sizeof(CollisionPair);
  device += capacity * sizeof(CollisionPair);
  output = {host, device, scratch.sort, scratch.scan, capacity};
  return {};
}
} // namespace tl::constraints::tied_shell::driver_detail
