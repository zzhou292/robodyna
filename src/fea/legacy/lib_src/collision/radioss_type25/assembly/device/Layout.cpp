// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Layout.h"
#include <climits>
namespace tlfea::contact::radioss_type25::assembly::device_detail {
IncidenceStatus CheckLimits(IncidenceLimits l) noexcept {
  // CUB uses a signed count in this qualified API. Reject, never truncate.
  if (!l.max_nodes || l.max_nodes >= UINT32_MAX || l.max_rows > INT_MAX / 5 ||
      l.max_cohorts > l.max_rows || !l.max_device_bytes ||
      (l.max_rows && !l.max_cohorts)) return IncidenceStatus::ResourceLimit;
  return IncidenceStatus::Ok;
}
IncidenceStatus MakeLayout(IncidenceLimits limits, std::size_t scratch,
    std::size_t host, Layout& output) noexcept {
  const auto status = CheckLimits(limits);
  if (status != IncidenceStatus::Ok) return status;
  Layout next;
  tl::util::BoundedArenaLayout arena(limits.max_device_bytes);
  if (!arena.Append<std::uint64_t>(5*limits.max_rows,next.keys) ||
      !arena.Append<std::uint64_t>(5*limits.max_rows,next.sorted_keys) ||
      !arena.Append<std::uint32_t>(5*limits.max_rows,next.occurrences) ||
      !arena.Append<std::uint32_t>(limits.max_nodes+1,next.offsets) ||
      !arena.Append<unsigned long long>(1,next.failure)) return IncidenceStatus::ResourceLimit;
  tl::util::ArenaRegion padding;
  if (!arena.Append<std::byte>((256-arena.bytes()%256)%256,padding) ||
      !arena.Append<std::byte>(scratch,next.cub)) return IncidenceStatus::ResourceLimit;
  next.forecast = {arena.bytes(),scratch,host};
  output = next;
  return IncidenceStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25::assembly::device_detail
