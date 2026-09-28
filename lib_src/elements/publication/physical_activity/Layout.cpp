// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../../ShellPhysicalOutputRanges.h"
namespace tl::fea::physical_activity {
bool MakeLayout(std::size_t parents, std::size_t cap, Layout& output) noexcept {
  util::BoundedArenaLayout builder(cap);
  Layout next;
  for (auto& mask : next.masks) if (!builder.Append<std::uint8_t>(parents, mask)) return false;
  if (!builder.Append<std::uint8_t>(parents, next.laws) ||
      !builder.Append<FamilyControl>(2, next.control)) return false;
  next.bytes = builder.bytes(); output = next; return true;
}
}
namespace tl::fea {
PhysicalActivityReport PhysicalActivitySnapshot::Preflight(const ShellPhysicalBinding& source,
    PhysicalActivityLimits limits, PhysicalActivityForecast& output) noexcept {
  using S = PhysicalActivityStatus;
  if (!source.prepared() || !source.shells() || !source.failure() || !source.catalog() ||
      !source.coefficients() || !limits.max_family_parents ||
      !shell_physical_owner::OutputDisjoint(source, &output, sizeof(output)))
    return {S::InvalidInput, "Complete physical source and disjoint forecast output are required"};
  const auto& shells = *source.shells();
  PhysicalActivityForecast next;
  next.qeph_count = shells.qeph_count(); next.t3_count = shells.t3_count(); next.qbat_count = shells.qbat_count();
  if (next.qeph_count > limits.max_family_parents || next.t3_count > limits.max_family_parents ||
      next.qbat_count > limits.max_family_parents || limits.max_family_parents > UINT32_MAX ||
      next.qeph_count > SIZE_MAX - next.t3_count)
    return {S::ResourceLimit, "Physical activity family count exceeds its limit"};
  const auto count = next.qeph_count + next.t3_count;
  physical_activity::Layout layout;
  if (!physical_activity::MakeLayout(count, limits.max_device_bytes, layout))
    return {S::ResourceLimit, "Physical activity device arena exceeds its limit"};
  // Includes a conservative shared_ptr control block reservation. Source
  // backing is shared and remains charged by the composing physical model.
  next.owned_host_bytes = sizeof(PhysicalActivitySnapshot) + sizeof(physical_activity::State) + 64;
  constexpr std::size_t PreparationBytes = 16384;
  static_assert(4*sizeof(ShellPhysicalDiagnostics) + sizeof(physical_activity::QephInput) +
      sizeof(physical_activity::T3Input) + 2*sizeof(physical_activity::FamilyControl) +
      2*sizeof(PhysicalActivityReport) < PreparationBytes, "Activity preparation reservation is too small");
  next.preparation_host_bytes = PreparationBytes;
  if (count > SIZE_MAX - next.owned_host_bytes - next.preparation_host_bytes)
    return {S::ResourceLimit, "Physical activity host forecast overflow"};
  next.startup_host_bytes = next.owned_host_bytes + next.preparation_host_bytes + count;
  next.device_bytes = layout.bytes;
  next.preparation_device_bytes = 0; // All scratch is in the retained arena.
  if (next.owned_host_bytes + next.preparation_host_bytes > limits.max_host_bytes ||
      next.startup_host_bytes > limits.max_startup_host_bytes)
    return {S::ResourceLimit, "Physical activity host workspace exceeds its limit"};
  output = next; return {};
}
} // namespace tl::fea
