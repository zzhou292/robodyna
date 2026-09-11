// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../ShellBatchPublicationImpl.h"
#include "../ShellPhysicalOwner.h"
#include "../ShellPhysicalOutputRanges.h"

namespace tl::fea {
ShellPublicationReport ShellBatchPublication::ForecastPhysical(const ShellPhysicalBinding& binding,
    std::size_t attachments,const ShellPublicationLimits& limits,
    ShellPhysicalPublicationForecast& output) noexcept {
  const bool vehicle = limits.profile == ShellResidentProfile::Vehicle;
  const auto ceiling = vehicle ? ShellPublicationLimits::Vehicle() : ShellPublicationLimits{};
  if ((!vehicle && limits.profile != ShellResidentProfile::Legacy) || !limits.max_nodes ||
      limits.max_nodes > ceiling.max_nodes || !limits.max_host_bytes ||
      limits.max_host_bytes > ceiling.max_host_bytes || !limits.max_device_bytes ||
      limits.max_device_bytes > ceiling.max_device_bytes)
    return {S::ResourceLimit,"Physical publication profile limits are invalid"};
  if (!binding.prepared() || !binding.domain())
    return {S::NotInitialized,"Physical publication requires an immutable physical binding"};
  if (!shell_physical_owner::OutputDisjoint(binding,&output,sizeof(output)) ||
      !trial_identity::Disjoint(&output,sizeof(output),&limits,sizeof(limits)))
    return {S::InvalidInput,"Physical forecast output overlaps its immutable input"};
  const auto count = binding.domain()->node_count();
  if (!count || count > limits.max_nodes)
    return {S::ResourceLimit,"Physical publication exceeds the explicit node capacity"};
  shell_physical_owner::ProofLayout proof;
  if (!shell_physical_owner::ForecastProof(count,attachments,limits.max_host_bytes,proof))
    return {S::ResourceLimit,"Physical publication initial proof exceeds the host cap"};
  util::BoundedArenaLayout budget(limits.max_host_bytes);
  util::ArenaRegion ignored;
  // Conservative shared allocation control reserve; allocator and driver RSS
  // are outside the owned payload convention, as in the retained value handles.
  constexpr std::size_t control_reserve = 64;
  if (!budget.Append<unsigned char>(sizeof(Impl),ignored) ||
      !budget.Append<unsigned char>(sizeof(PhysicalState),ignored) ||
      !budget.Append<unsigned char>(control_reserve,ignored))
    return {S::ResourceLimit,"Physical publication owned host payload exceeds its cap"};
  ShellPhysicalPublicationForecast next;
  next.owned_host_bytes = budget.bytes();
  if (!budget.Append<unsigned char>(proof.bytes,ignored))
    return {S::ResourceLimit,"Physical publication simultaneous proof exceeds its cap"};
  next.startup_host_bytes = budget.bytes();
  output = next;
  return Ok();
}
} // namespace tl::fea
