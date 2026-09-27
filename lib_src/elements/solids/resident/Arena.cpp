// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Arena.h"

namespace tl::fea::solids::batch_detail {
namespace {
bool LimitsValid(const BatchLimits& value, BatchProfile profile) noexcept {
  const BatchLimits hard = profile == BatchProfile::PhysicalCinSourceControlsV3
      ? SourceControlledBatchLimits() : BatchLimits{};
  return value.max_parents && value.max_parents <= hard.max_parents &&
      value.max_materials && value.max_materials <= hard.max_materials &&
      value.max_curve_points && value.max_curve_points <= hard.max_curve_points &&
      value.max_nodes && value.max_nodes <= hard.max_nodes &&
      value.max_device_bytes && value.max_device_bytes <= hard.max_device_bytes &&
      value.max_host_bytes && value.max_host_bytes <= hard.max_host_bytes;
}
template<class Traits> bool Append(std::size_t count, util::BoundedArenaLayout& device,
    util::BoundedArenaLayout& host, FamilyLayout& output) noexcept {
  return device.Append<typename Traits::Parent>(count, output.parents) &&
      device.Append<State<Traits>>(count, output.slab[0]) &&
      device.Append<State<Traits>>(count, output.slab[1]) &&
      device.Append<int>(count, output.status) &&
      device.Append<std::uint8_t>(count, output.result_valid) &&
      device.Append<MeasurementOperands<Traits::nodes>>(count, output.measurement) &&
      host.Append<State<Traits>>(count, output.staging);
}
template<class Traits> DeviceFamily<Traits> Rebase(void* base,
    const FamilyLayout& layout) noexcept {
  if (!layout.parents.count) return {};
  return {util::ArenaPointer<typename Traits::Parent>(base, layout.parents),
      {util::ArenaPointer<State<Traits>>(base, layout.slab[0]),
       util::ArenaPointer<State<Traits>>(base, layout.slab[1])},
      util::ArenaPointer<int>(base, layout.status), layout.parents.count,
      util::ArenaPointer<std::uint8_t>(base, layout.result_valid),
      util::ArenaPointer<MeasurementOperands<Traits::nodes>>(base, layout.measurement)};
}
} // namespace
bool MakeLayout(Counts count, const BatchConfig& config, ArenaLayout& output) noexcept {
  const auto& limits = config.limits;
  if (!LimitsValid(limits, config.profile) || count.solid18 > limits.max_parents ||
      count.solid24 > limits.max_parents - count.solid18 ||
      count.solid6z > limits.max_parents - count.solid18 - count.solid24 ||
      count.solid18_law44 > limits.max_parents - count.solid18 - count.solid24 - count.solid6z ||
      count.solid18_law90 > limits.max_parents - count.solid18 - count.solid24 - count.solid6z - count.solid18_law44 ||
      !(count.solid18 || count.solid24 || count.solid6z || count.solid18_law44 || count.solid18_law90) ||
      count.material36 > limits.max_materials ||
      count.material42 > limits.max_materials - count.material36 ||
      count.material44 > limits.max_materials - count.material36 - count.material42 ||
      count.material90 > limits.max_materials - count.material36 - count.material42 - count.material44 ||
      bool(count.solid18_law44) != bool(count.material44) ||
      bool(count.solid18_law90) != bool(count.material90) ||
      bool(count.solid18) != bool(count.material36) ||
      bool(count.solid24 || count.solid6z) != bool(count.material42) ||
      count.curve_points > limits.max_curve_points ||
      count.analytic_material44 > count.material44 ||
      count.controlled.h24>count.solid24||count.controlled.foam>count.solid18_law90||
      count.controlled.workers>controlled::Blocks*controlled::Threads||
      count.controlled.packets>limits.max_parents||count.controlled.members>limits.max_parents||
      bool(count.material36 || count.material44 - count.analytic_material44 || count.material90) != bool(count.curve_points) ||
      config.owner.node_count > limits.max_nodes) return false;
  ArenaLayout next;
  util::BoundedArenaLayout device(limits.max_device_bytes);
  util::BoundedArenaLayout host(limits.max_host_bytes);
  if (!device.Append<Storage>(1, next.header) ||
      !device.Append<solid18::Material>(count.material36, next.material36) ||
      !device.Append<solid24::Material>(count.material42, next.material42) ||
      !device.Append<solid18::law44::Material>(count.material44, next.material44) ||
      !device.Append<solid18::total_strain::Material>(count.material90, next.material90) ||
      !device.Append<double>(2 * count.curve_points, next.curves) ||
      !Append<Traits18>(count.solid18, device, host, next.solid18) ||
      !Append<Traits24>(count.solid24, device, host, next.solid24) ||
      !Append<Traits6z>(count.solid6z, device, host, next.solid6z) ||
      !device.Append<Scratch18>(Scratch18Count(count.solid18), next.scratch18) ||
      !Append<Traits18Law44>(count.solid18_law44, device, host, next.solid18_law44) ||
      !Append<Traits18Law90>(count.solid18_law90, device, host, next.solid18_law90) ||
      !device.Append<ExtendedScratch<Traits18Law44>>(Scratch18Count(count.solid18_law44), next.scratch44) ||
      !device.Append<ExtendedScratch<Traits18Law90>>(count.controlled.foam==count.solid18_law90?0:Scratch18Count(count.solid18_law90), next.scratch90) ||
      !controlled::Append(count.controlled,count.solid24,count.solid18_law90,device,next.controlled) ||
      !device.Append<std::uint32_t>(config.owner.node_count + 1, next.assembly_offsets) ||
      !device.Append<std::uint32_t>(8*(count.solid18 + count.solid24 + count.solid18_law44 +
          count.solid18_law90) + 6*count.solid6z, next.assembly_incidence) ||
      !device.Append<AssemblyNode>(config.owner.node_count, next.assembly_nodes) ||
      !shell_physical_owner::ForecastProof(config.owner.node_count,
          config.cin_attachment_count, limits.max_host_bytes, next.proof)) return false;
  next.bytes = device.bytes();
  next.staging_bytes = host.bytes();
  next.curve_points = count.curve_points;
  output = next;
  return true;
}
Storage RebasedHeader(void* base, const ArenaLayout& layout) noexcept {
  Storage next;
  next.controlled=controlled::Rebase(base,layout.controlled);
  if (layout.material36.count)
    next.material36 = util::ArenaPointer<solid18::Material>(base, layout.material36);
  if (layout.material42.count)
    next.material42 = util::ArenaPointer<solid24::Material>(base, layout.material42);
  if (layout.scratch18.count)
    next.scratch18 = util::ArenaPointer<Scratch18>(base, layout.scratch18);
  if (layout.material44.count)
    next.material44 = util::ArenaPointer<solid18::law44::Material>(base, layout.material44);
  if (layout.material90.count)
    next.material90 = util::ArenaPointer<solid18::total_strain::Material>(base, layout.material90);
  if (layout.scratch44.count)
    next.scratch44 = util::ArenaPointer<ExtendedScratch<Traits18Law44>>(base, layout.scratch44);
  if (layout.scratch90.count)
    next.scratch90 = util::ArenaPointer<ExtendedScratch<Traits18Law90>>(base, layout.scratch90);
  next.assembly = {util::ArenaPointer<std::uint32_t>(base, layout.assembly_offsets),
      util::ArenaPointer<std::uint32_t>(base, layout.assembly_incidence),
      util::ArenaPointer<AssemblyNode>(base, layout.assembly_nodes),
      layout.assembly_incidence.count, layout.bytes};
  next.solid18_law44 = Rebase<Traits18Law44>(base, layout.solid18_law44);
  next.solid18_law90 = Rebase<Traits18Law90>(base, layout.solid18_law90);
  next.solid18 = Rebase<Traits18>(base, layout.solid18);
  next.solid24 = Rebase<Traits24>(base, layout.solid24);
  next.solid6z = Rebase<Traits6z>(base, layout.solid6z);
  return next;
}
} // namespace tl::fea::solids::batch_detail
