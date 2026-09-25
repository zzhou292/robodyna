// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Layout.h"
#include "Ranges.h"
namespace tlfea::contact::radioss_type25::search::detail {
Status CheckSource(const Source& s,Limits limits) noexcept {
  const Limits hard;
  if (!limits.max_nodes || limits.max_nodes>hard.max_nodes ||
      !limits.max_role_entries || limits.max_role_entries>hard.max_role_entries ||
      !limits.max_main_segments || limits.max_main_segments>hard.max_main_segments ||
      !limits.max_device_bytes || limits.max_device_bytes>hard.max_device_bytes ||
      !limits.max_host_bytes || limits.max_host_bytes>hard.max_host_bytes) return Status::ResourceLimit;
  if (!s.stamp.source || !s.stamp.topology || !s.stamp.activity || !s.physical_nodes ||
      !normal_detail::Nonnegative(s.margin) ||
      (s.input_units!=InputUnits::Native && s.input_units!=InputUnits::Si) ||
      (s.gap_mode!=GapMode::Fixed && s.gap_mode!=GapMode::CurrentMainGaps)) return Status::InvalidInput;
  if (s.processors!=1 || s.edge_mode!=0 || s.converged!=1) return Status::UnsupportedProfile;
  if (s.physical_nodes>limits.max_nodes || s.secondaries>limits.max_role_entries ||
      s.mains>limits.max_role_entries-s.secondaries ||
      s.main_1d>limits.max_role_entries-s.secondaries-s.mains ||
      s.main_segments>limits.max_main_segments) return Status::ResourceLimit;
  if (!(s.secondaries || s.main_1d) || !(s.mains || s.main_1d) ||
      (s.gap_mode==GapMode::CurrentMainGaps && !s.main_segments)) return Status::UnsupportedProfile;
  units_detail::Factors factors;
  if (!units_detail::Make(s.units,factors)) return Status::InvalidInput;
  const std::uint32_t* maps[]{s.secondary_nodes,s.main_nodes,s.main_1d_nodes};
  const std::size_t counts[]{s.secondaries,s.mains,s.main_1d};
  for (unsigned group=0;group<3;++group) {
    if (!Span(maps[group], counts[group])) return Status::InvalidInput;
    for (std::size_t i=0;i<counts[group];++i)
      if (maps[group][i]>=s.physical_nodes && !(group && maps[group][i]==UINT32_MAX))
        return Status::InvalidInput;
  }
  return Status::Ok;
}
Status MakeLayout(const Source& source,Limits limits,std::size_t owner_bytes,Layout& output) noexcept {
  const auto status=CheckSource(source,limits);if(status!=Status::Ok)return status;
  Layout next;
  auto& f=next.forecast;
  f.role_entries=source.secondaries+source.mains+source.main_1d;
  f.compact_reference=f.role_entries<source.physical_nodes;
  f.reference_positions=f.compact_reference ? f.role_entries : source.physical_nodes;
  f.reference_gaps=source.gap_mode==GapMode::CurrentMainGaps ? source.main_segments : 0;
  tl::util::BoundedArenaLayout arena(limits.max_device_bytes);
  if (!arena.Append<std::uint32_t>(f.role_entries,next.roles) ||
      !arena.Append<Partial>(MaximumBlocks,next.partials) ||
      !arena.Append<DeviceControl>(1,next.control)) return Status::ResourceLimit;
  for(unsigned i=0;i<2;++i)
    if (!arena.Append<Vector>(f.reference_positions,next.reference[i]) ||
        !arena.Append<std::uint8_t>(source.secondaries,next.masks[i]) ||
        !arena.Append<double>(f.reference_gaps,next.gaps[i])) return Status::ResourceLimit;
  f.device_bytes=arena.bytes();
  tl::util::BoundedArenaLayout host(limits.max_host_bytes);tl::util::ArenaRegion ignored;
  if (!host.Append<unsigned char>(owner_bytes,ignored) ||
      !host.Append<std::uint32_t>(f.role_entries,ignored)) return Status::ResourceLimit;
  f.startup_host_bytes=host.bytes();output=next;return Status::Ok;
}
Device Bind(void* base,const Layout& l,const Source& s,const units_detail::Factors& f) noexcept {
  Device d;
  d.roles=tl::util::ArenaPointer<std::uint32_t>(base,l.roles);
  d.partials=tl::util::ArenaPointer<Partial>(base,l.partials);
  d.control=tl::util::ArenaPointer<DeviceControl>(base,l.control);
  for(unsigned i=0;i<2;++i) {
    d.reference[i]=tl::util::ArenaPointer<Vector>(base,l.reference[i]);
    d.masks[i]=s.secondaries ? tl::util::ArenaPointer<std::uint8_t>(base,l.masks[i]) : nullptr;
    d.gaps[i]=l.forecast.reference_gaps ? tl::util::ArenaPointer<double>(base,l.gaps[i]) : nullptr;
  }
  d.nodes=s.physical_nodes;d.secondary=s.secondaries;d.main=s.mains;d.one_d=s.main_1d;
  d.segments=l.forecast.reference_gaps;d.reference_count=l.forecast.reference_positions;
  d.compact=l.forecast.compact_reference;d.gap_changes=s.gap_mode==GapMode::CurrentMainGaps;
  d.si=s.input_units==InputUnits::Si;d.length=f.length;d.velocity=f.velocity;return d;
}
} // namespace tlfea::contact::radioss_type25::search::detail
