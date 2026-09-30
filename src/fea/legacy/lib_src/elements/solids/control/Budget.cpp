// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Impl.h"
namespace tl::fea::solids::control::detail {
Report Plan(Input in,std::size_t count,Limits limits,std::size_t fixed,Layout& output) noexcept {
  const Limits hard;
  if(!limits.max_parents||limits.max_parents>hard.max_parents||!limits.max_packets||limits.max_packets>hard.max_packets||
     !limits.max_partitions||limits.max_partitions>hard.max_partitions||!limits.max_host_bytes||limits.max_host_bytes>hard.max_host_bytes)
    return {Status::ResourceLimit,"Solid control limits exceed bounded scope"};
  Layout next;
  if(in.profile==Profile::LegacyNoStructuralIcontrol) {
    if(in.source_instance_id||in.native_nvsiz||in.compiled_mvsiz||in.units.length_m||in.units.mass_kg||in.units.time_s||
       in.parents.size()||in.parents.data()||in.packets.size()||in.packets.data()||
       in.partitions.size()||in.partitions.data()||in.ordered_element_ids.size()||in.ordered_element_ids.data())
      return {Status::InvalidInput,"Legacy control profile requires no optional source arrays"};
    next.budget={sizeof(Selection),sizeof(Selection)};
    if(next.budget.startup_bytes>limits.max_host_bytes)return {Status::ResourceLimit,"Legacy control handle exceeds cap"};
    output=next;return {};
  }
  if(in.profile!=Profile::SourceDeclared)return {Status::UnsupportedProfile,"Unknown solid control source profile"};
  if(count>limits.max_parents||in.parents.size()>limits.max_parents||in.packets.size()>limits.max_packets||
     in.partitions.size()>limits.max_partitions||in.ordered_element_ids.size()>limits.max_parents)
    return {Status::ResourceLimit,"Solid control source count exceeds cap"};
  if(!count||in.parents.size()!=count||in.ordered_element_ids.size()!=count||!in.packets.size()||!in.partitions.size()||
     in.packets.size()>count||!in.source_instance_id||!in.native_nvsiz||!in.compiled_mvsiz||in.native_nvsiz>in.compiled_mvsiz)
    return {Status::InvalidInput,"Complete native solid roster and vector extents required"};
  solid_common::distortion::units_detail::Factors factors;
  if(!solid_common::distortion::units_detail::Make(in.units,factors))return {Status::UnsupportedProfile,"Unsupported solid native working units"};
  util::BoundedArenaLayout arena(limits.max_host_bytes),scratch(limits.max_host_bytes),peak(limits.max_host_bytes);
  util::ArenaRegion unused;
  if(!arena.Append<Parent>(count,next.parents)||!arena.Append<NativePartition>(in.partitions.size(),next.partitions)||
     !arena.Append<Packet>(in.packets.size(),next.packets)||!arena.Append<Member>(count,next.members)||
     !scratch.Append<std::uint8_t>(count,next.seen)||!peak.Append<unsigned char>(fixed,unused)||
     !peak.Append<unsigned char>(arena.bytes(),unused)||!peak.Append<unsigned char>(sizeof(Scratch),unused)||
     !peak.Append<unsigned char>(scratch.bytes(),unused)||!peak.Append<unsigned char>(Index::Bytes(count)-sizeof(Index),unused))
    return {Status::ResourceLimit,"Solid control ownership and validation bytes exceed cap before source reads"};
  if(!Range(in.parents)||!Range(in.partitions)||!Range(in.packets)||!Range(in.ordered_element_ids))
    return {Status::InvalidInput,"Solid control source pointer/count pair is invalid"};
  next.arena_bytes=arena.bytes();next.scratch_bytes=scratch.bytes();
  next.budget={fixed+arena.bytes(),peak.bytes()};output=next;return {};
}
} // namespace tl::fea::solids::control::detail
namespace tl::fea::solids::control {
Report Selection::Forecast(Input in,std::size_t count,Limits limits,Budget& output) noexcept {
  detail::Layout next;const auto report=detail::Plan(in,count,limits,sizeof(Selection)+sizeof(Impl)+64,next);
  if(report)output=next.budget;return report;
}
} // namespace tl::fea::solids::control
