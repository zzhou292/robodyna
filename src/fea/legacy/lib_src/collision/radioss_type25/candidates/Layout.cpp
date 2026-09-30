// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Layout.h"
#include "../search/Ranges.h"
#include <climits>
namespace tlfea::contact::radioss_type25::candidates::detail {
namespace {
Status CheckStrategy(Limits l) noexcept {
  if(l.strategy==EnumerationStrategy::LegacyAxisSweep)return l.max_encounters?Status::InvalidInput:Status::Ok;
  if(l.strategy!=EnumerationStrategy::CompactGrid)return Status::UnsupportedProfile;
  return !l.max_encounters||l.max_encounters>MaximumCompactEncounters?Status::ResourceLimit:Status::Ok;
}
}
Status CheckSource(const Source& s,Limits l) noexcept {
  const auto strategy=CheckStrategy(l);if(strategy!=Status::Ok)return strategy;
  const Limits hard;
  if(!l.max_nodes||l.max_nodes>hard.max_nodes||!l.max_secondaries||l.max_secondaries>hard.max_secondaries||
     !l.max_mains||l.max_mains>hard.max_mains||!l.max_removals||l.max_removals>hard.max_removals||
     !l.max_tasks||l.max_tasks>hard.max_tasks||!l.max_pairs||l.max_pairs>hard.max_pairs||
     !l.max_device_bytes||l.max_device_bytes>hard.max_device_bytes||
     !l.max_host_bytes||l.max_host_bytes>hard.max_host_bytes)return Status::ResourceLimit;
  units_detail::Factors factors;
  if(!units_detail::Make(s.units,factors)||(s.input_units!=InputUnits::Native&&s.input_units!=InputUnits::Si))return Status::InvalidInput;
  if(!s.stamp.source||!s.stamp.topology||!s.physical_nodes||s.primary_main_count<0||std::size_t(s.primary_main_count)!=s.mains)
    return Status::InvalidInput;
  if(s.processors!=1||s.edge_mode!=0||s.gap_mode!=1||s.level!=1||s.neighbor_removal!=2||
      (s.main_coefficient_domain!=MainCoefficientDomain::Nonnegative&&
       s.main_coefficient_domain!=MainCoefficientDomain::NativeSigned))
    return Status::UnsupportedProfile;
  if(s.physical_nodes>l.max_nodes||s.secondaries>l.max_secondaries||s.mains>l.max_mains||
     s.removals>l.max_removals)return Status::ResourceLimit;
  namespace r=search::detail;
  if(!r::Span(s.node_ids,s.physical_nodes)||!r::Span(s.constraint_codes,s.physical_nodes)||
     !r::Span(s.secondary_nodes,s.secondaries)||!r::Span(s.main,s.mains)||
     !r::Span(s.removal_offsets,s.mains+1)||!r::Span(s.removal_nodes,s.removals))return Status::InvalidInput;
  for(std::size_t i=0;i<s.physical_nodes;++i)
    if(!s.node_ids[i]||s.constraint_codes[i]<0||s.constraint_codes[i]>7)return Status::InvalidInput;
  for(std::size_t i=0;i<s.secondaries;++i)if(s.secondary_nodes[i]>=s.physical_nodes)return Status::InvalidInput;
  if(s.removal_offsets[0]!=0||s.removal_offsets[s.mains]!=s.removals)return Status::InvalidInput;
  for(std::size_t i=0;i<s.mains;++i) {
    if(!s.main[i].source_id||s.removal_offsets[i]>s.removal_offsets[i+1])return Status::InvalidInput;
    if(s.main[i].segment_type!=0&&s.main[i].segment_type<=s.primary_main_count)return Status::UnsupportedProfile;
    for(unsigned j=0;j<4;++j)if(s.main[i].nodes[j]>=s.physical_nodes)return Status::InvalidInput;
  }
  for(std::size_t i=0;i<s.removals;++i)if(s.removal_nodes[i]>=s.physical_nodes)return Status::InvalidInput;
  return Status::Ok;
}
Status MakeLayout(const Source& s,Limits l,std::size_t scratch,std::size_t owner,Layout& out) noexcept {
  const auto status=CheckSource(s,l);if(status!=Status::Ok)return status;
  return MakeStorageLayout({s.physical_nodes,s.secondaries,s.mains,s.removals},l,scratch,owner,out);
}
Status MakeStorageLayout(StorageShape s,Limits l,std::size_t scratch,std::size_t owner,Layout& out) noexcept {
  const auto strategy=CheckStrategy(l);if(strategy!=Status::Ok)return strategy;
  const Limits hard;
  if(!s.nodes||s.nodes>l.max_nodes||s.nodes>hard.max_nodes||s.secondaries>l.max_secondaries||
      s.secondaries>hard.max_secondaries||s.mains>l.max_mains||s.mains>hard.max_mains||
      s.removals>l.max_removals||s.removals>hard.max_removals||!l.max_tasks||l.max_tasks>8388608||
      !l.max_pairs||l.max_pairs>hard.max_pairs||!l.max_device_bytes||!l.max_host_bytes)return Status::ResourceLimit;
  Layout n;tl::util::BoundedArenaLayout a(l.max_device_bytes);
  if(!a.Append<std::uint64_t>(s.nodes,n.ids)||!a.Append<int>(s.nodes,n.codes)||
     !a.Append<std::uint32_t>(s.secondaries,n.secondary)||!a.Append<MainEntry>(s.mains,n.mains)||
     !a.Append<std::uint32_t>(s.mains,n.ranks)||!a.Append<std::uint64_t>(s.mains+1,n.removal_offsets)||
     !a.Append<std::uint32_t>(s.removals,n.removals)||!a.Append<double>(s.secondaries,n.keys)||
     !a.Append<double>(s.secondaries,n.sorted_keys)||!a.Append<std::uint32_t>(s.secondaries,n.ordinals)||
     !a.Append<std::uint32_t>(s.secondaries,n.sorted_ordinals)||!a.Append<Range>(s.mains,n.ranges)||
     !a.Append<unsigned long long>(s.mains+1,n.task_counts)||!a.Append<unsigned long long>(s.mains+1,n.task_offsets)||
     !a.Append<Task>(l.max_tasks,n.tasks)||!a.Append<unsigned long long>(l.max_tasks+1,n.pair_counts)||
     !a.Append<unsigned long long>(l.max_tasks+1,n.pair_offsets)||!a.Append<std::uint64_t>(l.max_pairs,n.pair_keys)||
     !a.Append<std::uint64_t>(l.max_pairs,n.sorted_pair_keys)||!a.Append<Pair>(l.max_pairs,n.pairs)||
     !a.Append<std::uint64_t>(s.secondaries+1,n.secondary_offsets)||!a.Append<Control>(1,n.control)||
     !a.Append<std::byte>(scratch,n.cub))return Status::ResourceLimit;
  if(l.strategy==EnumerationStrategy::CompactGrid) {
    const auto before=a.bytes();
    if(!a.Append<unsigned long long>(s.mains+1,n.encounter_counts)||
       !a.Append<unsigned long long>(s.mains+1,n.encounter_offsets)||
       !a.Append<std::uint32_t>(l.max_encounters,n.encounter_ordinals)||
       !a.Append<GridControl>(1,n.grid))return Status::ResourceLimit;
    n.forecast.index_device_bytes=a.bytes()-before;
  }
  n.forecast.device_bytes=a.bytes();n.forecast.cub_bytes=scratch;
  tl::util::BoundedArenaLayout host(l.max_host_bytes);tl::util::ArenaRegion ignored;
  if(!host.Append<std::byte>(owner,ignored)||!host.Append<std::uint64_t>(s.nodes,ignored)||
     !host.Append<MainEntry>(s.mains,ignored)||!host.Append<std::uint32_t>(s.mains,ignored)||
     !host.Append<std::uint32_t>(s.removals,ignored))return Status::ResourceLimit;
  n.forecast.startup_host_bytes=host.bytes();out=n;return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::candidates::detail
