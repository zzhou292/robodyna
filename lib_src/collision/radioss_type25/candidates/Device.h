// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Layout.h"
namespace tlfea::contact::radioss_type25::candidates::detail {
struct Device {
  const std::uint64_t* ids=nullptr;const int* codes=nullptr;
  const std::uint32_t* secondary=nullptr;const MainEntry* mains=nullptr;
  const std::uint32_t* ranks=nullptr;const std::uint64_t* removal_offsets=nullptr;
  const std::uint32_t* removals=nullptr;
  double* keys=nullptr;double* sorted_keys=nullptr;
  std::uint32_t* ordinals=nullptr;std::uint32_t* sorted_ordinals=nullptr;
  Range* ranges=nullptr;unsigned long long* task_counts=nullptr;unsigned long long* task_offsets=nullptr;
  Task* tasks=nullptr;unsigned long long* pair_counts=nullptr;unsigned long long* pair_offsets=nullptr;
  std::uint64_t* pair_keys=nullptr;std::uint64_t* sorted_pair_keys=nullptr;
  Pair* pairs=nullptr;std::uint64_t* secondary_offsets=nullptr;Control* control=nullptr;
  unsigned long long* encounter_counts=nullptr;unsigned long long* encounter_offsets=nullptr;
  std::uint32_t* encounter_ordinals=nullptr;GridControl* grid=nullptr;
  void* cub=nullptr;std::size_t cub_bytes=0,nodes=0,secondary_count=0,main_count=0,task_capacity=0,pair_capacity=0;
  EnumerationStrategy strategy=EnumerationStrategy::LegacyAxisSweep;std::size_t encounter_capacity=0;
  MainCoefficientDomain main_coefficient_domain=MainCoefficientDomain::Nonnegative;
  int primary_main_count=0;bool si=false;double length=1,velocity=1,time=1;
};
inline Device Bind(void* base,const Layout& l,const Source& s,Limits limits) noexcept {
  Device d;units_detail::Factors factors;units_detail::Make(s.units,factors);
  d.si=s.input_units==InputUnits::Si;d.length=factors.length;d.velocity=factors.velocity;d.time=factors.time;
#define BIND(field,type) d.field=tl::util::ArenaPointer<type>(base,l.field)
  BIND(ids,std::uint64_t);BIND(codes,int);BIND(secondary,std::uint32_t);BIND(mains,MainEntry);
  BIND(ranks,std::uint32_t);BIND(removal_offsets,std::uint64_t);BIND(removals,std::uint32_t);
  BIND(keys,double);BIND(sorted_keys,double);BIND(ordinals,std::uint32_t);BIND(sorted_ordinals,std::uint32_t);
  BIND(ranges,Range);BIND(task_counts,unsigned long long);BIND(task_offsets,unsigned long long);
  BIND(tasks,Task);BIND(pair_counts,unsigned long long);BIND(pair_offsets,unsigned long long);
  BIND(pair_keys,std::uint64_t);BIND(sorted_pair_keys,std::uint64_t);BIND(pairs,Pair);
  BIND(secondary_offsets,std::uint64_t);BIND(control,Control);BIND(cub,std::byte);
  if(limits.strategy==EnumerationStrategy::CompactGrid) {
    BIND(encounter_counts,unsigned long long);BIND(encounter_offsets,unsigned long long);
    BIND(encounter_ordinals,std::uint32_t);BIND(grid,GridControl);
  }
#undef BIND
  d.cub_bytes=l.cub.bytes;d.nodes=s.physical_nodes;d.secondary_count=s.secondaries;d.main_count=s.mains;
  d.task_capacity=limits.max_tasks;d.pair_capacity=limits.max_pairs;d.primary_main_count=s.primary_main_count;d.main_coefficient_domain=s.main_coefficient_domain;
  d.strategy=limits.strategy;d.encounter_capacity=limits.max_encounters;return d;
}
} // namespace tlfea::contact::radioss_type25::candidates::detail
