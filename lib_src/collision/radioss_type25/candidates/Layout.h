// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "InventoryTypes.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::candidates::detail {
inline constexpr unsigned TaskWidth=256;
struct MainEntry {Main source;std::uint32_t rank=0;};
struct Range {std::uint32_t first=0,last=0;};
struct Task {std::uint32_t main=0,first=0,last=0;};
struct Control {
  unsigned long long failure=~0ull,maximum_gap_bits=0,active=0,encounters=0,tasks=0,pairs=0;
};
struct Layout {
  tl::util::ArenaRegion ids,codes,secondary,mains,ranks,removal_offsets,removals;
  tl::util::ArenaRegion keys,sorted_keys,ordinals,sorted_ordinals,ranges,task_counts,task_offsets,tasks;
  tl::util::ArenaRegion pair_counts,pair_offsets,pair_keys,sorted_pair_keys,pairs,secondary_offsets,control,cub;
  Forecast forecast;
};
// Policy-neutral allocation shape for the shared sweep. This is private storage,
// never an Engine source/count/profile admission bypass.
struct StorageShape {std::size_t nodes=0,secondaries=0,mains=0,removals=0;};
Status MakeStorageLayout(StorageShape,Limits,std::size_t,std::size_t,Layout&) noexcept;
Status CheckSource(const Source&,Limits) noexcept;
Status MakeLayout(const Source&,Limits,std::size_t,std::size_t,Layout&) noexcept;
} // namespace tlfea::contact::radioss_type25::candidates::detail
