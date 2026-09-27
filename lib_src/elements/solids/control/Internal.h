// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Selection.h"
#include "lib_utils/BoundedArena.h"
#include "lib_utils/SourceIdentityIndex.h"
namespace tl::fea::solids::control::detail {
using Index=util::SourceIdentityIndex<0>;
struct Layout {
  util::ArenaRegion parents,partitions,packets,members,seen;
  std::size_t arena_bytes=0,scratch_bytes=0;
  Budget budget;
};
struct Storage {
  util::HostArena arena;
  Parent* parents=nullptr;NativePartition* partitions=nullptr;Packet* packets=nullptr;Member* members=nullptr;
};
struct Scratch {Index identities;util::HostArena arena;std::uint8_t* seen=nullptr;};
Report Plan(Input,std::size_t,Limits,std::size_t,Layout&) noexcept;
Report BindParents(const SolidNodeContributions&,Input,Storage&,Scratch&,std::size_t&) noexcept;
Report BindPackets(Input,Storage&,Scratch&) noexcept;
inline bool Supported(Family family)noexcept {
  return family==Family::Solid24||family==Family::Solid6z||family==Family::Solid18Law90;
}
template<class T> inline bool Range(util::ConstView<T> v)noexcept {
  if(!v.size())return v.data()==nullptr;
  const auto at=reinterpret_cast<std::uintptr_t>(v.data());
  return v.data()&&at%alignof(T)==0&&v.size()<=(UINTPTR_MAX-at)/sizeof(T);
}
} // namespace tl::fea::solids::control::detail
