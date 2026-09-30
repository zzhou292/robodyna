// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "History.h"
#include "lib_utils/BoundedArena.h"
#include "lib_src/elements/solids/control/Selection.h"
namespace tl::fea::solids::batch_detail::controlled {
inline constexpr unsigned Threads=128,Blocks=8,MaximumBlocks=32;
struct Work24 {h24::Scratch scratch;h24::Result result;};
struct Work90 {foam::Scratch scratch;foam::Result result;};
struct Workspace {
  union Values {Work24 h24;Work90 foam;TL_BRICK_HD Values()noexcept:h24{}{};} values;
  unsigned active=24;
  TL_BRICK_HD Work24& H24()noexcept {
    if(active!=24){::new(static_cast<void*>(&values.h24)) Work24{};active=24;}return values.h24;
  }
  TL_BRICK_HD Work90& Foam()noexcept {
    if(active!=90){::new(static_cast<void*>(&values.foam)) Work90{};active=90;}return values.foam;
  }
};
static_assert(std::is_trivially_copyable_v<Workspace>&&std::is_trivially_destructible_v<Workspace>);
static_assert(std::is_nothrow_default_constructible_v<Workspace>);
struct Storage {
  std::size_t* index24=nullptr;std::size_t* index90=nullptr;
  h24::Reference* reference24=nullptr;foam::Reference* reference90=nullptr;
  control::Packet* packets=nullptr;control::Member* members=nullptr;
  Workspace* workspace=nullptr;
  std::size_t packet_count=0,member_count=0,count24=0,count90=0;
  unsigned blocks=0;std::size_t workspace_count=0;
  std::size_t worker_begin[MaximumBlocks]{};
};
struct Layout {
  unsigned blocks=0;
  util::ArenaRegion index24,index90,reference24,reference90,packets,members,workspace;
  std::size_t worker_begin[MaximumBlocks]{};
};
struct Counts {
  unsigned blocks=0;
  std::size_t h24=0,foam=0,packets=0,members=0,workers=0;
  std::size_t worker_begin[MaximumBlocks]{};
};
inline bool Append(const Counts& count,std::size_t all24,std::size_t all90,
    util::BoundedArenaLayout& arena,Layout& output)noexcept {
  Layout next;next.blocks=count.blocks;
  if(!arena.Append<std::size_t>(count.h24?all24:0,next.index24)||
     !arena.Append<std::size_t>(count.foam?all90:0,next.index90)||
     !arena.Append<h24::Reference>(count.h24,next.reference24)||
     !arena.Append<foam::Reference>(count.foam,next.reference90)||
     !arena.Append<control::Packet>(count.packets,next.packets)||
     !arena.Append<control::Member>(count.members,next.members)||
     !arena.Append<Workspace>(count.workers,next.workspace))return false;
  for(unsigned n=0;n<MaximumBlocks;++n)next.worker_begin[n]=count.worker_begin[n];
  output=next;return true;
}
inline Storage Rebase(void* base,const Layout& l)noexcept {
  Storage s;s.blocks=l.blocks;s.workspace_count=l.workspace.count;
  if(l.index24.count)s.index24=util::ArenaPointer<std::size_t>(base,l.index24);
  if(l.index90.count)s.index90=util::ArenaPointer<std::size_t>(base,l.index90);
  if(l.reference24.count)s.reference24=util::ArenaPointer<h24::Reference>(base,l.reference24);
  if(l.reference90.count)s.reference90=util::ArenaPointer<foam::Reference>(base,l.reference90);
  if(l.packets.count)s.packets=util::ArenaPointer<control::Packet>(base,l.packets);
  if(l.members.count)s.members=util::ArenaPointer<control::Member>(base,l.members);
  if(l.workspace.count)s.workspace=util::ArenaPointer<Workspace>(base,l.workspace);
  s.packet_count=l.packets.count;s.member_count=l.members.count;s.count24=l.reference24.count;s.count90=l.reference90.count;
  for(unsigned n=0;n<MaximumBlocks;++n)s.worker_begin[n]=l.worker_begin[n];return s;
}
} // namespace tl::fea::solids::batch_detail::controlled
