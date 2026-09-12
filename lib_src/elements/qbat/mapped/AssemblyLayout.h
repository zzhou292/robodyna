// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "AssemblyTypes.h"
#include "lib_utils/BoundedArena.h"
namespace tl::fea::qbat::mapped {
struct AssemblyLayout {
  util::ArenaRegion offsets,incidence,parent,node,failure,maximum;
  std::size_t bytes=0;
  bool Initialize(std::size_t base,std::size_t parents,std::size_t nodes,std::size_t cap) noexcept {
    if(!parents || parents>UINT32_MAX/4 || !nodes || nodes>=UINT32_MAX) return false;
    AssemblyLayout next;
    util::BoundedArenaLayout layout(cap);
    util::ArenaRegion prefix;
    if(!layout.Append<unsigned char>(base,prefix) ||
        !layout.Append<std::uint32_t>(nodes+1,next.offsets) ||
        !layout.Append<std::uint32_t>(4*parents,next.incidence) ||
        !layout.Append<AssemblyParent>(parents,next.parent) ||
        !layout.Append<AssemblyNode>(nodes,next.node) ||
        !layout.Append<unsigned long long>(1,next.failure) ||
        !layout.Append<MaximumSummary>(MaximumBlocks(nodes),next.maximum)) return false;
    next.bytes=layout.bytes(); *this=next; return true;
  }
  bool Construct(util::HostArena& arena,AssemblyMemory& out) const noexcept {
    if(!bytes) {out={}; return true;}
    out={arena.Construct<std::uint32_t>(offsets),arena.Construct<std::uint32_t>(incidence),
      arena.Construct<AssemblyParent>(parent),arena.Construct<AssemblyNode>(node),
      arena.Construct<unsigned long long>(failure),arena.Construct<MaximumSummary>(maximum)};
    return out.offsets && out.incidence && out.parent && out.node && out.failure && out.maximum;
  }
  AssemblyMemory Rebase(void* base) const noexcept {
    if(!bytes) return {};
    return {util::ArenaPointer<std::uint32_t>(base,offsets),util::ArenaPointer<std::uint32_t>(base,incidence),
      util::ArenaPointer<AssemblyParent>(base,parent),util::ArenaPointer<AssemblyNode>(base,node),
      util::ArenaPointer<unsigned long long>(base,failure),util::ArenaPointer<MaximumSummary>(base,maximum)};
  }
};
} // namespace tl::fea::qbat::mapped
