// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "AssemblyTypes.h"
#include "lib_utils/BoundedArena.h"

namespace tl::fea::t3::mapped {
// This complete optional tail is charged as both startup host staging and
// resident device storage. Legacy initializers retain no gather arrays.
struct AssemblyLayout {
  util::ArenaRegion offsets, incidence, parent, node, failure;
  std::size_t bytes = 0;
  bool Initialize(std::size_t base, std::size_t parents, std::size_t nodes, std::size_t cap) noexcept {
    if (!parents || parents > UINT32_MAX / 3 || !nodes || nodes >= UINT32_MAX) return false;
    AssemblyLayout next;
    util::BoundedArenaLayout layout(cap);
    util::ArenaRegion ignored;
    if (!layout.Append<unsigned char>(base, ignored) ||
        !layout.Append<std::uint32_t>(nodes + 1, next.offsets) ||
        !layout.Append<std::uint32_t>(3 * parents, next.incidence) ||
        !layout.Append<AssemblyParent>(parents, next.parent) ||
        !layout.Append<AssemblyNode>(nodes, next.node) ||
        !layout.Append<unsigned long long>(1, next.failure)) return false;
    next.bytes = layout.bytes();
    *this = next;
    return true;
  }
  bool Construct(util::HostArena& arena, AssemblyMemory& memory) const noexcept {
    if (!bytes) {
      memory = {};
      return true;
    }
    memory = {arena.Construct<std::uint32_t>(offsets), arena.Construct<std::uint32_t>(incidence),
        arena.Construct<AssemblyParent>(parent), arena.Construct<AssemblyNode>(node),
        arena.Construct<unsigned long long>(failure)};
    return memory.offsets && memory.incidence && memory.parent && memory.node && memory.failure;
  }
  AssemblyMemory Rebase(void* base) const noexcept {
    if (!bytes) return {};
    return {util::ArenaPointer<std::uint32_t>(base, offsets), util::ArenaPointer<std::uint32_t>(base, incidence),
        util::ArenaPointer<AssemblyParent>(base, parent), util::ArenaPointer<AssemblyNode>(base, node),
        util::ArenaPointer<unsigned long long>(base, failure)};
  }
};
} // namespace tl::fea::t3::mapped
