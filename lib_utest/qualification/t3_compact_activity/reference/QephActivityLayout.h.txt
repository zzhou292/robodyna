// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utils/BoundedArena.h"
#include <cstdint>

namespace tl::fea::qeph::mapped {
struct ActivityMemory {
  std::uint32_t* first_invalid = nullptr;
  std::uint8_t* active = nullptr;
};

// One contiguous readback packet: first invalid parent, then one byte/parent.
// Only the explicit mapped arena owns this tail. It holds no accepted verdict.
inline constexpr std::size_t ActivityBytes(std::size_t parents) noexcept {
  return sizeof(std::uint32_t) + parents;
}
struct ActivityLayout {
  util::ArenaRegion first_invalid, active;
  std::size_t bytes = 0;

  bool Initialize(std::size_t base, std::size_t parents, std::size_t cap) noexcept {
    if (!parents || parents >= UINT32_MAX) return false;
    ActivityLayout next;
    util::BoundedArenaLayout layout(cap);
    util::ArenaRegion prefix;
    if (!layout.Append<unsigned char>(base,prefix) ||
        !layout.Append<std::uint32_t>(1,next.first_invalid) ||
        !layout.Append<std::uint8_t>(parents,next.active)) return false;
    next.bytes = layout.bytes();
    *this = next;
    return true;
  }
  bool Construct(util::HostArena& arena, ActivityMemory& output) const noexcept {
    if (!bytes) {
      output = {};
      return true;
    }
    output = {arena.Construct<std::uint32_t>(first_invalid),
        arena.Construct<std::uint8_t>(active)};
    return output.first_invalid && output.active;
  }
  ActivityMemory Rebase(void* base) const noexcept {
    if (!bytes) return {};
    return {util::ArenaPointer<std::uint32_t>(base,first_invalid),
        util::ArenaPointer<std::uint8_t>(base,active)};
  }
};
} // namespace tl::fea::qeph::mapped
