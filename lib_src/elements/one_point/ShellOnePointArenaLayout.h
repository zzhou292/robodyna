#pragma once
#include "../ShellBatchOnePointSection.h"
#include "../ShellResidentLimits.h"
#include "lib_utils/BoundedArena.h"

namespace tl::fea::shell_batch_plasticity_detail {
// Optional T3 storage. Both arrays follow the existing T3 accepted slab index.
struct OnePointDeviceStorage {
  ShellBatchOnePointSectionState* section[2]{};
};
struct OnePointLayout {
  util::ArenaRegion header, section[2];
  std::size_t bytes = 0;
  bool Initialize(std::size_t count, std::size_t cap) noexcept {
    if (!count || count > MaxVehicleShellResidentParents) return false;
    OnePointLayout next;
    util::BoundedArenaLayout arena(cap);
    if (!arena.Append<OnePointDeviceStorage>(1, next.header)) return false;
    for (auto& region : next.section) {
      if (!arena.Append<ShellBatchOnePointSectionState>(count, region)) return false;
    }
    next.bytes = arena.bytes();
    *this = next;
    return true;
  }
  OnePointDeviceStorage* Construct(util::HostArena& arena) const noexcept {
    auto* out = arena.Construct<OnePointDeviceStorage>(header);
    if (!out) return nullptr;
    for (unsigned slab = 0; slab < 2; ++slab) {
      out->section[slab] = arena.Construct<ShellBatchOnePointSectionState>(section[slab]);
      if (!out->section[slab]) return nullptr;
    }
    return out;
  }
  OnePointDeviceStorage Rebase(void* device) const noexcept {
    OnePointDeviceStorage out;
    for (unsigned slab = 0; slab < 2; ++slab) {
      out.section[slab] = util::ArenaPointer<ShellBatchOnePointSectionState>(device, section[slab]);
    }
    return out;
  }
};
} // namespace tl::fea::shell_batch_plasticity_detail
