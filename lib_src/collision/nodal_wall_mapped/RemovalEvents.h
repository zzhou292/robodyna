// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Layout.h"
#include <type_traits>

namespace tlfea::contact::nodal_wall_mapped::removal_events {
constexpr unsigned Threads = 256;
constexpr unsigned WarpSize = 32;
constexpr unsigned Words = Threads / WarpSize;
static_assert(sizeof(unsigned) == 4 && Threads % WarpSize == 0, "Complete 32-lane words");
static_assert(MaxVehicleNodalWallDeviceParents <= UINT32_MAX - Threads,
    "Every admitted final tile increment fits unsigned");
enum class Event : unsigned { None, Invalid, Removed };

// Private within-launch integer schedule. Every word is overwritten each tile;
// no potential, activity verdict or prior-attempt result is retained here.
struct Word {
  unsigned invalid;
  unsigned removed;
};
struct Tile {
  Word words[Words];
};
static_assert(std::is_trivial<Tile>::value, "No shared-memory constructor");
static_assert(sizeof(Tile) == 64, "Fixed shared scratch, no arena growth");

TL_SURFACE_HD inline Event Classify(std::uint8_t accepted, std::uint8_t proposed) {
  if (accepted > 1 || proposed > accepted) return Event::Invalid;
  return accepted && !proposed ? Event::Removed : Event::None;
}

TL_SURFACE_HD inline unsigned First(unsigned nonzero) {
#ifdef __CUDA_ARCH__
  return static_cast<unsigned>(__ffs(static_cast<int>(nonzero)) - 1);
#else
  return static_cast<unsigned>(__builtin_ctz(nonzero));
#endif
}

// The lane reaching an event alone reads its potential. In particular, an
// invalid event and every nonremoved parent leave that payload unconsumed.
// Sum is the original certified, ordered operation, including zero operands.
TL_SURFACE_HD inline bool Consume(nodal_wall_device_detail::Storage& storage,
    Sidecar side, unsigned parent, Event event) {
  namespace d = nodal_wall_device_detail;
  if (event == Event::Invalid)
    return d::Fail(storage.control, NodalWallDeviceStatus::InvalidInput, UINT32_MAX, parent);
  if (event == Event::Removed && !nodal_wall_reduction::Sum(
      side.summary->removed_potential, storage.result.parents[parent].potential))
    return d::Fail(storage.control, NodalWallDeviceStatus::NonFiniteArithmetic, UINT32_MAX, parent);
  return true;
}

TL_SURFACE_HD inline bool FoldWord(nodal_wall_device_detail::Storage& storage,
    Sidecar side, unsigned first, Word word) {
  unsigned events = word.invalid | word.removed;
  while (events) {
    const unsigned lane = First(events);
    const auto event = word.invalid & (1u << lane) ? Event::Invalid : Event::Removed;
    if (!Consume(storage, side, first + lane, event)) return false;
    events &= events - 1;
  }
  return true;
}
} // namespace tlfea::contact::nodal_wall_mapped::removal_events
