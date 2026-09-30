// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "RemovalEvents.h"

namespace tlfea::contact::nodal_wall_mapped::removal_events {
// Exactly one complete block participates, including the partial final tile.
// Only integer events are combined. The original parent-order floating fold
// and first failure remain on lane zero; later packets never publish status.
__device__ inline bool RemovedPotential(nodal_wall_device_detail::Storage& storage,
    Sidecar side) {
  if (storage.control.status != NodalWallDeviceStatus::Ok) return false;
  __shared__ Tile tile;
  if (!threadIdx.x) side.summary->removed_potential = {};
  for (unsigned first = 0; first < storage.model.parent_count; first += Threads) {
    const unsigned parent = first + threadIdx.x;
    Event event = Event::None;
    if (parent < storage.model.parent_count) {
      const auto accepted = side.accepted[parent];
      event = accepted > 1 ? Event::Invalid : Classify(accepted, side.proposed[parent]);
    }
    const unsigned invalid = __ballot_sync(0xffffffffu, event == Event::Invalid);
    const unsigned removed = __ballot_sync(0xffffffffu, event == Event::Removed);
    if (threadIdx.x % WarpSize == 0)
      tile.words[threadIdx.x / WarpSize] = {invalid, removed};
    __syncthreads();
    if (!threadIdx.x) {
      for (unsigned word = 0; word < Words; ++word)
        if (!FoldWord(storage, side, first + word * WarpSize, tile.words[word])) break;
    }
    __syncthreads();
    if (storage.control.status != NodalWallDeviceStatus::Ok) return false;
  }
  return true;
}
} // namespace tlfea::contact::nodal_wall_mapped::removal_events
