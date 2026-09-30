// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Layout.h"

namespace tlfea::contact::nodal_wall_mapped {
// Each producer writes its complete Control before selecting the lowest compact
// index. An ordered kernel boundary completes all writers before this read.
// Compact order is the original scan order; global NIDs need not be monotonic.
TL_SURFACE_HD inline void CopyNodeFailure(nodal_wall_device_detail::Storage& storage,
    unsigned long long first) {
  if (first == ~0ull) return;
  if (first < storage.model.node_count &&
      storage.node_status[first].status != NodalWallDeviceStatus::Ok) {
    storage.control = storage.node_status[first];
    return;
  }
  // A malformed key falls back to the complete source-order scan so existing
  // row failures cannot be skipped, including their original point payloads.
  for (unsigned i = 0; i < storage.model.node_count; ++i) {
    if (storage.node_status[i].status != NodalWallDeviceStatus::Ok) {
      storage.control = storage.node_status[i];
      return;
    }
  }
}
} // namespace tlfea::contact::nodal_wall_mapped
