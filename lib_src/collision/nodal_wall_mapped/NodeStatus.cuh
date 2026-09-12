// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodeStatus.h"

namespace tlfea::contact::nodal_wall_mapped {
static __device__ void SelectNodeFailure(const nodal_wall_device_detail::Storage& storage,
    Sidecar side, unsigned compact) {
  if (side.summary && storage.node_status[compact].status != NodalWallDeviceStatus::Ok)
    atomicMin(&side.summary->parent_failure, static_cast<unsigned long long>(compact));
}
} // namespace tlfea::contact::nodal_wall_mapped
