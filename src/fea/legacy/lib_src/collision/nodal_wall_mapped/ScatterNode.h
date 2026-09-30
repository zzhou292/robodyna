// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Layout.h"
#include "../NodalWallContactScatter.h"
#include "lib_src/solvers/NodalCinRuntime.h"

namespace tlfea::contact::nodal_wall_mapped {
namespace d=nodal_wall_device_detail;
namespace fe=tl::fea;
TL_SURFACE_HD inline bool StageStiffnessNode(d::Storage& storage,Sidecar side,
    const fe::NodalCinAssemblyView& cin,unsigned compact,d::Control& status) {
  const auto node=storage.model.nodes[compact].node;
  const double before=cin.translational_stiffness[node];
  const double value=before+storage.result.nodes[compact].stiffness.value;
  if(!IsFinite(before) || before<0 || !IsFinite(value) || value<0)
    return d::Fail(status,NodalWallDeviceStatus::AssemblyFailure,node);
  side.stiffness[compact]=value;
  return true;
}
// A kernel boundary completes every status writer before this source-order scan.
TL_SURFACE_HD inline void CheckScatterNodes(d::Storage& storage) {
  if(storage.control.status!=NodalWallDeviceStatus::Ok) return;
  for(unsigned i=0;i<storage.model.node_count;++i) {
    if(storage.node_status[i].status!=NodalWallDeviceStatus::Ok) {
      storage.control=storage.node_status[i];
      return;
    }
  }
}
} // namespace tlfea::contact::nodal_wall_mapped
