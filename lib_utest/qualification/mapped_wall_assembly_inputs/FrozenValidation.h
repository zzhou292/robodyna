// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/nodal_wall_mapped/Layout.h"
namespace wall_assembly_frozen {
namespace d = tlfea::contact::nodal_wall_device_detail;
namespace fe = tl::fea;
using namespace tlfea::contact;
using nodal_wall_mapped::Sidecar;
using Code = NodalWallDeviceStatus;
TL_SURFACE_HD inline bool ValidateAssembly(d::Storage& storage,Sidecar side,const fe::NodalAssemblyView& view) {
  if(view.result->base_epoch!=view.accepted.base_epoch || view.result->attempt!=view.attempt ||
      view.bounds->base_epoch!=view.accepted.base_epoch || view.bounds->attempt!=view.attempt ||
      !view.bounds->initialized || !view.bounds->valid || view.bounds->sealed || view.result->status!=Status::kOk)
    return d::Fail(storage.control,Code::AssemblyFailure);
  for(unsigned i=0;i<storage.model.node_count;++i) {
    const auto node=storage.model.nodes[i].node;
    const auto inverse=view.mass.inverse_mass[node];
    if(view.mass.fixed[node] || view.translation_fixed_bits[node] || !IsFinite(inverse) ||
        inverse<0 || (side.roots[i]==UINT32_MAX && inverse<=0))
      return d::Fail(storage.control,Code::InvalidMass,node);
    side.inverse[i]=inverse;
    if(view.accepted.base_epoch==0) {
      const auto expected=storage.model.initial_position[node];
      const auto* x=view.accepted.position_xyz+3*node;
      if(x[0]!=expected.x || x[1]!=expected.y || x[2]!=expected.z)
        return d::Fail(storage.control,Code::GeometryFailure,node);
    }
  }
  return true;
}
} // namespace wall_assembly_frozen
