// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalWallContactStorage.h"
#include "lib_src/solvers/NodalForceAssembly.h"

namespace tlfea::contact::nodal_wall_device_detail {
namespace fea=tl::fea;
// The immutable compact-node map has unique global destinations. Callers own
// the schedule: legacy uses channel-first copies; mapped stages independent nodes.
TL_SURFACE_HD inline double* ForceChannel(const fea::NodalAssemblyView& view,unsigned channel) {
  double* actual[]{view.forces.force_x,view.forces.force_y,view.forces.force_z,
      view.forces.couple_x,view.forces.couple_y,view.forces.couple_z};
  return actual[channel];
}
TL_SURFACE_HD inline fea::DeviceNodalForceView StagedForces(Storage& storage,
    const fea::NodalAssemblyView& view) {
  const auto nodes=storage.model.config.owner.node_count;
  return {storage.staged_force,storage.staged_force+nodes,storage.staged_force+2*nodes,
      storage.staged_force+3*nodes,storage.staged_force+4*nodes,storage.staged_force+5*nodes,
      view.forces.node_count,view.forces.base_epoch};
}
TL_SURFACE_HD inline void CopyScatterChannel(Storage& storage,const fea::NodalAssemblyView& view,
    unsigned compact,unsigned channel) {
  const auto node=storage.model.nodes[compact].node;
  storage.staged_force[channel*storage.model.config.owner.node_count+node]=ForceChannel(view,channel)[node];
}
TL_SURFACE_HD inline bool AccumulateScatterNode(Storage& storage,const fea::NodalAssemblyView& view,
    const fea::DeviceNodalForceView& staged,unsigned compact,Control& status) {
  const auto node=storage.model.nodes[compact].node;
  const std::size_t index=node;
  const tl::math::Vec3 force{storage.result.nodes[compact].force_world.x,0,0},couple{};
  const double before=view.forces.force_x[node];
  Q4IntegralInterval sum;
  if(fea::AccumulateNodalForces<1>(&index,&force,&couple,staged)!=fea::NodalForceAssemblyStatus::Success ||
      !q4_bounds::Add({before,before},{force.x,force.x},&sum) ||
      !Radius(storage.staged_force[node],sum,&storage.addition_error[compact]))
    return Fail(status,Code::AssemblyFailure,node);
  return true;
}
TL_SURFACE_HD inline void PublishScatterChannel(Storage& storage,const fea::NodalAssemblyView& view,
    unsigned compact,unsigned channel) {
  const auto node=storage.model.nodes[compact].node;
  ForceChannel(view,channel)[node]=storage.staged_force[channel*storage.model.config.owner.node_count+node];
}
} // namespace tlfea::contact::nodal_wall_device_detail
