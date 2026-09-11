// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "AssemblyTypes.h"
#include "Result.h"
#include "../../ShellMappedNode.h"
#include "../../../solvers/NodalForceAssembly.h"

namespace tl::fea::qeph::mapped {
template<class Model>
TL_QEPH_HD inline AssemblyParent PrepareAssemblyParent(const Model& model,const ForceTrial& result,
    std::size_t parent,ShellSectionLaw law,const NodalAssemblyView& view,bool initial) noexcept {
  AssemblyParent next;
  const auto& element=model.element[parent];
  for (auto node:element.nodes) {
    if (shell_mapped_detail::ValidNode(model,view,node,initial)) continue;
    next.status=BatchStatus::InvalidInput;
    next.node=static_cast<std::uint32_t>(node);
    return next;
  }
  const bool skin=law==ShellSectionLaw::RigidSkin;
  if (!ValidResult(element.reference,result,view.position_time,view.accepted.base_epoch,skin) ||
      (!skin && !(initial?InitialStiffness(element.reference,law,next.stiffness):
          AcceptedStiffness(result,next.stiffness)))) {
    next.status=BatchStatus::NonfiniteResult;
  } else if (!skin) {
    for (unsigned slot=0;slot<4;++slot) for (unsigned prior=0;prior<slot;++prior) {
      if (element.nodes[slot]==element.nodes[prior]) next.status=BatchStatus::AssemblyFailure;
    }
  }
  return next;
}
// Same leaf accumulation as serial scatter, one incidence at a time. Existing
// destination values are the starting values; even a signed zero is retained.
// A failure is attributed to the first source parent affecting this node.
TL_QEPH_HD inline std::uint32_t GatherAssemblyNode(std::size_t node,const AssemblyMemory& memory,
    const ForceTrial* accepted,const ShellSectionLaw* law,const DeviceNodalForceView& forces,
    const double* translation,const double* rotation,AssemblyNode& output) noexcept {
  AssemblyNode next;
  const double* arrays[]{forces.force_x,forces.force_y,forces.force_z,forces.couple_x,
      forces.couple_y,forces.couple_z,translation,rotation};
  for (unsigned channel=0;channel<8;++channel) next.value[channel]=arrays[channel][node];
  DeviceNodalForceView local=forces;
  local.node_count=1;
  local.force_x=&next.value[0];local.force_y=&next.value[1];local.force_z=&next.value[2];
  local.couple_x=&next.value[3];local.couple_y=&next.value[4];local.couple_z=&next.value[5];
  const std::size_t local_node[]{0};
  for (auto index=memory.offsets[node];index<memory.offsets[node+1];++index) {
    const auto incidence=memory.incidence[index];
    const auto parent=incidence/4,slot=incidence%4;
    if (memory.parent[parent].status!=BatchStatus::Success) break;
    if (law[parent]==ShellSectionLaw::RigidSkin) continue;
    const auto& force=accepted[parent];
    const auto& stiffness=memory.parent[parent].stiffness;
    const shell_nodal_stiffness::Packet<1> packet{{stiffness.translation[slot]},{stiffness.rotation[slot]}};
    if (AccumulateNodalForces<1>(local_node,force.internal_force+slot,force.internal_couple+slot,local,-1)
            !=NodalForceAssemblyStatus::Success ||
        !shell_nodal_stiffness::Add(local_node,packet,&next.value[6],&next.value[7],1)) return parent;
    next.touched=true;
  }
  output=next;
  return UINT32_MAX;
}
TL_QEPH_HD inline void PublishAssemblyNode(std::size_t node,const AssemblyNode& value,
    DeviceNodalForceView forces,double* translation,double* rotation) noexcept {
  if (!value.touched) return;
  double* arrays[]{forces.force_x,forces.force_y,forces.force_z,forces.couple_x,
      forces.couple_y,forces.couple_z,translation,rotation};
  for (unsigned channel=0;channel<8;++channel) arrays[channel][node]=value.value[channel];
}
} // namespace tl::fea::qeph::mapped
