// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "AssemblyTypes.h"
#include "Stiffness.h"
#include "../QbatBatchResultChecks.h"
#include "../../ShellMappedNode.h"
namespace tl::fea::qbat::mapped {
TL_QBAT_HD inline AssemblyParent PrepareAssemblyParent(const batch_detail::Model& model,
    const BatchResult& result,std::size_t parent,const NodalAssemblyView& view,bool initial) noexcept {
  AssemblyParent next;
  const auto& element=model.element[parent];
  for(auto node:element.nodes) {
    if(shell_mapped_detail::ValidNode(model,view,node,initial)) continue;
    next.status=BatchStatus::InvalidInput;
    next.node=static_cast<std::uint32_t>(node);
    return next;
  }
  if(!batch_detail::ValidResult(result,element.material,view.position_time,view.accepted.base_epoch) ||
      !AcceptedStiffness(element,result,next.stiffness)) {
    next.status=BatchStatus::NonfiniteResult;
  } else {
    for(unsigned slot=0;slot<4;++slot) for(unsigned prior=0;prior<slot;++prior)
      if(element.nodes[slot]==element.nodes[prior]) next.status=BatchStatus::AssemblyFailure;
  }
  return next;
}
TL_QBAT_HD inline std::uint32_t GatherAssemblyNode(std::size_t node,const AssemblyMemory& memory,
    const BatchResult* accepted,const DeviceNodalForceView& forces,const double* translation,
    const double* rotation,AssemblyNode& output) noexcept {
  return mapped_shell::GatherNodeValues<4,AssemblyMemory,BatchStatus>(node,memory,
      ForceAccess{accepted},forces,translation,rotation,output);
}
} // namespace tl::fea::qbat::mapped
