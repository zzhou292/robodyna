// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/elements/t3/mapped/Stiffness.h"
#include "lib_src/elements/t3/mapped/Result.h"
#include "lib_src/elements/t3/T3BatchStorage.h"
#include "lib_src/elements/ShellMixedSectionArenaLayout.h"
#include "lib_src/elements/ShellMappedNode.h"
#include "lib_src/solvers/NodalForceAssembly.h"
#include "lib_src/solvers/NodalCinRuntime.h"

namespace t3_gather_test {
using namespace tl::fea;
using namespace tl::fea::t3;
using namespace tl::fea::t3::batch_detail;
namespace {
__global__ void AssembleMapped(Storage* storage,const Slab* accepted,NodalAssemblyView view,
    NodalCinAssemblyView cin,const shell_batch_plasticity_detail::MixedDeviceStorage* mixed,bool initial) {
  auto& state=*storage;
  state.control={};
  if (!state.model.mapped || !mixed || view.result->base_epoch!=view.accepted.base_epoch ||
      view.result->attempt!=view.attempt || view.bounds->base_epoch!=view.accepted.base_epoch ||
      view.bounds->attempt!=view.attempt || !view.bounds->initialized || !view.bounds->valid ||
      view.bounds->sealed || view.result->status!=tlfea::contact::Status::kOk) {
    state.control.status=BatchStatus::AssemblyFailure;
  }
  for (std::size_t parent=0;parent<state.model.config.element_count &&
      state.control.status==BatchStatus::Success;++parent) {
    const auto& element=state.model.element[parent];
    const auto& result=accepted->element[parent];
    for (auto node:element.nodes) {
      if (shell_mapped_detail::ValidNode(state.model,view,node,initial)) continue;
      state.control.status=BatchStatus::InvalidInput;
      state.control.node=static_cast<std::uint32_t>(node);
      break;
    }
    if (state.control.status!=BatchStatus::Success) break;
    const auto law=mixed->law[parent];
    const bool skin=law==ShellSectionLaw::RigidSkin;
    mapped::NodalStiffness stiffness;
    if (!mapped::ValidResult(element.reference,result,view.position_time,view.accepted.base_epoch,skin) ||
        (!skin && !(initial ? mapped::InitialStiffness(element.reference,law,stiffness) :
            mapped::AcceptedStiffness(result,stiffness)))) {
      state.control.status=BatchStatus::NonfiniteResult;
    } else if (!skin && (AccumulateNodalForces<3>(element.nodes,result.internal_force,
        result.internal_couple,view.forces,-1)!=NodalForceAssemblyStatus::Success ||
        !shell_nodal_stiffness::Add(element.nodes,stiffness,cin.translational_stiffness,
            cin.rotational_stiffness,cin.node_count))) {
      state.control.status=BatchStatus::AssemblyFailure;
    }
    if (state.control.status!=BatchStatus::Success) state.control.element=static_cast<std::uint32_t>(parent);
  }
  if (state.control.status!=BatchStatus::Success) {
    RecordNodalAssemblyFailure(view,tlfea::contact::Status::kInvalidArgument,state.control.node);
  }
}
} // namespace
void LaunchSerialAssembly(Storage* storage,const Slab* accepted,NodalAssemblyView view,
    NodalCinAssemblyView cin,const shell_batch_plasticity_detail::MixedDeviceStorage* mixed,bool initial) {
  AssembleMapped<<<1,1,0,view.stream>>>(storage,accepted,view,cin,mixed,initial);
}
} // namespace t3_gather_test
