// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Stiffness.h"
#include "../Type25BatchStorage.h"
#include "../../ShellBatchFields.h"
#include "../../../solvers/NodalForceAssembly.h"
#include "../../../solvers/NodalCinRuntime.h"

namespace tl::fea::type25::batch_detail {
namespace {
__device__ bool MappedNode(const DeviceModel& model,const NodalAssemblyView& view,std::size_t node,bool initial) {
  if (node>=model.config.owner.node_count || view.mass.fixed[node] ||
      view.translation_fixed_bits[node] || view.rotation_fixed[node] ||
      (view.rotation_present && view.rotation_present[node]!=1)) return false;
  // Constrained raw/inverse M/J is current owner state, not a fixed source
  // coefficient test. CIN-dependent inverses may authentically be zero;
  // this profile separately excludes actual rigid endpoints at initialization.
  const double inverse_mass=view.mass.inverse_mass[node];
  const double inverse_inertia=view.inverse_inertia[node];
  if (!tl::math::Finite(inverse_mass) || inverse_mass<0 ||
      !tl::math::Finite(inverse_inertia) || inverse_inertia<0) return false;
  const auto position=shell_batch_fields::ReadVector(view.accepted.position_xyz,node);
  const auto velocity=shell_batch_fields::ReadVector(view.accepted.velocity_xyz,node);
  const auto omega=shell_batch_fields::ReadVector(view.accepted.angular_velocity_xyz,node);
  const auto* q=view.accepted.orientation_wxyz+4*node;
  return tl::math::fixed3::Finite(position) && tl::math::fixed3::Finite(velocity) && tl::math::fixed3::Finite(omega) &&
      tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]}) &&
      (!initial || shell_startup_detail::MatchesInitialFreePhysicalNode(model.config.startup,position,
          model.nodes[node].reference,velocity,omega,q));
}
__global__ void AssembleMapped(Storage* storage,const Slab* accepted,NodalAssemblyView view,
    NodalCinAssemblyView cin,bool initial) {
  auto& state=*storage;
  state.control={};
  const auto count=state.model.config.element_count;
  if (view.result->base_epoch!=view.accepted.base_epoch ||
      view.result->attempt!=view.attempt || view.bounds->base_epoch!=view.accepted.base_epoch ||
      view.bounds->attempt!=view.attempt || !view.bounds->initialized || !view.bounds->valid ||
      view.bounds->sealed || view.result->status!=tlfea::contact::Status::kOk) {
    state.control.status=BatchStatus::AssemblyFailure;
  }
  for (std::size_t parent=0;parent<count && state.control.status==BatchStatus::Success;++parent) {
    const auto& element=state.model.elements[parent];
    const auto& result=accepted->element[parent];
    for (auto node:element.nodes) {
      if (MappedNode(state.model,view,node,initial)) continue;
      state.control.status=BatchStatus::InvalidInput;
      state.control.node=static_cast<std::uint32_t>(node);
      state.control.element=static_cast<std::uint32_t>(parent);
      break;
    }
    if (state.control.status!=BatchStatus::Success) break;
    mapped::NodalStiffness stiffness;
    const auto& property=state.model.properties[element.property_index];
    const auto* rhs=result.endpoints;
    const Vec3 force[2]{rhs[0].force_N,rhs[1].force_N};
    const Vec3 couple[2]{rhs[0].couple_Nm,rhs[1].couple_Nm};
    if (!mapped::AcceptedStiffness(property,result,stiffness)) {
      state.control.status=BatchStatus::NonfiniteResult;
    } else if (AccumulateNodalForces<2>(element.nodes,force,couple,view.forces,+1)
        !=NodalForceAssemblyStatus::Success ||
        !mapped::AddStiffness(element.nodes,stiffness,cin.translational_stiffness,
            cin.rotational_stiffness,cin.node_count)) {
      state.control.status=BatchStatus::AssemblyFailure;
    }
    if (state.control.status!=BatchStatus::Success) state.control.element=static_cast<std::uint32_t>(parent);
  }
  if (state.control.status!=BatchStatus::Success) {
    RecordNodalAssemblyFailure(view,tlfea::contact::Status::kInvalidArgument,state.control.node);
  }
}
} // namespace
void LaunchMappedAssembly(Storage* storage,const Slab* accepted,NodalAssemblyView view,
    NodalCinAssemblyView cin,bool initial) {
  AssembleMapped<<<1,1,0,view.stream>>>(storage,accepted,view,cin,initial);
}
} // namespace tl::fea::type25::batch_detail
