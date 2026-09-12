// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/elements/qbat/mapped/Stiffness.h"
#include "lib_src/elements/qbat/QbatBatchStorage.h"
#include "lib_src/elements/qbat/QbatBatchResultChecks.h"
#include "lib_src/elements/ShellBatchFields.h"
#include "lib_src/solvers/NodalForceAssembly.h"

namespace qbat_gather_test::serial {
using namespace tl::fea;
using namespace tl::fea::qbat;
using namespace tl::fea::qbat::batch_detail;
namespace {
__device__ bool MappedNode(const Model& model,const NodalAssemblyView& view,std::size_t node,bool initial) {
  if (node>=model.config.owner.node_count || view.mass.fixed[node] ||
      view.translation_fixed_bits[node] || view.rotation_fixed[node] ||
      (view.rotation_present && view.rotation_present[node]!=1)) return false;
  // Constrained raw/inverse M/J is current owner state, not a fixed source
  // coefficient test. Dependent and PART inverses may authentically be zero.
  const double inverse_mass=view.mass.inverse_mass[node];
  const double inverse_inertia=view.inverse_inertia[node];
  if (!tl::math::Finite(inverse_mass) || inverse_mass<0 ||
      !tl::math::Finite(inverse_inertia) || inverse_inertia<0) return false;
  const auto position=shell_batch_fields::ReadVector(view.accepted.position_xyz,node);
  const auto velocity=shell_batch_fields::ReadVector(view.accepted.velocity_xyz,node);
  const auto omega=shell_batch_fields::ReadVector(view.accepted.angular_velocity_xyz,node);
  const auto* q=view.accepted.orientation_wxyz+4*node;
  return detail::Finite(position) && detail::Finite(velocity) && detail::Finite(omega) &&
      tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]}) &&
      (!initial || shell_startup_detail::MatchesInitialNode(model.config.startup,position,
          model.initial_position[node],velocity,omega,q));
}
__global__ void AssembleMapped(Storage* storage,const Slab* accepted,NodalAssemblyView view,
    NodalCinAssemblyView cin,bool initial) {
  auto& state=*storage;
  state.control={};
  const auto count=state.model.config.element_count;
  if (!state.model.mapped || view.result->base_epoch!=view.accepted.base_epoch ||
      view.result->attempt!=view.attempt || view.bounds->base_epoch!=view.accepted.base_epoch ||
      view.bounds->attempt!=view.attempt || !view.bounds->initialized || !view.bounds->valid ||
      view.bounds->sealed || view.result->status!=tlfea::contact::Status::kOk) {
    state.control.status=BatchStatus::AssemblyFailure;
  }
  for (std::size_t parent=0;parent<count && state.control.status==BatchStatus::Success;++parent) {
    const auto& element=state.model.element[parent];
    const auto& result=accepted->element[parent];
    for (auto node:element.nodes) {
      if (MappedNode(state.model,view,node,initial)) continue;
      state.control.status=BatchStatus::InvalidInput;
      state.control.node=static_cast<std::uint32_t>(node);
      break;
    }
    if (state.control.status!=BatchStatus::Success) break;
    mapped::NodalStiffness stiffness;
    if (!ValidResult(result,element.material,view.position_time,view.accepted.base_epoch) ||
        !mapped::AcceptedStiffness(element,result,stiffness)) {
      state.control.status=BatchStatus::NonfiniteResult;
    } else if (AccumulateNodalForces<4>(element.nodes,result.internal_force_n,
        result.internal_couple_nm,view.forces,-1)!=NodalForceAssemblyStatus::Success ||
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
} // namespace qbat_gather_test::serial
