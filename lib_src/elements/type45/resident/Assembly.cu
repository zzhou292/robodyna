// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Arena.h"
#include "ResultChecks.h"
#include "../../ShellBatchFields.h"
#include "../../../solvers/NodalForceAssembly.h"

namespace tl::fea::type45::resident_detail {
namespace {
__device__ bool CheckEndpoints(Storage& s,unsigned slab,const NodalAssemblyView& v) {
  for(std::size_t j=0;j<s.count;++j) {
    const auto& joint=s.joints[j];
    if(!ValidResult(joint,s.slab[slab][j],v.position_time,v.accepted.base_epoch,s.config.owner.fixed_dt)) {
      s.control.status=BatchStatus::NonfiniteResult;s.control.joint=j;return false;
    }
    for(unsigned e=0;e<2;++e) {
      const auto n=joint.domain_nodes[e];
      if(n>=v.accepted.node_count || v.mass.fixed[n] || v.translation_fixed_bits[n] ||
          v.rotation_fixed[n] || (v.rotation_present && v.rotation_present[n]!=1) ||
          !detail::Nonnegative(v.mass.inverse_mass[n]) || !detail::Nonnegative(v.inverse_inertia[n])) {
        s.control.status=BatchStatus::InvalidInput;s.control.joint=j;s.control.node=n;return false;
      }
      const auto x=shell_batch_fields::ReadVector(v.accepted.position_xyz,n);
      const auto velocity=shell_batch_fields::ReadVector(v.accepted.velocity_xyz,n);
      const auto omega=shell_batch_fields::ReadVector(v.accepted.angular_velocity_xyz,n);
      const auto* q=v.accepted.orientation_wxyz+4*n;
      if(!detail::Finite(x) || !detail::Finite(velocity) || !detail::Finite(omega) ||
          !tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]}) ||
          (!v.accepted.base_epoch && !shell_startup_detail::MatchesInitialNode(
              s.config.startup,x,joint.geometry.position_m[e],velocity,omega,q))) {
        s.control.status=BatchStatus::InvalidInput;s.control.joint=j;s.control.node=n;return false;
      }
    }
  }
  return true;
}
__device__ bool Add(const Joint& joint,const Cache& cache,NodalAssemblyView v,NodalCinAssemblyView cin) {
  double kn[2]{},kr[2]{};
  Vec3 force[2],couple[2];std::size_t nodes[2];
  for(unsigned e=0;e<2;++e) {
    const auto n=joint.domain_nodes[e];const auto& value=cache.endpoint[e];
    nodes[e]=n;force[e]=value.force_n;couple[e]=value.couple_nm;
    kn[e]=cin.translational_stiffness[n]+value.translational_stiffness_n_m;
    kr[e]=cin.rotational_stiffness[n]+value.rotational_stiffness_nm;
    if(!detail::Nonnegative(cin.translational_stiffness[n]) ||
        !detail::Nonnegative(cin.rotational_stiffness[n]) || !detail::Nonnegative(kn[e]) ||
        !detail::Nonnegative(kr[e])) return false;
  }
  if(AccumulateNodalForces<2>(nodes,force,couple,v.forces,+1)!=NodalForceAssemblyStatus::Success) return false;
  for(unsigned e=0;e<2;++e) {
    cin.translational_stiffness[nodes[e]]=kn[e];cin.rotational_stiffness[nodes[e]]=kr[e];
  }
  return true;
}
__global__ void Assemble(Storage* s,unsigned accepted,NodalAssemblyView v,NodalCinAssemblyView cin) {
  s->control={};
  if(v.result->base_epoch!=v.accepted.base_epoch || v.result->attempt!=v.attempt ||
      v.bounds->base_epoch!=v.accepted.base_epoch || v.bounds->attempt!=v.attempt ||
      !v.bounds->initialized || !v.bounds->valid || v.bounds->sealed ||
      v.result->status!=tlfea::contact::Status::kOk) s->control.status=BatchStatus::AssemblyFailure;
  else if(CheckEndpoints(*s,accepted,v)) {
    // Model/source joint order, then endpoint order. Failure invalidates the
    // entire owner's disposable trial; no accepted cache has been changed.
    for(std::size_t j=0;j<s->count;++j) if(!Add(s->joints[j],s->slab[accepted][j].cache,v,cin)) {
      s->control.status=BatchStatus::AssemblyFailure;s->control.joint=j;break;
    }
  }
  if(s->control.status!=BatchStatus::Success)
    RecordNodalAssemblyFailure(v,tlfea::contact::Status::kInvalidArgument,
        static_cast<std::uint32_t>(s->control.node));
}
}
void LaunchAssembly(Storage* s,unsigned accepted,NodalAssemblyView v,NodalCinAssemblyView cin) {
  Assemble<<<1,1,0,v.stream>>>(s,accepted,v,cin);
}
} // namespace tl::fea::type45::resident_detail
