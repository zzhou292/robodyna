// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
#include "../NodalWallContactKernels.cuh"
namespace tlfea::contact::nodal_wall_mapped {
__device__ inline bool ValidateAssembly(d::Storage& storage,Sidecar side,const fe::NodalAssemblyView& view) {
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
__device__ inline bool Response(d::Storage& storage,Sidecar side,const fe::DeviceNodalKinematicsView& k) {
  side.summary->rate=0;
  for(std::size_t g=0;g<side.groups;++g) side.traces[g]=0;
  for(unsigned i=0;i<storage.model.node_count;++i) {
    const auto& node=storage.result.nodes[i];
    const auto upper=::fmax(node.stiffness.upper,node.stiffness.value);
    if(upper==0) continue;
    const auto root=side.roots[i];
    if(root==UINT32_MAX) {
      double rate=0;
      if(!mass_detail::UpperProduct(upper,side.inverse[i],&rate))
        return d::Fail(storage.control,Code::NonFiniteArithmetic,node.node);
      side.summary->rate=::fmax(side.summary->rate,rate);
    } else {
      if(root>=side.groups) return d::Fail(storage.control,Code::InvalidInput,node.node);
      const auto* x=k.position_xyz+3*node.node;
      RigidNormalResponse response;
      if(EvaluateRigidNormalResponse(side.bodies[root],{x[0],x[1],x[2]},{-1,0,0},response)!=Status::kOk ||
          AccumulateRigidContactTrace(upper,response,side.traces[root])!=Status::kOk)
        return d::Fail(storage.control,Code::NonFiniteArithmetic,node.node);
    }
  }
  for(std::size_t g=0;g<side.groups;++g) side.summary->rate=::fmax(side.summary->rate,side.traces[g]);
  storage.result.diagnostics.stiffness_rate_bound=side.summary->rate;
  return true;
}
__device__ inline bool StageStiffness(d::Storage& storage,Sidecar side,const fe::NodalCinAssemblyView& cin) {
  for(unsigned i=0;i<storage.model.node_count;++i) {
    const auto node=storage.model.nodes[i].node;
    const double before=cin.translational_stiffness[node];
    const double value=before+storage.result.nodes[i].stiffness.value;
    if(!IsFinite(before) || before<0 || !IsFinite(value) || value<0)
      return d::Fail(storage.control,Code::AssemblyFailure,node);
    side.stiffness[i]=value;
  }
  return true;
}
__device__ inline bool RemovedPotential(d::Storage& storage,Sidecar side) {
  side.summary->removed_potential={};
  for(unsigned p=0;p<storage.model.parent_count;++p) {
    if(side.accepted[p]>1 || side.proposed[p]>side.accepted[p])
      return d::Fail(storage.control,Code::InvalidInput,UINT32_MAX,p);
    if(side.accepted[p] && !side.proposed[p] &&
        !nodal_wall_reduction::Sum(side.summary->removed_potential,storage.result.parents[p].potential))
      return d::Fail(storage.control,Code::NonFiniteArithmetic,UINT32_MAX,p);
  }
  return true;
}
} // namespace tlfea::contact::nodal_wall_mapped
