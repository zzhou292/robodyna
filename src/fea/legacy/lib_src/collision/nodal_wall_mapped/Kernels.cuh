// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
#include "ScatterNode.h"
#include "AssemblyValidation.h"
#include "../NodalWallContactKernels.cuh"
namespace tlfea::contact::nodal_wall_mapped {
__device__ inline bool ValidateAssembly(d::Storage& storage,Sidecar side,const fe::NodalAssemblyView& view) {
  namespace a = assembly_validation;
  if (!a::Header(view, storage.control)) return false;
  for (unsigned i = 0; i < storage.model.node_count; ++i) {
    double inverse = 0;
    if (!a::Mass(storage, side, view, i, inverse, storage.control)) return false;
    side.inverse[i] = inverse;
    if (!a::Geometry(storage, view, i, storage.control)) return false;
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
  for(unsigned i=0;i<storage.model.node_count;++i)
    if(!StageStiffnessNode(storage,side,cin,i,storage.control)) return false;
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
