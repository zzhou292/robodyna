#pragma once
#include "NodalWallContactDiagnostics.h"
#include "NodalWallContactScatter.h"
#include "NodalWallContactEvaluation.cuh"
#include "lib_src/solvers/NodalForceAssembly.h"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"

namespace tlfea::contact::nodal_wall_device_detail {
namespace fea=tl::fea;
// Avoid whole-result temporaries on one CUDA lane as the collection grows.
// All worker lanes call these helpers; arithmetic fields retain their owning
// default initialization and each record has exactly one writer.
__device__ inline void ResetResult(ActiveResults& result,unsigned parents,unsigned nodes,
    unsigned lane=threadIdx.x,unsigned stride=Workers) {
  if (lane==0) result.diagnostics={};
  for (unsigned p=lane;p<parents;p+=stride) result.parents[p]={};
  for (unsigned n=lane;n<nodes;n+=stride) {
    result.nodes[n]={}; result.wall_face[n]=0;
  }
}
__device__ inline void CopyBase(Storage& s,unsigned lane=threadIdx.x,unsigned stride=Workers) {
  if (lane==0) s.base.diagnostics=s.result.diagnostics;
  for (unsigned p=lane;p<s.model.parent_count;p+=stride) s.base.parents[p]=s.result.parents[p];
  for (unsigned n=lane;n<s.model.node_count;n+=stride) {
    s.base.nodes[n]=s.result.nodes[n]; s.base.wall_face[n]=s.result.wall_face[n];
  }
}
__device__ inline bool ValidateAssembly(Storage& s,const fea::NodalAssemblyView& v) {
  if (v.result->base_epoch!=v.accepted.base_epoch || v.result->attempt!=v.attempt ||
      v.bounds->base_epoch!=v.accepted.base_epoch || v.bounds->attempt!=v.attempt ||
      !v.bounds->initialized || !v.bounds->valid || v.bounds->sealed || v.result->status!=Status::kOk)
    return Fail(s.control,Code::AssemblyFailure);
  if (!fea::native_physical_coefficients::Admitted(s.model.config.owner.rigid_groups,v.rigid_groups,
      v.mass.model,s.model.config.owner.node_count)) return Fail(s.control,Code::InvalidMass);
  for (unsigned i=0;i<s.model.node_count;++i) {
    const auto n=s.model.nodes[i].node;
    if (v.mass.inverse_mass[n]!=s.model.inverse_mass[n] || v.mass.fixed[n]!=s.model.fixed[n] ||
        v.translation_fixed_bits[n]!=(s.model.fixed[n]?7:0)) return Fail(s.control,Code::InvalidMass,n);
    if (v.accepted.base_epoch==0) {
      const auto initial=s.model.initial_position[n]; const auto* x=v.accepted.position_xyz+3*n;
      if (x[0]!=initial.x || x[1]!=initial.y || x[2]!=initial.z) return Fail(s.control,Code::GeometryFailure,n);
    }
  }
  return true;
}
// All lanes enter each barrier. Compact slots are selected through immutable
// native shares; no parent-count constants or source fixture arrays are copied.
template<bool Physical=false>
__device__ inline void Evaluate(Storage& s,const fea::DeviceNodalKinematicsView& k,
                               const NodalWallDiagnostics& identity,const std::uint8_t* activity=nullptr) {
  const auto lane=threadIdx.x;
  ResetResult(s.result,s.model.parent_count,s.model.node_count);
  if (lane==0) s.result.diagnostics=identity;
  for (unsigned i=lane;i<s.model.node_count;i+=Workers) s.node_status[i]={};
  __syncthreads();
  if (s.control.status!=Code::Ok) return;
  for (unsigned compact=lane;compact<s.model.node_count;compact+=Workers)
    EvaluatePoint<Physical>(s,k,identity,compact,activity);
  __syncthreads();
  if (lane==0) {
    for (unsigned i=0;i<s.model.node_count;++i) if (s.node_status[i].status!=Code::Ok) {
      s.control=s.node_status[i]; break;
    }
    if (s.control.status==Code::Ok) for (unsigned p=0;p<s.model.parent_count;++p)
      if (!ReduceParent(s,p,s.control)) break;
    ReduceNodes<Physical>(s,k);
  }
  __syncthreads();
}
__device__ inline bool Scatter(Storage& s,const fea::NodalAssemblyView& v,bool publish=true) {
  for(unsigned c=0;c<6;++c) for(unsigned i=0;i<s.model.node_count;++i)
    CopyScatterChannel(s,v,i,c);
  const auto staged=StagedForces(s,v);
  for(unsigned i=0;i<s.model.node_count;++i)
    if(!AccumulateScatterNode(s,v,staged,i,s.control)) return false;
  if(publish) for(unsigned c=0;c<6;++c) for(unsigned i=0;i<s.model.node_count;++i)
    PublishScatterChannel(s,v,i,c);
  return true; // Legacy timestep rows are deliberately untouched.
}
// Infallible copy after mapped force AND stiffness destinations passed.
__device__ inline void PublishScatter(Storage& s,const fea::NodalAssemblyView& view) {
  for(unsigned c=0;c<6;++c) for(unsigned i=0;i<s.model.node_count;++i)
    PublishScatterChannel(s,view,i,c);
}
} // namespace tlfea::contact::nodal_wall_device_detail
