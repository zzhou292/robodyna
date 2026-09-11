// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Layout.h"
#include "../NodalWallContactKernels.cuh"
namespace tlfea::contact::nodal_wall_mapped::parallel {
namespace d=nodal_wall_device_detail;
namespace fe=tl::fea;
using Code=NodalWallDeviceStatus;
// Stream boundaries separate writers of point shares, parent certificates and
// global diagnostics. No floating atomic or change to a certificate sum.
static __global__ void Points(d::Storage* pointer,Sidecar side,fe::DeviceNodalKinematicsView k,
    NodalWallDiagnostics identity,bool reset_base) {
  auto& storage=*pointer;
  const unsigned first=blockIdx.x*blockDim.x+threadIdx.x;
  const unsigned stride=gridDim.x*blockDim.x;
  if(reset_base) d::ResetResult(storage.base,storage.model.parent_count,storage.model.node_count,first,stride);
  d::ResetResult(storage.result,storage.model.parent_count,storage.model.node_count,first,stride);
  if(first==0) storage.result.diagnostics=identity;
  for(unsigned i=first;i<storage.model.node_count;i+=stride) {
    storage.node_status[i]={};
    if(side.summary->points_admitted)
      d::EvaluatePoint<true>(storage,k,identity,i,side.accepted);
  }
}
static __global__ void CheckPoints(d::Storage* pointer,Sidecar side) {
  auto& storage=*pointer;
  if(!side.summary->points_admitted) return;
  for(unsigned i=0;i<storage.model.node_count;++i) {
    if(storage.node_status[i].status!=Code::Ok) {
      storage.control=storage.node_status[i];
      return;
    }
  }
}
static __global__ void Parents(d::Storage* pointer,Sidecar side) {
  auto& storage=*pointer;
  if(storage.control.status!=Code::Ok) return;
  const unsigned first=blockIdx.x*blockDim.x+threadIdx.x;
  const unsigned stride=gridDim.x*blockDim.x;
  for(unsigned p=first;p<storage.model.parent_count;p+=stride) {
    d::Control status;
    if(!d::ReduceParent(storage,p,status)) {
      const auto key=2ull*p+(status.status==Code::Accuracy?1ull:0ull);
      atomicMin(&side.summary->parent_failure,key);
    }
  }
}
static __global__ void Finish(d::Storage* pointer,Sidecar side,fe::DeviceNodalKinematicsView k) {
  auto& storage=*pointer;
  if(!side.summary->points_admitted) return;
  const auto failure=side.summary->parent_failure;
  if(storage.control.status==Code::Ok && failure!=~0ull)
    d::Fail(storage.control,(failure&1)?Code::Accuracy:Code::NonFiniteArithmetic,
        UINT32_MAX,static_cast<unsigned>(failure/2));
  d::ReduceNodes<true>(storage,k);
}
inline unsigned Blocks(std::size_t count) {
  return 1u+static_cast<unsigned>((count-1)/d::Workers);
}
inline void Evaluate(d::Storage* storage,Sidecar side,fe::DeviceNodalKinematicsView k,
    NodalWallDiagnostics identity,std::size_t nodes,std::size_t parents,cudaStream_t stream,bool reset_base) {
  const auto largest=nodes>parents?nodes:parents;
  Points<<<Blocks(largest),d::Workers,0,stream>>>(storage,side,k,identity,reset_base);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  CheckPoints<<<1,1,0,stream>>>(storage,side);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  Parents<<<Blocks(parents),d::Workers,0,stream>>>(storage,side);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  Finish<<<1,1,0,stream>>>(storage,side,k);
}
} // namespace tlfea::contact::nodal_wall_mapped::parallel
