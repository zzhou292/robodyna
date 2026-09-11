// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Layout.h"
#include "ObserverValues.h"
#include "../NodalWallContactEvaluation.cuh"
namespace tlfea::contact::nodal_wall_mapped::parallel {
namespace observer_detail {
namespace d=nodal_wall_device_detail;
static __device__ void MergeBlock(ObserverSummary* values) {
  for(unsigned offset=ObserverThreads/2;offset;offset/=2) {
    __syncthreads();
    if(threadIdx.x<offset)MergeObservers(values[threadIdx.x],values[threadIdx.x+offset]);
  }
  __syncthreads();
}
static __global__ void CheckParents(d::Storage* pointer,Sidecar side) {
  if(!side.summary->points_admitted)return;
  const auto failure=side.summary->parent_failure;
  if(pointer->control.status==NodalWallDeviceStatus::Ok && failure!=~0ull)
    d::Fail(pointer->control,(failure&1)?NodalWallDeviceStatus::Accuracy:NodalWallDeviceStatus::NonFiniteArithmetic,
        UINT32_MAX,static_cast<unsigned>(failure/2));
}
static __global__ void Observe(d::Storage* pointer,Sidecar side,
    tl::fea::DeviceNodalKinematicsView k,ObserverScratch scratch) {
  auto& storage=*pointer;
  if(!side.summary->points_admitted || storage.control.status!=NodalWallDeviceStatus::Ok)return;
  const auto blocks=ObserverBlocks(storage.model.node_count);
  if(blockIdx.x>=blocks)return;
  __shared__ ObserverSummary values[ObserverThreads];
  ObserverSummary local{};
  const unsigned first=blockIdx.x*blockDim.x+threadIdx.x,stride=blocks*blockDim.x;
  const VectorView x{k.position_xyz,static_cast<std::uint32_t>(k.node_count),3,1};
  for(unsigned i=first;i<storage.model.node_count;i+=stride) {
    const auto& node=storage.result.nodes[i];
    ObserveNode(node,x.at(node.node).x-storage.model.config.law.wall_x,local);
  }
  values[threadIdx.x]=local;MergeBlock(values);
  if(!threadIdx.x)scratch.data[blockIdx.x]=values[0];
}
static __global__ void Finish(d::Storage* pointer,Sidecar side,
    tl::fea::DeviceNodalKinematicsView k,ObserverScratch scratch) {
  auto& storage=*pointer;
  if(!side.summary->points_admitted)return;
  // The old call still publishes counts/rate on a prior point/parent error.
  if(storage.control.status!=NodalWallDeviceStatus::Ok) {
    if(!threadIdx.x)d::ReduceNodes<true>(storage,k);
    return;
  }
  const auto blocks=ObserverBlocks(storage.model.node_count);
  __shared__ ObserverSummary values[ObserverThreads];
  ObserverSummary local{};
  for(unsigned i=threadIdx.x;i<blocks;i+=blockDim.x)MergeObservers(local,scratch.data[i]);
  values[threadIdx.x]=local;MergeBlock(values);
  if(!threadIdx.x) {
    auto diagnostics=storage.result.diagnostics;
    if(!ApplyObservers(values[0],storage.model.node_count,diagnostics))d::ReduceNodes<true>(storage,k);
    else {
      diagnostics.node_count=storage.model.node_count;diagnostics.parent_count=storage.model.parent_count;
      diagnostics.stiffness_rate_bound=storage.model.rate;
      storage.result.diagnostics=diagnostics;
    }
  }
}
} // namespace observer_detail
inline void ReduceGlobalObservers(nodal_wall_device_detail::Storage* storage,Sidecar side,
    tl::fea::DeviceNodalKinematicsView k,ObserverScratch scratch,cudaStream_t stream) {
  observer_detail::CheckParents<<<1,1,0,stream>>>(storage,side);
  if(cudaPeekAtLastError()!=cudaSuccess)return;
  observer_detail::Observe<<<ObserverMaxBlocks,ObserverThreads,0,stream>>>(storage,side,k,scratch);
  if(cudaPeekAtLastError()!=cudaSuccess)return;
  observer_detail::Finish<<<1,ObserverThreads,0,stream>>>(storage,side,k,scratch);
}
} // namespace tlfea::contact::nodal_wall_mapped::parallel
