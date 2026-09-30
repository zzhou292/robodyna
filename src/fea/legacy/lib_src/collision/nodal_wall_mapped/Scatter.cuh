// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ScatterNode.h"
#include "NodeStatus.cuh"

namespace tlfea::contact::nodal_wall_mapped::parallel {
// Reuse evaluation's per-node status only after all its consumers finish. Each
// phase resets every slot it may read; an earlier failure prevents later work.
static __global__ void BeginScatter(d::Storage* pointer, Sidecar side) {
  if (pointer->control.status == NodalWallDeviceStatus::Ok && side.summary)
    side.summary->parent_failure = ~0ull;
}
static __global__ void Stiffness(d::Storage* pointer,Sidecar side,fe::NodalCinAssemblyView cin) {
  auto& storage=*pointer;
  if(storage.control.status!=NodalWallDeviceStatus::Ok) return;
  const unsigned first=blockIdx.x*blockDim.x+threadIdx.x;
  const unsigned stride=gridDim.x*blockDim.x;
  for(unsigned i=first;i<storage.model.node_count;i+=stride) {
    storage.node_status[i]={};
    StageStiffnessNode(storage,side,cin,i,storage.node_status[i]);
    SelectNodeFailure(storage, side, i);
  }
}
static __global__ void CheckScatter(d::Storage* pointer, Sidecar side) {
  auto& storage = *pointer;
  if (storage.control.status != NodalWallDeviceStatus::Ok) return;
  if (!side.summary) {
    CheckScatterNodes(storage);
    return;
  }
  CopyNodeFailure(storage, side.summary->parent_failure);
  // Stiffness and force phases never share live status writers.
  side.summary->parent_failure = ~0ull;
}
static __global__ void Forces(d::Storage* pointer,Sidecar side,fe::NodalAssemblyView view) {
  auto& storage=*pointer;
  if(storage.control.status!=NodalWallDeviceStatus::Ok) return;
  const auto staged=d::StagedForces(storage,view);
  const unsigned first=blockIdx.x*blockDim.x+threadIdx.x;
  const unsigned stride=gridDim.x*blockDim.x;
  for(unsigned i=first;i<storage.model.node_count;i+=stride) {
    storage.node_status[i]={};
    for(unsigned c=0;c<6;++c) d::CopyScatterChannel(storage,view,i,c);
    d::AccumulateScatterNode(storage,view,staged,i,storage.node_status[i]);
    SelectNodeFailure(storage, side, i);
  }
}
static __global__ void Publish(d::Storage* pointer,Sidecar side,fe::NodalAssemblyView view,
    fe::NodalCinAssemblyView cin) {
  auto& storage=*pointer;
  if(storage.control.status!=NodalWallDeviceStatus::Ok) return;
  const unsigned first=blockIdx.x*blockDim.x+threadIdx.x;
  const unsigned stride=gridDim.x*blockDim.x;
  for(unsigned i=first;i<storage.model.node_count;i+=stride) {
    for(unsigned c=0;c<6;++c) d::PublishScatterChannel(storage,view,i,c);
    cin.translational_stiffness[storage.model.nodes[i].node]=side.stiffness[i];
  }
}
inline void Scatter(d::Storage* storage,Sidecar side,fe::NodalAssemblyView view,
    fe::NodalCinAssemblyView cin,std::size_t nodes,cudaStream_t stream) {
  const auto blocks=1u+static_cast<unsigned>((nodes-1)/d::Workers);
  BeginScatter<<<1,1,0,stream>>>(storage,side);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  Stiffness<<<blocks,d::Workers,0,stream>>>(storage,side,cin);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  CheckScatter<<<1,1,0,stream>>>(storage,side);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  Forces<<<blocks,d::Workers,0,stream>>>(storage,side,view);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  CheckScatter<<<1,1,0,stream>>>(storage,side);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  Publish<<<blocks,d::Workers,0,stream>>>(storage,side,view,cin);
}
} // namespace tlfea::contact::nodal_wall_mapped::parallel
