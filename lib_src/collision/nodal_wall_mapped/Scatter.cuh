// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ScatterNode.h"

namespace tlfea::contact::nodal_wall_mapped::parallel {
// Reuse evaluation's per-node status only after all its consumers finish. Each
// phase resets every slot it may read; an earlier failure prevents later work.
static __global__ void Stiffness(d::Storage* pointer,Sidecar side,fe::NodalCinAssemblyView cin) {
  auto& storage=*pointer;
  if(storage.control.status!=NodalWallDeviceStatus::Ok) return;
  const unsigned first=blockIdx.x*blockDim.x+threadIdx.x;
  const unsigned stride=gridDim.x*blockDim.x;
  for(unsigned i=first;i<storage.model.node_count;i+=stride) {
    storage.node_status[i]={};
    StageStiffnessNode(storage,side,cin,i,storage.node_status[i]);
  }
}
static __global__ void CheckScatter(d::Storage* pointer) {
  CheckScatterNodes(*pointer);
}
static __global__ void Forces(d::Storage* pointer,fe::NodalAssemblyView view) {
  auto& storage=*pointer;
  if(storage.control.status!=NodalWallDeviceStatus::Ok) return;
  const auto staged=d::StagedForces(storage,view);
  const unsigned first=blockIdx.x*blockDim.x+threadIdx.x;
  const unsigned stride=gridDim.x*blockDim.x;
  for(unsigned i=first;i<storage.model.node_count;i+=stride) {
    storage.node_status[i]={};
    for(unsigned c=0;c<6;++c) d::CopyScatterChannel(storage,view,i,c);
    d::AccumulateScatterNode(storage,view,staged,i,storage.node_status[i]);
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
  Stiffness<<<blocks,d::Workers,0,stream>>>(storage,side,cin);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  CheckScatter<<<1,1,0,stream>>>(storage);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  Forces<<<blocks,d::Workers,0,stream>>>(storage,view);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  CheckScatter<<<1,1,0,stream>>>(storage);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  Publish<<<blocks,d::Workers,0,stream>>>(storage,side,view,cin);
}
} // namespace tlfea::contact::nodal_wall_mapped::parallel
