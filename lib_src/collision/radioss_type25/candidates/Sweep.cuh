// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Device.h"
#include <cub/cub.cuh>
namespace tlfea::contact::radioss_type25::candidates::detail::sweep {
// Private shared sweep machinery. Only source-checked closed Engine/Starter
// policies call it; it owns no profile authority, physical state or clock.
template<class SweepDevice>
__device__ inline void Failure(SweepDevice d,std::size_t row,Status status) {
  atomicMin(&d.control->failure,(static_cast<unsigned long long>(row)<<8)|unsigned(status));
}
template<class SweepDevice>
__global__ void BuildTasks(SweepDevice d,std::size_t count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.main_count;i+=blockDim.x*gridDim.x) {
    const auto range=d.ranges[i];
    for(std::uint32_t first=range.first;first<range.last;first+=TaskWidth) {
      const auto index=d.task_offsets[i]+(first-range.first)/TaskWidth;
      if(index>=count){Failure(d,d.secondary_count+i,Status::ResourceLimit);break;}
      d.tasks[index]={std::uint32_t(i),first,(range.last<first+TaskWidth?range.last:first+TaskWidth)};
    }
  }
  if(!blockIdx.x&&!threadIdx.x)d.pair_counts[count]=0;
}
template<class Policy,class SweepDevice,class Input>
__global__ void Count(SweepDevice d,Input in,std::size_t tasks) {
  using Reduce=cub::BlockReduce<unsigned,TaskWidth>;__shared__ typename Reduce::TempStorage scratch;
  for(std::size_t task=blockIdx.x;task<tasks;task+=gridDim.x) {
    std::uint64_t key=0;const unsigned included=Policy::Candidate(d,in,d.tasks[task],threadIdx.x,key)?1:0;
    const auto count=Reduce(scratch).Sum(included);
    if(!threadIdx.x)d.pair_counts[task]=count;
    __syncthreads();
  }
}
template<class Policy,class SweepDevice,class Input>
__global__ void Fill(SweepDevice d,Input in,std::size_t tasks) {
  using Scan=cub::BlockScan<unsigned,TaskWidth>;__shared__ typename Scan::TempStorage scratch;
  for(std::size_t task=blockIdx.x;task<tasks;task+=gridDim.x) {
    std::uint64_t key=0;const unsigned included=Policy::Candidate(d,in,d.tasks[task],threadIdx.x,key)?1:0;
    unsigned local=0,total=0;Scan(scratch).ExclusiveSum(included,local,total);
    if(total!=d.pair_counts[task])Failure(d,d.tasks[task].main,Status::InvalidInput);
    if(included) {
      const auto offset=d.pair_offsets[task]+local;
      if(offset<d.pair_capacity&&offset<d.control->pairs)d.pair_keys[offset]=key;
      else Failure(d,d.tasks[task].main,Status::ResourceLimit);
    }
    __syncthreads();
  }
}
template<class SweepDevice>
__global__ void Decode(SweepDevice d,std::size_t count) {
  if(d.control->failure!=~0ull)return;
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x) {
    const auto key=d.sorted_pair_keys[i];d.pairs[i]={std::uint32_t(key>>32),d.ranks[std::uint32_t(key)]};
  }
}
template<class SweepDevice>
__global__ void Incidence(SweepDevice d,std::size_t count) {
  if(d.control->failure!=~0ull)return;
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<=d.secondary_count;i+=blockDim.x*gridDim.x) {
    const auto key=static_cast<std::uint64_t>(i)<<32;std::size_t lo=0,hi=count;
    while(lo<hi){const auto mid=lo+(hi-lo)/2;if(d.sorted_pair_keys[mid]<key)lo=mid+1;else hi=mid;}
    d.secondary_offsets[i]=lo;
  }
}
} // namespace tlfea::contact::radioss_type25::candidates::detail::sweep
