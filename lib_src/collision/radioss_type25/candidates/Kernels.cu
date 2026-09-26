// SPDX-License-Identifier: AGPL-3.0-or-later
// Bipartite sweep and count/scan/fill follow SelfContactBroadphase's bounded
// explicit-stream machinery. Role keys and exact filter are native TYPE25.
#include "Launch.h"
#include "Packing.h"
#include "Sweep.cuh"
#include "DeviceRead.cuh"
#include <cub/cub.cuh>
#include <math_constants.h>
#include <algorithm>
#include <limits>
namespace tlfea::contact::radioss_type25::candidates::detail {
namespace {
unsigned Blocks(std::size_t n) {return unsigned(std::min<std::size_t>(256,(n+255)/256));}
__device__ void Fail(Device d,std::size_t row,Status status) {
  sweep::Failure(d,row,status);
}
__device__ bool InDomain(Vector x,const Bounds& b) {
  return x.x>=b.minimum.x&&x.x<=b.maximum.x&&x.y>=b.minimum.y&&x.y<=b.maximum.y&&
      x.z>=b.minimum.z&&x.z<=b.maximum.z;
}
__global__ void Reset(Device d) {
  if(threadIdx.x||blockIdx.x)return;*d.control={};
  d.task_counts[d.main_count]=0;
  if(d.strategy==EnumerationStrategy::CompactGrid) {
    *d.grid={};for(unsigned k=0;k<3;++k)d.grid->lower_bits[k]=~0ull;
    d.encounter_counts[d.main_count]=0;
  }
}
__global__ void SecondaryKeys(Device d,Current in) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.secondary_count;i+=blockDim.x*gridDim.x) {
    d.keys[i]=CUDART_INF;d.ordinals[i]=std::uint32_t(i);
    const auto node=d.secondary[i];const double stiffness=in.secondary_stiffness[i];
    if(!Nonnegative(stiffness)){Fail(d,i,Status::InvalidInput);continue;}
    if(stiffness==0.)continue;
    const auto x=Read(d,in.positions,node);
    if(!tl::math::fixed3::Finite(x)){Fail(d,i,Status::InvalidInput);continue;}
    if(in.domain_policy==DomainPolicy::Bounded&&!InDomain(x,in.domain))continue;
    const double gap=Gap(d,in.secondary_gaps[i]);
    if(!Nonnegative(gap)){Fail(d,i,Status::InvalidInput);continue;}
    d.keys[i]=x.x;
    if(d.strategy==EnumerationStrategy::CompactGrid) {
      const double xyz[]{x.x,x.y,x.z};
      for(unsigned k=0;k<3;++k) {
        const auto bits=static_cast<unsigned long long>(__double_as_longlong(xyz[k]));
        const auto ordered=(bits>>63)?~bits:(bits^(1ull<<63));
        atomicMin(&d.grid->lower_bits[k],ordered);atomicMax(&d.grid->upper_bits[k],ordered);
      }
    }
    atomicAdd(&d.control->active,1ull);
    atomicMax(&d.control->maximum_gap_bits,static_cast<unsigned long long>(__double_as_longlong(gap==0.?0.:gap)));
  }
}
__device__ std::uint32_t Lower(const double* keys,std::uint32_t n,double x,bool strict) {
  std::uint32_t lo=0,hi=n;
  while(lo<hi) {const auto mid=lo+(hi-lo)/2;
    if(strict?keys[mid]<=x:keys[mid]<x)lo=mid+1;else hi=mid;
  }
  return lo;
}
__global__ void MainRanges(Device d,Current in) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.main_count;i+=blockDim.x*gridDim.x) {
    d.ranges[i]={};d.task_counts[i]=0;
    Envelope envelope;bool enabled=false;const auto status=MainEnvelope(d,in,i,envelope,enabled);
    if(status!=Status::Ok){Fail(d,d.secondary_count+i,status);continue;}
    if(!enabled)continue;
    Range range;
    range.first=Lower(d.sorted_keys,d.secondary_count,envelope.bounds.minimum.x,false);
    range.last=Lower(d.sorted_keys,d.secondary_count,envelope.bounds.maximum.x,true);
    d.ranges[i]=range;
    const auto encounters=range.last-range.first;
    d.task_counts[i]=(static_cast<unsigned long long>(encounters)+TaskWidth-1)/TaskWidth;
    atomicAdd(&d.control->encounters,static_cast<unsigned long long>(encounters));
  }
}
__global__ void TaskTotal(Device d) {if(!threadIdx.x&&!blockIdx.x)d.control->tasks=d.task_offsets[d.main_count];}
__device__ bool Removed(Device d,std::uint32_t main,std::uint32_t node) {
  auto lo=d.removal_offsets[main],hi=d.removal_offsets[main+1];
  while(lo<hi){const auto mid=lo+(hi-lo)/2;if(d.removals[mid]<node)lo=mid+1;else hi=mid;}
  return lo<d.removal_offsets[main+1]&&d.removals[lo]==node;
}
__device__ bool Candidate(Device d,const Current& in,const Task& task,unsigned lane,std::uint64_t& key) {
  if(d.strategy==EnumerationStrategy::CompactGrid&&d.grid->packing_failed)return false;
  if(task.first+lane>=task.last)return false;
  const auto secondary=d.strategy==EnumerationStrategy::CompactGrid?
      d.encounter_ordinals[task.first+lane]:d.sorted_ordinals[task.first+lane];
  const auto node=d.secondary[secondary];
  const auto m=d.mains[task.main];
  for(unsigned j=0;j<4;++j)if(node==m.source.nodes[j])return false;
  if(Removed(d,task.main,node))return false;
  LocalRow row;row.secondary_node=d.ids[node];row.secondary_velocity=Read(d,in.velocities,node,true);
  row.constraint_codes[4]=d.codes[node];row.previous_dt=in.previous_dt;
  row.segment_type=m.source.segment_type;row.main_count=d.primary_main_count;
  row.screen.secondary=Read(d,in.positions,node);row.screen.margin=in.margin;
  row.screen.secondary_gap=Gap(d,in.secondary_gaps[secondary]);row.screen.main_gap=Gap(d,in.main_gaps[task.main]);
  row.screen.curvature=Gap(d,in.main_curvature[task.main]);row.screen.gap_load=in.gap_load;
  row.screen.drad=in.drad;row.screen.stored_motion=in.stored_motion;
  for(unsigned j=0;j<4;++j) {
    const auto main_node=m.source.nodes[j];row.nodes[j]=d.ids[main_node];
    row.screen.vertices[j]=Read(d,in.positions,main_node);row.main_velocities[j]=Read(d,in.velocities,main_node,true);
    row.constraint_codes[j]=d.codes[main_node];
  }
  FilterResult result;const auto status=EvaluateLocal(row,&result);
  if(status!=Status::Ok){Fail(d,secondary,status);return false;}
  key=(static_cast<std::uint64_t>(secondary)<<32)|m.rank;return result.included;
}
__global__ void PairTotal(Device d,std::size_t tasks) {if(!threadIdx.x&&!blockIdx.x)d.control->pairs=d.pair_offsets[tasks];}
struct EnginePolicy {
  __device__ static bool Candidate(Device d,const Current& in,const Task& task,unsigned lane,std::uint64_t& key) {
    return detail::Candidate(d,in,task,lane,key);
  }
};
}
cudaError_t QueryScratch(const Source& s,Limits limits,std::size_t& bytes) noexcept {
  return QueryStorageScratch({s.physical_nodes,s.secondaries,s.mains,s.removals},limits,bytes);
}
cudaError_t QueryStorageScratch(StorageShape s,Limits limits,std::size_t& bytes) noexcept {
  std::size_t maximum=0,next=0;
  auto error=cub::DeviceRadixSort::SortPairs(nullptr,next,static_cast<double*>(nullptr),static_cast<double*>(nullptr),
      static_cast<std::uint32_t*>(nullptr),static_cast<std::uint32_t*>(nullptr),int(s.secondaries));
  if(error!=cudaSuccess)return error;maximum=std::max(maximum,next);
  error=cub::DeviceScan::ExclusiveSum(nullptr,next,static_cast<unsigned long long*>(nullptr),
      static_cast<unsigned long long*>(nullptr),int(std::max(s.mains,limits.max_tasks)+1));
  if(error!=cudaSuccess)return error;maximum=std::max(maximum,next);
  error=cub::DeviceRadixSort::SortKeys(nullptr,next,static_cast<std::uint64_t*>(nullptr),
      static_cast<std::uint64_t*>(nullptr),int(limits.max_pairs));
  if(error==cudaSuccess)bytes=std::max(maximum,next);return error;
}
cudaError_t BuildRanges(Device d,const Current& in,cudaStream_t stream) noexcept {
  Reset<<<1,1,0,stream>>>(d);auto error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;
  if(d.secondary_count) {
    SecondaryKeys<<<Blocks(d.secondary_count),256,0,stream>>>(d,in);
    error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;
    if(d.strategy==EnumerationStrategy::CompactGrid) {
      error=FinishCompactKeys(d,in,stream);if(error!=cudaSuccess)return error;
    }
    auto bytes=d.cub_bytes;
    error=cub::DeviceRadixSort::SortPairs(d.cub,bytes,d.keys,d.sorted_keys,d.ordinals,d.sorted_ordinals,
        int(d.secondary_count),0,64,stream);if(error!=cudaSuccess)return error;
  }
  if(d.main_count) {
    if(d.strategy==EnumerationStrategy::CompactGrid)error=BuildCompactRanges(d,in,stream);
    else {MainRanges<<<Blocks(d.main_count),256,0,stream>>>(d,in);error=cudaPeekAtLastError();}
    if(error!=cudaSuccess)return error;
  }
  auto bytes=d.cub_bytes;
  error=cub::DeviceScan::ExclusiveSum(d.cub,bytes,d.task_counts,d.task_offsets,int(d.main_count+1),stream);
  if(error!=cudaSuccess)return error;
  TaskTotal<<<1,1,0,stream>>>(d);return cudaPeekAtLastError();
}
cudaError_t CountPairs(Device d,const Current& in,std::size_t tasks,cudaStream_t stream) noexcept {
  if(d.strategy==EnumerationStrategy::CompactGrid) {
    const auto error=FillCompactEncounters(d,in,stream);if(error!=cudaSuccess)return error;
  }
  // tasks may be zero; initialize the scan sentinel without launching zero grids.
  sweep::BuildTasks<<<std::max(1u,Blocks(d.main_count)),256,0,stream>>>(d,tasks);
  auto error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;
  if(tasks){sweep::Count<EnginePolicy><<<std::min<std::size_t>(tasks,256),TaskWidth,0,stream>>>(d,in,tasks);
    error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;}
  auto bytes=d.cub_bytes;
  error=cub::DeviceScan::ExclusiveSum(d.cub,bytes,d.pair_counts,d.pair_offsets,int(tasks+1),stream);
  if(error!=cudaSuccess)return error;
  PairTotal<<<1,1,0,stream>>>(d,tasks);return cudaPeekAtLastError();
}
cudaError_t FillPairs(Device d,const Current& in,std::size_t tasks,std::size_t pairs,cudaStream_t stream) noexcept {
  auto error=cudaSuccess;
  if(tasks){sweep::Fill<EnginePolicy><<<std::min<std::size_t>(tasks,256),TaskWidth,0,stream>>>(d,in,tasks);
    error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;}
  if(pairs) {
    auto bytes=d.cub_bytes;
    error=cub::DeviceRadixSort::SortKeys(d.cub,bytes,d.pair_keys,d.sorted_pair_keys,int(pairs),0,64,stream);
    if(error!=cudaSuccess)return error;
    sweep::Decode<<<Blocks(pairs),256,0,stream>>>(d,pairs);
    error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;
  }
  sweep::Incidence<<<Blocks(d.secondary_count+1),256,0,stream>>>(d,pairs);return cudaPeekAtLastError();
}
} // namespace tlfea::contact::radioss_type25::candidates::detail
