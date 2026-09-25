// SPDX-License-Identifier: AGPL-3.0-or-later
// Bipartite sweep and count/scan/fill follow SelfContactBroadphase's bounded
// explicit-stream machinery. Role keys and exact filter are native TYPE25.
#include "Launch.h"
#include "Packing.h"
#include <cub/cub.cuh>
#include <math_constants.h>
#include <algorithm>
#include <limits>
namespace tlfea::contact::radioss_type25::candidates::detail {
namespace {
unsigned Blocks(std::size_t n) {return unsigned(std::min<std::size_t>(256,(n+255)/256));}
__device__ Vector Read(Device d,VectorView view,std::uint32_t i,bool velocity=false) {
  const auto v=view.at(i);const double divisor=d.si?(velocity?d.velocity:d.length):1.;
  return {v.x/divisor,v.y/divisor,v.z/divisor};
}
__device__ double Gap(Device d,double x){return d.si?x/d.length:x;}
__device__ void Fail(Device d,std::size_t row,Status status) {
  atomicMin(&d.control->failure,(static_cast<unsigned long long>(row)<<8)|unsigned(status));
}
__device__ bool InDomain(Vector x,const Bounds& b) {
  return x.x>=b.minimum.x&&x.x<=b.maximum.x&&x.y>=b.minimum.y&&x.y<=b.maximum.y&&
      x.z>=b.minimum.z&&x.z<=b.maximum.z;
}
__global__ void Reset(Device d) {
  if(threadIdx.x||blockIdx.x)return;*d.control={};
  d.task_counts[d.main_count]=0;
}
__global__ void SecondaryKeys(Device d,Current in) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.secondary_count;i+=blockDim.x*gridDim.x) {
    d.keys[i]=CUDART_INF;d.ordinals[i]=std::uint32_t(i);
    const auto node=d.secondary[i];const double stiffness=in.secondary_stiffness[i];
    if(!Nonnegative(stiffness)){Fail(d,i,Status::InvalidInput);continue;}
    if(stiffness==0.)continue;
    const auto x=Read(d,in.positions,node);
    if(!tl::math::fixed3::Finite(x)){Fail(d,i,Status::InvalidInput);continue;}
    if(!InDomain(x,in.domain))continue;
    const double gap=Gap(d,in.secondary_gaps[i]);
    if(!Nonnegative(gap)){Fail(d,i,Status::InvalidInput);continue;}
    d.keys[i]=x.x;
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
    const auto m=d.mains[i].source;
    if(!Nonnegative(in.main_stiffness[i])){Fail(d,d.secondary_count+i,Status::InvalidInput);continue;}
    if(in.main_stiffness[i]==0.)continue;
    bool valid=Nonnegative(Gap(d,in.main_gaps[i]))&&Nonnegative(Gap(d,in.main_curvature[i]));
    ScreenRow row;row.margin=in.margin;row.curvature=Gap(d,in.main_curvature[i]);row.main_gap=Gap(d,in.main_gaps[i]);
    row.secondary_gap=__longlong_as_double(static_cast<long long>(d.control->maximum_gap_bits));
    row.gap_load=in.gap_load;row.drad=in.drad;row.stored_motion=in.stored_motion;
    for(unsigned j=0;j<4;++j) {
      row.vertices[j]=Read(d,in.positions,m.nodes[j]);
      valid=valid&&tl::math::fixed3::Finite(row.vertices[j]);
    }
    if(!valid){Fail(d,d.secondary_count+i,Status::InvalidInput);continue;}
    if(in.main_stiffness[i]==0.||!d.control->active)continue;
    Envelope envelope;const auto status=ScreenBounds(row,&envelope);
    if(status!=Status::Ok){Fail(d,d.secondary_count+i,status);continue;}
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
__global__ void BuildTasks(Device d,std::size_t count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.main_count;i+=blockDim.x*gridDim.x) {
    const auto range=d.ranges[i];
    for(std::uint32_t first=range.first;first<range.last;first+=TaskWidth) {
      const auto index=d.task_offsets[i]+(first-range.first)/TaskWidth;
      if(index>=count){Fail(d,d.secondary_count+i,Status::ResourceLimit);break;}
      d.tasks[index]={std::uint32_t(i),first,(range.last<first+TaskWidth?range.last:first+TaskWidth)};
    }
  }
  if(!blockIdx.x&&!threadIdx.x)d.pair_counts[count]=0;
}
__device__ bool Removed(Device d,std::uint32_t main,std::uint32_t node) {
  auto lo=d.removal_offsets[main],hi=d.removal_offsets[main+1];
  while(lo<hi){const auto mid=lo+(hi-lo)/2;if(d.removals[mid]<node)lo=mid+1;else hi=mid;}
  return lo<d.removal_offsets[main+1]&&d.removals[lo]==node;
}
__device__ bool Candidate(Device d,const Current& in,const Task& task,unsigned lane,std::uint64_t& key) {
  if(task.first+lane>=task.last)return false;
  const auto secondary=d.sorted_ordinals[task.first+lane],node=d.secondary[secondary];
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
__global__ void Count(Device d,Current in,std::size_t tasks) {
  using Reduce=cub::BlockReduce<unsigned,TaskWidth>;__shared__ typename Reduce::TempStorage scratch;
  for(std::size_t task=blockIdx.x;task<tasks;task+=gridDim.x) {
    std::uint64_t key=0;const unsigned included=Candidate(d,in,d.tasks[task],threadIdx.x,key)?1:0;
    const auto count=Reduce(scratch).Sum(included);
    if(!threadIdx.x)d.pair_counts[task]=count;
    __syncthreads();
  }
}
__global__ void PairTotal(Device d,std::size_t tasks) {if(!threadIdx.x&&!blockIdx.x)d.control->pairs=d.pair_offsets[tasks];}
__global__ void Fill(Device d,Current in,std::size_t tasks) {
  using Scan=cub::BlockScan<unsigned,TaskWidth>;__shared__ typename Scan::TempStorage scratch;
  for(std::size_t task=blockIdx.x;task<tasks;task+=gridDim.x) {
    std::uint64_t key=0;const unsigned included=Candidate(d,in,d.tasks[task],threadIdx.x,key)?1:0;
    unsigned local=0,total=0;Scan(scratch).ExclusiveSum(included,local,total);
    if(total!=d.pair_counts[task])Fail(d,d.tasks[task].main,Status::InvalidInput);
    if(included) {
      const auto offset=d.pair_offsets[task]+local;
      if(offset<d.pair_capacity&&offset<d.control->pairs)d.pair_keys[offset]=key;
      else Fail(d,d.tasks[task].main,Status::ResourceLimit);
    }
    __syncthreads();
  }
}
__global__ void Decode(Device d,std::size_t count) {
  if(d.control->failure!=~0ull)return;
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x) {
    const auto key=d.sorted_pair_keys[i];d.pairs[i]={std::uint32_t(key>>32),d.ranks[std::uint32_t(key)]};
  }
}
__global__ void Incidence(Device d,std::size_t count) {
  if(d.control->failure!=~0ull)return;
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<=d.secondary_count;i+=blockDim.x*gridDim.x) {
    const auto key=static_cast<std::uint64_t>(i)<<32;std::size_t lo=0,hi=count;
    while(lo<hi){const auto mid=lo+(hi-lo)/2;if(d.sorted_pair_keys[mid]<key)lo=mid+1;else hi=mid;}
    d.secondary_offsets[i]=lo;
  }
}
}
cudaError_t QueryScratch(const Source& s,Limits limits,std::size_t& bytes) noexcept {
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
    auto bytes=d.cub_bytes;
    error=cub::DeviceRadixSort::SortPairs(d.cub,bytes,d.keys,d.sorted_keys,d.ordinals,d.sorted_ordinals,
        int(d.secondary_count),0,64,stream);if(error!=cudaSuccess)return error;
  }
  if(d.main_count) {
    MainRanges<<<Blocks(d.main_count),256,0,stream>>>(d,in);
    error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;
  }
  auto bytes=d.cub_bytes;
  error=cub::DeviceScan::ExclusiveSum(d.cub,bytes,d.task_counts,d.task_offsets,int(d.main_count+1),stream);
  if(error!=cudaSuccess)return error;
  TaskTotal<<<1,1,0,stream>>>(d);return cudaPeekAtLastError();
}
cudaError_t CountPairs(Device d,const Current& in,std::size_t tasks,cudaStream_t stream) noexcept {
  // tasks may be zero; initialize the scan sentinel without launching zero grids.
  BuildTasks<<<std::max(1u,Blocks(d.main_count)),256,0,stream>>>(d,tasks);
  auto error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;
  if(tasks){Count<<<std::min<std::size_t>(tasks,256),TaskWidth,0,stream>>>(d,in,tasks);
    error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;}
  auto bytes=d.cub_bytes;
  error=cub::DeviceScan::ExclusiveSum(d.cub,bytes,d.pair_counts,d.pair_offsets,int(tasks+1),stream);
  if(error!=cudaSuccess)return error;
  PairTotal<<<1,1,0,stream>>>(d,tasks);return cudaPeekAtLastError();
}
cudaError_t FillPairs(Device d,const Current& in,std::size_t tasks,std::size_t pairs,cudaStream_t stream) noexcept {
  auto error=cudaSuccess;
  if(tasks){Fill<<<std::min<std::size_t>(tasks,256),TaskWidth,0,stream>>>(d,in,tasks);
    error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;}
  if(pairs) {
    auto bytes=d.cub_bytes;
    error=cub::DeviceRadixSort::SortKeys(d.cub,bytes,d.pair_keys,d.sorted_pair_keys,int(pairs),0,64,stream);
    if(error!=cudaSuccess)return error;
    Decode<<<Blocks(pairs),256,0,stream>>>(d,pairs);
    error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;
  }
  Incidence<<<Blocks(d.secondary_count+1),256,0,stream>>>(d,pairs);return cudaPeekAtLastError();
}
} // namespace tlfea::contact::radioss_type25::candidates::detail
