// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../candidates/Sweep.cuh"
#include <cub/cub.cuh>
#include <math_constants.h>
#include <algorithm>
#include <climits>
namespace tlfea::contact::radioss_type25::initial_source::detail {
namespace {
namespace c=candidates::detail;namespace v=tl::math::fixed3;
unsigned Blocks(std::size_t n){return unsigned(std::max<std::size_t>(1,std::min<std::size_t>(256,(n+255)/256)));}
__device__ void Fail(Device a,std::size_t row,Status status) {atomicMin(&a.control->failure,(static_cast<unsigned long long>(row)<<8)|unsigned(status));}
__device__ int Cell(Device a,unsigned axis,double coordinate) {
  const auto* ctrl=a.control;
  const double value=double(ctrl->grid[axis])*(coordinate-ctrl->minimum[axis])/(ctrl->maximum[axis]-ctrl->minimum[axis]);
  if(!tl::math::Finite(value)||value<double(INT_MIN)||value>double(INT_MAX)){Fail(a,axis,Status::NonfiniteResult);return 0;}
  return max(1,2+min(ctrl->grid[axis],int(value))); // Literal INT truncation and native clamping.
}
__device__ std::uint32_t Lower(const double* keys,std::uint32_t n,double value,bool strict) {
  std::uint32_t lo=0,hi=n;
  while(lo<hi){const auto middle=lo+(hi-lo)/2;if(strict?keys[middle]<=value:keys[middle]<value)lo=middle+1;else hi=middle;}
  return lo;
}
__global__ void SecondaryCells(Device a) {
  if(a.control->failure!=~0ull)return;
  for(std::size_t row=blockIdx.x*blockDim.x+threadIdx.x;row<a.secondary_count;row+=gridDim.x*blockDim.x) {
    a.sweep.keys[row]=CUDART_INF;a.sweep.ordinals[row]=std::uint32_t(row);
    if(a.secondary[row].coefficient==0)continue;
    const auto p=a.positions[a.secondary[row].node];const double xyz[]{p.x,p.y,p.z};bool outside=false;
    for(unsigned k=0;k<3;++k)outside=outside||xyz[k]<a.control->minimum[k]||xyz[k]>a.control->maximum[k];
    if(outside)continue;
    for(unsigned k=0;k<3;++k)a.node_cells[3*row+k]=Cell(a,k,xyz[k]);
    a.sweep.keys[row]=double(a.node_cells[3*row]);atomicAdd(&a.sweep.control->active,1ull);
  }
}
__global__ void MainRanges(Device a) {
  if(a.control->failure!=~0ull)return;
  for(std::size_t m=blockIdx.x*blockDim.x+threadIdx.x;m<a.mains_count;m+=gridDim.x*blockDim.x) {
    a.sweep.ranges[m]={};a.sweep.task_counts[m]=0;
    if(a.internal_main[m])continue;
    const auto& main=a.mains[m];Vector points[4];for(unsigned k=0;k<4;++k)points[k]=a.positions[main.nodes[k]];
    const auto box=c::Box(points);const bool solid=main.segment_type==0||main.segment_type>int(a.mains_count);
    const double gap=a.control->maximum_secondary_gap+a.main_gap[m]+a.controls.gap_load;
    double radius=a.initial_margin+c::Max(gap,a.controls.drad);
    if(solid)radius=c::Max(radius,a.control->edge_average+a.control->maximum_secondary_gap+a.main_gap[m]+a.controls.gap_load);
    if(!c::Nonnegative(radius)){Fail(a,m,Status::NonfiniteResult);continue;}
    const double low[]{box.minimum.x-radius,box.minimum.y-radius,box.minimum.z-radius};
    const double high[]{box.maximum.x+radius,box.maximum.y+radius,box.maximum.z+radius};
    for(unsigned k=0;k<3;++k){a.main_cells[6*m+k]=Cell(a,k,low[k]);a.main_cells[6*m+3+k]=Cell(a,k,high[k]);}
    const auto first=Lower(a.sweep.sorted_keys,std::uint32_t(a.secondary_count),double(a.main_cells[6*m]),false);
    const auto last=Lower(a.sweep.sorted_keys,std::uint32_t(a.secondary_count),double(a.main_cells[6*m+3]),true);
    a.sweep.ranges[m]={first,last};const auto encounters=last-first;
    a.sweep.task_counts[m]=(static_cast<unsigned long long>(encounters)+c::TaskWidth-1)/c::TaskWidth;
    atomicAdd(&a.sweep.control->encounters,static_cast<unsigned long long>(encounters));
  }
}
__global__ void TaskTotal(Device a){if(!blockIdx.x&&!threadIdx.x)a.sweep.control->tasks=a.sweep.task_offsets[a.mains_count];}
__device__ bool Removed(candidates::detail::Device a,std::size_t main,std::uint32_t node) {
  auto lo=a.removal_offsets[main],hi=a.removal_offsets[main+1];
  while(lo<hi){const auto mid=lo+(hi-lo)/2;if(a.removals[mid]<node)lo=mid+1;else hi=mid;}
  return lo<a.removal_offsets[main+1]&&a.removals[lo]==node;
}
struct StarterPolicy {
  __device__ static bool Candidate(candidates::detail::Device sweep,const Device& a,
      const candidates::detail::Task& task,unsigned lane,std::uint64_t& key) {
    if(task.first+lane>=task.last||a.control->failure!=~0ull)return false;
    const auto row=sweep.sorted_ordinals[task.first+lane],m=task.main;
    for(unsigned k=0;k<3;++k)if(a.node_cells[3*row+k]<a.main_cells[6*m+k]||
        a.node_cells[3*row+k]>a.main_cells[6*m+3+k])return false;
    const auto node=a.secondary[row].node;const auto& main=a.mains[m];
    for(auto corner:main.nodes)if(node==corner)return false;
    if(Removed(sweep,m,node))return false;
    if(a.support_solid[m]!=UINT32_MAX) {
      const auto part=a.solids[a.support_solid[m]].part_source_id;
      for(auto j=a.solid_offsets[node];j<a.solid_offsets[node+1];++j)
        if(a.solids[a.solid_incidence[j]].part_source_id==part)return false;
    }
    initial_state::SearchPair in;in.secondary=a.positions[node];in.secondary_gap=a.secondary[row].gap;
    in.main_gap=a.main_gap[m];in.margin=a.initial_margin;in.edge_length=a.edge_length[row];
    in.drad=a.controls.drad;in.gap_load=a.controls.gap_load;in.segment_type=main.segment_type;
    in.expanded_main_count=int(a.mains_count);in.constraint_codes[4]=a.nodes[node].constraint;
    for(unsigned k=0;k<4;++k) {
      in.nodes[k]=a.nodes[main.nodes[k]].source_id;in.vertices[k]=a.positions[main.nodes[k]];
      in.constraint_codes[k]=a.nodes[main.nodes[k]].constraint;
    }
    const auto box=c::Box(in.vertices);const bool solid=main.segment_type==0||main.segment_type>int(a.mains_count);
    const double sum=in.secondary_gap+in.main_gap+in.gap_load;
    double radius=in.margin+c::Max(sum,in.drad);
    if(solid)radius=c::Max(radius,in.edge_length+in.secondary_gap+in.main_gap+in.gap_load);
    if(!c::Nonnegative(radius)){Fail(a,row,Status::NonfiniteResult);return false;}
    const auto p=in.secondary;
    if(p.x<=box.minimum.x-radius||p.x>=box.maximum.x+radius||p.y<=box.minimum.y-radius||
        p.y>=box.maximum.y+radius||p.z<=box.minimum.z-radius||p.z>=box.maximum.z+radius)return false;
    const auto diagonal1=v::Subtract(in.vertices[2],in.vertices[0]),diagonal2=v::Subtract(in.vertices[3],in.vertices[1]);
    const auto normal=v::Cross(diagonal1,diagonal2),first=v::Subtract(p,in.vertices[0]),second=v::Subtract(p,in.vertices[1]);
    const double norm=v::Dot(normal,normal),d1=v::Dot(first,normal),d2=v::Dot(second,normal),product=d1*d2;
    if(!c::Nonnegative(norm)||!tl::math::Finite(product)){Fail(a,row,Status::NonfiniteResult);return false;}
    if(product>0) {
      const double distance=c::Min(d1*d1,d2*d2),bound=radius*radius*norm;
      if(!c::Nonnegative(distance)||!c::Nonnegative(bound)){Fail(a,row,Status::NonfiniteResult);return false;}
      if(distance>bound)return false;
    }
    candidates::FilterResult result;const auto status=initial_state::EvaluateSearchPair(in,&result);
    if(status!=candidates::Status::Ok){Fail(a,row,Status::NonfiniteResult);return false;}
    key=(static_cast<std::uint64_t>(row)<<32)|m;return result.included;
  }
};
__global__ void PairTotal(Device a,std::size_t tasks){if(!blockIdx.x&&!threadIdx.x)a.sweep.control->pairs=a.sweep.pair_offsets[tasks];}
}
cudaError_t BuildRanges(Device a,cudaStream_t stream) noexcept {
  SecondaryCells<<<Blocks(a.secondary_count),256,0,stream>>>(a);auto e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  auto bytes=a.sweep.cub_bytes;
  e=cub::DeviceRadixSort::SortPairs(a.sweep.cub,bytes,a.sweep.keys,a.sweep.sorted_keys,a.sweep.ordinals,a.sweep.sorted_ordinals,int(a.secondary_count),0,64,stream);
  if(e!=cudaSuccess)return e;
  MainRanges<<<Blocks(a.mains_count),256,0,stream>>>(a);e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  bytes=a.sweep.cub_bytes;e=cub::DeviceScan::ExclusiveSum(a.sweep.cub,bytes,a.sweep.task_counts,a.sweep.task_offsets,int(a.mains_count+1),stream);
  if(e!=cudaSuccess)return e;TaskTotal<<<1,1,0,stream>>>(a);return cudaPeekAtLastError();
}
cudaError_t CountPairs(Device a,std::size_t tasks,cudaStream_t stream) noexcept {
  c::sweep::BuildTasks<<<Blocks(a.mains_count),256,0,stream>>>(a.sweep,tasks);auto e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  if(tasks){c::sweep::Count<StarterPolicy><<<std::min<std::size_t>(tasks,256),c::TaskWidth,0,stream>>>(a.sweep,a,tasks);e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;}
  auto bytes=a.sweep.cub_bytes;e=cub::DeviceScan::ExclusiveSum(a.sweep.cub,bytes,a.sweep.pair_counts,a.sweep.pair_offsets,int(tasks+1),stream);
  if(e!=cudaSuccess)return e;PairTotal<<<1,1,0,stream>>>(a,tasks);return cudaPeekAtLastError();
}
cudaError_t FillPairs(Device a,std::size_t tasks,std::size_t pairs,cudaStream_t stream) noexcept {
  cudaError_t e=cudaSuccess;
  if(tasks){c::sweep::Fill<StarterPolicy><<<std::min<std::size_t>(tasks,256),c::TaskWidth,0,stream>>>(a.sweep,a,tasks);e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;}
  if(pairs) {
    auto bytes=a.sweep.cub_bytes;e=cub::DeviceRadixSort::SortKeys(a.sweep.cub,bytes,a.sweep.pair_keys,a.sweep.sorted_pair_keys,int(pairs),0,64,stream);
    if(e!=cudaSuccess)return e;c::sweep::Decode<<<Blocks(pairs),256,0,stream>>>(a.sweep,pairs);e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  }
  c::sweep::Incidence<<<Blocks(a.secondary_count+1),256,0,stream>>>(a.sweep,pairs);return cudaPeekAtLastError();
}
}
