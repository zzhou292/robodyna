// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Launch.h"
#include "DeviceRead.cuh"
#include "Sweep.cuh"
#include <cub/cub.cuh>
#include <math_constants.h>
#include <algorithm>
namespace tlfea::contact::radioss_type25::candidates::detail {
namespace {
unsigned Blocks(std::size_t n){return unsigned(std::min<std::size_t>(256,(n+255)/256));}
__device__ double Decode(unsigned long long key) {
  const auto bits=(key&(1ull<<63))?(key^(1ull<<63)):~key;
  return __longlong_as_double(static_cast<long long>(bits));
}
__global__ void FinishGrid(Device d) {
  if(blockIdx.x||threadIdx.x||d.control->failure!=~0ull)return;
  auto& grid=*d.grid;double low[3]{},high[3]{};
  const auto resolution=GridResolution(d.control->active);
  for(unsigned k=0;k<3;++k) {
    if(d.control->active){low[k]=Decode(grid.lower_bits[k]);high[k]=Decode(grid.upper_bits[k]);}
    grid.span[k]=high[k]-low[k];
    // An overflowed global span must not reject otherwise valid local rows.
    // One bin conservatively visits every active position along that axis.
    grid.cells[k]=grid.span[k]>0&&tl::math::Finite(grid.span[k])?resolution:1;
  }
  grid.bounds={{low[0],low[1],low[2]},{high[0],high[1],high[2]}};
}
__global__ void BinKeys(Device d,Current in) {
  if(d.control->failure!=~0ull)return;
  const auto grid=*d.grid;
  const double low[]{grid.bounds.minimum.x,grid.bounds.minimum.y,grid.bounds.minimum.z};
  const double high[]{grid.bounds.maximum.x,grid.bounds.maximum.y,grid.bounds.maximum.z};
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.secondary_count;i+=gridDim.x*blockDim.x) {
    // SecondaryKeys alone owns validation and its zero-STF/domain non-reads.
    if(d.keys[i]==CUDART_INF)continue;
    const auto p=Read(d,in.positions,d.secondary[i]);const double xyz[]{p.x,p.y,p.z};unsigned cell[3];
    for(unsigned k=0;k<3;++k)cell[k]=GridCell(xyz[k],low[k],high[k],grid.span[k],grid.cells[k]);
    d.keys[i]=GridKey(grid.cells,cell[0],cell[1],cell[2]);
  }
}
__device__ std::uint32_t Lower(const double* keys,std::uint32_t n,double key,bool strict) {
  std::uint32_t lo=0,hi=n;
  while(lo<hi){const auto middle=lo+(hi-lo)/2;if(strict?keys[middle]<=key:keys[middle]<key)lo=middle+1;else hi=middle;}
  return lo;
}
__device__ void PackingFailure(Device d,std::size_t main,Status status) {
  sweep::Failure(d,d.secondary_count+main,status);atomicExch(&d.grid->packing_failed,1u);
}
template<bool Write>
__device__ unsigned long long Visit(Device d,const Current& in,std::size_t main,
    const Bounds& bounds,unsigned long long offset=0,unsigned long long end=0) {
  const auto& grid=*d.grid;
  const double low[]{grid.bounds.minimum.x,grid.bounds.minimum.y,grid.bounds.minimum.z};
  const double high[]{grid.bounds.maximum.x,grid.bounds.maximum.y,grid.bounds.maximum.z};
  const double first[]{bounds.minimum.x,bounds.minimum.y,bounds.minimum.z};
  const double last[]{bounds.maximum.x,bounds.maximum.y,bounds.maximum.z};
  unsigned begin[3],finish[3];
  for(unsigned k=0;k<3;++k) {
    if(last[k]<low[k]||first[k]>high[k])return 0;
    begin[k]=GridCell(first[k],low[k],high[k],grid.span[k],grid.cells[k]);
    finish[k]=GridCell(last[k],low[k],high[k],grid.span[k],grid.cells[k]);
  }
  unsigned long long count=0;
  for(unsigned z=begin[2];z<=finish[2];++z)for(unsigned y=begin[1];y<=finish[1];++y) {
    const auto from=Lower(d.sorted_keys,std::uint32_t(d.secondary_count),GridKey(grid.cells,begin[0],y,z),false);
    const auto to=Lower(d.sorted_keys,std::uint32_t(d.secondary_count),GridKey(grid.cells,finish[0],y,z),true);
    for(auto at=from;at<to;++at) {
      const auto row=d.sorted_ordinals[at];const auto x=Read(d,in.positions,d.secondary[row]);
      // Same conservative ScreenBounds, before any new velocity read or exact
      // native pair predicate. Only these admitted ordinals occupy storage.
      if(x.x<first[0]||x.x>last[0]||x.y<first[1]||x.y>last[1]||x.z<first[2]||x.z>last[2])continue;
      if constexpr(Write) {
        if(offset+count>=d.encounter_capacity)PackingFailure(d,main,Status::ResourceLimit);
        else if(offset+count>=end)PackingFailure(d,main,Status::InvalidInput);
        else d.encounter_ordinals[offset+count]=row;
      }
      ++count;
    }
  }
  return count;
}
__global__ void CountEncounters(Device d,Current in) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.main_count;i+=gridDim.x*blockDim.x) {
    d.ranges[i]={};d.encounter_counts[i]=0;d.task_counts[i]=0;
    Envelope envelope;bool enabled=false;const auto status=MainEnvelope(d,in,i,envelope,enabled);
    if(status!=Status::Ok){sweep::Failure(d,d.secondary_count+i,status);continue;}
    if(!enabled)continue;
    const auto count=Visit<false>(d,in,i,envelope.bounds);
    d.encounter_counts[i]=count;d.task_counts[i]=(count+TaskWidth-1)/TaskWidth;
    atomicAdd(&d.control->encounters,count);
  }
}
__global__ void FillEncounters(Device d,Current in) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.main_count;i+=gridDim.x*blockDim.x) {
    const auto offset=d.encounter_offsets[i],end=d.encounter_offsets[i+1];
    if(end>d.encounter_capacity||offset>end) {
      d.ranges[i]={};PackingFailure(d,i,Status::ResourceLimit);continue;
    }
    d.ranges[i]={std::uint32_t(offset),std::uint32_t(end)};
    Envelope envelope;bool enabled=false;const auto status=MainEnvelope(d,in,i,envelope,enabled);
    if(status!=Status::Ok){PackingFailure(d,i,status);continue;}
    const auto count=enabled?Visit<true>(d,in,i,envelope.bounds,offset,end):0;
    if(count!=end-offset||count!=d.encounter_counts[i])PackingFailure(d,i,Status::InvalidInput);
  }
}
}
cudaError_t FinishCompactKeys(Device d,const Current& in,cudaStream_t stream) noexcept {
  FinishGrid<<<1,1,0,stream>>>(d);auto error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;
  if(d.secondary_count){BinKeys<<<Blocks(d.secondary_count),256,0,stream>>>(d,in);error=cudaPeekAtLastError();}
  return error;
}
cudaError_t BuildCompactRanges(Device d,const Current& in,cudaStream_t stream) noexcept {
  CountEncounters<<<Blocks(d.main_count),256,0,stream>>>(d,in);auto error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;
  auto bytes=d.cub_bytes;
  return cub::DeviceScan::ExclusiveSum(d.cub,bytes,d.encounter_counts,d.encounter_offsets,int(d.main_count+1),stream);
}
cudaError_t FillCompactEncounters(Device d,const Current& in,cudaStream_t stream) noexcept {
  if(!d.main_count)return cudaSuccess;
  FillEncounters<<<Blocks(d.main_count),256,0,stream>>>(d,in);return cudaPeekAtLastError();
}
}
