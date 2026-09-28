// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Launch.h"
#include "RowLaunch.h"
#include "Response.h"
#include "diagnostics/Read.h"
#include "../assembly/Gather.h"
#include <cub/cub.cuh>
#include <algorithm>
namespace tlfea::contact::radioss_type25::runtime_detail {
namespace {
unsigned Blocks(std::size_t count){return unsigned(std::max<std::size_t>(1,std::min<std::size_t>(256,(count+127)/128)));}
__device__ void Fail(Device d,std::size_t row,TransactionStatus status,
    selection::Status numerical=selection::Status::Ok) {
  atomicMin(&d.control->failure,(static_cast<unsigned long long>(row)<<16)|
      (static_cast<unsigned>(numerical)<<8)|static_cast<unsigned>(status));
}
__global__ void Reset(Device d) {if(!threadIdx.x&&!blockIdx.x)*d.control={};}
__global__ void CheckNodes(Device d,tl::fea::NodalAssemblyView view) {
  for(std::size_t node=blockIdx.x*blockDim.x+threadIdx.x;node<d.source.node_count;node+=blockDim.x*gridDim.x) {
    const auto code=d.source.nodes[node].constraint;
    const unsigned world=((code&1)<<2)|(code&2)|((code&4)>>2);
    if(view.translation_fixed_bits[node]!=world||!SupportedConstraint(code,d.source.nodes[node].skew)||
       (d.native_mass&&!normal_detail::Nonnegative(d.native_mass[node])))Fail(d,node,TransactionStatus::SourceMismatch);
    for(unsigned c=0;c<3;++c)
      if(!tl::math::Finite(view.accepted.position_xyz[3*node+c])||
         !tl::math::Finite(view.accepted.velocity_xyz[3*node+c]))Fail(d,node,TransactionStatus::NumericalFailure);
  }
}
__global__ void CheckFixedMain(Device d,tl::fea::NodalAssemblyView view) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.primary_count;i+=blockDim.x*gridDim.x)
    for(unsigned slot=0;slot<4;++slot) {
      const auto node=d.source.mains[i].nodes[slot];const auto x=d.reference_positions[node];
      if(view.translation_fixed_bits[node]!=7||!view.rotation_fixed[node]||
         __double_as_longlong(view.accepted.position_xyz[3*node])!=__double_as_longlong(x.x)||
         __double_as_longlong(view.accepted.position_xyz[3*node+1])!=__double_as_longlong(x.y)||
         __double_as_longlong(view.accepted.position_xyz[3*node+2])!=__double_as_longlong(x.z)||
         view.accepted.velocity_xyz[3*node]!=0||view.accepted.velocity_xyz[3*node+1]!=0||
         view.accepted.velocity_xyz[3*node+2]!=0)Fail(d,node,TransactionStatus::SourceMismatch);
    }
}
__global__ void InventoryRows(Device d,const candidates::Pair* pairs,const std::uint64_t* offsets,std::size_t count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x) {
    const auto pair=pairs[i];d.spatial[i]={int(pair.secondary_row+1),int(pair.main_occurrence+1)};
    d.spatial_entries[i]=std::uint32_t(i);
  }
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<=d.source.secondary_count;i+=blockDim.x*gridDim.x)
    d.spatial_offsets[i]=std::uint32_t(offsets[i]);
}
__global__ void Requirements(Device d,lifecycle::Input input) {
  for(std::size_t row=blockIdx.x*blockDim.x+threadIdx.x;row<d.source.secondary_count;row+=blockDim.x*gridDim.x) {
    const auto need=lifecycle::detail::Requirements(input,row);
    d.sliding_counts[row]=need.sliding;
    if(need.status!=selection::Status::Ok)Fail(d,row,TransactionStatus::NumericalFailure,need.status);
  }
  if(!threadIdx.x&&!blockIdx.x)d.sliding_counts[d.source.secondary_count]=0;
}
__global__ void SlidingTotal(Device d){if(!threadIdx.x&&!blockIdx.x)d.control->required_sliding=d.sliding_offsets[d.source.secondary_count];}
__global__ void PrepareRows(Device d,lifecycle::Input input,units_detail::Factors units) {
  for(std::size_t row=blockIdx.x*blockDim.x+threadIdx.x;row<d.source.secondary_count;row+=blockDim.x*gridDim.x) {
    lifecycle::RowScratch scratch;
    scratch.sliding_mains=d.sliding+d.sliding_offsets[row];scratch.sliding_capacity=d.sliding_counts[row];
    const auto prepared=lifecycle::detail::PrepareRow(input,row,scratch,units);d.prepared[row]=prepared;
    d.candidate_counts[row]=prepared.stage.report.required_candidates;
    if(prepared.stage.report.status!=selection::Status::Ok)
      Fail(d,row,TransactionStatus::NumericalFailure,prepared.stage.report.status);
  }
  if(!threadIdx.x&&!blockIdx.x)d.candidate_counts[d.source.secondary_count]=0;
}
__global__ void CandidateTotal(Device d){if(!threadIdx.x&&!blockIdx.x)d.control->required_candidates=d.candidate_offsets[d.source.secondary_count];}
__global__ void CompleteRows(Device d,lifecycle::Input input,units_detail::Factors units) {
  for(std::size_t row=blockIdx.x*blockDim.x+threadIdx.x;row<d.source.secondary_count;row+=blockDim.x*gridDim.x) {
    const auto base=d.candidate_offsets[row],count=d.candidate_counts[row];
    lifecycle::RowScratch scratch{d.occurrences+base,d.geometry+base,std::size_t(count),
      d.sliding+d.sliding_offsets[row],std::size_t(d.sliding_counts[row]),std::size_t(base)};
    const auto result=lifecycle::detail::CompleteRow(input,row,scratch,units,d.prepared[row]);
    d.row_results[row]=result;
    if(result.report.status!=selection::Status::Ok) {Fail(d,row,TransactionStatus::NumericalFailure,result.report.status);continue;}
    for(std::size_t j=0;j<count;++j) {
      const auto slot=base+j;const auto& o=d.occurrences[slot];
      std::uint64_t ordinal=o.source_ordinal;
      if(o.origin==lifecycle::Origin::Sliding)ordinal=(std::uint64_t(row)<<32)|o.source_ordinal;
      d.order_keys[slot]=(std::uint64_t(o.origin)<<62)|ordinal;
      d.order_slots[slot]=std::uint32_t(slot);
    }
  }
}
__global__ void CanonicalOrder(Device d,std::size_t count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x) {
    auto& occurrence=d.occurrences[d.sorted_slots[i]];occurrence.cache.occurrence=i;
    d.positive_flags[i]=occurrence.secondary>0?1:0;
    if((occurrence.secondary>0)!=occurrence.selected.enabled)Fail(d,occurrence.selected.key.history_index,TransactionStatus::NumericalFailure);
  }
  if(!threadIdx.x&&!blockIdx.x)d.positive_flags[count]=0;
}
__global__ void ForceRank(Device d,std::size_t count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x)
    d.force_rank[d.sorted_slots[i]]=d.positive_flags[i]?d.positive_offsets[i]:UINT32_MAX;
  if(!threadIdx.x&&!blockIdx.x)d.control->kept=d.positive_offsets[count];
}
__global__ void ResponseRows(Device d,lifecycle::Input input,TransactionConfig config,
    units_detail::Factors units,MassOperands mass,double kick,unsigned trial) {
  for(std::size_t row=blockIdx.x*blockDim.x+threadIdx.x;row<d.source.secondary_count;row+=blockDim.x*gridDim.x) {
    auto history=d.row_results[row].value.history;
    const auto begin=d.candidate_offsets[row],end=d.candidate_offsets[row+1];
    for(std::size_t i=begin;i<end;++i){d.responses[i]={};d.row_packets[i]={};d.finalized[i]={};}
    std::size_t first=begin;
    while(first<end) {
      while(first<end&&d.force_rank[first]==UINT32_MAX)++first;
      if(first==end)break;
      const auto cohort=d.force_rank[first]/d.force_packet_size;std::size_t last=first+1;
      while(last<end&&(d.force_rank[last]==UINT32_MAX||d.force_rank[last]/d.force_packet_size==cohort))++last;
      const auto result=RespondRowCohort(config,input,d.occurrences,d.geometry,first,last,
          mass,units,kick,history.row,d.finalized,d.responses,d.row_packets);
      if(result.status!=TransactionStatus::Ok){Fail(d,row,result.status);break;}
      first=last;
    }
    NativeGeometryHistory finished;
    const auto status=lifecycle::FinishNativeRow(history,&finished);
    if(status!=selection::Status::Ok){Fail(d,row,TransactionStatus::NumericalFailure,status);continue;}
    d.history[trial][row]=finished;
    auto secondary=input.source.secondary[row];
    secondary.initial_contact_flag=d.row_results[row].value.initial_contact_flag;d.secondary[trial][row]=secondary;
  }
}
__global__ void PackForces(Device d,std::size_t candidates,std::size_t kept) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<candidates;i+=blockDim.x*gridDim.x) {
    if(!d.positive_flags[i])continue;
    const auto slot=d.sorted_slots[i],rank=d.positive_offsets[i];const auto& o=d.occurrences[slot];
    const auto& main=d.source.mains[o.selected.local_main-1];assembly::Connectivity c;
    for(unsigned j=0;j<4;++j)c.main[j]=main.nodes[j];
    c.secondary=d.source.secondary[o.selected.key.history_index].node;
    d.force_connectivity[rank]=c;d.force_packets[rank]=d.row_packets[slot];
  }
  const auto cohorts=kept/d.force_packet_size+(kept%d.force_packet_size!=0);
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<cohorts;i+=blockDim.x*gridDim.x)
    d.cohort_ends[i]=std::uint32_t((i+1)*d.force_packet_size<kept?(i+1)*d.force_packet_size:kept);
}
__global__ void Diagnostics(Device d,std::size_t candidates,units_detail::Factors units) {
  if(blockIdx.x)return;
  __shared__ diagnostics::Tile tile;
  const unsigned lane=threadIdx.x;
  std::uint64_t active=0;
  double elastic_energy=0,damping_work=0,friction_work=0;
  if(!lane) {
    active=d.control->active;
    elastic_energy=d.control->elastic_energy;damping_work=d.control->damping_work;
    friction_work=d.control->friction_work;
  }
  for(std::size_t first=0;first<candidates;) {
    const auto remaining=candidates-first;
    const auto count=remaining<diagnostics::Threads?remaining:diagnostics::Threads;
    if(lane<count)diagnostics::Store(tile,lane,diagnostics::Read(d,first+lane));
    __syncthreads();
    // Keep every original canonical addition, including incoming totals. No
    // tile subtotal, inactive +0, floating atomics or reassociated reduction.
    if(!lane)for(unsigned local=0;local<count;++local)if(tile.positive[local]) {
      if(tile.contact_active[local])++active;
      elastic_energy+=tile.elastic_energy[local];
      damping_work+=tile.damping_work[local];
      friction_work+=tile.friction_work[local];
    }
    __syncthreads(); // All leader reads finish before another tile overwrites it.
    first+=count;
  }
  if(lane)return;
  elastic_energy*=units.energy;damping_work*=units.energy;friction_work*=units.energy;
  d.control->active=active;d.control->elastic_energy=elastic_energy;
  d.control->damping_work=damping_work;d.control->friction_work=friction_work;
  if(!tl::math::Finite(d.control->elastic_energy)||!tl::math::Finite(d.control->damping_work)||
     !tl::math::Finite(d.control->friction_work))Fail(d,0,TransactionStatus::NumericalFailure);
}
__global__ void GatherNodes(Device d,assembly::Schedule schedule,assembly::Incidence incidence,
    tl::fea::NodalAssemblyView view,tl::fea::NodalCinAssemblyView cin) {
  for(std::size_t node=blockIdx.x*blockDim.x+threadIdx.x;node<d.source.node_count;node+=blockDim.x*gridDim.x) {
    const assembly::SiNodalValue incoming{{view.forces.force_x[node],view.forces.force_y[node],view.forces.force_z[node]},cin.translational_stiffness[node]};
    const auto status=assembly::GatherNode(node,d.force_connectivity,d.force_packets,schedule,incidence,incoming,&d.nodal_output[node]);
    if(status!=assembly::Status::Ok)Fail(d,node,TransactionStatus::NumericalFailure);
  }
}
__global__ void ApplyNodes(Device d,tl::fea::NodalAssemblyView view,tl::fea::NodalCinAssemblyView cin) {
  if(d.control->failure!=~0ull)return;
  for(std::size_t node=blockIdx.x*blockDim.x+threadIdx.x;node<d.source.node_count;node+=blockDim.x*gridDim.x) {
    const auto value=d.nodal_output[node];view.forces.force_x[node]=value.force.x;
    view.forces.force_y[node]=value.force.y;view.forces.force_z[node]=value.force.z;
    cin.translational_stiffness[node]=value.stiffness;
  }
}
}
cudaError_t QueryScratch(std::size_t rows,std::size_t count,std::size_t& output) noexcept {
  std::size_t a=0,b=0,c=0;
  auto e=cub::DeviceScan::ExclusiveSum(nullptr,a,(std::uint64_t*)nullptr,(std::uint64_t*)nullptr,int(rows+1));if(e!=cudaSuccess)return e;
  e=cub::DeviceRadixSort::SortPairs(nullptr,b,(std::uint64_t*)nullptr,(std::uint64_t*)nullptr,
      (std::uint32_t*)nullptr,(std::uint32_t*)nullptr,int(count));if(e!=cudaSuccess)return e;
  e=cub::DeviceScan::ExclusiveSum(nullptr,c,(std::uint32_t*)nullptr,(std::uint32_t*)nullptr,int(count+1));
  if(e==cudaSuccess)output=std::max(a,std::max(b,c));return e;
}
cudaError_t ValidateCurrent(Device d,const tl::fea::NodalAssemblyView& view,cudaStream_t s) noexcept {
  Reset<<<1,1,0,s>>>(d);auto e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  CheckNodes<<<Blocks(d.source.node_count),128,0,s>>>(d,view);e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  if(!d.normal.shape.enabled){CheckFixedMain<<<Blocks(d.primary_count),128,0,s>>>(d,view);return cudaPeekAtLastError();}
  return cudaSuccess;
}
cudaError_t ConvertInventory(Device d,const candidates::Pair* p,const std::uint64_t* offsets,std::size_t count,cudaStream_t s) noexcept {
  InventoryRows<<<Blocks(std::max(count,d.source.secondary_count+1)),128,0,s>>>(d,p,offsets,count);return cudaPeekAtLastError();
}
cudaError_t Prepare(Device d,lifecycle::Input input,const units_detail::Factors&,cudaStream_t s) noexcept {
  Requirements<<<Blocks(d.source.secondary_count),128,0,s>>>(d,input);auto e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  auto bytes=d.cub_bytes;e=cub::DeviceScan::ExclusiveSum(d.cub,bytes,d.sliding_counts,d.sliding_offsets,int(d.source.secondary_count+1),s);
  if(e!=cudaSuccess)return e;SlidingTotal<<<1,1,0,s>>>(d);return cudaPeekAtLastError();
}
cudaError_t CountCandidates(Device d,lifecycle::Input input,const units_detail::Factors& units,cudaStream_t s) noexcept {
  PrepareRows<<<Blocks(d.source.secondary_count),128,0,s>>>(d,input,units);auto e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  auto bytes=d.cub_bytes;e=cub::DeviceScan::ExclusiveSum(d.cub,bytes,d.candidate_counts,d.candidate_offsets,int(d.source.secondary_count+1),s);
  if(e!=cudaSuccess)return e;CandidateTotal<<<1,1,0,s>>>(d);return cudaPeekAtLastError();
}
cudaError_t Complete(Device d,lifecycle::Input input,const units_detail::Factors& units,std::size_t,cudaStream_t s) noexcept {
  CompleteRows<<<IndependentRowBlocks(d.source.secondary_count),RowThreads,0,s>>>(d,input,units);return cudaPeekAtLastError();
}
cudaError_t Order(Device d,std::size_t count,cudaStream_t s) noexcept {
  if(count) {auto bytes=d.cub_bytes;const auto e=cub::DeviceRadixSort::SortPairs(d.cub,bytes,d.order_keys,d.sorted_keys,
      d.order_slots,d.sorted_slots,int(count),0,64,s);if(e!=cudaSuccess)return e;}
  CanonicalOrder<<<Blocks(count),128,0,s>>>(d,count);auto e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  auto bytes=d.cub_bytes;e=cub::DeviceScan::ExclusiveSum(d.cub,bytes,d.positive_flags,d.positive_offsets,int(count+1),s);
  if(e!=cudaSuccess)return e;ForceRank<<<Blocks(count),128,0,s>>>(d,count);return cudaPeekAtLastError();
}
cudaError_t Respond(Device d,lifecycle::Input input,const TransactionConfig& config,const units_detail::Factors& units,
    MassOperands mass,double kick,unsigned trial,std::size_t count,std::size_t kept,cudaStream_t s) noexcept {
  ResponseRows<<<Blocks(d.source.secondary_count),128,0,s>>>(d,input,config,units,mass,kick,trial);auto e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  PackForces<<<Blocks(count),128,0,s>>>(d,count,kept);e=cudaPeekAtLastError();if(e!=cudaSuccess)return e;
  Diagnostics<<<1,diagnostics::Threads,0,s>>>(d,count,units);return cudaPeekAtLastError();
}
cudaError_t Gather(Device d,assembly::Schedule schedule,assembly::Incidence incidence,const tl::fea::NodalAssemblyView& view,
    const tl::fea::NodalCinAssemblyView& cin,cudaStream_t s) noexcept {
  GatherNodes<<<Blocks(d.source.node_count),128,0,s>>>(d,schedule,incidence,view,cin);return cudaPeekAtLastError();
}
cudaError_t Apply(Device d,const tl::fea::NodalAssemblyView& view,const tl::fea::NodalCinAssemblyView& cin,cudaStream_t s) noexcept {
  ApplyNodes<<<Blocks(d.source.node_count),128,0,s>>>(d,view,cin);return cudaPeekAtLastError();
}
} // namespace tlfea::contact::radioss_type25::runtime_detail
