// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NormalStage.h"
#include "../normal_activation/Values.h"
#include <cub/cub.cuh>
#include <algorithm>
namespace tlfea::contact::radioss_type25::runtime_detail {
namespace {
unsigned Blocks(std::size_t n){return unsigned(std::max<std::size_t>(1,std::min<std::size_t>(256,(n+127)/128)));}
__device__ void Fail(Device d,std::size_t row,selection::Status numerical) {
  atomicMin(&d.control->failure,(static_cast<unsigned long long>(row)<<16)|
      (static_cast<unsigned>(numerical)<<8)|static_cast<unsigned>(TransactionStatus::NumericalFailure));
}
__device__ void Check(Device d,current_normals::Report report) {
  if(report.status!=current_normals::Status::Ok)
    Fail(d,report.main==SIZE_MAX?0:report.main,report.status==current_normals::Status::NonfiniteResult?
        selection::Status::NonfiniteResult:selection::Status::InvalidInput);
}
__global__ void Before(Device d,lifecycle::Input input,units_detail::Factors units) {
  for(std::size_t row=blockIdx.x*blockDim.x+threadIdx.x;row<d.source.secondary_count;row+=blockDim.x*gridDim.x) {
    const auto value=lifecycle::detail::PrepareRowBeforeNormals(input,row,units);
    d.normal.optimized[row]=value;d.candidate_counts[row]=value.stage.value.optimized_count;
    if(value.stage.report.status!=selection::Status::Ok)Fail(d,row,value.stage.report.status);
  }
  if(!threadIdx.x&&!blockIdx.x)d.candidate_counts[d.source.secondary_count]=0;
}
__global__ void CountTotal(Device d){if(!threadIdx.x&&!blockIdx.x)d.control->required_candidates=d.candidate_offsets[d.source.secondary_count];}
__global__ void Emit(Device d,lifecycle::Input input,units_detail::Factors units) {
  for(std::size_t row=blockIdx.x*blockDim.x+threadIdx.x;row<d.source.secondary_count;row+=blockDim.x*gridDim.x) {
    const auto& saved=d.normal.optimized[row];const auto& csr=input.spatial_by_secondary;
    const auto begin=d.candidate_offsets[row],end=d.candidate_offsets[row+1];auto used=begin;
    for(auto i=csr.offsets[row];i<csr.offsets[row+1];++i) {
      const auto raw=csr.entries[i];const auto main=input.spatial[raw].local_main;
      if(!lifecycle::detail::OptimizedCandidate(input,row,main,saved.optimization_main,saved.optimization_leave,units))continue;
      if(used>=end){Fail(d,row,selection::Status::InvalidInput);break;}
      d.order_keys[used]=raw;d.order_slots[used]=std::uint32_t(main);++used;
    }
    if(used!=end)Fail(d,row,selection::Status::InvalidInput);
  }
}
__global__ void OrderedSuffix(Device d,std::size_t count,std::size_t raw_count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x)
    if((i&&d.sorted_keys[i]<=d.sorted_keys[i-1])||d.sorted_keys[i]>=raw_count||
       !d.sorted_slots[i]||d.sorted_slots[i]>d.source.main_count)Fail(d,i,selection::Status::InvalidInput);
}
normal_activation::Input Activation(Device d,lifecycle::Input input,std::size_t count) {
  return {d.normal.shape.activation,input.source,d.normal.optimized,input.source.secondary_count,
      count?d.sorted_slots:nullptr,count,d.normal.free_mains,d.normal.shape.free_count};
}
current_normals::Input Normals(Device d,lifecycle::Input input,unsigned accepted) {
  current_normals::Input result;result.profile=d.normal.shape.mixed?current_normals::Profile::MixedSurfaceLocal:current_normals::Profile::OrdinaryShellLocal;
  result.free_roster=normal_activation::FreeRosterPolicy::FreshComplete;result.topology=d.normal.topology;
  result.positions=input.current.positions;result.coordinates=startup::Coordinates::Si;result.units=input.current.native_units;
  result.main_coefficients=d.normal.coefficients;result.coefficient_count=d.source.main_count;
  result.main_active=d.normal.active;result.active_count=d.source.main_count;
  result.node_tag=d.normal.tags;result.tag_count=d.source.node_count;
  result.free_main_ids=d.normal.free_mains;result.free_count=d.normal.shape.free_count;
  result.prior_normals=d.normal.face[accepted];result.prior_count=4*d.source.main_count;return result;
}
current_normals::detail::Work Work(Device d,unsigned trial) {
  return {d.normal.face[trial],d.normal.neighbor,d.normal.eligible,d.normal.tage,d.normal.slots,d.normal.references[trial]};
}
__global__ void Seed(Device d,unsigned accepted,unsigned trial) {
  const auto start=blockIdx.x*blockDim.x+threadIdx.x,stride=blockDim.x*gridDim.x;
  for(std::size_t i=start;i<4*d.source.main_count;i+=stride) {
    d.normal.face[trial][i]=d.normal.face[accepted][i];d.normal.neighbor[i]={};d.normal.eligible[i]=0;
  }
  for(std::size_t i=start;i<d.source.normal_count;i+=stride) {
    d.normal.references[trial][i]={};d.normal.slots[2*i]=d.normal.slots[2*i+1]=0;
  }
  for(std::size_t i=start;i<d.source.main_count;i+=stride)d.normal.active[i]=0;
  for(std::size_t i=start;i<d.source.node_count;i+=stride)d.normal.tags[i]=0;
  for(std::size_t i=start;i<d.primary_count;i+=stride)d.normal.tage[i]=0;
}
struct Writer {
  NormalDevice d;
  __device__ void Main(std::uint32_t i){atomicExch(d.active+i,1u);}
  __device__ void Node(std::uint32_t i){atomicExch(d.tags+i,1u);}
};
__global__ void Activate(Device d,normal_activation::Input in) {
  Writer writer{d.normal};
  const auto start=blockIdx.x*blockDim.x+threadIdx.x,stride=blockDim.x*gridDim.x;
  for(std::size_t i=start;i<in.row_count;i+=stride)normal_activation::detail::RetainedRow(in,i,writer);
  for(std::size_t i=start;i<in.optimized_count;i+=stride)normal_activation::detail::OptimizedMain(in,i,writer);
  for(std::size_t i=start;i<in.free_count;i+=stride)normal_activation::detail::FreeMain(in,i,writer);
}
__global__ void Primary(Device d,current_normals::Input in,current_normals::detail::Work work,double length) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.primary_count;i+=blockDim.x*gridDim.x)
    Check(d,current_normals::detail::Primary(in,work,i,length));
}
__global__ void Eligibility(Device d,current_normals::Input in,current_normals::detail::Work work) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<in.free_count;i+=blockDim.x*gridDim.x)
    current_normals::detail::FreeEligibility(in,work,i);
}
__global__ void Slots(Device d,current_normals::Input in,current_normals::detail::Work work) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.source.normal_count;i+=blockDim.x*gridDim.x)
    current_normals::detail::ReferenceSlots(in,work,i);
}
__global__ void Free(Device d,current_normals::Input in,current_normals::detail::Work work,double length) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<in.free_count;i+=blockDim.x*gridDim.x)
    Check(d,current_normals::detail::TransformFree(in,work,i,length));
}
__global__ void References(Device d,current_normals::detail::Work work) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.source.normal_count;i+=blockDim.x*gridDim.x)
    current_normals::detail::FillReference(work,i);
}
__global__ void Gather(Device d,current_normals::Input in,current_normals::detail::Work work) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.source.main_count;i+=blockDim.x*gridDim.x)
    current_normals::detail::GatherNeighbor(in,work,i);
}
__global__ void Average(Device d,current_normals::Input in,current_normals::detail::Work work) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<d.source.main_count;i+=blockDim.x*gridDim.x)
    Check(d,current_normals::detail::Average(in,work,i));
}
__global__ void After(Device d,lifecycle::Input input,units_detail::Factors units) {
  for(std::size_t row=blockIdx.x*blockDim.x+threadIdx.x;row<d.source.secondary_count;row+=blockDim.x*gridDim.x) {
    lifecycle::RowScratch scratch;scratch.sliding_mains=d.sliding+d.sliding_offsets[row];scratch.sliding_capacity=d.sliding_counts[row];
    const auto value=lifecycle::detail::PrepareRowAfterNormals(input,row,scratch,units,d.normal.optimized[row]);
    d.prepared[row]=value;d.candidate_counts[row]=value.stage.report.required_candidates;
    if(value.stage.report.status!=selection::Status::Ok)Fail(d,row,value.stage.report.status);
  }
  if(!threadIdx.x&&!blockIdx.x)d.candidate_counts[d.source.secondary_count]=0;
}
cudaError_t Scan(Device d,cudaStream_t s) {
  auto bytes=d.cub_bytes;auto error=cub::DeviceScan::ExclusiveSum(d.cub,bytes,d.candidate_counts,d.candidate_offsets,int(d.source.secondary_count+1),s);
  if(error!=cudaSuccess)return error;CountTotal<<<1,1,0,s>>>(d);return cudaPeekAtLastError();
}
}
cudaError_t CountBeforeNormals(Device d,lifecycle::Input input,const units_detail::Factors& units,cudaStream_t s) noexcept {
  Before<<<Blocks(d.source.secondary_count),128,0,s>>>(d,input,units);auto error=cudaPeekAtLastError();
  return error==cudaSuccess?Scan(d,s):error;
}
cudaError_t EmitOptimized(Device d,lifecycle::Input input,const units_detail::Factors& units,std::size_t count,cudaStream_t s) noexcept {
  Emit<<<Blocks(d.source.secondary_count),128,0,s>>>(d,input,units);auto error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;
  if(count) {
    auto bytes=d.cub_bytes;error=cub::DeviceRadixSort::SortPairs(d.cub,bytes,d.order_keys,d.sorted_keys,d.order_slots,d.sorted_slots,int(count),0,64,s);
    if(error!=cudaSuccess)return error;
  }
  OrderedSuffix<<<Blocks(count),128,0,s>>>(d,count,input.spatial_count);return cudaPeekAtLastError();
}
cudaError_t UpdateNormals(Device d,lifecycle::Input input,const units_detail::Factors& units,unsigned accepted,unsigned trial,std::size_t count,cudaStream_t s) noexcept {
  const auto in=Normals(d,input,accepted);const auto work=Work(d,trial);
#define LAUNCH(call) call; {const auto error=cudaPeekAtLastError();if(error!=cudaSuccess)return error;}
  LAUNCH((Seed<<<Blocks(std::max(4*d.source.main_count,d.source.node_count)),128,0,s>>>(d,accepted,trial)))
  LAUNCH((Activate<<<Blocks(std::max(count,std::max(d.source.secondary_count,d.normal.shape.free_count))),128,0,s>>>(d,Activation(d,input,count))))
  LAUNCH((Primary<<<Blocks(d.primary_count),128,0,s>>>(d,in,work,units.length)))
  LAUNCH((Eligibility<<<Blocks(in.free_count),128,0,s>>>(d,in,work)))
  LAUNCH((Slots<<<Blocks(d.source.normal_count),128,0,s>>>(d,in,work)))
  LAUNCH((Free<<<Blocks(in.free_count),128,0,s>>>(d,in,work,units.length)))
  LAUNCH((References<<<Blocks(d.source.normal_count),128,0,s>>>(d,work)))
  // Separate kernels are the native gather-before-any-average barrier.
  LAUNCH((Gather<<<Blocks(d.source.main_count),128,0,s>>>(d,in,work)))
  LAUNCH((Average<<<Blocks(d.source.main_count),128,0,s>>>(d,in,work)))
#undef LAUNCH
  return cudaSuccess;
}
cudaError_t CountAfterNormals(Device d,lifecycle::Input input,const units_detail::Factors& units,cudaStream_t s) noexcept {
  After<<<Blocks(d.source.secondary_count),128,0,s>>>(d,input,units);auto error=cudaPeekAtLastError();
  return error==cudaSuccess?Scan(d,s):error;
}
} // namespace tlfea::contact::radioss_type25::runtime_detail
