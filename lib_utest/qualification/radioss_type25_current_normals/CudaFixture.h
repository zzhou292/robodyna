// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "lib_src/collision/radioss_type25/current_normals/Admission.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
namespace type25_current_normals_test::device {
// Qualification-only bounded storage, matching the independent native wrapper.
// This orchestrates per-item leaves; it is not a runtime owner or admission API.
constexpr std::size_t Nodes=256,Primaries=160,Mains=2*Primaries,References=4*Mains,Normals=4*Mains;
struct Image {
  c::Input in;
  n::startup::Main mains[Mains];double positions[3*Nodes],coefficient[Mains];
  std::uint32_t active[Mains],tags[Nodes],free_ids[Mains],offsets[References+1],entries[Normals];
  n::StoredNormal prior[Normals];
};
struct Published {
  n::StoredNormal first[Normals],normals[Normals];
  n::startup::NormalReference references[References];int skip[Primaries];
};
struct State {
  c::Report report;double length=1;
  n::StoredNormal normal[Normals],neighbor[Normals],first[Normals];
  unsigned char eligible[Normals],tage[Primaries];
  std::uint32_t slots[2*References];n::startup::NormalReference references[References];
  c::Report errors[Mains];Published published;
};
__device__ inline c::Input Bind(const Image& image) {
  auto in=image.in;in.topology.mains=image.mains;in.topology.normal_to_main.offsets=image.offsets;
  in.topology.normal_to_main.entries=image.entries;in.positions.data=image.positions;
  in.main_coefficients=image.coefficient;in.main_active=image.active;in.node_tag=image.tags;
  in.free_main_ids=in.free_count?image.free_ids:nullptr;in.prior_normals=image.prior;return in;
}
__device__ inline c::detail::Work Work(State& state) {
  return {state.normal,state.neighbor,state.eligible,state.tage,state.slots,state.references};
}
__device__ inline std::size_t Item(std::size_t i,std::size_t size,bool reverse) {return reverse?size-1-i:i;}
__global__ void Admit(const Image* image,State* state,c::Limits limits) {
  if(blockIdx.x||threadIdx.x)return;
  state->report=c::detail::Validate(Bind(*image),limits,state->length);
}
__global__ void CopyPrior(const Image* image,State* state) {
  if(state->report.status!=c::Status::Ok)return;
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<image->in.prior_count;i+=gridDim.x*blockDim.x)
    state->normal[i]=image->prior[i];
  // Remaining private scratch was explicitly zeroed in the uploaded State.
}
__global__ void Primary(const Image* image,State* state,bool reverse) {
  if(state->report.status!=c::Status::Ok)return;
  const auto in=Bind(*image);const auto size=in.topology.primary_count;
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<size;i+=gridDim.x*blockDim.x) {
    const auto m=Item(i,size,reverse);state->errors[m]=c::detail::Primary(in,Work(*state),m,state->length);
  }
}
__global__ void Fold(State* state,std::size_t size) {
  if(blockIdx.x||threadIdx.x||state->report.status!=c::Status::Ok)return;
  // Source-order error selection after all per-item writes have completed.
  for(std::size_t i=0;i<size;++i)if(state->errors[i].status!=c::Status::Ok) {state->report=state->errors[i];return;}
}
__global__ void Eligibility(const Image* image,State* state,bool reverse) {
  if(state->report.status!=c::Status::Ok)return;const auto in=Bind(*image);
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<in.free_count;i+=gridDim.x*blockDim.x)
    c::detail::FreeEligibility(in,Work(*state),Item(i,in.free_count,reverse));
}
__global__ void Slots(const Image* image,State* state,bool reverse) {
  if(state->report.status!=c::Status::Ok)return;const auto in=Bind(*image);
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<in.topology.references;i+=gridDim.x*blockDim.x)
    c::detail::ReferenceSlots(in,Work(*state),Item(i,in.topology.references,reverse));
}
__global__ void Transform(const Image* image,State* state,bool reverse) {
  if(state->report.status!=c::Status::Ok)return;const auto in=Bind(*image);
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<in.free_count;i+=gridDim.x*blockDim.x) {
    const auto j=Item(i,in.free_count,reverse);state->errors[j]=c::detail::TransformFree(in,Work(*state),j,state->length);
  }
}
__global__ void Fill(const Image* image,State* state,bool reverse) {
  if(state->report.status!=c::Status::Ok)return;
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<image->in.topology.references;i+=gridDim.x*blockDim.x)
    c::detail::FillReference(Work(*state),Item(i,image->in.topology.references,reverse));
}
__global__ void SnapshotFlag1(const Image* image,State* state) {
  if(state->report.status!=c::Status::Ok)return;
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<image->in.prior_count;i+=gridDim.x*blockDim.x)state->first[i]=state->normal[i];
}
__global__ void Gather(const Image* image,State* state,bool reverse) {
  if(state->report.status!=c::Status::Ok)return;const auto in=Bind(*image);
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<in.topology.main_count;i+=gridDim.x*blockDim.x)
    c::detail::GatherNeighbor(in,Work(*state),Item(i,in.topology.main_count,reverse));
}
__global__ void Average(const Image* image,State* state,bool reverse) {
  if(state->report.status!=c::Status::Ok)return;const auto in=Bind(*image);
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<in.topology.main_count;i+=gridDim.x*blockDim.x) {
    const auto m=Item(i,in.topology.main_count,reverse);state->errors[m]=c::detail::Average(in,Work(*state),m);
  }
}
__global__ void Publish(const Image* image,State* state) {
  if(state->report.status!=c::Status::Ok)return;
  const auto start=blockIdx.x*blockDim.x+threadIdx.x,stride=gridDim.x*blockDim.x;
  for(std::size_t i=start;i<image->in.prior_count;i+=stride) {
    state->published.first[i]=state->first[i];state->published.normals[i]=state->normal[i];
  }
  for(std::size_t i=start;i<image->in.topology.references;i+=stride)state->published.references[i]=state->references[i];
  for(std::size_t i=start;i<image->in.topology.primary_count;i+=stride)state->published.skip[i]=state->tage[i];
}
inline void Check(cudaError_t code) {if(code!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(code));}
template<class T,std::size_t N>void Copy(T(&to)[N],const T* from,std::size_t size) {
  if(size>N||(!from&&size))throw std::runtime_error("CUDA normal fixture exceeds its typed source extent");
  if(size)std::copy_n(from,size,to);
}
inline void Pack(const c::Input& in,Image& image) {
  const auto& t=in.topology;
  if(t.nodes>Nodes||t.primary_count>Primaries||t.main_count>Mains||t.references>References||
      !in.positions.valid()||in.positions.node_count>Nodes)
    throw std::runtime_error("CUDA normal fixture is outside qualification storage");
  image.in=in;
  Copy(image.mains,t.mains,t.main_count);Copy(image.coefficient,in.main_coefficients,in.coefficient_count);
  Copy(image.active,in.main_active,in.active_count);Copy(image.tags,in.node_tag,in.tag_count);
  Copy(image.free_ids,in.free_main_ids,in.free_count);Copy(image.prior,in.prior_normals,in.prior_count);
  Copy(image.offsets,t.normal_to_main.offsets,t.normal_to_main.offset_count);
  Copy(image.entries,t.normal_to_main.entries,t.normal_to_main.entry_count);
  // Only a fixture packing change; source double bits and declared units remain
  // intact. Actual device arithmetic performs the native/SI conversion.
  for(std::size_t i=0;i<in.positions.node_count;++i) {const auto p=in.positions.at(std::uint32_t(i));
    image.positions[3*i]=p.x;image.positions[3*i+1]=p.y;image.positions[3*i+2]=p.z;}
  image.in.positions.node_stride=3;image.in.positions.component_stride=1;
}
struct Observation {c::Report report;NativeResult values;bool publication_unchanged=false;};
class CurrentNormalsCuda : public type25_friction_test::PacketCuda<> {
 protected:
  Published last_{};bool initialized_=false;
  Observation EvaluateDevice(const c::Input& in,unsigned threads,bool reverse,c::Limits cap) {
    static_assert(sizeof(Image)<=Capacity*RowBytes&&sizeof(State)<=Capacity*RowBytes);
    if(!threads||threads>64)throw std::runtime_error("Unsupported qualification launch shape");
    auto host=std::make_unique<Image>();Pack(in,*host);auto unchanged=std::make_unique<Image>();
    auto next=std::make_unique<State>();auto result=std::make_unique<State>();
    if(!initialized_) {std::memset(&last_,0xa5,sizeof(last_));initialized_=true;}
    next->published=last_;
    auto* image=static_cast<Image*>(input);auto* state=static_cast<State*>(output);
    // Declared after every borrowed host allocation: drains before any of them
    // die, including exceptions during upload, launch or download.
    type25_friction_test::Drain drain{stream};
    Check(cudaMemcpyAsync(image,host.get(),sizeof(Image),cudaMemcpyHostToDevice,stream));
    Check(cudaMemcpyAsync(state,next.get(),sizeof(State),cudaMemcpyHostToDevice,stream));
    Admit<<<1,1,0,stream>>>(image,state,cap);Check(cudaGetLastError());
    CopyPrior<<<3,threads,0,stream>>>(image,state);Check(cudaGetLastError());
    Primary<<<3,threads,0,stream>>>(image,state,reverse);Check(cudaGetLastError());
    Fold<<<1,1,0,stream>>>(state,in.topology.primary_count);Check(cudaGetLastError());
    Eligibility<<<3,threads,0,stream>>>(image,state,reverse);Check(cudaGetLastError());
    Slots<<<3,threads,0,stream>>>(image,state,reverse);Check(cudaGetLastError());
    Transform<<<3,threads,0,stream>>>(image,state,reverse);Check(cudaGetLastError());
    Fold<<<1,1,0,stream>>>(state,in.free_count);Check(cudaGetLastError());
    Fill<<<3,threads,0,stream>>>(image,state,reverse);Check(cudaGetLastError());
    SnapshotFlag1<<<3,threads,0,stream>>>(image,state);Check(cudaGetLastError());
    Gather<<<3,threads,0,stream>>>(image,state,reverse);Check(cudaGetLastError());
    Average<<<3,threads,0,stream>>>(image,state,reverse);Check(cudaGetLastError());
    Fold<<<1,1,0,stream>>>(state,in.topology.main_count);Check(cudaGetLastError());
    Publish<<<3,threads,0,stream>>>(image,state);Check(cudaGetLastError());
    Check(cudaMemcpyAsync(result.get(),state,sizeof(State),cudaMemcpyDeviceToHost,stream));
    Check(cudaMemcpyAsync(unchanged.get(),image,sizeof(Image),cudaMemcpyDeviceToHost,stream));
    Check(cudaStreamSynchronize(stream));EXPECT_EQ(std::memcmp(host.get(),unchanged.get(),sizeof(Image)),0);
    Observation observed;observed.report=result->report;
    observed.publication_unchanged=std::memcmp(&last_,&result->published,sizeof(last_))==0;
    last_=result->published;auto& v=observed.values;
    v.flag1_normals.assign(last_.first,last_.first+in.prior_count);v.normals.assign(last_.normals,last_.normals+in.prior_count);
    v.references.assign(last_.references,last_.references+in.topology.references);
    v.primary_skip.assign(last_.skip,last_.skip+in.topology.primary_count);
    for(const auto& x:v.flag1_normals)v.finite&=std::isfinite(x.x)&&std::isfinite(x.y)&&std::isfinite(x.z);
    for(const auto& x:v.normals)v.finite&=std::isfinite(x.x)&&std::isfinite(x.y)&&std::isfinite(x.z);
    for(const auto& r:v.references)for(const auto& x:r.bisector)
      v.finite&=std::isfinite(x.x)&&std::isfinite(x.y)&&std::isfinite(x.z);
    return observed;
  }
};
} // namespace type25_current_normals_test::device
