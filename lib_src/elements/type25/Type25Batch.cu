// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type25BatchStorage.h"
#include <new>
#include <stdexcept>

namespace tl::fea::type25 {
namespace { BatchReport Ok() noexcept { return {BatchStatus::Success,"OK"}; } }
Batch::Impl::~Impl() { if(storage)cudaFree(storage); }
BatchReport Batch::Impl::Runtime(cudaError_t error,const char* message) noexcept {
  if(error==cudaSuccess)return Ok();
  usable=false;Discard();return {BatchStatus::DeviceFailure,message};
}
BatchReport Batch::Impl::PendingError() noexcept {
  if(!usable)return {BatchStatus::Unusable,"TYPE25 CUDA storage is poisoned"};
  return Runtime(cudaGetLastError(),"Pending CUDA error before TYPE25 operation");
}
BatchReport Batch::Impl::ReadControl() {
  auto r=Runtime(cudaGetLastError(),"TYPE25 kernel launch failed");if(r.status!=BatchStatus::Success)return r;
  r=Runtime(cudaMemcpyAsync(&control,&storage->control,sizeof(control),cudaMemcpyDeviceToHost,stream),"TYPE25 control readback failed");
  if(r.status!=BatchStatus::Success)return r;
  r=Runtime(cudaStreamSynchronize(stream),"TYPE25 owner stream failed");if(r.status!=BatchStatus::Success)return r;
  return {control.status,control.status==BatchStatus::Success?"OK":"TYPE25 device validation failed",control.element,control.node,control.element_status};
}
BatchReport Batch::Impl::ReadResults(const batch_detail::Slab* from) {
  auto r=PendingError();if(r.status!=BatchStatus::Success)return r;
  const unsigned slab=from==&storage->slab[0]?0u:1u;
  if(from!=&storage->slab[slab])return {BatchStatus::InvalidInput,"Unknown TYPE25 result slab"};
  r=Runtime(cudaMemcpyAsync(staging.get(),device_header.slab[slab].element,config.element_count*sizeof(Evaluation),
      cudaMemcpyDeviceToHost,stream),"TYPE25 evaluation readback failed");
  if(r.status!=BatchStatus::Success)return r;
  return Runtime(cudaStreamSynchronize(stream),"TYPE25 evaluation readback stream failed");
}
Batch::Batch()=default;
Batch::~Batch()=default;
BatchReport Batch::InitializeJoined(const BatchConfig& config,const Model& model,const NodalMassBinding& mass) try {
  if(impl_)return {BatchStatus::InvalidInput,"TYPE25 batch is already initialized"};
  // Count/byte preflight uses immutable scalar extents before reading arrays or
  // allocating either startup staging or device storage. Shared source handles
  // are conservatively charged in full even when other participants retain them.
  batch_detail::ArenaLayout layout;
  if(!config.max_nodes||config.max_nodes>2048||!config.max_connections||config.max_connections>1024||
     config.owner.node_count>config.max_nodes||config.element_count>config.max_connections||
     !config.max_host_bytes||config.max_host_bytes>16*1024*1024||
     !batch_detail::MakeLayout(model.property_count(),config.element_count,config.owner.node_count,config.max_device_bytes,layout))
    return {BatchStatus::ResourceLimit,"TYPE25 active counts or device/host limits are invalid"};
  util::BoundedArenaLayout budget(config.max_host_bytes);util::ArenaRegion ignored;
  if(!budget.Append<unsigned char>(sizeof(Impl),ignored)||!budget.Append<unsigned char>(layout.bytes,ignored)||
     !budget.Append<Evaluation>(config.element_count,ignored)||!budget.Append<unsigned char>(64,ignored)||
     !budget.Append<unsigned char>(model.owned_payload_bytes(),ignored)||!budget.Append<unsigned char>(mass.host_bytes(),ignored))
    return {BatchStatus::ResourceLimit,"TYPE25 complete startup payload exceeds host cap"};
  util::HostArena arena;
  if(!arena.Initialize(layout.bytes))return {BatchStatus::ResourceLimit,"TYPE25 startup arena allocation failed"};
  batch_detail::Storage header;BatchDiagnostics diagnostics;
  auto r=batch_detail::BuildStartup(config,model,mass,arena,layout,header,diagnostics);
  if(r.status!=BatchStatus::Success)return r;
  auto next=std::make_unique<Impl>();next->config=config;next->accepted_stamp=config.owner;next->layout=layout;
  next->source.emplace(model);next->combined.emplace(mass);next->accepted_diagnostics=diagnostics;
  next->staging=std::make_unique<Evaluation[]>(config.element_count);next->host_payload_bytes=budget.bytes();
  r=next->PendingError();if(r.status!=BatchStatus::Success)return r;
  r=next->Runtime(cudaMalloc(reinterpret_cast<void**>(&next->storage),layout.bytes),"TYPE25 device arena allocation failed");
  if(r.status!=BatchStatus::Success)return r;
  auto device=batch_detail::RebasedHeader(next->storage,layout);
  device.model.config=header.model.config;device.model.units=header.model.units;device.model.source_instance_id=header.model.source_instance_id;
  device.control=header.control;next->device_header=device;
  *util::ArenaPointer<batch_detail::Storage>(arena.data(),layout.header)=device;
  r=next->Runtime(cudaMemcpy(next->storage,arena.data(),layout.bytes,cudaMemcpyHostToDevice),"TYPE25 device startup copy failed");
  if(r.status!=BatchStatus::Success)return r;
  next->accepted=&next->storage->slab[0];next->trial=&next->storage->slab[1];impl_=std::move(next);return Ok();
} catch(const std::bad_alloc&) { return {BatchStatus::ResourceLimit,"TYPE25 host allocation failed"}; }
  catch(const std::length_error&) { return {BatchStatus::ResourceLimit,"TYPE25 host allocation extent overflow"}; }
void Batch::DiscardTrial() noexcept { if(impl_)impl_->Discard(); }
NodalAllocationInfo Batch::allocations() const noexcept {return impl_?NodalAllocationInfo{impl_->layout.bytes,1}:NodalAllocationInfo{};}
std::size_t Batch::host_bytes() const noexcept {return impl_?impl_->host_payload_bytes:0;}
void Batch::Poison() noexcept {if(impl_) {impl_->usable=false;impl_->Discard();}}
} // namespace tl::fea::type25
