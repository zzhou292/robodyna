// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <new>

namespace tl::fea::type45 {
Batch::Batch()=default;
Batch::~Batch()=default;
Batch::Impl::~Impl() {if(device) cudaFree(device);}
BatchReport Batch::Impl::Runtime(cudaError_t error,const char* message) noexcept {
  if(error==cudaSuccess) return {};
  usable=false;Discard();return {BatchStatus::DeviceFailure,message};
}
BatchReport Batch::Impl::PendingError() noexcept {
  if(!usable) return {BatchStatus::Unusable,"Joint device storage is poisoned"};
  return Runtime(cudaGetLastError(),"Pending CUDA error before joint operation");
}
BatchReport Batch::Impl::ReadControl() {
  auto r=Runtime(cudaGetLastError(),"Joint kernel launch failed");
  if(r) r=Runtime(cudaMemcpyAsync(&control,&device->control,sizeof(control),cudaMemcpyDeviceToHost,stream),
      "Joint control readback failed");
  if(r) r=Runtime(cudaStreamSynchronize(stream),"Joint owner stream failed");
  if(!r) return r;
  return {control.status,control.status==BatchStatus::Success?"OK":"Joint device validation rejected",
      control.joint,control.node,control.joint_status};
}
BatchReport Batch::InitializeJoined(const BatchConfig& config,const Model& model) try {
  if(impl_) return {BatchStatus::InvalidInput,"Joint batch is already initialized"};
  BatchForecast forecast;
  auto report=Forecast(config,model,forecast);
  if(!report) return report;
  resident_detail::ArenaLayout layout;
  report=resident_detail::Plan(config,model,layout);
  if(!report) return report;
  util::HostArena upload;
  if(!upload.Initialize(layout.bytes)) return {BatchStatus::ResourceLimit,"Joint upload allocation failed"};
  report=resident_detail::BuildUpload(config,model,upload,layout);
  if(!report) return report;
  auto next=std::make_unique<Impl>(config,model);
  next->layout=layout;next->host_bytes=forecast.startup_host_bytes;
  if(!next->staging.Initialize(layout.staging_bytes) ||
      !next->staging.Construct<resident_detail::State>(layout.staging) ||
      !next->staging.Construct<AutomaticStiffnessContext>(layout.host_contexts) ||
      !next->staging.Construct<NodalCinPhysicalMain>(layout.mains))
    return {BatchStatus::ResourceLimit,"Joint staging allocation or typed construction failed"};
  report=next->PendingError();
  if(report) report=next->Runtime(cudaMalloc(reinterpret_cast<void**>(&next->device),layout.bytes),
      "Joint device arena allocation failed");
  if(!report) return report;
  auto header=resident_detail::RebasedHeader(next->device,layout);
  header.config=config;header.source_instance_id=model.source_instance_id();next->device_header=header;
  *util::ArenaPointer<resident_detail::Storage>(upload.data(),layout.header)=header;
  report=next->Runtime(cudaMemcpy(next->device,upload.data(),layout.bytes,cudaMemcpyHostToDevice),
      "Joint model upload failed");
  if(!report) return report;
  resident_detail::LaunchInitialize(next->device,next->stream);
  report=next->ReadControl();
  if(!report) return report;
  next->accepted_diagnostics=next->control.diagnostics;
  report=next->ReadResults(0,next->accepted_diagnostics);
  if(!report) return report;
  impl_=std::move(next);return {};
} catch(const std::bad_alloc&) {return {BatchStatus::ResourceLimit,"Joint host allocation failed"};}
void Batch::DiscardTrial() noexcept {if(impl_) impl_->Discard();}
NodalAllocationInfo Batch::allocations() const noexcept {
  return impl_?NodalAllocationInfo{impl_->layout.bytes,1}:NodalAllocationInfo{};
}
std::size_t Batch::startup_host_bytes() const noexcept {return impl_?impl_->host_bytes:0;}
} // namespace tl::fea::type45
