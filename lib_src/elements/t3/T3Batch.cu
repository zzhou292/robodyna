// Lifecycle adapted from the qualified QEPH participant; no force equations.
#include "T3BatchStorage.h"
#include <new>
#include <utility>

namespace tl::fea::t3 {
using batch_detail::Storage;
namespace { BatchReport Ok() { return {BatchStatus::Success,"OK"}; } }
T3Batch::Impl::~Impl() { if(storage) cudaFree(storage); }
BatchReport T3Batch::Impl::Runtime(cudaError_t error,const char* message) noexcept {
  if(error==cudaSuccess) return Ok();
  usable=false; Discard(); return {BatchStatus::DeviceFailure,message};
}
BatchReport T3Batch::Impl::PendingError() noexcept {
  if(!usable) return {BatchStatus::DeviceFailure,"CUDA T3 batch is poisoned"};
  return Runtime(cudaGetLastError(),"Pending CUDA error before T3 operation");
}
BatchReport T3Batch::Impl::ReadControl() {
  auto r=Runtime(cudaGetLastError(),"T3 kernel launch failed"); if(r.status!=BatchStatus::Success) return r;
  r=Runtime(cudaMemcpyAsync(&control,&storage->control,sizeof(control),cudaMemcpyDeviceToHost,stream),"T3 control readback failed");
  if(r.status!=BatchStatus::Success) return r;
  r=Runtime(cudaStreamSynchronize(stream),"T3 owner stream failed"); if(r.status!=BatchStatus::Success) return r;
  return {control.status,control.status==BatchStatus::Success?"OK":"T3 device validation failed",
          control.element,control.node,control.element_status};
}
BatchReport T3Batch::Impl::ReadResults(const batch_detail::Slab* source) {
  auto r=PendingError(); if(r.status!=BatchStatus::Success) return r;
  r=Runtime(cudaMemcpyAsync(staging.data(),source->element,config.element_count*sizeof(ForceTrial),
      cudaMemcpyDeviceToHost,stream),"T3 element readback failed");
  if(r.status!=BatchStatus::Success) return r;
  return Runtime(cudaStreamSynchronize(stream),"T3 element readback stream failed");
}
T3Batch::T3Batch()=default;
T3Batch::~T3Batch()=default;
BatchReport T3Batch::Initialize(const T3BatchConfig& config,const T3BatchElement* elements) {
  return InitializeImpl(config,elements,nullptr);
}
BatchReport T3Batch::InitializeJoined(const T3BatchConfig& config,const ShellBatchBinding& binding) {
  if(!binding.prepared()) return {BatchStatus::InvalidInput,"Mixed binding is not prepared"};
  return InitializeImpl(config,nullptr,&binding);
}
BatchReport T3Batch::InitializeImpl(const T3BatchConfig& config,const T3BatchElement* elements,const ShellBatchBinding* joined) {
  if(impl_) return {BatchStatus::InvalidInput,"T3 batch is already initialized"};
  // Startup staging is bounded and heap-backed; it is released after the one
  // resident device allocation is initialized. Per-step storage is unchanged.
  std::unique_ptr<Storage> initial(new(std::nothrow) Storage{});
  if(!initial) return {BatchStatus::ResourceLimit,"T3 startup staging allocation failed"};
  auto report=batch_detail::BuildModel(config,elements,initial->model,initial->slab[0],joined);
  if(report.status!=BatchStatus::Success) return report;
  std::unique_ptr<Impl> candidate(new(std::nothrow) Impl);
  if(!candidate) return {BatchStatus::ResourceLimit,"T3 host allocation failed"};
  candidate->config=config; candidate->accepted_stamp=config.owner;
  candidate->accepted_diagnostics=batch_detail::InitialDiagnostics(config,joined!=nullptr);
  if(joined) candidate->joined_binding.emplace(*joined);
  report=candidate->PendingError(); if(report.status!=BatchStatus::Success) return report;
  report=candidate->Runtime(cudaMalloc(reinterpret_cast<void**>(&candidate->storage),sizeof(Storage)),"T3 allocation failed");
  if(report.status!=BatchStatus::Success) return report;
  report=candidate->Runtime(cudaMemcpy(candidate->storage,initial.get(),sizeof(Storage),cudaMemcpyHostToDevice),"T3 initialization copy failed");
  if(report.status!=BatchStatus::Success) return report;
  candidate->accepted=&candidate->storage->slab[0]; candidate->trial=&candidate->storage->slab[1];
  impl_=std::move(candidate); return Ok();
}
void T3Batch::DiscardTrial() noexcept { if(impl_) impl_->Discard(); }
NodalAllocationInfo T3Batch::allocations() const noexcept {
  return impl_?NodalAllocationInfo{sizeof(Storage),1}:NodalAllocationInfo{};
}
} // namespace tl::fea::t3
