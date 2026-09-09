#include "QephBatchStorage.h"
#include <new>
#include <utility>

namespace tl::fea::qeph {
using batch_detail::Storage;
namespace { BatchReport Ok() { return {BatchStatus::Success,"OK"}; } }
QephBatch::Impl::~Impl() { if(storage) cudaFree(storage); }
BatchReport QephBatch::Impl::Runtime(cudaError_t error,const char* message) noexcept {
  if(error==cudaSuccess) return Ok();
  usable=false; Discard(); return {BatchStatus::DeviceFailure,message};
}
BatchReport QephBatch::Impl::PendingError() noexcept {
  if(!usable) return {BatchStatus::DeviceFailure,"CUDA QEPH batch is poisoned"};
  return Runtime(cudaGetLastError(),"Pending CUDA error before QEPH operation");
}
BatchReport QephBatch::Impl::ReadControl() {
  auto r=Runtime(cudaGetLastError(),"QEPH kernel launch failed"); if(r.status!=BatchStatus::Success) return r;
  r=Runtime(cudaMemcpyAsync(&control,&storage->control,sizeof(control),cudaMemcpyDeviceToHost,stream),"QEPH control readback failed");
  if(r.status!=BatchStatus::Success) return r;
  r=Runtime(cudaStreamSynchronize(stream),"QEPH owner stream failed"); if(r.status!=BatchStatus::Success) return r;
  return {control.status,control.status==BatchStatus::Success?"OK":"QEPH device validation failed",
          control.element,control.node,control.element_status};
}
BatchReport QephBatch::Impl::ReadResults(const batch_detail::Slab* source) {
  auto r=PendingError(); if(r.status!=BatchStatus::Success) return r;
  r=Runtime(cudaMemcpyAsync(staging.data(),source->element,config.element_count*sizeof(ForceTrial),
      cudaMemcpyDeviceToHost,stream),"QEPH element readback failed");
  if(r.status!=BatchStatus::Success) return r;
  return Runtime(cudaStreamSynchronize(stream),"QEPH element readback stream failed");
}
QephBatch::QephBatch()=default;
QephBatch::~QephBatch()=default;
BatchReport QephBatch::Initialize(const QephBatchConfig& config,const QephBatchElement* elements) {
  if(impl_) return {BatchStatus::InvalidInput,"QEPH batch is already initialized"};
  Storage initial{};
  auto report=batch_detail::BuildModel(config,elements,initial.model,initial.slab[0]);
  if(report.status!=BatchStatus::Success) return report;
  std::unique_ptr<Impl> candidate(new(std::nothrow) Impl);
  if(!candidate) return {BatchStatus::ResourceLimit,"QEPH host allocation failed"};
  candidate->config=config; candidate->accepted_stamp=config.owner;
  candidate->accepted_diagnostics=batch_detail::InitialDiagnostics(config);
  report=candidate->PendingError(); if(report.status!=BatchStatus::Success) return report;
  report=candidate->Runtime(cudaMalloc(reinterpret_cast<void**>(&candidate->storage),sizeof(Storage)),"QEPH allocation failed");
  if(report.status!=BatchStatus::Success) return report;
  report=candidate->Runtime(cudaMemcpy(candidate->storage,&initial,sizeof(initial),cudaMemcpyHostToDevice),"QEPH initialization copy failed");
  if(report.status!=BatchStatus::Success) return report;
  candidate->accepted=&candidate->storage->slab[0]; candidate->trial=&candidate->storage->slab[1];
  impl_=std::move(candidate); return Ok();
}
void QephBatch::DiscardTrial() noexcept { if(impl_) impl_->Discard(); }
NodalAllocationInfo QephBatch::allocations() const noexcept {
  return impl_?NodalAllocationInfo{sizeof(Storage),1}:NodalAllocationInfo{};
}
} // namespace tl::fea::qeph
