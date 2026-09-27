// SPDX-License-Identifier: AGPL-3.0-or-later
#include "QbatBatchStorage.h"
#include "QbatBatchStartup.h"
#include "../ShellResidentHostAccounting.h"
#include <new>
#include <stdexcept>

namespace tl::fea::qbat {
namespace {
BatchReport Ok() noexcept { return {BatchStatus::Success,"OK"}; }
}
Batch::Impl::~Impl() {
  if(storage) cudaFree(storage);
}
BatchReport Batch::Impl::Runtime(cudaError_t error,const char* message) noexcept {
  if(error==cudaSuccess) return Ok();
  usable=false;
  Discard();
  return {BatchStatus::DeviceFailure,message};
}
BatchReport Batch::Impl::PendingError() noexcept {
  if(!usable) return {BatchStatus::DeviceFailure,"QBAT CUDA storage is poisoned"};
  return Runtime(cudaGetLastError(),"Pending CUDA error before QBAT operation");
}
BatchReport Batch::Impl::ReadControl() {
  auto report=Runtime(cudaGetLastError(),"QBAT kernel launch failed");
  if(report.status!=BatchStatus::Success) return report;
  report=Runtime(cudaMemcpyAsync(&control,&storage->control,sizeof(control),cudaMemcpyDeviceToHost,stream),
      "QBAT control readback failed");
  if(report.status!=BatchStatus::Success) return report;
  report=Runtime(cudaStreamSynchronize(stream),"QBAT owner stream failed");
  if(report.status!=BatchStatus::Success) return report;
  return {control.status,control.status==BatchStatus::Success?"OK":"QBAT device validation failed",
      control.element,control.node,control.element_status};
}
BatchReport Batch::Impl::Upload(util::HostArena& arena,batch_detail::Storage& host,
    const ShellBatchPlasticityBinding& catalog) {
  auto report=PendingError();
  if(report.status!=BatchStatus::Success) return report;
  report=Runtime(cudaMalloc(reinterpret_cast<void**>(&storage),layout.bytes),"QBAT device arena allocation failed");
  if(report.status!=BatchStatus::Success) return report;
  device_header=layout.Rebase(host,storage);
  if(!batch_detail::RebaseMaterials(host,device_header,catalog)) {
    return {BatchStatus::InvalidInput,"QBAT immutable material curve rebase failed"};
  }
  host=device_header;
  report=Runtime(cudaMemcpy(storage,arena.data(),layout.bytes,cudaMemcpyHostToDevice),
      "QBAT device startup copy failed");
  if(report.status!=BatchStatus::Success) return report;
  accepted=&storage->slab[0];
  trial=&storage->slab[1];
  return Ok();
}
Batch::Batch()=default;
Batch::~Batch()=default;
BatchReport Batch::InitializeFormulations(const BatchConfig& config,const ShellFormulationScope& scope) try {
  if(impl_) return {BatchStatus::InvalidInput,"QBAT batch is already initialized"};
  // Counts/byte limits are inspected before any potentially large allocation.
  // The immutable scope validator consumes only already prepared value owners.
  if(!ValidShellResidentLimits(config.storage_limits,config.element_count,config.owner.node_count,
      config.max_device_bytes)) return {BatchStatus::ResourceLimit,"QBAT resident limits are invalid"};
  auto report=batch_detail::ValidateStartup(config,scope);
  if(report.status!=BatchStatus::Success) return report;
  batch_detail::Layout layout;
  if(!layout.Initialize(config.element_count,config.owner.node_count,scope.catalog->curve_point_count(),
      config.max_device_bytes)) return {BatchStatus::ResourceLimit,"QBAT four-point arena exceeds device cap"};
  const bool vehicle=VehicleShellResidentLimits(config.storage_limits);
  std::size_t binding_bytes=0,catalog_bytes=0;
  if(!shell_batch_detail::RetainedScopeBytes(scope.binding,scope.catalog,vehicle,binding_bytes,catalog_bytes)||
      scope.failure->host_bytes()<scope.catalog->host_bytes()) {
    return {BatchStatus::ResourceLimit,"QBAT immutable scope byte accounting is inconsistent"};
  }
  const auto failure_bytes=scope.failure->host_bytes()-scope.catalog->host_bytes()+catalog_bytes;
  util::BoundedArenaLayout budget(config.storage_limits.max_host_bytes);
  util::ArenaRegion ignored;
  if(!budget.Append<unsigned char>(sizeof(Impl),ignored)||!budget.Append<unsigned char>(layout.bytes,ignored)||
      !budget.Append<BatchResult>(config.element_count,ignored)||
      !budget.Append<std::uint8_t>(mapped_shell::ActivityBytes(config.element_count),ignored)||
      !budget.Append<unsigned char>(64,ignored)||
      !budget.Append<unsigned char>(binding_bytes,ignored)||!budget.Append<unsigned char>(failure_bytes,ignored)||
      (scope.mass&&!budget.Append<unsigned char>(scope.mass->host_bytes(),ignored))) {
    return {BatchStatus::ResourceLimit,"QBAT complete startup/staging payload exceeds host cap"};
  }
  util::HostArena arena;
  if(!arena.Initialize(layout.bytes)) return {BatchStatus::ResourceLimit,"QBAT startup arena allocation failed"};
  auto* host=layout.Construct(arena);
  if(!host) return {BatchStatus::ResourceLimit,"QBAT startup layout is invalid"};
  BatchDiagnostics diagnostics;
  report=batch_detail::BuildStartup(config,scope,*host,diagnostics);
  if(report.status!=BatchStatus::Success) return report;
  auto next=std::make_unique<Impl>();
  next->config=config;
  next->accepted_stamp=config.owner;
  next->layout=layout;
  next->binding.emplace(*scope.binding);
  next->failure.emplace(*scope.failure);
  if(scope.mass) next->combined.emplace(*scope.mass);
  next->accepted_diagnostics=diagnostics;
  next->staging=std::make_unique<BatchResult[]>(config.element_count);
  next->activity_staging.resize(mapped_shell::ActivityBytes(config.element_count));
  next->host_payload_bytes=budget.bytes();
  report=next->Upload(arena,*host,*scope.catalog);
  if(report.status!=BatchStatus::Success) return report;
  impl_=std::move(next);
  return Ok();
} catch(const std::bad_alloc&) {
  return {BatchStatus::ResourceLimit,"QBAT host allocation failed"};
} catch(const std::length_error&) {
  return {BatchStatus::ResourceLimit,"QBAT host allocation extent overflow"};
}
void Batch::DiscardTrial() noexcept {
  if(impl_) impl_->Discard();
}
void Batch::Poison() noexcept {
  if(impl_) {
    impl_->usable=false;
    impl_->Discard();
  }
}
NodalAllocationInfo Batch::allocations() const noexcept {
  return impl_?NodalAllocationInfo{impl_->layout.bytes,1}:NodalAllocationInfo{};
}
std::size_t Batch::host_bytes() const noexcept { return impl_?impl_->host_payload_bytes:0; }
} // namespace tl::fea::qbat
