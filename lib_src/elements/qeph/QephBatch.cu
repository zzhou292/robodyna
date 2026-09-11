#include "QephBatchStorage.h"
#include "../ShellResidentStartupIndex.h"
#include "../ShellResidentHostAccounting.h"
#include <new>
#include <stdexcept>
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
  const unsigned slab=source==&storage->slab[0]?0u:1u;
  if(source!=&storage->slab[slab]) return {BatchStatus::InvalidInput,"Unknown QEPH result slab"};
  r=Runtime(cudaMemcpyAsync(staging.data(),device_header.slab[slab].element,config.element_count*sizeof(ForceTrial),
      cudaMemcpyDeviceToHost,stream),"QEPH element readback failed");
  if(r.status!=BatchStatus::Success) return r;
  return Runtime(cudaStreamSynchronize(stream),"QEPH element readback stream failed");
}
QephBatch::QephBatch()=default;
QephBatch::~QephBatch()=default;
BatchReport QephBatch::Initialize(const QephBatchConfig& config,const QephBatchElement* elements) {
  return InitializeImpl(config,elements,nullptr);
}
BatchReport QephBatch::InitializeJoined(const QephBatchConfig& config,const ShellBatchBinding& binding) {
  if(!binding.prepared()) return {BatchStatus::InvalidInput,"Mixed binding is not prepared"};
  return InitializeImpl(config,nullptr,&binding);
}
BatchReport QephBatch::Initialize(const QephBatchConfig& config,const QephBatchElement* elements,
    const ShellBatchPlasticityConfig& plasticity) {
  return InitializeImpl(config,elements,nullptr,&plasticity);
}
BatchReport QephBatch::InitializeJoined(const QephBatchConfig& config,const ShellBatchBinding& binding,
    const ShellBatchPlasticityConfig& plasticity) {
  if(!binding.prepared()) return {BatchStatus::InvalidInput,"Mixed binding is not prepared"};
  return InitializeImpl(config,nullptr,&binding,&plasticity);
}
BatchReport QephBatch::InitializeJoined(const QephBatchConfig& config,const ShellBatchBinding& binding,
    const NodalMassBinding& mass) {
  return InitializeImpl(config,nullptr,&binding,nullptr,nullptr,&mass);
}
BatchReport QephBatch::InitializeJoined(const QephBatchConfig& config,const ShellBatchBinding& binding,
    const ShellBatchPlasticityBinding& plasticity,const NodalMassBinding& mass) {
  if(!plasticity.Matches(binding))
    return {BatchStatus::InvalidInput,"Complete plasticity catalog differs from the joined native binding"};
  return InitializeImpl(config,nullptr,&binding,nullptr,&plasticity,&mass);
}
BatchReport QephBatch::InitializeImpl(const QephBatchConfig& config,const QephBatchElement* elements,
    const ShellBatchBinding* joined,const ShellBatchPlasticityConfig* plasticity,
    const ShellBatchPlasticityBinding* collection_plasticity,const NodalMassBinding* nodal_mass,
    const ShellBatchFailureBinding* failure,const ShellBatchFailureLimits* failure_limits) try {
  if(impl_) return {BatchStatus::InvalidInput,"QEPH batch is already initialized"};
  if(joined&&joined->qbat_count()!=0)
    return {BatchStatus::InvalidInput,"QBAT requires a complete formulation publication participant"};
  if(nodal_mass&&(!joined||!nodal_mass->Matches(*joined)))
    return {BatchStatus::InvalidInput,"Combined nodal mass differs from complete joined shell inventory"};
  batch_detail::Layout layout;
  if(!ValidShellResidentLimits(config.storage_limits,config.element_count,config.owner.node_count,config.max_device_bytes)||
     !layout.Initialize(config.element_count,config.owner.node_count,config.max_device_bytes))
    return {BatchStatus::ResourceLimit,"QEPH active element/node/device capacity exceeded"};
  const auto host_cap=config.storage_limits.max_host_bytes;
  const bool vehicle=VehicleShellResidentLimits(config.storage_limits);
  std::size_t binding_bytes=0,catalog_bytes=0;
  if(!shell_batch_detail::RetainedScopeBytes(joined,collection_plasticity,vehicle,binding_bytes,catalog_bytes))
    return {BatchStatus::ResourceLimit,"Invalid retained immutable scope payload"};
  util::BoundedArenaLayout host_budget(host_cap); util::ArenaRegion ignored;
  if(!host_budget.Append<unsigned char>(sizeof(Impl),ignored)||!host_budget.Append<unsigned char>(layout.bytes,ignored)||
     !host_budget.Append<ForceTrial>(config.element_count,ignored)||!host_budget.Append<unsigned char>(64,ignored)||
     !host_budget.Append<bool>(config.owner.node_count,ignored)||!host_budget.Append<std::uint64_t>(config.owner.node_count,ignored)||
     (joined&&!host_budget.Append<unsigned char>(binding_bytes,ignored))||
     (nodal_mass&&!host_budget.Append<unsigned char>(nodal_mass->host_bytes(),ignored))||
     (vehicle&&!host_budget.Append<unsigned char>(shell_batch_detail::ResidentIndexBytes(config.element_count,false),ignored)))
    return {BatchStatus::ResourceLimit,"QEPH startup payload exceeds host cap"};
  if(plasticity||collection_plasticity) {
    using namespace shell_batch_plasticity_detail;
    if(plasticity&&(!plasticity->material_id||!plasticity->curve_id||!plasticity->curve.plastic_strain||
       !plasticity->curve.yield_stress_pa||plasticity->curve.count<2||plasticity->curve.count>MaxCurvePoints))
      return {BatchStatus::InvalidInput,"Invalid plastic material identity/curve shape"};
    const auto points=plasticity?plasticity->curve.count:collection_plasticity->curve_point_count();
    Layout plastic_layout; std::size_t plastic_host_bytes=0;
    const bool mixed=collection_plasticity&&collection_plasticity->heterogeneous_sections();
    if (failure && (!failure_limits || !mixed || !failure->Matches(*collection_plasticity) ||
                    failure->host_bytes() < collection_plasticity->host_bytes())) {
      return {BatchStatus::InvalidInput, "Failure sidecar requires explicit mixed scope and limits"};
    }
    bool forecast = false;
    if (failure) {
      const auto failure_bytes = failure->host_bytes() - collection_plasticity->host_bytes() + catalog_bytes;
      forecast = HostStorage::ForecastFailureSections(config.element_count, points, failure_bytes,
          config.max_device_bytes - layout.bytes, host_cap, *failure_limits, plastic_host_bytes);
    } else if (mixed) {
      forecast = HostStorage::ForecastSections(config.element_count, points, catalog_bytes,
          config.max_device_bytes - layout.bytes, host_cap, plastic_host_bytes);
    } else {
      forecast = HostStorage::Forecast(config.element_count, points, catalog_bytes,
          config.max_device_bytes - layout.bytes, host_cap, plastic_layout, plastic_host_bytes);
    }
    if(!forecast||
       !host_budget.Append<unsigned char>(plastic_host_bytes,ignored)||
       !host_budget.Append<ReferenceMaterial>(config.element_count,ignored))
      return {BatchStatus::ResourceLimit,"QEPH combined plasticity payload exceeds startup budgets"};
  }
  util::HostArena arena;
  if(!arena.Initialize(layout.bytes)) return {BatchStatus::ResourceLimit,"QEPH startup staging allocation failed"};
  auto* initial=layout.Construct(arena);
  if(!initial) return {BatchStatus::ResourceLimit,"QEPH startup arena layout is invalid"};
  auto report=batch_detail::BuildModel(config,elements,initial->model,initial->slab[0],joined,failure);
  if(report.status!=BatchStatus::Success) return report;
  std::unique_ptr<Impl> candidate(new(std::nothrow) Impl);
  if(!candidate) return {BatchStatus::ResourceLimit,"QEPH host allocation failed"};
  candidate->config=config; candidate->accepted_stamp=config.owner; candidate->layout=layout;
  candidate->staging.Resize(config.element_count);
  candidate->accepted_diagnostics=batch_detail::InitialDiagnostics(config,joined!=nullptr);
  if(joined) candidate->joined_binding.emplace(*joined);
  if(nodal_mass) {
    candidate->joined_mass.emplace(*nodal_mass);
    for(std::size_t n=0;n<nodal_mass->node_count();++n) {
      const auto& values=nodal_mass->nodes()[n].coefficients;
      initial->model.mass[n]=values.mass; initial->model.inertia[n]=values.isotropic_inertia;
    }
  }
  report=candidate->PendingError(); if(report.status!=BatchStatus::Success) return report;
  if(plasticity) {
    report=candidate->InitializePlasticity(*plasticity,initial->model);
    if(report.status!=BatchStatus::Success) return report;
  }
  if(collection_plasticity) {
    if (failure) {
      report = candidate->InitializeFailure(*failure, *failure_limits);
    } else {
      report = candidate->InitializePlasticity(*collection_plasticity);
    }
    if(report.status!=BatchStatus::Success) return report;
  }
  report=candidate->Runtime(cudaMalloc(reinterpret_cast<void**>(&candidate->storage),layout.bytes),"QEPH allocation failed");
  if(report.status!=BatchStatus::Success) return report;
  candidate->device_header=layout.Rebase(*initial,candidate->storage);
  *initial=candidate->device_header;
  report=candidate->Runtime(cudaMemcpy(candidate->storage,arena.data(),layout.bytes,cudaMemcpyHostToDevice),"QEPH initialization copy failed");
  if(report.status!=BatchStatus::Success) return report;
  candidate->accepted=&candidate->storage->slab[0]; candidate->trial=&candidate->storage->slab[1];
  impl_=std::move(candidate); return Ok();
} catch(const std::bad_alloc&) { return {BatchStatus::ResourceLimit,"QEPH host allocation failed"}; }
  catch(const std::length_error&) { return {BatchStatus::ResourceLimit,"QEPH host size overflow"}; }
void QephBatch::DiscardTrial() noexcept { if(impl_) impl_->Discard(); }
NodalAllocationInfo QephBatch::allocations() const noexcept {
  return impl_?NodalAllocationInfo{impl_->layout.bytes+(impl_->plasticity?impl_->plasticity->device_bytes():0),
      impl_->plasticity?(impl_->plasticity->failure_sections()?3u:2u):1u}:NodalAllocationInfo{};
}
} // namespace tl::fea::qeph
