// Lifecycle adapted from the qualified QEPH participant; no force equations.
#include "T3BatchStorage.h"
#include "../ShellResidentStartupIndex.h"
#include "../ShellResidentHostAccounting.h"
#include <new>
#include <stdexcept>
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
  const unsigned slab=source==&storage->slab[0]?0u:1u;
  if(source!=&storage->slab[slab]) return {BatchStatus::InvalidInput,"Unknown T3 result slab"};
  r=Runtime(cudaMemcpyAsync(staging.data(),device_header.slab[slab].element,config.element_count*sizeof(ForceTrial),
      cudaMemcpyDeviceToHost,stream),"T3 element readback failed");
  if(r.status!=BatchStatus::Success) return r;
  r=Runtime(cudaStreamSynchronize(stream),"T3 element readback stream failed");
  return r.status==BatchStatus::Success?ValidateMappedResults(slab):r;
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
BatchReport T3Batch::Initialize(const T3BatchConfig& config,const T3BatchElement* elements,
    const ShellBatchPlasticityConfig& plasticity) {
  return InitializeImpl(config,elements,nullptr,&plasticity);
}
BatchReport T3Batch::InitializeJoined(const T3BatchConfig& config,const ShellBatchBinding& binding,
    const ShellBatchPlasticityConfig& plasticity) {
  if(!binding.prepared()) return {BatchStatus::InvalidInput,"Mixed binding is not prepared"};
  return InitializeImpl(config,nullptr,&binding,&plasticity);
}
BatchReport T3Batch::InitializeJoined(const T3BatchConfig& config,const ShellBatchBinding& binding,
    const NodalMassBinding& mass) {
  return InitializeImpl(config,nullptr,&binding,nullptr,nullptr,&mass);
}
BatchReport T3Batch::InitializeJoined(const T3BatchConfig& config,const ShellBatchBinding& binding,
    const ShellBatchPlasticityBinding& plasticity,const NodalMassBinding& mass) {
  if(!plasticity.Matches(binding))
    return {BatchStatus::InvalidInput,"Complete plasticity catalog differs from the joined native binding"};
  return InitializeImpl(config,nullptr,&binding,nullptr,&plasticity,&mass);
}
BatchReport T3Batch::InitializeFormulations(const T3BatchConfig& config,const ShellFormulationScope& scope,
    const ShellBatchFailureLimits& limits) {
  const auto checked=ValidateShellFormulationScope(scope);
  if(checked.status!=ShellPlasticityBindingStatus::Success)
    return {BatchStatus::InvalidInput,checked.message};
  return InitializeImpl(config,nullptr,scope.binding,nullptr,scope.catalog,scope.mass,scope.failure,&limits,true);
}
BatchReport T3Batch::InitializeImpl(const T3BatchConfig& config,const T3BatchElement* elements,
    const ShellBatchBinding* joined,const ShellBatchPlasticityConfig* plasticity,
    const ShellBatchPlasticityBinding* collection_plasticity,const NodalMassBinding* nodal_mass,
    const ShellBatchFailureBinding* failure,const ShellBatchFailureLimits* failure_limits,bool formulations) try {
  if(impl_) return {BatchStatus::InvalidInput,"T3 batch is already initialized"};
  if(joined&&joined->qbat_count()!=0&&!formulations)
    return {BatchStatus::InvalidInput,"QBAT requires a complete formulation publication participant"};
  if(nodal_mass&&(!joined||!nodal_mass->Matches(*joined)))
    return {BatchStatus::InvalidInput,"Combined nodal mass differs from complete joined shell inventory"};
  // Startup staging is bounded and heap-backed; it is released after the one
  // resident device allocation is initialized. Per-step storage is unchanged.
  batch_detail::Layout layout;
  if(!ValidShellResidentLimits(config.storage_limits,config.element_count,config.owner.node_count,config.max_device_bytes)||
     !layout.Initialize(config.element_count,config.owner.node_count,config.max_device_bytes))
    return {BatchStatus::ResourceLimit,"T3 active element/node/device capacity exceeded"};
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
     (vehicle&&!host_budget.Append<unsigned char>(shell_batch_detail::ResidentIndexBytes(config.element_count,true),ignored)))
    return {BatchStatus::ResourceLimit,"T3 startup payload exceeds host cap"};
  if(plasticity||collection_plasticity) {
    using namespace shell_batch_plasticity_detail;
    if(plasticity&&(!plasticity->material_id||!plasticity->curve_id||!plasticity->curve.plastic_strain||
       !plasticity->curve.yield_stress_pa||plasticity->curve.count<2||plasticity->curve.count>MaxCurvePoints))
      return {BatchStatus::InvalidInput,"Invalid plastic material identity/curve shape"};
    const auto points=plasticity?plasticity->curve.count:collection_plasticity->curve_point_count();
    Layout plastic_layout; std::size_t plastic_host_bytes=0;
    const bool mixed=collection_plasticity&&collection_plasticity->heterogeneous_sections();
    ShellSectionCounts section_counts;
    const bool one_point=collection_plasticity&&
        collection_plasticity->Counts(ShellBindingFamily::T3,&section_counts)&&section_counts.law44_nip1;
    if(one_point&&!failure)
      return {BatchStatus::InvalidInput,"One-point T3 requires complete constant failure binding"};
    if (failure && (!failure_limits || !mixed || !failure->Matches(*collection_plasticity) ||
                    failure->host_bytes() < collection_plasticity->host_bytes())) {
      return {BatchStatus::InvalidInput, "Failure sidecar requires explicit mixed scope and limits"};
    }
    bool forecast = false;
    if (failure) {
      const auto failure_bytes = failure->host_bytes() - collection_plasticity->host_bytes() + catalog_bytes;
      forecast = HostStorage::ForecastFailureSections(config.element_count, points, failure_bytes,
          config.max_device_bytes - layout.bytes, host_cap, *failure_limits, plastic_host_bytes, one_point);
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
      return {BatchStatus::ResourceLimit,"T3 combined plasticity payload exceeds startup budgets"};
  }
  util::HostArena arena;
  if(!arena.Initialize(layout.bytes)) return {BatchStatus::ResourceLimit,"T3 startup staging allocation failed"};
  auto* initial=layout.Construct(arena);
  if(!initial) return {BatchStatus::ResourceLimit,"T3 startup arena layout is invalid"};
  auto report=batch_detail::BuildModel(config,elements,initial->model,initial->slab[0],joined,failure,formulations);
  if(report.status!=BatchStatus::Success) return report;
  std::unique_ptr<Impl> candidate(new(std::nothrow) Impl);
  if(!candidate) return {BatchStatus::ResourceLimit,"T3 host allocation failed"};
  candidate->config=config; candidate->accepted_stamp=config.owner; candidate->layout=layout;
  candidate->formulations=formulations;
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
  report=candidate->Runtime(cudaMalloc(reinterpret_cast<void**>(&candidate->storage),layout.bytes),"T3 allocation failed");
  if(report.status!=BatchStatus::Success) return report;
  candidate->device_header=layout.Rebase(*initial,candidate->storage);
  *initial=candidate->device_header;
  report=candidate->Runtime(cudaMemcpy(candidate->storage,arena.data(),layout.bytes,cudaMemcpyHostToDevice),"T3 initialization copy failed");
  if(report.status!=BatchStatus::Success) return report;
  candidate->accepted=&candidate->storage->slab[0]; candidate->trial=&candidate->storage->slab[1];
  impl_=std::move(candidate); return Ok();
} catch(const std::bad_alloc&) { return {BatchStatus::ResourceLimit,"T3 host allocation failed"}; }
  catch(const std::length_error&) { return {BatchStatus::ResourceLimit,"T3 host size overflow"}; }
void T3Batch::DiscardTrial() noexcept { if(impl_) impl_->Discard(); }
NodalAllocationInfo T3Batch::allocations() const noexcept {
  return impl_?NodalAllocationInfo{impl_->layout.bytes+(impl_->plasticity?impl_->plasticity->device_bytes():0),
      impl_->plasticity?(impl_->plasticity->failure_sections()?3u+
          (impl_->plasticity->one_point_sections()?1u:0u):2u):1u}:NodalAllocationInfo{};
}
} // namespace tl::fea::t3
