#include "T3BatchStorage.h"
#include <cstring>
#include <new>

namespace tl::fea::t3 {
BatchReport T3Batch::InitializeJoined(const T3BatchConfig& config,const ShellBatchBinding& binding,
    const ShellBatchPlasticityBinding& plasticity) {
  if(!plasticity.Matches(binding))
    return {BatchStatus::InvalidInput,"Complete plasticity catalog differs from the joined native binding"};
  return InitializeImpl(config,nullptr,&binding,nullptr,&plasticity);
}
BatchReport T3Batch::Impl::InitializePlasticity(const ShellBatchPlasticityBinding& catalog) {
  using namespace shell_batch_plasticity_detail;
  if(!joined_binding) return {BatchStatus::InvalidInput,"Collection plasticity requires a joined native binding"};
  std::unique_ptr<HostStorage> next(new(std::nothrow) HostStorage);
  if(!next) return {BatchStatus::ResourceLimit,"Collection plastic section host allocation failed"};
  const auto setup=next->InitializeCollection(catalog,*joined_binding,ShellBindingFamily::T3,
      config.element_count,config.max_device_bytes-layout.bytes,config.storage_limits.max_host_bytes,
      VehicleShellResidentLimits(config.storage_limits));
  if(setup.status==SetupStatus::DeviceFailure) return Runtime(setup.cuda_status,setup.message);
  if(setup.status!=SetupStatus::Success)
    return {setup.status==SetupStatus::ResourceLimit?BatchStatus::ResourceLimit:BatchStatus::InvalidInput,setup.message};
  plasticity=std::move(next);
  return {BatchStatus::Success,"OK"};
}
BatchReport T3Batch::Impl::InitializePlasticity(const ShellBatchPlasticityConfig& declaration,
    const batch_detail::Model& model) {
  using namespace shell_batch_plasticity_detail;
  std::unique_ptr<ReferenceMaterial[]> references(new(std::nothrow) ReferenceMaterial[config.element_count]{});
  if(!references) return {BatchStatus::ResourceLimit,"Active plastic reference staging allocation failed"};
  for(std::size_t e=0;e<config.element_count;++e) {
    const auto& r=model.element[e].reference.input;
    references[e]={r.young_modulus,r.poisson_ratio,r.density};
  }
  std::unique_ptr<HostStorage> next(new(std::nothrow) HostStorage);
  if(!next) return {BatchStatus::ResourceLimit,"T3 plastic section host allocation failed"};
  const auto setup=next->Initialize(declaration,references.get(),config.element_count,
      config.max_device_bytes-layout.bytes,config.storage_limits.max_host_bytes);
  if(setup.status==SetupStatus::DeviceFailure) return Runtime(setup.cuda_status,setup.message);
  if(setup.status!=SetupStatus::Success)
    return {setup.status==SetupStatus::ResourceLimit?BatchStatus::ResourceLimit:BatchStatus::InvalidInput,setup.message};
  plasticity=std::move(next);
  return {BatchStatus::Success,"OK"};
}
BatchReport T3Batch::CopyAcceptedSectionHistory(const NodalStamp& expected,ShellBatchSectionState* output,
    std::size_t capacity,BatchDiagnostics* diagnostics) {
  using trial_identity::Disjoint;
  if(!impl_) return {BatchStatus::NotInitialized,"T3 batch is not initialized"};
  auto& s=*impl_;
  if(!s.plasticity||s.plasticity->heterogeneous_sections()) return {BatchStatus::InvalidInput,"T3 has no plastic section history"};
  if(!s.bound) return {BatchStatus::NotBound,"Initial T3 source binding is required"};
  if(!batch_detail::SameStamp(expected,s.accepted_stamp))
    return {BatchStatus::StaleTrial,"Accepted T3 section endpoint identity mismatch"};
  if(capacity<s.config.element_count) return {BatchStatus::ResourceLimit,"T3 section readback capacity is insufficient"};
  const auto bytes=s.config.element_count*sizeof(ShellBatchSectionState);
  if(!Disjoint(output,bytes,diagnostics,sizeof(*diagnostics))||!Disjoint(output,bytes,&expected,sizeof expected)||
     !Disjoint(diagnostics,sizeof(*diagnostics),&expected,sizeof expected))
    return {BatchStatus::InvalidInput,"T3 section readback output is missing or overlaps inputs"};
  auto r=s.PendingError(); if(r.status!=BatchStatus::Success) return r;
  r=s.Runtime(s.plasticity->Read(s.AcceptedSlabIndex(),s.config.element_count,s.stream),"T3 section readback failed");
  if(r.status!=BatchStatus::Success) return r;
  std::memcpy(output,s.plasticity->staging(),bytes); *diagnostics=s.accepted_diagnostics;
  return {BatchStatus::Success,"OK"};
}
BatchReport T3Batch::CopyPreparedSectionHistory(const BatchDiagnostics& expected,ShellBatchSectionState* output,
    std::size_t capacity) {
  using trial_identity::Disjoint;
  if(!impl_) return {BatchStatus::NotInitialized,"T3 batch is not initialized"};
  auto& s=*impl_;
  if(!s.plasticity||s.plasticity->heterogeneous_sections()) return {BatchStatus::InvalidInput,"T3 has no plastic section history"};
  if(!s.bound) return {BatchStatus::NotBound,"Initial T3 source binding is required"};
  if(!s.pending||!batch_detail::SameDiagnostics(expected,s.candidate_diagnostics))
    return {BatchStatus::StaleTrial,"Prepared T3 section identity mismatch"};
  if(capacity<s.config.element_count) return {BatchStatus::ResourceLimit,"T3 section readback capacity is insufficient"};
  const auto bytes=s.config.element_count*sizeof(ShellBatchSectionState);
  if(!Disjoint(output,bytes,&expected,sizeof expected))
    return {BatchStatus::InvalidInput,"T3 section readback output is missing or overlaps its receipt"};
  auto r=s.PendingError(); if(r.status!=BatchStatus::Success) return r;
  r=s.Runtime(s.plasticity->Read(1u-s.AcceptedSlabIndex(),s.config.element_count,s.stream),"T3 section readback failed");
  if(r.status!=BatchStatus::Success) return r;
  std::memcpy(output,s.plasticity->staging(),bytes);
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::t3
