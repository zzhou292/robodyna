#include "QephBatchStorage.h"
#include <cstring>
#include <new>

namespace tl::fea::qeph {
BatchReport QephBatch::Impl::InitializePlasticity(const ShellBatchPlasticityConfig& declaration,
    const batch_detail::Model& model) {
  using namespace shell_batch_plasticity_detail;
  std::array<ReferenceMaterial,MaxBatchElements> references{};
  for(std::size_t e=0;e<config.element_count;++e) {
    const auto& r=model.element[e].reference.input;
    references[e]={r.young_modulus,r.poisson_ratio,r.density};
  }
  std::unique_ptr<HostStorage> next(new(std::nothrow) HostStorage);
  if(!next) return {BatchStatus::ResourceLimit,"QEPH plastic section host allocation failed"};
  const auto setup=next->Initialize(declaration,references.data(),config.element_count,
      config.max_device_bytes-sizeof(batch_detail::Storage));
  if(setup.status==SetupStatus::DeviceFailure) return Runtime(setup.cuda_status,setup.message);
  if(setup.status!=SetupStatus::Success)
    return {setup.status==SetupStatus::ResourceLimit?BatchStatus::ResourceLimit:BatchStatus::InvalidInput,setup.message};
  plasticity=std::move(next);
  return {BatchStatus::Success,"OK"};
}
BatchReport QephBatch::CopyAcceptedSectionHistory(const NodalStamp& expected,ShellBatchSectionState* output,
    std::size_t capacity,BatchDiagnostics* diagnostics) {
  using trial_identity::Disjoint;
  if(!impl_) return {BatchStatus::NotInitialized,"QEPH batch is not initialized"};
  auto& s=*impl_;
  if(!s.plasticity) return {BatchStatus::InvalidInput,"QEPH has no plastic section history"};
  if(!s.bound) return {BatchStatus::NotBound,"Initial QEPH source binding is required"};
  if(!batch_detail::SameStamp(expected,s.accepted_stamp))
    return {BatchStatus::StaleTrial,"Accepted QEPH section endpoint identity mismatch"};
  if(capacity<s.config.element_count) return {BatchStatus::ResourceLimit,"QEPH section readback capacity is insufficient"};
  const auto bytes=s.config.element_count*sizeof(ShellBatchSectionState);
  if(!Disjoint(output,bytes,diagnostics,sizeof(*diagnostics))||!Disjoint(output,bytes,&expected,sizeof expected)||
     !Disjoint(diagnostics,sizeof(*diagnostics),&expected,sizeof expected))
    return {BatchStatus::InvalidInput,"QEPH section readback output is missing or overlaps inputs"};
  auto r=s.PendingError(); if(r.status!=BatchStatus::Success) return r;
  r=s.Runtime(s.plasticity->Read(s.AcceptedSlabIndex(),s.config.element_count,s.stream),"QEPH section readback failed");
  if(r.status!=BatchStatus::Success) return r;
  std::memcpy(output,s.plasticity->staging(),bytes); *diagnostics=s.accepted_diagnostics;
  return {BatchStatus::Success,"OK"};
}
BatchReport QephBatch::CopyPreparedSectionHistory(const BatchDiagnostics& expected,ShellBatchSectionState* output,
    std::size_t capacity) {
  using trial_identity::Disjoint;
  if(!impl_) return {BatchStatus::NotInitialized,"QEPH batch is not initialized"};
  auto& s=*impl_;
  if(!s.plasticity) return {BatchStatus::InvalidInput,"QEPH has no plastic section history"};
  if(!s.bound) return {BatchStatus::NotBound,"Initial QEPH source binding is required"};
  if(!s.pending||!batch_detail::SameDiagnostics(expected,s.candidate_diagnostics))
    return {BatchStatus::StaleTrial,"Prepared QEPH section identity mismatch"};
  if(capacity<s.config.element_count) return {BatchStatus::ResourceLimit,"QEPH section readback capacity is insufficient"};
  const auto bytes=s.config.element_count*sizeof(ShellBatchSectionState);
  if(!Disjoint(output,bytes,&expected,sizeof expected))
    return {BatchStatus::InvalidInput,"QEPH section readback output is missing or overlaps its receipt"};
  auto r=s.PendingError(); if(r.status!=BatchStatus::Success) return r;
  r=s.Runtime(s.plasticity->Read(1u-s.AcceptedSlabIndex(),s.config.element_count,s.stream),"QEPH section readback failed");
  if(r.status!=BatchStatus::Success) return r;
  std::memcpy(output,s.plasticity->staging(),bytes);
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::qeph
