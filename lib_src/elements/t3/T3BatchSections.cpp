#include "T3BatchStorage.h"
#include <cstring>

namespace tl::fea::t3 {
namespace {
template<class Impl> BatchReport ReadLayered(Impl& s,unsigned slab,double time,std::uint64_t epoch) {
  auto report=s.PendingError();if(report.status!=BatchStatus::Success)return report;
  const auto read=s.plasticity->ReadSections(slab,s.config.element_count,s.stream,time);
  using Setup=shell_batch_plasticity_detail::SetupStatus;
  if(read.status==Setup::DeviceFailure)return s.Runtime(read.cuda_status,read.message);
  if(read.status!=Setup::Success)
    return {read.status==Setup::NonfiniteResult?BatchStatus::NonfiniteResult:BatchStatus::InvalidInput,read.message};
  if(s.plasticity->one_point_sections()) return s.ValidateOnePointReadback(slab,time,epoch);
  return {BatchStatus::Success,"OK"};
}
}
BatchReport T3Batch::CopyAcceptedLayeredSectionHistory(const NodalStamp& expected,
    ShellBatchLayeredSection* output,std::size_t capacity,BatchDiagnostics* diagnostics) {
  using trial_identity::Disjoint;
  if(!impl_)return {BatchStatus::NotInitialized,"T3 batch is not initialized"};
  auto& s=*impl_;
  if(!s.plasticity||!s.plasticity->heterogeneous_sections())
    return {BatchStatus::InvalidInput,"T3 has no explicit layered section history"};
  if(!s.bound)return {BatchStatus::NotBound,"Initial T3 source binding is required"};
  if(!batch_detail::SameStamp(expected,s.accepted_stamp))
    return {BatchStatus::StaleTrial,"Accepted T3 layered section identity mismatch"};
  if(capacity<s.config.element_count)return {BatchStatus::ResourceLimit,"T3 layered section capacity is insufficient"};
  const auto bytes=s.config.element_count*sizeof(ShellBatchLayeredSection);
  if(!Disjoint(output,bytes,diagnostics,sizeof(*diagnostics))||!Disjoint(output,bytes,&expected,sizeof expected)||
     !Disjoint(diagnostics,sizeof(*diagnostics),&expected,sizeof expected)||
     !Disjoint(output,bytes,this,sizeof(*this))||!Disjoint(diagnostics,sizeof(*diagnostics),this,sizeof(*this)))
    return {BatchStatus::InvalidInput,"T3 layered section outputs are missing or overlap inputs"};
  const auto report=ReadLayered(s,s.AcceptedSlabIndex(),s.accepted_diagnostics.time,s.accepted_stamp.epoch);if(report.status!=BatchStatus::Success)return report;
  std::memcpy(output,s.plasticity->section_staging(),bytes);*diagnostics=s.accepted_diagnostics;
  return {BatchStatus::Success,"OK"};
}
BatchReport T3Batch::CopyPreparedLayeredSectionHistory(const BatchDiagnostics& expected,
    ShellBatchLayeredSection* output,std::size_t capacity) {
  using trial_identity::Disjoint;
  if(!impl_)return {BatchStatus::NotInitialized,"T3 batch is not initialized"};
  auto& s=*impl_;
  if(!s.plasticity||!s.plasticity->heterogeneous_sections())
    return {BatchStatus::InvalidInput,"T3 has no explicit layered section history"};
  if(!s.bound)return {BatchStatus::NotBound,"Initial T3 source binding is required"};
  if(!s.pending||!batch_detail::SameDiagnostics(expected,s.candidate_diagnostics))
    return {BatchStatus::StaleTrial,"Prepared T3 layered section identity mismatch"};
  if(capacity<s.config.element_count)return {BatchStatus::ResourceLimit,"T3 layered section capacity is insufficient"};
  const auto bytes=s.config.element_count*sizeof(ShellBatchLayeredSection);
  if(!Disjoint(output,bytes,&expected,sizeof expected)||!Disjoint(output,bytes,this,sizeof(*this)))
    return {BatchStatus::InvalidInput,"T3 layered section output is missing or overlaps its receipt"};
  const auto report=ReadLayered(s,1u-s.AcceptedSlabIndex(),s.candidate_diagnostics.time,s.candidate_diagnostics.epoch);
  if(report.status!=BatchStatus::Success) { s.Discard();return report; }
  std::memcpy(output,s.plasticity->section_staging(),bytes);return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::t3
