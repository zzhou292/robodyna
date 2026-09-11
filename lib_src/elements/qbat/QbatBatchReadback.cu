// SPDX-License-Identifier: AGPL-3.0-or-later
#include "QbatBatchStorage.h"
#include "QbatBatchResultChecks.h"
#include "../ShellFormulationOutputRanges.h"
#include "../ShellPhysicalOutputRanges.h"
#include <cstring>

namespace tl::fea::qbat {
bool Batch::Impl::OutputDisjoint(const void* output,std::size_t bytes) const noexcept {
  using trial_identity::Disjoint;
  if(!Disjoint(output,bytes,this,sizeof(*this))||
      !Disjoint(output,bytes,staging.get(),config.element_count*sizeof(BatchResult))||
      !Disjoint(output,bytes,Scope().binding->nodes().data(),Scope().binding->node_count()*sizeof(ShellBindingNode))) return false;
  return physical ? shell_physical_owner::OutputDisjoint(*physical,output,bytes)
      : shell_formulation_detail::OutputDisjoint(Scope(),output,bytes);
}
BatchReport Batch::Impl::ReadResults(const batch_detail::Slab* source,const BatchDiagnostics& expected) {
  auto report=PendingError();
  if(report.status!=BatchStatus::Success) return report;
  const unsigned slab=source==&storage->slab[0]?0u:1u;
  if(source!=&storage->slab[slab]) return {BatchStatus::InvalidInput,"Unknown QBAT result slab"};
  report=Runtime(cudaMemcpyAsync(staging.get(),device_header.slab[slab].element,
      config.element_count*sizeof(BatchResult),cudaMemcpyDeviceToHost,stream),"QBAT result readback failed");
  if(report.status!=BatchStatus::Success) return report;
  report=Runtime(cudaStreamSynchronize(stream),"QBAT result readback stream failed");
  if(report.status!=BatchStatus::Success) return report;
  for(std::size_t parent=0;parent<config.element_count;++parent) {
    Material material;
    if(!Failure().catalog()->Parameters(ShellBindingFamily::Qbat,parent,&material)||
        !batch_detail::ValidResult(staging[parent],material,expected.time,expected.epoch)) {
      Discard();
      return {BatchStatus::NonfiniteResult,"QBAT readback contains an invalid four-point result",
          static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}
BatchReport Batch::CopyAcceptedResults(const NodalStamp& expected,BatchResult* output,
    std::size_t capacity,BatchDiagnostics* diagnostics) {
  if(!impl_) return {BatchStatus::NotInitialized,"QBAT batch is not initialized"};
  auto& s=*impl_;
  if(!s.bound) return {BatchStatus::NotBound,"QBAT initial live sources are not bound"};
  if(!trial_identity::SameStamp(expected,s.accepted_stamp)) return {BatchStatus::StaleTrial,"QBAT accepted identity differs"};
  if(capacity<s.config.element_count||capacity>SIZE_MAX/sizeof(BatchResult)) {
    return {BatchStatus::ResourceLimit,"QBAT result capacity is invalid"};
  }
  using trial_identity::Disjoint;
  const auto bytes=s.config.element_count*sizeof(BatchResult);
  if(!s.OutputDisjoint(output,bytes)||!s.OutputDisjoint(diagnostics,sizeof(*diagnostics))||
      !Disjoint(output,bytes,diagnostics,sizeof(*diagnostics))||!Disjoint(output,bytes,&expected,sizeof(expected))||
      !Disjoint(diagnostics,sizeof(*diagnostics),&expected,sizeof(expected))||
      !Disjoint(output,bytes,this,sizeof(*this))||!Disjoint(diagnostics,sizeof(*diagnostics),this,sizeof(*this))) {
    return {BatchStatus::InvalidInput,"QBAT result outputs are missing, overlapping or overflowing"};
  }
  const auto report=s.ReadResults(s.accepted,s.accepted_diagnostics);
  if(report.status!=BatchStatus::Success) return report;
  std::memcpy(output,s.staging.get(),bytes);
  *diagnostics=s.accepted_diagnostics;
  return {BatchStatus::Success,"OK"};
}
BatchReport Batch::CopyAcceptedParentActivity(const NodalStamp& expected,
    std::uint8_t* output,std::size_t capacity,BatchDiagnostics* diagnostics) {
  if (!impl_) return {BatchStatus::NotInitialized,"QBAT batch is not initialized"};
  auto& state = *impl_;
  if (!state.bound) return {BatchStatus::NotBound,"QBAT initial sources and virgin history are not bound"};
  if (capacity != state.config.element_count) {
    return {BatchStatus::ResourceLimit,"QBAT activity needs exact complete family capacity"};
  }
  if (!trial_identity::SameStamp(expected,state.accepted_stamp)) {
    return {BatchStatus::StaleTrial,"QBAT accepted activity endpoint differs"};
  }
  using trial_identity::Disjoint;
  if (!state.OutputDisjoint(output,capacity) || !state.OutputDisjoint(diagnostics,sizeof(*diagnostics)) ||
      !Disjoint(output,capacity,diagnostics,sizeof(*diagnostics)) ||
      !Disjoint(output,capacity,&expected,sizeof(expected)) ||
      !Disjoint(diagnostics,sizeof(*diagnostics),&expected,sizeof(expected)) ||
      !Disjoint(output,capacity,this,sizeof(*this)) ||
      !Disjoint(diagnostics,sizeof(*diagnostics),this,sizeof(*this))) {
    return {BatchStatus::InvalidInput,"QBAT activity output overlaps inspected or owned data"};
  }
  const auto report = state.ReadResults(state.accepted,state.accepted_diagnostics);
  if (report.status != BatchStatus::Success) return report;
  for (std::size_t parent = 0; parent < capacity; ++parent) {
    output[parent] = state.staging[parent].history.element_active ? 1 : 0;
  }
  *diagnostics = state.accepted_diagnostics;
  return {BatchStatus::Success,"Accepted QBAT parent activity copied"};
}
BatchReport Batch::CopyPreparedResults(const BatchDiagnostics& expected,BatchResult* output,std::size_t capacity) {
  if(!impl_) return {BatchStatus::NotInitialized,"QBAT batch is not initialized"};
  auto& s=*impl_;
  if(!s.bound) return {BatchStatus::NotBound,"QBAT initial live sources are not bound"};
  if(!s.pending||!batch_detail::SameDiagnostics(expected,s.candidate_diagnostics)) {
    return {BatchStatus::StaleTrial,"QBAT prepared identity differs"};
  }
  if(capacity<s.config.element_count||capacity>SIZE_MAX/sizeof(BatchResult)) {
    return {BatchStatus::ResourceLimit,"QBAT result capacity is invalid"};
  }
  const auto bytes=s.config.element_count*sizeof(BatchResult);
  if(!s.OutputDisjoint(output,bytes)||!trial_identity::Disjoint(output,bytes,&expected,sizeof(expected))||
      !trial_identity::Disjoint(output,bytes,this,sizeof(*this))) {
    return {BatchStatus::InvalidInput,"QBAT result output overlaps input or owned data"};
  }
  const auto report=s.ReadResults(s.trial,s.candidate_diagnostics);
  if(report.status!=BatchStatus::Success) return report;
  std::memcpy(output,s.staging.get(),bytes);
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::qbat
