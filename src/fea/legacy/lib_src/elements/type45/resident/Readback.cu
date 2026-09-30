// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "ResultChecks.h"

namespace tl::fea::type45 {
BatchReport Batch::Impl::ReadResults(unsigned slab,const BatchDiagnostics& d) {
  auto r=PendingError();if(!r) return r;
  if(slab>1 || !d.valid || d.joint_count!=model.joints().size() ||
      d.automatic_stiffness_initialized!=bool(d.epoch))
    return {BatchStatus::InvalidInput,"Joint result slab or completeness differs"};
  auto* values=util::ArenaPointer<resident_detail::State>(staging.data(),layout.staging);
  const auto* contexts=util::ArenaPointer<AutomaticStiffnessContext>(staging.data(),layout.host_contexts);
  r=Runtime(cudaMemcpyAsync(values,device_header.slab[slab],layout.staging.bytes,
      cudaMemcpyDeviceToHost,stream),"Joint complete state readback failed");
  if(r) r=Runtime(cudaStreamSynchronize(stream),"Joint state readback stream failed");
  if(!r) return r;
  for(std::size_t j=0;j<model.joints().size();++j) {
    const auto& row=model.joints()[j];
    if(!resident_detail::ValidResult(row,values[j],d.time,d.epoch,config.owner.fixed_dt))
      return {BatchStatus::NonfiniteResult,"Joint state/source/phase validation failed",j};
    if(d.epoch) {
      Reference expected;
      if(Reference::Prepare(row.property,row.geometry,row.damping,contexts[j],expected)!=Status::Success ||
          !values[j].history.Matches(expected))
        return {BatchStatus::InvalidInput,"Joint accepted automatic context differs",j};
    }
  }
  return {};
}
void Batch::Impl::PublishResults(ResultBuffer output) const noexcept {
  const auto* values=util::ArenaPointer<resident_detail::State>(staging.data(),layout.staging);
  // ReadResults already validated every named field before caller publication.
  for(std::size_t j=0;j<output.count;++j) resident_detail::Export(model.joints()[j],values[j],output.joints[j]);
}
BatchReport Batch::CopyAcceptedResults(const NodalStamp& expected,ResultBuffer output,BatchDiagnostics* diagnostics) {
  if(!impl_) return {BatchStatus::NotInitialized,"Joint batch is not initialized"};
  auto& s=*impl_;
  if(!s.bound) return {BatchStatus::NotBound,"Joint common publication claim required"};
  if(!trial_identity::SameStamp(expected,s.accepted_stamp)) return {BatchStatus::StaleTrial,"Joint accepted stamp differs"};
  using trial_identity::Disjoint;
  if(reinterpret_cast<std::uintptr_t>(diagnostics)%alignof(BatchDiagnostics) ||
      !s.OutputBuffer(output,&expected,sizeof(expected),this,sizeof(*this)) ||
      !s.OutputBuffer(output,diagnostics,sizeof(*diagnostics),this,sizeof(*this)) ||
      !s.OutputDisjoint(diagnostics,sizeof(*diagnostics)) ||
      !Disjoint(diagnostics,sizeof(*diagnostics),&expected,sizeof(expected)) ||
      !Disjoint(diagnostics,sizeof(*diagnostics),this,sizeof(*this)))
    return {BatchStatus::InvalidInput,"Joint readback counts or ranges overlap"};
  const auto r=s.ReadResults(s.accepted_slab,s.accepted_diagnostics);if(!r) return r;
  s.PublishResults(output);*diagnostics=s.accepted_diagnostics;return {};
}
BatchReport Batch::CopyPreparedResults(const BatchDiagnostics& expected,ResultBuffer output) {
  if(!impl_) return {BatchStatus::NotInitialized,"Joint batch is not initialized"};
  auto& s=*impl_;
  if(!s.bound || !s.pending || !resident_detail::SameDiagnostics(expected,s.candidate_diagnostics))
    return {BatchStatus::StaleTrial,"Joint prepared identity differs"};
  if(!s.OutputBuffer(output,&expected,sizeof(expected),this,sizeof(*this)))
    return {BatchStatus::InvalidInput,"Joint readback counts or ranges overlap"};
  const auto r=s.ReadResults(s.TrialSlab(),s.candidate_diagnostics);if(!r) return r;
  s.PublishResults(output);return {};
}
} // namespace tl::fea::type45
