// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchPublicationImpl.h"

namespace tl::fea {
ShellPublicationReport ShellBatchPublication::Impl::MeasureCandidate(const NodalPreparedView& authentic,
    ShellBatchDiagnostics& next) {
  auto report=Runtime(cudaGetLastError(),"Pending CUDA error before mixed kinetic measurement");
  if(report.status!=S::Success) return report;
  shell_publication_detail::LaunchMeasure(storage,authentic);
  report=Runtime(cudaGetLastError(),"Mixed kinetic kernel launch failed");
  if(report.status!=S::Success) return report;
  report=Runtime(cudaMemcpyAsync(&control,&storage->control,sizeof(control),cudaMemcpyDeviceToHost,authentic.stream),
      "Mixed kinetic readback failed");
  if(report.status!=S::Success) return report;
  report=Runtime(cudaStreamSynchronize(authentic.stream),"Mixed kinetic stream failed");
  if(report.status!=S::Success) return report;
  if(control.status!=S::Success) return {control.status,"Mixed native kinetic reduction is nonfinite"};
  next.base_kinetic=control.base;
  next.kinetic=control.endpoint;
  next.valid=true;
  return Ok();
}
ShellPublicationReport ShellBatchPublication::PrepareFormulations(FENodalState& owner,const NodalTrialToken& token,
    const ShellFormulationCandidates& input,ShellBatchDiagnostics* output) {
  auto fail=[&](ShellPublicationReport report) {
    if(impl_&&(report.status==S::DeviceFailure||report.nodal_status==NodalStatus::DeviceFailure)) impl_->Poison();
    owner.Discard();
    DiscardTrial();
    return report;
  };
  if(!impl_||impl_->physical||!impl_->bbatch) return fail({S::NotJoined,"Explicit QBAT formulation scope is required"});
  auto& s=*impl_;
  using trial_identity::Disjoint;
  if(!s.FormulationOutputDisjoint(output,sizeof(*output))||
      !Disjoint(output,sizeof(*output),this,sizeof(*this))||!Disjoint(output,sizeof(*output),impl_.get(),sizeof(Impl))||
      !Disjoint(output,sizeof(*output),&owner,sizeof(owner))||!Disjoint(output,sizeof(*output),&token,sizeof(token))||
      !Disjoint(output,sizeof(*output),&input,sizeof(input))||
      (input.qeph&&!Disjoint(output,sizeof(*output),input.qeph,sizeof(*input.qeph)))||
      (input.t3&&!Disjoint(output,sizeof(*output),input.t3,sizeof(*input.t3)))||
      (input.qbat&&!Disjoint(output,sizeof(*output),input.qbat,sizeof(*input.qbat)))||
      (input.connector&&!Disjoint(output,sizeof(*output),input.connector,sizeof(*input.connector)))) {
    return fail({S::InvalidInput,"Formulation diagnostic output overlaps an inspected input"});
  }
  s.pending=false;
  s.candidate={};
  s.candidate_view={};
  NodalPreparedView authentic;
  auto checked=s.PreflightFormulations(owner,token,input,authentic);
  if(checked.status!=S::Success) return fail(checked);
  ShellBatchDiagnostics next;
  next.has_qeph=input.qeph!=nullptr;
  next.has_t3=input.t3!=nullptr;
  next.has_qbat=true;
  next.has_connector=input.connector!=nullptr;
  if(input.qeph) next.qeph=*input.qeph;
  if(input.t3) next.t3=*input.t3;
  next.qbat=*input.qbat;
  if(input.connector) next.connector=*input.connector;
  checked=s.MeasureCandidate(authentic,next);
  if(checked.status!=S::Success) return fail(checked);
  s.candidate=next;
  s.candidate_view=authentic;
  s.pending=true;
  *output=next;
  return Ok();
}
ShellPublicationReport ShellBatchPublication::CopyAcceptedFormulations(const NodalStamp& expected,
    ShellBatchDiagnostics* output) const noexcept {
  const auto& s=*impl_;
  if(!s.FormulationOutputDisjoint(output,sizeof(*output))||
      !trial_identity::Disjoint(output,sizeof(*output),&expected,sizeof(expected))||
      !trial_identity::Disjoint(output,sizeof(*output),this,sizeof(*this))||
      !trial_identity::Disjoint(output,sizeof(*output),impl_.get(),sizeof(Impl))) {
    return {S::InvalidInput,"Formulation accepted output overlaps identity or owned data"};
  }
  if(!s.usable||!s.bbatch->impl_->usable||(s.qbatch&&!s.qbatch->impl_->usable)||
      (s.tbatch&&!s.tbatch->impl_->usable)) return {S::DeviceFailure,"Formulation publication scope is poisoned"};
  if(!s.SameFormulationScope()||s.bbatch->impl_->publication_scope!=this||
      (s.qbatch&&s.qbatch->impl_->publication_scope!=this)||(s.tbatch&&s.tbatch->impl_->publication_scope!=this)||
      !trial_identity::SameStamp(expected,s.bbatch->impl_->accepted_stamp)||
      !qbat::batch_detail::SameDiagnostics(s.accepted.qbat,s.bbatch->impl_->accepted_diagnostics)||
      (s.qbatch&&!qeph::batch_detail::SameDiagnostics(s.accepted.qeph,s.qbatch->impl_->accepted_diagnostics))||
      (s.tbatch&&!t3::batch_detail::SameDiagnostics(s.accepted.t3,s.tbatch->impl_->accepted_diagnostics))) {
    return {S::StaleTrial,"Formulation accepted diagnostics belong to another complete endpoint"};
  }
  if(s.connector) {
    type25::BatchDiagnostics actual;
    const auto checked=Connector(s.connector->CopyAcceptedDiagnostics(expected,&actual));
    if(checked.status!=S::Success) return checked;
    if(!type25::batch_detail::SameDiagnostics(actual,s.accepted.connector)) {
      return {S::StaleTrial,"Connector accepted cache differs from common publication"};
    }
  }
  *output=s.accepted;
  return Ok();
}
} // namespace tl::fea
