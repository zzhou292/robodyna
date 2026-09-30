// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchPublicationImpl.h"
#include "ShellFormulationOutputRanges.h"
#include "type25/Type25BatchStorage.h"

namespace tl::fea {
namespace {
template<class Family,class Usage>
bool SameFamily(const Family& family,std::size_t count,const ShellFormulationScope& scope,
    const qbat::BatchConfig& config,const NodalStamp& accepted,cudaStream_t stream,Usage coupled) noexcept {
  const auto* failure=family.plasticity?family.plasticity->failure_binding():nullptr;
  return family.formulations&&family.joined_binding&&family.joined_binding->inventory()==scope.binding->inventory()&&
      count>0&&family.config.element_count==count&&family.plasticity&&failure&&failure->SameScope(*scope.failure)&&
      bool(family.joined_mass)==bool(scope.mass)&&(!scope.mass||family.joined_mass->Matches(*scope.mass))&&
      family.config.configuration_id==config.configuration_id&&family.config.qualification_id==config.qualification_id&&
      (family.config.usage==coupled)==(config.usage==qbat::BatchUsage::CoupledForces)&&
      shell_startup_detail::SameStartup(family.config.startup,config.startup)&&
      trial_identity::SameStamp(family.config.owner,config.owner)&&
      trial_identity::SameStamp(family.accepted_stamp,accepted)&&family.stream==stream;
}
template<class Family,class Diagnostics,class Equal>
bool SameCandidate(const Family& family,const Diagnostics& diagnostics,
    const NodalPreparedView& authentic,const NodalStamp& accepted,bool coupled,Equal equal) noexcept {
  return family.usable&&family.bound&&family.pending&&
      trial_identity::SameStamp(family.accepted_stamp,accepted)&&
      equal(diagnostics,family.candidate_diagnostics)&&UnavailableKinetic(diagnostics)&&
      diagnostics.accepted_force_assembled==coupled&&
      trial_identity::SamePrepared(authentic,family.candidate_view);
}
}
bool ShellBatchPublication::Impl::FormulationOutputDisjoint(const void* output,std::size_t bytes) const noexcept {
  using trial_identity::Disjoint;
  if(!bbatch||!bbatch->impl_||!bbatch->impl_->OutputDisjoint(output,bytes)||
      !Disjoint(output,bytes,this,sizeof(*this))||!Disjoint(output,bytes,scope,sizeof(*scope))||
      !Disjoint(output,bytes,bbatch,sizeof(*bbatch))) return false;
  if(qbatch) {
    if(!qbatch->impl_) return false;
    const auto& q=*qbatch->impl_;
    const auto* failure=q.plasticity?q.plasticity->failure_binding():nullptr;
    const ShellFormulationScope source{q.joined_binding?&*q.joined_binding:nullptr,
        failure?failure->catalog():nullptr,failure,
        q.joined_mass?&*q.joined_mass:nullptr};
    if(!Disjoint(output,bytes,qbatch,sizeof(*qbatch))||!Disjoint(output,bytes,&q,sizeof(q))||
        !shell_formulation_detail::OutputDisjoint(source,output,bytes)) return false;
  }
  if(tbatch) {
    if(!tbatch->impl_) return false;
    const auto& t=*tbatch->impl_;
    const auto* failure=t.plasticity?t.plasticity->failure_binding():nullptr;
    const ShellFormulationScope source{t.joined_binding?&*t.joined_binding:nullptr,
        failure?failure->catalog():nullptr,failure,
        t.joined_mass?&*t.joined_mass:nullptr};
    if(!Disjoint(output,bytes,tbatch,sizeof(*tbatch))||!Disjoint(output,bytes,&t,sizeof(t))||
        !shell_formulation_detail::OutputDisjoint(source,output,bytes)) return false;
  }
  return !connector||(connector->impl_&&Disjoint(output,bytes,connector,sizeof(*connector))&&
      connector->impl_->OutputDisjoint(output,bytes));
}
bool ShellBatchPublication::Impl::SameFormulationScope() const noexcept {
  if(!bbatch||!bbatch->impl_) return false;
  const auto& b=*bbatch->impl_;
  const auto source=b.Scope();
  if(ValidateShellFormulationScope(source).status!=ShellPlasticityBindingStatus::Success||
      bool(qbatch)!=(source.binding->qeph_count()!=0)||bool(tbatch)!=(source.binding->t3_count()!=0)||
      bool(connector)!=bool(source.mass)||(connector&&b.config.usage!=qbat::BatchUsage::CoupledForces)||
      b.config.element_count!=source.binding->qbat_count()) return false;
  if(qbatch&&(!qbatch->impl_||!SameFamily(*qbatch->impl_,source.binding->qeph_count(),source,
      b.config,b.accepted_stamp,b.stream,qeph::BatchUsage::CoupledForces))) return false;
  if(tbatch&&(!tbatch->impl_||!SameFamily(*tbatch->impl_,source.binding->t3_count(),source,
      b.config,b.accepted_stamp,b.stream,t3::BatchUsage::CoupledForces))) return false;
  return true;
}
ShellPublicationReport ShellBatchPublication::Impl::PreflightFormulations(FENodalState& owner,
    const NodalTrialToken& token,const ShellFormulationCandidates& candidates,NodalPreparedView& authentic) noexcept {
  if(!usable||!bbatch||!bbatch->impl_||!bbatch->impl_->usable||
      (qbatch&&(!qbatch->impl_||!qbatch->impl_->usable))||
      (tbatch&&(!tbatch->impl_||!tbatch->impl_->usable))) {
    return {S::DeviceFailure,"A required formulation participant is unavailable or poisoned"};
  }
  if(!SameFormulationScope()||bbatch->impl_->publication_scope!=scope||
      (qbatch&&qbatch->impl_->publication_scope!=scope)||(tbatch&&tbatch->impl_->publication_scope!=scope)||
      bool(candidates.qeph)!=bool(qbatch)||bool(candidates.t3)!=bool(tbatch)||
      !candidates.qbat||bool(candidates.connector)!=bool(connector)) {
    return {S::NotJoined,"Candidate presence or complete formulation publication scope differs"};
  }
  const auto borrowed=owner.BorrowPrepared(token,&authentic);
  if(borrowed.status!=NodalStatus::Ok) return Nodal(borrowed);
  const auto& b=*bbatch->impl_;
  auto checked=Qbat(bbatch->PreflightPublication(owner,token,authentic,*candidates.qbat,scope));
  if(checked.status!=S::Success) return checked;
  const bool coupled=b.config.usage==qbat::BatchUsage::CoupledForces;
  if(candidates.qbat->accepted_force_assembled!=coupled||
      (qbatch&&!SameCandidate(*qbatch->impl_,*candidates.qeph,authentic,b.accepted_stamp,coupled,
          qeph::batch_detail::SameDiagnostics))||
      (tbatch&&!SameCandidate(*tbatch->impl_,*candidates.t3,authentic,b.accepted_stamp,coupled,
          t3::batch_detail::SameDiagnostics))) {
    return {S::StaleTrial,"Complete typed formulation candidates differ from the authenticated owner interval"};
  }
  if(!b.accepted_stamp.epoch) {
    if(qbatch) {
      const auto initial=owner.ValidateAcceptedAssemblySources(qbatch->impl_->initial_sources);
      if(initial.status!=NodalStatus::Ok) return Nodal(initial);
    }
    if(tbatch) {
      const auto initial=owner.ValidateAcceptedAssemblySources(tbatch->impl_->initial_sources);
      if(initial.status!=NodalStatus::Ok) return Nodal(initial);
    }
  }
  if(connector) return Connector(connector->PreflightPublication(owner,token,authentic,*candidates.connector,scope));
  return Ok();
}
} // namespace tl::fea
