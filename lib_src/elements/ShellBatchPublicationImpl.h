// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchPublicationStorage.h"
#include "ShellBatchPublicationValues.h"
#include "qeph/QephBatchStorage.h"
#include "t3/T3BatchStorage.h"
#include "qbat/QbatBatchStorage.h"
#include "publication/PhysicalState.h"

namespace tl::fea {
using namespace shell_publication_detail;
struct ShellBatchPublication::Impl {
  qeph::QephBatch* qbatch=nullptr;
  t3::T3Batch* tbatch=nullptr;
  type25::Batch* connector=nullptr;
  qbat::Batch* bbatch=nullptr;
  const ShellBatchPublication* scope=nullptr;
  shell_publication_detail::Storage* storage=nullptr;
  shell_publication_detail::Layout layout;
  shell_publication_detail::Control control;
  std::shared_ptr<shell_publication_detail::PhysicalState> physical;
  ShellBatchDiagnostics accepted,candidate;
  NodalPreparedView candidate_view;
  bool pending=false,usable=true;
  ~Impl() { if(storage) cudaFree(storage); }
  void Discard() noexcept {
    pending=false; candidate={}; candidate_view={};
    if(qbatch) qbatch->DiscardTrial();
    if(tbatch) tbatch->DiscardTrial();
    if(connector) connector->DiscardTrial();
    if(bbatch) bbatch->DiscardTrial();
    if(physical) {
      physical->candidate={};
      if(physical->beams) physical->beams->DiscardTrial();
      if(physical->solids) physical->solids->DiscardTrial();
      if(physical->joints) physical->joints->DiscardTrial();
    }
  }
  void Poison() noexcept {
    usable=false;
    if(qbatch&&qbatch->impl_) qbatch->impl_->usable=false;
    if(tbatch&&tbatch->impl_) tbatch->impl_->usable=false;
    if(connector) connector->Poison();
    if(bbatch) bbatch->Poison();
    if(physical) {
      if(physical->beams) physical->beams->Poison();
      if(physical->solids) physical->solids->Poison();
      if(physical->joints) physical->joints->Poison();
    }
    Discard();
  }
  ShellPublicationReport Runtime(cudaError_t error,const char* message) noexcept {
    if(error==cudaSuccess) return Ok();
    Poison(); return {S::DeviceFailure,message};
  }
  bool SameFormulationScope() const noexcept;
  bool PhysicalOutputDisjoint(const void*,std::size_t) const noexcept;
  void CapturePhysicalDiagnostics(ShellPhysicalDiagnostics&,bool prepared) const noexcept;
  bool SamePhysicalScope(const NodalStamp&) const noexcept;
  bool PhysicalUsable() const noexcept;
  ShellPublicationReport PreflightPhysical(FENodalState&,const NodalTrialToken&,
      const ShellPhysicalCandidates&,NodalPreparedView&) noexcept;
  void ReleasePhysical() noexcept;
  bool FormulationOutputDisjoint(const void*,std::size_t) const noexcept;
  ShellPublicationReport PreflightFormulations(FENodalState&,const NodalTrialToken&,
      const ShellFormulationCandidates&,NodalPreparedView&) noexcept;
  ShellPublicationReport MeasureCandidate(const NodalPreparedView&,ShellBatchDiagnostics&);
  bool SameScope() const noexcept {
    if(bbatch) return SameFormulationScope();
    if(!qbatch||!tbatch||!qbatch->impl_||!tbatch->impl_) return false;
    const auto& q=*qbatch->impl_; const auto& t=*tbatch->impl_;
    const bool same_usage=(q.config.usage==qeph::BatchUsage::PrescribedFields&&t.config.usage==t3::BatchUsage::PrescribedFields)||
      (q.config.usage==qeph::BatchUsage::CoupledForces&&t.config.usage==t3::BatchUsage::CoupledForces);
    return q.joined_binding&&t.joined_binding&&
      q.joined_binding->qbat_count()==0&&t.joined_binding->qbat_count()==0&&
      q.joined_binding->inventory()==t.joined_binding->inventory()&&
      ((!q.joined_mass&&!t.joined_mass)||(q.joined_mass&&t.joined_mass&&q.joined_mass->Matches(*t.joined_mass)))&&
      bool(connector)==bool(q.joined_mass)&&
      (!connector||q.config.usage==qeph::BatchUsage::CoupledForces)&&
      q.config.element_count==q.joined_binding->qeph_count()&&q.config.element_count>0&&
      t.config.element_count==t.joined_binding->t3_count()&&t.config.element_count>0&&
      q.config.configuration_id==t.config.configuration_id&&
      q.config.qualification_id==t.config.qualification_id&&same_usage&&
      shell_batch_plasticity_detail::SameMaterialScope(q.plasticity,t.plasticity)&&
      shell_startup_detail::SameStartup(q.config.startup,t.config.startup)&&trial_identity::SameStamp(q.config.owner,t.config.owner)&&
      trial_identity::SameStamp(q.accepted_stamp,t.accepted_stamp)&&q.stream==t.stream;
  }
  ShellPublicationReport InitialSources(const FENodalState& owner) const noexcept {
    for(const auto* source:{&qbatch->impl_->initial_sources,&tbatch->impl_->initial_sources}) {
      const auto checked=owner.ValidateAcceptedAssemblySources(*source);
      if(checked.status!=NodalStatus::Ok)
        return {checked.status==NodalStatus::DeviceFailure?S::NodalFailure:S::StaleTrial,
                checked.message,checked.status};
    }
    return Ok();
  }
  ShellPublicationReport Preflight(FENodalState& owner,const NodalTrialToken& token,
      const qeph::BatchDiagnostics& qd,const t3::BatchDiagnostics& td,
      const type25::BatchDiagnostics* cd,NodalPreparedView& authentic) noexcept {
    if(!usable||!qbatch->impl_->usable||!tbatch->impl_->usable) return {S::DeviceFailure,"Mixed shell participant is poisoned"};
    if(qbatch->impl_->publication_scope!=scope||tbatch->impl_->publication_scope!=scope)
      return {S::NotJoined,"Mixed participant belongs to a different publication scope"};
    if(!SameScope()) return {S::NotJoined,"Mixed shell participant inventories or immutable scopes differ"};
    if(bool(connector)!=bool(cd)) return {S::NotJoined,"Complete connector candidate is required by this scope"};
    const auto& q=*qbatch->impl_; const auto& t=*tbatch->impl_;
    const bool coupled=q.config.usage==qeph::BatchUsage::CoupledForces;
    if(!q.bound||!t.bound||!q.pending||!t.pending||
       !trial_identity::SameStamp(owner.accepted(),q.accepted_stamp)||
       !qeph::batch_detail::SameDiagnostics(qd,q.candidate_diagnostics)||
       !t3::batch_detail::SameDiagnostics(td,t.candidate_diagnostics)||
       !UnavailableKinetic(qd)||!UnavailableKinetic(td)||
       qd.accepted_force_assembled!=coupled||td.accepted_force_assembled!=coupled)
      return {S::StaleTrial,"Both complete typed candidates must match the accepted owner and immutable scope"};
    if(q.accepted_stamp.epoch==0) {
      const auto binding=InitialSources(owner);
      if(binding.status!=S::Success) return binding;
    }
    const auto nodal=owner.BorrowPrepared(token,&authentic);
    if(nodal.status!=NodalStatus::Ok) return Nodal(nodal);
    if(!trial_identity::SamePrepared(authentic,q.candidate_view)||
       !trial_identity::SamePrepared(authentic,t.candidate_view))
      return {S::StaleTrial,"Mixed candidate views differ from the owner's actual prepared token"};
    if(connector) return Connector(connector->PreflightPublication(owner,token,authentic,*cd,scope));
    return Ok();
  }
};
} // namespace tl::fea
