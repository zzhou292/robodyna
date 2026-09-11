// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchPublicationImpl.h"
#include "qeph/QephBatchStorage.h"
#include "t3/T3BatchStorage.h"
#include "ShellBatchStartup.h"
#include <stdexcept>
#include <cstring>
#include <new>

namespace tl::fea {
using namespace shell_publication_detail;



ShellBatchPublication::ShellBatchPublication()=default;
ShellBatchPublication::~ShellBatchPublication() {
  if(!impl_) return;
  // Borrowed participants must outlive this coordinator, including destruction.
  if(impl_->qbatch&&impl_->qbatch->impl_->publication_scope==this) impl_->qbatch->impl_->publication_scope=nullptr;
  if(impl_->tbatch&&impl_->tbatch->impl_->publication_scope==this) impl_->tbatch->impl_->publication_scope=nullptr;
  if(impl_->connector) impl_->connector->ReleasePublication(this);
  if(impl_->bbatch) impl_->bbatch->ReleasePublication(this);
}
ShellPublicationReport ShellBatchPublication::Initialize(FENodalState& owner,qeph::QephBatch& q,t3::T3Batch& t,
    const ShellPublicationLimits& limits) { return InitializeImpl(owner,q,t,nullptr,limits); }
ShellPublicationReport ShellBatchPublication::Initialize(FENodalState& owner,qeph::QephBatch& q,t3::T3Batch& t,
    type25::Batch& connector,const ShellPublicationLimits& limits) {
  return InitializeImpl(owner,q,t,&connector,limits);
}
ShellPublicationReport ShellBatchPublication::InitializeImpl(FENodalState& owner,qeph::QephBatch& q,t3::T3Batch& t,
    type25::Batch* connector,
    const ShellPublicationLimits& limits) try {
  if(impl_) return {S::InvalidInput,"Mixed publication scope is already initialized"};
  if(!q.impl_||!t.impl_) return {S::NotInitialized,"Both shell participants must be initialized"};
  if(!q.impl_->joined_binding||!t.impl_->joined_binding) return {S::NotJoined,"Both shell participants must use InitializeJoined"};
  if(bool(q.impl_->joined_mass)!=bool(connector)||bool(t.impl_->joined_mass)!=bool(connector))
    return {S::NotJoined,"Combined nodal mass requires the connector publication participant"};
  const auto count=q.impl_->joined_binding->node_count();
  const bool vehicle=limits.profile==ShellResidentProfile::Vehicle;
  const auto ceiling=vehicle?ShellPublicationLimits::Vehicle():ShellPublicationLimits{MaxShellResidentNodes,
    MaxShellResidentDeviceBytes,MaxShellResidentHostBytes};
  shell_publication_detail::Layout layout;
  if((limits.profile!=ShellResidentProfile::Legacy&&!vehicle)||
     !limits.max_nodes||limits.max_nodes>ceiling.max_nodes||count>limits.max_nodes||
     !limits.max_device_bytes||limits.max_device_bytes>ceiling.max_device_bytes||
     !limits.max_host_bytes||limits.max_host_bytes>ceiling.max_host_bytes||
     !layout.Initialize(count,limits.max_device_bytes,connector!=nullptr))
    return {S::ResourceLimit,"Mixed publication active-node/device capacity exceeded"};
  util::BoundedArenaLayout host_budget(limits.max_host_bytes); util::ArenaRegion ignored;
  if(!host_budget.Append<unsigned char>(sizeof(Impl),ignored)||!host_budget.Append<unsigned char>(layout.bytes,ignored)||
     !host_budget.Append<double>(13*count,ignored))
    return {S::ResourceLimit,"Mixed publication startup payload exceeds host cap"};
  std::unique_ptr<Impl> next(new(std::nothrow) Impl);
  if(!next) return {S::ResourceLimit,"Mixed publication host allocation failed"};
  next->qbatch=&q; next->tbatch=&t; next->connector=connector; next->scope=this; next->layout=layout;
  if(q.impl_->publication_scope||t.impl_->publication_scope||!next->SameScope()||!q.impl_->bound||!t.impl_->bound||q.impl_->pending||t.impl_->pending||
     q.impl_->accepted_stamp.epoch||!q.impl_->usable||!t.impl_->usable)
    return {S::InvalidInput,"Mixed initialization requires same-scope bound epoch-zero participants"};
  if(!trial_identity::SameStamp(owner.accepted(),q.impl_->accepted_stamp))
    return {S::StaleTrial,"Mixed initialization requires the actual accepted owner"};
  const auto binding_check=next->InitialSources(owner);
  if(binding_check.status!=S::Success) {
    if(binding_check.nodal_status==NodalStatus::DeviceFailure) next->Poison();
    return binding_check;
  }
  if(!UnavailableKinetic(q.impl_->accepted_diagnostics)||!UnavailableKinetic(t.impl_->accepted_diagnostics))
    return {S::InvalidInput,"Joined initial participants must not publish duplicate kinetic energy"};
  if(connector) {
    const auto admitted=Connector(connector->PreflightAttach(owner.accepted(),*q.impl_->joined_mass,
      q.impl_->config.configuration_id,q.impl_->config.qualification_id,q.impl_->config.startup,this));
    if(admitted.status!=S::Success) return admitted;
    const auto copied=Connector(connector->CopyAcceptedDiagnostics(owner.accepted(),&next->accepted.connector));
    if(copied.status!=S::Success) return copied;
    next->accepted.has_connector=true;
  }
  ShellBatchKinetic initial_kinetic;
  if(q.impl_->config.startup.kind==ShellBatchStartupKind::ReferenceUniformTranslation) {
    const auto measured=InitialMovingKinetic(owner,*q.impl_->joined_binding,
      q.impl_->joined_mass?&*q.impl_->joined_mass:nullptr,q.impl_->config.startup,q.impl_->accepted_stamp,initial_kinetic);
    if(measured.status!=S::Success) {
      if(measured.nodal_status==NodalStatus::DeviceFailure) next->Poison();
      return measured;
    }
  }
  util::HostArena arena;
  if(!arena.Initialize(layout.bytes)) return {S::ResourceLimit,"Mixed kinetic staging allocation failed"};
  auto* initial=layout.Construct(arena);
  if(!initial) return {S::ResourceLimit,"Mixed kinetic startup layout is invalid"};
  const auto& binding=*q.impl_->joined_binding;
  PopulateKineticModel(binding,connector?&*q.impl_->joined_mass:nullptr,initial->model);
  auto r=next->Runtime(cudaGetLastError(),"Pending CUDA error before mixed publication initialization");
  if(r.status!=S::Success) return r;
  r=next->Runtime(cudaMalloc(reinterpret_cast<void**>(&next->storage),layout.bytes),"Mixed kinetic allocation failed");
  if(r.status!=S::Success) return r;
  *initial=layout.Rebase(*initial,next->storage);
  r=next->Runtime(cudaMemcpy(next->storage,arena.data(),layout.bytes,cudaMemcpyHostToDevice),"Mixed kinetic initialization failed");
  if(r.status!=S::Success) return r;
  next->accepted.qeph=q.impl_->accepted_diagnostics; next->accepted.t3=t.impl_->accepted_diagnostics;
  next->accepted.kinetic=initial_kinetic;
  // Epoch zero has no completed interval/base kinetic. Rest remains known zero;
  // moving K0 is measured once from fresh owner fields, without h=0 evaluation.
  next->accepted.valid=true;
  q.impl_->publication_scope=this; t.impl_->publication_scope=this;
  if(connector) connector->AttachPublication(this);
  impl_=std::move(next); return Ok();
} catch(const std::bad_alloc&) { return {S::ResourceLimit,"Mixed publication host allocation failed"}; }
  catch(const std::length_error&) { return {S::ResourceLimit,"Mixed publication host size overflow"}; }
ShellPublicationReport ShellBatchPublication::Prepare(FENodalState& owner,const NodalTrialToken& token,
    const qeph::BatchDiagnostics& q,const t3::BatchDiagnostics& t,ShellBatchDiagnostics* output) {
  return PrepareImpl(owner,token,q,t,nullptr,output);
}
ShellPublicationReport ShellBatchPublication::Prepare(FENodalState& owner,const NodalTrialToken& token,
    const qeph::BatchDiagnostics& q,const t3::BatchDiagnostics& t,
    const type25::BatchDiagnostics& connector,ShellBatchDiagnostics* output) {
  return PrepareImpl(owner,token,q,t,&connector,output);
}
ShellPublicationReport ShellBatchPublication::PrepareImpl(FENodalState& owner,const NodalTrialToken& token,
    const qeph::BatchDiagnostics& q,const t3::BatchDiagnostics& t,
    const type25::BatchDiagnostics* connector,ShellBatchDiagnostics* output) {
  auto fail=[&](ShellPublicationReport r) {
    if(impl_&&(r.status==S::DeviceFailure||r.nodal_status==NodalStatus::DeviceFailure)) impl_->Poison();
    owner.Discard(); DiscardTrial(); return r;
  };
  if(!impl_) return fail({S::NotInitialized,"Mixed publication scope is not initialized"});
  if(impl_->bbatch) return fail({S::NotJoined,"QBAT requires the complete typed formulation candidate"});
  auto& s=*impl_; s.pending=false; s.candidate={}; s.candidate_view={};
  if(!trial_identity::Disjoint(output,sizeof(*output),&q,sizeof(q))||
     !trial_identity::Disjoint(output,sizeof(*output),&t,sizeof(t))||
     (connector&&!trial_identity::Disjoint(output,sizeof(*output),connector,sizeof(*connector))))
    return fail({S::InvalidInput,"Mixed diagnostic output is missing or overlaps typed inputs"});
  NodalPreparedView authentic;
  auto r=s.Preflight(owner,token,q,t,connector,authentic); if(r.status!=S::Success) return fail(r);
  ShellBatchDiagnostics next; next.qeph=q; next.t3=t;
  if(connector) { next.connector=*connector; next.has_connector=true; }
  r=s.MeasureCandidate(authentic,next); if(r.status!=S::Success) return fail(r);
  s.candidate=next; s.candidate_view=authentic; s.pending=true; *output=next;
  return Ok();
}
ShellPublicationReport ShellBatchPublication::Commit(FENodalState& owner,const NodalTrialToken& token,
    const ShellBatchDiagnostics& expected,const NodalValidationReceipt& receipt) noexcept {
  auto fail=[&](ShellPublicationReport r) {
    if(impl_&&(r.status==S::DeviceFailure||r.nodal_status==NodalStatus::DeviceFailure)) impl_->Poison();
    owner.Discard(); DiscardTrial(); return r;
  };
  if(!impl_) return fail({S::NotInitialized,"Mixed publication scope is not initialized"});
  auto& s=*impl_;
  if(!s.pending||!SameDiagnostics(expected,s.candidate))
    return fail({S::StaleTrial,"Mixed publication does not match its measured complete candidate"});
  NodalPreparedView authentic;
  const ShellFormulationCandidates candidates{expected.has_qeph?&expected.qeph:nullptr,
      expected.has_t3?&expected.t3:nullptr,expected.has_qbat?&expected.qbat:nullptr,
      expected.has_connector?&expected.connector:nullptr};
  auto r=s.bbatch?s.PreflightFormulations(owner,token,candidates,authentic):
      s.Preflight(owner,token,expected.qeph,expected.t3,candidates.connector,authentic);
  if(r.status!=S::Success) return fail(r);
  const auto owner_id=s.bbatch?expected.qbat.owner_id:expected.qeph.owner_id;
  const auto base_epoch=s.bbatch?expected.qbat.base_epoch:expected.qeph.base_epoch;
  const auto attempt=s.bbatch?expected.qbat.attempt:expected.qeph.attempt;
  const auto qualification=s.bbatch?expected.qbat.qualification_id:expected.qeph.qualification_id;
  if(!trial_identity::SamePrepared(authentic,s.candidate_view)||!receipt.passed||
     receipt.owner_id!=owner_id||receipt.base_epoch!=base_epoch||
     receipt.attempt!=attempt||receipt.qualification_id!=qualification)
    return fail({S::StaleTrial,"Mixed receipt or authentic measured token mismatch"});
  // Do not drain pending CUDA errors here. The sole owner must observe and
  // poison them before any accepted state or material slab changes.
  auto nodal=CompleteNodalValidation(owner,token,receipt);
  if(nodal.status==NodalStatus::Ok) nodal=owner.Commit(token);
  if(nodal.status!=NodalStatus::Ok) {
    if(nodal.status==NodalStatus::DeviceFailure) s.Poison();
    return fail(Nodal(nodal));
  }
  // Infallible publication only below: no CUDA, allocation, readback, numerical
  // checks, callbacks or independently committing participant wrappers.
  const auto stamp=owner.accepted();
  if(s.qbatch) s.qbatch->impl_->Publish(stamp);
  if(s.tbatch) s.tbatch->impl_->Publish(stamp);
  if(s.bbatch) s.bbatch->Publish(stamp);
  if(s.connector) s.connector->Publish(stamp);
  s.accepted=s.candidate;
  if(s.qbatch) s.accepted.qeph.phase=qeph::BatchPhase::Accepted;
  if(s.tbatch) s.accepted.t3.phase=t3::BatchPhase::Accepted;
  if(s.bbatch) s.accepted.qbat.phase=qbat::BatchPhase::Accepted;
  if(s.connector) s.accepted.connector.phase=type25::BatchPhase::Accepted;
  s.pending=false; s.candidate={}; s.candidate_view={};
  return {S::Success,"Owner and complete native material/cache slabs published"};
}
ShellPublicationReport ShellBatchPublication::CopyAcceptedDiagnostics(const NodalStamp& expected,
    ShellBatchDiagnostics* output) const noexcept {
  if(!impl_) return {S::NotInitialized,"Mixed publication scope is not initialized"};
  if(impl_->bbatch) return CopyAcceptedFormulations(expected,output);
  const auto& s=*impl_;
  if(!output||!trial_identity::Disjoint(output,sizeof(*output),&expected,sizeof(expected)))
    return {S::InvalidInput,"Mixed accepted diagnostic output is missing or overlaps identity"};
  if(!s.usable||!s.qbatch->impl_->usable||!s.tbatch->impl_->usable) return {S::DeviceFailure,"Mixed publication scope is poisoned"};
  if(s.qbatch->impl_->publication_scope!=this||s.tbatch->impl_->publication_scope!=this||
     !s.SameScope()||!trial_identity::SameStamp(expected,s.qbatch->impl_->accepted_stamp)||
     s.accepted.qeph.epoch!=expected.epoch||s.accepted.t3.epoch!=expected.epoch)
    return {S::StaleTrial,"Mixed accepted diagnostics belong to another endpoint"};
  if(s.connector) {
    type25::BatchDiagnostics actual;
    const auto copied=Connector(s.connector->CopyAcceptedDiagnostics(expected,&actual));
    if(copied.status!=S::Success) return copied;
    if(!type25::batch_detail::SameDiagnostics(actual,s.accepted.connector))
      return {S::StaleTrial,"Connector accepted diagnostics differ from common publication"};
  }
  *output=s.accepted; return Ok();
}
void ShellBatchPublication::DiscardTrial() noexcept { if(impl_) impl_->Discard(); }
NodalAllocationInfo ShellBatchPublication::allocations() const noexcept {
  return impl_?NodalAllocationInfo{impl_->layout.bytes,1}:NodalAllocationInfo{};
}
} // namespace tl::fea
