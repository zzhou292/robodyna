// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchPublicationStorage.h"
#include "qeph/QephBatchStorage.h"
#include "t3/T3BatchStorage.h"
#include <cstring>
#include <new>

namespace tl::fea {
namespace {
using S=ShellPublicationStatus;
ShellPublicationReport Ok() noexcept { return {S::Success,"OK"}; }
ShellPublicationReport Nodal(const NodalReport& r) noexcept { return {S::NodalFailure,r.message,r.status}; }
bool SameDouble(double a,double b) noexcept { return std::memcmp(&a,&b,sizeof(a))==0; }
bool SameKinetic(const ShellBatchKinetic& a,const ShellBatchKinetic& b) noexcept {
  return SameDouble(a.translation,b.translation)&&SameDouble(a.rotation,b.rotation)&&
    SameDouble(a.physical_isotropic,b.physical_isotropic)&&SameDouble(a.added_isotropic,b.added_isotropic);
}
bool SameDiagnostics(const ShellBatchDiagnostics& a,const ShellBatchDiagnostics& b) noexcept {
  return a.valid==b.valid&&qeph::batch_detail::SameDiagnostics(a.qeph,b.qeph)&&
    t3::batch_detail::SameDiagnostics(a.t3,b.t3)&&SameKinetic(a.base_kinetic,b.base_kinetic)&&SameKinetic(a.kinetic,b.kinetic);
}
template<class D> bool UnavailableKinetic(const D& d) noexcept {
  return !d.kinetic_available&&d.kinetic_translation==0&&d.kinetic_rotation==0&&
    d.kinetic_physical_isotropic==0&&d.kinetic_added_isotropic==0;
}
} // namespace

struct ShellBatchPublication::Impl {
  qeph::QephBatch* qbatch=nullptr;
  t3::T3Batch* tbatch=nullptr;
  const ShellBatchPublication* scope=nullptr;
  shell_publication_detail::Storage* storage=nullptr;
  shell_publication_detail::Control control;
  ShellBatchDiagnostics accepted,candidate;
  NodalPreparedView candidate_view;
  bool pending=false,usable=true;
  ~Impl() { if(storage) cudaFree(storage); }
  void Discard() noexcept {
    pending=false; candidate={}; candidate_view={};
    if(qbatch) qbatch->DiscardTrial();
    if(tbatch) tbatch->DiscardTrial();
  }
  void Poison() noexcept {
    usable=false;
    if(qbatch&&qbatch->impl_) qbatch->impl_->usable=false;
    if(tbatch&&tbatch->impl_) tbatch->impl_->usable=false;
    Discard();
  }
  ShellPublicationReport Runtime(cudaError_t error,const char* message) noexcept {
    if(error==cudaSuccess) return Ok();
    Poison(); return {S::DeviceFailure,message};
  }
  bool SameScope() const noexcept {
    if(!qbatch||!tbatch||!qbatch->impl_||!tbatch->impl_) return false;
    const auto& q=*qbatch->impl_; const auto& t=*tbatch->impl_;
    const bool same_usage=(q.config.usage==qeph::BatchUsage::PrescribedFields&&t.config.usage==t3::BatchUsage::PrescribedFields)||
      (q.config.usage==qeph::BatchUsage::CoupledForces&&t.config.usage==t3::BatchUsage::CoupledForces);
    return q.joined_binding&&t.joined_binding&&q.joined_binding->inventory()==t.joined_binding->inventory()&&
      q.config.element_count==1&&t.config.element_count==1&&q.config.configuration_id==t.config.configuration_id&&
      q.config.qualification_id==t.config.qualification_id&&same_usage&&trial_identity::SameStamp(q.config.owner,t.config.owner)&&
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
      const qeph::BatchDiagnostics& qd,const t3::BatchDiagnostics& td,NodalPreparedView& authentic) noexcept {
    if(!usable||!qbatch->impl_->usable||!tbatch->impl_->usable) return {S::DeviceFailure,"Mixed shell participant is poisoned"};
    if(qbatch->impl_->publication_scope!=scope||tbatch->impl_->publication_scope!=scope)
      return {S::NotJoined,"Mixed participant belongs to a different publication scope"};
    if(!SameScope()) return {S::NotJoined,"Mixed shell participant inventories or immutable scopes differ"};
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
    return Ok();
  }
};

ShellBatchPublication::ShellBatchPublication()=default;
ShellBatchPublication::~ShellBatchPublication() {
  if(!impl_) return;
  // Borrowed participants must outlive this coordinator, including destruction.
  if(impl_->qbatch->impl_->publication_scope==this) impl_->qbatch->impl_->publication_scope=nullptr;
  if(impl_->tbatch->impl_->publication_scope==this) impl_->tbatch->impl_->publication_scope=nullptr;
}
ShellPublicationReport ShellBatchPublication::Initialize(FENodalState& owner,qeph::QephBatch& q,t3::T3Batch& t) {
  if(impl_) return {S::InvalidInput,"Mixed publication scope is already initialized"};
  if(!q.impl_||!t.impl_) return {S::NotInitialized,"Both shell participants must be initialized"};
  if(!q.impl_->joined_binding||!t.impl_->joined_binding) return {S::NotJoined,"Both shell participants must use InitializeJoined"};
  std::unique_ptr<Impl> next(new(std::nothrow) Impl);
  if(!next) return {S::ResourceLimit,"Mixed publication host allocation failed"};
  next->qbatch=&q; next->tbatch=&t; next->scope=this;
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
  shell_publication_detail::Storage initial;
  const auto& binding=*q.impl_->joined_binding; initial.model.node_count=binding.node_count();
  for(std::size_t n=0;n<binding.node_count();++n) {
    const auto& mass=binding.nodes()[n].native;
    initial.model.mass[n]=mass.mass; initial.model.inertia[n]=mass.isotropic_inertia;
    initial.model.physical[n]=mass.physical_inertia; initial.model.added[n]=mass.added_inertia;
  }
  auto r=next->Runtime(cudaGetLastError(),"Pending CUDA error before mixed publication initialization");
  if(r.status!=S::Success) return r;
  r=next->Runtime(cudaMalloc(reinterpret_cast<void**>(&next->storage),sizeof(initial)),"Mixed kinetic allocation failed");
  if(r.status!=S::Success) return r;
  r=next->Runtime(cudaMemcpy(next->storage,&initial,sizeof(initial),cudaMemcpyHostToDevice),"Mixed kinetic initialization failed");
  if(r.status!=S::Success) return r;
  next->accepted.qeph=q.impl_->accepted_diagnostics; next->accepted.t3=t.impl_->accepted_diagnostics;
  next->accepted.valid=true; // Known zero actual rest state; no h=0 evaluation.
  q.impl_->publication_scope=this; t.impl_->publication_scope=this;
  impl_=std::move(next); return Ok();
}
ShellPublicationReport ShellBatchPublication::Prepare(FENodalState& owner,const NodalTrialToken& token,
    const qeph::BatchDiagnostics& q,const t3::BatchDiagnostics& t,ShellBatchDiagnostics* output) {
  auto fail=[&](ShellPublicationReport r) {
    if(impl_&&(r.status==S::DeviceFailure||r.nodal_status==NodalStatus::DeviceFailure)) impl_->Poison();
    owner.Discard(); DiscardTrial(); return r;
  };
  if(!impl_) return fail({S::NotInitialized,"Mixed publication scope is not initialized"});
  auto& s=*impl_; s.pending=false; s.candidate={}; s.candidate_view={};
  if(!trial_identity::Disjoint(output,sizeof(*output),&q,sizeof(q))||
     !trial_identity::Disjoint(output,sizeof(*output),&t,sizeof(t)))
    return fail({S::InvalidInput,"Mixed diagnostic output is missing or overlaps typed inputs"});
  NodalPreparedView authentic;
  auto r=s.Preflight(owner,token,q,t,authentic); if(r.status!=S::Success) return fail(r);
  r=s.Runtime(cudaGetLastError(),"Pending CUDA error before mixed kinetic measurement");
  if(r.status!=S::Success) return fail(r);
  shell_publication_detail::LaunchMeasure(s.storage,authentic);
  r=s.Runtime(cudaGetLastError(),"Mixed kinetic kernel launch failed"); if(r.status!=S::Success) return fail(r);
  r=s.Runtime(cudaMemcpyAsync(&s.control,&s.storage->control,sizeof(s.control),cudaMemcpyDeviceToHost,authentic.stream),
              "Mixed kinetic readback failed"); if(r.status!=S::Success) return fail(r);
  r=s.Runtime(cudaStreamSynchronize(authentic.stream),"Mixed kinetic stream failed"); if(r.status!=S::Success) return fail(r);
  if(s.control.status!=S::Success) return fail({s.control.status,"Mixed native kinetic reduction is nonfinite"});
  ShellBatchDiagnostics next; next.qeph=q; next.t3=t;
  next.base_kinetic=s.control.base; next.kinetic=s.control.endpoint; next.valid=true;
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
  auto r=s.Preflight(owner,token,expected.qeph,expected.t3,authentic); if(r.status!=S::Success) return fail(r);
  if(!trial_identity::SamePrepared(authentic,s.candidate_view)||!receipt.passed||
     receipt.owner_id!=expected.qeph.owner_id||receipt.base_epoch!=expected.qeph.base_epoch||
     receipt.attempt!=expected.qeph.attempt||receipt.qualification_id!=expected.qeph.qualification_id)
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
  s.qbatch->impl_->Publish(stamp); s.tbatch->impl_->Publish(stamp);
  s.accepted=s.candidate;
  s.accepted.qeph.phase=qeph::BatchPhase::Accepted; s.accepted.t3.phase=t3::BatchPhase::Accepted;
  s.pending=false; s.candidate={}; s.candidate_view={};
  return {S::Success,"Owner and both native material/cache slabs published"};
}
ShellPublicationReport ShellBatchPublication::CopyAcceptedDiagnostics(const NodalStamp& expected,
    ShellBatchDiagnostics* output) const noexcept {
  if(!impl_) return {S::NotInitialized,"Mixed publication scope is not initialized"};
  const auto& s=*impl_;
  if(!output||!trial_identity::Disjoint(output,sizeof(*output),&expected,sizeof(expected)))
    return {S::InvalidInput,"Mixed accepted diagnostic output is missing or overlaps identity"};
  if(!s.usable||!s.qbatch->impl_->usable||!s.tbatch->impl_->usable) return {S::DeviceFailure,"Mixed publication scope is poisoned"};
  if(s.qbatch->impl_->publication_scope!=this||s.tbatch->impl_->publication_scope!=this||
     !s.SameScope()||!trial_identity::SameStamp(expected,s.qbatch->impl_->accepted_stamp)||
     s.accepted.qeph.epoch!=expected.epoch||s.accepted.t3.epoch!=expected.epoch)
    return {S::StaleTrial,"Mixed accepted diagnostics belong to another endpoint"};
  *output=s.accepted; return Ok();
}
void ShellBatchPublication::DiscardTrial() noexcept { if(impl_) impl_->Discard(); }
NodalAllocationInfo ShellBatchPublication::allocations() const noexcept {
  return impl_?NodalAllocationInfo{sizeof(shell_publication_detail::Storage),1}:NodalAllocationInfo{};
}
} // namespace tl::fea
