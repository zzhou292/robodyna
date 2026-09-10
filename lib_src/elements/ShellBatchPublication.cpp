// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchPublicationStorage.h"
#include "qeph/QephBatchStorage.h"
#include "t3/T3BatchStorage.h"
#include "ShellBatchStartup.h"
#include <stdexcept>
#include <cstring>
#include <new>

namespace tl::fea {
namespace {
using S=ShellPublicationStatus;
ShellPublicationReport Ok() noexcept { return {S::Success,"OK"}; }
ShellPublicationReport Nodal(const NodalReport& r) noexcept { return {S::NodalFailure,r.message,r.status}; }
ShellPublicationReport Connector(const type25::BatchReport& r) noexcept {
  using B=type25::BatchStatus;
  if(r.status==B::Success) return Ok();
  if(r.status==B::DeviceFailure||r.status==B::Unusable) return {S::DeviceFailure,r.message,r.nodal_status};
  if(r.status==B::NodalFailure) return {S::NodalFailure,r.message,r.nodal_status};
  return {S::StaleTrial,r.message,r.nodal_status};
}
bool SameDouble(double a,double b) noexcept { return std::memcmp(&a,&b,sizeof(a))==0; }
bool SameKinetic(const ShellBatchKinetic& a,const ShellBatchKinetic& b) noexcept {
  return SameDouble(a.translation,b.translation)&&SameDouble(a.rotation,b.rotation)&&
    SameDouble(a.physical_isotropic,b.physical_isotropic)&&SameDouble(a.added_isotropic,b.added_isotropic)&&
    SameDouble(a.connector_translation,b.connector_translation)&&SameDouble(a.connector_rotation,b.connector_rotation);
}
bool SameDiagnostics(const ShellBatchDiagnostics& a,const ShellBatchDiagnostics& b) noexcept {
  return a.valid==b.valid&&a.has_connector==b.has_connector&&
    type25::batch_detail::SameDiagnostics(a.connector,b.connector)&&qeph::batch_detail::SameDiagnostics(a.qeph,b.qeph)&&
    t3::batch_detail::SameDiagnostics(a.t3,b.t3)&&SameKinetic(a.base_kinetic,b.base_kinetic)&&SameKinetic(a.kinetic,b.kinetic);
}
template<class D> bool UnavailableKinetic(const D& d) noexcept {
  return !d.kinetic_available&&d.kinetic_translation==0&&d.kinetic_rotation==0&&
    d.kinetic_physical_isotropic==0&&d.kinetic_added_isotropic==0;
}
ShellPublicationReport InitialMovingKinetic(FENodalState& owner,const ShellBatchBinding& binding,
    const NodalMassBinding* combined,const ShellBatchStartup& startup,
    const NodalStamp& expected,ShellBatchKinetic& output) {
  // Initial assembly views have expired after caller Discard. Authenticate
  // those source identities separately, then obtain fresh actual owner fields
  // through its accepted-only readback contract; never dereference old views.
  const auto count=binding.node_count();
  // The caller has preflighted this complete 13-double/node temporary payload.
  util::HostArena fields; util::BoundedArenaLayout layout(13*count*sizeof(double));
  util::ArenaRegion xr,vr,wr,qr;
  if(!layout.Append<double>(3*count,xr)||!layout.Append<double>(3*count,vr)||
     !layout.Append<double>(3*count,wr)||!layout.Append<double>(4*count,qr)||!fields.Initialize(layout.bytes()))
    return {S::ResourceLimit,"Initial kinetic readback staging allocation failed"};
  auto* x=fields.Construct<double>(xr); auto* v=fields.Construct<double>(vr);
  auto* w=fields.Construct<double>(wr); auto* orientation=fields.Construct<double>(qr);
  NodalStamp stamp;
  const auto copied=owner.CopyAccepted({x,v,count,orientation,w},&stamp);
  if(copied.status!=NodalStatus::Ok) return Nodal(copied);
  if(!trial_identity::SameStamp(stamp,expected)||stamp.epoch||stamp.time!=0||stamp.velocity_time!=0||stamp.node_count!=binding.node_count()||
     stamp.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart||stamp.velocity_phase!=NodalVelocityPhase::Collocated)
    return {S::StaleTrial,"Common initial kinetic requires the actual epoch-zero physical owner"};
  ShellBatchKinetic measured;
  for(std::size_t n=0;n<binding.node_count();++n) {
    const tl::math::Vec3 position{x[3*n],x[3*n+1],x[3*n+2]},velocity{v[3*n],v[3*n+1],v[3*n+2]},omega{w[3*n],w[3*n+1],w[3*n+2]};
    if(!shell_startup_detail::MatchesInitialNode(startup,position,binding.nodes()[n].position,velocity,omega,orientation+4*n))
      return {S::InvalidInput,"Actual common initial state differs from the bound motion declaration"};
    const auto mass=combined?combined->nodes()[n].coefficients.mass:binding.nodes()[n].native.mass;
    if(!shell_startup_detail::AddInitialTranslationKinetic(mass,velocity,measured.translation))
      return {S::NonfiniteResult,"Measured common initial kinetic energy overflows"};
    if(combined) {
      const auto connector_mass=combined->nodes()[n].coefficients.connector_mass;
      // The startup helper requires positive mass; ordinary nodes may carry
      // exactly zero connector mass and contribute a known zero partition.
      if(connector_mass>0&&!shell_startup_detail::AddInitialTranslationKinetic(
          connector_mass,velocity,measured.connector_translation))
        return {S::NonfiniteResult,"Measured connector initial kinetic energy overflows"};
    }
  }
  output=measured; return Ok();
}
} // namespace

struct ShellBatchPublication::Impl {
  qeph::QephBatch* qbatch=nullptr;
  t3::T3Batch* tbatch=nullptr;
  type25::Batch* connector=nullptr;
  const ShellBatchPublication* scope=nullptr;
  shell_publication_detail::Storage* storage=nullptr;
  shell_publication_detail::Layout layout;
  shell_publication_detail::Control control;
  ShellBatchDiagnostics accepted,candidate;
  NodalPreparedView candidate_view;
  bool pending=false,usable=true;
  ~Impl() { if(storage) cudaFree(storage); }
  void Discard() noexcept {
    pending=false; candidate={}; candidate_view={};
    if(qbatch) qbatch->DiscardTrial();
    if(tbatch) tbatch->DiscardTrial();
    if(connector) connector->DiscardTrial();
  }
  void Poison() noexcept {
    usable=false;
    if(qbatch&&qbatch->impl_) qbatch->impl_->usable=false;
    if(tbatch&&tbatch->impl_) tbatch->impl_->usable=false;
    if(connector) connector->Poison();
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

ShellBatchPublication::ShellBatchPublication()=default;
ShellBatchPublication::~ShellBatchPublication() {
  if(!impl_) return;
  // Borrowed participants must outlive this coordinator, including destruction.
  if(impl_->qbatch->impl_->publication_scope==this) impl_->qbatch->impl_->publication_scope=nullptr;
  if(impl_->tbatch->impl_->publication_scope==this) impl_->tbatch->impl_->publication_scope=nullptr;
  if(impl_->connector) impl_->connector->ReleasePublication(this);
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
  for(std::size_t n=0;n<binding.node_count();++n) {
    const auto& mass=binding.nodes()[n].native;
    initial->model.mass[n]=mass.mass; initial->model.inertia[n]=mass.isotropic_inertia;
    initial->model.physical[n]=mass.physical_inertia; initial->model.added[n]=mass.added_inertia;
    if(connector) {
      const auto& combined=q.impl_->joined_mass->nodes()[n].coefficients;
      initial->model.mass[n]=combined.mass; initial->model.inertia[n]=combined.isotropic_inertia;
      initial->model.connector_mass[n]=combined.connector_mass;
      initial->model.connector_inertia[n]=combined.connector_inertia;
    }
  }
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
  auto& s=*impl_; s.pending=false; s.candidate={}; s.candidate_view={};
  if(!trial_identity::Disjoint(output,sizeof(*output),&q,sizeof(q))||
     !trial_identity::Disjoint(output,sizeof(*output),&t,sizeof(t))||
     (connector&&!trial_identity::Disjoint(output,sizeof(*output),connector,sizeof(*connector))))
    return fail({S::InvalidInput,"Mixed diagnostic output is missing or overlaps typed inputs"});
  NodalPreparedView authentic;
  auto r=s.Preflight(owner,token,q,t,connector,authentic); if(r.status!=S::Success) return fail(r);
  r=s.Runtime(cudaGetLastError(),"Pending CUDA error before mixed kinetic measurement");
  if(r.status!=S::Success) return fail(r);
  shell_publication_detail::LaunchMeasure(s.storage,authentic);
  r=s.Runtime(cudaGetLastError(),"Mixed kinetic kernel launch failed"); if(r.status!=S::Success) return fail(r);
  r=s.Runtime(cudaMemcpyAsync(&s.control,&s.storage->control,sizeof(s.control),cudaMemcpyDeviceToHost,authentic.stream),
              "Mixed kinetic readback failed"); if(r.status!=S::Success) return fail(r);
  r=s.Runtime(cudaStreamSynchronize(authentic.stream),"Mixed kinetic stream failed"); if(r.status!=S::Success) return fail(r);
  if(s.control.status!=S::Success) return fail({s.control.status,"Mixed native kinetic reduction is nonfinite"});
  ShellBatchDiagnostics next; next.qeph=q; next.t3=t;
  if(connector) { next.connector=*connector; next.has_connector=true; }
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
  auto r=s.Preflight(owner,token,expected.qeph,expected.t3,
      expected.has_connector?&expected.connector:nullptr,authentic); if(r.status!=S::Success) return fail(r);
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
  if(s.connector) s.connector->Publish(stamp);
  s.accepted=s.candidate;
  s.accepted.qeph.phase=qeph::BatchPhase::Accepted; s.accepted.t3.phase=t3::BatchPhase::Accepted;
  if(s.connector) s.accepted.connector.phase=type25::BatchPhase::Accepted;
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
