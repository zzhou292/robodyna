// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchPublicationImpl.h"
#include <new>
#include <stdexcept>

namespace tl::fea {
ShellPublicationReport ShellBatchPublication::InitializeFormulations(FENodalState& owner,
    const ShellFormulationParticipants& participants,const ShellPublicationLimits& limits) try {
  if(impl_) return {S::InvalidInput,"Formulation publication is already initialized"};
  if(!participants.qbat||!participants.qbat->impl_) return {S::NotInitialized,"QBAT is required by this named publication"};
  auto& b=*participants.qbat->impl_;
  const auto source=b.Scope();
  const auto source_check=ValidateShellFormulationScope(source);
  if(source_check.status!=ShellPlasticityBindingStatus::Success) return {S::NotJoined,source_check.message};
  if(bool(participants.qeph)!=(source.binding->qeph_count()!=0)||
      bool(participants.t3)!=(source.binding->t3_count()!=0)||bool(participants.connector)!=bool(source.mass)) {
    return {S::NotJoined,"Every nonempty native family and combined coefficient contributor is required exactly once"};
  }
  const bool vehicle=limits.profile==ShellResidentProfile::Vehicle;
  const auto ceiling=vehicle?ShellPublicationLimits::Vehicle():ShellPublicationLimits{MaxShellResidentNodes,
      MaxShellResidentDeviceBytes,MaxShellResidentHostBytes};
  const auto count=source.binding->node_count();
  shell_publication_detail::Layout layout;
  if((limits.profile!=ShellResidentProfile::Legacy&&!vehicle)||!limits.max_nodes||
      limits.max_nodes>ceiling.max_nodes||count>limits.max_nodes||!limits.max_device_bytes||
      limits.max_device_bytes>ceiling.max_device_bytes||!limits.max_host_bytes||
      limits.max_host_bytes>ceiling.max_host_bytes||!layout.Initialize(count,limits.max_device_bytes,source.mass!=nullptr)) {
    return {S::ResourceLimit,"Formulation publication active-node/device capacity exceeded"};
  }
  util::BoundedArenaLayout budget(limits.max_host_bytes);
  util::ArenaRegion ignored;
  if(!budget.Append<unsigned char>(sizeof(Impl),ignored)||!budget.Append<unsigned char>(layout.bytes,ignored)||
      !budget.Append<double>(13*count,ignored)) return {S::ResourceLimit,"Formulation publication startup exceeds host cap"};
  auto next=std::make_unique<Impl>();
  next->qbatch=participants.qeph;
  next->tbatch=participants.t3;
  next->bbatch=participants.qbat;
  next->connector=participants.connector;
  next->scope=this;
  next->layout=layout;
  auto checked=Qbat(participants.qbat->PreflightAttach(owner,source,b.config.configuration_id,
      b.config.qualification_id,b.config.startup,this));
  if(checked.status!=S::Success) return checked;
  if(!next->SameFormulationScope()) return {S::NotJoined,"Native formulation immutable scopes differ"};
  if(participants.qeph) {
    const auto& q=*participants.qeph->impl_;
    if(!q.usable||!q.bound||q.pending||q.publication_scope||!UnavailableKinetic(q.accepted_diagnostics)) {
      return {S::NotJoined,"QEPH requires an unattached bound initial formulation cache"};
    }
    const auto initial=owner.ValidateAcceptedAssemblySources(q.initial_sources);
    if(initial.status!=NodalStatus::Ok) return Nodal(initial);
    next->accepted.qeph=q.accepted_diagnostics;
  }
  if(participants.t3) {
    const auto& t=*participants.t3->impl_;
    if(!t.usable||!t.bound||t.pending||t.publication_scope||!UnavailableKinetic(t.accepted_diagnostics)) {
      return {S::NotJoined,"T3 requires an unattached bound initial formulation cache"};
    }
    const auto initial=owner.ValidateAcceptedAssemblySources(t.initial_sources);
    if(initial.status!=NodalStatus::Ok) return Nodal(initial);
    next->accepted.t3=t.accepted_diagnostics;
  }
  if(participants.connector) {
    checked=Connector(participants.connector->PreflightAttach(owner.accepted(),*source.mass,
        b.config.configuration_id,b.config.qualification_id,b.config.startup,this));
    if(checked.status!=S::Success) return checked;
    checked=Connector(participants.connector->CopyAcceptedDiagnostics(owner.accepted(),&next->accepted.connector));
    if(checked.status!=S::Success) return checked;
  }
  if(b.config.startup.kind==ShellBatchStartupKind::ReferenceUniformTranslation) {
    checked=InitialMovingKinetic(owner,*source.binding,source.mass,b.config.startup,b.accepted_stamp,next->accepted.kinetic);
    if(checked.status!=S::Success) {
      if(checked.nodal_status==NodalStatus::DeviceFailure) next->Poison();
      return checked;
    }
  }
  util::HostArena arena;
  if(!arena.Initialize(layout.bytes)) return {S::ResourceLimit,"Formulation kinetic staging allocation failed"};
  auto* initial=layout.Construct(arena);
  if(!initial) return {S::ResourceLimit,"Formulation kinetic startup layout is invalid"};
  PopulateKineticModel(*source.binding,source.mass,initial->model);
  checked=next->Runtime(cudaGetLastError(),"Pending CUDA error before formulation publication initialization");
  if(checked.status!=S::Success) return checked;
  checked=next->Runtime(cudaMalloc(reinterpret_cast<void**>(&next->storage),layout.bytes),"Formulation kinetic allocation failed");
  if(checked.status!=S::Success) return checked;
  *initial=layout.Rebase(*initial,next->storage);
  checked=next->Runtime(cudaMemcpy(next->storage,arena.data(),layout.bytes,cudaMemcpyHostToDevice),
      "Formulation kinetic initialization failed");
  if(checked.status!=S::Success) return checked;
  next->accepted.qbat=b.accepted_diagnostics;
  next->accepted.has_qeph=participants.qeph!=nullptr;
  next->accepted.has_t3=participants.t3!=nullptr;
  next->accepted.has_qbat=true;
  next->accepted.has_connector=participants.connector!=nullptr;
  next->accepted.valid=true;
  if(participants.qeph) participants.qeph->impl_->publication_scope=this;
  if(participants.t3) participants.t3->impl_->publication_scope=this;
  participants.qbat->AttachPublication(this);
  if(participants.connector) participants.connector->AttachPublication(this);
  impl_=std::move(next);
  return Ok();
} catch(const std::bad_alloc&) {
  return {S::ResourceLimit,"Formulation publication host allocation failed"};
} catch(const std::length_error&) {
  return {S::ResourceLimit,"Formulation publication host allocation extent overflow"};
}
} // namespace tl::fea
