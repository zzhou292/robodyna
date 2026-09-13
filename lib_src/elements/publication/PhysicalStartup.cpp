// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../ShellBatchPublicationImpl.h"
#include "PhysicalChecks.h"
#include "../ShellPhysicalOwner.h"
#include "../type13/resident/Storage.h"
#include "../solids/resident/Storage.h"
#include "../type45/resident/Storage.h"
#include "../beam18/resident/Storage.h"
#include <new>

namespace tl::fea {
ShellPublicationReport ShellBatchPublication::InitializePhysical(FENodalState& owner,
    const ShellPhysicalBinding& binding,const NodalRigidAssemblyBinding& rigid,
    const NodalCinWitnessSource& cin,const ShellPhysicalParticipants& p,
    const ShellPhysicalPublicationIdentity& identity,const ShellPublicationLimits& limits) {
  return InitializePhysicalImpl(owner,binding,rigid,cin,nullptr,p,identity,limits);
}
ShellPublicationReport ShellBatchPublication::InitializePhysicalWithJoints(FENodalState& owner,
    const ShellPhysicalBinding& binding,const NodalRigidAssemblyBinding& rigid,
    const NodalCinWitnessSource& cin,const type45::Model& joints,const ShellPhysicalParticipants& p,
    const ShellPhysicalPublicationIdentity& identity,const ShellPublicationLimits& limits) {
  return InitializePhysicalImpl(owner,binding,rigid,cin,&joints,p,identity,limits);
}
ShellPublicationReport ShellBatchPublication::InitializePhysicalImpl(FENodalState& owner,
    const ShellPhysicalBinding& binding,const NodalRigidAssemblyBinding& rigid,
    const NodalCinWitnessSource& cin,const type45::Model* joints,const ShellPhysicalParticipants& p,
    const ShellPhysicalPublicationIdentity& identity,const ShellPublicationLimits& limits) try {
  if (impl_) return {S::InvalidInput,"Publication is already initialized"};
  ShellPhysicalPublicationForecast forecast;
  auto checked = ForecastPhysical(binding,cin.range_count,limits,forecast);
  if (checked.status != S::Success) return checked;
  if (!identity.configuration_id || !identity.qualification_id ||
      bool(joints)!=bool(p.type45) || (joints && !joints->prepared()) ||
      !CompletePhysicalParticipants(binding,p) || !rigid.prepared() ||
      !rigid.coefficients()->Matches(*binding.coefficients()))
    return {S::NotJoined,"Physical publication requires every declared family and the exact rigid ledger"};
  auto next = std::make_unique<Impl>();
  next->physical = std::make_shared<PhysicalState>(binding,rigid,joints);
  auto& physical = *next->physical;
  physical.owner = &owner;
  physical.beams = p.type13;
  physical.solids = p.solids;
  physical.joints = p.type45;
  physical.structural_beams = p.beam18;
  physical.identity = identity;
  physical.forecast = forecast;
  physical.accepted_stamp = owner.accepted();
  physical.SetCinCounts(cin.range_count,cin.witness_count);
  next->scope = this;
  next->qbatch = p.qeph;
  next->tbatch = p.t3;
  next->bbatch = p.qbat;
  next->connector = p.type25;
  const auto configuration = identity.configuration_id;
  const auto qualification = identity.qualification_id;
  const auto& startup = identity.startup;
  if (p.qeph) {
    checked = PhysicalReport(p.qeph->PreflightAttachMapped(owner,binding,configuration,qualification,startup,this));
    if (checked.status != S::Success) return checked;
  }
  if (p.t3) {
    checked = PhysicalReport(p.t3->PreflightAttachMapped(owner,binding,configuration,qualification,startup,this));
    if (checked.status != S::Success) return checked;
  }
  if (p.qbat) {
    checked = Qbat(p.qbat->PreflightAttachMapped(owner,binding,configuration,qualification,startup,this));
    if (checked.status != S::Success) return checked;
  }
  if (p.type25) {
    checked = Connector(p.type25->PreflightAttachMapped(owner,binding,configuration,qualification,startup,this));
    if (checked.status != S::Success) return checked;
  }
  if (p.type13) {
    checked = PhysicalReport(p.type13->PreflightAttachMapped(owner,binding,rigid,
        configuration,qualification,startup,this));
    if (checked.status != S::Success) return checked;
  }
  if (p.solids) {
    if (!p.solids->impl_) return {S::NotInitialized,"Declared solid participant is not initialized"};
    const auto& solid = *p.solids->impl_;
    auto config = solid.config;
    config.owner = owner.accepted();
    config.configuration_id = configuration;
    config.qualification_id = qualification;
    config.startup = startup;
    config.profile = solid.model.profile() == solids::ModelProfile::ExtendedLaw44Law90
        ? solids::BatchProfile::PhysicalCinExtendedLaw44Law90V2
        : solids::BatchProfile::PhysicalCinV1;
    config.cin_attachment_count = cin.range_count;
    config.cin_witness_count = cin.witness_count;
    // This is the participant's sole immutable force model. Its complete native
    // contributions must match the ledger; no second catalog is fabricated.
    checked = PhysicalReport(p.solids->PreflightAttach(owner,*binding.coefficients(),rigid,
        cin,solid.model,config,this));
    if (checked.status != S::Success) return checked;
  }
  if (p.type45) {
    if (!p.type45->impl_) return {S::NotInitialized,"Declared joint participant is not initialized"};
    auto config=p.type45->impl_->config;
    config.owner=owner.accepted();config.configuration_id=configuration;config.qualification_id=qualification;
    config.startup=startup;config.profile=type45::BatchProfile::PhysicalAggregateV1;
    config.cin_attachment_count=cin.range_count;config.cin_witness_count=cin.witness_count;
    checked=PhysicalReport(p.type45->PreflightAttach(owner,binding,cin,*joints,config,this));
    if (checked.status!=S::Success) return checked;
  }
  if (p.beam18) {
    if (!p.beam18->impl_) return {S::NotInitialized,"Declared structural beam participant is not initialized"};
    const auto& beam = *p.beam18->impl_;
    auto config = beam.config;
    config.owner = owner.accepted();
    config.configuration_id = configuration;
    config.qualification_id = qualification;
    config.startup = startup;
    config.profile = beam18::BatchProfile::PhysicalCinCircularFourPointLaw44V1;
    config.cin_attachment_count = cin.range_count;
    config.cin_witness_count = cin.witness_count;
    checked = PhysicalReport(p.beam18->PreflightAttach(owner,*binding.coefficients(),rigid,
        cin,beam.model,config,this));
    if (checked.status != S::Success) return checked;
  }
  // Includes the actual CIN roster and current epoch-zero M/J, with dependent
  // zero inertia retained. No constrained inverse is reconstructed here.
  auto nodal = owner.ValidateRigidAssemblyBinding(rigid);
  shell_physical_owner::ProofLayout proof;
  if (!shell_physical_owner::ForecastProof(binding.domain()->node_count(),cin.range_count,
      limits.max_host_bytes,proof)) return {S::ResourceLimit,"Physical proof extent changed"};
  if (nodal.status == NodalStatus::Ok)
    nodal = shell_physical_owner::AuthenticateInitial(*binding.coefficients(),owner,owner.accepted(),startup,cin,proof);
  if (nodal.status != NodalStatus::Ok) {
    if (nodal.status == NodalStatus::DeviceFailure) next->Poison();
    return Nodal(nodal);
  }
  auto& accepted = physical.accepted;
  accepted.base_stamp = owner.accepted();
  accepted.has_qeph = p.qeph != nullptr;
  accepted.has_t3 = p.t3 != nullptr;
  accepted.has_qbat = p.qbat != nullptr;
  accepted.has_type25 = p.type25 != nullptr;
  accepted.has_type13 = p.type13 != nullptr;
  accepted.has_solids = p.solids != nullptr;
  accepted.has_type45 = p.type45 != nullptr;
  accepted.has_beam18 = p.beam18 != nullptr;
  if (p.qeph) accepted.qeph = p.qeph->impl_->accepted_diagnostics;
  if (p.t3) accepted.t3 = p.t3->impl_->accepted_diagnostics;
  if (p.qbat) accepted.qbat = p.qbat->impl_->accepted_diagnostics;
  if (p.type25) {
    checked = Connector(p.type25->CopyAcceptedDiagnostics(owner.accepted(),&accepted.type25));
    if (checked.status != S::Success) return checked;
  }
  if (p.type13) accepted.type13 = p.type13->impl_->accepted_diagnostics;
  if (p.solids) accepted.solids = p.solids->impl_->accepted_diagnostics;
  if (p.type45) accepted.type45 = p.type45->impl_->accepted_diagnostics;
  if (p.beam18) accepted.beam18 = p.beam18->impl_->accepted_diagnostics;
  accepted.valid = true;
  // All allocations, readbacks and participant checks precede the first claim.
  // These private claims and the final pointer move cannot fail.
  if (p.qeph) p.qeph->impl_->publication_scope = this;
  if (p.t3) p.t3->impl_->publication_scope = this;
  if (p.qbat) p.qbat->AttachPublication(this);
  if (p.type25) p.type25->AttachPublication(this);
  if (p.type13) p.type13->AttachPublication(this);
  if (p.solids) p.solids->AttachPublication(this);
  if (p.type45) p.type45->AttachPublication(this,physical.binding);
  if (p.beam18) p.beam18->AttachPublication(this);
  impl_ = std::move(next);
  return Ok();
} catch (const std::bad_alloc&) {
  return {S::ResourceLimit,"Physical publication host allocation failed"};
}
void ShellBatchPublication::Impl::ReleasePhysical() noexcept {
  if (!physical) return;
  if (physical->beams) physical->beams->ReleasePublication(scope);
  if (physical->solids) physical->solids->ReleasePublication(scope);
  if (physical->joints) physical->joints->ReleasePublication(scope);
  if (physical->structural_beams) physical->structural_beams->ReleasePublication(scope);
}
} // namespace tl::fea
