// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "GroupSealSession.h"
#include "NormalStage.h"
#include "AssemblyTail.h"
#include "lib_src/elements/ShellPhysicalOwner.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <cstring>
namespace tlfea::contact::radioss_type25 {
namespace fe=tl::fea;namespace rd=runtime_detail;
namespace {
TransactionReport Error(TransactionStatus s,const char* message){return {s,message};}
VectorView View(const double* data,std::size_t nodes){return {data,std::uint32_t(nodes),3,1};}
bool Phase(const fe::NodalStamp& s,double& drift,double& kick) {
  if(s.temporal_scheme!=fe::NodalTemporalScheme::StaggeredHalfKickStart||!s.has_rotations||
     !tl::math::Finite(s.time)||!normal_detail::Nonnegative(s.fixed_dt)||!s.fixed_dt)return false;
  if(!s.epoch) {
    if(s.time!=0||s.reactions_valid||s.velocity_phase!=fe::NodalVelocityPhase::Collocated)return false;
    drift=kick=0;return true;
  }
  if(!s.reactions_valid||s.reaction_base_epoch!=s.epoch-1||s.reaction_time>=s.time||
     s.velocity_phase!=fe::NodalVelocityPhase::PreviousMidpoint||
     !normal_detail::Nonnegative(s.reaction_kick_dt)||s.reaction_kick_dt==0)return false;
  // This owner has a fixed drift policy. Its actual fixed operand is retained;
  // subtraction of accumulated TT values is not an equivalent dt observation.
  drift=s.fixed_dt;kick=s.reaction_kick_dt;return true;
}
}
TransactionReport Transaction::Impl::Fence(cudaError_t error) noexcept {
  if(error==cudaSuccess)error=cudaMemcpyAsync(&control,device.control,sizeof(control),cudaMemcpyDeviceToHost,stream);
  const auto drained=cudaStreamSynchronize(stream);
  if(error!=cudaSuccess||drained!=cudaSuccess){usable=false;return Error(TransactionStatus::DeviceFailure,"Native GPU stage failed");}
  if(control.failure!=~0ull)return {static_cast<TransactionStatus>(control.failure&255),
      "Native GPU stage rejected",std::size_t(control.failure>>16),SIZE_MAX,
      static_cast<selection::Status>((control.failure>>8)&255)};
  return {TransactionStatus::Ok,"OK"};
}
void Transaction::Impl::DiscardLocal() noexcept {
  const auto accepted=state.Accepted(*owner);
  if(phase!=Phase::Idle&&trial_selectors.has_reference&&
     (!accepted.available||!accepted.selectors.has_reference||trial_selectors.reference!=accepted.selectors.reference)) {
    inventory[trial_selectors.reference].Discard();maintenance[trial_selectors.reference].DiscardReference();
    inventory_view[trial_selectors.reference]={};
  }
  if(activity) {
    activity->snapshot.DiscardTrial();
    if(accepted.available)activity->operands.DiscardStaged(accepted.selectors.activity,accepted.selectors.activity^1u);
    activity->accepted={};activity->prepared={};
  }
  incidence.Discard();normal_ready=false;phase=Phase::Idle;assembly_view={};assembly_stamp={};trial_selectors={};
}
TransactionReport Transaction::Impl::Fail(TransactionReport result) noexcept {
  if(result.status==TransactionStatus::DeviceFailure)usable=false;
  owner->Discard();if(issuer.configured())publication->DiscardTrial();else issuer.DiscardTrial();
  DiscardLocal();return result;
}
bool Transaction::Impl::OutputDisjoint(const void* p,std::size_t bytes) const noexcept {
  using fe::trial_identity::Disjoint;
  return p&&Disjoint(p,bytes,this,sizeof(*this))&&Disjoint(p,bytes,arena,layout.bytes)&&
      Disjoint(p,bytes,readback.data(),readback.bytes())&&
      (!activity||(Disjoint(p,bytes,activity.get(),sizeof(*activity))&&activity->operands.OutputDisjoint(p,bytes)))&&issuer.configured()&&publication->PhysicalOutputDisjoint(p,bytes);
}
TransactionReport Transaction::AssembleAccepted(fe::FENodalState& owner,const fe::NodalTrialToken& token,
    const fe::NodalAssemblyView& view) noexcept {
  if(!impl_)return Error(TransactionStatus::NotInitialized,"Native transaction is not initialized");auto& p=*impl_;
  if(!p.usable)return Error(TransactionStatus::Unusable,"Native transaction is poisoned");
  if(&owner!=p.owner)return p.Fail(Error(TransactionStatus::OwnerFailure,"Wrong native physical owner"));
  if(!p.issuer.configured())return p.Fail(Error(TransactionStatus::PublicationFailure,"Native publication binding is unavailable"));
  auto check=owner.AuthenticateAssemblyView(token,view);
  if(check.status!=fe::NodalStatus::Ok)return p.Fail(Error(TransactionStatus::OwnerFailure,check.message));
  auto publication=p.publication->ValidatePhysicalAssembly(owner,token,view);
  if(publication.status!=fe::ShellPublicationStatus::Success)return p.Fail(Error(TransactionStatus::PublicationFailure,publication.message));
  // Enforce explicit group order before any numerical kernel/force write.
  publication=p.issuer.CheckNativeContactAssembly(owner,token,view);
  if(publication.status!=fe::ShellPublicationStatus::Success)
    return p.Fail(Error(TransactionStatus::PublicationFailure,publication.message));
  const bool static_mass=p.config.response_mass==ResponseMassPolicy::StaticPhysicalLedger;
  const auto accepted=p.state.Accepted(owner);double drift=0,kick=0;
  if(!accepted.available||!p.issuer.configured()||accepted.generation==UINT64_MAX||
     !Phase(accepted.stamp,drift,kick)||view.stream!=p.stream||
     !view.translation_fixed_bits||!view.rotation_fixed||(static_mass&&p.issuer.witness_count_))
    return p.Fail(Error(TransactionStatus::UnsupportedProfile,"Native owner/source/phase is not admitted"));
  if(p.phase!=Impl::Phase::Idle) {
    if(view.attempt==p.assembly_view.attempt)return p.Fail(Error(TransactionStatus::StaleAttempt,"Native assembly already ran for this attempt"));
    p.DiscardLocal();
  }
  p.diagnostics={};p.normal_ready=false;p.trial_selectors=accepted.selectors;p.trial_selectors.history=accepted.selectors.history^1u;
  p.assembly_view=view;p.assembly_stamp=accepted.stamp;p.phase=Impl::Phase::Assembled;
  auto activity_status=p.CaptureAcceptedActivity(token,view,accepted);
  if(activity_status.status!=TransactionStatus::Ok)return p.Fail(activity_status);
  fe::NodalCinAssemblyView cin;
  check=fe::shell_physical_owner::BorrowAssembly(owner,token,accepted.stamp,view,p.issuer.witness_count_,&cin);
  if(check.status!=fe::NodalStatus::Ok)return p.Fail(Error(TransactionStatus::OwnerFailure,check.message));
  if((static_mass&&cin.witness_count)||cin.node_count!=p.source.selection.node_count||!cin.translational_stiffness)
    return p.Fail(Error(TransactionStatus::UnsupportedProfile,"Native coefficient assembly scope is not admitted"));
  rd::MassOperands mass{p.device.native_mass,rd::MassOperands::Units::Native};
  if(!static_mass) {
    // This local borrow cannot escape the accepted force stage. Authenticate
    // against the same live token and CIN assembly before any response launch.
    fe::NodalAcceptedRawMassView raw_mass;
    check=owner.BorrowAcceptedRawMass(token,&raw_mass);
    if(check.status==fe::NodalStatus::Ok)check=owner.AuthenticateAcceptedRawMass(token,raw_mass);
    if(check.status!=fe::NodalStatus::Ok)return p.Fail(Error(TransactionStatus::OwnerFailure,check.message));
    if(raw_mass.owner_id!=view.owner_id||raw_mass.base_epoch!=accepted.stamp.epoch||
       raw_mass.attempt!=view.attempt||raw_mass.node_count!=p.source.selection.node_count||
       raw_mass.qualification_id!=cin.qualification_id||raw_mass.stream!=p.stream)
      return p.Fail(Error(TransactionStatus::SourceMismatch,"Accepted response mass differs from the physical attempt"));
    mass={raw_mass.mass_kg,rd::MassOperands::Units::Si};
  }
  const auto pending=cudaGetLastError();
  if(pending!=cudaSuccess)return p.Fail(Error(TransactionStatus::DeviceFailure,"Pending CUDA error before native assembly"));
  auto status=p.Fence(rd::ValidateCurrent(p.device,view,p.stream));if(status.status!=TransactionStatus::Ok)return p.Fail(status);
  search::Current current;current.stamp={{p.source.source_id,p.source.topology_generation,p.source.selection.generation},accepted.stamp.epoch,view.attempt};
  current.positions=View(view.accepted.position_xyz,view.accepted.node_count);current.velocities=View(view.accepted.velocity_xyz,view.accepted.node_count);
  current.secondary_stiffness=p.device.secondary_stiffness;current.secondary_count=p.source.selection.secondary_count;
  // Immutable source gaps select Maintenance::GapMode::Fixed, independently
  // of whether main geometry moves; this profile does not update thickness gaps.
  current.main_gaps=nullptr;current.main_gap_count=0;
  search::Report budget;bool rebuild=!accepted.selectors.has_reference||
      accepted.selectors.reference_activity_generation!=accepted.selectors.activity_generation;unsigned reference=accepted.selectors.reference;
  if(!rebuild) {
    const auto evaluated=p.maintenance[reference].Evaluate(current,drift,false,budget);
    if(evaluated!=search::Status::Ok)return p.Fail(Error(evaluated==search::Status::DeviceFailure?
        TransactionStatus::DeviceFailure:TransactionStatus::NumericalFailure,"Native maintenance rejected"));
    rebuild=budget.budget.requires_sort;
  }
  if(rebuild) {
    if(accepted.selectors.reference_generation==UINT64_MAX)return p.Fail(Error(TransactionStatus::ResourceLimit,"Native reference generation exhausted"));
    reference=accepted.selectors.has_reference?(accepted.selectors.reference^1u):1u;
    p.trial_selectors.reference=reference;p.trial_selectors.reference_generation=accepted.selectors.reference_generation+1;
    p.trial_selectors.has_reference=true;
    p.trial_selectors.reference_activity_generation=accepted.selectors.activity_generation;
    search::ReferenceToken captured;
    auto maintenance_status=p.maintenance[reference].StageReference(current,captured);
    if(maintenance_status==search::Status::Ok)maintenance_status=p.maintenance[reference].PublishReference(captured);
    if(maintenance_status==search::Status::Ok)maintenance_status=p.maintenance[reference].Evaluate(current,drift,true,budget);
    if(maintenance_status!=search::Status::Ok)return p.Fail(Error(maintenance_status==search::Status::DeviceFailure?
        TransactionStatus::DeviceFailure:TransactionStatus::NumericalFailure,"Native trial reference rebuild rejected"));
    candidates::Current query;query.stamp={{p.source.source_id,p.source.topology_generation},p.source.selection.generation,
      p.source.selection.generation,accepted.stamp.epoch+1,view.attempt,p.trial_selectors.reference_generation};
    query.positions=current.positions;query.velocities=current.velocities;
    query.secondary_stiffness=p.device.secondary_stiffness;query.secondary_gaps=p.device.secondary_gaps;
    query.main_stiffness=p.device.main_stiffness;query.main_gaps=p.device.main_gaps;query.main_curvature=p.device.main_curvature;
    query.domain_policy=candidates::DomainPolicy::AllFinite;
    query.margin=p.source.margin*p.units.length;query.gap_load=p.source.gap_load*p.units.length;
    query.drad=p.source.drad*p.units.length;query.stored_motion=budget.budget.stored_motion*p.units.length;query.previous_dt=drift;
    const auto candidate_status=p.inventory[reference].Stage(query);
    p.diagnostics.candidate_rebuild=p.inventory[reference].last_report();
    p.diagnostics.candidate_rebuild_available=true;
    if(candidate_status!=candidates::Status::Ok)return p.Fail(Error(candidate_status==candidates::Status::DeviceFailure?
        TransactionStatus::DeviceFailure:candidate_status==candidates::Status::ResourceLimit?TransactionStatus::ResourceLimit:
        TransactionStatus::NumericalFailure,"Complete native candidate rebuild rejected"));
    p.inventory_view[reference]=p.inventory[reference].view();
  }
  const auto inventory=p.inventory_view[reference];
  if(!p.inventory[reference].IsCurrent(inventory)||inventory.stamp().reference!=p.trial_selectors.reference_generation||
     inventory.stamp().source.source!=p.source.source_id||inventory.stamp().source.topology!=p.source.topology_generation)
    return p.Fail(Error(TransactionStatus::StaleAttempt,"Native candidate view is stale"));
  p.diagnostics.raw_candidates=inventory.pair_count();p.diagnostics.reference_rebuilt=rebuild;
  auto error=rd::ConvertInventory(p.device,inventory.pairs(),inventory.secondary_offsets(),inventory.pair_count(),p.stream);
  lifecycle::Input input;input.profile=p.config.lifecycle;input.step={accepted.stamp.time/p.units.time,drift/p.units.time};
  input.source=p.device.source;input.source.secondary=p.device.secondary[accepted.selectors.history];
  input.current={current.positions,current.velocities,lifecycle::KinematicsUnits::Si,p.config.units};
  input.accepted_rows=p.device.history[accepted.selectors.history];input.accepted_row_count=input.source.secondary_count;
  input.spatial=p.device.spatial;input.spatial_count=inventory.pair_count();
  input.spatial_by_secondary={p.device.spatial_offsets,input.source.secondary_count+1,p.device.spatial_entries,inventory.pair_count()};
  if(error==cudaSuccess)error=rd::Prepare(p.device,input,p.units,p.stream);
  status=p.Fence(error);if(status.status!=TransactionStatus::Ok)return p.Fail(status);
  if(p.control.required_sliding>p.limits.sliding_entries)return p.Fail(Error(TransactionStatus::ResourceLimit,"Complete sliding scratch exceeds cap"));
  if(p.device.normal.shape.enabled) {
    status=p.Fence(rd::CountBeforeNormals(p.device,input,p.units,p.stream));
    if(status.status!=TransactionStatus::Ok)return p.Fail(status);
    if(p.control.required_candidates>p.limits.optimized_candidates)
      return p.Fail(Error(TransactionStatus::ResourceLimit,"Complete OPTCD suffix exceeds candidate cap"));
    const auto optimized=std::size_t(p.control.required_candidates);
    status=p.Fence(rd::EmitOptimized(p.device,input,p.units,optimized,p.stream));
    if(status.status!=TransactionStatus::Ok)return p.Fail(status);
    status=p.Fence(rd::UpdateNormals(p.device,input,p.units,accepted.selectors.history,
        p.trial_selectors.history,optimized,p.stream));
    if(status.status!=TransactionStatus::Ok)return p.Fail(status);
    const auto trial=p.trial_selectors.history;
    input.current_normals={p.device.normal.face[trial],4*input.source.main_count,
        p.device.normal.references[trial],input.source.normal_count};
    p.normal_ready=true;
    status=p.Fence(rd::CountAfterNormals(p.device,input,p.units,p.stream));
  } else status=p.Fence(rd::CountCandidates(p.device,input,p.units,p.stream));
  if(status.status!=TransactionStatus::Ok)return p.Fail(status);
  if(p.control.required_candidates>p.limits.optimized_candidates)return p.Fail(Error(TransactionStatus::ResourceLimit,"Complete optimized candidate set exceeds cap"));
  const auto count=std::size_t(p.control.required_candidates);p.diagnostics.optimized_candidates=count;
  status=p.Fence(rd::Complete(p.device,input,p.units,count,p.stream));if(status.status!=TransactionStatus::Ok)return p.Fail(status);
  status=p.Fence(rd::Order(p.device,count,p.stream));if(status.status!=TransactionStatus::Ok)return p.Fail(status);
  const auto kept=std::size_t(p.control.kept);p.diagnostics.kept_occurrences=kept;
  status=p.Fence(rd::Respond(p.device,input,p.config,p.units,mass,kick/p.units.time,p.trial_selectors.history,count,kept,p.stream));
  if(status.status!=TransactionStatus::Ok)return p.Fail(status);
  p.diagnostics.active_forces=p.control.active;p.diagnostics.elastic_energy=p.control.elastic_energy;
  p.diagnostics.damping_work=p.control.damping_work;p.diagnostics.friction_work=p.control.friction_work;
  const auto cohorts=kept/p.source.force_packet_size+(kept%p.source.force_packet_size!=0);
  const assembly::Schedule schedule{cohorts?p.device.cohort_ends:nullptr,cohorts,kept};
  const assembly::IncidenceStamp incidence_stamp{p.source.source_id,p.source.topology_generation,
    accepted.generation+1,accepted.generation+1,view.attempt};
  const auto incidence_status=p.incidence.Stage({kept?p.device.force_connectivity:nullptr,schedule,p.source.selection.node_count,incidence_stamp});
  if(incidence_status!=assembly::IncidenceStatus::Ok)return p.Fail(Error(incidence_status==assembly::IncidenceStatus::DeviceFailure?
      TransactionStatus::DeviceFailure:TransactionStatus::NumericalFailure,"Native dynamic ASS0 incidence rejected"));
  const auto incidence=p.incidence.view();if(!p.incidence.IsCurrent(incidence))return p.Fail(Error(TransactionStatus::StaleAttempt,"Native ASS0 incidence expired"));
  status=rd::AssembleTail(
      [&]{return rd::Gather(p.device,schedule,incidence.incidence(),view,cin,p.stream);},
      [&]{return rd::Apply(p.device,view,cin,p.stream);},
      [&](cudaError_t error){return p.Fence(error);},[]{return cudaGetLastError();});
  if(status.status!=TransactionStatus::Ok)return p.Fail(status);
  publication=p.issuer.RecordNativeContactAcceptedAssembly(p.source.source_id,owner,token,view);
  if(publication.status!=fe::ShellPublicationStatus::Success)return p.Fail(Error(TransactionStatus::PublicationFailure,publication.message));
  return {TransactionStatus::Ok,"OK"};
}
TransactionReport Transaction::SealCandidate(fe::FENodalState& owner,const fe::NodalTrialToken& token,
    const fe::NodalPreparedView& view,const fe::ShellPhysicalDiagnostics& physical,
    fe::ShellPhysicalScratchParticipationReceipt* output) noexcept {
  return SealCandidateImpl(owner,token,view,physical,output,nullptr,SIZE_MAX);
}
TransactionReport Transaction::SealCandidateImpl(fe::FENodalState& owner,const fe::NodalTrialToken& token,
    const fe::NodalPreparedView& view,const fe::ShellPhysicalDiagnostics& physical,
    fe::ShellPhysicalScratchParticipationReceipt* output,GroupSealSession* group,std::size_t index) noexcept {
  if(!impl_)return Error(TransactionStatus::NotInitialized,"Native transaction is not initialized");auto& p=*impl_;
  if(!p.usable)return Error(TransactionStatus::Unusable,"Native transaction is poisoned");
  // A foreign group descriptor cannot discard another owner's pending state.
  // Standalone behavior is unchanged; the group wrapper revokes its own scope.
  if(group && (&owner!=p.owner || p.publication!=&group->publication))
    return Error(TransactionStatus::StaleAttempt,"Native candidate does not match its accepted force stage");
  if(&owner!=p.owner||p.phase!=Impl::Phase::Assembled||
     (p.device.normal.shape.enabled&&!p.normal_ready)||!p.OutputDisjoint(output,sizeof(*output))||
     !fe::trial_identity::Disjoint(output,sizeof(*output),this,sizeof(*this))||
     !fe::trial_identity::Disjoint(output,sizeof(*output),&token,sizeof(token))||
     !fe::trial_identity::Disjoint(output,sizeof(*output),&view,sizeof(view))||
     !fe::trial_identity::Disjoint(output,sizeof(*output),&physical,sizeof(physical))||
     view.owner_id!=p.assembly_view.owner_id||view.attempt!=p.assembly_view.attempt||
     view.kinematics.base_epoch!=p.assembly_stamp.epoch||!fe::trial_identity::SameStamp(owner.accepted(),p.assembly_stamp))
    return p.Fail(Error(TransactionStatus::StaleAttempt,"Native candidate does not match its accepted force stage"));
  const auto valid=p.publication->ValidatePhysicalCandidate(owner,token,physical,view);
  if(valid.status!=fe::ShellPublicationStatus::Success)return p.Fail(Error(TransactionStatus::PublicationFailure,valid.message));
  if(p.config.activity==ContactActivityPolicy::AllActivePrefix) {
    // Initialization authenticated a wholly active epoch0. Every later owner
    // commit requires this mandatory participant's receipt, proving the active
    // accepted prefix by induction without another full accepted-state read.
    const auto checked=group ? p.active_prefix.CheckPreparedWithinGroup(owner,*p.publication,
        token,physical,view,group->activity) :
        p.active_prefix.CheckPrepared(owner,*p.publication,token,physical,view);
    if(checked.status!=fe::ActivePrefixStatus::Ok)
      return p.Fail({checked.status==fe::ActivePrefixStatus::InactiveParent?TransactionStatus::ActivityChange:
          TransactionStatus::PublicationFailure,checked.message,checked.parent});
  }
  const auto activity_status=p.StageCandidateActivity(token,view,physical);
  if(activity_status.status!=TransactionStatus::Ok)return p.Fail(activity_status);
  if(!p.state.Stage(view,p.trial_selectors))return p.Fail(Error(TransactionStatus::PublicationFailure,"Native selector plan rejected"));
  fe::ShellPhysicalScratchParticipationReceipt result;
  if(group) {
    const auto member=group->publication.CheckNativeCandidateGroupMember(p.issuer,group->count,index);
    if(member.status!=fe::ShellPublicationStatus::Success)
      return p.Fail(Error(TransactionStatus::PublicationFailure,member.message));
  }
  const auto sealed=p.issuer.SealNativeContactCandidate(p.source.source_id,owner,token,view,&result);
  if(sealed.status!=fe::ShellPublicationStatus::Success)return p.Fail(Error(TransactionStatus::PublicationFailure,sealed.message));
  p.phase=Impl::Phase::Sealed;*output=result;return {TransactionStatus::Ok,"OK"};
}
void Transaction::DiscardTrial() noexcept {if(impl_){impl_->owner->Discard();
  if(impl_->issuer.configured())impl_->publication->DiscardTrial();else impl_->issuer.DiscardTrial();impl_->DiscardLocal();}}
fe::NativeContactPublicationSnapshot Transaction::accepted() const noexcept {return impl_?impl_->state.Accepted(*impl_->owner):fe::NativeContactPublicationSnapshot{};}
TransactionReport Transaction::CopyAccepted(AcceptedContactBuffer output,fe::NativeContactPublicationSnapshot* snapshot) const noexcept {
  if(!impl_)return Error(TransactionStatus::NotInitialized,"Native transaction is not initialized");auto& p=*impl_;
  if(!p.usable)return Error(TransactionStatus::Unusable,"Native transaction is poisoned");
  const auto selected=p.state.Accepted(*p.owner);const auto rows=p.source.selection.secondary_count;
  const auto row_bytes=rows*sizeof(NativeGeometryHistory),flag_bytes=rows*sizeof(int);
  using fe::trial_identity::Disjoint;
  if(!selected.available||output.row_capacity<rows||!p.OutputDisjoint(output.rows,row_bytes)||
     !p.OutputDisjoint(output.initial_contact_flags,flag_bytes)||!p.OutputDisjoint(snapshot,sizeof(*snapshot))||
     !Disjoint(output.rows,row_bytes,this,sizeof(*this))||!Disjoint(output.initial_contact_flags,flag_bytes,this,sizeof(*this))||
     !Disjoint(snapshot,sizeof(*snapshot),this,sizeof(*this))||
     !Disjoint(output.rows,row_bytes,&output,sizeof(output))||!Disjoint(output.initial_contact_flags,flag_bytes,&output,sizeof(output))||
     !Disjoint(snapshot,sizeof(*snapshot),&output,sizeof(output))||
     !Disjoint(output.rows,row_bytes,output.initial_contact_flags,flag_bytes)||
     !Disjoint(output.rows,row_bytes,snapshot,sizeof(*snapshot))||!Disjoint(output.initial_contact_flags,flag_bytes,snapshot,sizeof(*snapshot)))
    return Error(TransactionStatus::InvalidInput,"Invalid native accepted snapshot output");
  auto* history=tl::util::ArenaPointer<NativeGeometryHistory>(p.readback.data(),p.readback_rows);
  auto* secondary=tl::util::ArenaPointer<lifecycle::Secondary>(p.readback.data(),p.readback_secondary);
  auto error=cudaGetLastError();
  if(error==cudaSuccess)error=cudaMemcpyAsync(history,p.device.history[selected.selectors.history],row_bytes,cudaMemcpyDeviceToHost,p.stream);
  if(error==cudaSuccess)error=cudaMemcpyAsync(secondary,p.device.secondary[selected.selectors.history],p.readback_secondary.bytes,cudaMemcpyDeviceToHost,p.stream);
  const auto drained=cudaStreamSynchronize(p.stream);
  if(error!=cudaSuccess||drained!=cudaSuccess){p.usable=false;return Error(TransactionStatus::DeviceFailure,"Native accepted readback failed");}
  std::memcpy(output.rows,history,row_bytes);
  for(std::size_t row=0;row<rows;++row)output.initial_contact_flags[row]=secondary[row].initial_contact_flag;
  *snapshot=selected;return {TransactionStatus::Ok,"OK"};
}
} // namespace tlfea::contact::radioss_type25
