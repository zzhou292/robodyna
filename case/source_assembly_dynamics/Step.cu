#include "State.h"
#include "lib_src/solvers/ExplicitNodalRigidStep.h"
#include <cuda_runtime.h>
#include <cmath>

namespace crash::cases::source_assembly_dynamics {
namespace {
Report CopyLoads(const fe::NodalAssemblyView& v,std::vector<double>& soa,
                 std::vector<double>& force,std::vector<double>& couple) {
    const auto n=v.forces.node_count;
    if(!n||soa.size()!=6*n||force.size()!=3*n||couple.size()!=3*n)
        return Failure(Status::ComponentFailure,"Exact assembled load snapshot extent is invalid");
    const double* arrays[]{v.forces.force_x,v.forces.force_y,v.forces.force_z,
                           v.forces.couple_x,v.forces.couple_y,v.forces.couple_z};
    for(unsigned a=0;a<6;++a) {
        if(!arrays[a])return Failure(Status::ComponentFailure,"Missing actual assembly load component");
        if(cudaMemcpyAsync(soa.data()+a*n,arrays[a],n*sizeof(double),cudaMemcpyDeviceToHost,v.stream)!=cudaSuccess)
            return Failure(Status::DeviceFailure,"Actual assembled load readback failed");
    }
    if(cudaStreamSynchronize(v.stream)!=cudaSuccess)
        return Failure(Status::DeviceFailure,"Actual assembled load readback stream failed");
    for(std::size_t node=0;node<n;++node)for(unsigned a=0;a<3;++a) {
        const double f=soa[a*n+node],c=soa[(a+3)*n+node];
        if(!std::isfinite(f)||!std::isfinite(c))
            return Failure(Status::EnvelopeFailure,"Actual assembled load is nonfinite",0,node);
        force[3*node+a]=f;couple[3*node+a]=c;
    }
    return Success();
}
}
Report SourceAssemblyWallCase::Impl::Prepare() {
    const auto stamp=owner.accepted();const auto& old=accepted().diagnostics;
    if(!fe::trial_identity::SameStamp(stamp,old.stamp))
        return Failure(Status::ComponentFailure,"Owner changed outside its assembly wall case");
    candidate().diagnostics={};candidate().has_force_stage=false;
    prepared={};group_prepared={};force_capture.prepared={};base_contact={};
    fe::NodalAssemblyView assembly;
    auto r=timer.Measure<StepStage::BeginTrial>([&] {return Convert(owner.BeginTrial(&token,&assembly));});if(!r)return r;
    r=timer.Measure<StepStage::AssembleQeph>([&] {return QReport(qeph.AssembleAccepted(owner,assembly));});if(!r)return r;
    r=timer.Measure<StepStage::AssembleT3>([&] {return TReport(t3.AssembleAccepted(owner,assembly));});if(!r)return r;
    r=AssembleConnector(assembly);if(!r)return r;
    // Contact is last so its certificate includes the actual preceding native
    // shell-force additions. Keep this order when observing applied work.
    r=timer.Measure<StepStage::AssembleWall>([&] {return Convert(wall.AssembleAccepted(owner,assembly,&base_contact));});if(!r)return r;
    if(!base_contact.valid||base_contact.phase!=contact::NodalWallDevicePhase::AcceptedBase||
       base_contact.owner_id!=stamp.owner_id||base_contact.base_epoch!=stamp.epoch||
       base_contact.attempt!=assembly.attempt||base_contact.time!=stamp.time||
       base_contact.velocity_time!=stamp.velocity_time||
       base_contact.configuration_id!=setup.settings()->configuration_id||
       base_contact.qualification_id!=setup.settings()->qualification_id||
       base_contact.wall_binding_id!=setup.settings()->wall_binding_id)
        return Failure(Status::ComponentFailure,"Accepted-base contact source/attempt/timing mismatch");
    if(!stamp.epoch&&(base_contact.resultant.value!=0||base_contact.potential.value!=0||
                     base_contact.maximum_penetration!=0))
        return Failure(Status::EnvelopeFailure,"Certified initially separated assembly has nonzero initial contact");
    // Snapshot is complete while assembly pointers are still valid. No candidate
    // evaluation below scatters into or recomputes this accepted-base load.
    r=timer.Measure<StepStage::CopyAppliedLoads>([&] {return CopyLoads(assembly,load_soa,applied_force,applied_couple);});if(!r)return r;
    r=timer.Measure<StepStage::SealAssembly>([&] {return Convert(owner.SealAssembly(token));});if(!r)return r;
    const fe::NodalStaggeredHistoryAdmission admission{stamp.owner_id,stamp.epoch,assembly.attempt,
        config.fixed_dt,config.deformation.maximum_rotation_increment,setup.settings()->qualification_id};
    r=timer.Measure<StepStage::AdvanceOwner>([&] {return Convert(fe::AdvanceStaggeredRigidGroups(owner,token,admission));});if(!r)return r;
    r=timer.Measure<StepStage::ReadPreparedNodes>([&] {return Convert(owner.CopyPrepared(token,candidate().fields.buffer(),&prepared));});if(!r)return r;
    r=timer.Measure<StepStage::ReadPreparedGroups>([&] {
        return Convert(owner.CopyPreparedRigidGroups(token,{candidate().fields.groups.data(),groups()},&group_prepared));
    });if(!r)return r;
    if(!fe::trial_identity::SamePrepared(prepared,group_prepared))
        return Failure(Status::ComponentFailure,"Candidate nodal/reaction/group readbacks identify different prepared states");
    return CaptureForceStage();
}
Report SourceAssemblyWallCase::Impl::Evaluate() {
    auto& next=candidate();auto& d=next.diagnostics.shells;
    auto r=timer.Measure<StepStage::EvaluateQeph>([&] {return QReport(qeph.EvaluateCandidate(owner,token,prepared,&d.qeph));});if(!r)return r;
    r=timer.Measure<StepStage::EvaluateT3>([&] {return TReport(t3.EvaluateCandidate(owner,token,prepared,&d.t3));});if(!r)return r;
    r=timer.Measure<StepStage::ReadQephResults>([&] {return QReport(qeph.CopyPreparedResults(d.qeph,next.parents.qeph.data(),quads()));});if(!r)return r;
    r=timer.Measure<StepStage::ReadT3Results>([&] {return TReport(t3.CopyPreparedResults(d.t3,next.parents.t3.data(),triangles()));});if(!r)return r;
    r=timer.Measure<StepStage::ReadQephSections>([&] {return QReport(qeph.CopyPreparedSectionHistory(d.qeph,next.parents.qsection.data(),quads()));});if(!r)return r;
    r=timer.Measure<StepStage::ReadT3Sections>([&] {return TReport(t3.CopyPreparedSectionHistory(d.t3,next.parents.tsection.data(),triangles()));});if(!r)return r;
    r=EvaluateConnector();if(!r)return r;
    contact::NodalWallDiagnostics contact_diagnostics;
    r=timer.Measure<StepStage::EvaluateWall>([&] {return Convert(wall.EvaluateCandidate(owner,token,prepared,&contact_diagnostics));});if(!r)return r;
    r=timer.Measure<StepStage::ReadWallResults>([&] {return Convert(wall.CopyResults(contact_diagnostics,next.wall.buffer()));});if(!r)return r;
    fe::ShellBatchDiagnostics common;
    r=timer.Measure<StepStage::PreparePublication>([&] {return PreparePublication(common);});if(!r)return r;
    d=common;return Success();
}
Report SourceAssemblyWallCase::Impl::CheckMotion() {
    const auto& old=accepted();auto& next=candidate();observation::Input input;
    input.bindings=&bindings;input.base=old.diagnostics.stamp;input.prepared=prepared;
    input.before=old.fields.view();input.after=next.fields.view();
    input.before_groups=old.fields.groups.data();input.after_groups=next.fields.groups.data();input.group_count=groups();
    input.applied_force_xyz=applied_force.data();input.applied_couple_xyz=applied_couple.data();
    input.reaction_force_xyz=next.fields.reaction.data();input.reaction_couple_xyz=next.fields.couple.data();
    input.base_kinetic=next.diagnostics.shells.base_kinetic;input.kinetic=next.diagnostics.shells.kinetic;
    return Convert(observation::ObserveInterval(input,&next.diagnostics.motion));
}
Report SourceAssemblyWallCase::Impl::Check() {
    auto r=timer.Measure<StepStage::CheckShells>([&] {return CheckShells();});if(!r)return r;
    r=CheckConnector();if(!r)return r;
    r=timer.Measure<StepStage::CheckContact>([&] {return CheckContact();});if(!r)return r;
    r=timer.Measure<StepStage::CheckMotion>([&] {return CheckMotion();});if(!r)return r;
    r=CheckForceStage();if(!r)return r;
    return CheckQephSpin();
}
Report SourceAssemblyWallCase::Impl::Commit() {
    auto& next=candidate();const auto& d=next.diagnostics.shells;
    const fe::NodalValidationReceipt receipt{d.qeph.owner_id,d.qeph.base_epoch,d.qeph.attempt,d.qeph.qualification_id,true};
    const auto r=timer.Measure<StepStage::CommitPublication>([&] {return Convert(publication.Commit(owner,token,d,receipt));});if(!r)return r;
    // Only infallible value updates and selection follow the sole owner/native
    // history publication. Contact keeps its completed candidate phase tag.
    next.diagnostics.stamp=owner.accepted();next.diagnostics.has_interval=true;
    if(spin) {
        auto& record=(*spin)[1-accepted_slot];record.enclosing=next.diagnostics.stamp;record.completed=true;
    }
    next.group_stamp=next.diagnostics.stamp;
    next.diagnostics.shells.qeph.phase=q::BatchPhase::Accepted;
    next.diagnostics.shells.t3.phase=t::BatchPhase::Accepted;
    if(connector)next.diagnostics.shells.connector.phase=fe::type25::BatchPhase::Accepted;
    accepted_slot=1-accepted_slot;
    return Success();
}
} // namespace crash::cases::source_assembly_dynamics
