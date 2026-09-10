#include "SourcePartElasticInternal.h"
#include "lib_src/solvers/ExplicitNodalStep.h"

namespace crash::cases::source_part_elastic {
namespace {
__global__ void AddPulse(fe::NodalAssemblyView view,const double* force,double scale) {
    const unsigned n=threadIdx.x;
    if(n<view.accepted.node_count) {
        view.forces.force_x[n]+=force[3*n]*scale;
        view.forces.force_y[n]+=force[3*n+1]*scale;
        view.forces.force_z[n]+=force[3*n+2]*scale;
    }
}
bool ReadPrepared(const fe::NodalPreparedView& p,Snapshot& out) {
    const auto copy=[&](double* to,const double* from,std::size_t count) {
        return cudaMemcpyAsync(to,from,count*sizeof(double),cudaMemcpyDeviceToHost,p.stream)==cudaSuccess;
    };
    return copy(out.position.data(),p.kinematics.position_xyz,3*NodeCount)&&
        copy(out.velocity.data(),p.kinematics.velocity_xyz,3*NodeCount)&&
        copy(out.orientation.data(),p.kinematics.orientation_wxyz,4*NodeCount)&&
        copy(out.omega.data(),p.kinematics.angular_velocity_xyz,3*NodeCount)&&
        cudaStreamSynchronize(p.stream)==cudaSuccess;
}
}
Report SourcePartElasticCase::Impl::Prepare() {
    const auto stamp=owner.accepted();
    if(stamp.epoch!=accepted.stamp.epoch||stamp.owner_id!=accepted.stamp.owner_id||stamp.time!=accepted.stamp.time)
        return Failure(Status::ComponentFailure,"Owner was advanced outside its source-part case");
    trial={};
    fe::NodalAssemblyView assembly;
    auto nr=owner.BeginTrial(&token,&assembly);
    if(nr.status!=fe::NodalStatus::Ok) return Failure(Status::ComponentFailure,nr.message);
    if(config.experiment==Experiment::ElasticPulse) {
        AddPulse<<<1,128,0,assembly.stream>>>(assembly,device_pulse,PulseScale(stamp.time,config.pulse_duration));
        if(cudaGetLastError()!=cudaSuccess) return Failure(Status::DeviceFailure,"Pulse contributor launch failed");
    }
    const auto qr=qeph.AssembleAccepted(assembly);
    if(qr.status!=q::BatchStatus::Success) return Failure(Status::ComponentFailure,qr.message,0,0,
        qr.element<source::Q4Count?binding.qeph_source_id(qr.element):0);
    const auto tr=t3.AssembleAccepted(assembly);
    if(tr.status!=t::BatchStatus::Success) return Failure(Status::ComponentFailure,tr.message,0,0,
        tr.element<source::T3Count?binding.t3_source_id(tr.element):0);
    if(wall) {
        const auto wr=AssembleWall(assembly);
        if(!wr) return wr; // Last contributor measures actual force-addition roundoff.
    }
    nr=owner.SealAssembly(token);
    if(nr.status!=fe::NodalStatus::Ok) return Failure(Status::ComponentFailure,nr.message);
    nr=fe::AdvanceStaggeredHistory(owner,token,{assembly.owner_id,assembly.accepted.base_epoch,
        assembly.attempt,config.dt,1,config.qualification_id});
    if(nr.status!=fe::NodalStatus::Ok) return Failure(Status::ComponentFailure,nr.message);
    nr=owner.BorrowPrepared(token,&prepared);
    if(nr.status!=fe::NodalStatus::Ok) return Failure(Status::ComponentFailure,nr.message);
    if(!ReadPrepared(prepared,trial)) return Failure(Status::DeviceFailure,"Candidate nodal readback failed");
    return Success();
}
Report SourcePartElasticCase::Impl::EvaluateShells() {
    auto& d=trial.diagnostics.shells;
    auto qr=qeph.EvaluateCandidate(prepared,&d.qeph);
    if(qr.status!=q::BatchStatus::Success) return Failure(Status::ComponentFailure,qr.message,0,0,
        qr.element<source::Q4Count?binding.qeph_source_id(qr.element):0);
    qr=qeph.CopyPreparedResults(d.qeph,qresult.data(),qresult.size());
    if(qr.status!=q::BatchStatus::Success) return Failure(Status::ComponentFailure,qr.message);
    auto tr=t3.EvaluateCandidate(prepared,&d.t3);
    if(tr.status!=t::BatchStatus::Success) return Failure(Status::ComponentFailure,tr.message,0,0,
        tr.element<source::T3Count?binding.t3_source_id(tr.element):0);
    tr=t3.CopyPreparedResults(d.t3,tresult.data(),tresult.size());
    if(tr.status!=t::BatchStatus::Success) return Failure(Status::ComponentFailure,tr.message);
    if(config.material_model!=MaterialModel::ElasticLaw1) return ObservePlasticSections();
    return Success();
}
Report SourcePartElasticCase::Impl::Evaluate() {
    const auto shells=EvaluateShells();
    if(!shells) return shells;
    if(wall) {
        const auto wr=EvaluateWall();
        if(!wr) return wr;
    }
    auto& d=trial.diagnostics.shells;
    fe::ShellBatchDiagnostics common;
    const auto pr=publication.Prepare(owner,token,d.qeph,d.t3,&common);
    if(pr.status!=fe::ShellPublicationStatus::Success) return Failure(Status::ComponentFailure,pr.message);
    d=common;
    return Success();
}
Report SourcePartElasticCase::Impl::Commit() {
    const auto& d=trial.diagnostics.shells;
    const auto pr=publication.Commit(owner,token,d,{d.qeph.owner_id,d.qeph.base_epoch,d.qeph.attempt,
        d.qeph.qualification_id,true});
    if(pr.status!=fe::ShellPublicationStatus::Success) return Failure(Status::ComponentFailure,pr.message);
    // No fallible work follows the owner+two-history publication. Actual owner
    // metadata is retained; scratch diagnostic phase is changed by value only.
    trial.stamp=owner.accepted();
    trial.diagnostics.shells.qeph.phase=q::BatchPhase::Accepted;
    trial.diagnostics.shells.t3.phase=t::BatchPhase::Accepted;
    accepted=trial;
    if(config.material_model!=MaterialModel::ElasticLaw1) accepted_plastic=trial_plastic;
    accepted_q_work_magnitude=trial_q_work_magnitude;
    accepted_t_work_magnitude=trial_t_work_magnitude;
    CommitWall();
    return Success();
}
} // namespace crash::cases::source_part_elastic
