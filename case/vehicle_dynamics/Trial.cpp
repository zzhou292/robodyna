#include "Storage.h"
#include "Reports.h"
#include "lib_src/solvers/NodalCinRuntime.h"

namespace crash::cases::vehicle_dynamics {
void VehiclePhysicalDynamics::Storage::Prepare() {
    auto& s=state();
    output::Require(tl::fea::trial_identity::SameStamp(stamp,s.owner.accepted()),"Physical owner changed outside its case");
    detail::Require(activity->CaptureAcceptedPhysical(s.owner,s.publication,{&s.qeph,&s.t3,&s.qbat,&s.type25}),
        "Accepted physical CIN witness activity");
    candidate()={};candidate().base=stamp;
    tl::fea::NodalAssemblyView assembly;
    detail::Require(s.owner.BeginTrial(&token,&assembly),"Begin physical assembly");
    detail::Require(s.qeph.AssembleMappedAccepted(s.owner,token,assembly),"QEPH accepted assembly");
    detail::Require(s.t3.AssembleMappedAccepted(s.owner,token,assembly),"T3 accepted assembly");
    detail::Require(s.qbat.AssembleMappedAccepted(s.owner,token,assembly),"QBAT accepted assembly");
    detail::Require(s.type25.AssembleMappedAccepted(s.owner,token,assembly),"Weld accepted assembly");
    detail::Require(s.type13.AssembleMappedAccepted(s.owner,token,assembly),"Beam accepted assembly");
    detail::Require(s.solids.AssembleAccepted(s.owner,token,assembly),"Solid accepted assembly");
    detail::Require(activity->UploadAttempt(s.owner,token),"Actual accepted CIN witness upload");
    detail::Require(s.owner.SealAssembly(token),"Seal complete physical assembly");
    detail::Require(tl::fea::AdvanceStaggeredCin(s.owner,token,
        {stamp.owner_id,stamp.epoch,assembly.attempt,config.startup.qualification_id,
         config.startup.reserved_step_s,config.maximum_rotation_increment,true}),"Advance physical CIN/rigid owner");
    detail::Require(s.owner.BorrowPrepared(token,&prepared),"Borrow complete prepared physical owner");
}
void VehiclePhysicalDynamics::Storage::Evaluate() {
    auto& s=state();tl::fea::ShellPhysicalDiagnostics d;
    detail::Require(s.qeph.EvaluateCandidate(s.owner,token,prepared,&d.qeph),"QEPH candidate");
    detail::Require(s.t3.EvaluateCandidate(s.owner,token,prepared,&d.t3),"T3 candidate");
    detail::Require(s.qbat.EvaluateCandidate(s.owner,token,prepared,&d.qbat),"QBAT candidate");
    detail::Require(s.type25.EvaluateCandidate(s.owner,token,prepared,&d.type25),"Weld candidate");
    detail::Require(s.type13.EvaluateCandidate(s.owner,token,prepared,&d.type13),"Beam candidate");
    detail::Require(s.solids.EvaluateCandidate(s.owner,token,prepared,&d.solids),"Solid candidate");
    detail::Require(s.publication.PreparePhysical(s.owner,token,
        {&d.qeph,&d.t3,&d.qbat,&d.type25,&d.type13,&d.solids},&candidate().mechanics),"Prepare complete publication");
}
void VehiclePhysicalDynamics::Storage::Capture() {
    tl::fea::NodalPreparedView copied;
    detail::Require(state().owner.CopyPrepared(token,candidate_fields().buffer(),&copied),"Prepared physical fields");
    output::Require(tl::fea::trial_identity::SamePrepared(prepared,copied),"Prepared physical capture identity differs");
    auto& out=candidate();out.proposed_time=copied.proposed_time;
    out.uniform_motion=ObserveUniformMotion(startup.execution().model().coefficients(),candidate_fields(),
        vehicle_runtime::InitialSpeedMps,copied.proposed_time);
}
} // namespace crash::cases::vehicle_dynamics
