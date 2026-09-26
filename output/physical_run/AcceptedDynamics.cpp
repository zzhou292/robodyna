#include "AcceptedInterval.h"
#include "AcceptedWall.h"
#include "AcceptedSelfContact.h"
#include "AcceptedNativeGroup.h"
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"
#include "output/physical_frames/PhysicalAcceptedFrames.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace crash::output::physical_run {
namespace {
template<class T> void CheckParticipant(const T& d,const tl::fea::NodalStamp& s,const Values& v,
    const records::Identity& id) {
    // Dynamics retains the validated prepared observation in its accepted slot.
    // The actual immutable accepted owner stamp below proves common commit.
    Require(d.valid && d.has_completed_interval && d.phase==decltype(d.phase)::Prepared &&
        d.owner_id==s.owner_id && d.configuration_id==id.configuration && d.qualification_id==id.qualification &&
        d.epoch==s.epoch && d.base_epoch==v.stamp.base_epoch && d.attempt==v.stamp.attempt &&
        Bits(d.time)==Bits(v.stamp.time) && Bits(d.base_time)==Bits(v.stamp.base_time) &&
        Bits(d.velocity_time)==Bits(v.stamp.velocity_time) && Bits(d.kick_dt)==Bits(v.stamp.kick_dt),
        "Actual committed physical participant observation differs");
}
}
AcceptedInterval CaptureAcceptedInterval(const cases::vehicle_dynamics::VehiclePhysicalDynamics& run,
    const physical_frames::PhysicalAcceptedFrames& capture,Profile profile) {
    const auto& context=capture.context(); // Existing live capture authenticated complete source/owner.
    const auto& identity=context.identity();
    const auto stamp=run.accepted();
    Require(stamp.owner_id==identity.owner && stamp.epoch && !run.has_prepared_step() &&
        Bits(stamp.fixed_dt)==Bits(context.fixed_dt()) && stamp.reactions_valid,
        "Physical ledger requires an actual committed dynamics endpoint");
    const auto& step=run.last_accepted_step();
    const auto& d=step.mechanics;
    Require(profile.type45==d.has_type45,"Physical interval profile differs from actual joint participant");
    const auto* beam_model=capture.mapping().structural_beams();
    Require(profile.beam18==d.has_beam18 && d.has_beam18==bool(beam_model),
        "Physical interval profile differs from actual structural beam participant");
    Require(d.valid && !d.kinetic_available && d.has_qeph && d.has_t3 && d.has_qbat &&
        d.has_type25 && d.has_type13 && d.has_solids && step.base.owner_id==stamp.owner_id &&
        step.base.epoch==stamp.reaction_base_epoch && Bits(step.base.time)==Bits(stamp.reaction_time) &&
        Bits(step.proposed_time)==Bits(stamp.time),"Physical accepted observation is incomplete or stale");
    Values value{stamp.owner_id,{stamp.epoch,stamp.reaction_base_epoch,d.qeph.attempt,stamp.time,
        stamp.reaction_time,stamp.velocity_time,stamp.reaction_kick_dt},std::nullopt};
    if(profile.structural_limit)value.structural_limit_s=step.structural_step_limit;
    else Require(step.structural_step_limit==0,"Physical profile hides an available structural bound");
    detail::CaptureSelfContact(run,identity,profile,value);
    detail::CaptureNativeGroup(run,profile,value);
    CheckValues(context,profile,value);
    CheckParticipant(d.qeph,stamp,value,identity);CheckParticipant(d.t3,stamp,value,identity);
    CheckParticipant(d.qbat,stamp,value,identity);CheckParticipant(d.type25,stamp,value,identity);
    CheckParticipant(d.type13,stamp,value,identity);CheckParticipant(d.solids,stamp,value,identity);
    if(d.has_type45) {
        CheckParticipant(d.type45,stamp,value,identity);
        Require(d.type45.source_instance_id==identity.source_instance && d.type45.joint_count &&
            d.type45.automatic_stiffness_initialized,"Committed joint source/count/automatic phase differs");
    }
    if(d.has_beam18) {
        CheckParticipant(d.beam18,stamp,value,identity);
        Require(d.beam18.source_instance_id==identity.source_instance &&
                d.beam18.source_instance_id==beam_model->source_instance_id() &&
                d.beam18.parent_count==beam_model->parents().size() && d.beam18.parent_count &&
                d.beam18.accepted_force_assembled &&
                Bits(d.beam18.base_velocity_time)==Bits(step.base.velocity_time),
                "Committed structural beam source/count differs");
    }
    auto wall=detail::CaptureWall(run,identity,value);
    Require(tl::fea::trial_identity::SameStamp(stamp,run.accepted()),"Physical owner changed during accepted observation");
    return AcceptedInterval(std::move(value),identity,profile,std::move(wall));
}
} // namespace crash::output::physical_run
