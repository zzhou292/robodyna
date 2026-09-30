#include "NativeCaptureChecks.h"
#include "ParticipantPhase.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace crash::output::physical_frames::detail {
records::FrameStamp NativePhase(const NativeCaptureScope& c) {
    const auto& s=c.stamp;const auto& d=c.diagnostics;const auto& n=c.contact;const auto& info=c.source;
    Require(d.valid&&!d.kinetic_available&&d.has_qeph&&d.has_t3&&!d.has_qbat&&!d.has_type25&&!d.has_type13&&
        !d.has_solids&&!d.has_type45&&!d.has_beam18&&s.owner_id&&s.node_count&&
        s.temporal_scheme==tl::fea::NodalTemporalScheme::StaggeredHalfKickStart,
        "Native scene capture requires its exact common physical participant set");
    Require(info.available&&info.source_id&&info.topology_generation&&info.source_generation&&
        info.nodes==s.node_count&&info.secondaries&&info.secondaries<=info.nodes&&info.primary_mains&&
        info.primary_mains<=SIZE_MAX/2&&info.expanded_mains==2*info.primary_mains&&n.available&&
        tl::fea::trial_identity::SameStamp(n.stamp,s),"Native contact source/publication is unavailable or stale");
    records::FrameStamp frame;
    if(s.epoch) {
        Require(s.reactions_valid&&n.force_phase_available&&n.selectors.has_reference&&n.selectors.reference_generation&&n.generation==s.epoch&&n.selectors.reference_generation<=n.generation&&
            n.force_base_stamp.owner_id==s.owner_id&&n.force_base_stamp.node_count==s.node_count&&
            n.force_base_stamp.epoch==s.reaction_base_epoch&&Bits(n.force_base_stamp.time)==Bits(s.reaction_time)&&
            Bits(n.force_base_stamp.fixed_dt)==Bits(s.fixed_dt)&&
            tl::fea::trial_identity::SameStamp(n.force_base_stamp,d.base_stamp),
            "Native accepted contact history has another force-base owner phase");
        frame={s.epoch,s.reaction_base_epoch,d.qeph.attempt,s.time,s.reaction_time,s.velocity_time,s.reaction_kick_dt};
        Require(Bits(d.qeph.base_velocity_time)==Bits(n.force_base_stamp.velocity_time)&&
            Bits(d.t3.base_velocity_time)==Bits(n.force_base_stamp.velocity_time),"Native and material base velocity phases differ");
    } else Require(!s.reactions_valid&&s.time==0&&s.velocity_time==0&&!n.force_phase_available&&n.generation==0&&
        !n.selectors.has_reference&&n.selectors.reference_generation==0&&
        tl::fea::trial_identity::SameStamp(d.base_stamp,s),"Initial native capture contains a future force/reference phase");
    records::CheckStamp(s.fixed_dt,frame);
    const auto config=d.qeph.configuration_id,qualification=d.qeph.qualification_id;
    Require(config&&qualification,"Missing native physical publication identity");
    CheckAcceptedParticipant(d.qeph,s,frame,config,qualification);CheckAcceptedParticipant(d.t3,s,frame,config,qualification);
    return frame;
}
void CheckSameNativeScope(const NativeCaptureScope& a,const NativeCaptureScope& b) {
    const auto af=NativePhase(a),bf=NativePhase(b);
    Require(tl::fea::trial_identity::SameStamp(a.stamp,b.stamp)&&records::SameStamp(af,bf)&&
        a.diagnostics.qeph.configuration_id==b.diagnostics.qeph.configuration_id&&
        a.diagnostics.qeph.qualification_id==b.diagnostics.qeph.qualification_id&&
        a.contact.generation==b.contact.generation&&a.contact.force_phase_available==b.contact.force_phase_available&&
        (!a.contact.force_phase_available||tl::fea::trial_identity::SameStamp(a.contact.force_base_stamp,b.contact.force_base_stamp))&&
        a.contact.selectors.history==b.contact.selectors.history&&a.contact.selectors.reference==b.contact.selectors.reference&&
        a.contact.selectors.reference_generation==b.contact.selectors.reference_generation&&
        a.contact.selectors.activity==b.contact.selectors.activity&&
        a.contact.selectors.activity_generation==b.contact.selectors.activity_generation&&
        a.contact.selectors.reference_activity_generation==b.contact.selectors.reference_activity_generation&&
        a.source.source_id==b.source.source_id&&a.source.topology_generation==b.source.topology_generation&&
        a.source.source_generation==b.source.source_generation&&a.source.nodes==b.source.nodes&&
        a.source.secondaries==b.source.secondaries&&a.source.primary_mains==b.source.primary_mains&&a.source.expanded_mains==b.source.expanded_mains,
        "Accepted native physical source or endpoint changed during capture");
}
physical_run::NativeContactValues NativeContactObservation(const NativeCaptureScope& c) {
    NativePhase(c);Require(c.stamp.epoch&&c.contact.force_phase_available,"Initial state has no completed native interval");
    const auto& i=c.source;const auto& n=c.contact;
    physical_run::NativeContactValues out{i.source_id,i.topology_generation,i.source_generation,i.nodes,i.secondaries,
        i.primary_mains,i.expanded_mains,n.generation,n.selectors.reference_generation,n.force_base_stamp.epoch,
        n.force_base_stamp.time,n.force_base_stamp.velocity_time};
    physical_run::CheckNativeContactValues(out);return out;
}
}
