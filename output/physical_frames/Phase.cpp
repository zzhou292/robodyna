#include "Fields.h"
#include "ParticipantPhase.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace crash::output::physical_frames::detail {
records::FrameStamp Phase(const CaptureScope& c) {
    const auto& s=c.stamp;
    const auto& d=c.diagnostics;
    Require(d.valid && !d.kinetic_available && d.has_qeph && d.has_t3 && d.has_qbat &&
        d.has_type25 && d.has_type13 && d.has_solids && s.owner_id && s.node_count &&
        s.temporal_scheme==tl::fea::NodalTemporalScheme::StaggeredHalfKickStart,
        "Capture requires the complete accepted physical publication");
    records::FrameStamp f;
    if(s.epoch) f={s.epoch,s.reaction_base_epoch,d.qeph.attempt,s.time,s.reaction_time,s.velocity_time,s.reaction_kick_dt};
    else Require(!s.reactions_valid && s.time==0 && s.velocity_time==0,"Initial accepted owner has advanced");
    records::CheckStamp(s.fixed_dt,f);
    const auto config=d.qeph.configuration_id,qualification=d.qeph.qualification_id;
    Require(config && qualification,"Missing accepted publication configuration");
    CheckAcceptedParticipant(d.qeph,s,f,config,qualification);
    CheckAcceptedParticipant(d.t3,s,f,config,qualification);
    CheckAcceptedParticipant(d.qbat,s,f,config,qualification);
    CheckAcceptedParticipant(d.type25,s,f,config,qualification);
    CheckAcceptedParticipant(d.type13,s,f,config,qualification);
    CheckAcceptedParticipant(d.solids,s,f,config,qualification);
    Require(d.has_type45==bool(c.type45_joint_count) &&
        d.has_type45==bool(c.type45_source_instance_id),"Accepted joint source presence differs");
    if(d.has_type45) {
        CheckAcceptedParticipant(d.type45,s,f,config,qualification);
        Require(d.type45.source_instance_id==c.type45_source_instance_id &&
            d.type45.joint_count==c.type45_joint_count &&
            d.type45.automatic_stiffness_initialized==bool(s.epoch),
            "Accepted joint source/count/automatic-stiffness phase differs");
        if(!s.epoch)Require(d.type45.attempt==0 && d.type45.base_epoch==0 &&
            d.type45.base_time==0 && d.type45.velocity_time==0 && d.type45.base_velocity_time==0 && d.type45.kick_dt==0,
            "Initial accepted joints contain a future interval phase");
    }
    Require(d.has_beam18==bool(c.beam18_parent_count) &&
        d.has_beam18==bool(c.beam18_source_instance_id),"Accepted structural beam source presence differs");
    if(d.has_beam18) {
        CheckAcceptedParticipant(d.beam18,s,f,config,qualification);
        Require(d.beam18.source_instance_id==c.beam18_source_instance_id &&
                d.beam18.parent_count==c.beam18_parent_count &&
                d.beam18.accepted_force_assembled==bool(s.epoch) &&
                Bits(d.beam18.base_velocity_time)==Bits(d.qeph.base_velocity_time),
                "Accepted structural beam source/count differs");
        if(!s.epoch)Require(d.beam18.attempt==0 && d.beam18.base_epoch==0 &&
            d.beam18.base_time==0 && d.beam18.velocity_time==0 && d.beam18.base_velocity_time==0 &&
            d.beam18.kick_dt==0,"Initial accepted structural beams contain a future interval phase");
    }
    return f;
}
void CheckReadback(const CaptureScope& scope,const tl::fea::qeph::BatchDiagnostics& value) {
    CheckAcceptedParticipant(value,scope.stamp,Phase(scope),scope.diagnostics.qeph.configuration_id,scope.diagnostics.qeph.qualification_id);
}
void CheckReadback(const CaptureScope& scope,const tl::fea::t3::BatchDiagnostics& value) {
    CheckAcceptedParticipant(value,scope.stamp,Phase(scope),scope.diagnostics.t3.configuration_id,scope.diagnostics.t3.qualification_id);
}
void CheckReadback(const CaptureScope& scope,const tl::fea::qbat::BatchDiagnostics& value) {
    CheckAcceptedParticipant(value,scope.stamp,Phase(scope),scope.diagnostics.qbat.configuration_id,scope.diagnostics.qbat.qualification_id);
}
void CheckSameEndpoint(const CaptureScope& a,const CaptureScope& b) {
    Require(tl::fea::trial_identity::SameStamp(a.stamp,b.stamp) && records::SameStamp(Phase(a),Phase(b)) &&
        a.diagnostics.qeph.configuration_id==b.diagnostics.qeph.configuration_id &&
        a.diagnostics.qeph.qualification_id==b.diagnostics.qeph.qualification_id,
        "Accepted owner changed during serialized capture");
    Require(a.type45_joint_count==b.type45_joint_count &&
        a.type45_source_instance_id==b.type45_source_instance_id &&
        a.diagnostics.has_type45==b.diagnostics.has_type45,"Accepted joint source changed during capture");
    Require(a.beam18_parent_count==b.beam18_parent_count &&
        a.beam18_source_instance_id==b.beam18_source_instance_id &&
        a.diagnostics.has_beam18==b.diagnostics.has_beam18,
        "Accepted structural beam source changed during capture");
}
} // namespace crash::output::physical_frames::detail
