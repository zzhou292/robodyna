#include "../Fields.h"
#include <gtest/gtest.h>
namespace crash::output::physical_frames::test {
namespace {
template<class Diagnostics> void Initialize(Diagnostics& value,const tl::fea::NodalStamp& s,bool advanced) {
    value.valid=true;value.owner_id=s.owner_id;value.configuration_id=32;value.qualification_id=33;
    value.phase=decltype(value.phase)::Accepted;value.epoch=s.epoch;value.attempt=advanced?8:0;
    value.time=s.time;value.velocity_time=s.velocity_time;value.kick_dt=s.reaction_kick_dt;
    value.has_completed_interval=advanced;
}
detail::CaptureScope Scope(bool joint,bool advanced) {
    detail::CaptureScope scope;
    auto& s=scope.stamp;
    s.owner_id=31;s.node_count=5;s.fixed_dt=.125;
    s.temporal_scheme=tl::fea::NodalTemporalScheme::StaggeredHalfKickStart;
    s.epoch=advanced?1:0;s.time=advanced?.125:0;s.velocity_time=advanced?.0625:0;
    s.reactions_valid=advanced;s.reaction_kick_dt=advanced?.0625:0;
    auto& d=scope.diagnostics;
    d.valid=d.has_qeph=d.has_t3=d.has_qbat=d.has_type25=d.has_type13=d.has_solids=true;
    d.has_type45=joint;
    const auto initialize=[&](auto& value) {Initialize(value,s,advanced);};
    initialize(d.qeph);initialize(d.t3);initialize(d.qbat);
    initialize(d.type25);initialize(d.type13);initialize(d.solids);
    if(joint) {
        scope.type45_source_instance_id=91;scope.type45_joint_count=38;
        initialize(d.type45);d.type45.source_instance_id=91;d.type45.joint_count=38;
        d.type45.automatic_stiffness_initialized=advanced;
    }
    return scope;
}
detail::CaptureScope BeamScope(bool beam,bool advanced,bool joint) {
    auto scope=Scope(joint,advanced);
    if(beam) {
        scope.beam18_source_instance_id=91;scope.beam18_parent_count=142;
        auto& d=scope.diagnostics;
        d.has_beam18=true;
        Initialize(d.beam18,scope.stamp,advanced);
        d.beam18.source_instance_id=91;d.beam18.parent_count=142;
        d.beam18.accepted_force_assembled=advanced;
    }
    return scope;
}
}
TEST(PhysicalCaptureJoints, ActualOptionalRoleChecksSourceCountAndAutomaticPhase) {
    for(bool joint:{false,true})for(bool advanced:{false,true}) {
        const auto good=Scope(joint,advanced);
        EXPECT_EQ(detail::Phase(good).epoch,advanced?1u:0u);
        auto bad=good;bad.diagnostics.has_type45=!joint;
        EXPECT_THROW(detail::Phase(bad),std::exception);
        if(!joint)continue;
        bad=good;++bad.diagnostics.type45.source_instance_id;
        EXPECT_THROW(detail::Phase(bad),std::exception);
        bad=good;--bad.diagnostics.type45.joint_count;
        EXPECT_THROW(detail::Phase(bad),std::exception);
        bad=good;bad.diagnostics.type45.automatic_stiffness_initialized=!advanced;
        EXPECT_THROW(detail::Phase(bad),std::exception);
        bad=good;bad.diagnostics.type45.phase=tl::fea::type45::BatchPhase::Prepared;
        EXPECT_THROW(detail::Phase(bad),std::exception);
        if(!advanced) {
            bad=good;bad.diagnostics.type45.velocity_time=.0625;
            EXPECT_THROW(detail::Phase(bad),std::exception);
        }
        EXPECT_NO_THROW(detail::Phase(good));
    }
}
TEST(PhysicalCaptureJoints, LateSeventhPhaseAndSourceChangesRejectBeforePublication) {
    const auto good=Scope(true,true);
    auto bad=good;++bad.diagnostics.type45.attempt;
    EXPECT_THROW(detail::CheckSameEndpoint(good,bad),std::exception);
    bad=good;bad.diagnostics.type45.kick_dt=.125;
    EXPECT_THROW(detail::CheckSameEndpoint(good,bad),std::exception);
    bad=good;--bad.type45_joint_count;--bad.diagnostics.type45.joint_count;
    EXPECT_NO_THROW(detail::Phase(bad));
    EXPECT_THROW(detail::CheckSameEndpoint(good,bad),std::exception);
    EXPECT_NO_THROW(detail::CheckSameEndpoint(good,good));
}
TEST(PhysicalCaptureBeams, ActualOptionalRoleChecksSourceCountAndInitialAcceptedPhase) {
    for(bool joint:{false,true})for(bool beam:{false,true})for(bool advanced:{false,true}) {
        const auto good=BeamScope(beam,advanced,joint);
        EXPECT_EQ(detail::Phase(good).epoch,advanced?1u:0u);
        auto bad=good;bad.diagnostics.has_beam18=!beam;
        EXPECT_THROW(detail::Phase(bad),std::exception);
        if(!beam)continue;
        for(unsigned fault=0;fault<12;++fault) {
            SCOPED_TRACE(fault);
            bad=good;
            if(fault==0)++bad.diagnostics.beam18.source_instance_id;
            if(fault==1)--bad.diagnostics.beam18.parent_count;
            if(fault==2)bad.diagnostics.beam18.phase=tl::fea::beam18::BatchPhase::Prepared;
            if(fault==3)++bad.diagnostics.beam18.attempt;
            if(fault==4)bad.diagnostics.beam18.velocity_time+=.0625;
            if(fault==5)bad.diagnostics.beam18.base_velocity_time=.0625;
            if(fault==6)bad.diagnostics.beam18.accepted_force_assembled=!advanced;
            if(fault==7)bad.beam18_parent_count=0;
            if(fault==8)++bad.diagnostics.beam18.epoch;
            if(fault==9)++bad.diagnostics.beam18.base_epoch;
            if(fault==10)bad.diagnostics.beam18.base_time+=.0625;
            if(fault==11)++bad.diagnostics.beam18.owner_id;
            EXPECT_THROW(detail::Phase(bad),std::exception);
        }
        EXPECT_NO_THROW(detail::Phase(good));
    }
}
TEST(PhysicalCaptureBeams, EighthParticipantSourceOrAttemptChangeRejectsWholeCapture) {
    const auto good=BeamScope(true,true,true);
    auto bad=good;++bad.diagnostics.beam18.attempt;
    EXPECT_THROW(detail::CheckSameEndpoint(good,bad),std::exception);
    bad=good;++bad.diagnostics.beam18.source_instance_id;++bad.beam18_source_instance_id;
    EXPECT_NO_THROW(detail::Phase(bad));
    EXPECT_THROW(detail::CheckSameEndpoint(good,bad),std::exception);
    bad=good;--bad.diagnostics.beam18.parent_count;--bad.beam18_parent_count;
    EXPECT_NO_THROW(detail::Phase(bad));
    EXPECT_THROW(detail::CheckSameEndpoint(good,bad),std::exception);
    bad=BeamScope(false,true,true);
    EXPECT_NO_THROW(detail::Phase(bad));
    EXPECT_THROW(detail::CheckSameEndpoint(good,bad),std::exception);
    EXPECT_NO_THROW(detail::CheckSameEndpoint(good,good));
}
} // namespace crash::output::physical_frames::test
