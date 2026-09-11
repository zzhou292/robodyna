#include "../Fields.h"
#include <gtest/gtest.h>
namespace crash::output::physical_frames::test {
namespace {
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
    const auto initialize=[&](auto& value) {
        value.valid=true;value.owner_id=s.owner_id;value.configuration_id=32;value.qualification_id=33;
        value.phase=decltype(value.phase)::Accepted;value.epoch=s.epoch;value.attempt=advanced?8:0;
        value.time=s.time;value.velocity_time=s.velocity_time;value.kick_dt=s.reaction_kick_dt;
        value.has_completed_interval=advanced;
    };
    initialize(d.qeph);initialize(d.t3);initialize(d.qbat);
    initialize(d.type25);initialize(d.type13);initialize(d.solids);
    if(joint) {
        scope.type45_source_instance_id=91;scope.type45_joint_count=38;
        initialize(d.type45);d.type45.source_instance_id=91;d.type45.joint_count=38;
        d.type45.automatic_stiffness_initialized=advanced;
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
} // namespace crash::output::physical_frames::test
