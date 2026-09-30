#include "../NativeCaptureChecks.h"
#include "FieldsFixture.h"
namespace crash::output::physical_frames::test {
namespace {
detail::NativeCaptureScope NativeScope(unsigned epoch) {
    detail::NativeCaptureScope c;auto& s=c.stamp;
    s.owner_id=31;s.node_count=5;s.fixed_dt=.125;s.temporal_scheme=f::NodalTemporalScheme::StaggeredHalfKickStart;
    const auto base=s;
    if(epoch){s.epoch=1;s.time=.125;s.velocity_time=.0625;s.reactions_valid=true;s.reaction_kick_dt=.0625;}
    auto& d=c.diagnostics;d.valid=true;d.has_qeph=d.has_t3=true;d.base_stamp=base;
    const auto init=[&](auto& p){p.valid=true;p.owner_id=31;p.configuration_id=32;p.qualification_id=33;
        p.phase=decltype(p.phase)::Accepted;p.epoch=epoch;p.time=s.time;p.velocity_time=s.velocity_time;
        p.has_completed_interval=bool(epoch);if(epoch){p.attempt=8;p.kick_dt=.0625;}};
    init(d.qeph);init(d.t3);
    c.source={41,42,43,5,5,2,4,true};
    c.contact.stamp=s;c.contact.available=true;c.contact.generation=epoch;c.contact.force_base_stamp=base;
    c.contact.force_phase_available=bool(epoch);c.contact.selectors.has_reference=bool(epoch);
    c.contact.selectors.reference_generation=epoch;return c;
}
}
TEST(NativeCapture, ActualPhaseShapeRejectsMixedEpochSourceAndUnsupportedParticipants) {
    const auto initial=NativeScope(0),accepted=NativeScope(1);
    EXPECT_EQ(detail::NativePhase(initial).epoch,0u);EXPECT_EQ(detail::NativePhase(accepted).epoch,1u);
    EXPECT_THROW(detail::NativeContactObservation(initial),std::exception);
    EXPECT_EQ(detail::NativeContactObservation(accepted).publication_generation,1u);
    auto bad=accepted;++bad.contact.force_base_stamp.owner_id;EXPECT_THROW(detail::NativePhase(bad),std::exception);
    bad=accepted;++bad.diagnostics.t3.attempt;EXPECT_THROW(detail::NativePhase(bad),std::exception);
    bad=accepted;bad.diagnostics.has_solids=true;EXPECT_THROW(detail::NativePhase(bad),std::exception);
    bad=accepted;bad.contact.generation=2;EXPECT_THROW(detail::NativePhase(bad),std::exception);
    bad=accepted;bad.contact.selectors.reference_generation=2;EXPECT_THROW(detail::NativePhase(bad),std::exception);
    bad=accepted;++bad.source.source_generation;EXPECT_THROW(detail::CheckSameNativeScope(accepted,bad),std::exception);
    bad=accepted;bad.contact.selectors.history^=1;EXPECT_THROW(detail::CheckSameNativeScope(accepted,bad),std::exception);
    bad=initial;bad.contact.selectors.reference_generation=1;EXPECT_THROW(detail::NativePhase(bad),std::exception);
    bad=initial;++bad.diagnostics.base_stamp.owner_id;EXPECT_THROW(detail::NativePhase(bad),std::exception);
}
TEST(NativeCapture, LateNativePhaseFailureLeavesPriorFrameAndActivityThenAllowsRetry) {
    Fixture f;f.Stage();f.buffers.Finish(f.context,{});
    const auto selected=f.buffers.selected;const auto prior=f.buffers.frames[selected];
    const auto words=f.buffers.activity[selected]->words();
    const auto before=NativeScope(1);auto after=before;++after.contact.generation;
    f.Stage();EXPECT_THROW(detail::CheckSameNativeScope(before,after),std::exception);
    EXPECT_EQ(f.buffers.selected,selected);EXPECT_EQ(f.buffers.frames[selected].position_xyz,prior.position_xyz);
    EXPECT_EQ(f.buffers.activity[selected]->words(),words);
    EXPECT_NO_THROW(detail::CheckSameNativeScope(before,before));
    f.buffers.Finish(f.context,detail::NativePhase(before));
    EXPECT_NE(f.buffers.selected,selected);EXPECT_EQ(f.buffers.frames[f.buffers.selected].stamp.epoch,1u);
}
}
