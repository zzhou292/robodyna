#include "FieldsFixture.h"
#include "../ArchiveState.h"
#include <gtest/gtest.h>
#include <limits>
namespace crash::output::physical_frames::test {
TEST(PhysicalCaptureValues, OriginalOrderRetainsZeroOneThreeFourPointsAndSeparateActivity) {
    Fixture f;
    EXPECT_FALSE(f.buffers.available);
    f.Stage();
    f.buffers.Finish(f.context,{});
    const auto& frame=f.buffers.frames[f.buffers.selected];
    EXPECT_EQ(frame.position_xyz,(std::vector<double>{3,4,5,-0.,1,2,6,7,8}));
    EXPECT_EQ(Bits(frame.position_xyz[3]),Bits(-0.));
    EXPECT_EQ(frame.plastic_points,(std::vector<double>{.02,.04,.06,.08,.875,.1,.2,.1*3}));
    EXPECT_EQ(f.context.point_offsets(),(std::vector<std::size_t>{0,0,4,5,5,8}));
    const auto& active=*f.buffers.activity[f.buffers.selected];
    EXPECT_EQ(active.active_count(),4u);
    EXPECT_FALSE(active.active(4));
    auto maxima=records::ParentPlasticMaxima(f.context,frame);
    EXPECT_FALSE(maxima[0]);
    EXPECT_FALSE(maxima[3]);
    EXPECT_EQ(maxima[1],.08);
}
TEST(PhysicalCaptureValues, LatePointActivityAndPhaseFailuresKeepPublishedPairThenRetry) {
    Fixture f;
    f.Stage();f.buffers.Finish(f.context,{});
    const auto selected=f.buffers.selected;
    const auto old=f.buffers.frames[selected];
    auto words=f.buffers.activity[selected]->words();
    f.b[0].history.point[3].material.plastic_strain=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(f.Stage(),std::exception);
    EXPECT_EQ(f.buffers.selected,selected);
    EXPECT_EQ(f.buffers.frames[selected].plastic_points,old.plastic_points);
    f.b[0].history.point[3].material.plastic_strain=.3;
    f.Stage();f.buffers.flags.back()=2;
    EXPECT_THROW(f.buffers.Finish(f.context,{}),std::exception);
    EXPECT_EQ(f.buffers.selected,selected);
    EXPECT_EQ(f.buffers.activity[selected]->words(),words);
    f.Stage();
    EXPECT_THROW(f.buffers.Finish(f.context,{1,0,1,.125,0,.125,.0625}),std::exception);
    EXPECT_EQ(f.buffers.selected,selected);
    f.buffers.Finish(f.context,{});
    EXPECT_NE(f.buffers.selected,selected);
    EXPECT_EQ(f.buffers.frames[f.buffers.selected].plastic_points[3],.3);
}
TEST(PhysicalCaptureValues, CompleteBufferCapAndCadenceAreIndependentOfPhysicalStep) {
    Fixture f;
    const auto forecast=detail::PlanBuffers(f.context,5,3,1,1,2048,{});
    Limits exact;exact.host_bytes=forecast.peak_bytes;
    EXPECT_EQ(detail::PlanBuffers(f.context,5,3,1,1,2048,exact).peak_bytes,forecast.peak_bytes);
    --exact.host_bytes;
    EXPECT_THROW(detail::PlanBuffers(f.context,5,3,1,1,2048,exact),std::exception);
    EXPECT_THROW(detail::PlanBuffers(f.context,SIZE_MAX,3,1,1,0,{}),std::exception);
    records::PlanRequest request;
    request.nodes=3;request.parents=5;request.plastic_points=8;
    request.frames=4;request.intervals=10;request.fixed_dt=.125;request.requested_duration=1.25;
    request.static_files={{"manifest.json",16384},{"frame-index.json",16384},{"configuration.json",16384}};
    const auto plan=records::activity::PlanWithActivity(f.context,request,"parent-activity.json");
    EXPECT_EQ(plan.archive.frame_epochs,(std::vector<std::uint64_t>{0,3,6,10}));
    request.total_byte_cap=plan.archive.forecast_bytes;
    EXPECT_EQ(records::activity::PlanWithActivity(f.context,request,"parent-activity.json").archive.forecast_bytes,
        plan.archive.forecast_bytes);
    --request.total_byte_cap;
    EXPECT_THROW(records::activity::PlanWithActivity(f.context,request,"parent-activity.json"),std::exception);
}
TEST(PhysicalCaptureValues, ActualCommonPhaseChecksEveryFamilyBeforeReadbackPublication) {
    detail::CaptureScope scope;
    auto& stamp=scope.stamp;
    stamp.owner_id=31;stamp.node_count=5;stamp.fixed_dt=.125;
    stamp.temporal_scheme=f::NodalTemporalScheme::StaggeredHalfKickStart;
    stamp.epoch=1;stamp.time=.125;stamp.velocity_time=.0625;
    stamp.reactions_valid=true;stamp.reaction_kick_dt=.0625;
    auto& common=scope.diagnostics;
    common.valid=true;common.has_qeph=common.has_t3=common.has_qbat=true;
    common.has_type25=common.has_type13=common.has_solids=true;
    const auto initialize=[&](auto& d) {
        d.valid=true;d.owner_id=31;d.configuration_id=32;d.qualification_id=33;
        d.phase=decltype(d.phase)::Accepted;d.epoch=1;d.attempt=8;
        d.time=.125;d.velocity_time=.0625;d.kick_dt=.0625;d.has_completed_interval=true;
    };
    initialize(common.qeph);initialize(common.t3);initialize(common.qbat);
    initialize(common.type25);initialize(common.type13);initialize(common.solids);
    EXPECT_EQ(detail::Phase(scope).attempt,8u);
    const auto accepted=scope;
    common.solids.kick_dt=.125;
    EXPECT_THROW(detail::Phase(scope),std::exception);
    EXPECT_THROW(detail::CheckSameEndpoint(accepted,scope),std::exception);
    scope=accepted;
    auto read=scope.diagnostics.t3;++read.attempt;
    EXPECT_THROW(detail::CheckReadback(scope,read),std::exception);
    EXPECT_NO_THROW(detail::CheckReadback(scope,scope.diagnostics.t3));
}
TEST(PhysicalCaptureValues, LateActivityDestinationRejectsBeforeEitherFrameArray) {
    records::test::Directory directory;
    Fixture f;f.Stage();f.buffers.Finish(f.context,{});
    const auto& frame=f.buffers.frames[f.buffers.selected];
    const auto& activity=*f.buffers.activity[f.buffers.selected];
    EXPECT_NO_THROW(detail::CheckPair(f.context,frame,activity));
    WriteBytes(directory.path/"initial.activity.json","existing");
    EXPECT_THROW(detail::FrameDestinations(directory.path,"initial"),std::exception);
    EXPECT_FALSE(std::filesystem::exists(directory.path/"initial.positions.bin"));
    EXPECT_FALSE(std::filesystem::exists(directory.path/"initial.plastic.bin"));
    std::filesystem::remove(directory.path/"initial.activity.json");
    EXPECT_NO_THROW(detail::FrameDestinations(directory.path,"initial"));
    const auto fr=records::WriteFrame(directory.path,"initial",f.context,
        {frame.stamp,frame.position_xyz.data(),frame.position_xyz.size(),frame.plastic_points.data(),frame.plastic_points.size()});
    const auto ar=records::activity::WriteActivity(directory.path,"initial",activity);
    const auto restored=records::ReadFrame(directory.path,f.context,fr,{});
    EXPECT_EQ(restored.plastic_points,frame.plastic_points);
    EXPECT_EQ(records::activity::ReadActivity(directory.path,f.context,ar,{}).words(),activity.words());
}
} // namespace crash::output::physical_frames::test
