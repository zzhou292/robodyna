#include "../RunArchive.h"
#include "../Replay.h"
#include "case/vehicle_dynamics/output/VehicleAcceptedFrames.h"
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
#include "output/full_shell/tests/TestSupport.h"
#include "case/vehicle_startup/joints/VehicleJointModel.h"
#include "case/vehicle_runtime/CaptureAccess.h"
namespace crash::output::physical_run::test {
namespace {
namespace dynamics=cases::vehicle_dynamics;
const cases::vehicle_runtime::Attachments& Source() {return cases::vehicle_startup::physical_attachments::test::Actual();}
const cases::vehicle_runtime::Execution& Shells() {
    static const auto source=cases::vehicle_runtime::Execution::Prepare(Source().physical());return source;
}
const physical_frames::Mapping& Map() {static const auto mapping=physical_frames::Mapping::Prepare(Shells());return mapping;}
records::Identity Identity() {records::Identity id;id.run=0x505245464958ULL;id.topology=0x5941524953ULL;return id;}
void CheckReplay(const std::filesystem::path& root,const records::RecordFile& file,
    const records::FrameRecord& frame,const records::activity::ActivityRecord& activity) {
    const auto replay=Replay::Open(root,file,Map().source_mapping().source().data().inputs,Map().source_mapping().digest());
    EXPECT_EQ(replay.context().parents().size(),349645u);EXPECT_EQ(replay.context().nodes(),359785u);
    EXPECT_EQ(replay.context().points(),956346u);EXPECT_FALSE(replay.index().horizon_complete);
    EXPECT_EQ(replay.index().accepted_intervals,frame.stamp.epoch);
    const auto saved=replay.ReadSample(replay.index().frames.size()-1);
    EXPECT_TRUE(records::SameStamp(saved.frame.stamp,frame.stamp));
    ASSERT_EQ(saved.frame.position_xyz.size(),frame.position_xyz.size());
    for(std::size_t i=0;i<frame.position_xyz.size();++i)ASSERT_EQ(Bits(saved.frame.position_xyz[i]),Bits(frame.position_xyz[i]));
    EXPECT_EQ(saved.frame.plastic_points,frame.plastic_points);EXPECT_EQ(saved.activity.words(),activity.words());
    ::testing::Test::RecordProperty("reader_peak_host_bytes",std::to_string(replay.peak_host_bytes()));
}
}
TEST(PhysicalRunOriginal, InitialOnlyPrefixHasCompleteSourceAndNoAcceptedIntervalClaim) {
    auto run=physical_frames::Run::Prepare(Shells(),Source());
    physical_frames::PhysicalAcceptedFrames capture(Map(),run,Identity());capture.Capture(run);
    records::test::Directory directory;
    const auto request=MakeRequest(capture.context(),1,capture.context().fixed_dt(),2);
    auto archive=RunArchive::Prepare(directory.path,Map().source_mapping(),capture.context(),request,{});
    archive.Sample(*capture.frame(),*capture.activity());
    EXPECT_THROW(archive.Finish(),std::exception);
    const auto manifest=archive.FinishPrefix("initial snapshot only; no interval requested by this gate");
    CheckReplay(directory.path,manifest,*capture.frame(),*capture.activity());
    EXPECT_EQ(archive.accepted_intervals(),0u);
    RecordProperty("whole_run_forecast_bytes",std::to_string(archive.forecast().archive.archive.forecast_bytes));
}
TEST(PhysicalRunOriginal, OneActualAcceptedIntervalFactoryDiscardAndFailedPrefixRoundTrip) {
    dynamics::Config config;config.structural={tl::fea::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8};
    auto run=dynamics::VehiclePhysicalDynamics::Prepare(Shells(),Source(),config);
    dynamics::capture::VehicleAcceptedFrames capture(Map(),run,Identity());capture.Capture(run);
    const auto& producer=capture.frames();
    records::test::Directory directory;
    const auto request=MakeRequest(producer.context(),4,3.5*producer.context().fixed_dt(),2);
    auto archive=RunArchive::Prepare(directory.path,Map().source_mapping(),producer.context(),request,{false,true});
    archive.Sample(*producer.frame(),*producer.activity());
    EXPECT_THROW(CaptureAcceptedInterval(run,producer,{false,true}),std::exception);
    run.PrepareStep();EXPECT_THROW(CaptureAcceptedInterval(run,producer,{false,true}),std::exception);run.DiscardStep();
    run.PrepareStep();run.CommitStep();
    const auto interval=CaptureAcceptedInterval(run,producer,{false,true});archive.Append(interval);
    EXPECT_THROW(CaptureAcceptedInterval(run,producer,{true,true}),std::exception);
    EXPECT_THROW(archive.Append(interval),std::exception);EXPECT_EQ(archive.accepted_intervals(),1u);
    capture.Capture(run);archive.Sample(*producer.frame(),*producer.activity());
    const auto manifest=archive.FinishPrefix("diagnostic stops at first accepted endpoint; no joint/contact closure claim");
    CheckReplay(directory.path,manifest,*producer.frame(),*producer.activity());
    EXPECT_EQ(run.accepted().epoch,1u);
}
TEST(PhysicalRunOriginal, OriginalJointInitialPrefixAuthenticatesSeventhSourceAndVirginPhase) {
    const auto source=modelio::type45::VehicleType45Source::Prepare(Source().physical().source_domain(),
        modelio::type45::Policy::OriginalDirectSdiType45V1);
    const auto joints=cases::vehicle_runtime::JointModel::Prepare(Source().physical(),source);
    auto run=physical_frames::Run::Prepare(Shells(),Source(),{},&joints);
    physical_frames::PhysicalAcceptedFrames capture(Map(),run,Identity());capture.Capture(run);
    records::test::Directory directory;
    const auto request=MakeRequest(capture.context(),1,capture.context().fixed_dt(),2);
    auto archive=RunArchive::Prepare(directory.path,Map().source_mapping(),capture.context(),request,{true,false});
    archive.Sample(*capture.frame(),*capture.activity());
    const auto manifest=archive.FinishPrefix("initial seventh-participant snapshot; automatic stiffness remains uninitialized");
    CheckReplay(directory.path,manifest,*capture.frame(),*capture.activity());
    const auto scope=cases::vehicle_runtime::detail::CaptureAccess::Scope(run);
    EXPECT_TRUE(scope.diagnostics.has_type45);EXPECT_EQ(scope.type45_joint_count,38u);
    EXPECT_EQ(scope.diagnostics.type45.source_instance_id,capture.context().identity().source_instance);
    EXPECT_FALSE(scope.diagnostics.type45.automatic_stiffness_initialized);
    EXPECT_EQ(run.accepted().epoch,0u);
}
} // namespace crash::output::physical_run::test
