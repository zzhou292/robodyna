#include "../RunArchive.h"
#include "../Replay.h"
#include "case/vehicle_dynamics/output/VehicleAcceptedFrames.h"
#include "case/vehicle_wall/LoadedWall.h"
#include "case/vehicle_startup/joints/VehicleJointModel.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
#include "output/full_shell/tests/TestSupport.h"
#include <cstdlib>
#include <sstream>
namespace crash::output::physical_run::test {
namespace {
namespace wall_case=cases::vehicle_wall;
struct Sources {
    Sources():attachments(cases::vehicle_startup::physical_attachments::test::Actual()),
        execution(cases::vehicle_runtime::Execution::Prepare(attachments.physical())),
        mapping(physical_frames::Mapping::Prepare(execution)) {
        const auto* path=std::getenv("ROBO_VEHICLE_WALL");Require(path && *path,"Original wall required");
        bytes=case_data::ReadPinnedWallManifest(path);
        std::istringstream input(bytes);
        Require(wall.Load(input).status==case_data::WallStatus::Ok,"Original wall fixture failed");
    }
    const cases::vehicle_runtime::Attachments& attachments;
    cases::vehicle_runtime::Execution execution;
    physical_frames::Mapping mapping;
    std::string bytes;
    case_data::CanonicalWall wall;
    wall_case::VehicleWallSetup Setup(double gap) {
        auto settings=wall_case::LoadedWallSettings();
        settings.requested_duration_s=.005;settings.leading_gap_m=gap;
        return wall_case::VehicleWallSetup::Prepare(execution,attachments,wall,bytes,settings);
    }
};
records::Identity Identity() {
    records::Identity id;id.run=0x57414c4c5345414cULL;id.topology=0x5941524953ULL;return id;
}
}
TEST(PhysicalRunWallOriginal, LoadedRowsRequireExactWallBackingEvenWithReusedBindingId) {
    Sources source;
    const auto a=source.Setup(1e-6),b=source.Setup(2e-6),equal=source.Setup(1e-6);
    const auto copy=a;
    EXPECT_TRUE(a.SharesStorage(copy));EXPECT_TRUE(a.identity().Matches(copy.identity()));
    EXPECT_FALSE(a.SharesStorage(equal));EXPECT_FALSE(a.identity().Matches(equal.identity()));
    EXPECT_FALSE(a.identity().Matches(b.identity()));
    ASSERT_EQ(a.settings().wall_binding_id,b.settings().wall_binding_id);
    const auto declarations=modelio::type45::VehicleType45Source::Prepare(source.attachments.physical().source_domain(),
        modelio::type45::Policy::OriginalDirectSdiType45V1);
    const auto joints=cases::vehicle_runtime::JointModel::Prepare(source.attachments.physical(),declarations);
    auto run=wall_case::LoadedWall::Prepare(a,wall_case::LoadedWallConfig(),{},&joints);
    cases::vehicle_dynamics::capture::VehicleAcceptedFrames capture(source.mapping,run,Identity());
    capture.Capture(run);const auto& producer=capture.frames();
    const auto request=MakeWallRequest(producer.context(),16667,.005,2);
    records::test::Directory good_dir,bad_dir;
    auto good=RunArchive::PrepareWithWall(good_dir.path,copy,source.mapping,producer.context(),request,{true,true});
    auto bad=RunArchive::PrepareWithWall(bad_dir.path,b,source.mapping,producer.context(),request,{true,true});
    good.Sample(*producer.frame(),*producer.activity());bad.Sample(*producer.frame(),*producer.activity());
    EXPECT_GT(good.forecast().shared_wall_setup_upper_bound,0u);
    run.PrepareStep();EXPECT_THROW(CaptureAcceptedInterval(run,producer,{true,true}),std::exception);run.DiscardStep();
    run.PrepareStep();run.CommitStep();
    const auto row=CaptureAcceptedInterval(run,producer,{true,true});
    EXPECT_THROW(bad.Append(row),std::exception);EXPECT_EQ(bad.accepted_intervals(),0u);EXPECT_FALSE(bad.failed());
    good.Append(row);EXPECT_EQ(good.accepted_intervals(),1u);
    capture.Capture(run);good.Sample(*producer.frame(),*producer.activity());
    const auto manifest=good.FinishPrefix("one actual loaded accepted interval; wall identity gate only");
    const auto replay=Replay::Open(good_dir.path,manifest,source.mapping.source_mapping().source().data().inputs,
        source.mapping.source_mapping().digest());
    EXPECT_EQ(replay.index().accepted_intervals,1u);ASSERT_TRUE(replay.wall());
    EXPECT_EQ(replay.wall()->wall_binding_id,a.settings().wall_binding_id);
    EXPECT_NO_THROW(bad.FinishPrefix("foreign wall row rejected; initial-only prefix retained"));
}
TEST(PhysicalRunWallOriginal, FreeFlightRowCannotCreateAContactArchiveClaim) {
    Sources source;const auto setup=source.Setup(1e-6);
    auto config=wall_case::LoadedWallConfig();
    auto run=cases::vehicle_dynamics::VehiclePhysicalDynamics::Prepare(source.execution,source.attachments,config);
    cases::vehicle_dynamics::capture::VehicleAcceptedFrames capture(source.mapping,run,Identity());
    capture.Capture(run);const auto& producer=capture.frames();
    records::test::Directory directory;
    const auto request=MakeWallRequest(producer.context(),16667,.005,2);
    auto archive=RunArchive::PrepareWithWall(directory.path,setup,source.mapping,producer.context(),request,{false,true});
    archive.Sample(*producer.frame(),*producer.activity());
    run.PrepareStep();run.CommitStep();
    const auto row=CaptureAcceptedInterval(run,producer,{false,true});
    EXPECT_THROW(archive.Append(row),std::exception);
    EXPECT_EQ(archive.accepted_intervals(),0u);EXPECT_FALSE(archive.failed());
    EXPECT_NO_THROW(archive.FinishPrefix("free-flight interval rejected; initial-only wall evidence"));
}
} // namespace crash::output::physical_run::test
