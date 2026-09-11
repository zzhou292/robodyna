#include "../RunArchive.h"
#include "../Replay.h"
#include "output/physical_frames/PhysicalAcceptedFrames.h"
#include "output/full_shell/tests/TestSupport.h"
#include "case/vehicle_wall/VehicleWallSetup.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <cstdlib>
#include <sstream>
namespace crash::output::physical_run::test {
TEST(PhysicalRunWallOriginal, CompleteInitialPrefixRetainsSelectedMeshAndOriginalProvenance) {
    const auto& source=cases::vehicle_startup::physical_attachments::test::Actual();
    const auto execution=cases::vehicle_runtime::Execution::Prepare(source.physical());
    const auto mapping=physical_frames::Mapping::Prepare(execution);
    const auto path=std::getenv("ROBO_VEHICLE_WALL");Require(path && *path,"Explicit original canonical wall required");
    const auto original=case_data::ReadPinnedWallManifest(path);
    case_data::CanonicalWall wall;std::istringstream input(original);
    Require(wall.Load(input).status==case_data::WallStatus::Ok,"Original wall fixture failed");
    const auto setup=cases::vehicle_wall::VehicleWallSetup::Prepare(execution,source,wall,original);
    cases::vehicle_runtime::Config config;config.reserved_step_s=3e-7;
    auto run=physical_frames::Run::Prepare(execution,source,config);
    records::Identity identity;identity.run=0x57414c4c52554eULL;identity.topology=0x5941524953ULL;
    physical_frames::PhysicalAcceptedFrames capture(mapping,run,identity);capture.Capture(run);
    const auto request=MakeWallRequest(capture.context(),66667,setup.settings().requested_duration_s,3);
    const auto forecast=RunArchive::PreflightWithWall(setup,mapping,capture.context(),request,{});
    Limits limits;limits.host_bytes=forecast.peak_host_bytes-1;
    records::test::Directory directory;
    EXPECT_THROW(RunArchive::PrepareWithWall(directory.path,setup,mapping,capture.context(),request,{},limits),std::exception);
    EXPECT_TRUE(std::filesystem::is_empty(directory.path));
    ++limits.host_bytes;
    auto archive=RunArchive::PrepareWithWall(directory.path,setup,mapping,capture.context(),request,{},limits);
    archive.Sample(*capture.frame(),*capture.activity());
    const auto manifest=archive.FinishPrefix("initial wall archive only; no contact interval claim");
    const auto replay=Replay::Open(directory.path,manifest,mapping.source_mapping().source().data().inputs,mapping.source_mapping().digest());
    ASSERT_TRUE(replay.wall());ASSERT_TRUE(replay.wall_mesh());
    EXPECT_EQ(replay.wall()->files[0].sha256,case_data::kCanonicalWallManifestSha256);
    EXPECT_EQ(replay.wall()->wall_binding_id,setup.settings().wall_binding_id);
    EXPECT_EQ(replay.wall_mesh()->GetNumVertices(),4u);EXPECT_EQ(replay.wall_mesh()->GetNumTriangles(),2u);
    const auto view=setup.selected_wall_view();
    for(std::size_t n=0;n<view.vertex_count;++n) {
        const auto& x=replay.wall_mesh()->GetCoordsVertices()[n];
        EXPECT_EQ(Bits(x.x()),Bits(view.vertices[n].position.x));
        EXPECT_EQ(Bits(x.y()),Bits(view.vertices[n].position.y));
        EXPECT_EQ(Bits(x.z()),Bits(view.vertices[n].position.z));
    }
    EXPECT_EQ(replay.index().accepted_intervals,0u);
    const auto selected_bytes=ReadFile(directory.path,replay.wall()->files[4],WallFileCap);
    records::test::Overwrite(directory.path/replay.wall()->files[4].file,selected_bytes+"corruption");
    EXPECT_THROW(Replay::Open(directory.path,manifest,mapping.source_mapping().source().data().inputs,
        mapping.source_mapping().digest()),std::exception);
    EXPECT_EQ(replay.wall_mesh()->GetNumVertices(),4u); // Prior immutable publication remains intact.
    records::test::Overwrite(directory.path/replay.wall()->files[4].file,selected_bytes);
    const auto retry=Replay::Open(directory.path,manifest,mapping.source_mapping().source().data().inputs,mapping.source_mapping().digest());
    EXPECT_EQ(retry.wall_mesh()->GetNumTriangles(),2u);
    RecordProperty("whole_run_forecast_bytes",std::to_string(forecast.archive.archive.forecast_bytes));
    RecordProperty("writer_peak_host_bytes",std::to_string(forecast.peak_host_bytes));
    RecordProperty("reader_peak_host_bytes",std::to_string(replay.peak_host_bytes()));
}
} // namespace crash::output::physical_run::test
