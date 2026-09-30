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
    ASSERT_TRUE(replay.wall_composition());
    const auto& composition=*replay.wall_composition();
    const auto& model=setup.execution().model();
    EXPECT_EQ(composition.profile,CompositionProfile::RetainedV1);
    EXPECT_EQ(composition.physical_nodes,model.source_domain().domain().node_count());
    EXPECT_EQ(composition.solid_parents,(std::array<std::uint64_t,5>{908,1309,195,0,0}));
    EXPECT_EQ(composition.point_mass_records,model.coefficients().scope().element_mass_records);
    EXPECT_EQ(Bits(composition.initial_mass_kg),Bits(model.coefficients().totals().mass));
    EXPECT_EQ(Bits(composition.point_mass_kg),Bits(model.coefficients().totals().element_mass));
    EXPECT_EQ(composition.rigid_groups,model.rigid_assembly().groups().size());
    EXPECT_EQ(composition.rigid_members,model.rigid_assembly().members().size());
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
    // Parse a valid composition, then reject a later wall-plane discrepancy.
    // The optional caller destination and prior immutable replay stay intact.
    auto altered=*replay.wall();
    const auto setup_bytes=ReadFile(directory.path,altered.files[6],WallFileCap);
    auto bad_setup=array_json::Parse(setup_bytes,WallFileCap);
    bad_setup["represented_wall_x_m"].SetDouble(setup.placement().represented_wall_x_m+1);
    const auto temporary=WriteDocument(directory.path,"bad-wall-setup.json",bad_setup,WallFileCap);
    const auto bad_bytes=ReadFile(directory.path,temporary,WallFileCap);
    records::test::Overwrite(directory.path/altered.files[6].file,bad_bytes);
    altered.files[6].bytes=bad_bytes.size();altered.files[6].sha256=Sha256(bad_bytes);
    std::optional<WallComposition> staged=composition;
    staged->initial_mass_kg=123;
    EXPECT_THROW(ReadWallArtifacts(directory.path,altered,mapping.source_mapping().source().data(),
        capture.context(),&staged),std::exception);
    ASSERT_TRUE(staged);EXPECT_EQ(staged->initial_mass_kg,123);
    EXPECT_EQ(Bits(replay.wall_composition()->initial_mass_kg),Bits(model.coefficients().totals().mass));
    records::test::Overwrite(directory.path/altered.files[6].file,setup_bytes);
    std::filesystem::remove(directory.path/temporary.file);
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
