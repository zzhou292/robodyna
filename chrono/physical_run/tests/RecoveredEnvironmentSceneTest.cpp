#include "../Scene.h"
#include "viewer/physical_run/Options.h"
#include "chrono/ReplayPartColors.h"
#include "chrono/ReplayVisuals.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"
#include <gtest/gtest.h>
#include <cstdlib>
namespace crash::visual::physical_run::test {
TEST(RecoveredEnvironmentScene, OriginalWallAndPartColorsPreserveSampleOnlyClaims) {
    const auto* path=std::getenv("ROBO_DYNA_RECOVERED_REPLAY_INPUT");
    const auto* hash=std::getenv("ROBO_DYNA_RECOVERED_REPLAY_SHA256");
    ASSERT_NE(path,nullptr);ASSERT_NE(hash,nullptr);
    viewer::physical_run::Options options;options.input=path;options.recovered=true;
    options.expected_receipt_sha256=hash;
    const auto input=viewer::physical_run::ReadInput(options);
    const auto source=viewer::physical_run::OpenSamples(input);
    ASSERT_NE(source.recovered(),nullptr);ASSERT_EQ(source.normal(),nullptr);
    ASSERT_NE(source.environment(),nullptr);ASSERT_EQ(source.wall(),nullptr);
    EXPECT_TRUE(source.configuration().environment);EXPECT_TRUE(source.configuration().profile.native_group);
    Scene scene;SceneOptions scene_options;scene_options.colors=ReplayColorMode::PartId;scene_options.part_palette_seed=2;
    ASSERT_EQ(scene.Initialize(source,scene_options).status,ReplaySceneStatus::Ok);
    ASSERT_EQ(scene.system().GetBodies().size(),2u);
    const auto model=scene.system().GetBodies().back()->GetVisualModel();ASSERT_TRUE(model);
    const auto wall=std::dynamic_pointer_cast<chrono::ChVisualShapeTriangleMesh>(model->GetShape(0));
    ASSERT_TRUE(wall);ASSERT_TRUE(wall->GetMesh());ASSERT_TRUE(source.wall_mesh());
    ASSERT_EQ(wall->GetMesh()->GetNumVertices(),4u);ASSERT_EQ(wall->GetMesh()->GetNumTriangles(),2u);
    const auto wall_color=ReplayPartColor(source.environment()->part_id,2);
    ASSERT_EQ(wall->GetMesh()->GetCoordsColors().size(),4u);
    for(unsigned node=0;node<4;++node) {
        for(unsigned axis=0;axis<3;++axis)
            EXPECT_EQ(output::Bits(wall->GetMesh()->GetCoordsVertices()[node][axis]),
                output::Bits(source.wall_mesh()->GetCoordsVertices()[node][axis]));
        const auto& color=wall->GetMesh()->GetCoordsColors()[node];
        EXPECT_EQ(color.R,wall_color.R);EXPECT_EQ(color.G,wall_color.G);EXPECT_EQ(color.B,wall_color.B);
    }
    for(std::size_t i=0;i<source.frames().size();++i) {
        ASSERT_EQ(scene.Publish(i).status,ReplaySceneStatus::Ok);
        const auto mesh=scene.geometry()->mesh();const auto& parts=*scene.geometry()->triangle_source_parts();
        ASSERT_EQ(parts.size(),mesh->GetIndicesColors().size());
        for(std::size_t triangle=0;triangle<parts.size();++triangle) {
            const auto color=mesh->GetCoordsColors()[mesh->GetIndicesColors()[triangle][0]];
            const auto expected=ReplayPartColor(parts[triangle],2);
            ASSERT_EQ(color.R,expected.R);ASSERT_EQ(color.G,expected.G);ASSERT_EQ(color.B,expected.B);
        }
        EXPECT_EQ(scene.stamp()->epoch,source.frames()[i].stamp.epoch);
        EXPECT_EQ(scene.system().GetChTime(),source.frames()[i].stamp.time);
    }
    ReplayClipping clip;ASSERT_TRUE(MakeReplayClipping(*scene.camera(),*scene.bounds(),clip));
    EXPECT_GT(clip.near_m,0);EXPECT_GT(clip.far_m,clip.maximum_depth_m);
    const auto metadata=viewer::physical_run::SourceMetadata(input,source);
    EXPECT_FALSE(metadata["interval_ledger_available"].GetBool());
    EXPECT_FALSE(metadata["continuous_accepted_history_available"].GetBool());
    EXPECT_STREQ(metadata["input_horizon_completion"].GetString(),"unknown");
    EXPECT_FALSE(metadata.HasMember("input_archive_manifest"));EXPECT_FALSE(metadata.HasMember("final_epoch"));
    EXPECT_EQ(metadata["final_saved_epoch"].GetUint64(),source.frames().back().stamp.epoch);
    EXPECT_DOUBLE_EQ(metadata["final_saved_time_s"].GetDouble(),source.frames().back().stamp.time);
}
} // namespace crash::visual::physical_run::test
