#include "../Scene.h"
#include "viewer/physical_run/Options.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChSystemNSC.h"
#include <gtest/gtest.h>
#include <cstdlib>
namespace crash::visual::physical_run::test {
TEST(RecoveredSceneArchive, SavedGeometryStampsAndHonestCaptureMetadata) {
    const auto* path=std::getenv("ROBO_DYNA_RECOVERED_REPLAY_INPUT");
    ASSERT_NE(path,nullptr);
    viewer::physical_run::Options options;options.input=path;options.recovered=true;
    if(const auto* hash=std::getenv("ROBO_DYNA_RECOVERED_REPLAY_SHA256"))options.expected_receipt_sha256=hash;
    const auto input=viewer::physical_run::ReadInput(options);
    const auto source=viewer::physical_run::OpenSamples(input);
    ASSERT_NE(source.recovered(),nullptr);EXPECT_EQ(source.normal(),nullptr);
    ASSERT_FALSE(source.frames().empty());
    Scene scene;
    ASSERT_EQ(scene.Initialize(source).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(scene.replay(),nullptr);ASSERT_NE(scene.samples(),nullptr);
    EXPECT_EQ(scene.system().GetBodies().size(),(source.wall()||source.environment())?2u:1u);
    for (std::size_t i=0;i<source.frames().size();++i) {
        ASSERT_EQ(scene.Publish(i).status,ReplaySceneStatus::Ok);
        const auto sample=source.ReadSample(i);
        const auto mesh=scene.geometry()->mesh();
        ASSERT_EQ(mesh->GetCoordsVertices().size()*3,sample.frame.position_xyz.size());
        for (std::size_t n=0;n<mesh->GetCoordsVertices().size();++n)
            for (unsigned a=0;a<3;++a)
                EXPECT_EQ(output::Bits(mesh->GetCoordsVertices()[n][a]),output::Bits(sample.frame.position_xyz[3*n+a]));
        EXPECT_EQ(scene.stamp()->epoch,source.frames()[i].stamp.epoch);
        EXPECT_EQ(scene.system().GetChTime(),source.frames()[i].stamp.time);
    }
    const auto before=*scene.stamp();
    EXPECT_EQ(scene.Publish(source.frames().size()).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.stamp()->epoch,before.epoch);EXPECT_EQ(scene.stamp()->time,before.time);
    const auto metadata=viewer::physical_run::SourceMetadata(input,source);
    EXPECT_STREQ(metadata["schema"].GetString(),"robo_dyna.recovered_sample_capture.v1");
    EXPECT_FALSE(metadata["interval_ledger_available"].GetBool());
    EXPECT_FALSE(metadata["continuous_accepted_history_available"].GetBool());
    EXPECT_STREQ(metadata["input_horizon_completion"].GetString(),"unknown");
    EXPECT_FALSE(metadata.HasMember("input_archive_manifest"));
    EXPECT_FALSE(metadata.HasMember("input_horizon_complete"));
    EXPECT_FALSE(metadata.HasMember("final_epoch"));
    EXPECT_EQ(metadata["final_saved_epoch"].GetUint64(),source.frames().back().stamp.epoch);
    auto foreign=input;foreign.receipt.sha256=output::Sha256("foreign");
    EXPECT_THROW(viewer::physical_run::SourceMetadata(foreign,source),std::exception);
}
} // namespace crash::visual::physical_run::test
