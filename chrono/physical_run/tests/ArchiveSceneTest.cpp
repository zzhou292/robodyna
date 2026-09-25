#include "../Scene.h"
#include "output/physical_run/ViewerInput.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChSystemNSC.h"
#include <gtest/gtest.h>
#include <cstdlib>
namespace crash::visual::physical_run::test {
TEST(PhysicalSceneArchive, ExactOriginalArchiveWallActivityAndFailedSeekPreserveDisplay) {
    namespace run=output::physical_run;
    const auto* path=std::getenv("ROBO_DYNA_PHYSICAL_REPLAY_INPUT");
    ASSERT_NE(path,nullptr);
    const std::filesystem::path file=path;
    const auto bytes=output::ReadBounded(file,run::ViewerInputByteCap);
    const auto in=run::ReadViewerInput(file.parent_path(),{file.filename().string(),output::Sha256(bytes),bytes.size()});
    const auto replay=run::Replay::Open(run::ViewerArchivePath(file.parent_path(),in),in.manifest,in.source,in.mapping_sha256);
    for(auto colors:{ReplayColorMode::PartId,ReplayColorMode::PlasticStrain}) {
        Scene scene;SceneOptions options;options.colors=colors;
        ASSERT_EQ(scene.Initialize(replay,options).status,ReplaySceneStatus::Ok);
        const auto mesh=scene.geometry()->mesh();
        ASSERT_NE(scene.bounds(),nullptr);const auto bounds=*scene.bounds();
        EXPECT_FALSE(scene.moving_shape()->IsFixedConnectivity());
        EXPECT_EQ(scene.system().GetBodies().size(),replay.wall()?2u:1u);
        for(std::size_t i=0;i<replay.index().frames.size();++i) {
            ASSERT_EQ(scene.Publish(i).status,ReplaySceneStatus::Ok);
            const auto sample=replay.ReadSample(i);
            ASSERT_EQ(mesh->GetCoordsVertices().size()*3,sample.frame.position_xyz.size());
            for(std::size_t n=0;n<mesh->GetCoordsVertices().size();++n)
                for(unsigned a=0;a<3;++a) {
                    ASSERT_EQ(output::Bits(mesh->GetCoordsVertices()[n][a]),output::Bits(sample.frame.position_xyz[3*n+a]));
                    EXPECT_GE(sample.frame.position_xyz[3*n+a],bounds.low[a]);
                    EXPECT_LE(sample.frame.position_xyz[3*n+a],bounds.high[a]);
                }
            EXPECT_EQ(scene.stamp()->epoch,sample.frame.stamp.epoch);
            EXPECT_EQ(scene.system().GetChTime(),sample.frame.stamp.time);
            EXPECT_EQ(scene.geometry()->mesh().get(),mesh.get());
        }
        const auto stamp=*scene.stamp();
        const auto faces=mesh->GetNumTriangles();
        EXPECT_EQ(scene.Publish(replay.index().frames.size()).status,ReplaySceneStatus::InvalidFrame);
        EXPECT_EQ(scene.stamp()->index,stamp.index);EXPECT_EQ(mesh->GetNumTriangles(),faces);
        EXPECT_EQ(scene.Publish(0).status,ReplaySceneStatus::Ok);
    }
}
} // namespace crash::visual::physical_run::test
