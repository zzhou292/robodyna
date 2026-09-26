#include "../Scene.h"
#include "chrono/ReplayVisuals.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/physics/ChBody.h"
#include <algorithm>
#include <limits>
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
    for(auto seed:{UINT64_C(1),UINT64_C(2)}) for(auto colors:{ReplayColorMode::PartId,ReplayColorMode::PlasticStrain}) {
        if(seed==2 && colors==ReplayColorMode::PlasticStrain) continue;
        Scene scene;SceneOptions options;options.colors=colors;options.part_palette_seed=seed;
        ASSERT_EQ(scene.Initialize(replay,options).status,ReplaySceneStatus::Ok);
        EXPECT_EQ(scene.geometry()->part_palette_seed(),seed);
        const auto mesh=scene.geometry()->mesh();
        ASSERT_NE(scene.bounds(),nullptr);const auto bounds=*scene.bounds();
        EXPECT_FALSE(scene.moving_shape()->IsFixedConnectivity());
        EXPECT_EQ(scene.system().GetBodies().size(),(replay.wall()||replay.environment())?2u:1u);
        if(replay.environment()) {
            std::array<double,3> vehicle_low,vehicle_high;
            vehicle_low.fill(std::numeric_limits<double>::infinity());
            vehicle_high.fill(-std::numeric_limits<double>::infinity());
            for(std::size_t i=0;i<replay.index().frames.size();++i) {
                const auto sample=replay.ReadSample(i);
                for(std::size_t k=0;k<sample.frame.position_xyz.size();++k) {
                    vehicle_low[k%3]=std::min(vehicle_low[k%3],sample.frame.position_xyz[k]);
                    vehicle_high[k%3]=std::max(vehicle_high[k%3],sample.frame.position_xyz[k]);
                }
            }
            ReplayCamera expected;
            ASSERT_TRUE(MakeBoundsCamera(vehicle_low,vehicle_high,{-1.,-1.,.45},.85,
                ReplayVertical::Z,options.view,expected));
            EXPECT_EQ(scene.camera()->position,expected.position);EXPECT_EQ(scene.camera()->target,expected.target);
            const auto model=scene.system().GetBodies().back()->GetVisualModel();
            ASSERT_TRUE(model);ASSERT_EQ(model->GetShapeInstances().size(),1u);
            const auto wall=std::dynamic_pointer_cast<chrono::ChVisualShapeTriangleMesh>(model->GetShape(0));
            ASSERT_TRUE(wall);ASSERT_TRUE(wall->GetMesh());
            ASSERT_EQ(wall->GetMesh()->GetNumVertices(),4u);ASSERT_EQ(wall->GetMesh()->GetNumTriangles(),2u);
            for(std::size_t n=0;n<4;++n)for(unsigned k=0;k<3;++k) {
                const double x=wall->GetMesh()->GetCoordsVertices()[n][k];
                EXPECT_EQ(output::Bits(x),output::Bits(replay.wall_mesh()->GetCoordsVertices()[n][k]));
                EXPECT_GE(x,bounds.low[k]);EXPECT_LE(x,bounds.high[k]);
            }
            if(colors==ReplayColorMode::PartId) {
                const auto expected_color=ReplayPartColor(replay.environment()->part_id,seed);
                ASSERT_EQ(wall->GetMesh()->GetCoordsColors().size(),4u);
                for(const auto& color:wall->GetMesh()->GetCoordsColors()) {
                    EXPECT_EQ(color.R,expected_color.R);EXPECT_EQ(color.G,expected_color.G);EXPECT_EQ(color.B,expected_color.B);
                }
            }
            ReplayClipping clipping;ASSERT_TRUE(MakeReplayClipping(*scene.camera(),bounds,clipping));
            EXPECT_GT(clipping.near_m,0.);EXPECT_GT(clipping.far_m,clipping.maximum_depth_m);
        }
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
            if(colors==ReplayColorMode::PartId) {
                const auto& parts=*scene.geometry()->triangle_source_parts();
                ASSERT_EQ(parts.size(),mesh->GetIndicesColors().size());
                for(std::size_t t=0;t<parts.size();++t) {
                    const auto actual=mesh->GetCoordsColors()[mesh->GetIndicesColors()[t][0]];
                    const auto expected=ReplayPartColor(parts[t],seed);
                    ASSERT_EQ(actual.R,expected.R);ASSERT_EQ(actual.G,expected.G);ASSERT_EQ(actual.B,expected.B);
                }
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
