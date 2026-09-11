#include "../Scene.h"
#include "chrono/ReplayVisuals.h"
#include <gtest/gtest.h>
#include <limits>
namespace crash::visual::physical_run::test {
TEST(PhysicalSceneValues, CompleteSourceReadbackPresentationBudgetAndLateCapacity) {
    const auto budget=Forecast(512375075,359785,349645,677989,956346);
    RecordProperty("application_peak_host_bytes",std::to_string(budget.peak_host_bytes));
    RecordProperty("geometry_bytes",std::to_string(budget.geometry_bytes));
    RecordProperty("sample_workspace_bytes",std::to_string(budget.sample_workspace_bytes));
    EXPECT_LT(budget.peak_host_bytes,1024u<<20);
    EXPECT_GT(budget.peak_host_bytes,budget.replay_peak_bytes+budget.geometry_bytes);
    SceneOptions options;options.host_bytes=budget.peak_host_bytes;
    EXPECT_EQ(Forecast(512375075,359785,349645,677989,956346,options).peak_host_bytes,budget.peak_host_bytes);
    --options.host_bytes;
    EXPECT_THROW(Forecast(512375075,359785,349645,677989,956346,options),std::exception);
    EXPECT_THROW(Forecast(1,3,1,1,SIZE_MAX),std::exception);
    Scene scene;EXPECT_EQ(scene.Publish(0).status,ReplaySceneStatus::NotInitialized);
    EXPECT_EQ(scene.geometry(),nullptr);EXPECT_EQ(scene.stamp(),nullptr);
    EXPECT_THROW(scene.system(),std::exception);
}
TEST(PhysicalSceneValues, FixedCameraUsesWholeBoundsWithoutChangingGeometryAndStagesFailure) {
    ReplayCamera camera;
    ASSERT_TRUE(MakeBoundsCamera({-2,-1,0},{3,2,2},{-1,-1,.45},1.25,ReplayVertical::Z,
        ReplayView::IncidentSide,camera));
    const auto accepted=camera;
    EXPECT_EQ(camera.target,(std::array<double,3>{.5,.5,1}));
    EXPECT_FALSE(MakeBoundsCamera({0,0,0},{0,0,0},{-1,-1,.45},1.25,ReplayVertical::Z,
        ReplayView::IncidentSide,camera));
    EXPECT_EQ(camera.position,accepted.position);
    EXPECT_FALSE(MakeBoundsCamera({0,0,0},{1,1,std::numeric_limits<double>::infinity()},
        {-1,-1,.45},1.25,ReplayVertical::Z,ReplayView::IncidentSide,camera));
    EXPECT_EQ(camera.position,accepted.position);
    ASSERT_TRUE(MakeBoundsCamera({-2,-1,0},{3,2,2},{-1,-1,.45},1.25,ReplayVertical::Z,
        ReplayView::WallSide,camera));
    EXPECT_EQ(camera.position[0]-camera.target[0],-(accepted.position[0]-accepted.target[0]));
}
} // namespace crash::visual::physical_run::test
