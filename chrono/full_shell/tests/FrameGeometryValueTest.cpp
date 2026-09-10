#include "../FullShellFrameGeometry.h"
#include "chrono/ReplayDisplayGeometry.h"
#include <gtest/gtest.h>
#include <limits>

namespace crash::visual::full_shell::test {
TEST(FullShellGeometryValues,ExplicitSourceSizedGeometryAndByteBudget) {
    FrameGeometryOptions options;
    EXPECT_THROW(FrameGeometryBudget(359785,349645,677989,options),std::exception);
    options.geometry=ReplayGeometryLimits::Vehicle();
    const auto bytes=FrameGeometryBudget(359785,349645,677989,options);
    EXPECT_LT(bytes,384u*1024*1024);
    options.host_bytes=bytes;EXPECT_EQ(FrameGeometryBudget(359785,349645,677989,options),bytes);
    --options.host_bytes;EXPECT_THROW(FrameGeometryBudget(359785,349645,677989,options),std::exception);
    EXPECT_THROW(FrameGeometryBudget(SIZE_MAX,1,1,options),std::exception);
    options.host_bytes=384*1024*1024;options.colors=static_cast<ReplayColorMode>(99);
    EXPECT_THROW(FrameGeometryBudget(1,1,1,options),std::exception);
}
TEST(FullShellGeometryValues,SharedRendererValidationAndUninitializedAdapter) {
    FullShellFrameGeometry geometry;
    EXPECT_EQ(geometry.Update({},{}).status,ReplaySceneStatus::NotInitialized);
    EXPECT_EQ(geometry.mesh(),nullptr);EXPECT_EQ(geometry.fields(),nullptr);EXPECT_EQ(geometry.stamp(),nullptr);
    const std::vector<chrono::ChVector3i> indices{{0,1,2}};
    std::vector<chrono::ChVector3d> positions{{-0.,0,0},{1,0,0},{0,1,0}};
    EXPECT_TRUE(CheckReplayDisplayGeometry(positions,indices));
    positions.back()={0,0,0};EXPECT_FALSE(CheckReplayDisplayGeometry(positions,indices));
    positions.back()={0,std::numeric_limits<double>::quiet_NaN(),0};EXPECT_FALSE(CheckReplayDisplayGeometry(positions,indices));
    positions={{1,0,0},{1+1e-9,0,0},{1,1e-9,0}};
    EXPECT_FALSE(CheckReplayDisplayGeometry(positions,indices)); // Binary32 collapses first edge.
}
} // namespace crash::visual::full_shell::test
