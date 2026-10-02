#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"
#include "robodyna/simulation/RbSystemSMC.h"

#include <gtest/gtest.h>
#include <string>

#ifndef CHRONO_OPENCRG
#error "The OpenCRG implementation and consumer must share the enabled Vehicle profile"
#endif

namespace {
std::string road_file;
}

TEST(OpenCrgHost, OriginalTerrainLoadsTheRetainedRoadAndEvaluatesItsHeight) {
    robodyna::simulation::RbSystemSMC system;
    chrono::vehicle::CRGTerrain terrain(&system);
    terrain.UseMeshVisualization(false);
    terrain.Initialize(road_file);
    EXPECT_DOUBLE_EQ(terrain.GetLength(), 22);
    EXPECT_DOUBLE_EQ(terrain.GetWidth(), 3);
    // Exact road file has a zero-height strip at y=-1 and a .0111111-m sample
    // at u=1,v=0. The library stores real-valued road samples at its own precision.
    EXPECT_NEAR(terrain.GetHeight(chrono::ChVector3d(0.25, -1, 0)), 0, 1e-12);
    EXPECT_NEAR(terrain.GetHeight(chrono::ChVector3d(1, 0, 0)), 0.0111111, 1e-8);
    const auto normal = terrain.GetNormal(chrono::ChVector3d(0.25, -1, 0));
    EXPECT_NEAR((normal - chrono::ChVector3d(0, 0, 1)).Length(), 0, 1e-12);
    EXPECT_FLOAT_EQ(terrain.GetCoefficientFriction(chrono::ChVector3d(1, 0, 0)), 0.8f);
    ASSERT_EQ(system.GetBodies().size(), 1);
    EXPECT_TRUE(system.GetBodies()[0]->IsFixed());
    EXPECT_DOUBLE_EQ(system.GetTime(), 0);
    EXPECT_EQ(system.GetVisualSystem(), nullptr);
}

int main(int argc, char** argv) {
    if (argc < 2)
        return 2;
    road_file = argv[1];
    for (int index = 1; index + 1 < argc; ++index)
        argv[index] = argv[index + 1];
    --argc;
    argv[argc] = nullptr;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
