#include "FmuInstance.h"
#include <gtest/gtest.h>
#include <cmath>

namespace {
std::string tire_archive;
std::string powertrain_archive;

TEST(VehicleFmu, ForceElementTireDistinguishesSeparationFromCompression) {
    robodyna::test::FmuInstance tire(tire_archive, "tire");
    tire.Initialize();  // Continuous inputs become writable in the FMI step state.
    tire.SetVector("terrain_normal", {0, 0, 1});
    tire.Set("terrain_height", 0);
    tire.Set("terrain_mu", .8);
    tire.SetVector("wheel_state.pos", {0, 0, 1});
    tire.SetVector("wheel_state.lin_vel", {0, 0, 0});
    tire.SetVector("wheel_state.ang_vel", {0, 0, 0});
    robodyna::test::RequireFmi(tire.unit.SetQuatVariable("wheel_state.rot", chrono::QUNIT), "wheel rotation");
    tire.Step(0, .001);
    EXPECT_NEAR(tire.GetVector("wheel_load.force").Length(), 0, 1e-10);
    // Packaged TMeasy tire has unloaded radius0.4699m.
    tire.SetVector("wheel_state.pos", {0, 0, .45});
    tire.Step(.001, .001);
    const auto force = tire.GetVector("wheel_load.force");
    EXPECT_TRUE(std::isfinite(force.z()));
    EXPECT_GT(force.z(), 0);
    EXPECT_NEAR(tire.GetVector("query_point").z(), .45, 1e-14);
}

TEST(VehicleFmu, PowertrainLoadsPackagedShaftModelsAndProducesFiniteOutputs) {
    robodyna::test::FmuInstance powertrain(powertrain_archive, "powertrain");
    powertrain.Initialize();  // Fixed resource parameters retain their defaults.
    powertrain.Set("throttle", .5);
    powertrain.Set("clutch", 0);
    powertrain.Set("driveshaft_speed", 10);
    for (int step = 0; step < 20; ++step) powertrain.Step(step * .001, .001);
    EXPECT_TRUE(std::isfinite(powertrain.Get("driveshaft_torque")));
    EXPECT_TRUE(std::isfinite(powertrain.Get("engine_reaction")));
    EXPECT_TRUE(std::isfinite(powertrain.Get("transmission_reaction")));
}
}
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 3) return 2;
    tire_archive = argv[1]; powertrain_archive = argv[2];
    return RUN_ALL_TESTS();
}
