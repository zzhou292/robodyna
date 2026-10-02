#include "FmuInstance.h"
#include <gtest/gtest.h>
#include <cmath>

namespace {
std::string vehicle_archive;
std::string powertrain_archive;

void CheckUniformGravity(const std::string& archive, const std::string& name) {
    robodyna::test::FmuInstance zero(archive, name + "-zero");
    robodyna::test::FmuInstance gravity(archive, name + "-gravity");
    zero.SetVector("g_acc", {0, 0, 0});
    gravity.SetVector("g_acc", {0, 0, -9.8});
    constexpr double dt = 0.001;
    constexpr int steps = 20;
    zero.Set("step_size", dt);
    gravity.Set("step_size", dt);
    zero.Initialize();
    gravity.Initialize();
    for (int step = 0; step < steps; ++step) {
        zero.Step(step * dt, dt);
        gravity.Step(step * dt, dt);
    }
    const auto frame0 = zero.Frame();
    const auto frameg = gravity.Frame();
    ASSERT_TRUE(std::isfinite(frameg.GetPos().z()));
    // All unconstrained-to-ground vehicle masses receive the same acceleration.
    // Internal suspension transients cancel in this independent physical check.
    EXPECT_NEAR(frameg.GetPosDt().z() - frame0.GetPosDt().z(), -9.8 * steps * dt, 5e-5);
    EXPECT_NEAR(frameg.GetPos().z() - frame0.GetPos().z(), -9.8 * dt * dt * steps * (steps + 1) / 2, 5e-6);
    for (const char* wheel : {"FL", "FR", "RL", "RR"}) {
        const auto p = gravity.GetVector("wheel_" + std::string(wheel) + ".pos");
        EXPECT_TRUE(std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()));
    }
}
TEST(VehicleFmu, WheeledVehiclePreservesUniformGravityResponse) {
    ASSERT_NO_FATAL_FAILURE(CheckUniformGravity(vehicle_archive, "vehicle"));
}
TEST(VehicleFmu, VehicleWithPowertrainPreservesUniformGravityResponse) {
    ASSERT_NO_FATAL_FAILURE(CheckUniformGravity(powertrain_archive, "vehicle-powertrain"));
}
}
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 3) return 2;
    vehicle_archive = argv[1]; powertrain_archive = argv[2];
    return RUN_ALL_TESTS();
}
