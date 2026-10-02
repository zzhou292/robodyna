#include "FmuInstance.h"
#include <gtest/gtest.h>
#include <cmath>

namespace {
std::string archive;
TEST(VehicleFmu, CoSimulationDriverRespondsToRequestedSpeed) {
    robodyna::test::FmuInstance driver(archive, "driver");
    driver.Initialize();
    chrono::ChFrameMoving<> reference;
    reference.SetPos({-125, -125, .1});
    reference.SetPosDt({0, 0, 0});
    robodyna::test::RequireFmi(driver.unit.SetFrameMovingVariable("ref_frame", reference), "driver frame");
    driver.Set("target_speed", 12);
    driver.Step(0, .001);
    const double throttle = driver.Get("throttle");
    EXPECT_GT(throttle, 0);
    EXPECT_LE(throttle, 1);
    EXPECT_DOUBLE_EQ(driver.Get("braking"), 0);
    EXPECT_TRUE(std::isfinite(driver.Get("steering")));
}
}
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 2) return 2;
    archive = argv[1];
    return RUN_ALL_TESTS();
}
