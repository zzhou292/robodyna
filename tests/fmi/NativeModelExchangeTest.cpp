#include "chrono_fmi/ChExternalFmu.h"
#include "chrono/physics/ChSystemSMC.h"

#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

namespace {
std::string archives[2];

void CheckModelExchange(unsigned version) {
    const char* temporary = std::getenv("TEST_TMPDIR");
    ASSERT_NE(temporary, nullptr);
    const auto unpack = std::filesystem::path(temporary) / ("vdp-fmi" + std::to_string(version));
    ASSERT_TRUE(std::filesystem::create_directory(unpack));
    auto fmu = std::make_shared<chrono::ChExternalFmu>();
    fmu->Load("bounded-native-vdp", archives[version - 2], unpack.string());
    ASSERT_EQ(fmu->GetNumStates(), 2u);
    EXPECT_EQ(fmu->GetStatesList().count("x"), 1u);
    EXPECT_EQ(fmu->GetStatesList().count("v"), 1u);
    fmu->SetInitialCondition("x", 2.5);
    fmu->SetInitialCondition("v", 0.0);
    fmu->SetRealParameterValue("mu", 1.5);
    fmu->SetRealInputFunction("u", [](double) { return 0.0; });
    fmu->Initialize();
    ASSERT_EQ(fmu->GetStates().size(), 2);
    EXPECT_NEAR(fmu->GetStates()[0], 2.5, 1e-14);
    EXPECT_NEAR(fmu->GetStates()[1], 0.0, 1e-14);

    chrono::ChSystemSMC system;
    system.SetNumThreads(1, 1, 1);
    system.SetGravitationalAcceleration({0, 0, 0});
    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    system.Add(fmu);
    constexpr double dt = 1e-5;
    ASSERT_TRUE(system.DoStepDynamics(dt, false));
    EXPECT_DOUBLE_EQ(system.GetChTime(), dt);
    EXPECT_DOUBLE_EQ(fmu->GetChTime(), system.GetChTime());
    EXPECT_EQ(system.GetNumSteps(), 1u);
    // Retained VdP equation: x'=v, v'=mu*(1-x*x)*v-x+u.
    // The initial acceleration is -2.5; this tolerance admits O(dt^2).
    EXPECT_NEAR(fmu->GetStates()[0], 2.5, 1e-8);
    EXPECT_NEAR(fmu->GetStates()[1], -2.5 * dt, 1e-8);
    EXPECT_NEAR(fmu->GetRealVariable("x"), fmu->GetStates()[0], 1e-12);
    EXPECT_NEAR(fmu->GetRealVariable("v"), fmu->GetStates()[1], 1e-12);
}

TEST(NativeFmi, Fmi2ModelExchangeUsesTheRealSystemClockAndDerivative) {
    ASSERT_NO_FATAL_FAILURE(CheckModelExchange(2));
}
TEST(NativeFmi, Fmi3ModelExchangeUsesTheRealSystemClockAndDerivative) {
    ASSERT_NO_FATAL_FAILURE(CheckModelExchange(3));
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) return 2;
    archives[0] = argv[1];
    archives[1] = argv[2];
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
