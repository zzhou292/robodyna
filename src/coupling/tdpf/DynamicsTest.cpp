#include "robodyna/mbd/RbBody.h"
#include "robodyna/simulation/RbSystemNSC.h"
#include "chrono_fsi/tdpf/ChFsiSystemTDPF.h"

#include <gtest/gtest.h>
#include <cmath>
#include <string>

namespace {
std::string hydro_file;

struct Result {
    chrono::ChVector3d position;
    chrono::ChVector3d velocity;
    double time;
};

Result Advance(bool generic_interface) {
    robodyna::simulation::RbSystemNSC mechanics;
    mechanics.SetNumThreads(1, 1, 1);
    // Match the retained sphere-decay solver. TDPF contributes added mass,
    // which the generic NSC default PSOR solver cannot accept.
    mechanics.SetSolverType(chrono::ChSolver::Type::BARZILAIBORWEIN);
    mechanics.GetSolver()->AsIterative()->SetMaxIterations(300);
    mechanics.GetSolver()->AsIterative()->EnableDiagonalPreconditioner(true);
    auto sphere = std::make_shared<robodyna::mbd::RbBody>();
    sphere->SetName("body1");
    sphere->SetMass(261.8e3);  // The retained sphere-decay model.
    sphere->SetInertiaXX({1, 1, 1});
    sphere->SetPos({0, 0, -1});
    mechanics.AddBody(sphere);

    chrono::fsi::tdpf::ChFsiFluidSystemTDPF fluid;
    fluid.SetHydroFilename(hydro_file);
    chrono::fsi::tdpf::ChFsiSystemTDPF coupled(&mechanics, &fluid, generic_interface);
    coupled.SetGravitationalAcceleration({0, 0, -9.81});
    coupled.SetVerbose(false);
    constexpr double dt = 1e-3;
    coupled.SetStepSizeCFD(dt);
    coupled.SetStepsizeMBD(dt);
    coupled.AddRigidBody(sphere, nullptr, false);
    coupled.Initialize();
    for (unsigned step = 0; step < 100; ++step)
        coupled.DoStepDynamics(dt);
    return {sphere->GetPos(), sphere->GetPosDt(), mechanics.GetChTime()};
}

TEST(TdpfCoupling, GenericAndNativeInterfacesAdvanceTheSamePhysicalBody) {
    const auto native = Advance(false);
    const auto generic = Advance(true);
    EXPECT_NEAR(native.time, .1, 1e-12);
    EXPECT_NEAR(generic.time, .1, 1e-12);
    EXPECT_TRUE(std::isfinite(native.position.z()));
    EXPECT_TRUE(std::isfinite(native.velocity.z()));
    EXPECT_GT(std::abs(native.position.z() + 1), 1e-8);
    EXPECT_NEAR((native.position - generic.position).Length(), 0, 1e-10);
    EXPECT_NEAR((native.velocity - generic.velocity).Length(), 0, 1e-10);
}
}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 2)
        return 2;
    hydro_file = argv[1];
    return RUN_ALL_TESTS();
}
