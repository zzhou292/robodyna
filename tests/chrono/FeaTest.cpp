#include <gtest/gtest.h>

#include "chrono/ChConfig.h"
#include "chrono/fea/ChElementSpring.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/physics/ChSystemSMC.h"

#include <cmath>
#include <memory>

#ifndef CHRONO_FEA
#error "The declared Chrono bridge must enable FEA"
#endif

TEST(ChronoHostBridge, FiniteElementSpringAdvancesInMechanicalAssembly) {
    chrono::ChSystemSMC system;
    system.SetNumThreads(1, 1, 1);
    system.SetGravitationalAcceleration({0, 0, 0});
    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    system.SetSolverType(chrono::ChSolver::Type::MINRES);
    auto mesh = std::make_shared<chrono::fea::ChMesh>();
    auto fixed = std::make_shared<chrono::fea::ChNodeFEAxyz>(chrono::ChVector3d(0, 0, 0));
    auto moving = std::make_shared<chrono::fea::ChNodeFEAxyz>(chrono::ChVector3d(1, 0, 0));
    fixed->SetFixed(true);
    moving->SetMass(3);
    auto element = std::make_shared<chrono::fea::ChElementSpring>();
    element->SetNodes(fixed, moving);
    element->SetSpringCoefficient(12);
    element->SetDampingCoefficient(0);
    mesh->AddNode(fixed);
    mesh->AddNode(moving);
    mesh->AddElement(element);
    mesh->SetAutomaticGravity(false);
    system.AddMesh(mesh);
    moving->SetPos({1.1, 0, 0});

    constexpr double dt = .0001;
    constexpr int steps = 1000;
    for (int i = 0; i < steps; ++i)
        ASSERT_TRUE(system.DoStepDynamics(dt, false));
    const double time = steps * dt;
    const double expected = 1 + .1 * std::cos(2 * time);  // omega = sqrt(k / m).
    EXPECT_NEAR(system.GetChTime(), time, 1e-13);
    // The inherited first-order implicit integrator adds numerical damping.
    EXPECT_NEAR(moving->GetPos().x(), expected, 3e-6);
    EXPECT_NEAR(moving->GetPos().y(), 0, 1e-14);
    EXPECT_NEAR(moving->GetPos().z(), 0, 1e-14);
    EXPECT_EQ(mesh->GetNumNodes(), 2u);
    EXPECT_EQ(mesh->GetNumElements(), 1u);
    EXPECT_GT(mesh->GetNumCallsInternalForces(), 0u);
}
