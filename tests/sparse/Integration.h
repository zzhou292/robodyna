#pragma once

#include "tests/mesh_compat/Fixture.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/physics/ChBody.h"

#include <gtest/gtest.h>
#include <cmath>

namespace robodyna::tests::sparse {

// Reuse the qualified spring mesh and original system configuration. Replacing
// only the solver exercises the real sparse wrapper through an assembled solve.
inline void CheckCoupledSolver(std::shared_ptr<chrono::ChSolver> solver, chrono::ChSolver::Type expected_type) {
    mesh_compat::SpringMesh fixture(1);
    chrono::ChSystemSMC system;
    mesh_compat::Configure(system);
    system.SetSolver(solver);
    ASSERT_EQ(system.GetSolver()->GetType(), expected_type);
    system.AddMesh(fixture.mesh);
    auto body = std::make_shared<chrono::ChBody>();
    body->SetMass(2);
    body->SetInertiaXX({1, 1, 1});
    body->SetSleepingAllowed(false);
    body->SetPos({1.1, 0, 0});
    fixture.moving->SetPos({1.1, 0, 0});
    system.AddBody(body);
    auto joint = std::make_shared<chrono::fea::ChLinkNodeFrame>();
    ASSERT_TRUE(joint->Initialize(fixture.moving, body));
    system.AddLink(joint);
    constexpr double dt = 1e-4;
    for (unsigned step = 0; step < 1000; ++step) {
        ASSERT_TRUE(system.DoStepDynamics(dt, false));
        ASSERT_TRUE(std::isfinite(body->GetPos().x()));
        ASSERT_LT((body->GetPos() - fixture.moving->GetPos()).Length(), 1e-9);
        ASSERT_LT(joint->GetConstraintViolation().norm(), 1e-9);
    }
    EXPECT_NEAR(system.GetChTime(), .1, 1e-13);
    EXPECT_EQ(system.GetNumSteps(), 1000u);
    EXPECT_NEAR(body->GetPos().x(), 1 + .1 * std::cos(.2), 3e-6);
    EXPECT_LT(body->GetPosDt().x(), 0);
    EXPECT_DOUBLE_EQ(body->GetChTime(), system.GetChTime());
    EXPECT_DOUBLE_EQ(fixture.mesh->GetChTime(), system.GetChTime());
    EXPECT_NEAR(body->GetMass() * body->GetPosDt2().x(), joint->GetReactionOnBody().x(), 1e-8);
    EXPECT_LT((joint->GetReactionOnNode() + joint->GetReactionOnBody()).Length(), 1e-12);
}

}  // namespace robodyna::tests::sparse
