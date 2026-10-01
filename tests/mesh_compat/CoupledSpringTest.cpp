#include "Fixture.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/physics/ChBody.h"

#include <gtest/gtest.h>

#include <cmath>

TEST(MeshCompatibility, FeSpringAndBodyAdvanceInOneConstrainedSystem) {
    using namespace robodyna::tests::mesh_compat;
    SpringMesh fixture(1);
    chrono::ChSystemSMC system;
    Configure(system);
    system.AddMesh(fixture.mesh);
    auto body = chrono_types::make_shared<chrono::ChBody>();
    body->SetMass(2);
    body->SetInertiaXX({1, 1, 1});
    body->SetSleepingAllowed(false);
    body->SetPos({1.1, 0, 0});
    fixture.moving->SetPos({1.1, 0, 0});
    system.AddBody(body);
    auto link = chrono_types::make_shared<chrono::fea::ChLinkNodeFrame>();
    ASSERT_TRUE(link->Initialize(fixture.moving, body));
    system.AddLink(link);

    constexpr double dt = 1e-4;
    constexpr unsigned steps = 1000;
    for (unsigned step = 0; step < steps; ++step) {
        ASSERT_TRUE(system.DoStepDynamics(dt, false));
        ASSERT_TRUE(std::isfinite(body->GetPos().x()));
        EXPECT_LT((body->GetPos() - fixture.moving->GetPos()).Length(), 1e-9);
        EXPECT_LT(link->GetConstraintViolation().norm(), 1e-9);
    }
    const double time = steps * dt;
    // k=12, total moving mass=1+2: omega=2. No prescribed motion is used.
    const double expected = 1 + .1 * std::cos(2 * time);
    EXPECT_NEAR(body->GetPos().x(), expected, 3e-6);
    EXPECT_NEAR(fixture.moving->GetPos().x(), expected, 3e-6);
    EXPECT_LT(body->GetPosDt().x(), 0);
    EXPECT_NEAR(system.GetChTime(), time, 1e-13);
    EXPECT_DOUBLE_EQ(body->GetChTime(), system.GetChTime());
    EXPECT_DOUBLE_EQ(fixture.mesh->GetChTime(), system.GetChTime());
    EXPECT_DOUBLE_EQ(link->GetChTime(), system.GetChTime());
    EXPECT_EQ(body->GetSystem(), &system);
    EXPECT_EQ(fixture.mesh->GetSystem(), &system);
    EXPECT_EQ(link->GetSystem(), &system);
    EXPECT_EQ(link->GetConstrainedNode(), fixture.moving);
    EXPECT_EQ(link->GetConstrainedBodyFrame().get(), static_cast<chrono::ChBodyFrame*>(body.get()));
    EXPECT_LT(link->GetReactionOnBody().x(), 0);
    EXPECT_NEAR(body->GetMass() * body->GetPosDt2().x(), link->GetReactionOnBody().x(), 1e-8);
    EXPECT_LT((link->GetReactionOnNode() + link->GetReactionOnBody()).Length(), 1e-12);
    EXPECT_EQ(system.GetNumMeshes(), 1u);
    EXPECT_EQ(system.GetBodies().size(), 1u);
}
