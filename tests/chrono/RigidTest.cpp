#include <gtest/gtest.h>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/serialization/ChArchiveJSON.h"

#include <memory>
#include <sstream>

TEST(ChronoHostBridge, UnloadedBodyPreservesVelocity) {
    chrono::ChSystemSMC system;
    system.SetNumThreads(1, 1, 1);
    system.SetGravitationalAcceleration({0, 0, 0});
    auto body = std::make_shared<chrono::ChBody>();
    const chrono::ChVector3d initial(1, -2, .25);
    const chrono::ChVector3d velocity(.3, -.2, .1);
    body->SetPos(initial);
    body->SetPosDt(velocity);
    body->SetMass(2);
    body->SetInertiaXX({1, 2, 3});
    body->SetSleepingAllowed(false);
    system.AddBody(body);
    constexpr double dt = .001;
    constexpr int steps = 100;
    for (int i = 0; i < steps; ++i)
        ASSERT_TRUE(system.DoStepDynamics(dt, false));
    EXPECT_NEAR(system.GetChTime(), steps * dt, 1e-14);
    EXPECT_LT((body->GetPos() - initial - velocity * (steps * dt)).Length(), 1e-11);
    EXPECT_LT((body->GetPosDt() - velocity).Length(), 1e-13);
}

TEST(ChronoHostBridge, RigidBodyRestoresThroughArchiveFactory) {
    std::stringstream stream;
    std::shared_ptr<chrono::ChPhysicsItem> item = std::make_shared<chrono::ChBody>();
    auto body = std::dynamic_pointer_cast<chrono::ChBody>(item);
    body->SetMass(3.25);
    body->SetPos({1.5, -2.25, .125});
    {
        chrono::ChArchiveOutJSON output(stream);
        output << CHNVP(item);
    }
    item.reset();
    {
        chrono::ChArchiveInJSON input(stream);
        input >> CHNVP(item);
    }
    const auto restored = std::dynamic_pointer_cast<chrono::ChBody>(item);
    ASSERT_NE(restored, nullptr);
    EXPECT_DOUBLE_EQ(restored->GetMass(), 3.25);
    EXPECT_LT((restored->GetPos() - body->GetPos()).Length(), 1e-14);
}
