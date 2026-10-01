#include "robodyna/fea/RbMesh.h"
#include "robodyna/mbd/RbBody.h"
#include "robodyna/simulation/RbSystemNSC.h"
#include "robodyna/simulation/RbSystemSMC.h"
#include "chrono/timestepper/ChTimestepperImplicit.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <type_traits>

namespace {
namespace rd = robodyna;

class Body final : public rd::mbd::RbBody {
  public:
    void Update(double time, chrono::UpdateFlags flags) override {
        ++updates;
        RbBody::Update(time, flags);
    }
    unsigned updates = 0;
};

class Timestepper final : public chrono::ChTimestepperEulerImplicitLinearized {
  public:
    explicit Timestepper(chrono::ChIntegrableIIorder* owner)
        : ChTimestepperEulerImplicitLinearized(owner) {}
    void SetTime(double time) override {
        ++set_calls;
        ChTimestepperEulerImplicitLinearized::SetTime(time);
    }
    unsigned set_calls = 0;
};

static_assert(std::is_same_v<decltype(&rd::simulation::RbSystem::GetTime),
                             double (rd::simulation::RbSystem::*)() const>);
static_assert(std::is_same_v<decltype(&rd::simulation::RbSystem::SetTime),
                             void (rd::simulation::RbSystem::*)(double)>);

template <class System>
void CheckIndependentOwners() {
    System system;
    system.SetNumThreads(1, 1, 1);
    auto first = std::make_shared<Body>();
    auto second = std::make_shared<Body>();
    auto mesh = std::make_shared<rd::fea::RbMesh>();
    system.AddBody(first);
    system.AddBody(second);
    system.AddMesh(mesh);
    auto timestepper = std::make_shared<Timestepper>(&system);
    system.SetTimestepper(timestepper);
    const rd::simulation::RbSystem& read_only = system;

    first->SetChTime(11.0);
    second->SetTime(12.0);
    mesh->SetTime(13.0);
    system.GetTimestepper()->SetTime(14.0);  // Existing virtual API remains independent.
    const auto first_updates = first->updates;
    const auto second_updates = second->updates;
    const auto initial_step = system.GetStep();
    const auto initial_steps = system.GetNumSteps();
    const auto assembly_time = system.GetAssembly().GetChTime();

    system.SetChTime(1.25);
    EXPECT_DOUBLE_EQ(read_only.GetTime(), 1.25);
    system.SetTime(-2.5);
    EXPECT_DOUBLE_EQ(read_only.GetChTime(), -2.5);
    EXPECT_DOUBLE_EQ(first->GetTime(), 11.0);
    EXPECT_DOUBLE_EQ(second->GetChTime(), 12.0);
    EXPECT_DOUBLE_EQ(mesh->GetChTime(), 13.0);
    EXPECT_DOUBLE_EQ(system.GetTimestepper()->GetTime(), 14.0);
    EXPECT_EQ(timestepper->set_calls, 1u);
    EXPECT_EQ(system.GetAssembly().GetChTime(), assembly_time);
    EXPECT_EQ(system.GetStep(), initial_step);
    EXPECT_EQ(system.GetNumSteps(), initial_steps);
    EXPECT_EQ(first->updates, first_updates);
    EXPECT_EQ(second->updates, second_updates);
    EXPECT_FALSE(system.IsInitialized());
    EXPECT_EQ(system.GetVisualSystem(), nullptr);

    first->SetTime(21.0);
    mesh->SetChTime(23.0);
    EXPECT_DOUBLE_EQ(first->GetChTime(), 21.0);
    EXPECT_DOUBLE_EQ(mesh->GetTime(), 23.0);
    EXPECT_DOUBLE_EQ(read_only.GetTime(), -2.5);
    EXPECT_DOUBLE_EQ(second->GetTime(), 12.0);
    EXPECT_DOUBLE_EQ(system.GetTimestepper()->GetTime(), 14.0);

    system.GetTimestepper()->SetTime(24.0);
    EXPECT_EQ(timestepper->set_calls, 2u);
    EXPECT_DOUBLE_EQ(read_only.GetTime(), -2.5);
    EXPECT_DOUBLE_EQ(first->GetTime(), 21.0);
    EXPECT_DOUBLE_EQ(mesh->GetTime(), 23.0);
}

TEST(SystemTime, NscSetterDoesNotAdvanceOrSynchronizeOtherOwners) {
    CheckIndependentOwners<rd::simulation::RbSystemNSC>();
}

TEST(SystemTime, SmcSetterDoesNotAdvanceOrSynchronizeOtherOwners) {
    CheckIndependentOwners<rd::simulation::RbSystemSMC>();
}

TEST(SystemTime, AssignmentStillAcceptsTheLegacyFloatingPointDomain) {
    rd::simulation::RbSystemNSC system;
    system.SetTime(-0.0);
    EXPECT_TRUE(std::signbit(system.GetChTime()));
    system.SetChTime(std::numeric_limits<double>::infinity());
    EXPECT_TRUE(std::isinf(system.GetTime()));
    system.SetTime(std::numeric_limits<double>::quiet_NaN());
    EXPECT_TRUE(std::isnan(system.GetChTime()));
    EXPECT_EQ(system.GetNumSteps(), 0u);
    EXPECT_FALSE(system.IsInitialized());
}

}  // namespace
