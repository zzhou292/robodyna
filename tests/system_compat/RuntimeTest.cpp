#include "chrono/collision/ChCollisionModel.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/physics/ChContactContainerSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include <gtest/gtest.h>

#include <limits>
#include <memory>

namespace {
class CollisionObserver : public chrono::ChSystem::CustomCollisionCallback {
  public:
    void OnCustomCollision(chrono::ChSystem* system) override {
        owner = system;
        time = system->GetChTime();
        ++calls;
    }
    chrono::ChSystem* owner = nullptr;
    double time = 0;
    unsigned calls = 0;
};

TEST(SystemCompatibility, ConstructorsKeepTheirCurrentContactAndSolverProfiles) {
    chrono::ChSystemNSC nsc("named NSC");
    EXPECT_EQ(nsc.GetName(), "named NSC");
    EXPECT_EQ(nsc.GetContactMethod(), chrono::ChContactMethod::NSC);
    EXPECT_DOUBLE_EQ(chrono::ChCollisionModel::GetDefaultSuggestedEnvelope(), .03);
    EXPECT_DOUBLE_EQ(chrono::ChCollisionModel::GetDefaultSuggestedMargin(), .01);
    nsc.SetMinBounceSpeed(.2);  // The baseline never reads an unset container field.
    auto nsc_contacts = std::dynamic_pointer_cast<chrono::ChContactContainerNSC>(nsc.GetContactContainer());
    ASSERT_NE(nsc_contacts, nullptr);
    EXPECT_DOUBLE_EQ(nsc_contacts->GetMinBounceSpeed(), .2);

    chrono::ChSystemSMC smc("named SMC");
    EXPECT_EQ(smc.GetName(), "named SMC");
    EXPECT_EQ(smc.GetContactMethod(), chrono::ChContactMethod::SMC);
    EXPECT_DOUBLE_EQ(chrono::ChCollisionModel::GetDefaultSuggestedEnvelope(), 0);
    EXPECT_TRUE(smc.UsingMaterialProperties());
    EXPECT_EQ(smc.GetContactForceModel(), chrono::ChSystemSMC::Hertz);
    EXPECT_EQ(smc.GetAdhesionForceModel(), chrono::ChSystemSMC::AdhesionForceModel::Constant);
    EXPECT_EQ(smc.GetTangentialDisplacementModel(), chrono::ChSystemSMC::OneStep);
    EXPECT_FALSE(smc.IsContactStiff());
    for (chrono::ChSystem* system : {static_cast<chrono::ChSystem*>(&nsc), static_cast<chrono::ChSystem*>(&smc)}) {
        EXPECT_EQ(system->GetSolverType(), chrono::ChSolver::Type::PSOR);
        EXPECT_EQ(system->GetTimestepperType(), chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
        EXPECT_DOUBLE_EQ(system->GetChTime(), 0);
        EXPECT_DOUBLE_EQ(system->GetStep(), .04);
        EXPECT_DOUBLE_EQ(system->GetGravitationalAcceleration().Length2(), 0);
        EXPECT_EQ(system->GetNumThreadsChrono(), 1u);
        EXPECT_EQ(system->GetNumThreadsCollision(), 1u);
        EXPECT_EQ(system->GetNumThreadsEigen(), 1u);
        EXPECT_EQ(system->GetContactContainer()->GetSystem(), system);
        EXPECT_FALSE(system->IsInitialized());
        EXPECT_EQ(system->GetVisualSystem(), nullptr);
    }
}

TEST(SystemCompatibility, FactoryAndStaticCreateRetainConcreteDynamicTypes) {
    auto nsc = chrono::ChSystem::Create(chrono::ChContactMethod::NSC);
    auto smc = chrono::ChSystem::Create(chrono::ChContactMethod::SMC);
    ASSERT_NE(std::dynamic_pointer_cast<chrono::ChSystemNSC>(nsc), nullptr);
    ASSERT_NE(std::dynamic_pointer_cast<chrono::ChSystemSMC>(smc), nullptr);
    EXPECT_EQ(chrono::ChSystem::Create(static_cast<chrono::ChContactMethod>(99)), nullptr);
    chrono::ChSystemNSC* raw_nsc = nullptr;
    chrono::ChSystemSMC* raw_smc = nullptr;
    chrono::ChClassFactory::create("ChSystemNSC", &raw_nsc);
    chrono::ChClassFactory::create("ChSystemSMC", &raw_smc);
    std::unique_ptr<chrono::ChSystemNSC> restored_nsc(raw_nsc);
    std::unique_ptr<chrono::ChSystemSMC> restored_smc(raw_smc);
    ASSERT_NE(restored_nsc, nullptr);
    ASSERT_NE(restored_smc, nullptr);
    EXPECT_EQ(restored_nsc->GetContactMethod(), chrono::ChContactMethod::NSC);
    EXPECT_EQ(restored_smc->GetContactMethod(), chrono::ChContactMethod::SMC);
}

TEST(SystemCompatibility, ContainerReplacementAdmitsOnlyTheMatchingContactMethod) {
    chrono::ChSystemNSC nsc;
    chrono::ChSystemSMC smc;
    const auto original_nsc = nsc.GetContactContainer();
    const auto original_smc = smc.GetContactContainer();
    nsc.SetContactContainer(original_smc);
    smc.SetContactContainer(original_nsc);
    EXPECT_EQ(nsc.GetContactContainer(), original_nsc);
    EXPECT_EQ(smc.GetContactContainer(), original_smc);
    auto new_nsc = chrono_types::make_shared<chrono::ChContactContainerNSC>();
    auto new_smc = chrono_types::make_shared<chrono::ChContactContainerSMC>();
    nsc.SetContactContainer(new_nsc);
    smc.SetContactContainer(new_smc);
    EXPECT_EQ(nsc.GetContactContainer(), new_nsc);
    EXPECT_EQ(smc.GetContactContainer(), new_smc);
    EXPECT_EQ(new_nsc->GetSystem(), &nsc);
    EXPECT_EQ(new_smc->GetSystem(), &smc);
    nsc.SetMinBounceSpeed(.375);
    EXPECT_DOUBLE_EQ(new_nsc->GetMinBounceSpeed(), .375);
    smc.SetSlipVelocityThreshold(-1);
    EXPECT_DOUBLE_EQ(smc.GetSlipVelocityThreshold(), std::numeric_limits<double>::epsilon());
    smc.SetSlipVelocityThreshold(.03);
    EXPECT_DOUBLE_EQ(smc.GetSlipVelocityThreshold(), .03);
}

TEST(SystemCompatibility, RealStepsShareTheBodyClockAndInvokeThenRemoveCallbacks) {
    for (const auto method : {chrono::ChContactMethod::NSC, chrono::ChContactMethod::SMC}) {
        auto system = chrono::ChSystem::Create(method);
        system->SetNumThreads(1, 1, 1);
        system->SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
        if (auto nsc = std::dynamic_pointer_cast<chrono::ChSystemNSC>(system))
            nsc->SetMinBounceSpeed(.2);
        const chrono::ChVector3d initial(1, -2, .25);
        const chrono::ChVector3d velocity(.3, -.2, .1);
        auto body = chrono_types::make_shared<chrono::ChBody>();
        body->SetMass(2);
        body->SetInertiaXX({1, 2, 3});
        body->SetSleepingAllowed(false);
        body->SetPos(initial);
        body->SetPosDt(velocity);
        system->AddBody(body);
        auto observer = std::make_shared<CollisionObserver>();
        system->RegisterCustomCollisionCallback(observer);
        constexpr double dt = .001;
        constexpr unsigned steps = 100;
        for (unsigned step = 0; step < steps; ++step) {
            ASSERT_TRUE(system->DoStepDynamics(dt));
            EXPECT_DOUBLE_EQ(body->GetChTime(), system->GetChTime());
        }
        EXPECT_EQ(observer->calls, steps);
        EXPECT_EQ(observer->owner, system.get());
        EXPECT_NEAR(observer->time, (steps - 1) * dt, 1e-13);
        EXPECT_EQ(system->GetNumSteps(), steps);
        EXPECT_DOUBLE_EQ(system->GetStep(), dt);
        EXPECT_LT((body->GetPos() - initial - velocity * (steps * dt)).Length(), 1e-11);
        EXPECT_LT((body->GetPosDt() - velocity).Length(), 1e-13);
        EXPECT_EQ(body->GetSystem(), system.get());
        system->UnregisterCustomCollisionCallback(observer);
        ASSERT_TRUE(system->DoStepDynamics(dt));
        EXPECT_EQ(observer->calls, steps);
        system->Clear();
        EXPECT_TRUE(system->GetBodies().empty());
        EXPECT_EQ(body->GetSystem(), nullptr);
    }
}
}  // namespace
