#include "chrono/ChConfig.h"
#include "chrono/collision/multicore/ChCollisionSystemMulticore.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChOpenMP.h"
#include "chrono_multicore/ChConfigMulticore.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"

#include <gtest/gtest.h>
#include <omp.h>
#include <cmath>
#include <type_traits>

#if !defined(CHRONO_MULTICORE) || !defined(CHRONO_COLLISION) || !defined(CHRONO_OPENMP_ENABLED)
#error "This gate requires the coherent Multicore configuration"
#endif
#if !defined(_OPENMP) || _OPENMP < 201307
#error "The selected host compiler must enable OpenMP 4.0 or newer"
#endif

static_assert(std::is_same_v<chrono::real, double>);

namespace {
TEST(MulticoreProfile, ExistingChOmpOwnerReportsTheRealParallelTeam) {
    const int previous = omp_get_max_threads();
    chrono::ChOMP::SetNumThreads(2);
    EXPECT_EQ(chrono::ChOMP::GetMaxThreads(), 2);
    EXPECT_EQ(chrono::ChOMP::GetThreadNum(), 0);
    int identities[2] = {-1, -1};
    int sizes[2] = {0, 0};
#pragma omp parallel num_threads(2)
    {
        const int id = omp_get_thread_num();
        identities[id] = chrono::ChOMP::GetThreadNum();
        sizes[id] = chrono::ChOMP::GetNumThreads();
    }
    EXPECT_EQ(identities[0], 0);
    EXPECT_EQ(identities[1], 1);
    EXPECT_EQ(sizes[0], 2);
    EXPECT_EQ(sizes[1], 2);
    chrono::ChOMP::SetNumThreads(previous);
}

TEST(MulticoreProfile, GenericSystemFactoryUsesTheActualCollisionBackend) {
    chrono::ChSystemNSC system;
    system.SetNumThreads(2, 2, 1);
    system.SetGravitationalAcceleration({0, 0, 0});
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::MULTICORE);
    auto collision = std::dynamic_pointer_cast<chrono::ChCollisionSystemMulticore>(system.GetCollisionSystem());
    ASSERT_NE(collision, nullptr) << "The core factory must not silently fall back to Bullet";
    collision->SetBroadphaseGridResolution({2, 2, 2});
    auto material = std::make_shared<chrono::ChContactMaterialNSC>();
    auto fixed = std::make_shared<chrono::ChBodyEasySphere>(.5, 1000, false, true, material);
    auto moving = std::make_shared<chrono::ChBodyEasySphere>(.5, 1000, false, true, material);
    fixed->SetFixed(true);
    moving->SetPos({.99, 0, 0});
    system.AddBody(fixed);
    system.AddBody(moving);
    ASSERT_TRUE(system.DoStepDynamics(1e-4));
    EXPECT_GT(system.GetNumContacts(), 0u);
    EXPECT_TRUE(std::isfinite(moving->GetPos().x()));
    chrono::ChCollisionSystem::ChRayhitResult result;
    ASSERT_TRUE(collision->RayHit({-2, 0, 0}, {2, 0, 0}, result));
    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.abs_hitPoint.x(), -.5, 1e-7);
}

TEST(MulticoreProfile, SmcRetainsItsOwnStepAndFiniteFreeFall) {
    chrono::ChSystemMulticoreSMC system;
    system.SetNumThreads(2);
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::MULTICORE);
    system.SetGravitationalAcceleration({0, 0, -9.81});
    auto body = std::make_shared<chrono::ChBody>();
    body->SetMass(1);
    body->SetInertiaXX({1, 1, 1});
    body->SetPos({0, 0, 1});
    system.AddBody(body);
    constexpr double dt = 1e-5;
    for (unsigned i = 0; i < 100; ++i) {
        ASSERT_TRUE(system.DoStepDynamics(dt, false));
    }
    EXPECT_NEAR(system.GetChTime(), .001, 1e-15);
    EXPECT_NEAR(body->GetPosDt().z(), -.00981, 1e-12);
    EXPECT_NEAR(body->GetPos().z(), 1 - .5 * 9.81e-6, 1e-7);
    EXPECT_EQ(system.GetContactMethod(), chrono::ChContactMethod::SMC);
}
}  // namespace
