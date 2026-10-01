#include "robodyna/core/RbTypes.h"
#include "robodyna/core/RbVector3.h"
#include "robodyna/fea/RbElementSpring.h"
#include "robodyna/fea/RbMesh.h"
#include "robodyna/fea/RbNodeFEAxyz.h"
#include "robodyna/mbd/RbBody.h"
#include "robodyna/mbd/RbLinkTSDA.h"
#include "robodyna/numerics/RbSolver.h"
#include "robodyna/numerics/RbTimestepper.h"
#include "robodyna/simulation/RbSystemNSC.h"
#include "robodyna/simulation/RbSystemSMC.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
namespace rd = robodyna;
constexpr double kDt = 1e-4;
constexpr unsigned kSteps = 1000;
using Sample = std::array<double, 4>;  // time, x, vx, signed spring force
using History = std::vector<Sample>;

class LinearForce final : public rd::mbd::RbLinkTSDA::ForceFunctor {
  public:
    double evaluate(double, double rest, double length, double velocity,
                    const rd::mbd::RbLinkTSDA&) override {
        ++calls;
        return 0.0 - 12.0 * (length - rest) - 0.0 * velocity;
    }
    unsigned calls = 0;
};

template <class System, class Body, class Link>
History RigidHistory(const std::shared_ptr<LinearForce>& callback = nullptr) {
    System system;
    system.SetNumThreads(1, 1, 1);
    system.SetGravitationalAcceleration({0, 0, 0});
    system.SetTimestepperType(rd::numerics::RbTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    auto fixed = rd::core::make_shared<Body>();
    auto moving = rd::core::make_shared<Body>();
    fixed->SetFixed(true);
    moving->SetMass(3);
    moving->SetInertiaXX({1, 1, 1});
    moving->SetPos({1.1, 0, 0});
    moving->SetSleepingAllowed(false);
    system.AddBody(fixed);
    system.AddBody(moving);
    auto spring = rd::core::make_shared<Link>();
    spring->Initialize(fixed, moving, true, {0, 0, 0}, {0, 0, 0});
    spring->SetRestLength(1);
    spring->SetSpringCoefficient(12);
    spring->SetDampingCoefficient(0);
    if (callback)
        spring->RegisterForceFunctor(callback);
    system.AddLink(spring);
    History history;
    history.reserve(kSteps);
    for (unsigned step = 0; step < kSteps; ++step) {
        if (!system.DoStepDynamics(kDt, false))
            throw std::runtime_error("Rigid spring failed to advance");
        history.push_back({system.GetChTime(), moving->GetPos().x(),
                           moving->GetPosDt().x(), spring->GetForce()});
    }
    EXPECT_EQ(system.GetVisualSystem(), nullptr);
    EXPECT_EQ(system.GetBodies().size(), 2u);
    return history;
}

template <class System, class Mesh, class Node, class Element>
History FeHistory() {
    System system;
    system.SetNumThreads(1, 1, 1);
    system.SetGravitationalAcceleration({0, 0, 0});
    system.SetTimestepperType(rd::numerics::RbTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    system.SetSolverType(rd::numerics::RbSolver::Type::MINRES);
    auto mesh = rd::core::make_shared<Mesh>();
    auto fixed = rd::core::make_shared<Node>(rd::core::RbVector3d(0, 0, 0));
    auto moving = rd::core::make_shared<Node>(rd::core::RbVector3d(1, 0, 0));
    fixed->SetFixed(true);
    moving->SetMass(3);
    auto spring = rd::core::make_shared<Element>();
    spring->SetNodes(fixed, moving);
    spring->SetSpringCoefficient(12);
    spring->SetDampingCoefficient(0);
    mesh->AddNode(fixed);
    mesh->AddNode(moving);
    mesh->AddElement(spring);
    mesh->SetAutomaticGravity(false);
    system.AddMesh(mesh);
    moving->SetPos({1.1, 0, 0});
    History history;
    history.reserve(kSteps);
    for (unsigned step = 0; step < kSteps; ++step) {
        if (!system.DoStepDynamics(kDt, false))
            throw std::runtime_error("FE spring failed to advance");
        history.push_back({system.GetChTime(), moving->GetPos().x(),
                           moving->GetPosDt().x(), spring->GetCurrentForce()});
    }
    EXPECT_EQ(system.GetVisualSystem(), nullptr);
    EXPECT_EQ(mesh->GetNumNodes(), 2u);
    EXPECT_EQ(mesh->GetNumElements(), 1u);
    EXPECT_GT(mesh->GetNumCallsInternalForces(), 0u);
    return history;
}

void ExpectSameDynamics(const History& canonical, const History& legacy) {
    ASSERT_EQ(canonical.size(), kSteps);
    ASSERT_EQ(canonical.size(), legacy.size());
    for (unsigned step = 0; step < kSteps; ++step) {
        SCOPED_TRACE(step);
        for (unsigned field = 0; field < canonical[step].size(); ++field) {
            EXPECT_TRUE(std::isfinite(canonical[step][field]));
            EXPECT_EQ(canonical[step][field], legacy[step][field]);
        }
    }
    const double time = kSteps * kDt;
    EXPECT_NEAR(canonical.back()[0], time, 1e-13);
    // Both existing first-order integrators have small discretization error.
    EXPECT_NEAR(canonical.back()[1], 1 + .1 * std::cos(2 * time), 3e-6);
    EXPECT_LT(canonical.back()[1], 1.099);
    EXPECT_LT(canonical.back()[2], 0);
}
}  // namespace

TEST(RobodynaDynamics, BodySpringMatchesLegacyStepByStep) {
    const auto canonical = RigidHistory<rd::simulation::RbSystemNSC, rd::mbd::RbBody, rd::mbd::RbLinkTSDA>();
    const auto legacy = RigidHistory<chrono::ChSystemNSC, chrono::ChBody, chrono::ChLinkTSDA>();
    ExpectSameDynamics(canonical, legacy);
}

TEST(RobodynaDynamics, FeSpringMatchesLegacyStepByStep) {
    const auto canonical = FeHistory<rd::simulation::RbSystemSMC, rd::fea::RbMesh,
                                     rd::fea::RbNodeFEAxyz, rd::fea::RbElementSpring>();
    const auto legacy = FeHistory<chrono::ChSystemSMC, chrono::fea::ChMesh,
                                  chrono::fea::ChNodeFEAxyz, chrono::fea::ChElementSpring>();
    ExpectSameDynamics(canonical, legacy);
}

TEST(RobodynaDynamics, CanonicalCallbackDispatchesThroughLegacyLink) {
    const auto callback = rd::core::make_shared<LinearForce>();
    const auto custom = RigidHistory<chrono::ChSystemNSC, chrono::ChBody, chrono::ChLinkTSDA>(callback);
    const auto native = RigidHistory<rd::simulation::RbSystemNSC, rd::mbd::RbBody, rd::mbd::RbLinkTSDA>();
    EXPECT_GT(callback->calls, 0u);
    ExpectSameDynamics(custom, native);
}
