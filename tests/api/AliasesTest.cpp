#include "robodyna/contact/RbContactMaterial.h"
#include "robodyna/contact/RbContactMaterialNSC.h"
#include "robodyna/contact/RbContactMaterialSMC.h"
#include "robodyna/core/RbFrame.h"
#include "robodyna/core/RbMatrix33.h"
#include "robodyna/core/RbQuaternion.h"
#include "robodyna/core/RbTypes.h"
#include "robodyna/core/RbVector3.h"
#include "robodyna/fea/RbElementSpring.h"
#include "robodyna/fea/RbMesh.h"
#include "robodyna/fea/RbNodeFEAxyz.h"
#include "robodyna/mbd/RbBody.h"
#include "robodyna/mbd/RbBodyAuxRef.h"
#include "robodyna/mbd/RbBodyEasy.h"
#include "robodyna/mbd/RbLinkTSDA.h"
#include "robodyna/mechanics/RbBodyFrame.h"
#include "robodyna/mechanics/RbPhysicsItem.h"
#include "robodyna/numerics/RbSolver.h"
#include "robodyna/numerics/RbSolverMINRES.h"
#include "robodyna/numerics/RbSolverPSOR.h"
#include "robodyna/numerics/RbTimestepper.h"
#include "robodyna/numerics/RbTimestepperEulerImplicit.h"
#include "robodyna/simulation/RbAssembly.h"
#include "robodyna/simulation/RbSystem.h"
#include "robodyna/simulation/RbSystemNSC.h"
#include "robodyna/simulation/RbSystemSMC.h"

// Intentional legacy consumer: compatibility is part of this test's contract.
#include "chrono/core/ChClassFactory.h"
#include "chrono/physics/ChBody.h"
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <type_traits>

namespace rd = robodyna;

static_assert(std::is_same_v<rd::core::RbVector3<>, chrono::ChVector3d>);
static_assert(std::is_same_v<rd::core::RbVector3<float>, chrono::ChVector3f>);
static_assert(std::is_same_v<rd::core::RbQuaternion<>, chrono::ChQuaterniond>);
static_assert(std::is_same_v<rd::core::RbFrame<>, chrono::ChFrame<>>);
static_assert(std::is_same_v<rd::core::RbMatrix33<>, chrono::ChMatrix33<>>);
static_assert(std::is_same_v<rd::mechanics::RbBodyFrame, chrono::ChBodyFrame>);
static_assert(std::is_same_v<rd::mechanics::RbPhysicsItem, chrono::ChPhysicsItem>);
static_assert(std::is_same_v<rd::mbd::RbBody, chrono::ChBody>);
static_assert(std::is_same_v<rd::mbd::RbBodyAuxRef, chrono::ChBodyAuxRef>);
static_assert(std::is_same_v<rd::mbd::RbBodyEasyBox, chrono::ChBodyEasyBox>);
static_assert(std::is_same_v<rd::mbd::RbBodyEasySphere, chrono::ChBodyEasySphere>);
static_assert(std::is_same_v<rd::mbd::RbLinkTSDA, chrono::ChLinkTSDA>);
static_assert(std::is_same_v<rd::fea::RbMesh, chrono::fea::ChMesh>);
static_assert(std::is_same_v<rd::fea::RbNodeFEAxyz, chrono::fea::ChNodeFEAxyz>);
static_assert(std::is_same_v<rd::fea::RbElementSpring, chrono::fea::ChElementSpring>);
static_assert(std::is_same_v<rd::simulation::RbSystem, chrono::ChSystem>);
static_assert(std::is_same_v<rd::simulation::RbSystemNSC, chrono::ChSystemNSC>);
static_assert(std::is_same_v<rd::simulation::RbSystemSMC, chrono::ChSystemSMC>);
static_assert(std::is_same_v<rd::simulation::RbAssembly, chrono::ChAssembly>);
static_assert(std::is_same_v<rd::contact::RbContactMaterial, chrono::ChContactMaterial>);
static_assert(std::is_same_v<rd::contact::RbContactMaterialNSC, chrono::ChContactMaterialNSC>);
static_assert(std::is_same_v<rd::contact::RbContactMaterialSMC, chrono::ChContactMaterialSMC>);
static_assert(std::is_same_v<rd::numerics::RbSolver, chrono::ChSolver>);
static_assert(std::is_same_v<rd::numerics::RbSolverPSOR, chrono::ChSolverPSOR>);
static_assert(std::is_same_v<rd::numerics::RbSolverMINRES, chrono::ChSolverMINRES>);
static_assert(std::is_same_v<rd::numerics::RbTimestepper, chrono::ChTimestepper>);
static_assert(std::is_same_v<rd::numerics::RbTimestepperEulerImplicitLinearized,
                             chrono::ChTimestepperEulerImplicitLinearized>);

TEST(RobodynaAliases, SharedPointersAndVirtualClonesKeepOneNativeType) {
    auto body = rd::core::make_shared<rd::mbd::RbBody>();
    body->SetMass(2.5);
    body->SetPos({1, 2, 3});
    std::shared_ptr<chrono::ChBody> legacy_body = body;
    std::shared_ptr<rd::mechanics::RbPhysicsItem> participant = legacy_body;
    EXPECT_EQ(std::dynamic_pointer_cast<rd::mbd::RbBody>(participant).get(), body.get());
    EXPECT_EQ(std::dynamic_pointer_cast<rd::mechanics::RbBodyFrame>(participant).get(),
              static_cast<rd::mechanics::RbBodyFrame*>(body.get()));
    legacy_body->SetMass(4.25);
    EXPECT_DOUBLE_EQ(body->GetMass(), 4.25);
    std::unique_ptr<rd::mechanics::RbPhysicsItem> clone(participant->Clone());
    auto* cloned_body = dynamic_cast<rd::mbd::RbBody*>(clone.get());
    ASSERT_NE(cloned_body, nullptr);
    EXPECT_NE(cloned_body, body.get());
    EXPECT_DOUBLE_EQ(cloned_body->GetMass(), 4.25);
    EXPECT_DOUBLE_EQ((cloned_body->GetPos() - body->GetPos()).Length2(), 0);
}

TEST(RobodynaAliases, LegacyFactoryConstructsCanonicalBody) {
    rd::mbd::RbBody* raw = nullptr;
    chrono::ChClassFactory::create("ChBody", &raw);
    std::unique_ptr<rd::mbd::RbBody> body(raw);
    ASSERT_NE(body, nullptr);
    body->SetMass(3.5);
    EXPECT_DOUBLE_EQ(body->GetMass(), 3.5);
}

TEST(RobodynaAliases, MathOperatorsRetainAdlAndTransforms) {
    const rd::core::RbVector3d x(1, 0, 0);
    const rd::core::RbVector3d y(0, 1, 0);
    // Unqualified calls exercise ADL through the canonical facade types.
    EXPECT_DOUBLE_EQ(Vdot(x, y), 0);
    EXPECT_DOUBLE_EQ(Vcross(x, y).z(), 1);
    const rd::core::RbQuaterniond identity(1, 0, 0, 0);
    const rd::core::RbFrame<> frame({2, -1, 3}, identity);
    const auto world = frame.TransformPointLocalToParent(x + y);
    EXPECT_DOUBLE_EQ(world.x(), 3);
    EXPECT_DOUBLE_EQ(world.y(), 0);
    EXPECT_DOUBLE_EQ(world.z(), 3);
    EXPECT_DOUBLE_EQ((frame.TransformPointParentToLocal(world) - x - y).Length2(), 0);
}

TEST(RobodynaAliases, EasyBodyMassAndContactLawIdentityArePreserved) {
    rd::mbd::RbBodyEasyBox box(1, 2, 3, 4, false, false);
    EXPECT_DOUBLE_EQ(box.GetMass(), 24);
    EXPECT_DOUBLE_EQ(box.GetInertiaXX().x(), 26);
    EXPECT_DOUBLE_EQ(box.GetInertiaXX().y(), 20);
    EXPECT_DOUBLE_EQ(box.GetInertiaXX().z(), 10);
    rd::mbd::RbBodyEasySphere sphere(2, 3, false, false);
    EXPECT_NEAR(sphere.GetMass(), 32 * std::acos(-1.0), 1e-12);
    auto nsc = rd::core::make_shared<rd::contact::RbContactMaterialNSC>();
    auto smc = rd::core::make_shared<rd::contact::RbContactMaterialSMC>();
    std::shared_ptr<rd::contact::RbContactMaterial> material = nsc;
    EXPECT_EQ(material->GetContactMethod(), rd::contact::RbContactMethod::NSC);
    material = smc;
    EXPECT_EQ(material->GetContactMethod(), rd::contact::RbContactMethod::SMC);
}

TEST(RobodynaAliases, SolverAndTimestepperRemainNativeObjects) {
    rd::simulation::RbSystemSMC system;
    system.SetNumThreads(1, 1, 1);
    auto solver = rd::core::make_shared<rd::numerics::RbSolverMINRES>();
    solver->SetMaxIterations(80);
    solver->SetTolerance(1e-12);
    system.SetSolver(solver);
    auto stepper = rd::core::make_shared<rd::numerics::RbTimestepperEulerImplicitLinearized>(&system);
    system.SetTimestepper(stepper);
    EXPECT_EQ(system.GetSolver().get(), solver.get());
    EXPECT_EQ(system.GetTimestepper().get(), stepper.get());
    EXPECT_EQ(solver->GetMaxIterations(), 80);
    EXPECT_EQ(stepper->GetType(), rd::numerics::RbTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
}
