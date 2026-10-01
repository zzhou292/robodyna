#pragma once

#include "tests/mesh_compat/Fixture.h"
#include "chrono/physics/ChBody.h"

#include <gtest/gtest.h>

namespace robodyna::tests::participant_services {

inline void StepAndCheckPrepared(chrono::ChSystemSMC& system) {
    ASSERT_TRUE(system.DoStepDynamics(1e-4, false));
    ASSERT_TRUE(system.IsInitialized());
    ASSERT_TRUE(system.IsUpdated());
}

inline chrono::ChVectorDynamic<> BodyResidual(chrono::ChBody& body) {
    body.Update(0, chrono::UpdateFlags::UPDATE_ALL);
    chrono::ChVectorDynamic<> residual(6);
    residual.setZero();
    // RbBody's override is private; the existing participant interface is public.
    static_cast<chrono::ChPhysicsItem&>(body).IntLoadResidual_F(0, residual, 1);
    return residual;
}

inline chrono::ChVectorDynamic<> MeshResidual(chrono::fea::ChMesh& mesh) {
    chrono::ChVectorDynamic<> residual(mesh.GetNumCoordsVelLevel());
    residual.setZero();
    mesh.IntLoadResidual_F(0, residual, 1);
    return residual;
}

inline void ExpectVector(const chrono::ChVectorDynamic<>& actual,
                         unsigned offset,
                         const chrono::ChVector3d& expected,
                         double tolerance = 1e-12) {
    for (unsigned axis = 0; axis < 3; ++axis)
        EXPECT_NEAR(actual(offset + axis), expected[axis], tolerance);
}

}  // namespace robodyna::tests::participant_services
