#include "Fixture.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

#include <gtest/gtest.h>

namespace {
using namespace robodyna::tests::mesh_compat;

TEST(MeshCompatibility, FixedNodesHaveOffsetsButConsumeNoActiveCoordinates) {
    chrono::fea::ChMesh mesh;
    auto fixed = chrono_types::make_shared<chrono::fea::ChNodeFEAxyz>();
    auto moving = chrono_types::make_shared<chrono::fea::ChNodeFEAxyz>();
    auto rotating = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzrot>();
    auto fixed_rotating = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzrot>();
    fixed->SetFixed(true);
    fixed_rotating->SetFixed(true);
    mesh.AddNode(fixed);
    mesh.AddNode(moving);
    mesh.AddNode(rotating);
    mesh.AddNode(fixed_rotating);
    mesh.SetOffset_x(5);
    mesh.SetOffset_w(11);
    mesh.Setup();

    EXPECT_EQ(mesh.GetNumCoordsPosLevel(), 10u);
    EXPECT_EQ(mesh.GetNumCoordsVelLevel(), 9u);
    EXPECT_EQ(fixed->NodeGetOffsetPosLevel(), 5u);
    EXPECT_EQ(fixed->NodeGetOffsetVelLevel(), 11u);
    EXPECT_EQ(moving->NodeGetOffsetPosLevel(), 5u);
    EXPECT_EQ(moving->NodeGetOffsetVelLevel(), 11u);
    EXPECT_EQ(rotating->NodeGetOffsetPosLevel(), 8u);
    EXPECT_EQ(rotating->NodeGetOffsetVelLevel(), 14u);
    EXPECT_EQ(fixed_rotating->NodeGetOffsetPosLevel(), 15u);
    EXPECT_EQ(fixed_rotating->NodeGetOffsetVelLevel(), 20u);
    for (unsigned i = 0; i < mesh.GetNumNodes(); ++i)
        EXPECT_EQ(mesh.GetNodes()[i]->GetIndex(), i + 1);

    moving->SetFixed(true);
    mesh.Setup();
    EXPECT_EQ(mesh.GetNumCoordsPosLevel(), 7u);
    EXPECT_EQ(mesh.GetNumCoordsVelLevel(), 6u);
    EXPECT_EQ(rotating->NodeGetOffsetPosLevel(), 5u);
    EXPECT_EQ(rotating->NodeGetOffsetVelLevel(), 11u);
}

TEST(MeshCompatibility, CloneSharesTopologyAndSurfacesButDetachesFromSystem) {
    SpringMesh fixture;
    chrono::ChSystemSMC system;
    Configure(system);
    system.AddMesh(fixture.mesh);
    fixture.moving->SetPos({1.1, 0, 0});
    ASSERT_TRUE(system.DoStepDynamics(1e-4, false));
    ASSERT_GT(fixture.mesh->GetNumCallsInternalForces(), 0u);
    std::unique_ptr<chrono::fea::ChMesh> copy(fixture.mesh->Clone());
    ASSERT_NE(copy, nullptr);
    EXPECT_EQ(copy->GetSystem(), nullptr);
    EXPECT_EQ(fixture.mesh->GetSystem(), &system);
    EXPECT_EQ(copy->GetNode(1), fixture.moving);
    EXPECT_EQ(copy->GetElement(0), fixture.spring);
    EXPECT_EQ(copy->GetContactSurface(0), fixture.contact);
    EXPECT_EQ(copy->GetMeshSurface(0), fixture.surface);
    // The inherited clone shares surfaces. It does not rebind their raw owner.
    EXPECT_EQ(copy->GetContactSurface(0)->GetPhysicsItem(), fixture.mesh.get());
    EXPECT_EQ(copy->GetMeshSurface(0)->GetMesh(), fixture.mesh.get());
    EXPECT_EQ(copy->GetNumCallsInternalForces(), 0u);
    EXPECT_EQ(copy->GetNumCallsJacobianLoad(), 0u);
    EXPECT_FALSE(copy->GetAutomaticGravity());
    EXPECT_DOUBLE_EQ(copy->GetChTime(), fixture.mesh->GetChTime());
    fixture.moving->SetPos({2, 0, 0});
    EXPECT_DOUBLE_EQ(std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyz>(copy->GetNode(1))->GetPos().x(), 2);
}

TEST(MeshCompatibility, ClearElementsRetainsNodesAndLoadSurfaces) {
    SpringMesh fixture;
    ASSERT_EQ(fixture.mesh->GetNumContactSurfaces(), 1u);
    fixture.mesh->ClearElements();
    EXPECT_EQ(fixture.mesh->GetNumElements(), 0u);
    EXPECT_EQ(fixture.mesh->GetNumNodes(), 2u);
    EXPECT_EQ(fixture.mesh->GetNumContactSurfaces(), 0u);
    ASSERT_EQ(fixture.mesh->GetNumMeshSurfaces(), 1u);
    EXPECT_EQ(fixture.mesh->GetMeshSurface(0), fixture.surface);
    EXPECT_EQ(fixture.surface->GetMesh(), fixture.mesh.get());
}

TEST(MeshCompatibility, ClearNodesRetainsLoadSurfacesUntilExplicitlyCleared) {
    SpringMesh fixture;
    fixture.mesh->ClearNodes();
    EXPECT_EQ(fixture.mesh->GetNumNodes(), 0u);
    EXPECT_EQ(fixture.mesh->GetNumElements(), 0u);
    EXPECT_EQ(fixture.mesh->GetNumContactSurfaces(), 0u);
    ASSERT_EQ(fixture.mesh->GetNumMeshSurfaces(), 1u);
    EXPECT_EQ(fixture.mesh->GetMeshSurface(0), fixture.surface);
    fixture.mesh->ClearMeshSurfaces();
    EXPECT_EQ(fixture.mesh->GetNumMeshSurfaces(), 0u);
}

TEST(MeshCompatibility, AddingANodeInvalidatesInitializationAndUpdatesActiveSize) {
    SpringMesh fixture;
    chrono::ChSystemSMC system;
    Configure(system);
    system.AddMesh(fixture.mesh);
    ASSERT_TRUE(system.DoStepDynamics(1e-4, false));
    ASSERT_TRUE(system.IsInitialized());
    EXPECT_EQ(fixture.mesh->GetNumCoordsVelLevel(), 3u);
    auto extra = chrono_types::make_shared<chrono::fea::ChNodeFEAxyz>(chrono::ChVector3d(2, 0, 0));
    extra->SetMass(1);
    fixture.mesh->AddNode(extra);
    EXPECT_FALSE(system.IsInitialized());
    ASSERT_TRUE(system.DoStepDynamics(1e-4, false));
    EXPECT_TRUE(system.IsInitialized());
    EXPECT_EQ(fixture.mesh->GetNumCoordsVelLevel(), 6u);
    EXPECT_EQ(extra->GetIndex(), 3u);
    EXPECT_EQ(extra->NodeGetOffsetVelLevel(), fixture.moving->NodeGetOffsetVelLevel() + 3);
}
}  // namespace
