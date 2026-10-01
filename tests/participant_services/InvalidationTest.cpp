#include "Fixture.h"
#include "chrono/physics/ChForce.h"
#include "chrono/physics/ChMarker.h"

namespace robodyna::tests::participant_services {

TEST(ParticipantServicesBaseline, BodyInsertionAndRemovalKeepDifferentInvalidationScopes) {
    chrono::ChSystemSMC system;
    mesh_compat::Configure(system);
    auto body = std::make_shared<chrono::ChBody>();
    system.AddBody(body);
    StepAndCheckPrepared(system);

    auto marker = std::make_shared<chrono::ChMarker>();
    body->AddMarker(marker);
    EXPECT_FALSE(system.IsInitialized());
    EXPECT_FALSE(system.IsUpdated());
    EXPECT_EQ(marker->GetBody(), body.get());
    StepAndCheckPrepared(system);

    body->RemoveMarker(marker);
    EXPECT_TRUE(system.IsInitialized());
    EXPECT_FALSE(system.IsUpdated());
    EXPECT_EQ(marker->GetBody(), nullptr);
    StepAndCheckPrepared(system);

    auto force = std::make_shared<chrono::ChForce>();
    body->AddForce(force);
    EXPECT_FALSE(system.IsInitialized());
    EXPECT_FALSE(system.IsUpdated());
    EXPECT_EQ(force->GetBody(), body.get());
    StepAndCheckPrepared(system);

    body->RemoveForce(force);
    EXPECT_TRUE(system.IsInitialized());
    EXPECT_FALSE(system.IsUpdated());
    EXPECT_EQ(force->GetBody(), nullptr);
    StepAndCheckPrepared(system);
}

TEST(ParticipantServicesBaseline, MeshInsertionAndClearingKeepDifferentInvalidationScopes) {
    chrono::ChSystemSMC system;
    mesh_compat::Configure(system);
    mesh_compat::SpringMesh fixture;
    system.AddMesh(fixture.mesh);
    StepAndCheckPrepared(system);

    auto extra = std::make_shared<chrono::fea::ChNodeFEAxyz>(chrono::ChVector3d(2, 0, 0));
    extra->SetMass(2);
    fixture.mesh->AddNode(extra);
    EXPECT_FALSE(system.IsInitialized());
    EXPECT_FALSE(system.IsUpdated());
    StepAndCheckPrepared(system);
    EXPECT_EQ(fixture.mesh->GetNumCoordsVelLevel(), 6u);

    auto extra_spring = std::make_shared<chrono::fea::ChElementSpring>();
    extra_spring->SetNodes(fixture.moving, extra);
    fixture.mesh->AddElement(extra_spring);
    EXPECT_FALSE(system.IsInitialized());
    EXPECT_FALSE(system.IsUpdated());
    StepAndCheckPrepared(system);

    fixture.mesh->ClearElements();
    EXPECT_TRUE(system.IsInitialized());
    EXPECT_FALSE(system.IsUpdated());
    StepAndCheckPrepared(system);
    EXPECT_EQ(fixture.mesh->GetNumCoordsVelLevel(), 6u);

    fixture.mesh->ClearNodes();
    EXPECT_TRUE(system.IsInitialized());
    EXPECT_FALSE(system.IsUpdated());
    system.Setup();
    EXPECT_EQ(fixture.mesh->GetNumCoordsVelLevel(), 0u);
    EXPECT_EQ(system.GetNumCoordsVelLevel(), 0u);
}

TEST(ParticipantServicesBaseline, DirectReparentingInvalidatesOnlyTheCurrentOwner) {
    chrono::ChSystemSMC first;
    chrono::ChSystemSMC second;
    mesh_compat::Configure(first);
    mesh_compat::Configure(second);
    first.AddBody(std::make_shared<chrono::ChBody>());
    second.AddBody(std::make_shared<chrono::ChBody>());
    StepAndCheckPrepared(first);
    StepAndCheckPrepared(second);

    chrono::ChBody body;
    body.SetSystem(&first);
    body.SetSystem(&second);
    body.SetSystem(&second);
    body.AddMarker(std::make_shared<chrono::ChMarker>());
    EXPECT_TRUE(first.IsInitialized());
    EXPECT_TRUE(first.IsUpdated());
    EXPECT_FALSE(second.IsInitialized());
    EXPECT_FALSE(second.IsUpdated());

    StepAndCheckPrepared(second);
    body.SetSystem(nullptr);
    body.AddForce(std::make_shared<chrono::ChForce>());
    EXPECT_TRUE(second.IsInitialized());
    EXPECT_TRUE(second.IsUpdated());
}

}  // namespace robodyna::tests::participant_services
