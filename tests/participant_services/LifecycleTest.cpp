#include "Fixture.h"
#include "chrono/physics/ChConveyor.h"
#include "chrono/physics/ChLinkMotorRotationDriveline.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "tests/archive_support/StreamArchive.h"

#include <vector>

namespace robodyna::tests::participant_services {

TEST(ParticipantServicesBaseline, CopiesDetachButInheritedAssignmentAndShaftCopyRetainParent) {
    chrono::ChSystemSMC first;
    chrono::ChSystemSMC second;
    first.SetGravitationalAcceleration({1, 2, 3});
    second.SetGravitationalAcceleration({4, 5, 6});
    chrono::ChBody body;
    body.SetMass(2);
    body.SetSystem(&first);
    chrono::ChBody copied(body);
    EXPECT_EQ(copied.GetSystem(), nullptr);
    ExpectVector(BodyResidual(copied), 0, {0, 0, 0});
    chrono::ChBody assigned;
    assigned.SetSystem(&second);
    assigned = body;
    EXPECT_EQ(assigned.GetSystem(), &first);
    ExpectVector(BodyResidual(assigned), 0, {2, 4, 6});

    chrono::fea::ChMesh mesh;
    mesh.SetSystem(&first);
    chrono::fea::ChMesh copied_mesh(mesh);
    EXPECT_EQ(copied_mesh.GetSystem(), nullptr);
    chrono::fea::ChMesh assigned_mesh;
    assigned_mesh.SetSystem(&second);
    assigned_mesh = mesh;
    EXPECT_EQ(assigned_mesh.GetSystem(), &first);

    chrono::ChShaft shaft;
    shaft.SetSystem(&first);
    chrono::ChShaft copied_shaft(shaft);
    EXPECT_EQ(copied_shaft.GetSystem(), &first);
}

TEST(ParticipantServicesBaseline, SystemCopyUsesItsOwnGravityAfterRebinding) {
    chrono::ChSystemSMC original;
    original.SetGravitationalAcceleration({1, 2, 3});
    chrono::ChSystemSMC copy(original);
    copy.SetGravitationalAcceleration({4, 5, 6});
    chrono::ChBody body;
    body.SetMass(2);
    body.SetSystem(&copy);
    ExpectVector(BodyResidual(body), 0, {8, 10, 12});
    original.SetGravitationalAcceleration({-1, -2, -3});
    ExpectVector(BodyResidual(body), 0, {8, 10, 12});
    body.SetSystem(&original);
    ExpectVector(BodyResidual(body), 0, {-2, -4, -6});
}

TEST(ParticipantServicesBaseline, CompositeSetterPropagatesParentAndNullToOwnedChildren) {
    chrono::ChSystemSMC first;
    chrono::ChSystemSMC second;
    chrono::ChConveyor conveyor;
    chrono::ChLinkMotorRotationDriveline driveline;
    for (chrono::ChSystem* owner : {static_cast<chrono::ChSystem*>(&first),
                                   static_cast<chrono::ChSystem*>(&second),
                                   static_cast<chrono::ChSystem*>(nullptr)}) {
        conveyor.SetSystem(owner);
        driveline.SetSystem(owner);
        EXPECT_EQ(conveyor.GetSystem(), owner);
        EXPECT_EQ(conveyor.GetTruss()->GetSystem(), owner);
        EXPECT_EQ(conveyor.GetPlate()->GetSystem(), owner);
        EXPECT_EQ(driveline.GetSystem(), owner);
        EXPECT_EQ(driveline.GetInnerShaft1()->GetSystem(), owner);
        EXPECT_EQ(driveline.GetInnerShaft2()->GetSystem(), owner);
    }
}

TEST(ParticipantServicesBaseline, ArchiveReadIntoAttachedBasePreservesParentAndOffsets) {
    chrono::ChSystemSMC system;
    chrono::ChPhysicsItem original;
    original.SetName("archived participant");
    original.Update(.375, chrono::UpdateFlags::UPDATE_ALL);
    original.SetOffset_x(1);
    original.SetOffset_w(2);
    original.SetOffset_L(3);
    const auto bytes = archive_test::Write<chrono::ChArchiveOutJSON>(original);

    chrono::ChPhysicsItem restored;
    restored.SetSystem(&system);
    restored.SetOffset_x(11);
    restored.SetOffset_w(12);
    restored.SetOffset_L(13);
    archive_test::Read<chrono::ChArchiveInJSON>(bytes, restored);
    EXPECT_EQ(restored.GetSystem(), &system);
    EXPECT_EQ(restored.GetOffset_x(), 11u);
    EXPECT_EQ(restored.GetOffset_w(), 12u);
    EXPECT_EQ(restored.GetOffset_L(), 13u);
    EXPECT_EQ(restored.GetName(), "archived participant");
    EXPECT_DOUBLE_EQ(restored.GetChTime(), .375);
    EXPECT_DOUBLE_EQ(system.GetChTime(), 0);
}

TEST(ParticipantServicesBaseline, OwnerDestructionDetachesSurvivingBodyAndMesh) {
    auto body = std::make_shared<chrono::ChBody>();
    auto mesh = std::make_shared<chrono::fea::ChMesh>();
    {
        chrono::ChSystemSMC system;
        system.AddBody(body);
        system.AddMesh(mesh);
        ASSERT_EQ(body->GetSystem(), &system);
        ASSERT_EQ(mesh->GetSystem(), &system);
    }
    EXPECT_EQ(body->GetSystem(), nullptr);
    EXPECT_EQ(mesh->GetSystem(), nullptr);
    ExpectVector(BodyResidual(*body), 0, {0, 0, 0});
}

class InitialOwnerNode : public chrono::fea::ChNodeFEAxyz {
  public:
    explicit InitialOwnerNode(const chrono::ChVector3d& position) : ChNodeFEAxyz(position) {}
    void SetupInitial(chrono::ChSystem* owner) override {
        owners.push_back(owner);
        ChNodeFEAxyz::SetupInitial(owner);
    }
    std::vector<chrono::ChSystem*> owners;
};

class InitialOwnerSpring : public chrono::fea::ChElementSpring {
  public:
    // The inherited spring setup hook is empty. Observe the existing virtual
    // extension point without replacing any force or matrix calculation.
    void SetupInitial(chrono::ChSystem* owner) override { owners.push_back(owner); }
    std::vector<chrono::ChSystem*> owners;
};

TEST(ParticipantServicesBaseline, SetupHooksKeepOriginalOwnerAndFixedNodeAdmission) {
    chrono::ChSystemSMC first;
    chrono::ChSystemSMC second;
    mesh_compat::Configure(first);
    mesh_compat::Configure(second);
    auto mesh = std::make_shared<chrono::fea::ChMesh>();
    auto fixed = std::make_shared<InitialOwnerNode>(chrono::ChVector3d(0, 0, 0));
    auto moving = std::make_shared<InitialOwnerNode>(chrono::ChVector3d(1, 0, 0));
    fixed->SetFixed(true);
    moving->SetMass(2);
    auto spring = std::make_shared<InitialOwnerSpring>();
    spring->SetNodes(fixed, moving);
    mesh->AddNode(fixed);
    mesh->AddNode(moving);
    mesh->AddElement(spring);
    first.AddMesh(mesh);
    first.Update(0, chrono::UpdateFlags::UPDATE_ALL);
    ASSERT_EQ(moving->owners, std::vector<chrono::ChSystem*>({&first}));
    ASSERT_EQ(spring->owners, std::vector<chrono::ChSystem*>({&first}));
    EXPECT_TRUE(fixed->owners.empty());
    first.RemoveMesh(mesh);
    EXPECT_EQ(mesh->GetSystem(), nullptr);
    second.AddMesh(mesh);
    second.Update(0, chrono::UpdateFlags::UPDATE_ALL);
    EXPECT_EQ(moving->owners, std::vector<chrono::ChSystem*>({&first, &second}));
    EXPECT_EQ(spring->owners, std::vector<chrono::ChSystem*>({&first, &second}));
    EXPECT_TRUE(fixed->owners.empty());
}

}  // namespace robodyna::tests::participant_services
