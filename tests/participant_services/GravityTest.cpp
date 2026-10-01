#include "Fixture.h"
#include "chrono/fea/ChContinuumMaterial.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

#include <array>

namespace robodyna::tests::participant_services {

TEST(ParticipantServicesBaseline, BodyGravityFollowsCurrentParentAndDetachment) {
    chrono::ChSystemSMC first;
    chrono::ChSystemSMC second;
    first.SetGravitationalAcceleration({1, -2, 3});
    second.SetGravitationalAcceleration({-4, 5, 6});
    chrono::ChBody body;
    body.SetMass(2);

    ExpectVector(BodyResidual(body), 0, {0, 0, 0});
    body.SetSystem(&first);
    ExpectVector(BodyResidual(body), 0, {2, -4, 6});
    first.SetGravitationalAcceleration({3, 2, -1});
    ExpectVector(BodyResidual(body), 0, {6, 4, -2});
    body.SetSystem(&second);
    ExpectVector(BodyResidual(body), 0, {-8, 10, 12});
    body.SetSystem(nullptr);
    ExpectVector(BodyResidual(body), 0, {0, 0, 0});
}

TEST(ParticipantServicesBaseline, MeshGravityRespectsFixedAndRotationalNodeOffsets) {
    chrono::ChSystemSMC system;
    mesh_compat::Configure(system);
    auto mesh = std::make_shared<chrono::fea::ChMesh>();
    auto fixed = std::make_shared<chrono::fea::ChNodeFEAxyz>();
    fixed->SetFixed(true);
    fixed->SetMass(100);
    auto xyz = std::make_shared<chrono::fea::ChNodeFEAxyz>();
    xyz->SetMass(2);
    auto rotating = std::make_shared<chrono::fea::ChNodeFEAxyzrot>();
    rotating->SetMass(3);
    mesh->AddNode(fixed);
    mesh->AddNode(xyz);
    mesh->AddNode(rotating);
    system.AddMesh(mesh);
    system.Setup();
    system.Update(0, chrono::UpdateFlags::UPDATE_ALL);
    ASSERT_EQ(mesh->GetNumCoordsVelLevel(), 9u);
    ASSERT_EQ(xyz->NodeGetOffsetVelLevel(), 0u);
    ASSERT_EQ(rotating->NodeGetOffsetVelLevel(), 3u);
    for (const chrono::ChVector3d gravity : {chrono::ChVector3d(1, -2, 3), chrono::ChVector3d(-4, 5, -6)}) {
        system.SetGravitationalAcceleration(gravity);
        const auto residual = MeshResidual(*mesh);
        ExpectVector(residual, 0, gravity * 2);
        ExpectVector(residual, 3, gravity * 3);
        ExpectVector(residual, 6, {0, 0, 0});
    }
    mesh->SetAutomaticGravity(false);
    EXPECT_DOUBLE_EQ(MeshResidual(*mesh).norm(), 0);
}

TEST(ParticipantServicesBaseline, RealTetrahedronGravityResamplesOwnerAndPreservesAssembly) {
    chrono::ChSystemSMC system;
    mesh_compat::Configure(system);
    auto mesh = std::make_shared<chrono::fea::ChMesh>();
    const std::array<chrono::ChVector3d, 4> positions{{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<std::shared_ptr<chrono::fea::ChNodeFEAxyz>, 4> nodes;
    for (unsigned i = 0; i < nodes.size(); ++i) {
        nodes[i] = std::make_shared<chrono::fea::ChNodeFEAxyz>(positions[i]);
        nodes[i]->SetMass(0);  // Element gravity only; no added point masses.
        mesh->AddNode(nodes[i]);
    }
    auto material = std::make_shared<chrono::fea::ChContinuumElastic>();
    material->SetDensity(6);  // Unit tetrahedron volume 1/6 gives total mass 1.
    material->SetYoungModulus(1000);
    material->SetPoissonRatio(.25);
    auto tetrahedron = std::make_shared<chrono::fea::ChElementTetraCorot_4>();
    tetrahedron->SetNodes(nodes[0], nodes[1], nodes[2], nodes[3]);
    tetrahedron->SetMaterial(material);
    mesh->AddElement(tetrahedron);
    system.AddMesh(mesh);
    system.Setup();
    system.Update(0, chrono::UpdateFlags::UPDATE_ALL);
    ASSERT_NEAR(tetrahedron->GetVolume(), 1. / 6., 1e-15);
    for (const chrono::ChVector3d gravity : {chrono::ChVector3d(1, -2, 3), chrono::ChVector3d(-4, 5, -6)}) {
        system.SetGravitationalAcceleration(gravity);
        const auto residual = MeshResidual(*mesh);
        for (unsigned i = 0; i < nodes.size(); ++i)
            ExpectVector(residual, i * 3, gravity * .25);
    }
    mesh->SetAutomaticGravity(false);
    EXPECT_LT(MeshResidual(*mesh).norm(), 1e-12);
}

}  // namespace robodyna::tests::participant_services
