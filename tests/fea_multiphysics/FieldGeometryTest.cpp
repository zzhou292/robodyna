#include <gtest/gtest.h>

#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/multiphysics/ChBuilderVolume.h"
#include "chrono/fea/multiphysics/ChFEModelThermoDeformation.h"
#include "chrono/physics/ChContactMaterialSMC.h"

#ifndef CHRONO_FEA_MULTIPHYSICS
#error "The field admission test requires the coherent multiphysics build profile"
#endif

namespace {
using namespace chrono;
using namespace chrono::fea;

TEST(MultiphysicsField, SharedGeometryIndependentFieldsAndFixedDofs) {
    auto temperature = chrono_types::make_shared<ChFieldTemperature>();
    auto displacement = chrono_types::make_shared<ChFieldDisplacement3D>();
    auto model = chrono_types::make_shared<ChFEModelThermoDeformation>(temperature, displacement);
    ChBuilderVolumeBox builder;
    builder.BuildVolume(ChFrame<>(ChVector3d(1, 2, 3)), 1, 1, 1, 2, 4, 6);
    builder.AddToModel(model);
    ASSERT_EQ(builder.nodes.list().size(), 8u);
    ASSERT_EQ(builder.elements.list().size(), 1u);
    EXPECT_EQ(temperature->GetNumNodes(), 8u);
    EXPECT_EQ(displacement->GetNumNodes(), 8u);
    EXPECT_EQ(model->GetNumFields(), 2);
    EXPECT_TRUE(temperature->IsFirstOrderField());
    EXPECT_FALSE(displacement->IsFirstOrderField());
    const auto node = builder.nodes.at(0, 0, 0);
    EXPECT_EQ(model->GetField(0).get(), temperature.get());
    EXPECT_EQ(model->GetField(1).get(), displacement.get());
    EXPECT_NEAR((displacement->NodeData(node).GetPos() - ChVector3d(1, 2, 3)).Length(), 0, 1e-14);
    displacement->NodeData(node).SetFixed(true);
    temperature->Setup();
    displacement->Setup();
    EXPECT_EQ(temperature->GetNumCoordsPosLevel(), 8u);
    EXPECT_EQ(displacement->GetNumCoordsPosLevel(), 21u);
    EXPECT_FALSE(temperature->NodeData(node).IsFixed());
}

TEST(MultiphysicsField, HexahedronAffineInterpolationAndPhysicalJacobian) {
    ChBuilderVolumeBox builder;
    builder.BuildVolume(ChFrame<>(ChVector3d(1, 2, 3)), 1, 1, 1, 2, 4, 6);
    auto element = builder.elements.at(0, 0, 0);
    const ChVector3d eta(0.2, -0.5, 0.7);
    ChRowVectorDynamic<> shape;
    element->ComputeN(eta, shape);
    ASSERT_EQ(shape.size(), 8);
    EXPECT_NEAR(shape.sum(), 1, 1e-14);
    ChVector3d point(0, 0, 0);
    for (int i = 0; i < 8; ++i)
        point += shape(i) * element->GetHexahedronNode(i)->GetReferencePos();
    EXPECT_NEAR((point - ChVector3d(2.2, 3.0, 8.1)).Length(), 0, 1e-13);
    ChMatrix33d jacobian;
    EXPECT_NEAR(element->ComputeJ(eta, jacobian), 6, 1e-13);
    ChMatrixDynamic<> derivatives;
    element->ComputedNdX(eta, derivatives);
    ChVectorDynamic<> affine_values(8);
    for (int i = 0; i < 8; ++i) {
        auto position = element->GetHexahedronNode(i)->GetReferencePos();
        affine_values(i) = 2 * position.x() - 3 * position.y() + 4 * position.z() + 5;
    }
    EXPECT_NEAR((derivatives * affine_values - ChVector3d(2, -3, 4).eigen()).norm(), 0, 1e-13);
}

TEST(MultiphysicsField, ContactProxyUsesLiveFieldAndKeepsLegacyNodes) {
    auto material = chrono_types::make_shared<ChContactMaterialSMC>();
    ChContactSurfaceNodeCloud cloud(material);
    auto legacy_node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(-1, 0, 0));
    cloud.AddNode(legacy_node, 0.01);
    auto field_node = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(1, 2, 3));
    auto displacement = chrono_types::make_shared<ChFieldDisplacement3D>();
    displacement->AddNode(field_node);
    cloud.AddNode(field_node, displacement, 0.01);
    ASSERT_EQ(cloud.GetNodes().size(), 1u);
    ASSERT_EQ(cloud.GetNodesField().size(), 1u);
    EXPECT_EQ(cloud.GetNode(0)->GetNode(), legacy_node.get());
    displacement->NodeData(field_node).SetPos(ChVector3d(4, 5, 6));
    EXPECT_NEAR((cloud.GetNodesField()[0]->GetPos() - ChVector3d(4, 5, 6)).Length(), 0, 1e-14);
    EXPECT_NEAR((field_node->GetReferencePos() - ChVector3d(1, 2, 3)).Length(), 0, 1e-14);
    // This is layout/ownership admission, not qualification of the inherited
    // field-contact Jacobian path documented in this module's runtime limits.
}
}  // namespace
