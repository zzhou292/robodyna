#include "FamilySupport.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

namespace robodyna::body_family_test {
TEST(BodyFamilyConstructors, ConvexHullBothOverloadsAndCenteredTopology) {
    auto plain = std::make_shared<chrono::ChBodyEasyConvexHull>(BoxVertices(),3,false,false);
    auto attached = std::make_shared<chrono::ChBodyEasyConvexHull>(BoxVertices(),3,
                                                               std::make_shared<chrono::ChContactMaterialNSC>());
    for (const auto& body : {plain,attached}) {
        ExpectIdentity(body,"ChBodyEasyConvexHull");
        ExpectMassInertia(*body,144,{624,480,240});
        ASSERT_NE(body->GetMesh(),nullptr);
        EXPECT_EQ(body->GetMesh()->GetCoordsVertices().size(),8u);
        EXPECT_EQ(body->GetMesh()->GetNumTriangles(),12u);
        chrono::ChVector3d mean(0);
        for (const auto& vertex : body->GetMesh()->GetCoordsVertices()) mean += vertex;
        EXPECT_LT(mean.Length(),1e-8);
    }
    ExpectAttachments(*plain,0); ExpectAttachments(*attached,1);
}

TEST(BodyFamilyConstructors, AuxiliaryHullBothOverloadsAndReferenceTopology) {
    auto plain = std::make_shared<chrono::ChBodyEasyConvexHullAuxRef>(BoxVertices(),3,false,false);
    auto attached = std::make_shared<chrono::ChBodyEasyConvexHullAuxRef>(BoxVertices(),3,
                                                                     std::make_shared<chrono::ChContactMaterialNSC>());
    for (const auto& body : {plain,attached}) {
        ExpectIdentity(body,"ChBodyEasyConvexHullAuxRef");
        ExpectAuxBox(*body);
        ASSERT_NE(body->GetMesh(),nullptr);
        EXPECT_EQ(body->GetMesh()->GetCoordsVertices().size(),8u);
        EXPECT_EQ(body->GetMesh()->GetNumTriangles(),12u);
        chrono::ChVector3d mean(0);
        for (const auto& vertex : body->GetMesh()->GetCoordsVertices()) mean += vertex;
        EXPECT_LT((mean/8 - chrono::ChVector3d(3,5,7)).Length(),1e-8);
    }
    ExpectAttachments(*plain,0); ExpectAttachments(*attached,1);
}

TEST(BodyFamilyConstructors, TriangleMeshAllFourOverloads) {
    auto plain_mesh = std::make_shared<chrono::ChBodyEasyMesh>(BoxMesh(),3,true,false,false);
    auto plain_file = std::make_shared<chrono::ChBodyEasyMesh>(box_file,3,true,false,false);
    auto material = std::make_shared<chrono::ChContactMaterialNSC>();
    auto attached_mesh = std::make_shared<chrono::ChBodyEasyMesh>(BoxMesh(),3,material,.001);
    auto attached_file = std::make_shared<chrono::ChBodyEasyMesh>(box_file,3,material,.001);
    for (const auto& body : {plain_mesh,plain_file,attached_mesh,attached_file}) {
        ExpectIdentity(body,"ChBodyEasyMesh");
        ExpectAuxBox(*body);
    }
    ExpectAttachments(*plain_mesh,0); ExpectAttachments(*plain_file,0);
    ExpectAttachments(*attached_mesh,1); ExpectAttachments(*attached_file,1);
    for (const auto& body : {attached_mesh,attached_file}) {
        auto shape = std::dynamic_pointer_cast<chrono::ChVisualShapeTriangleMesh>(body->GetVisualShape(0));
        ASSERT_NE(shape,nullptr);
        EXPECT_EQ(shape->GetMesh()->GetCoordsVertices().size(),8u);
        EXPECT_EQ(shape->GetMesh()->GetNumTriangles(),12u);
    }
}
}  // namespace robodyna::body_family_test
