#include "FamilySupport.h"

namespace robodyna::body_family_test {
TEST(BodyFamilyConstructors, SphereBothOverloads) {
    auto plain = std::make_shared<chrono::ChBodyEasySphere>(1,3,false,false);
    auto attached = std::make_shared<chrono::ChBodyEasySphere>(1,3,std::make_shared<chrono::ChContactMaterialNSC>());
    for (const auto& body : {plain,attached}) {
        ExpectIdentity(body,"ChBodyEasySphere");
        ExpectMassInertia(*body,4*Pi(),chrono::ChVector3d(8*Pi()/5));
    }
    ExpectAttachments(*plain,0); ExpectAttachments(*attached,1);
}

TEST(BodyFamilyConstructors, EllipsoidBothOverloads) {
    auto plain = std::make_shared<chrono::ChBodyEasyEllipsoid>(chrono::ChVector3d(2,4,6),3,false,false);
    auto attached = std::make_shared<chrono::ChBodyEasyEllipsoid>(chrono::ChVector3d(2,4,6),3,
                                                               std::make_shared<chrono::ChContactMaterialNSC>());
    for (const auto& body : {plain,attached}) {
        ExpectIdentity(body,"ChBodyEasyEllipsoid");
        ExpectMassInertia(*body,24*Pi(),{62.4*Pi(),48*Pi(),24*Pi()});
    }
    ExpectAttachments(*plain,0); ExpectAttachments(*attached,1);
}

TEST(BodyFamilyConstructors, CylinderBothOverloadsAndAllAxes) {
    const chrono::ChAxis axes[] = {chrono::ChAxis::X,chrono::ChAxis::Y,chrono::ChAxis::Z};
    const chrono::ChVector3d inertia[] = {{3*Pi(),3.5*Pi(),3.5*Pi()},
                                        {3.5*Pi(),3*Pi(),3.5*Pi()},{3.5*Pi(),3.5*Pi(),3*Pi()}};
    for (unsigned i=0;i<3;++i) {
        auto plain = std::make_shared<chrono::ChBodyEasyCylinder>(axes[i],1,2,3,false,false);
        auto attached = std::make_shared<chrono::ChBodyEasyCylinder>(axes[i],1,2,3,
                                                                  std::make_shared<chrono::ChContactMaterialNSC>());
        for (const auto& body : {plain,attached}) {
            ExpectIdentity(body,"ChBodyEasyCylinder");
            ExpectMassInertia(*body,6*Pi(),inertia[i]);
        }
        ExpectAttachments(*plain,0); ExpectAttachments(*attached,1);
    }
}

TEST(BodyFamilyConstructors, BoxBothOverloads) {
    auto plain = std::make_shared<chrono::ChBodyEasyBox>(2,4,6,3,false,false);
    auto attached = std::make_shared<chrono::ChBodyEasyBox>(2,4,6,3,std::make_shared<chrono::ChContactMaterialNSC>());
    for (const auto& body : {plain,attached}) {
        ExpectIdentity(body,"ChBodyEasyBox");
        ExpectMassInertia(*body,144,{624,480,240});
    }
    ExpectAttachments(*plain,0); ExpectAttachments(*attached,1);
}

TEST(BodyFamilyConstructors, SphereClusterBothOverloads) {
    const std::vector<chrono::ChVector3d> positions{{-2,0,0},{2,0,0}};
    const std::vector<double> radii{1,1};
    auto plain = std::make_shared<chrono::ChBodyEasyClusterOfSpheres>(positions,radii,3,false,false);
    auto attached = std::make_shared<chrono::ChBodyEasyClusterOfSpheres>(positions,radii,3,
                                                                      std::make_shared<chrono::ChContactMaterialNSC>());
    for (const auto& body : {plain,attached}) {
        ExpectIdentity(body,"ChBodyEasyClusterOfSpheres");
        ExpectMassInertia(*body,8*Pi(),{16*Pi()/5,176*Pi()/5,176*Pi()/5});
    }
    ExpectAttachments(*plain,0); ExpectAttachments(*attached,2);
    EXPECT_EQ(attached->GetVisualModel()->GetShapeFrame(0).GetPos(),positions[0]);
    EXPECT_EQ(attached->GetVisualModel()->GetShapeFrame(1).GetPos(),positions[1]);
}
}  // namespace robodyna::body_family_test
