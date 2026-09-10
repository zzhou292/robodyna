#include "chrono/AcceptedReplayScene.h"
#include "output/AcceptedReplay.h"
#include "output/GuidedExperimentMetadata.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

namespace {
using crash::visual::AcceptedReplayScene;
using crash::visual::ReplaySceneStatus;
using crash::output::ReplayFrame;
using crash::output::ReplayInfo;
std::shared_ptr<chrono::ChTriangleMeshConnected> Mesh(double tip_z = 0) {
    auto mesh = std::make_shared<chrono::ChTriangleMeshConnected>();
    mesh->GetCoordsVertices() = {{0, -0.05, 0}, {0.2, -0.05, tip_z}, {0.2, 0.05, tip_z}, {0, 0.05, 0}};
    mesh->GetIndicesVertices() = {{0, 1, 2}, {0, 2, 3}};
    return mesh;
}
ReplayInfo Info() {
    ReplayInfo info;
    info.kind = crash::output::ReplayKind::ElasticCoupon;
    info.owner_id = 9;
    info.final_epoch = 12;
    info.final_time = 0.02;
    info.frame_count = 3;
    info.node_count = 4;
    info.triangle_count = 2;
    info.bounds_min = {0, -0.05, -0.002};
    info.bounds_max = {0.2, 0.05, 0.002};
    return info;
}
ReplayFrame Frame(std::size_t index, double z) { return {index, 9, 6 * index, 0.01 * index, Mesh(z)}; }
ReplayInfo PlasticInfo() {
    auto info=Info();info.kind=crash::output::ReplayKind::SourcePartWall;
    info.source_plasticity=true;info.node_count=5;info.triangle_count=3;
    info.bounds_min[0]=-.1;info.plastic_strain_color_max=.02;
    info.triangle_source_parent={100,100,200};return info;
}
ReplayFrame PlasticFrame(std::size_t index,double z,double q4,double t3) {
    auto frame=Frame(index,z);auto mesh=Mesh(z);
    mesh->GetCoordsVertices().push_back({-.1,0,0});mesh->GetIndicesVertices().push_back({0,3,4});
    frame.mesh=mesh;
    // Parent order intentionally differs from triangle order; source identity
    // determines colors, never adjacency, element-family order or vertex ID.
    frame.parent_plastic_strain={{200,t3},{100,q4}};return frame;
}
void ColorNear(const chrono::ChColor& actual,float r,float g,float b) {
    EXPECT_NEAR(actual.R,r,2e-7f);EXPECT_NEAR(actual.G,g,2e-7f);EXPECT_NEAR(actual.B,b,2e-7f);
}
void SameColors(const std::vector<chrono::ChColor>& actual,const std::vector<chrono::ChColor>& expected) {
    ASSERT_EQ(actual.size(),expected.size());
    for(std::size_t i=0;i<actual.size();++i) {
        EXPECT_FLOAT_EQ(actual[i].R,expected[i].R);EXPECT_FLOAT_EQ(actual[i].G,expected[i].G);EXPECT_FLOAT_EQ(actual[i].B,expected[i].B);
    }
}

TEST(AcceptedReplayScene, EmptyStateRejectsPublication) {
    AcceptedReplayScene scene;
    EXPECT_EQ(scene.Publish(Frame(1, 0.002)).status, ReplaySceneStatus::NotInitialized);
    EXPECT_THROW(scene.system(), std::logic_error);
    EXPECT_EQ(scene.stamp(), nullptr);
    EXPECT_EQ(scene.camera(), nullptr);
    EXPECT_EQ(scene.moving_shape(), nullptr);
}

TEST(AcceptedReplayScene, ActualMutableGeometryAndNormalsUpdateWithoutRebindingOrInputMutation) {
    AcceptedReplayScene scene;
    auto input = Frame(0, 0);
    ASSERT_EQ(scene.Initialize(Info(), input, {}).status, ReplaySceneStatus::Ok);
    const auto original_input = input.mesh->GetCoordsVertices();
    const auto shape = scene.moving_shape();
    const auto mesh = shape->GetMesh();
    const auto camera = *scene.camera();
    EXPECT_TRUE(shape->IsMutable());
    EXPECT_TRUE(shape->IsFixedConnectivity());
    EXPECT_TRUE(shape->IsDoubleFaced());
    EXPECT_EQ(shape->GetNumMaterials(), 1);
    EXPECT_EQ(scene.system().GetBodies().size(), 1u);
    EXPECT_TRUE(scene.system().GetBodies()[0]->IsFixed());
    EXPECT_FALSE(scene.system().GetBodies()[0]->IsCollisionEnabled());
    const auto before = mesh->GetFaceNormals();
    ASSERT_EQ(scene.Publish(Frame(1, 0.002)).status, ReplaySceneStatus::Ok);
    EXPECT_EQ(scene.moving_shape(), shape);
    EXPECT_EQ(scene.moving_mesh().get(), mesh.get());
    EXPECT_EQ(mesh->GetCoordsVertices()[1].z(), 0.002);
    const auto normal = mesh->GetFaceNormals()[0];
    // Independent analytical normal of z=0.01*x, not the implementation's
    // vertex cross product. Face orientation and actual normal recomputation.
    const double scale = std::sqrt(1.0001);
    EXPECT_NEAR(normal.x(), -0.01 / scale, 1e-14);
    EXPECT_NEAR(normal.y(), 0, 1e-14);
    EXPECT_NEAR(normal.z(), 1 / scale, 1e-14);
    EXPECT_NE(normal.x(), before[0].x());
    EXPECT_EQ(input.mesh->GetCoordsVertices(), original_input);
    EXPECT_EQ(scene.camera()->position, camera.position);
    EXPECT_EQ(scene.camera()->target, camera.target);
    EXPECT_EQ(scene.system().GetChTime(), 0.01);
    ASSERT_EQ(scene.Publish(Frame(2, -0.002)).status, ReplaySceneStatus::Ok);
    EXPECT_GT(mesh->GetFaceNormals()[0].x(), 0);
    EXPECT_EQ(scene.system().GetChTime(), 0.02);
}

TEST(AcceptedReplayScene, LateInvalidFramePreservesGeometryStampAndAllowsCorrectedRetry) {
    AcceptedReplayScene scene;
    ASSERT_EQ(scene.Initialize(Info(), Frame(0, 0), {}).status, ReplaySceneStatus::Ok);
    ASSERT_EQ(scene.Publish(Frame(1, 0.002)).status, ReplaySceneStatus::Ok);
    const auto accepted = scene.moving_mesh()->GetCoordsVertices();
    auto rejected = Frame(2, -0.002);
    auto bad_mesh = Mesh(-0.002);
    bad_mesh->GetCoordsVertices().back().z() = std::numeric_limits<double>::quiet_NaN();
    rejected.mesh = bad_mesh;
    EXPECT_EQ(scene.Publish(rejected).status, ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(), accepted);
    EXPECT_EQ(scene.stamp()->index, 1u);
    EXPECT_EQ(scene.system().GetChTime(), 0.01);
    auto wrong_topology = Mesh(-0.002);
    wrong_topology->GetIndicesVertices()[1] = {0, 1, 3};
    rejected.mesh = wrong_topology;
    EXPECT_EQ(scene.Publish(rejected).status, ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(), accepted);
    auto wrong_owner = Frame(2, -0.002);
    wrong_owner.owner_id = 10;
    EXPECT_EQ(scene.Publish(wrong_owner).status, ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.Publish(Frame(1, 0.002)).status, ReplaySceneStatus::InvalidFrame);
    ASSERT_EQ(scene.Publish(Frame(2, -0.002)).status, ReplaySceneStatus::Ok);
    EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices()[1].z(), -0.002);
}

TEST(AcceptedReplayScene, FixedWallCopiedOnceAndKeptSeparateFromMovingSurface) {
    auto info = Info();
    info.kind = crash::output::ReplayKind::NormalImpact;
    auto wall = Mesh();
    for (auto& p : wall->GetCoordsVertices()) p.z() = -1;
    AcceptedReplayScene scene;
    EXPECT_EQ(scene.Initialize(info, Frame(0, 0), {}).status, ReplaySceneStatus::InvalidFrame);
    ASSERT_EQ(scene.Initialize(info, Frame(0, 0), wall).status, ReplaySceneStatus::Ok);
    const auto bound_wall = scene.wall_mesh();
    const auto wall_positions = bound_wall->GetCoordsVertices();
    EXPECT_NE(bound_wall.get(), wall.get());
    EXPECT_EQ(scene.system().GetBodies().size(), 2u);
    const auto wall_body = scene.system().GetBodies()[1];
    const auto wall_shape = std::dynamic_pointer_cast<chrono::ChVisualShapeTriangleMesh>(wall_body->GetVisualShape(0));
    ASSERT_TRUE(wall_shape);
    EXPECT_TRUE(wall_shape->IsWireframe());
    EXPECT_TRUE(wall_body->GetVisualModel()->UseWireframe(0));
    EXPECT_FALSE(wall_shape->IsMutable());
    EXPECT_TRUE(wall_body->IsFixed());
    EXPECT_FALSE(wall_body->IsCollisionEnabled());
    EXPECT_EQ(wall_body->GetPos(), chrono::VNULL);
    EXPECT_EQ(wall_body->GetRot(), chrono::QUNIT);
    ASSERT_EQ(scene.Publish(Frame(1, 0.002)).status, ReplaySceneStatus::Ok);
    EXPECT_EQ(scene.wall_mesh(), bound_wall);
    EXPECT_EQ(scene.wall_mesh()->GetCoordsVertices(), wall_positions);
    EXPECT_EQ(wall_body->GetVisualShape(0), wall_shape);
    EXPECT_EQ(scene.Initialize(info, Frame(0, 0), wall).status, ReplaySceneStatus::AlreadyInitialized);
}

TEST(AcceptedReplayScene, GuidedPlateRequiresWallAndUsesFixedObliquePhysicalView) {
    // Experiment admission belongs to the reader; both accepted named
    // variants use identical physical-scale scene/publication operations.
    for(const char* name:{crash::output::guided_experiment_metadata::Original,
                         crash::output::guided_experiment_metadata::PenaltyMargin}) {
    SCOPED_TRACE(name);
    auto info = Info(); info.kind = crash::output::ReplayKind::GuidedPlate;
    info.guided_experiment=name;
    info.bounds_min = {.045,-.1,-.05}; info.bounds_max = {.052,.1,.05};
    auto initial = Mesh();
    for (auto& p : initial->GetCoordsVertices()) p = {.045,p.x()-.1,p.y()};
    auto wall = std::make_shared<chrono::ChTriangleMeshConnected>(*initial);
    for (auto& p : wall->GetCoordsVertices()) p.x()=.05;
    ReplayFrame first{0,9,0,0,initial}; AcceptedReplayScene scene;
    EXPECT_EQ(scene.Initialize(info,first,{}).status,ReplaySceneStatus::InvalidFrame);
    ASSERT_EQ(scene.Initialize(info,first,wall).status,ReplaySceneStatus::Ok);
    const auto camera = *scene.camera();
    EXPECT_EQ(camera.vertical,crash::visual::ReplayVertical::Y);
    EXPECT_LT(camera.position[0],camera.target[0]);
    EXPECT_LT(camera.position[2],camera.target[2]);
    // With Y up, the horizontal screen axis is perpendicular to the X/Z
    // sightline. Independently require useful projection of both the plate's
    // Z width and its much smaller physical X displacement; a grazing camera
    // can retain the latter while hiding nearly the entire surface.
    const double dx=camera.position[0]-camera.target[0];
    const double dy=camera.position[1]-camera.target[1];
    const double dz=camera.position[2]-camera.target[2];
    const double horizontal_distance=std::hypot(dx,dz);
    ASSERT_GT(horizontal_distance,0.);
    EXPECT_GE(std::abs(dx)/horizontal_distance,.6); // projected unit Z width
    EXPECT_GE(std::abs(dz)/horizontal_distance,.6); // projected unit X motion
    EXPECT_LT(std::abs(dy)/horizontal_distance,.2); // near-vertical long axis
    EXPECT_DOUBLE_EQ(camera.vertical_fov_degrees,40.);
    EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(),initial->GetCoordsVertices());
    const auto fixed = scene.wall_mesh()->GetCoordsVertices(); const auto moving=scene.moving_mesh();
    auto deformed=std::make_shared<chrono::ChTriangleMeshConnected>(*initial);
    deformed->GetCoordsVertices()[1].x()=.049; deformed->GetCoordsVertices()[2].x()=.049;
    ASSERT_EQ(scene.Publish({1,9,6,.01,deformed}).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(scene.moving_mesh(),moving); EXPECT_EQ(scene.wall_mesh()->GetCoordsVertices(),fixed);
    EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(),deformed->GetCoordsVertices());
    EXPECT_EQ(scene.camera()->position,camera.position); EXPECT_EQ(scene.camera()->target,camera.target);
    EXPECT_EQ(scene.camera()->vertical,camera.vertical);
    }
}

TEST(AcceptedReplayScene, SourcePartMagnificationIsExplicitPresentationAndPreservesInputAndFailedState) {
    auto info=Info();info.kind=crash::output::ReplayKind::SourcePartElastic;
    AcceptedReplayScene scene;auto first=Frame(0,0);auto next=Frame(1,.002);
    EXPECT_EQ(scene.Initialize(Info(),first,{},false,10).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.Initialize(info,first,{},false,0).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.Initialize(info,first,{},false,1001).status,ReplaySceneStatus::InvalidFrame);
    ASSERT_EQ(scene.Initialize(info,first,{},false,10).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(scene.deformation_scale(),10);EXPECT_FALSE(scene.wall_mesh());
    ASSERT_EQ(scene.system().GetBodies().size(),2u);
    const auto reference=scene.system().GetBodies()[1];
    EXPECT_EQ(reference->GetName(),"original source reference outline");
    EXPECT_TRUE(reference->IsFixed());EXPECT_FALSE(reference->IsCollisionEnabled());
    const auto outline=std::dynamic_pointer_cast<chrono::ChVisualShapeTriangleMesh>(reference->GetVisualShape(0));
    ASSERT_TRUE(outline);EXPECT_TRUE(outline->IsWireframe());EXPECT_FALSE(outline->IsMutable());
    EXPECT_EQ(outline->GetMesh()->GetCoordsVertices(),first.mesh->GetCoordsVertices());
    ASSERT_EQ(scene.Publish(next).status,ReplaySceneStatus::Ok);
    EXPECT_DOUBLE_EQ(scene.moving_mesh()->GetCoordsVertices()[1].z(),.02);
    EXPECT_DOUBLE_EQ(next.mesh->GetCoordsVertices()[1].z(),.002);
    EXPECT_DOUBLE_EQ(first.mesh->GetCoordsVertices()[1].z(),0);
    EXPECT_EQ(outline->GetMesh()->GetCoordsVertices(),first.mesh->GetCoordsVertices());
    const auto prior=scene.moving_mesh()->GetCoordsVertices();auto invalid=Frame(2,.002);invalid.owner_id=77;
    EXPECT_EQ(scene.Publish(invalid).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(),prior);EXPECT_EQ(scene.stamp()->index,1u);
    ASSERT_EQ(scene.Publish(Frame(2,-.002)).status,ReplaySceneStatus::Ok);
    EXPECT_DOUBLE_EQ(scene.moving_mesh()->GetCoordsVertices()[1].z(),-.02);
}

TEST(AcceptedReplayScene, SourceWallRequiresActualMeshAndKeepsPhysicalScaleWithIncidentCamera) {
    auto info=Info();info.kind=crash::output::ReplayKind::SourcePartWall;info.horizon_complete=false;
    auto wall=std::make_shared<chrono::ChTriangleMeshConnected>();
    wall->GetCoordsVertices()={{.201,-1,-1},{.201,1,-1},{.201,1,1},{.201,-1,1}};
    wall->GetIndicesVertices()={{0,2,1},{0,3,2}};
    auto initial=Frame(0,0);AcceptedReplayScene scene;
    EXPECT_EQ(scene.Initialize(info,initial,{}).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.Initialize(info,initial,wall,false,10).status,ReplaySceneStatus::InvalidFrame);
    ASSERT_EQ(scene.Initialize(info,initial,wall).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(scene.deformation_scale(),1);EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(),initial.mesh->GetCoordsVertices());
    EXPECT_EQ(scene.wall_mesh()->GetCoordsVertices(),wall->GetCoordsVertices());EXPECT_EQ(scene.wall_mesh()->GetIndicesVertices(),wall->GetIndicesVertices());
    EXPECT_EQ(scene.camera()->vertical,crash::visual::ReplayVertical::Z);EXPECT_LT(scene.camera()->position[0],scene.camera()->target[0]);
    ASSERT_EQ(scene.system().GetBodies().size(),2u);EXPECT_EQ(scene.system().GetBodies()[1]->GetName(),"placed original fixed wall");
    const auto fixed=scene.wall_mesh();const auto coordinates=fixed->GetCoordsVertices();const auto next=Frame(1,.002);
    ASSERT_EQ(scene.Publish(next).status,ReplaySceneStatus::Ok);EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(),next.mesh->GetCoordsVertices());
    EXPECT_EQ(scene.wall_mesh(),fixed);EXPECT_EQ(scene.wall_mesh()->GetCoordsVertices(),coordinates);
}

TEST(AcceptedReplayScene, AcceptedParentColorsUseExistingMutablePathAndKeepPhysicalCoordinates) {
    AcceptedReplayScene scene;const auto initial=PlasticFrame(0,0,0,0);const auto wall=Mesh();
    ASSERT_EQ(scene.Initialize(PlasticInfo(),initial,wall).status,ReplaySceneStatus::Ok);
    const auto shape=scene.moving_shape();const auto mesh=shape->GetMesh();
    EXPECT_TRUE(shape->IsMutable());EXPECT_TRUE(shape->IsFixedConnectivity());
    EXPECT_EQ(shape->GetNumMaterials(),0); // Owning VSG requires this for dynamic vertex/face colors.
    EXPECT_EQ(mesh->GetCoordsVertices(),initial.mesh->GetCoordsVertices());EXPECT_EQ(scene.deformation_scale(),1);
    ASSERT_EQ(mesh->GetIndicesColors().size(),3u);
    for(unsigned t=0;t<3;++t)EXPECT_EQ(mesh->GetIndicesColors()[t],chrono::ChVector3i(t,t,t));
    for(const auto& color:mesh->GetFaceColors())ColorNear(color,.12f,.64f,.94f);
    const auto next=PlasticFrame(1,.002,.01,.02);
    ASSERT_EQ(scene.Publish(next).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(scene.moving_shape(),shape);EXPECT_EQ(scene.moving_mesh().get(),mesh.get());
    EXPECT_EQ(mesh->GetCoordsVertices(),next.mesh->GetCoordsVertices());
    const auto& faces=mesh->GetFaceColors();ASSERT_EQ(faces.size(),9u);
    for(unsigned v=0;v<6;++v)ColorNear(faces[v],.98f,.84f,.16f); // Both original Q4 subtriangles.
    for(unsigned v=6;v<9;++v)ColorNear(faces[v],.90f,.12f,.10f); // Independent T3 parent.
    EXPECT_TRUE(initial.mesh->GetCoordsColors().empty());EXPECT_TRUE(next.mesh->GetCoordsColors().empty());
    const auto colors=mesh->GetCoordsColors();const auto camera=*scene.camera();
    ASSERT_EQ(scene.Publish(PlasticFrame(2,-.002,.01,.02)).status,ReplaySceneStatus::Ok);
    SameColors(mesh->GetCoordsColors(),colors);EXPECT_EQ(scene.camera()->position,camera.position);
    EXPECT_EQ(scene.wall_mesh()->GetCoordsVertices(),wall->GetCoordsVertices());
}

TEST(AcceptedReplayScene, LateColorOrGeometryFailurePreservesTheWholeAcceptedDisplayAndRetry) {
    AcceptedReplayScene scene;
    ASSERT_EQ(scene.Initialize(PlasticInfo(),PlasticFrame(0,0,0,0),Mesh()).status,ReplaySceneStatus::Ok);
    ASSERT_EQ(scene.Publish(PlasticFrame(1,.002,.005,.01)).status,ReplaySceneStatus::Ok);
    const auto positions=scene.moving_mesh()->GetCoordsVertices();const auto colors=scene.moving_mesh()->GetCoordsColors();
    auto invalid=PlasticFrame(2,-.002,.02,.02);
    invalid.parent_plastic_strain.back().value=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(scene.Publish(invalid).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(),positions);SameColors(scene.moving_mesh()->GetCoordsColors(),colors);
    invalid=PlasticFrame(2,-.002,.02,.02);invalid.parent_plastic_strain.back().source_parent=999;
    EXPECT_EQ(scene.Publish(invalid).status,ReplaySceneStatus::InvalidFrame);
    invalid=PlasticFrame(2,-.002,.02,.02);auto bad=std::make_shared<chrono::ChTriangleMeshConnected>(*invalid.mesh);
    bad->GetCoordsVertices().back().z()=std::numeric_limits<double>::quiet_NaN();invalid.mesh=bad;
    EXPECT_EQ(scene.Publish(invalid).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(),positions);SameColors(scene.moving_mesh()->GetCoordsColors(),colors);
    EXPECT_EQ(scene.stamp()->index,1u);EXPECT_EQ(scene.system().GetChTime(),.01);
    const auto retry=PlasticFrame(2,-.002,.02,.02);
    ASSERT_EQ(scene.Publish(retry).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(),retry.mesh->GetCoordsVertices());
    for(const auto& color:scene.moving_mesh()->GetCoordsColors())ColorNear(color,.90f,.12f,.10f);
    EXPECT_EQ(scene.stamp()->index,2u);EXPECT_EQ(scene.system().GetChTime(),.02);
}

TEST(AcceptedReplayScene, ParentAssociationAndScaleMustBeCompleteBeforeInitialization) {
    AcceptedReplayScene scene;auto info=PlasticInfo();const auto initial=PlasticFrame(0,0,0,0);
    info.plastic_strain_color_max=0;
    EXPECT_EQ(scene.Initialize(info,initial,Mesh()).status,ReplaySceneStatus::InvalidFrame);
    info=PlasticInfo();info.triangle_source_parent.back()=999;
    EXPECT_EQ(scene.Initialize(info,initial,Mesh()).status,ReplaySceneStatus::InvalidFrame);
    auto duplicate=initial;duplicate.parent_plastic_strain.back().source_parent=200;
    EXPECT_EQ(scene.Initialize(PlasticInfo(),duplicate,Mesh()).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.Initialize(PlasticInfo(),PlasticFrame(0,0,.001,0),Mesh()).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.stamp(),nullptr);
    ASSERT_EQ(scene.Initialize(PlasticInfo(),initial,Mesh()).status,ReplaySceneStatus::Ok);
}

TEST(AcceptedReplayScene, DisplayRangeAndDegenerateGeometryFailBeforePublication) {
    AcceptedReplayScene scene;
    auto invalid = Frame(0, 0);
    auto large = Mesh();
    large->GetCoordsVertices().back().z() = 1e100;
    invalid.mesh = large;
    EXPECT_EQ(scene.Initialize(Info(), invalid, {}).status, ReplaySceneStatus::InvalidFrame);
    auto degenerate = Mesh();
    degenerate->GetCoordsVertices()[2] = degenerate->GetCoordsVertices()[1];
    invalid.mesh = degenerate;
    EXPECT_EQ(scene.Initialize(Info(), invalid, {}).status, ReplaySceneStatus::InvalidFrame);
    auto info = Info();
    info.bounds_max[0] = std::numeric_limits<double>::infinity();
    EXPECT_EQ(scene.Initialize(info, Frame(0, 0), {}).status, ReplaySceneStatus::InvalidFrame);
    auto collapsed = Mesh();
    for (auto& p : collapsed->GetCoordsVertices()) p.x() += 1e8;
    invalid.mesh = collapsed;
    EXPECT_EQ(scene.Initialize(Info(), invalid, {}).status, ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.stamp(), nullptr);
    EXPECT_EQ(scene.Initialize(Info(), Frame(0, 0), {}).status, ReplaySceneStatus::Ok);
}

TEST(AcceptedReplayScene, InitialAndFinalRecordedTimesMustMatchDeclaredHorizon) {
    AcceptedReplayScene scene;
    auto first = Frame(0, 0);
    first.epoch = 1;
    EXPECT_EQ(scene.Initialize(Info(), first, {}).status, ReplaySceneStatus::InvalidFrame);
    first = Frame(0, 0);
    first.time = 0.001;
    EXPECT_EQ(scene.Initialize(Info(), first, {}).status, ReplaySceneStatus::InvalidFrame);
    ASSERT_EQ(scene.Initialize(Info(), Frame(0, 0), {}).status, ReplaySceneStatus::Ok);
    ASSERT_EQ(scene.Publish(Frame(1, 0.002)).status, ReplaySceneStatus::Ok);
    auto final = Frame(2, -0.002);
    final.epoch = 11;
    EXPECT_EQ(scene.Publish(final).status, ReplaySceneStatus::InvalidFrame);
    final = Frame(2, -0.002);
    final.time = 0.019;
    EXPECT_EQ(scene.Publish(final).status, ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.stamp()->index, 1u);
    ASSERT_EQ(scene.Publish(Frame(2, -0.002)).status, ReplaySceneStatus::Ok);
}
}  // namespace
