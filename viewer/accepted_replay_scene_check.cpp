#include "chrono/AcceptedReplayScene.h"
#include "output/AcceptedReplay.h"
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
