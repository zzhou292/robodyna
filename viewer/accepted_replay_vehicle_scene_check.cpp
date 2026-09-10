#include "chrono/AcceptedReplayScene.h"
#include "chrono/ReplayParentScalarColors.h"
#include "output/AcceptedReplay.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChSystem.h"
#include <gtest/gtest.h>
#include <limits>

namespace {
using namespace crash::visual;
using crash::output::ReplayFrame;
using crash::output::ReplayInfo;

// Synthetic geometry at the audited no-tire source counts. This exercises the
// real Chrono scene owner, not a vehicle dynamics or original-geometry fixture.
struct VehicleSceneFixture {
    static constexpr std::size_t Nodes = 359785, Parents = 349645, Quads = 328344;
    static constexpr std::size_t Triangles = Parents + Quads;
    ReplayInfo info;
    ReplayFrame frame;
    std::shared_ptr<chrono::ChTriangleMeshConnected> mesh, wall;

    VehicleSceneFixture() {
        mesh = std::make_shared<chrono::ChTriangleMeshConnected>();
        auto& vertices = mesh->GetCoordsVertices();
        vertices.reserve(Nodes);
        for (std::size_t n = 0; n < Nodes; ++n)
            vertices.push_back({double(n % 600) / 100, double(n / 600) / 100, 0});
        auto& triangles = mesh->GetIndicesVertices();
        triangles.reserve(Triangles);
        info.triangle_source_parent.reserve(Triangles);
        info.triangle_source_part.reserve(Triangles);
        frame.parent_plastic_strain.reserve(Parents);
        for (std::size_t p = 0; p < Parents; ++p) {
            const int n = int(p % (Nodes - 601));
            triangles.push_back({n, n + 1, n + 600});
            if (p < Quads) triangles.push_back({n + 1, n + 601, n + 600});
            for (unsigned t = 0; t < (p < Quads ? 2u : 1u); ++t) {
                info.triangle_source_parent.push_back(1000000 + p);
                info.triangle_source_part.push_back(2000000 + p % 867);
            }
            frame.parent_plastic_strain.push_back({1000000 + p, 0});
        }
        triangles.back() = {int(Nodes - 601), int(Nodes - 1), int(Nodes - 2)};
        wall = std::make_shared<chrono::ChTriangleMeshConnected>();
        wall->GetCoordsVertices() = {{0, 0, -1}, {7, 0, -1}, {0, 7, -1}};
        wall->GetIndicesVertices() = {{0, 1, 2}};
        info.kind = crash::output::ReplayKind::SourceAssemblyWall;
        info.owner_id = 19;
        info.node_count = Nodes;
        info.triangle_count = Triangles;
        info.frame_count = 2;
        info.final_epoch = 1;
        info.final_time = .01;
        info.bounds_min = {0, 0, -1};
        info.bounds_max = {7, 7, 1};
        info.source_plasticity = true;
        info.plastic_strain_color_max = .02;
        frame.owner_id = info.owner_id;
        frame.mesh = mesh;
    }
    ReplaySceneReport Initialize(AcceptedReplayScene& scene, ReplayColorMode mode,
                                ReplayGeometryLimits limits = {}) {
        return scene.Initialize(info, frame, wall, false, 1, ReplayView::WallSide, mode, limits);
    }
};

TEST(AcceptedReplayVehicleScene, ExplicitVehicleCapacityPublishesLastNodeAndBothColorModes) {
    for (auto mode : {ReplayColorMode::PartId, ReplayColorMode::PlasticStrain}) {
        VehicleSceneFixture f;
        AcceptedReplayScene scene;
        EXPECT_EQ(f.Initialize(scene, mode).status, ReplaySceneStatus::InvalidFrame);
        EXPECT_EQ(scene.stamp(), nullptr);
        ASSERT_EQ(f.Initialize(scene, mode, ReplayGeometryLimits::Vehicle()).status, ReplaySceneStatus::Ok);
        const auto shape = scene.moving_shape();
        const auto mesh = shape->GetMesh();
        const auto last_color = mesh->GetCoordsColors().back();
        const auto first_position = mesh->GetCoordsVertices().front();
        const auto last_position = mesh->GetCoordsVertices().back();
        const auto normals = mesh->GetFaceNormals();
        // Chrono expands a flat face normal to its three rendered vertices.
        ASSERT_EQ(normals.size(), 3 * VehicleSceneFixture::Triangles);
        f.frame.index = 1; f.frame.epoch = 1; f.frame.time = .01;
        f.mesh->GetCoordsVertices().back().z() = .125;
        f.frame.parent_plastic_strain.back().value = .02;
        ASSERT_EQ(scene.Publish(f.frame).status, ReplaySceneStatus::Ok);
        EXPECT_EQ(scene.moving_shape(), shape);
        EXPECT_EQ(scene.moving_mesh(), mesh);
        EXPECT_EQ(mesh->GetCoordsVertices().front(), first_position);
        EXPECT_EQ(last_position.z(), 0);
        EXPECT_EQ(mesh->GetCoordsVertices().back().z(), .125);
        EXPECT_NE(mesh->GetFaceNormals().back(), normals.back());
        const auto expected = mode == ReplayColorMode::PartId ? last_color : ReplayScalarColor(1);
        const auto actual = mesh->GetCoordsColors().back();
        EXPECT_FLOAT_EQ(actual.R, expected.R);
        EXPECT_FLOAT_EQ(actual.G, expected.G);
        EXPECT_FLOAT_EQ(actual.B, expected.B);
        EXPECT_EQ(scene.stamp()->epoch, 1u);
        EXPECT_EQ(scene.system().GetChTime(), .01);
    }
}

TEST(AcceptedReplayVehicleScene, LateInvalidGeometryAndHiddenFieldPreserveAcceptedDisplayAndRetry) {
    VehicleSceneFixture f;
    AcceptedReplayScene scene;
    ASSERT_EQ(f.Initialize(scene, ReplayColorMode::PartId, ReplayGeometryLimits::Vehicle()).status, ReplaySceneStatus::Ok);
    const auto positions = scene.moving_mesh()->GetCoordsVertices();
    const auto colors = scene.moving_mesh()->GetCoordsColors();
    const auto unchanged = [&] {
        EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(), positions);
        EXPECT_EQ(scene.stamp()->epoch, 0u);
        EXPECT_EQ(scene.system().GetChTime(), 0);
        const auto& current = scene.moving_mesh()->GetCoordsColors();
        ASSERT_EQ(current.size(), colors.size());
        for (std::size_t i = 0; i < colors.size(); ++i) {
            ASSERT_FLOAT_EQ(current[i].R, colors[i].R);
            ASSERT_FLOAT_EQ(current[i].G, colors[i].G);
            ASSERT_FLOAT_EQ(current[i].B, colors[i].B);
        }
    };
    f.frame.index = 1; f.frame.epoch = 1; f.frame.time = .01;
    f.mesh->GetCoordsVertices().back().z() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(scene.Publish(f.frame).status, ReplaySceneStatus::InvalidFrame);
    unchanged();
    f.mesh->GetCoordsVertices().back().z() = .125;
    f.frame.parent_plastic_strain.back().value = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(scene.Publish(f.frame).status, ReplaySceneStatus::InvalidFrame);
    unchanged();
    f.frame.parent_plastic_strain.back().value = .02;
    ASSERT_EQ(scene.Publish(f.frame).status, ReplaySceneStatus::Ok);
    EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices().back().z(), .125);
}

TEST(AcceptedReplayVehicleScene, InvalidCapacityRejectsBeforeFrameAccess) {
    for (unsigned fault = 0; fault < 6; ++fault) {
        auto limits = ReplayGeometryLimits::Vehicle();
        if (fault == 0) limits.vertices = 0;
        if (fault == 1) ++limits.vertices;
        if (fault == 2) limits.triangles = 0;
        if (fault == 3) ++limits.triangles;
        if (fault == 4) limits.parents = 0;
        if (fault == 5) ++limits.parents;
        AcceptedReplayScene scene;
        EXPECT_EQ(scene.Initialize({}, {}, {}, false, 1, ReplayView::IncidentSide,
                                   ReplayColorMode::Uniform, limits).status, ReplaySceneStatus::ResourceLimit);
        EXPECT_EQ(scene.stamp(), nullptr);
    }
}

TEST(AcceptedReplayVehicleScene, MetadataCapacityIsCheckedBeforeCopyInEveryDisplayMode) {
    for (auto mode : {ReplayColorMode::Uniform, ReplayColorMode::PartId}) {
        for (unsigned fault = 0; fault < 4; ++fault) {
            auto mesh = std::make_shared<chrono::ChTriangleMeshConnected>();
            mesh->GetCoordsVertices() = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
            mesh->GetIndicesVertices() = {{0, 1, 2}};
            ReplayInfo info;
            info.kind = crash::output::ReplayKind::SourceAssemblyWall;
            info.owner_id = 1; info.frame_count = 1; info.node_count = 3; info.triangle_count = 1;
            info.bounds_min = {0, 0, 0}; info.bounds_max = {1, 1, 1};
            info.triangle_source_part = {25};
            ReplayFrame frame; frame.owner_id = 1; frame.mesh = mesh;
            auto limits = ReplayGeometryLimits{3, 1, 1};
            const auto initialize = [&](AcceptedReplayScene& scene) {
                return scene.Initialize(info, frame, mesh, false, 1, ReplayView::WallSide, mode, limits);
            };
            if (fault == 0) info.triangle_source_part.push_back(26);
            if (fault == 1) info.triangle_source_parent = {50, 51};
            if (fault == 2) frame.parent_plastic_strain = {{50, 0}, {51, 0}};
            if (fault == 3) { limits = ReplayGeometryLimits::Vehicle(); info.triangle_source_part.push_back(26); }
            AcceptedReplayScene scene;
            EXPECT_EQ(initialize(scene).status, ReplaySceneStatus::InvalidFrame);
            EXPECT_EQ(scene.stamp(), nullptr);
            info.triangle_source_part = {25}; info.triangle_source_parent.clear();
            frame.parent_plastic_strain.clear();
            EXPECT_EQ(initialize(scene).status, ReplaySceneStatus::Ok);
        }
    }
}
}  // namespace
