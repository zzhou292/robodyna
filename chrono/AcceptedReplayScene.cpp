#include "AcceptedReplayScene.h"

#include "output/AcceptedReplay.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"
#include <algorithm>
#include <cmath>
#include <new>
#include <stdexcept>
#include <vector>

namespace crash::visual {
namespace {
bool Finite(const chrono::ChVector3d& p) {
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
}
bool DisplayGeometry(const std::vector<chrono::ChVector3d>& positions,
                     const std::vector<chrono::ChVector3i>& triangles) {
    if (positions.empty() || positions.size() > 4096 || triangles.empty() || triangles.size() > 8192) return false;
    // VSG's rendering buffers are float, and its actual GetFaceNormals uses a
    // binary64 cross product and length. Reject geometry those operations cannot
    // represent, even if it was valid for a more general archive consumer.
    for (const auto& p : positions)
        if (!Finite(p) || !std::isfinite(static_cast<float>(p.x())) ||
            !std::isfinite(static_cast<float>(p.y())) || !std::isfinite(static_cast<float>(p.z()))) return false;
    for (const auto& t : triangles) {
        for (int j = 0; j < 3; ++j)
            if (t[j] < 0 || static_cast<std::size_t>(t[j]) >= positions.size()) return false;
        const auto n = chrono::Vcross(positions[t[1]] - positions[t[0]], positions[t[2]] - positions[t[0]]);
        const double length2 = n.Length2();
        if (!Finite(n) || !std::isfinite(length2) || !(length2 > 0)) return false;
        chrono::ChVector3d displayed[3];
        for (int j = 0; j < 3; ++j) {
            const auto& p = positions[t[j]];
            displayed[j] = {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())};
        }
        const auto displayed_normal = chrono::Vcross(displayed[1] - displayed[0], displayed[2] - displayed[0]);
        if (!(displayed_normal.Length2() > 0)) return false;
    }
    return true;
}
std::shared_ptr<chrono::ChTriangleMeshConnected> CopyGeometry(const chrono::ChTriangleMeshConnected& source) {
    auto mesh = std::make_shared<chrono::ChTriangleMeshConnected>();
    mesh->GetCoordsVertices() = source.GetCoordsVertices();
    mesh->GetIndicesVertices() = source.GetIndicesVertices();
    return mesh;  // Archive materials/normals never replace our one-material policy.
}
std::shared_ptr<chrono::ChVisualShapeTriangleMesh> MakeShape(
    const std::shared_ptr<chrono::ChTriangleMeshConnected>& mesh, bool moving, bool wireframe) {
    auto shape = std::make_shared<chrono::ChVisualShapeTriangleMesh>();
    shape->SetMesh(mesh, false);
    shape->SetMutable(moving);
    shape->SetFixedConnectivity();
    shape->SetDoubleFaced(true);
    shape->SetBackfaceCull(false);
    shape->SetWireframe(wireframe);
    auto material = std::make_shared<chrono::ChVisualMaterial>();
    material->SetDiffuseColor(moving ? chrono::ChColor(0.12f, 0.64f, 0.94f) : chrono::ChColor(0.42f, 0.46f, 0.51f));
    material->SetMetallic(0.0f);
    material->SetRoughness(0.7f);
    shape->AddMaterial(material);
    return shape;
}
std::shared_ptr<chrono::ChBody> Carrier(const char* name,
                                      const std::shared_ptr<chrono::ChVisualShapeTriangleMesh>& shape) {
    auto body = std::make_shared<chrono::ChBody>();
    body->SetName(name);
    body->SetFixed(true);
    body->SetPos(chrono::VNULL);
    body->SetRot(chrono::QUNIT);
    body->EnableCollision(false);
    body->AddVisualShape(shape);
    // VSG's fixed-shape path reads the model instance flag; its mutable-mesh
    // path reads the shape flag. Keep both existing representations consistent.
    body->GetVisualModel()->EnableWireframe(0U, shape->IsWireframe());
    return body;
}
bool MakeCamera(const output::ReplayInfo& info, ReplayCamera& camera) {
    double extent[3];
    for (int i = 0; i < 3; ++i) {
        const double lo = info.bounds_min[i], hi = info.bounds_max[i];
        if (!std::isfinite(lo) || !std::isfinite(hi) || hi < lo) return false;
        camera.target[i] = lo * 0.5 + hi * 0.5;
        extent[i] = hi - lo;
        if (!std::isfinite(extent[i])) return false;
    }
    const double diagonal = std::hypot(extent[0], extent[1], extent[2]);
    if (!(diagonal > 0) || !std::isfinite(diagonal)) return false;
    // Fixed oblique view from the incident side of the -X-facing wall. Coupon
    // geometry is also visible from above. The full moving trajectory is framed;
    // a large fixed wall may extend beyond the image and is never rescaled.
    // Guided geometry is the coupon posed into Y/Z. View along its width,
    // from -X, with Y up to retain its physical normal travel in silhouette.
    camera.vertical = info.kind == output::ReplayKind::GuidedPlate ? ReplayVertical::Y : ReplayVertical::Z;
    const std::array<double, 3> direction = info.kind == output::ReplayKind::GuidedPlate
        ? std::array<double, 3>{-0.18, -0.3, -1.25}
        : info.kind == output::ReplayKind::ElasticCoupon
        ? std::array<double, 3>{-0.3, -1.25, 0.18}
        : std::array<double, 3>{-1.5, -1.25, 1.0};
    for (int i = 0; i < 3; ++i) {
        camera.position[i] = camera.target[i] + 1.6 * diagonal * direction[i];
        if (!std::isfinite(camera.position[i])) return false;
    }
    return true;
}
}  // namespace

struct AcceptedReplayScene::Impl {
    chrono::ChSystemNSC system;
    output::ReplayInfo info;
    std::shared_ptr<chrono::ChTriangleMeshConnected> moving, wall;
    std::shared_ptr<chrono::ChVisualShapeTriangleMesh> shape;
    std::vector<chrono::ChVector3d> staged;
    ReplayStamp stamp;
    ReplayCamera camera;
};
AcceptedReplayScene::AcceptedReplayScene() = default;
AcceptedReplayScene::~AcceptedReplayScene() = default;

ReplaySceneReport AcceptedReplayScene::Initialize(const output::ReplayInfo& info, const output::ReplayFrame& frame,
                                                std::shared_ptr<const chrono::ChTriangleMeshConnected> wall, bool wireframe) {
    if (impl_) return {ReplaySceneStatus::AlreadyInitialized, "Replay scene already initialized"};
    if (!frame.mesh || !info.owner_id || frame.owner_id != info.owner_id || frame.index != 0 || frame.epoch != 0 || frame.time != 0 ||
        !std::isfinite(info.final_time) || info.final_time < frame.time ||
        frame.epoch > info.final_epoch || !info.frame_count || info.frame_count > 1000 ||
        (info.frame_count == 1 && (info.final_epoch != 0 || info.final_time != 0)) ||
        (info.kind != output::ReplayKind::NormalImpact && info.kind != output::ReplayKind::ElasticCoupon &&
         info.kind != output::ReplayKind::GuidedPlate) ||
        frame.mesh->GetCoordsVertices().size() != info.node_count || frame.mesh->GetIndicesVertices().size() != info.triangle_count ||
        (info.kind != output::ReplayKind::ElasticCoupon) != static_cast<bool>(wall) ||
        !DisplayGeometry(frame.mesh->GetCoordsVertices(), frame.mesh->GetIndicesVertices()) ||
        (wall && !DisplayGeometry(wall->GetCoordsVertices(), wall->GetIndicesVertices())))
        return {ReplaySceneStatus::InvalidFrame, "Invalid validated replay geometry or metadata"};
    try {
        auto next = std::make_unique<Impl>();
        if (!MakeCamera(info, next->camera)) return {ReplaySceneStatus::InvalidFrame, "Invalid replay trajectory bounds"};
        next->info = info;
        next->moving = CopyGeometry(*frame.mesh);
        next->shape = MakeShape(next->moving, true, wireframe);
        next->staged.resize(info.node_count);
        next->system.SetGravitationalAcceleration(chrono::VNULL);
        next->system.AddBody(Carrier("accepted moving surface", next->shape));
        if (wall) {
            next->wall = CopyGeometry(*wall);
            next->system.AddBody(Carrier("canonical fixed wall", MakeShape(next->wall, false, true)));
        }
        next->stamp = {frame.index, frame.owner_id, frame.epoch, frame.time};
        next->system.SetChTime(frame.time);  // Recorded presentation time only.
        impl_ = std::move(next);
        return {ReplaySceneStatus::Ok, "Replay scene initialized"};
    } catch (const std::bad_alloc&) {
        return {ReplaySceneStatus::ResourceLimit, "Replay scene allocation failed"};
    }
}

ReplaySceneReport AcceptedReplayScene::Publish(const output::ReplayFrame& frame) {
    if (!impl_) return {ReplaySceneStatus::NotInitialized, "Replay scene is not initialized"};
    auto& state = *impl_;
    if (!frame.mesh || frame.owner_id != state.info.owner_id || frame.index != state.stamp.index + 1 ||
        frame.index >= state.info.frame_count || frame.epoch <= state.stamp.epoch || frame.epoch > state.info.final_epoch ||
        !std::isfinite(frame.time) || frame.time <= state.stamp.time || frame.time > state.info.final_time ||
        (frame.index + 1 == state.info.frame_count && (frame.epoch != state.info.final_epoch || frame.time != state.info.final_time)) ||
        frame.mesh->GetCoordsVertices().size() != state.staged.size() ||
        frame.mesh->GetIndicesVertices() != state.moving->GetIndicesVertices())
        return {ReplaySceneStatus::InvalidFrame, "Replay frame identity, ordering or connectivity changed"};
    std::copy(frame.mesh->GetCoordsVertices().begin(), frame.mesh->GetCoordsVertices().end(), state.staged.begin());
    if (!DisplayGeometry(state.staged, state.moving->GetIndicesVertices()))
        return {ReplaySceneStatus::InvalidFrame, "Replay frame cannot be represented by renderer geometry"};
    state.moving->GetCoordsVertices().swap(state.staged);
    state.stamp = {frame.index, frame.owner_id, frame.epoch, frame.time};
    state.system.SetChTime(frame.time);
    return {ReplaySceneStatus::Ok, "Replay frame published without rebinding"};
}
chrono::ChSystem& AcceptedReplayScene::system() {
    if (!impl_) throw std::logic_error("Replay scene is not initialized");
    return impl_->system;
}
const ReplayStamp* AcceptedReplayScene::stamp() const noexcept { return impl_ ? &impl_->stamp : nullptr; }
const ReplayCamera* AcceptedReplayScene::camera() const noexcept { return impl_ ? &impl_->camera : nullptr; }
std::shared_ptr<const chrono::ChTriangleMeshConnected> AcceptedReplayScene::moving_mesh() const noexcept {
    return impl_ ? impl_->moving : nullptr;
}
std::shared_ptr<const chrono::ChTriangleMeshConnected> AcceptedReplayScene::wall_mesh() const noexcept {
    return impl_ ? impl_->wall : nullptr;
}
std::shared_ptr<chrono::ChVisualShapeTriangleMesh> AcceptedReplayScene::moving_shape() const noexcept {
    return impl_ ? impl_->shape : nullptr;
}
}  // namespace crash::visual
