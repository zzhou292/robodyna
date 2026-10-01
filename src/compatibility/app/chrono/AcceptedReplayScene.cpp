#include "AcceptedReplayScene.h"
#include "ReplayParentScalarColors.h"
#include "ReplayDisplayGeometry.h"
#include "ReplayVisuals.h"

#include "output/AcceptedReplay.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "robodyna/mbd/RbBody.h"
#include "robodyna/core/RbVector3.h"
#include "robodyna/simulation/RbSystemNSC.h"
#include <algorithm>
#include <cmath>
#include <new>
#include <stdexcept>
#include <vector>

namespace crash::visual {
namespace {
bool SourceWall(output::ReplayKind kind) {
    return kind==output::ReplayKind::SourcePartWall||kind==output::ReplayKind::SourceAssemblyWall;
}
bool MakeCamera(const output::ReplayInfo& info, ReplayView view, ReplayCamera& camera) {
    // Fixed oblique view, optionally across the -X-facing source wall. Coupon
    // geometry is also visible from above. The full moving trajectory is framed;
    // a large fixed wall may extend beyond the image and is never rescaled.
    // Guided geometry is the coupon posed into Y/Z. A 45-degree X/Z view
    // from incident -X exposes its surface while retaining normal travel:
    // horizontal projection keeps sqrt(1/2) of both Z width and X motion.
    // Y up and a small Y offset keep the long plate axis almost vertical.
    camera.vertical = info.kind == output::ReplayKind::GuidedPlate ? ReplayVertical::Y : ReplayVertical::Z;
    camera.view = view;
    std::array<double, 3> direction = SourceWall(info.kind)
        ? std::array<double, 3>{-1.0, -1.0, 0.15}
        : info.kind == output::ReplayKind::GuidedPlate
        ? std::array<double, 3>{-1.0, -0.15, -1.0}
        : info.kind == output::ReplayKind::ElasticCoupon
        ? std::array<double, 3>{-0.3, -1.25, 0.18}
        : std::array<double, 3>{-1.5, -1.25, 1.0};
    return MakeBoundsCamera(info.bounds_min,info.bounds_max,direction,SourceWall(info.kind)?1.25:1.6,
        camera.vertical,view,camera);
}
}  // namespace

struct AcceptedReplayScene::Impl {
    robodyna::simulation::RbSystemNSC system;
    output::ReplayInfo info;
    std::shared_ptr<chrono::ChTriangleMeshConnected> moving, wall;
    std::shared_ptr<chrono::ChVisualShapeTriangleMesh> shape;
    std::vector<robodyna::core::RbVector3d> staged, reference;
    ReplayParentScalarColors parent_colors;
    ReplayPartColors part_colors;
    ReplayColorMode color_mode = ReplayColorMode::Uniform;
    ReplayGeometryLimits limits;
    std::vector<chrono::ChColor> staged_colors;
    double deformation_scale = 1;
    ReplayStamp stamp;
    ReplayCamera camera;
};
AcceptedReplayScene::AcceptedReplayScene() = default;
AcceptedReplayScene::~AcceptedReplayScene() = default;

ReplaySceneReport AcceptedReplayScene::Initialize(const output::ReplayInfo& info, const output::ReplayFrame& frame,
                                                std::shared_ptr<const chrono::ChTriangleMeshConnected> wall, bool wireframe,
                                                double deformation_scale, ReplayView view, ReplayColorMode colors,
                                                ReplayGeometryLimits limits) {
    if (impl_) return {ReplaySceneStatus::AlreadyInitialized, "Replay scene already initialized"};
    if (!limits.valid()) return {ReplaySceneStatus::ResourceLimit, "Invalid explicit replay geometry capacity"};
    if (info.triangle_source_part.size() > limits.triangles ||
        (!info.triangle_source_part.empty() && info.triangle_source_part.size() != info.triangle_count) ||
        info.triangle_source_parent.size() > limits.triangles ||
        frame.parent_plastic_strain.size() > limits.parents)
        return {ReplaySceneStatus::InvalidFrame, "Replay source associations exceed geometry capacity"};
    if (!ReplayColorModeName(colors)) return {ReplaySceneStatus::InvalidFrame, "Invalid replay color mode"};
    if (colors == ReplayColorMode::Automatic)
        colors = info.source_plasticity ? ReplayColorMode::PlasticStrain : ReplayColorMode::Uniform;
    if (colors == ReplayColorMode::PlasticStrain && !info.source_plasticity)
        return {ReplaySceneStatus::InvalidFrame, "Plastic-strain colors require accepted source plastic fields"};
    if (colors == ReplayColorMode::PartId &&
        ((!SourceWall(info.kind) && info.kind != output::ReplayKind::SourcePartElastic) ||
         info.triangle_source_part.size() != info.triangle_count))
        return {ReplaySceneStatus::InvalidFrame, "Part colors require complete original source part IDs"};
    if (!ReplayViewName(view) || (view == ReplayView::WallSide && !SourceWall(info.kind)))
        return {ReplaySceneStatus::InvalidFrame, "wall-side view requires a source-part or source-assembly wall replay"};
    if (!std::isfinite(deformation_scale) || deformation_scale < 1 || deformation_scale > 1000 ||
        (deformation_scale != 1 && info.kind != output::ReplayKind::SourcePartElastic) ||
        !frame.mesh || !info.owner_id || frame.owner_id != info.owner_id || frame.index != 0 || frame.epoch != 0 || frame.time != 0 ||
        !std::isfinite(info.final_time) || info.final_time < frame.time ||
        frame.epoch > info.final_epoch || !info.frame_count || info.frame_count > 1000 ||
        (info.frame_count == 1 && (info.final_epoch != 0 || info.final_time != 0)) ||
        (info.kind != output::ReplayKind::NormalImpact && info.kind != output::ReplayKind::ElasticCoupon &&
         info.kind != output::ReplayKind::GuidedPlate && info.kind != output::ReplayKind::SourcePartElastic &&
         !SourceWall(info.kind)) ||
        frame.mesh->GetCoordsVertices().size() != info.node_count || frame.mesh->GetIndicesVertices().size() != info.triangle_count ||
        (info.kind == output::ReplayKind::NormalImpact || info.kind == output::ReplayKind::GuidedPlate ||
         SourceWall(info.kind)) != static_cast<bool>(wall) ||
        !CheckReplayDisplayGeometry(frame.mesh->GetCoordsVertices(), frame.mesh->GetIndicesVertices(), limits) ||
        (wall && !CheckReplayDisplayGeometry(wall->GetCoordsVertices(), wall->GetIndicesVertices())))
        return {ReplaySceneStatus::InvalidFrame, "Invalid validated replay geometry or metadata"};
    try {
        auto next = std::make_unique<Impl>();
        if (!MakeCamera(info, view, next->camera)) return {ReplaySceneStatus::InvalidFrame, "Invalid replay trajectory bounds"};
        next->info = info;
        next->color_mode = colors;
        next->limits = limits;
        next->moving = CopyReplayGeometry(*frame.mesh);
        if(info.source_plasticity) {
            if(!SourceWall(info.kind)||info.triangle_source_parent.size()!=info.triangle_count||
               !next->parent_colors.Initialize(info.triangle_source_parent,frame.parent_plastic_strain,
                    info.plastic_strain_color_max,next->moving->GetCoordsColors(), limits))
                return {ReplaySceneStatus::InvalidFrame,"Invalid accepted plastic display association or fixed scale"};
            if (!next->parent_colors.legend().native)
                return {ReplaySceneStatus::InvalidFrame,"Declared plastic replay has no native plastic values"};
            for(const auto& parent:frame.parent_plastic_strain)
                if(parent.applicability == output::ReplayScalarApplicability::NativeValue && parent.value!=0)
                return {ReplaySceneStatus::InvalidFrame,"Initial accepted plastic display field is not zero"};
            next->staged_colors.resize(info.triangle_count);
        } else if(!frame.parent_plastic_strain.empty()||!info.triangle_source_parent.empty()||info.plastic_strain_color_max!=0)
            return {ReplaySceneStatus::InvalidFrame,"Plastic display values lack a declared material"};
        if (colors == ReplayColorMode::PartId &&
            !next->part_colors.Initialize(info.triangle_source_part,next->moving->GetCoordsColors()))
            return {ReplaySceneStatus::InvalidFrame, "Part colors require positive original source part IDs"};
        if (colors != ReplayColorMode::Uniform) {
            auto& indices=next->moving->GetIndicesColors();indices.reserve(info.triangle_count);
            for(std::size_t t=0;t<info.triangle_count;++t)indices.push_back({int(t),int(t),int(t)});
        } else next->moving->GetCoordsColors().clear();
        next->shape = MakeReplayShape(next->moving, true, wireframe,colors != ReplayColorMode::Uniform);
        next->staged.resize(info.node_count);
        next->deformation_scale = deformation_scale;
        if (deformation_scale != 1) next->reference = frame.mesh->GetCoordsVertices();
        next->system.SetGravitationalAcceleration(chrono::VNULL);
        next->system.AddBody(MakeReplayCarrier("accepted moving surface", next->shape));
        if (deformation_scale != 1)
            next->system.AddBody(MakeReplayCarrier("original source reference outline",
                MakeReplayShape(CopyReplayGeometry(*frame.mesh), false, true)));
        if (wall) {
            next->wall = CopyReplayGeometry(*wall);
            next->system.AddBody(MakeReplayCarrier(SourceWall(info.kind)?"placed original fixed wall":"canonical fixed wall",
                MakeReplayShape(next->wall, false, true)));
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
    if (state.deformation_scale != 1)
        for (std::size_t n = 0; n < state.staged.size(); ++n)
            state.staged[n] = state.reference[n] + state.deformation_scale * (state.staged[n] - state.reference[n]);
    if (!CheckReplayDisplayGeometry(state.staged, state.moving->GetIndicesVertices(), state.limits))
        return {ReplaySceneStatus::InvalidFrame, "Replay frame cannot be represented by renderer geometry"};
    if(state.info.source_plasticity) {
        if(!state.parent_colors.Stage(frame.parent_plastic_strain,state.staged_colors))
            return {ReplaySceneStatus::InvalidFrame,"Replay parent identity or plastic display scalar changed"};
    } else if(!frame.parent_plastic_strain.empty())
        return {ReplaySceneStatus::InvalidFrame,"Undeclared plastic display field"};
    state.moving->GetCoordsVertices().swap(state.staged);
    if(state.color_mode == ReplayColorMode::PlasticStrain)state.moving->GetCoordsColors().swap(state.staged_colors);
    state.stamp = {frame.index, frame.owner_id, frame.epoch, frame.time};
    state.system.SetChTime(frame.time);
    return {ReplaySceneStatus::Ok, "Replay frame published without rebinding"};
}
robodyna::simulation::RbSystem& AcceptedReplayScene::system() {
    if (!impl_) throw std::logic_error("Replay scene is not initialized");
    return impl_->system;
}
const ReplayStamp* AcceptedReplayScene::stamp() const noexcept { return impl_ ? &impl_->stamp : nullptr; }
double AcceptedReplayScene::deformation_scale() const noexcept { return impl_ ? impl_->deformation_scale : 1; }
ReplayColorMode AcceptedReplayScene::color_mode() const noexcept {
    return impl_ ? impl_->color_mode : ReplayColorMode::Automatic;
}
const ReplayScalarLegend* AcceptedReplayScene::scalar_legend() const noexcept {
    return impl_ && impl_->info.source_plasticity ? &impl_->parent_colors.legend() : nullptr;
}
const std::vector<ReplayPartLegendEntry>* AcceptedReplayScene::part_legend() const noexcept {
    return impl_ && impl_->color_mode == ReplayColorMode::PartId ? &impl_->part_colors.legend() : nullptr;
}
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
