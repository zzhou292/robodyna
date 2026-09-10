#include "FrameGeometryData.h"
#include "chrono/ReplayDisplayGeometry.h"

namespace crash::visual::full_shell {
FullShellFrameGeometry::FullShellFrameGeometry() = default;
FullShellFrameGeometry::~FullShellFrameGeometry() = default;
ReplaySceneReport FullShellFrameGeometry::Update(const output::full_shell::FrameRecord& frame,
        const output::full_shell::FrameStamp& expected) {
    if (!impl_) return {ReplaySceneStatus::NotInitialized, "Full-shell presentation is not initialized"};
    auto& s = *impl_;
    try {
        output::full_shell::CheckStamp(s.context, expected);
        output::Require(output::full_shell::SameStamp(frame.stamp, expected), "Frame differs from caller expected phase");
        output::full_shell::CheckFrame(s.context, {frame.stamp, frame.position_xyz.data(), frame.position_xyz.size(),
            frame.plastic_points.data(), frame.plastic_points.size()});
        for (std::size_t n = 0; n < s.staged_positions.size(); ++n)
            s.staged_positions[n] = {frame.position_xyz[3*n], frame.position_xyz[3*n+1], frame.position_xyz[3*n+2]};
        output::Require(CheckReplayDisplayGeometry(s.staged_positions, s.mesh->GetIndicesVertices(), s.options.geometry),
            "Binary frame is not representable as renderer geometry");
        detail::StageFields(s.context, frame, s.staged_fields);
        output::Require(s.scalar_colors.Stage(s.staged_fields, s.staged_colors), "Invalid staged native/missing fields");
        s.mesh->GetCoordsVertices().swap(s.staged_positions);
        if (s.options.colors == ReplayColorMode::PlasticStrain) s.mesh->GetCoordsColors().swap(s.staged_colors);
        s.fields.swap(s.staged_fields);
        s.stamp = frame.stamp;
        s.visible = true;
        return {ReplaySceneStatus::Ok, "Binary presentation frame published; run provenance remains external"};
    } catch (const std::bad_alloc&) {
        return {ReplaySceneStatus::ResourceLimit, "Full-shell presentation staging allocation failed"};
    } catch (const std::exception&) {
        return {ReplaySceneStatus::InvalidFrame, "Invalid full-shell binary frame or expected phase"};
    }
}
std::shared_ptr<const chrono::ChTriangleMeshConnected> FullShellFrameGeometry::mesh() const noexcept {
    return impl_ && impl_->visible ? impl_->mesh : nullptr;
}
const output::full_shell::FrameStamp* FullShellFrameGeometry::stamp() const noexcept {
    return impl_ && impl_->visible ? &impl_->stamp : nullptr;
}
const std::vector<output::ReplayParentScalar>* FullShellFrameGeometry::fields() const noexcept {
    return impl_ && impl_->visible ? &impl_->fields : nullptr;
}
const std::vector<std::uint64_t>* FullShellFrameGeometry::triangle_source_parents() const noexcept {
    return impl_ ? &impl_->triangle_parents : nullptr;
}
const std::vector<std::uint64_t>* FullShellFrameGeometry::triangle_source_parts() const noexcept {
    return impl_ ? &impl_->triangle_parts : nullptr;
}
const ReplayScalarLegend* FullShellFrameGeometry::scalar_legend() const noexcept {
    return impl_ ? &impl_->scalar_colors.legend() : nullptr;
}
const std::vector<ReplayPartLegendEntry>* FullShellFrameGeometry::part_legend() const noexcept {
    return impl_ && impl_->options.colors == ReplayColorMode::PartId ? &impl_->part_colors.legend() : nullptr;
}
ReplayColorMode FullShellFrameGeometry::color_mode() const noexcept {
    return impl_ ? impl_->options.colors : ReplayColorMode::Automatic;
}
std::size_t FullShellFrameGeometry::budget_bytes() const noexcept { return impl_ ? impl_->budget : 0; }
} // namespace crash::visual::full_shell
