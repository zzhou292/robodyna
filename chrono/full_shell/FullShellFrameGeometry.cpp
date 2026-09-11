#include "FrameGeometryData.h"

namespace crash::visual::full_shell {
FullShellFrameGeometry::FullShellFrameGeometry() = default;
FullShellFrameGeometry::~FullShellFrameGeometry() = default;
ReplaySceneReport FullShellFrameGeometry::Update(const output::full_shell::FrameRecord& frame,
        const output::full_shell::FrameStamp& expected) {
    if (!impl_) return {ReplaySceneStatus::NotInitialized, "Full-shell presentation is not initialized"};
    return detail::UpdateFrame(*impl_, frame, nullptr, expected);
}
ReplaySceneReport FullShellFrameGeometry::Update(const output::full_shell::FrameRecord& frame,
        const output::full_shell::activity::ActivityRecord& activity,
        const output::full_shell::FrameStamp& expected) {
    if (!impl_) return {ReplaySceneStatus::NotInitialized, "Full-shell presentation is not initialized"};
    return detail::UpdateFrame(*impl_, frame, &activity, expected);
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
