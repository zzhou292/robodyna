#pragma once
#include "FullShellFrameGeometry.h"
#include "chrono/ReplayParentScalarColors.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace crash::visual::full_shell::detail {
// Shared presentation values only. The public adapter separately authenticates
// and retains the original source mapping before initializing this state.
struct ActivityTopology {
    std::vector<chrono::ChVector3i> complete_faces, staged_faces, staged_color_indices;
    std::vector<std::uint32_t> triangle_parent_indices;
    std::vector<std::uint64_t> staged_parents, staged_parts;
};
struct FrameGeometryState {
    explicit FrameGeometryState(const output::full_shell::Context& c) : context(c) {}
    output::full_shell::Context context;
    FrameGeometryOptions options;
    std::size_t budget = 0;
    std::shared_ptr<chrono::ChTriangleMeshConnected> mesh;
    std::vector<chrono::ChVector3d> staged_positions;
    std::vector<chrono::ChColor> staged_colors;
    std::vector<output::ReplayParentScalar> fields, staged_fields;
    std::vector<std::uint64_t> triangle_parents, triangle_parts;
    ReplayParentScalarColors scalar_colors;
    ReplayPartColors part_colors;
    std::unique_ptr<ActivityTopology> activity;
    output::full_shell::FrameStamp stamp;
    bool visible = false;
};
void InitializeTopology(FrameGeometryState&, const std::vector<std::uint32_t>& triangles,
    const std::vector<std::uint32_t>& triangle_parents);
void StageActivity(FrameGeometryState&, const output::full_shell::activity::ActivityRecord&,
    const output::full_shell::FrameStamp& expected);
ReplaySceneReport UpdateFrame(FrameGeometryState&, const output::full_shell::FrameRecord&,
    const output::full_shell::activity::ActivityRecord*, const output::full_shell::FrameStamp& expected);
std::vector<output::ReplayParentScalar> InitialFields(const output::full_shell::Context&);
void StageFields(const output::full_shell::Context&, const output::full_shell::FrameRecord&,
    std::vector<output::ReplayParentScalar>&);
} // namespace crash::visual::full_shell::detail
