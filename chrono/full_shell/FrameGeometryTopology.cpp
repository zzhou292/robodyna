#include "FrameGeometryState.h"

namespace crash::visual::full_shell::detail {
void InitializeTopology(FrameGeometryState& s, const std::vector<std::uint32_t>& triangles,
        const std::vector<std::uint32_t>& parents) {
    const auto count = parents.size();
    output::Require(count && count <= s.options.geometry.triangles && triangles.size() == 3 * count,
        "Incomplete presentation topology");
    s.mesh = std::make_shared<chrono::ChTriangleMeshConnected>();
    s.mesh->GetCoordsVertices().resize(s.context.nodes());
    auto& faces = s.mesh->GetIndicesVertices();
    faces.reserve(count);
    s.triangle_parents.reserve(count);
    s.triangle_parts.reserve(count);
    for (std::size_t t = 0; t < count; ++t) {
        output::Require(parents[t] < s.context.parents().size(), "Unknown presentation parent index");
        for (unsigned j = 0; j < 3; ++j)
            output::Require(triangles[3*t+j] < s.context.nodes(), "Unknown presentation node index");
        faces.push_back({int(triangles[3*t]), int(triangles[3*t+1]), int(triangles[3*t+2])});
        const auto& parent = s.context.parents()[parents[t]];
        s.triangle_parents.push_back(parent.source_element);
        s.triangle_parts.push_back(parent.source_part);
    }
    s.fields = InitialFields(s.context);
    s.staged_fields = s.fields;
    output::Require(s.scalar_colors.Initialize(s.triangle_parents, s.fields,
        s.options.plastic_strain_maximum, s.staged_colors, s.options.geometry), "Invalid parent field mapping");
    if (s.options.colors == ReplayColorMode::PartId) {
        output::Require(s.part_colors.Initialize(s.triangle_parts, s.mesh->GetCoordsColors()),
            "Invalid original part color mapping");
    } else if (s.options.colors == ReplayColorMode::PlasticStrain) {
        s.mesh->GetCoordsColors() = s.staged_colors;
    }
    if (s.options.colors != ReplayColorMode::Uniform) {
        auto& indices = s.mesh->GetIndicesColors();
        indices.reserve(count);
        for (std::size_t t = 0; t < count; ++t) indices.push_back({int(t), int(t), int(t)});
    }
    s.staged_positions.resize(s.context.nodes());
    if (s.options.parent_activity) {
        s.activity = std::make_unique<ActivityTopology>();
        s.activity->complete_faces = faces;
        s.activity->triangle_parent_indices = parents;
        s.activity->staged_faces.reserve(count);
        s.activity->staged_color_indices.reserve(count);
        s.activity->staged_parents.reserve(count);
        s.activity->staged_parts.reserve(count);
    }
}
void StageActivity(FrameGeometryState& s, const output::full_shell::activity::ActivityRecord& activity,
        const output::full_shell::FrameStamp& expected) {
    namespace fs = output::full_shell;
    const auto& c = activity.context();
    output::Require(s.activity && fs::SameIdentity(c.identity(), s.context.identity()) &&
        c.point_layout_sha256() == s.context.point_layout_sha256() &&
        c.nodes() == s.context.nodes() && c.parents().size() == s.context.parents().size() &&
        c.points() == s.context.points() && c.fixed_dt() == s.context.fixed_dt(),
        "Activity source/owner/parent layout differs from presentation context");
    fs::CheckStamp(c, activity.stamp());
    output::Require(fs::SameStamp(activity.stamp(), expected), "Activity differs from expected frame phase");
    auto& a = *s.activity;
    a.staged_faces.clear();
    a.staged_color_indices.clear();
    a.staged_parents.clear();
    a.staged_parts.clear();
    for (std::size_t t = 0; t < a.complete_faces.size(); ++t) {
        const auto index = a.triangle_parent_indices[t];
        if (!activity.active(index)) continue;
        const auto& parent = s.context.parents()[index];
        a.staged_faces.push_back(a.complete_faces[t]);
        if (s.options.colors != ReplayColorMode::Uniform)
            a.staged_color_indices.push_back({int(t), int(t), int(t)});
        a.staged_parents.push_back(parent.source_element);
        a.staged_parts.push_back(parent.source_part);
    }
}
} // namespace crash::visual::full_shell::detail
