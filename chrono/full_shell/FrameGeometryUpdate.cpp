#include "FrameGeometryState.h"
#include "chrono/ReplayDisplayGeometry.h"

namespace crash::visual::full_shell::detail {
ReplaySceneReport UpdateFrame(FrameGeometryState& s, const output::full_shell::FrameRecord& frame,
        const output::full_shell::activity::ActivityRecord* activity,
        const output::full_shell::FrameStamp& expected) {
    try {
        output::Require(s.options.parent_activity == bool(activity),
            "Frame activity channel differs from explicit presentation profile");
        output::full_shell::CheckStamp(s.context, expected);
        output::Require(output::full_shell::SameStamp(frame.stamp, expected), "Frame differs from expected phase");
        output::full_shell::CheckFrame(s.context, {frame.stamp, frame.position_xyz.data(), frame.position_xyz.size(),
            frame.plastic_points.data(), frame.plastic_points.size()});
        if (activity) StageActivity(s, *activity, expected);
        for (std::size_t n = 0; n < s.staged_positions.size(); ++n)
            s.staged_positions[n] = {frame.position_xyz[3*n], frame.position_xyz[3*n+1], frame.position_xyz[3*n+2]};
        const auto& faces = activity ? s.activity->staged_faces : s.mesh->GetIndicesVertices();
        const bool valid = activity && faces.empty()
            ? CheckReplayDisplayPositions(s.staged_positions, s.options.geometry)
            : CheckReplayDisplayGeometry(s.staged_positions, faces, s.options.geometry);
        output::Require(valid, "Binary frame is not representable as renderer geometry");
        StageFields(s.context, frame, s.staged_fields);
        output::Require(s.scalar_colors.Stage(s.staged_fields, s.staged_colors), "Invalid staged native/missing fields");
        // All checks and staging precede every visible mutation. Palette slots
        // remain indexed by complete original triangle rank, even when hidden.
        s.mesh->GetCoordsVertices().swap(s.staged_positions);
        if (s.options.colors == ReplayColorMode::PlasticStrain) s.mesh->GetCoordsColors().swap(s.staged_colors);
        if (activity) {
            s.mesh->GetIndicesVertices().swap(s.activity->staged_faces);
            s.mesh->GetIndicesColors().swap(s.activity->staged_color_indices);
            s.triangle_parents.swap(s.activity->staged_parents);
            s.triangle_parts.swap(s.activity->staged_parts);
        }
        s.fields.swap(s.staged_fields);
        s.stamp = frame.stamp;
        s.visible = true;
        return {ReplaySceneStatus::Ok, "Binary presentation frame published; run provenance remains external"};
    } catch (const std::bad_alloc&) {
        return {ReplaySceneStatus::ResourceLimit, "Full-shell presentation staging allocation failed"};
    } catch (const std::exception&) {
        return {ReplaySceneStatus::InvalidFrame, "Invalid full-shell binary frame, activity or expected phase"};
    }
}
} // namespace crash::visual::full_shell::detail
