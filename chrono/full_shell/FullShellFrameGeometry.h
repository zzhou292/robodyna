#pragma once
#include "chrono/AcceptedReplayScene.h"
#include "output/full_shell/activity/ActivityRecord.h"
#include "output/full_shell/static_bundle/PreparedSourceMapping.h"

namespace crash::visual::full_shell {
struct FrameGeometryOptions {
    ReplayGeometryLimits geometry;
    // Conservative additional presentation/storage workspace; excludes already
    // retained shared source/context and caller frame storage, plus Chrono/VSG
    // driver allocations.
    std::size_t host_bytes = 384 * 1024 * 1024;
    ReplayColorMode colors = ReplayColorMode::PartId;
    // Required positive fixed scale when native PLA exists; zero if none exists.
    double plastic_strain_maximum = 0;
    // Explicit optional channel. Every update then requires an authenticated
    // ActivityRecord; the legacy overload remains all-parent presentation.
    bool parent_activity = false;
};
std::size_t FrameGeometryBudget(std::size_t nodes, std::size_t parents, std::size_t triangles,
    const FrameGeometryOptions&);

// Presentation-only adapter for checked binary frame values. It creates no
// ReplayInfo, accepted-run assertion, physical owner, clock or collision shape.
// The outer reader must prove complete run/interval/index provenance separately.
// Updates may seek any externally expected frame, without inventing chronology.
// Calls/rendering are externally serialized. The live mesh handle keeps its
// identity; do not treat it as an immutable historical snapshot or mutate it.
class FullShellFrameGeometry {
  public:
    FullShellFrameGeometry();
    ~FullShellFrameGeometry();
    FullShellFrameGeometry(const FullShellFrameGeometry&) = delete;
    FullShellFrameGeometry& operator=(const FullShellFrameGeometry&) = delete;
    ReplaySceneReport Initialize(const output::full_shell::source::PreparedSourceMapping&,
        const output::full_shell::Context&, FrameGeometryOptions = {});
    ReplaySceneReport Update(const output::full_shell::FrameRecord&,
        const output::full_shell::FrameStamp& expected);
    ReplaySceneReport Update(const output::full_shell::FrameRecord&,
        const output::full_shell::activity::ActivityRecord&,
        const output::full_shell::FrameStamp& expected);
    // No visible frame/fields before a successful Update. Failure preserves the
    // last complete positions, fields, colors and stamp; input bytes are untouched.
    std::shared_ptr<const chrono::ChTriangleMeshConnected> mesh() const noexcept;
    const output::full_shell::FrameStamp* stamp() const noexcept;
    const std::vector<output::ReplayParentScalar>* fields() const noexcept;
    // Associations align with the current visible face order after an update.
    // The optional activity profile keeps the complete original PID legend.
    const std::vector<std::uint64_t>* triangle_source_parents() const noexcept;
    const std::vector<std::uint64_t>* triangle_source_parts() const noexcept;
    const ReplayScalarLegend* scalar_legend() const noexcept;
    const std::vector<ReplayPartLegendEntry>* part_legend() const noexcept;
    ReplayColorMode color_mode() const noexcept;
    std::size_t budget_bytes() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace crash::visual::full_shell
