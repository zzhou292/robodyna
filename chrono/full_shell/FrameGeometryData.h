#pragma once
#include "FullShellFrameGeometry.h"
#include "chrono/ReplayParentScalarColors.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace crash::visual::full_shell {
struct FullShellFrameGeometry::Impl {
    Impl(const output::full_shell::source::PreparedSourceMapping& m, const output::full_shell::Context& c)
        : mapping(m), context(c) {}
    output::full_shell::source::PreparedSourceMapping mapping;
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
    output::full_shell::FrameStamp stamp;
    bool visible = false;
};
namespace detail {
void CheckContext(const output::full_shell::source::PreparedSourceMapping&, const output::full_shell::Context&);
std::vector<output::ReplayParentScalar> InitialFields(const output::full_shell::Context&);
void StageFields(const output::full_shell::Context&, const output::full_shell::FrameRecord&,
    std::vector<output::ReplayParentScalar>&);
} // namespace detail
} // namespace crash::visual::full_shell
