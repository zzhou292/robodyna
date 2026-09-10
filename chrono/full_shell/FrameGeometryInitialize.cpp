#include "FrameGeometryData.h"
#include "output/full_shell/static_bundle/MappingArrays.h"

namespace crash::visual::full_shell {
ReplaySceneReport FullShellFrameGeometry::Initialize(
        const output::full_shell::source::PreparedSourceMapping& mapping,
        const output::full_shell::Context& context, FrameGeometryOptions options) {
    if (impl_) return {ReplaySceneStatus::AlreadyInitialized, "Full-shell presentation already initialized"};
    try {
        const auto budget = FrameGeometryBudget(mapping.nodes(), mapping.parents().size(), mapping.triangles(), options);
        detail::CheckContext(mapping, context);
        output::Require((context.points() != 0) == (options.plastic_strain_maximum > 0),
            "Native plastic fields need a fixed scale; absent fields need no scalar scale");
        if (options.colors == ReplayColorMode::Automatic) options.colors = ReplayColorMode::PartId;
        output::Require(options.colors != ReplayColorMode::PlasticStrain || context.points() != 0,
            "Plastic-strain colors require native plastic values");
        auto next = std::make_unique<Impl>(mapping, context);
        next->options = options;
        next->budget = budget;
        namespace md = output::full_shell::source::detail;
        const auto& arrays = mapping.arrays();
        const auto triangles = output::arrays::Decode<std::uint32_t>(arrays[md::Triangles].descriptor, arrays[md::Triangles].bytes);
        const auto parents = output::arrays::Decode<std::uint32_t>(arrays[md::TriangleParents].descriptor, arrays[md::TriangleParents].bytes);
        const auto ids = output::arrays::Decode<std::uint64_t>(arrays[md::ParentIds].descriptor, arrays[md::ParentIds].bytes);
        next->mesh = std::make_shared<chrono::ChTriangleMeshConnected>();
        next->mesh->GetCoordsVertices().resize(mapping.nodes());
        auto& faces = next->mesh->GetIndicesVertices();
        faces.reserve(mapping.triangles());
        next->triangle_parents.reserve(mapping.triangles());
        next->triangle_parts.reserve(mapping.triangles());
        for (std::size_t t = 0; t < mapping.triangles(); ++t) {
            faces.push_back({int(triangles[3*t]), int(triangles[3*t+1]), int(triangles[3*t+2])});
            next->triangle_parents.push_back(ids[4 * parents[t]]);
            next->triangle_parts.push_back(ids[4 * parents[t] + 1]);
        }
        next->fields = detail::InitialFields(context);
        next->staged_fields = next->fields;
        output::Require(next->scalar_colors.Initialize(next->triangle_parents, next->fields,
            options.plastic_strain_maximum, next->staged_colors, options.geometry), "Invalid parent field mapping");
        if (options.colors == ReplayColorMode::PartId) {
            output::Require(next->part_colors.Initialize(next->triangle_parts, next->mesh->GetCoordsColors()),
                "Invalid original part color mapping");
        } else if (options.colors == ReplayColorMode::PlasticStrain) {
            next->mesh->GetCoordsColors() = next->staged_colors;
        }
        if (options.colors != ReplayColorMode::Uniform) {
            auto& indices = next->mesh->GetIndicesColors();
            indices.reserve(mapping.triangles());
            for (std::size_t t = 0; t < mapping.triangles(); ++t) indices.push_back({int(t), int(t), int(t)});
        }
        next->staged_positions.resize(mapping.nodes());
        impl_ = std::move(next);
        return {ReplaySceneStatus::Ok, "Full-shell presentation mapping initialized; no frame published"};
    } catch (const std::bad_alloc&) {
        return {ReplaySceneStatus::ResourceLimit, "Full-shell presentation allocation failed"};
    } catch (const std::exception&) {
        return {ReplaySceneStatus::InvalidFrame, "Invalid full-shell source/context/presentation configuration"};
    }
}
} // namespace crash::visual::full_shell
