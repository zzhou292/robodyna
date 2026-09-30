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
        detail::InitializeTopology(*next, triangles, parents);
        impl_ = std::move(next);
        return {ReplaySceneStatus::Ok, "Full-shell presentation mapping initialized; no frame published"};
    } catch (const std::bad_alloc&) {
        return {ReplaySceneStatus::ResourceLimit, "Full-shell presentation allocation failed"};
    } catch (const std::exception&) {
        return {ReplaySceneStatus::InvalidFrame, "Invalid full-shell source/context/presentation configuration"};
    }
}
} // namespace crash::visual::full_shell
