#include "MappingDraft.h"

namespace crash::output::full_shell::source::detail {
std::array<NamedArray, 8> EncodeMapping(const MappingDraft& input, arrays::Limits limits) {
    std::array<NamedArray, 8> result;
    auto encode = [&](std::size_t i, const auto& values) {
        auto& a = result[i];
        a.name = MappingSpecs()[i].name;
        const auto layout = MappingLayout(i, input.node_ids.size(), input.points.size(), input.triangle_parents.size());
        a.bytes = arrays::Encode(layout, values.data(), values.size(), limits);
        a.descriptor = {a.name + ".bin", layout, a.bytes.size(), Sha256(a.bytes)};
    };
    encode(NodeIds, input.node_ids);
    encode(NodeCanonical, input.node_canonical);
    encode(ParentIds, input.parent_ids);
    encode(ParentReference, input.parent_reference);
    encode(ParentPoints, input.parent_points);
    encode(ParentNodes, input.parent_nodes);
    encode(Triangles, input.triangles);
    encode(TriangleParents, input.triangle_parents);
    return result;
}
} // namespace crash::output::full_shell::source::detail
