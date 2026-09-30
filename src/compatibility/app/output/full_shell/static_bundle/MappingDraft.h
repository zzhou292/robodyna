#pragma once
#include "MappingArrays.h"

namespace crash::output::full_shell::source::detail {
struct MappingDraft {
    std::vector<std::uint64_t> node_ids, parent_ids;
    std::vector<std::uint32_t> node_canonical, parent_reference, parent_points;
    std::vector<std::uint32_t> parent_nodes, triangles, triangle_parents;
    std::vector<full_shell::ParentPoints> points;
};
std::size_t MappingWorkingBytes(const CanonicalData&,MappingInput);
MappingDraft BuildMapping(const CanonicalData&, MappingInput);
std::array<NamedArray, 8> EncodeMapping(const MappingDraft&, arrays::Limits);
} // namespace crash::output::full_shell::source::detail
