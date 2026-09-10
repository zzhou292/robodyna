#pragma once
#include "PreparedSourceMapping.h"

namespace crash::output::full_shell::source::detail {
enum MappingArray : std::size_t {
    NodeIds, NodeCanonical, ParentIds, ParentReference, ParentPoints, ParentNodes, Triangles, TriangleParents
};
struct MappingSpec { const char* name; arrays::Scalar scalar; unsigned columns; const char* fields; };
const std::array<MappingSpec, 8>& MappingSpecs();
arrays::Layout MappingLayout(std::size_t array, std::size_t nodes, std::size_t parents, std::size_t triangles);
void CheckMappingArrays(const std::array<NamedArray, 8>&, arrays::Limits);
} // namespace crash::output::full_shell::source::detail
