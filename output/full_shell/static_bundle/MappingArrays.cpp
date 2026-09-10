#include "MappingArrays.h"
#include "CanonicalSpecs.h"

namespace crash::output::full_shell::source::detail {
const std::array<MappingSpec, 8>& MappingSpecs() {
    using S = arrays::Scalar;
    static const std::array<MappingSpec, 8> specs{{
        {"node_source_ids", S::UInt64, 1, "source_node_id"},
        {"node_canonical_indices", S::UInt32, 1, "canonical_node_index"},
        {"parent_source_ids", S::UInt64, 4, "source_element_id,source_part_id,source_material_id,source_section_id"},
        {"parent_reference", S::UInt32, 3, "canonical_parent_index,native_family,family_local_index"},
        {"parent_points", S::UInt32, 3, "source_elform,native_points,plastic_field_applicability"},
        {"parent_nodes", S::UInt32, 4, "n1,n2,n3,n4"},
        {"triangles", S::UInt32, 3, "n1,n2,n3"},
        {"triangle_parents", S::UInt32, 1, "parent_index"}
    }};
    return specs;
}
arrays::Layout MappingLayout(std::size_t index, std::size_t nodes, std::size_t parents, std::size_t triangles) {
    Require(index < MappingSpecs().size(), "Unknown source mapping array");
    const auto& spec = MappingSpecs()[index];
    const auto rows = index <= NodeCanonical ? nodes : index >= Triangles ? triangles : parents;
    return {spec.scalar, rows, spec.columns, FieldNames(spec.fields)};
}
void CheckMappingArrays(const std::array<NamedArray, 8>& a, arrays::Limits cap) {
    const auto nodes = a[NodeIds].descriptor.layout.rows;
    const auto parents = a[ParentIds].descriptor.layout.rows;
    const auto triangles = a[Triangles].descriptor.layout.rows;
    Require(nodes && nodes <= 1048576 && parents && parents <= 1048576 &&
        triangles >= parents && triangles <= 2 * parents,
        "Invalid source mapping counts");
    for (std::size_t i = 0; i < a.size(); ++i) {
        arrays::CheckDescriptor(a[i].descriptor, cap);
        const auto wanted = MappingLayout(i, nodes, parents, triangles);
        const auto& layout = a[i].descriptor.layout;
        Require(a[i].name == MappingSpecs()[i].name && layout.scalar == wanted.scalar &&
            layout.rows == wanted.rows && layout.columns == wanted.columns && layout.fields == wanted.fields &&
            a[i].bytes.size() == a[i].descriptor.bytes && Sha256(a[i].bytes) == a[i].descriptor.sha256,
            "Source mapping layout/content identity changed");
    }
}
} // namespace crash::output::full_shell::source::detail
