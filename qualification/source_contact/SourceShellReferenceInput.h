#pragma once
#include "SourcePartContactFixture.h"
#include "lib_src/elements/qeph/QephData.h"
#include "lib_src/elements/t3/T3Data.h"
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace crash::qualification::source_contact {
namespace shell_input_detail {
// Shared typed conversion only. Startup producers still validate each native
// reference. The pinned source geometry, density and thickness are unchanged;
// LAW1 E/nu are explicit experimental choices, not source MAT024 processing.
template<class Input, unsigned Arity>
Input Reference(const SourcePartContactFixture& source, std::size_t parent_index) {
    if (!source.prepared() || parent_index >= ParentCount ||
        source.parents()[parent_index].arity != Arity)
        throw std::invalid_argument("Shell input requires an authenticated parent of the requested arity");
    Input result;
    const auto& parent = source.parents()[parent_index];
    using NodeId = std::remove_reference_t<decltype(result.node_ids[0])>;
    for (unsigned n = 0; n < Arity; ++n) {
        const auto i = parent.local_node_indices[n];
        const auto id = source.nodes()[i].source_id;
        if (id > std::numeric_limits<NodeId>::max())
            throw std::invalid_argument("Source node ID exceeds this native formulation's ID representation");
        const auto p = source.positions().at(i);
        result.position[n] = {p.x, p.y, p.z};
        result.node_ids[n] = static_cast<NodeId>(id);
    }
    result.density = source.surface_mass().density_kg_m3;
    result.thickness = source.surface_mass().thickness_m;
    result.young_modulus = 200e9;
    result.poisson_ratio = .3;
    return result;
}
}  // namespace shell_input_detail
inline tl::fea::qeph::ReferenceInput QephReferenceInput(const SourcePartContactFixture& source,
                                                     std::size_t parent) {
    return shell_input_detail::Reference<tl::fea::qeph::ReferenceInput, 4>(source, parent);
}
inline tl::fea::t3::ReferenceInput T3PortReferenceInput(const SourcePartContactFixture& source,
                                                    std::size_t parent) {
    return shell_input_detail::Reference<tl::fea::t3::ReferenceInput, 3>(source, parent);
}
}  // namespace crash::qualification::source_contact
