#pragma once

#include "output/full_shell/static_bundle/PreparedSourceMapping.h"
#include <array>
#include <cstdint>
#include <vector>

namespace crash::analysis::impact_response {

struct ParentCatalogEntry {
    std::uint64_t source_element = 0;
    std::uint64_t source_part = 0;
    std::uint64_t source_material = 0;
    std::uint64_t source_section = 0;
    std::uint32_t source_elform = 0;
    std::uint32_t native_family = 0;
    std::uint32_t family_index = 0;
    std::uint32_t canonical_parent = 0;
    std::uint32_t native_points = 0;
    output::full_shell::PlasticField plastic = output::full_shell::PlasticField::Unavailable;
    std::array<std::uint32_t, 4> mapped_nodes{};
    std::array<std::uint64_t, 4> source_nodes{};

    std::size_t node_count() const noexcept {
        return mapped_nodes[2] == mapped_nodes[3] ? 3 : 4;
    }
};

// Decode the authenticated retained mapping and independently reconcile every
// parent row with Context and the original PID material/section catalog.
// Mapping positions remain render indices; source node IDs remain source IDs.
std::vector<ParentCatalogEntry> BuildParentCatalog(
    const output::full_shell::source::PreparedSourceMapping&,
    const output::full_shell::Context&);

}  // namespace crash::analysis::impact_response
