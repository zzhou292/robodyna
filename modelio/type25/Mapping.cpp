#include "Internal.h"
#include <algorithm>

namespace crash::modelio::type25::detail {
void CheckDomain(const std::vector<std::uint64_t>& ids, const std::vector<double>& positions,
                 const std::vector<std::uint16_t>& roles, const tl::fea::NodalNodeDomain& domain,
                 std::uint64_t instance, std::size_t expected_count) {
    Require(ids.size() <= SIZE_MAX / 3 && positions.size() == ids.size() * 3 && roles.size() == ids.size() &&
        domain.prepared() && domain.source_instance_id() == instance && domain.node_count() >= expected_count,
        "TYPE25 common-domain source or extent changed");
    Require(!ids.empty() && ids.front() &&
        std::adjacent_find(ids.begin(), ids.end(), std::greater_equal<std::uint64_t>()) == ids.end(),
        "TYPE25 canonical source node order changed");
    std::size_t mapped = 0, declared = 0;
    for (std::size_t n = 0; n < ids.size(); ++n) {
        Require((roles[n] & ~127u) == 0, "TYPE25 physical source role changed");
        const bool required = roles[n] & (physical_scope::PhysicalRoles | physical_scope::ProvisionalType25);
        const auto index = domain.Find(ids[n]);
        if (!required && index == SIZE_MAX) continue;
        Require(index < domain.node_count(), "TYPE25 common domain omits an original physical node");
        const auto& actual = domain.nodes()[index];
        Require(actual.source_id == ids[n] && output::Bits(actual.position.x) == output::Bits(positions[3 * n]) &&
            output::Bits(actual.position.y) == output::Bits(positions[3 * n + 1]) &&
            output::Bits(actual.position.z) == output::Bits(positions[3 * n + 2]),
            "TYPE25 common-domain coordinate differs from original canonical SI bits");
        mapped += required;
        ++declared;
    }
    Require(mapped == expected_count && declared == domain.node_count(),
            "TYPE25 common domain contains a node outside authenticated original coordinates");
}
std::vector<native::ConnectionInput> Pack(const std::vector<physical_scope::Spotweld>& welds,
                                        const tl::fea::NodalNodeDomain& domain) {
    std::vector<native::ConnectionInput> result;
    result.reserve(welds.size());
    for (const auto& weld : welds) {
        Require(weld.default_only, "Original TYPE25 policy does not admit optional source fields");
        native::ConnectionInput connection;
        connection.source_element_id = weld.id;
        for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
            const auto index = domain.Find(weld.nodes[endpoint]);
            Require(index < domain.node_count(), "Original TYPE25 endpoint is missing from common domain");
            connection.source_node_id[endpoint] = weld.nodes[endpoint];
            connection.global_node[endpoint] = index;
            connection.position[endpoint] = domain.nodes()[index].position;
        }
        result.push_back(connection);
    }
    return result;
}
} // namespace crash::modelio::type25::detail
