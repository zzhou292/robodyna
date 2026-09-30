#include "Internal.h"
#include <algorithm>

namespace crash::modelio::type25::detail {
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
