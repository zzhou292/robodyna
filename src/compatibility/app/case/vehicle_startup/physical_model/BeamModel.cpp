#include "Internal.h"

namespace crash::cases::vehicle_startup::physical_model::detail {
void PrepareBeams(const modelio::type13::SourceType13& source, const fe::NodalNodeDomain& domain,
                  std::size_t cap, fe::type13::Model& model) {
    const auto& data = source.data();
    std::vector<fe::type13::ModelNode> nodes;
    std::vector<fe::type13::ModelConnection> connections;
    nodes.reserve(data.nodes.size()); connections.reserve(data.beams.size());
    for (const auto& node : data.nodes)
        nodes.push_back({node.id, domain.Find(node.id), node.position_native});
    for (const auto& beam : data.beams) {
        fe::type13::ModelConnection input;
        input.source_id = beam.id; input.property = 0;
        for (unsigned k = 0; k < 3; ++k) input.node[k] = beam.node_indices[k];
        connections.push_back(input);
    }
    // The authenticated original conversion declares one SECTION_BEAM/PROP13
    // identity. N3 remains reference-only when absent from the physical domain.
    const fe::type13::ModelPropertyInput property{2000486, data.converted.input()};
    const fe::type13::ModelInput input{domain.source_instance_id(), data.property.units(), nodes.data(), &property,
        connections.data(), nodes.size(), 1, connections.size(), domain.node_count()};
    fe::type13::ModelLimits limits; limits.max_host_bytes = cap;
    const auto report = model.Initialize(input, limits);
    Require(bool(report), report.message);
}
} // namespace crash::cases::vehicle_startup::physical_model::detail
