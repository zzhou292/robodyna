#include "Internal.h"
namespace crash::cases::vehicle_startup::tied_assessment_detail {
native_search::SearchDriverInput Inputs::View(const tied::SearchGeometryData& g) const {
    return {positions.data(), masters.data(), g.secondary_working_nodes.data(), positions.size(),
            masters.size(), g.secondary_working_nodes.size(), g.working_length_to_m,
            g.maximum_secondary_shell_thickness};
}
Inputs Pack(const tied::Data& declaration, const tied::PackingData& packing, const tied::SearchGeometryData& g) {
    using output::Require;
    Require(g.canonical_nodes.size() == g.working_positions.size() &&
            g.secondary_working_nodes.size() == declaration.slave_nodes.size() &&
            g.masters.size() == declaration.masters.size() && g.masters.size() == packing.master_rows.size(),
            "Tied assessment geometry/source extent mismatch");
    Inputs out;
    out.positions.reserve(g.working_positions.size());
    out.masters.reserve(g.masters.size());
    for (const auto& x : g.working_positions) out.positions.push_back({x[0],x[1],x[2]});
    for (std::size_t rank = 0; rank < g.masters.size(); ++rank) {
        const auto& m = g.masters[rank];
        Require(m.declaration_row == packing.master_rows[rank] && m.declaration_row < declaration.masters.size(),
                "Tied assessment IRECT/source row mismatch");
        Require(m.family == tied::SearchShellFamily::Q4 || m.family == tied::SearchShellFamily::T3,
                "Invalid tied assessment source family");
        const auto topology = m.family == tied::SearchShellFamily::Q4 ? native_search::MasterTopology::Quad :
            native_search::MasterTopology::TriangleRepeatedThird;
        out.masters.push_back({m.working_nodes, topology, m.bounds_thickness, m.projection_thickness});
    }
    for (std::size_t s = 0; s < g.secondary_working_nodes.size(); ++s) {
        const auto node = g.secondary_working_nodes[s];
        Require(node < g.canonical_nodes.size() && g.canonical_nodes[node] == declaration.slave_nodes[s].canonical_index,
                "Tied assessment NSV/source node mismatch");
    }
    return out;
}
} // namespace crash::cases::vehicle_startup::tied_assessment_detail
