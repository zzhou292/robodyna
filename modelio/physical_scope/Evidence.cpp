#include "Internal.h"
#include <set>

namespace crash::modelio::physical_scope::detail {
void BuildEvidence(const source::CanonicalData& canonical, const solid_source::VehicleSolidSource& solids, const beam18::Source* beams,
                   const std::vector<SourceId>& nodes, Data& data, Limits limits) {
    std::set<SourceId> requested;
    for (const auto* groups : {&data.plain_groups, &data.part_roots})
        for (const auto& group : *groups)
            for (const auto& member : group.members) requested.insert(member.node);
    for (const auto& weld : data.spotwelds) requested.insert(weld.nodes.begin(), weld.nodes.end());
    for (const auto& mass : data.point_masses) requested.insert(mass.node);
    if (beams) for (const auto& node : beams->data().nodes) requested.insert(node.id);
    Require(requested.size() <= limits.evidence_nodes, "Physical source evidence node cap exceeded");
    std::vector<std::uint32_t> evidence_index(nodes.size(), UINT32_MAX);
    data.evidence.reserve(requested.size());
    for (auto id : requested) {
        const auto index = NodeIndex(nodes, id);
        evidence_index[index] = data.evidence.size();
        data.evidence.push_back({id, data.node_roles[index], {}});
    }
    std::set<SourceId> selected_solids;
    for (const auto& row : solids.data().rows) selected_solids.insert(row.element_id);
    std::set<SourceId> selected_beams;
    if(beams) for(const auto& row:beams->data().rows) selected_beams.insert(row.element_id);
    std::size_t incidence_count = 0;
    const auto scan = [&](const char* record_name, const char* index_name, unsigned width,
                          unsigned slots, Family family) {
        const auto records = Decode<std::uint64_t>(canonical, record_name);
        const auto indices = Decode<std::uint32_t>(canonical, index_name);
        const unsigned indexed_slots = family == Family::Beam ? 2 : slots;
        Require(records.size() % width == 0 && indices.size() == records.size() / width * indexed_slots,
                "Physical source incidence extent changed");
        for (std::size_t row = 0; row < records.size() / width; ++row) {
            const auto* record = records.data() + row * width;
            bool selected = false;
            if (family == Family::Shell)
                selected = std::binary_search(canonical.selected_parts.begin(), canonical.selected_parts.end(), record[1]);
            else if (family == Family::Beam) selected = record[1] == 2000486 || selected_beams.count(record[0]) != 0;
            else selected = selected_solids.count(record[0]) != 0;
            for (unsigned slot = 0; slot < slots; ++slot) {
                // Beam N3=0 is an absent orientation field, not a physical node.
                if (family == Family::Beam && slot == 2 && record[slot + 2] == 0) continue;
                const auto index = slot < indexed_slots ? indices[row * indexed_slots + slot] :
                    NodeIndex(nodes, record[slot + 2]);
                Require(index < nodes.size() && nodes[index] == record[slot + 2],
                        "Physical incidence NID/index association changed");
                const auto evidence = evidence_index[index];
                if (evidence == UINT32_MAX) continue;
                auto& entries = data.evidence[evidence].incidence;
                const bool orientation = family == Family::Beam && slot == 2;
                if (!entries.empty() && entries.back().family == family && entries.back().canonical_row == row &&
                    entries.back().orientation_only == orientation) {
                    entries.back().local_slots |= 1u << slot;
                    continue;
                }
                Require(incidence_count < limits.incidences, "Physical source incidence count exceeds cap");
                ++incidence_count;
                entries.push_back({record[0], record[1], static_cast<std::uint32_t>(row),
                    static_cast<std::uint16_t>(1u << slot), family, selected && !orientation, orientation});
            }
        }
    };
    scan("shells_records", "shells_node_indices", 6, 4, Family::Shell);
    scan("beams_records", "beams_node_indices", 10, 3, Family::Beam);
    scan("solids_records", "solids_node_indices", 10, 8, Family::Solid);
}
} // namespace crash::modelio::physical_scope::detail
