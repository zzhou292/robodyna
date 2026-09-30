#include "Internal.h"
#include <set>

namespace crash::modelio::physical_scope::detail {
void BuildRoles(const rigid::point_mass::Source& masses, const type13::SourceType13& beams,
                const solid_source::VehicleSolidSource& solids, const beam18::Source* structural,
                const std::vector<SourceId>& nodes, Data& data) {
    const auto& plan = masses.rigid_source().source();
    const auto& canonical = plan.canonical().data();
    data.node_roles.assign(nodes.size(), 0);
    for (auto index : plan.canonical_nodes()) {
        Require(index < nodes.size(), "Selected shell node index exceeds canonical inventory");
        data.node_roles[index] |= Shell;
    }
    for (auto index : solids.data().canonical_nodes) {
        Require(index < nodes.size(), "Selected solid node index exceeds canonical inventory");
        data.node_roles[index] |= Solid;
    }
    {
        const auto records = Decode<std::uint64_t>(canonical, "beams_records");
        Require(records.size() % 10 == 0, "Canonical beam record shape changed");
        for (const auto& beam : beams.data().beams) {
            Require(beam.canonical_index < records.size() / 10, "TYPE13 canonical beam index changed");
            const auto* record = records.data() + beam.canonical_index * 10;
            Require(record[0] == beam.id && record[1] == 2000486,
                    "TYPE13 original EID/PID association changed");
            for (unsigned slot = 0; slot < 3; ++slot) {
                Require(beam.node_indices[slot] < beams.data().nodes.size(), "TYPE13 node index exceeds source");
                const auto id = beams.data().nodes[beam.node_indices[slot]].id;
                Require(record[slot + 2] == id, "TYPE13 canonical endpoint/orientation association changed");
                Mark(data.node_roles, nodes, id, slot == 2 ? BeamOrientation : Type13Endpoint);
            }
        }
    }
    if(structural) BuildBeamRoles(*structural,nodes,data);
    for (const auto& selected : masses.data().consumed) {
        Require(selected.record < masses.data().records.size(), "Point-mass source record index changed");
        Mark(data.node_roles, nodes, masses.data().records[selected.record].value.source_node_id, RetainedPointMass);
    }
    {
        const auto records = Decode<std::uint64_t>(canonical, "shells_records");
        Require(records.size() % 6 == 0, "Canonical shell record shape changed");
        for (const auto& parent : plan.parents()) {
            const auto* body = masses.rigid_source().body(parent.part_index);
            if (!body) continue;
            Require(parent.canonical_parent < records.size() / 6 &&
                records[parent.canonical_parent * 6 + 1] == body->source_part_id,
                "Rigid skin original parent/part association changed");
            data.rigid_skin.push_back({records[parent.canonical_parent * 6], body->source_part_id,
                parent.canonical_parent, masses.rigid_source().root_index(parent.part_index)});
        }
        for (std::size_t i = 0; i < records.size(); i += 6) {
            if (!std::binary_search(canonical.excluded_parts.begin(), canonical.excluded_parts.end(), records[i + 1])) continue;
            for (unsigned slot = 0; slot < 4; ++slot)
                Mark(data.node_roles, nodes, records[i + 2 + slot], ExcludedTireShell);
        }
    }
    Require(data.rigid_skin.size() == masses.rigid_source().data().shell_count,
            "Rigid skin complete original parent coverage changed");
    for (const auto& weld : data.spotwelds) {
        data.counts.optional_spotwelds += !weld.default_only;
        for (auto id : weld.nodes) Mark(data.node_roles, nodes, id, ProvisionalType25);
    }
    for (auto roles : data.node_roles) {
        data.counts.baseline_nodes += (roles & PhysicalRoles) != 0;
        data.counts.with_type25_nodes += (roles & (PhysicalRoles | ProvisionalType25)) != 0;
        for (unsigned bit = 0; bit < 8; ++bit) data.counts.role_nodes[bit] += (roles & (1u << bit)) != 0;
    }
    data.counts.type25_added_nodes = data.counts.with_type25_nodes - data.counts.baseline_nodes;
    std::vector<std::size_t> roots(masses.data().records.size(), SIZE_MAX);
    for (const auto& selected : masses.data().consumed) roots[selected.record] = selected.root_index;
    for (std::size_t i = 0; i < masses.data().records.size(); ++i) {
        const auto& source = masses.data().records[i].value;
        const auto roles = data.node_roles[NodeIndex(nodes, source.source_node_id)];
        data.point_masses.push_back({source.source_element_id, source.source_node_id, i, roots[i], roles});
        if (roots[i] == SIZE_MAX) {
            data.counts.outside_mass_in_baseline += (roles & PhysicalRoles) != 0;
            data.counts.outside_mass_with_type25 += (roles & (PhysicalRoles | ProvisionalType25)) != 0;
        }
    }
}
} // namespace crash::modelio::physical_scope::detail
