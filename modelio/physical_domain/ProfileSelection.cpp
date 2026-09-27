#include "Internal.h"
#include "Policy.h"

namespace crash::modelio::physical_domain::detail {
void CheckSelection(const std::vector<physical_scope::Group>& groups,
                    const std::vector<physical_scope::PointMass>& masses,
                    const Selection& selection, Policy policy) {
    (void)SolidPolicy(policy);
    if (policy == Policy::RetainedShellAssembliesV1) return;
    output::Require(groups.size() == selection.groups.size(), "Physical group selection extent changed");
    unsigned complete = 0;
    for (std::size_t i = 0; i < groups.size(); ++i) {
        const auto& group = groups[i];
        const bool known_extended = RequiresCompleteGroup(policy, group.id);
        bool beam_support = false;
        if (HasVehicleSupports(policy))
            for (const auto& member : group.members)
                beam_support |= (member.roles & physical_scope::Beam18Endpoint) != 0;
        if (!known_extended && !beam_support) continue;
        const auto& selected = selection.groups[i];
        const bool complete_group = selected.source_group == i && selected.disposition == GroupDisposition::Complete &&
            selected.case_node_set_id == group.node_set_id && selected.excluded_members.empty() &&
            selected.members.size() == group.members.size();
        if (!complete_group)
            throw std::runtime_error("Extended solids require complete original rigid group " + std::to_string(group.id));
        for (std::size_t n = 0; n < group.members.size(); ++n)
            output::Require(selected.members[n] == group.members[n].node,
                            "Extended rigid source member order changed");
        if (known_extended) ++complete;
    }
    output::Require(complete == 6, "Extended physical profile lacks an affected original rigid group");
    unsigned added = 0;
    for (const auto& mass : masses) {
        if (mass.element != 2409447 && mass.element != 2409448) continue;
        const auto expected_node = mass.element == 2409447 ? 2406557u : 2406558u;
        output::Require(mass.node == expected_node && selection.point_nodes.count(mass.node),
                        "Extended antiroll original point-mass association is absent");
        ++added;
    }
    output::Require(added == 2, "Extended physical point-card census changed");
    const bool supports = HasVehicleSupports(policy);
    if (supports) {
        const std::uint64_t ids[]{2409489, 2409491, 2409492, 2409494};
        const std::uint64_t nodes[]{2348766, 2348765, 2348729, 2348802};
        unsigned rod_cards = 0;
        for (const auto& mass : masses) for (unsigned i = 0; i < 4; ++i) {
            if (mass.element != ids[i]) continue;
            output::Require(mass.node == nodes[i] && selection.point_nodes.count(mass.node),
                            "Vehicle support original rod point-mass association is absent");
            ++rod_cards;
        }
        output::Require(rod_cards == 4, "Vehicle support rod point-card census changed");
    }
    output::Require(selection.counts.retained_point_masses == (supports ? 154u : 150u),
                    "Physical profile retained point-card census changed");
}
} // namespace crash::modelio::physical_domain::detail
