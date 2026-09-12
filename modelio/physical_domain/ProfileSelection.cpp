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
        if (!RequiresCompleteGroup(policy, group.id)) continue;
        const auto& selected = selection.groups[i];
        const bool complete_group = selected.source_group == i && selected.disposition == GroupDisposition::Complete &&
            selected.case_node_set_id == group.node_set_id && selected.excluded_members.empty() &&
            selected.members.size() == group.members.size();
        if (!complete_group)
            throw std::runtime_error("Extended solids require complete original rigid group " + std::to_string(group.id));
        for (std::size_t n = 0; n < group.members.size(); ++n)
            output::Require(selected.members[n] == group.members[n].node,
                            "Extended rigid source member order changed");
        ++complete;
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
    output::Require(added == 2 && selection.counts.retained_point_masses == 150,
                    "Extended physical point-card census changed");
}
} // namespace crash::modelio::physical_domain::detail
