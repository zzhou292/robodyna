#include "Internal.h"
#include <set>

namespace crash::modelio::physical_scope::detail {
std::size_t NodeIndex(const std::vector<SourceId>& nodes, SourceId id) {
    const auto found = std::lower_bound(nodes.begin(), nodes.end(), id);
    Require(found != nodes.end() && *found == id, "Physical source node is absent from canonical inventory");
    return static_cast<std::size_t>(found - nodes.begin());
}
void Mark(std::vector<std::uint16_t>& roles, const std::vector<SourceId>& nodes, SourceId id, Role role) {
    roles[NodeIndex(nodes, id)] |= role;
}
void Classify(Group& group) {
    Require(!group.members.empty(), "Empty physical coverage group");
    group.covered_before = group.covered_after = group.tire_members = 0;
    std::set<SourceId> seen;
    for (const auto& member : group.members) {
        Require(member.node && seen.insert(member.node).second && (member.roles & ~127u) == 0,
                "Invalid coverage member identity or role");
        group.covered_before += (member.roles & PhysicalRoles) != 0;
        group.covered_after += (member.roles & (PhysicalRoles | ProvisionalType25)) != 0;
        group.tire_members += (member.roles & ExcludedTireShell) != 0;
    }
    const auto state = [&](std::size_t count) {
        return count == group.members.size() ? Coverage::Complete :
               count ? Coverage::Partial : Coverage::None;
    };
    group.before = state(group.covered_before);
    group.after = state(group.covered_after);
}
void BuildGroups(const rigid::RigidPartSource& rigid, const tied_shell::TiedShellDeclaration& tied,
                 const std::vector<SourceId>& nodes, Data& data, Limits limits) {
    std::vector<SourceId> plain;
    std::size_t members = 0;
    const auto append = [&](Group& group, const SourceId* ids, std::size_t count) {
        Require(count <= limits.members - members, "Physical coverage group member cap exceeded");
        members += count;
        group.members.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
            group.members.push_back({ids[i], data.node_roles[NodeIndex(nodes, ids[i])]});
        Classify(group);
    };
    for (const auto& source : tied.data().groups) {
        Require(data.plain_groups.size() < limits.groups, "Physical coverage group count exceeds cap");
        Group group;
        group.id = source.id;
        group.node_set_id = source.node_set_id;
        group.source_index = source.source;
        append(group, source.source_nodes.data(), source.source_nodes.size());
        plain.insert(plain.end(), source.source_nodes.begin(), source.source_nodes.end());
        data.plain_groups.push_back(std::move(group));
    }
    // Source inventories retain their own order. Only copies are sorted for
    // agreement; no ordering assumption is borrowed from a declaration table.
    auto expected = rigid.data().plain_rigid_members;
    std::sort(plain.begin(), plain.end());
    std::sort(expected.begin(), expected.end());
    Require(plain == expected, "Plain-group complete source inventory changed");
    const auto& topology = rigid.topology();
    for (std::size_t i = 0; i < topology.root_count(); ++i) {
        const auto& root = topology.roots()[i];
        Group group;
        group.id = topology.parts()[root.part_index].source_part_id;
        group.source_index = root.part_index;
        if (root.child_part_index != SIZE_MAX)
            group.child_part_id = topology.parts()[root.child_part_index].source_part_id;
        append(group, topology.root_members() + root.member_offset, root.member_count);
        data.part_roots.push_back(std::move(group));
    }
    for (const auto& group : data.plain_groups) {
        data.counts.plain_complete_before += group.before == Coverage::Complete;
        data.counts.plain_complete_after += group.after == Coverage::Complete;
    }
    for (const auto& group : data.part_roots) {
        data.counts.roots_complete_before += group.before == Coverage::Complete;
        data.counts.roots_complete_after += group.after == Coverage::Complete;
    }
}
} // namespace crash::modelio::physical_scope::detail
