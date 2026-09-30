#include "Internal.h"
#include <algorithm>

namespace crash::modelio::physical_domain::detail {
namespace {
bool Physical(std::uint16_t roles) { return roles & (physical_scope::PhysicalRoles | physical_scope::ProvisionalType25); }
constexpr std::uint64_t RestrictedSetPrefix = 0x5952000000000000ULL;
constexpr std::uint64_t SourceSetLimit = 0x1000000000000ULL;
}
Selection Select(const std::vector<physical_scope::Group>& groups,
                 const std::vector<physical_scope::PointMass>& masses) {
    output::Require(groups.size() <= 1024 && masses.size() <= 4096, "Physical domain source selection exceeds caps");
    std::size_t members = 0;
    for (const auto& group : groups) {
        output::Require(group.members.size() <= 32768 - members, "Physical rigid source member cap exceeded");
        output::Require(group.node_set_id && group.node_set_id < SourceSetLimit,
                        "Original rigid node-set identity overlaps the generated case namespace");
        members += group.members.size();
    }
    Selection result;
    result.groups.reserve(groups.size());
    std::set<std::uint64_t> all_point_nodes;
    for (const auto& mass : masses) {
        all_point_nodes.insert(mass.node);
        if (Physical(mass.roles)) result.point_nodes.insert(mass.node);
    }
    for (const auto& group : groups)
        if (std::any_of(group.members.begin(), group.members.end(), [](const auto& m) { return Physical(m.roles); }))
            for (const auto& member : group.members)
                if (all_point_nodes.count(member.node)) result.point_nodes.insert(member.node);
    for (std::size_t row = 0; row < groups.size(); ++row) {
        const auto& source = groups[row];
        GroupSelection selected;
        selected.source_group = row;
        const bool retain = std::any_of(source.members.begin(), source.members.end(),
                                       [](const auto& member) { return Physical(member.roles); });
        for (const auto& member : source.members) {
            output::Require(!(member.roles & ~physical_scope::KnownRoles), "Unknown physical source member role");
            const bool represented = Physical(member.roles) || result.point_nodes.count(member.node);
            (retain && represented ? selected.members : selected.excluded_members).push_back(member.node);
        }
        if (retain) {
            output::Require(selected.members.size() >= 2, "Retained rigid interface has fewer than two physical members");
            if (selected.excluded_members.empty()) {
                selected.disposition = GroupDisposition::Complete;
                selected.case_node_set_id = source.node_set_id;
                ++result.counts.complete_groups;
            } else {
                selected.disposition = GroupDisposition::Restricted;
                selected.case_node_set_id = RestrictedSetPrefix | source.node_set_id;
                ++result.counts.restricted_groups;
            }
            result.counts.plain_members += selected.members.size();
        } else ++result.counts.omitted_groups;
        result.groups.push_back(std::move(selected));
    }
    for (const auto& mass : masses) result.counts.retained_point_masses += result.point_nodes.count(mass.node) != 0;
    return result;
}
} // namespace crash::modelio::physical_domain::detail
