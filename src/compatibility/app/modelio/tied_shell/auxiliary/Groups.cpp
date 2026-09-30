#include "Internal.h"
#include <algorithm>

namespace crash::modelio::tied_shell::auxiliary_detail {
void Groups(Draft& draft, const Data& declaration, AuxiliaryLimits limits) {
    std::map<SourceId, std::size_t> sets;
    std::set<SourceId> group_ids;
    for (const auto& group : declaration.groups) group_ids.insert(group.id);
    for (std::size_t i = 0; i < draft.data.sources.size(); ++i) {
        const auto& evidence = draft.data.sources[i];
        if (evidence.block.keyword.rfind("*SET_NODE_", 0) != 0) continue;
        const bool title = evidence.block.keyword.size() >= 6 &&
                           evidence.block.keyword.substr(evidence.block.keyword.size()-6) == "_TITLE";
        Require(evidence.cards.size() > std::size_t(title), "Missing auxiliary node-set header");
        const auto id = detail::CardId(evidence.cards[title].second, 0);
        Require(sets.emplace(id, i).second, "Duplicate auxiliary node-set identity");
    }
    Limits list_limits;
    list_limits.group_members = limits.group_members;
    auto& d = draft.data;
    for (std::size_t i = 0; i < d.sources.size(); ++i) {
        const auto& source = d.sources[i];
        const bool title = source.block.keyword == "*CONSTRAINED_NODAL_RIGID_BODY_TITLE";
        if (!title && source.block.keyword != "*CONSTRAINED_NODAL_RIGID_BODY") continue;
        Require(source.cards.size() == std::size_t(title)+1, "Unsupported auxiliary rigid card shape");
        const auto& card = source.cards[title].second;
        Require(card.size() <= 80, "Auxiliary rigid card exceeds eight fields");
        AuxiliaryGroup group;
        auto& g = group.evidence;
        g.id = detail::CardId(card, 0);
        g.node_set_id = detail::CardId(card, 2);
        g.source = i;
        Require(group_ids.insert(g.id).second, "Duplicate main/auxiliary rigid-group identity");
        for (unsigned column = 0; column < 8; ++column) {
            group.source_fields[column] = vehicle::detail::SourceScalar(card, column);
            if (column != 0 && column != 2)
                Require(!group.source_fields[column] || *group.source_fields[column] == 0,
                        "Unresolved auxiliary rigid-group option");
        }
        const auto found = sets.find(g.node_set_id);
        Require(found != sets.end(), "Missing auxiliary rigid node set");
        g.node_set_source = found->second;
        const auto& nodes = d.sources[g.node_set_source];
        const bool set_title = nodes.block.keyword == "*SET_NODE_LIST_TITLE";
        Require(set_title || nodes.block.keyword == "*SET_NODE_LIST", "Unresolved auxiliary node-set operator");
        for (unsigned column = 1; column < 8; ++column)
            Require(!vehicle::detail::SourceScalar(nodes.cards[set_title].second, column),
                    "Unresolved auxiliary node-set header option");
        g.source_nodes = detail::ListIds(nodes, set_title, list_limits);
        Require(g.source_nodes.size() <= limits.group_members-d.counts.member_occurrences,
                "Auxiliary member occurrence cap exceeded");
        d.counts.member_occurrences += g.source_nodes.size();
        draft.members.insert(g.source_nodes.begin(), g.source_nodes.end());
        d.groups.push_back(std::move(group));
    }
    d.counts.groups = d.groups.size();
    d.counts.distinct_members = draft.members.size();
}
} // namespace crash::modelio::tied_shell::auxiliary_detail
