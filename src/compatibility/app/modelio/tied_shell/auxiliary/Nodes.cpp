#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>
#include <cmath>

namespace crash::modelio::tied_shell::auxiliary_detail {
void Nodes(Draft& draft, const source::CanonicalData& canonical, const Data& declaration, AuxiliaryLimits limits) {
    const auto& id_array = source::FindArray(canonical, "node_ids");
    const auto& code_array = source::FindArray(canonical, "node_codes");
    const auto ids = output::arrays::Decode<SourceId>(id_array.descriptor, id_array.bytes);
    const auto codes = output::arrays::Decode<std::int32_t>(code_array.descriptor, code_array.bytes);
    Require(ids.size() == canonical.canonical_nodes && codes.size() == 2*ids.size(),
            "Auxiliary canonical node extent changed");
    std::vector<std::pair<SourceId, std::uint32_t>> order;
    order.reserve(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) order.emplace_back(ids[i], static_cast<std::uint32_t>(i));
    std::sort(order.begin(), order.end());
    const auto find = [&](SourceId id) {
        return std::lower_bound(order.begin(), order.end(), id,
            [](const auto& row, SourceId value) { return row.first < value; });
    };
    std::map<SourceId, AuxiliaryMemberNode> auxiliary_nodes;
    for (std::size_t i = 0; i < draft.data.sources.size(); ++i) {
        const auto& source = draft.data.sources[i];
        if (source.block.keyword != "*NODE") continue;
        for (const auto& [line, card] : source.cards) {
            if (card.empty()) continue;
            AuxiliaryMemberNode node;
            node.id = auxiliary::Id(card, 0, 8);
            const auto existing = find(node.id);
            Require(existing == order.end() || existing->first != node.id, "Auxiliary node duplicates canonical identity");
            Require(card.size() <= 72, "Unsupported auxiliary node card width");
            node.origin = AuxiliaryNodeOrigin::AuxiliaryMember;
            node.source = i;
            node.source_line = line;
            for (unsigned code = 0; code < 2; ++code) {
                const auto value = vehicle::detail::SourceScalar(card, 7+code, 8);
                if (!value) node.auxiliary_code_blank_mask |= 1u << code;
                else {
                    Require(std::trunc(*value) == *value && *value >= INT32_MIN && *value <= INT32_MAX,
                            "Invalid auxiliary source node code");
                    node.source_codes[code] = static_cast<std::int32_t>(*value);
                }
            }
            Require(auxiliary_nodes.size() < limits.group_members && auxiliary_nodes.emplace(node.id, node).second,
                    "Duplicate or excessive auxiliary node declarations");
        }
    }
    for (const auto index : declaration.master_nodes) {
        Require(index < ids.size(), "Auxiliary master node association changed");
    }
    const auto master = [&](SourceId id) {
        const auto row = std::lower_bound(declaration.master_nodes.begin(), declaration.master_nodes.end(), id,
            [&](std::uint32_t index, SourceId value) { return ids[index] < value; });
        return row != declaration.master_nodes.end() && ids[*row] == id;
    };
    const auto slave = [&](SourceId id) {
        const auto row = std::lower_bound(declaration.slave_nodes.begin(), declaration.slave_nodes.end(), id,
            [](const Node& node, SourceId value) { return node.id < value; });
        return row != declaration.slave_nodes.end() && row->id == id;
    };
    auto& d = draft.data;
    for (const auto id : draft.members) {
        AuxiliaryMemberNode node;
        const auto row = find(id);
        if (row != order.end() && row->first == id) {
            node.id = id;
            node.canonical_index = row->second;
            node.source_codes = {codes[2*row->second], codes[2*row->second+1]};
        } else {
            const auto found = auxiliary_nodes.find(id);
            Require(found != auxiliary_nodes.end(), "Auxiliary group member has no source node declaration");
            node = found->second;
            ++d.counts.auxiliary_member_nodes;
        }
        node.master = master(id);
        node.slave = slave(id);
        d.counts.distinct_master_members += node.master;
        d.counts.distinct_slave_members += node.slave;
        d.member_nodes.push_back(node);
    }
    for (auto& group : d.groups) {
        auto& g = group.evidence;
        for (const auto id : g.source_nodes) {
            if (master(id)) g.master_nodes.push_back(id);
            if (slave(id)) g.slave_nodes.push_back(id);
        }
        d.counts.groups_touching_masters += !g.master_nodes.empty();
        d.counts.groups_touching_slaves += !g.slave_nodes.empty();
    }
}
} // namespace crash::modelio::tied_shell::auxiliary_detail
