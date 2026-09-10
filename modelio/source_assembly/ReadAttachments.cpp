#include "JsonReader.h"
#include <algorithm>
#include <charconv>
#include <set>

namespace crash::modelio::assembly::reader {
namespace {
SourceId FieldId(const std::string& field) {
    const auto first = field.find_first_not_of(" \t");
    const auto last = field.find_last_not_of(" \t");
    Require(first != std::string::npos, "Required attachment ID is blank");
    SourceId id = 0;
    const auto parsed = std::from_chars(field.data() + first, field.data() + last + 1, id);
    Require(parsed.ec == std::errc{} && parsed.ptr == field.data() + last + 1 && id, "Invalid raw attachment source ID");
    return id;
}
std::vector<PartMembership> Membership(const Value& value, const ReadLimits& limits, const Data& data) {
    std::vector<PartMembership> result;
    std::set<SourceId> seen;
    for (const auto& row : Array(value, "selected_membership", limits.parts, 1).GetArray()) {
        PartMembership membership;
        membership.part_id = Unsigned(row, "part_id");
        membership.node_ids = Ids(Member(row, "node_ids"), limits.group_members);
        const auto found = std::find_if(data.parts.begin(), data.parts.end(), [&](const auto& p) { return p.id == membership.part_id; });
        Require(found != data.parts.end() && seen.insert(membership.part_id).second && !membership.node_ids.empty(),
                "Invalid attachment selected part membership");
        for (auto id : membership.node_ids) {
            const auto node = NodeIndex(data, id);
            Require(std::find(found->nodes.begin(), found->nodes.end(), node) != found->nodes.end(),
                    "Attachment node does not belong to selected source part");
        }
        result.push_back(std::move(membership));
    }
    return result;
}
void CheckMembers(const std::vector<SourceId>& members, const std::vector<SourceId>& external,
                  const std::vector<PartMembership>& selected, const Data& data,
                  std::vector<std::size_t>* selected_indices = nullptr) {
    std::set<SourceId> expected_external(external.begin(), external.end()), expected_selected;
    for (const auto& part : selected) expected_selected.insert(part.node_ids.begin(), part.node_ids.end());
    std::set<SourceId> actual_external, actual_selected;
    for (auto id : members) {
        const auto found = std::lower_bound(data.nodes.begin(), data.nodes.end(), id,
            [](const auto& node, SourceId source) { return node.source_id < source; });
        if (found != data.nodes.end() && found->source_id == id) {
            actual_selected.insert(id);
            if (selected_indices) selected_indices->push_back(static_cast<std::size_t>(found - data.nodes.begin()));
        } else actual_external.insert(id);
    }
    Require(actual_selected == expected_selected && actual_external == expected_external,
            "Attachment internal/external source membership changed");
}
std::vector<ReleasedTiedScope> TiedScopes(const Value& boundary, const char* key, const ReadLimits& limits, const Data& data) {
    std::vector<ReleasedTiedScope> result;
    std::set<std::pair<std::string, std::size_t>> identities;
    for (const auto& value : Array(boundary, key, limits.groups).GetArray()) {
        ReleasedTiedScope scope;
        const auto& identity = Array(value, "source_identity", 2, 2);
        scope.filename = Text(identity[0]); scope.source_line = Unsigned(identity[1], SIZE_MAX);
        scope.reason = Text(value, "reason"); Flag(value, "actual_pairing_qualified", false);
        scope.selected_master_parts = Ids(Member(value, "selected_master_part_ids"), limits.parts, true);
        scope.selected_slave_parts = Ids(Member(value, "selected_slave_part_ids"), limits.parts, true);
        Require(scope.source_line && identities.emplace(scope.filename, scope.source_line).second &&
            scope.selected_master_parts.empty() != scope.selected_slave_parts.empty(), "Invalid released tied source scope");
        for (const auto* ids : {&scope.selected_master_parts, &scope.selected_slave_parts})
            for (auto id : *ids) Require(std::any_of(data.parts.begin(), data.parts.end(), [=](const auto& p) { return p.id == id; }),
                                       "Released tie references an unselected part");
        result.push_back(std::move(scope));
    }
    return result;
}
}  // namespace
void ReadAttachments(const Value& document, const ReadLimits& limits, Data& data) {
    const auto& attachments = Member(document, "attachments");
    Flag(attachments, "attachment_mechanics_implemented", false); Flag(attachments, "full_attachment_closure_qualified", false);
    Flag(attachments, "literal_nodal_rigid_and_spotweld_frontier_validated", true);
    std::set<SourceId> group_ids, node_set_ids, internal_members, outgoing_groups, weld_ids, outgoing_welds, all_external;
    for (const auto& value : Array(attachments, "nodal_rigid_groups", limits.groups).GetArray()) {
        NodalRigidGroup group;
        const auto classification = Text(value, "classification");
        Require(classification == "internal" || classification == "outgoing", "Unsupported rigid group classification");
        group.internal = classification == "internal";
        const auto& rigid = Member(value, "rigid"); const auto& node_set = Member(value, "node_set");
        group.id = Unsigned(rigid, "identity"); group.node_set_id = Unsigned(rigid, "node_set_id");
        Require(group.id && group.node_set_id && group_ids.insert(group.id).second && node_set_ids.insert(group.node_set_id).second &&
            Unsigned(node_set, "identity") == group.node_set_id, "Repeated/mismatched rigid group identity");
        TextIs(node_set, "namespace", "node_set");
        group.rigid_source = Block(Member(rigid, "source")); group.node_set_source = Block(Member(node_set, "source"));
        group.rigid_cards = AttachmentCards(rigid, 1); group.node_set_cards = AttachmentCards(node_set, limits.group_members / 8 + 2);
        Require(group.rigid_source.keyword == "*CONSTRAINED_NODAL_RIGID_BODY" && group.node_set_source.keyword == "*SET_NODE_LIST" &&
            group.rigid_cards[0].blank_mask == 250 && group.node_set_cards[0].blank_mask == 254,
            "Rigid source subset requires PID/NSID only and plain node set");
        Require(FieldId(group.rigid_cards[0].fields[0]) == group.id && FieldId(group.rigid_cards[0].fields[2]) == group.node_set_id &&
            FieldId(group.node_set_cards[0].fields[0]) == group.node_set_id, "Rigid card fields disagree with declared IDs");
        group.members = Ids(Member(node_set, "members"), limits.group_members);
        Require(group.members.size() >= 3, "Rigid group must retain at least three supplied members");
        std::vector<SourceId> raw_members;
        for (std::size_t i = 1; i < group.node_set_cards.size(); ++i)
            for (unsigned field = 0; field < 8; ++field)
                if (!(group.node_set_cards[i].blank_mask & (1u << field))) raw_members.push_back(FieldId(group.node_set_cards[i].fields[field]));
        Require(raw_members == group.members, "Rigid node-set source order changed");
        group.external_nodes = Ids(Member(value, "external_node_ids"), limits.external_nodes, true);
        group.selected_membership = Membership(value, limits, data);
        CheckMembers(group.members, group.external_nodes, group.selected_membership, data, &group.selected_global_nodes);
        Require(group.internal == group.external_nodes.empty(), "Rigid classification disagrees with complete membership");
        if (group.internal) {
            for (auto id : group.members) Require(internal_members.insert(id).second, "Active source nodal-rigid groups overlap");
        } else outgoing_groups.insert(group.id);
        all_external.insert(group.external_nodes.begin(), group.external_nodes.end());
        data.nodal_rigid_groups.push_back(std::move(group));
    }
    for (const auto& value : Array(attachments, "spotwelds", limits.spotwelds).GetArray()) {
        const auto classification = Text(value, "classification");
        Require(classification == "internal" || classification == "outgoing", "Unsupported spotweld classification");
        const bool internal = classification == "internal";
        const auto& weld = Member(value, "weld");
        Spotweld output;
        output.id = Unsigned(weld, "identity"); output.filename = Text(weld, "filename");
        output.keyword_line = Unsigned(weld, "keyword_line", SIZE_MAX); Flag(weld, "mechanics_qualified", false);
        const auto nodes = Ids(Member(weld, "node_ids"), 2);
        Require(nodes.size() == 2 && nodes[0] != nodes[1], "Spotweld needs two distinct source nodes");
        output.node_ids = {nodes[0], nodes[1]}; output.cards = AttachmentCards(weld, 2);
        Require(output.id && output.keyword_line && weld_ids.insert(output.id).second && output.cards.size() == 2 &&
            output.cards[0].blank_mask == 254 && output.cards[1].blank_mask == 252 &&
            FieldId(output.cards[0].fields[0]) == output.id && FieldId(output.cards[1].fields[0]) == nodes[0] &&
            FieldId(output.cards[1].fields[1]) == nodes[1], "Unsupported/repeated source spotweld record");
        output.external_nodes = Ids(Member(value, "external_node_ids"), limits.external_nodes, true);
        output.selected_membership = Membership(value, limits, data);
        CheckMembers(nodes, output.external_nodes, output.selected_membership, data);
        Require(internal == output.external_nodes.empty(), "Spotweld classification disagrees with complete membership");
        all_external.insert(output.external_nodes.begin(), output.external_nodes.end());
        if (internal) {
            const std::array<std::size_t, 2> indices{NodeIndex(data, nodes[0]), NodeIndex(data, nodes[1])};
            data.internal_spotwelds.push_back({std::move(output), indices});
        } else {
            outgoing_welds.insert(output.id);
            data.released_spotwelds.push_back(std::move(output));
        }
    }
    const auto& boundary = Member(document, "boundary"); auto& retained = data.boundary;
    TextIs(boundary, "policy", "released_external_connections"); retained.policy = Text(boundary, "policy");
    Flag(boundary, "applied_to_dynamics", false); Flag(boundary, "mass_removed_from_selected_parts", false);
    retained.interpretation = Text(boundary, "interpretation"); retained.unresolved_tied_scope = Text(boundary, "unresolved_tied_scope");
    retained.nodal_rigid_ids = Ids(Member(boundary, "outgoing_nodal_rigid_ids"), limits.groups, true);
    retained.spotweld_ids = Ids(Member(boundary, "outgoing_spotweld_ids"), limits.spotwelds, true);
    Require(std::vector<SourceId>(outgoing_groups.begin(), outgoing_groups.end()) == retained.nodal_rigid_ids &&
        std::vector<SourceId>(outgoing_welds.begin(), outgoing_welds.end()) == retained.spotweld_ids,
        "Released boundary omits or changes an outgoing connection");
    retained.external_node_ids = Ids(Member(attachments, "external_node_ids"), limits.external_nodes, true);
    Require(std::vector<SourceId>(all_external.begin(), all_external.end()) == retained.external_node_ids,
        "Released external-node inventory changed");
    for (const auto& part : Array(attachments, "external_parts", limits.external_parts).GetArray()) {
        const auto id = Unsigned(part, "part_id");
        Require(id && (retained.external_part_ids.empty() || id > retained.external_part_ids.back()) &&
            std::none_of(data.parts.begin(), data.parts.end(), [=](const auto& p) { return p.id == id; }),
            "Released external part is duplicate or selected");
        retained.external_part_ids.push_back(id);
    }
    retained.tied_scopes = TiedScopes(boundary, "released_external_tied_source_scopes", limits, data);
    const auto outgoing_ties = TiedScopes(boundary, "outgoing_tied_source_scopes", limits, data);
    Require(outgoing_ties.size() == retained.tied_scopes.size(), "Released tied scope count changed");
    for (std::size_t i = 0; i < outgoing_ties.size(); ++i) {
        const auto& a = outgoing_ties[i]; const auto& b = retained.tied_scopes[i];
        Require(a.filename == b.filename && a.source_line == b.source_line && a.reason == b.reason &&
            a.selected_master_parts == b.selected_master_parts && a.selected_slave_parts == b.selected_slave_parts,
            "Released tied source scope changed");
    }
    const auto& frontier = Member(attachments, "frontier_incidence");
    Flag(frontier, "source_records_verified", true); TextIs(frontier, "source_member_sha256", data.member_sha256);
    const auto& frontier_nodes = Array(frontier, "nodes", limits.external_nodes);
    std::vector<SourceId> frontier_ids;
    for (const auto& node : frontier_nodes.GetArray()) frontier_ids.push_back(Unsigned(node, "source_id"));
    Require(frontier_ids == retained.external_node_ids, "Released frontier node evidence changed");
}
}  // namespace crash::modelio::assembly::reader
