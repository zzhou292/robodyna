#include "Internal.h"
#include <algorithm>

namespace crash::modelio::tied_shell::detail {
namespace {
bool Constraint(const std::string& keyword) {
    return keyword.rfind("*CONSTRAINED_", 0) == 0 || keyword.rfind("*BOUNDARY_", 0) == 0 ||
           keyword.rfind("*RIGIDWALL_", 0) == 0 || keyword == "*CONTROL_CONTACT" ||
           keyword.rfind("*CONTACT_TIED_", 0) == 0;
}
}
void Constraints(const source::CanonicalData&, const Value& scope, const Value& canonical,
                 Draft& draft, Limits limits) {
    auto& d = draft.data;
    const auto& files = Member(canonical, "source_files");
    const auto& originals = Member(Member(files, "yaris-coarse-v1l.key"), "blocks");
    const auto& groups = Array(Member(scope, "connections"), "nodal_rigid_groups", limits.groups);
    std::set<SourceId> unique;
    std::set<std::size_t> group_lines;
    std::size_t members = 0;
    for (const auto& value : groups.GetArray()) {
        GroupEvidence group;
        group.id = Unsigned(value, "identity");
        group.node_set_id = Unsigned(value, "node_set_id");
        Require(group.id && group.node_set_id && unique.insert(group.id).second,
                "Duplicate or zero tied group identity");
        group.source = RequestSource(draft, Member(value, "source"), originals, limits);
        group.node_set_source = RequestSource(draft, Member(value, "node_set_source"), originals, limits);
        Require(group_lines.insert(group.source).second, "Duplicate tied group source block");
        group.source_nodes = Ids(Member(value, "source_node_ids"), limits.group_members);
        Require(group.source_nodes.size() <= limits.group_members-members, "Tied group member count exceeds capacity");
        members += group.source_nodes.size();
        for (const auto node : group.source_nodes) {
            if (std::binary_search(draft.master_ids.begin(), draft.master_ids.end(), node)) group.master_nodes.push_back(node);
            if (std::binary_search(draft.slave_ids.begin(), draft.slave_ids.end(), node)) group.slave_nodes.push_back(node);
        }
        if (!group.slave_nodes.empty()) ++d.counts.groups_touching_slaves;
        if (!group.master_nodes.empty()) ++d.counts.groups_touching_masters;
        d.groups.push_back(std::move(group));
    }
    std::sort(d.groups.begin(), d.groups.end(), [](const auto& a, const auto& b) { return a.source < b.source; });
    std::size_t original_groups = 0;
    for (auto file = files.MemberBegin(); file != files.MemberEnd(); ++file) {
        const auto& blocks = Member(file->value, "blocks");
        Require(blocks.IsArray() && blocks.Size() <= limits.blocks, "Tied constraint inventory exceeds capacity");
        for (const auto& row : blocks.GetArray()) {
            const auto keyword = Text(row, "keyword");
            if (!Constraint(keyword)) continue;
            UnresolvedBlock block{Text(row, "file"), keyword, Text(row, "source_block_sha256"),
                Unsigned(row, "first_line"), Unsigned(row, "last_line")};
            output::arrays::CheckHash(block.sha256);
            Require(block.filename == std::string(file->name.GetString(), file->name.GetStringLength()) &&
                    block.first_line && block.last_line >= block.first_line,
                    "Invalid tied constraint block association");
            if (block.filename == "yaris-coarse-v1l.key") {
                const bool group = keyword == "*CONSTRAINED_NODAL_RIGID_BODY" ||
                                   keyword == "*CONSTRAINED_NODAL_RIGID_BODY_TITLE";
                if (group) {
                    ++original_groups;
                    Require(group_lines.count(block.first_line), "Original rigid-group declaration omitted");
                    continue;
                }
                // Raw block already authenticated by the complete original member;
                // no joint/rigid-extra/BC interpretation is invented here.
                Request request;
                request.evidence.block = {block.filename, keyword, {}, block.sha256,
                    block.first_line, block.last_line};
                draft.requests.emplace(block.first_line, std::move(request));
                block.retained_source = block.first_line;
            }
            Require(d.unresolved_constraints.size() < limits.blocks && draft.requests.size() <= limits.blocks,
                    "Tied constraint block count exceeds capacity");
            d.unresolved_constraints.push_back(std::move(block));
        }
    }
    Require(original_groups == d.groups.size(), "Complete original rigid-group coverage changed");
    d.counts.groups = d.groups.size();
}
} // namespace crash::modelio::tied_shell::detail
