#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>

namespace crash::modelio::tied_shell::classification_detail {
bool Observed(const Data& declaration, SourceId id) {
    const auto& nodes = declaration.slave_nodes;
    const auto found = std::lower_bound(nodes.begin(), nodes.end(), id,
        [](const Node& node, SourceId value) { return node.id < value; });
    return found != nodes.end() && found->id == id;
}
namespace {
void Away(const Data& declaration, SourceId id) {
    Require(!Observed(declaration, id), "Original constraint affects an observed tied slave");
}
void GroupOptions(const SourceEvidence& source, const Data& declaration) {
    const bool title = source.block.keyword == "*CONSTRAINED_NODAL_RIGID_BODY_TITLE";
    Require(title || source.block.keyword == "*CONSTRAINED_NODAL_RIGID_BODY",
            "Unresolved original rigid-group keyword");
    Require(source.cards.size() == 1 + std::size_t(title), "Unresolved original rigid-group card shape");
    const auto& row = source.cards[title].second;
    Require(row.size() <= 80, "Unresolved original rigid-group tail");
    for (unsigned column = 0; column < 8; ++column) {
        const auto value = vehicle::detail::SourceScalar(row, column);
        if (column == 0 || column == 2) {
            (void)detail::CardId(row, column);
        } else if (column == 3 && value && *value != 0) {
            // The converter adds PNODE to the slave SET and creates a separate
            // fresh main node. PNODE is not an alias for that generated node.
            Away(declaration, detail::CardId(row, column));
        } else {
            Require(!value || *value == 0, "Unresolved original rigid-group option");
        }
    }
}
void Joint(const SourceEvidence& source, const Data& declaration, ClassificationSourceReceipt& receipt) {
    const auto& key = source.block.keyword;
    Require(key == "*CONSTRAINED_JOINT_CYLINDRICAL_ID" ||
            key == "*CONSTRAINED_JOINT_REVOLUTE_ID" || key == "*CONSTRAINED_JOINT_SPHERICAL_ID",
            "Unresolved original joint keyword");
    Require(source.cards.size() == 2, "Unresolved original joint card shape");
    const auto& row = source.cards[1].second;
    // Keep the complete original N1..N5 read set even when TYPE45 consumes only
    // the first two or three entries. Source RPS/damping are not IKINE aliases.
    for (unsigned column = 0; column < 5; ++column) {
        const auto value = vehicle::detail::SourceScalar(row, column);
        if (!value || *value == 0) continue;
        Away(declaration, detail::CardId(row, column));
        ++receipt.checked_joint_node_fields;
    }
    ++receipt.joints;
}
}
void ReadSet(const Data& declaration, const AuxiliaryData& auxiliary,
             const vehicle::rigid_part::SourceData& rigid, ClassificationSourceReceipt& receipt) {
    SourceId previous = 0;
    for (const auto& node : declaration.slave_nodes) {
        Require(node.id > previous && node.source_codes[0] == 0 && node.source_codes[1] == 0,
                "Observed source slave identity or TC/RC is outside the projection profile");
        previous = node.id;
    }
    receipt.observed_slaves = declaration.slave_nodes.size();
    for (const auto& group : declaration.groups) {
        Require(group.source < declaration.sources.size(), "Missing original rigid-group evidence");
        GroupOptions(declaration.sources[group.source], declaration);
        for (const auto id : group.source_nodes) Away(declaration, id);
        ++receipt.plain_groups;
    }
    for (const auto& group : auxiliary.groups) {
        Require(group.evidence.source < auxiliary.sources.size(), "Missing auxiliary rigid-group evidence");
        GroupOptions(auxiliary.sources[group.evidence.source], declaration);
        for (const auto id : group.evidence.source_nodes) Away(declaration, id);
        ++receipt.auxiliary_groups;
    }
    for (const auto& source : auxiliary.sources) {
        if (source.block.keyword != "*ELEMENT_SOLID") continue;
        for (const auto& card : source.cards) {
            if (card.second.empty()) continue;
            Require(assembly::reader::auxiliary::BlankTail(card.second, 80),
                    "Auxiliary solid has unresolved high-order node fields");
            for (unsigned column = 0; column < 10; ++column)
                (void)assembly::reader::auxiliary::Id(card.second, 8 * column, 8);
        }
    }
    for (const auto id : rigid.plain_rigid_members) Away(declaration, id);
    for (const auto& body : rigid.bodies) {
        for (const auto id : body.part_nodes) Away(declaration, id);
        for (const auto id : body.extra_nodes) Away(declaration, id);
        ++receipt.rigid_parts;
    }
    for (const auto& source : rigid.sources) {
        if (source.block.keyword.rfind("*CONSTRAINED_NODAL_RIGID_BODY", 0) == 0)
            GroupOptions(source, declaration);
        if (source.block.keyword.rfind("*CONSTRAINED_JOINT_", 0) == 0)
            Joint(source, declaration, receipt);
    }
}
}
