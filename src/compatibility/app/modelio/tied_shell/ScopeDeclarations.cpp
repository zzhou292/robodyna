#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>

namespace crash::modelio::tied_shell::detail {
namespace {
using Index = std::map<SourceId, const Value*>;
Index IndexRows(const Value& rows, const char* key, Limits limits) {
    Require(rows.IsArray() && rows.Size() <= limits.blocks, "Tied declaration table exceeds capacity");
    Index result;
    for (const auto& row : rows.GetArray())
        Require(result.emplace(Unsigned(row, key), &row).second, "Duplicate tied source declaration");
    return result;
}
const Value& Find(const Index& rows, SourceId id) {
    const auto found = rows.find(id);
    Require(found != rows.end(), "Missing tied original declaration");
    return *found->second;
}
}
Draft Declarations(const source::CanonicalData& source, const Value& scope,
                   const Value& canonical, Limits limits) {
    const auto& contacts = Array(Member(scope, "connections"), "tied_contacts", 1, 1);
    const auto& contact = contacts[0];
    Flag(contact, "pairing_qualified", false);
    const auto& blocks = Member(Member(Member(canonical, "source_files"), "yaris-coarse-v1l.key"), "blocks");
    Draft draft;
    auto& d = draft.data;
    d.contact_source = RequestSource(draft, Member(contact, "source"), blocks, limits);
    d.slave_set_source = RequestSource(draft, Member(contact, "slave_source"), blocks, limits);
    d.master_set_source = RequestSource(draft, Member(contact, "master_source"), blocks, limits);
    d.slave_part_ids = Ids(Member(contact, "slave_part_ids"), limits.parts);
    d.master_part_ids = Ids(Member(contact, "master_part_ids"), limits.parts);
    Require(!d.slave_part_ids.empty() && !d.master_part_ids.empty(), "Empty tied part scope");
    Require(Ids(Member(contact, "retained_master_part_ids"), limits.parts) == d.master_part_ids,
            "Tied master parts are not all retained");
    std::set<SourceId> slaves(d.slave_part_ids.begin(), d.slave_part_ids.end());
    std::set<SourceId> masters(d.master_part_ids.begin(), d.master_part_ids.end());
    std::set<SourceId> selected = slaves;
    selected.insert(masters.begin(), masters.end());
    Require(selected.size() <= limits.parts, "Tied combined part count exceeds capacity");
    const auto& declarations = Member(scope, "declarations");
    const auto parts = IndexRows(Member(declarations, "parts"), "source_part_id", limits);
    const auto& tables = Member(declarations, "tables");
    const auto sections = IndexRows(Member(tables, "section"), "identity", limits);
    const auto materials = IndexRows(Member(tables, "material"), "identity", limits);
    for (const auto id : selected) {
        const auto& original = Find(parts, id);
        const auto& canonical_part = source::FindPart(source, id);
        Part p;
        p.id = id;
        p.material_id = canonical_part.material;
        p.section_id = canonical_part.section;
        p.master = masters.count(id);
        p.slave = slaves.count(id);
        Require(Unsigned(original, "source_material_id") == p.material_id &&
                Unsigned(original, "source_section_id") == p.section_id,
                "Tied part material/section association changed");
        if (p.master)
            Require(canonical_part.shell_section &&
                    std::binary_search(source.selected_parts.begin(), source.selected_parts.end(), id),
                    "Tied master is not a retained original shell part");
        p.part_source = RequestSource(draft, original, blocks, limits);
        p.section_source = RequestSource(draft, Find(sections, p.section_id), blocks, limits);
        p.material_source = RequestSource(draft, Find(materials, p.material_id), blocks, limits);
        draft.part_rows.emplace(id, d.parts.size());
        d.parts.push_back(p);
    }
    return draft;
}
void CheckContact(const SourceEvidence& source, SourceId slave, SourceId master) {
    Require(source.block.keyword == "*CONTACT_TIED_SHELL_EDGE_TO_SURFACE" && source.cards.size() == 3,
            "Unsupported tied contact keyword/card shape");
    const auto& header = source.cards[0].second;
    Require(CardId(header, 0) == slave && CardId(header, 1) == master &&
            CardId(header, 2) == 2 && CardId(header, 3) == 2,
            "Tied contact set identity/type changed");
    // Preserve supplied options but do not resolve them into native settings.
    Require(header.size() <= 80 && source.cards[1].second.size() <= 80 &&
            source.cards[2].second.size() <= 80, "Tied contact card width changed");
}
void ValidateCards(Draft& draft, Limits limits) {
    auto& d = draft.data;
    const auto& slave = d.sources.at(d.slave_set_source);
    const auto& master = d.sources.at(d.master_set_source);
    for (const auto* source : {&slave, &master})
        Require(source->block.keyword == "*SET_PART_LIST_TITLE" || source->block.keyword == "*SET_PART_LIST",
                "Unsupported tied part-set operator");
    const bool slave_title = slave.block.keyword == "*SET_PART_LIST_TITLE";
    const bool master_title = master.block.keyword == "*SET_PART_LIST_TITLE";
    d.slave_set_id = CardId(slave.cards.at(slave_title).second, 0);
    d.master_set_id = CardId(master.cards.at(master_title).second, 0);
    Require(ListIds(slave, slave_title, limits) == d.slave_part_ids &&
            ListIds(master, master_title, limits) == d.master_part_ids,
            "Tied part membership differs from original cards");
    CheckContact(d.sources.at(d.contact_source), d.slave_set_id, d.master_set_id);
    for (const auto& p : d.parts) {
        const auto& part = d.sources.at(p.part_source);
        const auto& section = d.sources.at(p.section_source);
        const auto& material = d.sources.at(p.material_source);
        Require(part.block.keyword == "*PART" && part.cards.size() == 2 &&
                CardId(part.cards[1].second, 0) == p.id &&
                CardId(part.cards[1].second, 1) == p.section_id &&
                CardId(part.cards[1].second, 2) == p.material_id,
                "Tied original PART coefficients/identity changed");
        Require(!section.cards.empty() && !material.cards.empty() &&
                CardId(section.cards[0].second, 0) == p.section_id &&
                CardId(material.cards[0].second, 0) == p.material_id,
                "Tied section/material card identity changed");
    }
    for (const auto& group : d.groups) {
        const auto& source = d.sources.at(group.source);
        const auto& nodes = d.sources.at(group.node_set_source);
        const bool title = source.block.keyword == "*CONSTRAINED_NODAL_RIGID_BODY_TITLE";
        Require(source.block.keyword == "*CONSTRAINED_NODAL_RIGID_BODY" || title,
                "Unsupported tied rigid-group keyword");
        Require(CardId(source.cards.at(title).second, 0) == group.id &&
                CardId(source.cards.at(title).second, 2) == group.node_set_id,
                "Tied rigid-group source identity changed");
        const bool node_title = nodes.block.keyword == "*SET_NODE_LIST_TITLE";
        Require(nodes.block.keyword == "*SET_NODE_LIST" || node_title,
                "Unresolved tied rigid-group set operator");
        Require(CardId(nodes.cards.at(node_title).second, 0) == group.node_set_id &&
                ListIds(nodes, node_title, limits) == group.source_nodes,
                "Tied rigid membership differs from original cards");
    }
}
} // namespace crash::modelio::tied_shell::detail
