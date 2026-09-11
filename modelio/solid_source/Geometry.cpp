#include "Internal.h"
#include "lib_src/elements/solid6z/CollapsedBrickTopology.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>
#include <map>
#include <set>

namespace crash::modelio::solid_source::detail {
namespace {
void CheckRawCards(const std::string& member, Data& data) {
    std::vector<std::pair<std::uint32_t, std::size_t>> requests;
    requests.reserve(data.rows.size());
    for (std::size_t row = 0; row < data.rows.size(); ++row)
        requests.emplace_back(data.rows[row].source_line, row);
    std::sort(requests.begin(), requests.end());
    std::size_t cursor = 0, line = 1, next = 0;
    std::string keyword;
    while (cursor < member.size() && next < requests.size()) {
        const auto end = member.find('\n', cursor);
        const auto stop = end == std::string::npos ? member.size() : end;
        Require(stop - cursor <= 4096, "Original solid source line exceeds bounded width");
        auto text = member.substr(cursor, stop - cursor);
        const auto comment = text.find('$');
        if (comment != std::string::npos) text.resize(comment);
        text = assembly::reader::auxiliary::Trim(std::move(text));
        if (!text.empty() && text.front() == '*') keyword = text;
        if (requests[next].first == line) {
            auto& row = data.rows[requests[next].second];
            Require(keyword == "*ELEMENT_SOLID" && assembly::reader::auxiliary::BlankTail(text, 80) &&
                assembly::reader::auxiliary::Id(text, 0, 8) == row.element_id &&
                assembly::reader::auxiliary::Id(text, 8, 8) == row.part_id,
                "Original selected solid row identity or format changed");
            for (unsigned slot = 0; slot < 8; ++slot)
                Require(assembly::reader::auxiliary::Id(text, 16 + 8 * slot, 8) == row.raw_node_ids[slot],
                        "Original selected solid connectivity changed");
            row.raw_card = std::move(text);
            ++next;
        }
        Require(next == requests.size() || requests[next].first > line,
                "Repeated or invalid selected solid source line");
        cursor = end == std::string::npos ? member.size() : end + 1;
        ++line;
    }
    Require(next == requests.size(), "Selected solid source card coverage is incomplete");
}
}
void ReadGeometry(const source::CanonicalData& source, const std::string& member, Data& data, Limits limits) {
    const auto records = Decode<std::uint64_t>(source, "solids_records");
    const auto indices = Decode<std::uint32_t>(source, "solids_node_indices");
    const auto lines = Decode<std::uint32_t>(source, "solids_source_lines");
    const auto masks = Decode<std::uint16_t>(source, "solids_blank_masks");
    const auto node_ids = Decode<std::uint64_t>(source, "node_ids");
    Require(records.size() % 10 == 0 && indices.size() == records.size() / 10 * 8 &&
        lines.size() == records.size() / 10 && masks.size() == lines.size() &&
        lines.size() <= limits.source_solids && node_ids.size() == source.canonical_nodes &&
        std::is_sorted(node_ids.begin(), node_ids.end()), "Solid canonical array extent or node order changed");
    data.original_solids = lines.size();
    std::map<std::uint64_t, std::size_t> parts;
    for (std::size_t i = 0; i < data.parts.size(); ++i) parts.emplace(data.parts[i].id, i);
    std::set<std::uint64_t> elements;
    std::vector<unsigned char> used(node_ids.size(), 0);
    data.rows.reserve(std::min(limits.parents, lines.size()));
    for (std::size_t e = 0; e < lines.size(); ++e) {
        const auto* record = records.data() + 10 * e;
        Require(elements.insert(record[0]).second, "Duplicate original solid element identity");
        const auto found = parts.find(record[1]);
        if (found == parts.end()) {
            Require(!Selected(record[1], data.policy), "Selected solid has no admitted material declaration");
            ++data.outside_solids;
            continue;
        }
        Require(data.rows.size() < limits.parents && masks[e] == 0 && lines[e] > 0,
                "Selected solid count or required source field changed");
        Row row;
        row.element_id = record[0];
        row.part_id = record[1];
        row.canonical_row = e;
        row.source_line = lines[e];
        row.part_index = found->second;
        std::uint64_t raw[8];
        std::set<std::uint64_t> unique;
        for (unsigned slot = 0; slot < 8; ++slot) {
            const auto node = indices[8 * e + slot];
            Require(node < node_ids.size() && node_ids[node] == record[2 + slot],
                    "Selected solid source NID/index association changed");
            row.raw_node_ids[slot] = raw[slot] = record[2 + slot];
            row.canonical_nodes[slot] = node;
            unique.insert(raw[slot]);
            used[node] = 1;
        }
        if (row.part_id == AdhesivePart) {
            Require(unique.size() == 8, "Adhesive source requires eight distinct solid18 slots");
            row.family = Family::Solid18;
        } else if (SelectedRadiator(row.part_id, data.policy)) {
            Require(unique.size() == 8, "Original radiator requires eight distinct solid18 slots");
            row.family = Family::Solid18Law90;
        } else if (SelectedRear(row.part_id, data.policy)) {
            // Keep the original eight slots. The qualified LAW44 reference
            // validates either eight distinct nodes or the exact repeated pairs.
            row.family = Family::Solid18Law44;
        } else if (unique.size() == 8) {
            row.family = Family::Solid24;
        } else {
            tl::fea::solid6z::CollapsedBrickTopology mapping;
            Require(tl::fea::solid6z::MapCollapsedTopEdges(raw, mapping) == tl::fea::solid6z::Status::Success,
                    "Rubber source is outside the explicit collapsed-top-edge wedge profile");
            row.family = Family::Solid6z;
            std::copy(std::begin(mapping.six_to_raw), std::end(mapping.six_to_raw), row.six_to_raw.begin());
        }
        data.rows.push_back(std::move(row));
    }
    for (std::size_t node = 0; node < used.size(); ++node)
        if (used[node]) data.canonical_nodes.push_back(node);
    CheckRawCards(member, data);
}
} // namespace crash::modelio::solid_source::detail
