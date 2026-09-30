#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>
#include <charconv>
#include <sstream>

namespace crash::modelio::tied_shell::detail {
std::size_t RequestSource(Draft& draft, const Value& expected, const Value& originals, Limits limits) {
    const auto first = Unsigned(expected, expected.HasMember("first_line") ? "first_line" : "source_line");
    const auto hash = Text(expected, expected.HasMember("sha256") ? "sha256" : "source_part_sha256");
    if (draft.original_blocks.empty()) {
        Require(originals.IsArray() && originals.Size() <= limits.blocks,
                "Tied original block inventory exceeds capacity");
        for (const auto& row : originals.GetArray())
            Require(draft.original_blocks.emplace(Unsigned(row, "first_line"), &row).second,
                    "Duplicate original source block location");
    }
    const auto found = draft.original_blocks.find(first);
    const Value* original = found == draft.original_blocks.end() ? nullptr : found->second;
    Require(original && Text(*original, "source_block_sha256") == hash,
            "Tied source block differs from original member inventory");
    Request request;
    request.evidence.block = {Text(*original, "file"), Text(*original, "keyword"), {}, hash,
        first, Unsigned(*original, "last_line")};
    Require(request.evidence.block.filename == "yaris-coarse-v1l.key" && first &&
            request.evidence.block.last_line >= first, "Invalid tied source block location");
    if (expected.HasMember("first_line")) {
        Require(Text(expected, "file") == request.evidence.block.filename &&
                Text(expected, "keyword") == request.evidence.block.keyword &&
                Unsigned(expected, "last_line") == request.evidence.block.last_line,
                "Tied source block location changed");
    }
    if (expected.HasMember("cards")) request.cards = &expected["cards"];
    auto inserted = draft.requests.emplace(first, std::move(request));
    if (!inserted.second) {
        Require(inserted.first->second.evidence.block.sha256 == hash, "Conflicting tied source block");
        if (expected.HasMember("cards")) {
            const auto* prior = inserted.first->second.cards;
            Require(!prior || *prior == expected["cards"], "Conflicting tied source cards");
            inserted.first->second.cards = &expected["cards"];
        }
    }
    Require(draft.requests.size() <= limits.blocks, "Tied source block count exceeds capacity");
    return first; // Temporary source-line identity; ReadSources replaces it with the owned row index.
}
std::vector<SourceEvidence> ReadRequestedSources(Requests& requests, const std::string& member, Limits limits) {
    std::size_t cursor = 0, line = 1, metadata = 0;
    std::vector<SourceEvidence> result;
    for (auto& [first, request] : requests) {
        while (line < first) {
            const auto end = member.find('\n', cursor);
            Require(end != std::string::npos, "Tied source block lies beyond original member");
            cursor = end+1;
            ++line;
        }
        const auto begin = cursor;
        while (line <= request.evidence.block.last_line) {
            Require(cursor < member.size(), "Truncated tied source block");
            const auto end = member.find('\n', cursor);
            cursor = end == std::string::npos ? member.size() : end+1;
            ++line;
        }
        Require(cursor-begin <= limits.metadata_bytes-metadata, "Tied source metadata byte cap exceeded");
        metadata += cursor-begin;
        auto& evidence = request.evidence;
        evidence.block.raw_text = member.substr(begin, cursor-begin);
        Require(output::Sha256(evidence.block.raw_text) == evidence.block.sha256,
                "Tied raw source block authentication failed");
        std::istringstream stream(evidence.block.raw_text);
        std::string text;
        Require(bool(std::getline(stream, text)) &&
                auxiliary::Trim(text) == evidence.block.keyword, "Tied raw source keyword changed");
        std::size_t number = first;
        while (std::getline(stream, text)) {
            ++number;
            const auto start = text.find_first_not_of(" \t\r");
            if (start != std::string::npos && text[start] == '$') continue;
            const auto comment = text.find('$');
            if (comment != std::string::npos) text.resize(comment);
            evidence.cards.emplace_back(number, auxiliary::Trim(text));
        }
        Require(number == evidence.block.last_line, "Tied source line count changed");
        if (request.cards) {
            Require(request.cards->IsArray() && request.cards->Size() == evidence.cards.size(),
                    "Tied source card inventory changed");
            for (std::size_t i = 0; i < evidence.cards.size(); ++i) {
                const auto& card = (*request.cards)[static_cast<unsigned>(i)];
                Require(Unsigned(card, "source_line") == evidence.cards[i].first &&
                        Text(card, "text") == evidence.cards[i].second, "Tied source card changed");
            }
        }
        result.push_back(std::move(evidence));
    }
    return result;
}
void ReadSources(Draft& draft, const std::string& member, Limits limits) {
    draft.data.sources = ReadRequestedSources(draft.requests, member, limits);
    std::map<std::size_t, std::size_t> rows;
    for (std::size_t i = 0; i < draft.data.sources.size(); ++i)
        rows.emplace(draft.data.sources[i].block.first_line, i);
    const auto remap = [&](std::size_t& row) { row = rows.at(row); };
    auto& d = draft.data;
    remap(d.contact_source);
    remap(d.slave_set_source);
    remap(d.master_set_source);
    for (auto& p : d.parts) {
        remap(p.part_source);
        remap(p.section_source);
        remap(p.material_source);
    }
    for (auto& g : d.groups) {
        remap(g.source);
        remap(g.node_set_source);
    }
    for (auto& block : d.unresolved_constraints)
        if (block.retained_source != SIZE_MAX) remap(block.retained_source);
}
SourceId CardId(const std::string& row, unsigned column) {
    const auto offset = std::size_t(column)*10;
    Require(offset < row.size(), "Missing tied source identity");
    const auto text = row.substr(offset, 10);
    const auto first = text.find_first_not_of(" \t"), last = text.find_last_not_of(" \t");
    Require(first != std::string::npos, "Blank tied source identity");
    SourceId id = 0;
    const auto parsed = std::from_chars(text.data()+first, text.data()+last+1, id);
    Require(parsed.ec == std::errc{} && parsed.ptr == text.data()+last+1 && id && id <= INT64_MAX,
            "Invalid tied source identity");
    return id;
}
std::vector<SourceId> ListIds(const SourceEvidence& source, bool title, Limits limits) {
    const auto header = std::size_t(title);
    Require(source.cards.size() > header, "Missing tied list header");
    const auto& text = source.cards[header].second;
    Require(text.size() <= 80, "Tied list header exceeds eight source columns");
    // Literal membership is independent of the retained optional header fields.
    // Their native interpretation remains unresolved in this declaration.
    std::vector<SourceId> result;
    std::set<SourceId> unique;
    for (std::size_t i = header+1; i < source.cards.size(); ++i) {
        const auto& row = source.cards[i].second;
        Require(row.size() <= 80, "Tied list exceeds eight source columns");
        for (unsigned column = 0; column < 8; ++column) {
            if (!vehicle::detail::SourceScalar(row, column)) continue;
            const auto id = CardId(row, column);
            Require(unique.insert(id).second && result.size() < limits.group_members,
                    "Duplicate or excessive tied list member");
            result.push_back(id);
        }
    }
    Require(!result.empty(), "Empty tied source list");
    return result;
}
} // namespace crash::modelio::tied_shell::detail
