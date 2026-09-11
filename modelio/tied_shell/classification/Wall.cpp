#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>
#include <set>

namespace crash::modelio::tied_shell::classification_detail {
OriginalWallAssemblyReceipt Wall(const source::CanonicalData& canonical, const Data& declaration,
        const std::string& member, ClassificationSourceLimits limits) {
    output::Document document;
    document.Parse<rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag |
        rapidjson::kParseValidateEncodingFlag>(canonical.canonical_bytes.data(), canonical.canonical_bytes.size());
    Require(!document.HasParseError() && document.IsObject(), "Invalid original wall source inventory");
    const auto& file = Member(Member(document, "source_files"), "wall.key");
    OriginalWallAssemblyReceipt out;
    out.filename = "wall.key";
    out.sha256 = Text(file, "sha256");
    Require(!member.empty() && member.size() <= limits.wall_member_bytes &&
            output::Sha256(member) == out.sha256, "Original wall member authentication failed");
    detail::Requests requests;
    const auto& blocks = Array(file, "blocks", limits.source_blocks, 1);
    std::size_t previous = 0;
    for (const auto& block : blocks.GetArray()) {
        const auto first = Unsigned(block, "first_line"), last = Unsigned(block, "last_line");
        Require(Text(block, "file") == out.filename && first > previous && last >= first,
                "Original wall source block order changed");
        previous = last;
        detail::Request request;
        request.evidence.block = {out.filename, Text(block, "keyword"), {},
            Text(block, "source_block_sha256"), first, last};
        requests.emplace(first, std::move(request));
    }
    Limits source_limits;
    source_limits.member_bytes = limits.wall_member_bytes;
    source_limits.metadata_bytes = limits.wall_member_bytes;
    source_limits.blocks = limits.source_blocks;
    out.sources = detail::ReadRequestedSources(requests, member, source_limits);
    std::set<SourceId> node_ids, shell_ids;
    std::vector<std::array<SourceId, 5>> shells;
    std::size_t parts = 0, materials = 0, sections = 0;
    for (const auto& source : out.sources) {
        const auto& key = source.block.keyword;
        if (key == "*PART") {
            Require(++parts == 1 && source.cards.size() == 2, "Unsupported original wall PART shape");
            const auto& row = source.cards[1].second;
            out.part_id = detail::CardId(row, 0);
            out.section_id = detail::CardId(row, 1);
            out.material_id = detail::CardId(row, 2);
        } else if (key == "*NODE") {
            for (const auto& card : source.cards) {
                if (card.second.empty()) continue;
                const auto id = auxiliary::Id(card.second, 0, 8);
                Require(node_ids.insert(id).second && !Observed(declaration, id),
                        "Original wall node is repeated or affects an observed tied slave");
            }
        } else if (key == "*ELEMENT_SHELL") {
            for (const auto& card : source.cards) {
                if (card.second.empty()) continue;
                Require(shell_ids.insert(auxiliary::Id(card.second, 0, 8)).second,
                        "Duplicate original wall shell identity");
                std::array<SourceId, 5> row{};
                for (unsigned column = 0; column < row.size(); ++column)
                    row[column] = auxiliary::Id(card.second, 8 * (column + 1), 8);
                shells.push_back(row);
            }
        } else if (key == "*MAT_RIGID") {
            Require(++materials == 1 && !source.cards.empty() && detail::CardId(source.cards[0].second, 0) == 1001,
                    "Original wall material identity changed");
        } else if (key == "*SECTION_SHELL") {
            Require(++sections == 1 && !source.cards.empty() && detail::CardId(source.cards[0].second, 0) == 1001,
                    "Original wall section identity changed");
        } else {
            Require(key == "*KEYWORD" || key == "*END" || key == "*SET_SEGMENT" ||
                    key == "*SET_NODE_GENERAL" || key == "*RIGIDWALL_PLANAR_FINITE_FORCES_ID" ||
                    key == "*RIGIDWALL_PLANAR_FINITE_ID", "Unresolved original wall assembly keyword");
        }
    }
    Require(parts == 1 && materials == 1 && sections == 1 && !node_ids.empty() && !shell_ids.empty() &&
            out.part_id == 1001 && out.section_id == 1001 && out.material_id == 1001,
            "Original wall replacement is not the complete declared assembly");
    for (const auto& shell : shells) {
        Require(shell[0] == out.part_id, "Original wall shell uses another PART");
        for (unsigned slot = 1; slot < shell.size(); ++slot)
            Require(node_ids.count(shell[slot]), "Original wall shell node is outside replaced assembly");
    }
    out.node_ids.assign(node_ids.begin(), node_ids.end());
    out.shell_ids.assign(shell_ids.begin(), shell_ids.end());
    return out;
}
}
