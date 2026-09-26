#include "Internal.h"
#include "modelio/self_contact/OriginalSelection.h"
#include "modelio/self_contact/PartSets.h"
#include "modelio/tied_shell/Internal.h"
#include "output/BoundedArrayIO.h"
#include <algorithm>
#include <map>

namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail {
namespace {
namespace reader = modelio::assembly::reader;
namespace source = output::full_shell::source;
namespace tied = modelio::tied_shell;
namespace sets = modelio::self_contact::part_sets;

std::string Member(const ids::ImportMembers& members, const char* name, std::size_t cap) {
    const ids::Member* found = nullptr;
    for (const auto& member : members.members) {
        if (member.filename != name) continue;
        Require(!found && member.bytes.size() <= cap, "Duplicate or oversized contact context member");
        found = &member;
    }
    Require(found, "Missing authenticated contact context member");
    return std::string(found->bytes);
}

void ReadFile(const output::Value& metadata, const char* filename,
    const std::string& bytes, std::vector<tied::SourceEvidence>& evidence,
    std::size_t metadata_cap) {
    Require(reader::Text(metadata, "sha256") == output::Sha256(bytes),
        "Contact context member hash differs from canonical source");
    constexpr std::size_t MaxBlocks = 8192;
    const auto& blocks = reader::Array(metadata, "blocks", MaxBlocks, 1);
    tied::detail::Requests requests;
    std::size_t previous = 0;
    for (const auto& block : blocks.GetArray()) {
        const auto first = reader::Unsigned(block, "first_line");
        const auto last = reader::Unsigned(block, "last_line");
        const auto keyword = reader::Text(block, "keyword");
        Require(reader::Text(block, "file") == filename && first > previous && last >= first,
            "Contact context source-block identity/order differs");
        previous = last;
        if (keyword != "*CONTACT_INTERIOR" && keyword.rfind("*SET_PART_", 0) != 0) continue;
        tied::detail::Request request;
        request.evidence.block = {filename, keyword, {},
            reader::Text(block, "source_block_sha256"), first, last};
        output::arrays::CheckHash(request.evidence.block.sha256);
        Require(requests.emplace(first, std::move(request)).second,
            "Duplicate contact context block request");
    }
    tied::Limits limits;
    limits.blocks = MaxBlocks;
    limits.member_bytes = 1u << 20;
    limits.metadata_bytes = metadata_cap;
    auto rows = tied::detail::ReadRequestedSources(requests, bytes, limits);
    Require(rows.size() <= MaxBlocks - evidence.size(), "Contact context block cap exceeded");
    for (auto& row : rows) evidence.push_back(std::move(row));
}
}

InteriorDisposition ReadInterior(const PhysicalModel& model,
    const ids::ImportMembers& members, Limits limits) {
    const auto& canonical = model.shell_source().references().source().canonical();
    const auto& data = canonical.data();
    const auto combine = Member(members, "combine.key", 64u << 10);
    const auto auxiliary = Member(members, "set-yaris-coarse-v1l.key", 1u << 20);
    // Reuse the existing complete SINGLE_SURFACE source/card/part-set check.
    // It supplies the selected nonzero interface-initialization context, not K.
    const auto selected = modelio::self_contact::OriginalSelection::Prepare(canonical, auxiliary, combine);
    Require(selected.data().source_fields.soft && *selected.data().source_fields.soft == 1.,
        "Contact seed requires its authenticated original selected contact profile");
    output::Document document;
    constexpr unsigned flags = rapidjson::kParseFullPrecisionFlag |
        rapidjson::kParseIterativeFlag | rapidjson::kParseValidateEncodingFlag;
    document.Parse<flags>(data.canonical_bytes.data(), data.canonical_bytes.size());
    Require(!document.HasParseError() && document.IsObject(), "Invalid canonical contact context metadata");
    reader::UniqueKeys(document);
    const auto& files = reader::Member(document, "source_files");
    std::vector<tied::SourceEvidence> evidence;
    ReadFile(reader::Member(files, "combine.key"), "combine.key", combine, evidence, limits.metadata_bytes);
    ReadFile(reader::Member(files, "set-yaris-coarse-v1l.key"), "set-yaris-coarse-v1l.key",
        auxiliary, evidence, limits.metadata_bytes);
    const auto part_sets = sets::Read(evidence, 8192, 1024);
    std::set<std::uint64_t> active, selected_parts;
    std::vector<std::uint64_t> ordered;
    std::vector<std::size_t> used_sources;
    std::size_t interior_blocks = 0;
    for (const auto& item : evidence) {
        if (item.block.keyword != "*CONTACT_INTERIOR") continue;
        ++interior_blocks;
        Require(!item.cards.empty(), "Empty original CONTACT_INTERIOR declaration");
        for (const auto& card : item.cards) {
            const auto id = tied::detail::CardId(card.second, 0);
            for (unsigned field = 1; field < 8; ++field) {
                const auto value = modelio::vehicle::detail::SourceScalar(card.second, field);
                Require(!value || *value == 0., "Unsupported CONTACT_INTERIOR option");
            }
            const auto& root = sets::Find(part_sets, id);
            if (root.additive) {
                for (const auto child : root.members)
                    Require(!sets::Find(part_sets, child).additive,
                        "CONTACT_INTERIOR converter does not recursively expand nested additive sets");
            }
            sets::Expand(part_sets, root, active, selected_parts, ordered, used_sources, 1024);
        }
    }
    Require(interior_blocks == 1 && !ordered.empty(), "Original CONTACT_INTERIOR closure differs");
    const auto& array = source::FindArray(data, "solids_records");
    const auto solids = output::arrays::Decode<std::uint64_t>(array.descriptor, array.bytes);
    Require(array.descriptor.layout.columns == 10 && solids.size() % 10 == 0,
        "Original solid census shape differs");
    std::map<std::uint64_t, std::size_t> original_count, retained_count;
    for (std::size_t i = 0; i < solids.size(); i += 10) ++original_count[solids[i + 1]];
    for (const auto& row : model.source_domain().source().solid_source().data().rows)
        ++retained_count[row.part_id];
    InteriorDisposition result;
    result.combine_sha256 = output::Sha256(combine);
    result.auxiliary_sha256 = output::Sha256(auxiliary);
    result.parts.reserve(ordered.size());
    for (const auto id : ordered) {
        const auto& part = source::FindPart(data, id);
        Require(!part.shell_section && original_count[id] > 0,
            "CONTACT_INTERIOR target is not a genuine original solid part");
        result.parts.push_back({id, part.section, part.material, retained_count[id], original_count[id]});
        result.original_solids += original_count[id];
        result.retained_solids += retained_count[id];
    }
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail
