#include "Internal.h"
#include "InterfaceProfile.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include "SourceIds.h"
#include "../TopologyDigestFields.h"
#include <algorithm>
#include <climits>
#include <functional>
#include <set>

namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
namespace {
namespace tied = modelio::tied_shell;
struct Edge { std::string child; std::int64_t offset = 0; };
struct File {
    std::vector<tied::SourceEvidence> evidence;
    std::vector<Edge> edges;
    std::int64_t offset = 0;
};
bool Starts(const std::string& text, const char* prefix) { return text.rfind(prefix, 0) == 0; }

void ReadFile(const output::Value& metadata, const ids::Member& member, File& file,
    InterfaceCensus& census, Limits limits) {
    if (output::Sha256(std::string(member.bytes)) != reader::Text(metadata, "sha256"))
        Reject(Status::InvalidInput, "Property-context member differs from authenticated source", member.filename);
    const auto& blocks = reader::Array(metadata, "blocks", limits.source_blocks, 1);
    tied::detail::Requests requests;
    for (const auto& block : blocks.GetArray()) {
        const auto keyword = reader::Text(block, "keyword");
        const auto first = reader::Unsigned(block, "first_line");
        CheckInterfaceKeyword(keyword, census, member.filename, first);
        if (keyword != "*PART" && !Starts(keyword, "*SECTION_") &&
            keyword != "*INCLUDE" && keyword != "*INCLUDE_TRANSFORM") continue;
        tied::detail::Request request;
        request.evidence.block = {member.filename, keyword, {}, reader::Text(block, "source_block_sha256"),
            first, reader::Unsigned(block, "last_line")};
        if (!requests.emplace(first, std::move(request)).second)
            Reject(Status::InvalidInput, "Repeated property-context source block", member.filename, first);
    }
    tied::Limits source_limits;
    source_limits.blocks = limits.source_blocks;
    source_limits.member_bytes = ids::Limits{}.member_bytes;
    source_limits.metadata_bytes = limits.metadata_bytes;
    file.evidence = tied::detail::ReadRequestedSources(requests, std::string(member.bytes), source_limits);
    for (const auto& evidence : file.evidence) {
        if (evidence.block.keyword != "*INCLUDE" && evidence.block.keyword != "*INCLUDE_TRANSFORM") continue;
        if (evidence.cards.empty()) Reject(Status::InvalidInput, "Missing include target in property context");
        file.edges.push_back({reader::auxiliary::Trim(evidence.cards[0].second), Offset(evidence)});
    }
}
std::string Digest(const std::vector<PartControl>& parts, const InterfaceCensus& interfaces,
    const std::string& source_digest, std::size_t cap) {
    ::crash::cases::vehicle_self_contact::native::detail::digest::Fields fields(
        "native-effective-control-single-mid-v1:" + source_digest, cap);
    fields.Add<std::uint64_t>("parts", parts.size(), 6, [&](auto index) {
        const auto& part = parts[index / 6];
        switch (index % 6) {
        case 0: return part.part_id;
        case 1: return part.section_id;
        case 2: return part.material_id;
        case 3: return part.native_property_id;
        case 4: return std::uint64_t(part.directly_requested);
        default: return std::uint64_t(part.effective_control);
        }
    });
    const std::uint64_t census[]{std::uint64_t(interfaces.disposition), interfaces.type25_sources,
        interfaces.type2_sources, interfaces.interior_sources, interfaces.rigid_wall_sources,
        interfaces.checked_source_blocks};
    fields.Add<std::uint64_t>("complete_interface_source_profile", 1, 6, [&](auto i) { return census[i]; });
    return fields.Finish().sha256;
}
}

Context ReadContext(const seed::PreCorrectionNodalSource& input, const ids::ImportMembers& members,
    const ids::ImportContext& imported, Limits limits) {
    if (imported.data().diagnostic.status != ids::Readiness::Ready ||
        imported.data().source_digest != input.provenance().import_source_digest)
        Reject(Status::InvalidInput, "Correction and pre-correction do not share the same closed import context");
    const auto& canonical = imported.canonical().data();
    output::Document document;
    constexpr unsigned flags = rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag |
        rapidjson::kParseValidateEncodingFlag;
    document.Parse<flags>(canonical.canonical_bytes.data(), canonical.canonical_bytes.size());
    Require(!document.HasParseError() && document.IsObject(), "Invalid authenticated correction source metadata");
    reader::UniqueKeys(document);
    const auto& inventory = reader::Member(document, "source_files");
    Context result;
    result.source_digest = imported.data().source_digest;
    std::map<std::string, File> files;
    for (const auto& member : members.members) {
        auto found = files.emplace(member.filename, File{});
        if (!found.second) Reject(Status::InvalidInput, "Repeated source member in property context");
        ReadFile(reader::Member(inventory, member.filename.c_str()), member, found.first->second,
            result.interfaces, limits);
    }
    std::set<std::string> visited;
    std::function<void(const std::string&, std::int64_t)> visit = [&](const std::string& name, std::int64_t offset) {
        auto found = files.find(name);
        if (found == files.end() || !visited.insert(name).second)
            Reject(Status::UnsupportedSource, "Repeated, cyclic or missing property include member", name);
        if (offset < -INT_MAX || offset > INT_MAX)
            Reject(Status::UnsupportedSource, "Accumulated native property include offset exceeds range", name);
        found->second.offset = offset;
        for (const auto& edge : found->second.edges) visit(edge.child, offset + edge.offset);
    };
    visit(members.entry_member, 0);
    if (visited.size() != files.size()) Reject(Status::InvalidInput, "Property context omits an authenticated source member");
    std::vector<Part> parts;
    std::vector<Section> sections;
    for (const auto& [name, file] : files) {
        for (const auto& evidence : file.evidence) {
            const auto& keyword = evidence.block.keyword;
            if (keyword == "*PART") {
                if (evidence.cards.size() != 2 || parts.size() >= limits.parts)
                    Reject(Status::UnsupportedSource, "Unsupported ordinary PART source shape or count", name, evidence.block.first_line);
                const auto& card = evidence.cards[1].second;
                parts.push_back({Shift(tied::detail::CardId(card, 0), file.offset),
                    Shift(tied::detail::CardId(card, 1), file.offset),
                    Shift(tied::detail::CardId(card, 2), file.offset), name, evidence.cards[1].first});
            } else if (Starts(keyword, "*SECTION_")) {
                if (evidence.cards.empty() || sections.size() >= limits.parts)
                    Reject(Status::UnsupportedSource, "Incomplete source section or count", name, evidence.block.first_line);
                sections.push_back({Shift(tied::detail::CardId(evidence.cards[0].second, 0), file.offset), keyword});
            }
        }
    }
    std::vector<std::uint64_t> requested;
    requested.reserve(input.interior().parts.size());
    for (const auto& part : input.interior().parts) requested.push_back(part.part_id);
    result.parts = ResolveSharing(parts, sections, requested);
    // OriginalSelection already authenticated SOFT1 SINGLE_SURFACE on this
    // same source digest. The complete census above also covers wall/constraint
    // generators and rejects native includes or unreviewed interface profiles.
    if (!result.interfaces.type25_sources || !result.interfaces.interior_sources)
        Reject(Status::UnsupportedSource, "Missing admitted original interface/control source");
    result.interfaces.disposition = InterfaceDisposition::CompleteNoApplicableType24;
    result.property_digest = Digest(result.parts, result.interfaces, result.source_digest + ":" +
        input.provenance().contributor_digest, limits.metadata_bytes);
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail
