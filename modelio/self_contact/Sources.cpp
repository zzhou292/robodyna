#include "Internal.h"

#include <map>

namespace crash::modelio::self_contact::detail {
namespace {

namespace reader = assembly::reader;

std::vector<tied_shell::SourceEvidence> FileSources(
    const reader::Value& file, const char* filename,
    const std::string& member, bool contact_file, Limits limits) {
    reader::Require(reader::Text(file, "sha256") == output::Sha256(member),
        "Original self-contact source member identity differs");
    const auto& blocks = reader::Array(file, "blocks", limits.blocks, 1);
    const auto& declared_counts = reader::Member(file, "keyword_counts");
    reader::Require(declared_counts.IsObject(),
        "Original self-contact keyword census is missing");

    std::map<std::string, std::size_t> actual_counts;
    tied_shell::detail::Requests requests;
    std::size_t previous = 0;
    std::size_t contacts = 0;
    for (const auto& block : blocks.GetArray()) {
        const auto keyword = reader::Text(block, "keyword");
        const auto first = reader::Unsigned(block, "first_line");
        const auto last = reader::Unsigned(block, "last_line");
        reader::Require(reader::Text(block, "file") == filename &&
                first > previous && last >= first,
            "Original self-contact source block order differs");
        previous = last;
        ++actual_counts[keyword];
        const bool contact =
            keyword == "*CONTACT_AUTOMATIC_SINGLE_SURFACE";
        const bool set = keyword.rfind("*SET_PART_", 0) == 0;
        if (contact) ++contacts;
        if (!set && !(contact_file && contact)) continue;
        tied_shell::detail::Request request;
        request.evidence.block = {filename, keyword, {},
            reader::Text(block, "source_block_sha256"), first, last};
        output::arrays::CheckHash(request.evidence.block.sha256);
        reader::Require(requests.emplace(first, std::move(request)).second,
            "Duplicate original self-contact source request");
    }
    reader::Require((contact_file && contacts == 1) ||
            (!contact_file && contacts == 0),
        "Original automatic-single-surface contact count differs");
    reader::Require(declared_counts.MemberCount() == actual_counts.size(),
        "Original self-contact keyword census extent differs");
    for (const auto& [keyword, count] : actual_counts)
        reader::Require(reader::Unsigned(declared_counts, keyword.c_str()) == count,
            "Original self-contact keyword census omitted a block");

    tied_shell::Limits source_limits;
    source_limits.member_bytes = contact_file
        ? limits.combine_member_bytes : limits.auxiliary_member_bytes;
    source_limits.metadata_bytes = limits.metadata_bytes;
    source_limits.blocks = limits.blocks;
    return tied_shell::detail::ReadRequestedSources(
        requests, member, source_limits);
}

}  // namespace

void ReadSources(const source::CanonicalData& canonical,
    const std::string& auxiliary_member, const std::string& combine_member,
    Draft& draft, Limits limits) {
    output::Document document;
    constexpr unsigned flags = rapidjson::kParseFullPrecisionFlag |
        rapidjson::kParseIterativeFlag | rapidjson::kParseValidateEncodingFlag;
    document.Parse<flags>(canonical.canonical_bytes.data(),
        canonical.canonical_bytes.size());
    output::Require(!document.HasParseError() && document.IsObject(),
        "Invalid authenticated original self-contact metadata");
    assembly::reader::UniqueKeys(document);
    const auto& files = assembly::reader::Member(document, "source_files");
    output::Require(files.IsObject() &&
            files.MemberCount() <= limits.blocks,
        "Original self-contact source file count exceeds cap");

    constexpr const char* Auxiliary = "set-yaris-coarse-v1l.key";
    constexpr const char* Combine = "combine.key";
    auto combine = FileSources(assembly::reader::Member(files, Combine),
        Combine, combine_member, true, limits);
    auto auxiliary = FileSources(assembly::reader::Member(files, Auxiliary),
        Auxiliary, auxiliary_member, false, limits);
    output::Require(combine.size() <= limits.blocks &&
            auxiliary.size() <= limits.blocks - combine.size(),
        "Original self-contact selected source block count exceeds cap");

    draft.data.combine_filename = Combine;
    draft.data.combine_sha256 = output::Sha256(combine_member);
    draft.data.auxiliary_filename = Auxiliary;
    draft.data.auxiliary_sha256 = output::Sha256(auxiliary_member);
    draft.candidates.reserve(combine.size() + auxiliary.size());
    for (auto& value : combine)
        draft.candidates.push_back(std::move(value));
    for (auto& value : auxiliary)
        draft.candidates.push_back(std::move(value));
}

}  // namespace crash::modelio::self_contact::detail
