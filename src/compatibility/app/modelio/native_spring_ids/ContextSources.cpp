#include "Internal.h"
#include <algorithm>
#include <climits>
#include <functional>
#include <map>
#include <set>
namespace crash::modelio::native_spring_ids::detail {
namespace {
struct Edge { std::string target; bool transformed = false; Location location; };
struct File {
    const Member* member = nullptr;
    const output::Value* metadata = nullptr;
    bool spring_candidate = false;
    std::vector<Edge> children;
};
bool Starts(const std::string& value, const char* prefix) { return value.rfind(prefix, 0) == 0; }
bool SelectedBlock(const std::string& keyword) {
    return keyword == "*ELEMENT_DISCRETE" || keyword == "*CONSTRAINED_SPOTWELD_ID" ||
        Starts(keyword, "*CONSTRAINED_JOINT_") || keyword == "*INCLUDE" || keyword == "*INCLUDE_TRANSFORM";
}
bool Candidate(const std::string& keyword) {
    return keyword == "*ELEMENT_BEAM" || keyword == "*ELEMENT_DISCRETE" ||
        keyword == "*CONSTRAINED_SPOTWELD_ID" || Starts(keyword, "*CONSTRAINED_JOINT_");
}
void Keyword(const std::string& keyword, const std::string& file, std::size_t line) {
    if (Starts(keyword, "*INCLUDE") && keyword != "*INCLUDE" && keyword != "*INCLUDE_TRANSFORM")
        Reject(Readiness::UnsupportedSource, "Unsupported preload/include import route", file, line);
    if (Starts(keyword, "*ELEMENT_") && keyword != "*ELEMENT_BEAM" && keyword != "*ELEMENT_DISCRETE" &&
        keyword != "*ELEMENT_SHELL" && keyword != "*ELEMENT_SOLID" && keyword != "*ELEMENT_MASS" &&
        keyword != "*ELEMENT_MASS_PART" && keyword != "*ELEMENT_SEATBELT_ACCELEROMETER")
        Reject(Readiness::UnsupportedSource, "Unresolved ordinary or generated element family", file, line);
    if (Starts(keyword, "*CONSTRAINED_JOINT") && keyword != "*CONSTRAINED_JOINT_SPHERICAL_ID" &&
        keyword != "*CONSTRAINED_JOINT_REVOLUTE_ID" && keyword != "*CONSTRAINED_JOINT_CYLINDRICAL_ID")
        Reject(Readiness::UnsupportedSource, "Stiffness or unsupported regular joint changes generated order", file, line);
    if (Starts(keyword, "*CONSTRAINED_SPOTWELD") && keyword != "*CONSTRAINED_SPOTWELD_ID")
        Reject(Readiness::UnsupportedSource, "Unsupported spotweld source variant", file, line);
    if (Starts(keyword, "*CONSTRAINED_") && !Starts(keyword, "*CONSTRAINED_JOINT_") &&
        keyword != "*CONSTRAINED_SPOTWELD_ID" && keyword != "*CONSTRAINED_NODAL_RIGID_BODY" &&
        keyword != "*CONSTRAINED_RIGID_BODIES" && keyword != "*CONSTRAINED_EXTRA_NODES_SET")
        Reject(Readiness::UnsupportedSource, "Unsupported constrained source generator", file, line);
}
void Name(const std::string& name) {
    if (name.empty() || name.size() > 128 || name == "." || name == ".." ||
        name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-") != std::string::npos)
        Reject(Readiness::UnsupportedSource, "Import member must be a bounded literal basename", name);
}
void ReadFile(const std::string& name, File& file, ContextData& data, Limits limits) {
    const auto& bytes = file.member->bytes;
    const auto& metadata = *file.metadata;
    const auto expected = reader::Text(metadata, "sha256"); output::arrays::CheckHash(expected);
    if (output::Sha256(std::string(bytes)) != expected)
        Reject(Readiness::InvalidSource, "Authenticated import member hash differs", name);
    data.members.push_back({name, expected, bytes.size()});
    const auto& blocks = reader::Array(metadata, "blocks", limits.blocks, 1);
    const auto& counts = reader::Member(metadata, "keyword_counts");
    Require(counts.IsObject(), "Missing complete import keyword census");
    std::map<std::string, std::size_t> observed;
    tied_shell::detail::Requests requests;
    std::size_t cursor = 0, line = 1, previous = 0;
    for (const auto& block : blocks.GetArray()) {
        const auto first = reader::Unsigned(block, "first_line"), last = reader::Unsigned(block, "last_line");
        const auto keyword = reader::Text(block, "keyword");
        if (reader::Text(block, "file") != name || first != previous+1 || last < first)
            Reject(Readiness::InvalidSource, "Incomplete or overlapping import block inventory", name, first);
        previous = last;
        const auto begin = cursor;
        while (line <= last) {
            if (cursor >= bytes.size()) Reject(Readiness::InvalidSource, "Truncated import block", name, line);
            const auto end = bytes.find('\n', cursor);
            cursor = end == std::string_view::npos ? bytes.size() : end+1;
            ++line;
        }
        const auto raw = std::string(bytes.substr(begin, cursor-begin));
        const auto sha = reader::Text(block, "source_block_sha256"); output::arrays::CheckHash(sha);
        const auto header_end = raw.find('\n');
        if (output::Sha256(raw) != sha || assembly::reader::auxiliary::Trim(raw.substr(0, header_end)) != keyword)
            Reject(Readiness::InvalidSource, "Import block hash or keyword differs", name, first);
        Keyword(keyword, name, first); ++observed[keyword];
        file.spring_candidate = file.spring_candidate || Candidate(keyword);
        if (SelectedBlock(keyword)) {
            tied_shell::detail::Request request;
            request.evidence.block = {name, keyword, {}, sha, first, last};
            requests.emplace(first, std::move(request));
        }
    }
    if (cursor != bytes.size() || counts.MemberCount() != observed.size())
        Reject(Readiness::InvalidSource, "Import file is not completely covered by its source census", name);
    for (const auto& [keyword, count] : observed)
        if (reader::Unsigned(counts, keyword.c_str()) != count)
            Reject(Readiness::InvalidSource, "Import keyword count differs", name);
    tied_shell::Limits source_limits;
    source_limits.blocks = limits.blocks; source_limits.metadata_bytes = limits.metadata_bytes;
    const auto sources = tied_shell::detail::ReadRequestedSources(requests, std::string(bytes), source_limits);
    for (const auto& evidence : sources) {
        const auto& keyword = evidence.block.keyword;
        if (keyword == "*INCLUDE" || keyword == "*INCLUDE_TRANSFORM") {
            const auto& cards = evidence.cards;
            if (cards.empty()) Reject(Readiness::InvalidSource, "Empty include directive", name, evidence.block.first_line);
            const auto child = assembly::reader::auxiliary::Trim(cards[0].second); Name(child);
            if (keyword == "*INCLUDE") {
                if (cards.size() != 1) Reject(Readiness::UnsupportedSource, "Unsupported include options", name, evidence.block.first_line);
            } else {
                if (cards.size() != 5) Reject(Readiness::UnsupportedSource, "Incomplete include-transform descriptor", name, evidence.block.first_line);
                // Validate literal offset/scaling/transform fields without
                // implementing another geometry transform or parameter parser.
                for (unsigned row = 1; row < 5; ++row) {
                    const unsigned width = row == 3 ? 5 : row == 1 ? 7 : 1;
                    if (!assembly::reader::auxiliary::BlankTail(cards[row].second, 10*width))
                        Reject(Readiness::UnsupportedSource, "Extra include-transform fields", name, cards[row].first);
                    for (unsigned column = 0; column < width; ++column) {
                        const auto value = vehicle::detail::SourceScalar(cards[row].second, column, 10);
                        if (value && row != 3 && (*value != std::floor(*value) || *value < INT_MIN || *value > INT_MAX))
                            Reject(Readiness::UnsupportedSource, "Unresolved native include offset/transform identity", name, cards[row].first);
                    }
                }
            }
            file.children.push_back({child, keyword == "*INCLUDE_TRANSFORM", {name, evidence.block.first_line}});
        }
        data.evidence.push_back(evidence);
    }
}
}
ContextData BuildContext(const source::CanonicalData& canonical, const ImportMembers& input, Limits limits) {
    if (input.profile != Profile::DirectKeywordR14FreshRadiossPoSortById)
        Reject(Readiness::UnsupportedProfile, "Unresolved direct-key reader/backend/selection profile");
    Name(input.entry_member);
    output::Document document;
    constexpr unsigned flags = rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag | rapidjson::kParseValidateEncodingFlag;
    document.Parse<flags>(canonical.canonical_bytes.data(), canonical.canonical_bytes.size());
    Require(!document.HasParseError() && document.IsObject(), "Invalid canonical import inventory");
    reader::UniqueKeys(document);
    const auto& inventory = reader::Member(document, "source_files");
    if (!inventory.IsObject() || inventory.MemberCount() != input.members.size() || input.members.size() > limits.members)
        Reject(Readiness::MissingSource, "Import members do not cover the complete authenticated source_files inventory");
    std::map<std::string, File> files;
    for (const auto& member : input.members) {
        Name(member.filename);
        if (!files.emplace(member.filename, File{&member, nullptr, false, {}}).second)
            Reject(Readiness::InvalidSource, "Repeated import member", member.filename);
    }
    ContextData result; result.profile = input.profile; result.entry_member = input.entry_member;
    for (const auto& file : inventory.GetObject()) {
        const std::string name(file.name.GetString(), file.name.GetStringLength());
        const auto found = files.find(name);
        if (found == files.end()) Reject(Readiness::MissingSource, "Missing authenticated import member", name);
        found->second.metadata = &file.value;
        ReadFile(name, found->second, result, limits);
    }
    std::set<std::string> visited;
    std::function<bool(const std::string&)> visit = [&](const std::string& name) {
        const auto found = files.find(name);
        if (found == files.end()) Reject(Readiness::MissingSource, "Unresolved include target", name);
        if (!visited.insert(name).second) Reject(Readiness::UnsupportedSource, "Repeated or cyclic include occurrence", name);
        bool spring = found->second.spring_candidate;
        for (const auto& edge : found->second.children) {
            const bool child_spring = visit(edge.target);
            if (edge.transformed && child_spring)
                Reject(Readiness::UnsupportedSource, "Transformed SPRING contributors are outside this first import profile", edge.location.file, edge.location.line);
            spring = spring || child_spring;
        }
        return spring;
    };
    (void)visit(input.entry_member);
    if (visited.size() != files.size()) Reject(Readiness::MissingSource, "Unreachable member in closed import inventory");
    PopulateElements(canonical, document, result, limits);
    PopulateConnections(result, limits);
    std::string binding = "native-spring-id-source-v1:direct-key-r14-po-sort-id:" + input.entry_member + ":" + canonical.inputs.canonical_manifest.sha256;
    for (const auto& member : result.members) binding += ":" + member.file + ":" + member.sha256;
    result.source_digest = output::Sha256(binding);
    result.diagnostic = {Readiness::Ready, "Closed direct-key original SPRING namespace", {}, 0, 0};
    return result;
}
} // namespace crash::modelio::native_spring_ids::detail
