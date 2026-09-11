#include "Internal.h"
#include <map>

namespace crash::modelio::solid_source::detail {
namespace {
using Index = std::map<std::uint64_t, const Value*>;
Index Rows(const Value& rows, const char* key, std::size_t limit) {
    Require(rows.IsArray() && rows.Size() <= limit, "Solid declaration table exceeds capacity");
    Index index;
    for (const auto& row : rows.GetArray())
        Require(index.emplace(Unsigned(row, key), &row).second, "Duplicate solid declaration identity");
    return index;
}
const Value& Find(const Index& index, std::uint64_t id) {
    const auto found = index.find(id);
    Require(found != index.end(), "Missing selected solid declaration");
    return *found->second;
}
}
void ReadDeclarations(const source::CanonicalData& source, const std::string& member, Data& data, Limits limits) {
    output::Document scope, canonical;
    constexpr unsigned flags = rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag |
                               rapidjson::kParseValidateEncodingFlag;
    scope.Parse<flags>(source.scope_bytes.data(), source.scope_bytes.size());
    canonical.Parse<flags>(source.canonical_bytes.data(), source.canonical_bytes.size());
    Require(!scope.HasParseError() && !canonical.HasParseError(), "Invalid solid declaration metadata");
    const auto& files = Member(canonical, "source_files");
    const auto& blocks = Array(Member(files, "yaris-coarse-v1l.key"), "blocks", limits.blocks, 1);
    for (const auto& file : files.GetObject()) {
        for (const auto& block : Member(file.value, "blocks").GetArray()) {
            const auto key = Text(block, "keyword");
            Require(key.rfind("*ELEMENT_SOLID_", 0) != 0 && key.rfind("*INITIAL_STRESS_SOLID", 0) != 0,
                    "Solid extended geometry or initial-stress source is outside selected profile");
        }
    }
    const auto& declarations = Member(scope, "declarations");
    const auto parts = Rows(Member(declarations, "parts"), "source_part_id", limits.blocks);
    const auto& tables = Member(declarations, "tables");
    const auto sections = Rows(Member(tables, "section"), "identity", limits.blocks);
    const auto materials = Rows(Member(tables, "material"), "identity", limits.blocks);
    const auto curves = Rows(Member(tables, "curve"), "identity", limits.blocks);
    tied_shell::detail::Draft requests;
    tied_shell::Limits source_limits;
    source_limits.metadata_bytes = limits.metadata_bytes;
    source_limits.blocks = limits.blocks;
    const auto request = [&](const Value& row) {
        return tied_shell::detail::RequestSource(requests, row, blocks, source_limits);
    };
    const auto census = ExpectedCensus(data.policy);
    data.parts.reserve(census.parts);
    for (const auto& [id, original] : parts) {
        if (!Selected(id, data.policy)) continue;
        const auto& declared = source::FindPart(source, id);
        Require(!declared.shell_section && Unsigned(*original, "source_section_id") == declared.section &&
                Unsigned(*original, "source_material_id") == declared.material,
                "Selected solid PART differs from canonical association");
        Part part;
        part.id = id;
        part.section_id = declared.section;
        part.material_id = declared.material;
        part.sources = {request(*original), request(Find(sections, declared.section)),
                        request(Find(materials, declared.material))};
        if (id == AdhesivePart) part.curve_source = request(Find(curves, 2100010));
        if (SelectedRear(id, data.policy)) part.curve_source = request(Find(curves, 2100270));
        if (SelectedRadiator(id, data.policy)) part.curve_source = request(Find(curves, 2100015));
        data.parts.push_back(part);
    }
    Require(data.parts.size() == census.parts, "Selected solid PART inventory is incomplete");
    // HOURGLASS is retained in the authenticated complete block inventory, not
    // the shell declaration tables. Read its source evidence using the same helper.
    for (const auto& block : blocks.GetArray()) {
        if (Text(block, "keyword") != "*HOURGLASS") continue;
        const auto first = Unsigned(block, "first_line");
        tied_shell::detail::Request value;
        value.evidence.block = {"yaris-coarse-v1l.key", "*HOURGLASS", {},
            Text(block, "source_block_sha256"), first, Unsigned(block, "last_line")};
        output::arrays::CheckHash(value.evidence.block.sha256);
        Require(requests.requests.emplace(first, std::move(value)).second, "Duplicate solid hourglass source");
    }
    data.sources = tied_shell::detail::ReadRequestedSources(requests.requests, member, source_limits);
    std::map<std::size_t, std::size_t> source_rows;
    for (std::size_t i = 0; i < data.sources.size(); ++i)
        source_rows.emplace(data.sources[i].block.first_line, i);
    for (auto& part : data.parts) {
        for (auto& row : part.sources) row = source_rows.at(row);
        if (part.curve_source != SIZE_MAX) part.curve_source = source_rows.at(part.curve_source);
        ReadPart(part, data.sources, data);
    }
    for (auto& part : data.parts) PrepareMaterial(part, data);
}
} // namespace crash::modelio::solid_source::detail
