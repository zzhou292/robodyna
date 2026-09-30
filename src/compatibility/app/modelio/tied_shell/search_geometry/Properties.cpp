#include "Internal.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace crash::modelio::tied_shell::search_detail {
namespace {
using Rows = std::map<SourceId, const Value*>;
Rows Index(const Value& values, const char* key, std::size_t cap) {
    Require(values.IsArray() && values.Size() <= cap, "Tied search declaration table exceeds capacity");
    Rows result;
    for (const auto& row : values.GetArray())
        Require(result.emplace(Unsigned(row, key), &row).second, "Duplicate tied search declaration");
    return result;
}
const Value& Find(const Rows& rows, SourceId id) {
    const auto found = rows.find(id);
    Require(found != rows.end(), "Missing original tied search coefficient declaration");
    return *found->second;
}
double Required(const std::string& row, unsigned field) {
    const auto value = vehicle::detail::SourceScalar(row, field);
    Require(value.has_value(), "Missing tied search source coefficient");
    return *value;
}
void ReadProperty(SearchProperty& p, const std::vector<SourceEvidence>& sources) {
    const auto& part = sources.at(p.sources[0]);
    const auto& section = sources.at(p.sources[1]);
    const auto& material = sources.at(p.sources[2]);
    Require(part.block.keyword == "*PART" && part.cards.size() == 2 &&
            detail::CardId(part.cards[1].second, 0) == p.part_id &&
            detail::CardId(part.cards[1].second, 1) == p.section_id &&
            detail::CardId(part.cards[1].second, 2) == p.material_id,
            "Tied search requires original plain PART identity");
    Require(section.block.keyword == "*SECTION_SHELL" && section.cards.size() == 2 &&
            detail::CardId(section.cards[0].second, 0) == p.section_id,
            "Tied search requires original ordinary SECTION_SHELL");
    const auto& first = section.cards[0].second;
    const auto elform = Required(first, 1);
    Require(elform == 2 || elform == 9 || elform == 16,
            "Tied search property conversion is outside ordinary TYPE1 scope");
    for (unsigned field = 4; field < 8; ++field) {
        const auto value = vehicle::detail::SourceScalar(first, field);
        Require(!value || *value == 0, "Tied search source integration/property modifier is unsupported");
    }
    Require(!material.cards.empty() && detail::CardId(material.cards[0].second, 0) == p.material_id,
            "Tied search original material identity changed");
    const auto& keyword = material.block.keyword;
    p.rank_modulus_available = keyword == "*MAT_PIECEWISE_LINEAR_PLASTICITY" ||
        keyword == "*MAT_MODIFIED_PIECEWISE_LINEAR_PLASTICITY";
    Require(p.rank_modulus_available || keyword == "*MAT_ELASTIC" || keyword == "*MAT_RIGID",
            "Tied search source material property conversion is unsupported");
    // Native ordinary section conversion uses T1, regardless of source NIP or
    // NLOC. This is geometry-only coefficient production, not shell admission.
    p.geometry_thickness = Required(section.cards[1].second, 0);
    Require(std::isfinite(p.geometry_thickness) && p.geometry_thickness > 0,
            "Tied search source thickness must be finite and positive");
    if (p.rank_modulus_available) {
        p.rank_modulus = Required(material.cards[0].second, 2);
        Require(p.rank_modulus > 0, "Tied native PM20 source modulus must be positive");
    }
    // Exact plain PART/ELEMENT_SHELL converter defaults; no virtual thickness
    // or element THICK extension is admitted by the source profile below.
    p.part_override = 0;
    p.element_override = 0;
}
}
void Properties(const source::CanonicalData& source, const Data&, const Topology& topology,
        const std::string& member, SearchGeometryData& d, SearchGeometryLimits limits) {
    output::Document scope, canonical;
    scope.Parse(source.scope_bytes.c_str());
    canonical.Parse(source.canonical_bytes.c_str());
    Require(!scope.HasParseError() && !canonical.HasParseError(), "Invalid tied search source metadata");
    const auto& files = Member(canonical, "source_files");
    const auto& blocks = Member(Member(files, "yaris-coarse-v1l.key"), "blocks");
    Require(blocks.IsArray() && blocks.Size() <= Limits{}.blocks, "Tied search source block cap exceeded");
    for (const auto& file : files.GetObject()) {
        for (const auto& block : Member(file.value, "blocks").GetArray()) {
            const auto keyword = Text(block, "keyword");
            Require(keyword.find("*ELEMENT_SHELL_") != 0 && keyword.find("*INITIAL_STRESS_SHELL") != 0 &&
                    keyword.find("*INITIAL_THICKNESS") != 0,
                    "Tied search original element/initial thickness override is unsupported");
        }
    }
    const auto& declarations = Member(scope, "declarations");
    const auto parts = Index(Member(declarations, "parts"), "source_part_id", limits.parts);
    const auto& tables = Member(declarations, "tables");
    const auto sections = Index(Member(tables, "section"), "identity", Limits{}.blocks);
    const auto materials = Index(Member(tables, "material"), "identity", Limits{}.blocks);
    std::set<SourceId> needed;
    for (const auto& match : d.matches) needed.insert(topology.shells[6*match.canonical_shell_row+1]);
    Require(needed.size() <= limits.parts, "Tied search matching part count exceeds capacity");
    detail::Draft requests;
    Limits read_limits;
    read_limits.metadata_bytes = limits.metadata_bytes;
    std::map<SourceId, std::uint32_t> part_rows;
    d.properties.reserve(needed.size());
    for (auto id : needed) {
        const auto& native = source::FindPart(source, id);
        const auto& original = Find(parts, id);
        Require(native.shell_section && Unsigned(original, "source_section_id") == native.section &&
                Unsigned(original, "source_material_id") == native.material,
                "Tied search original part coefficient association changed");
        SearchProperty p;
        p.part_id = id;
        p.section_id = native.section;
        p.material_id = native.material;
        p.sources = {detail::RequestSource(requests, original, blocks, read_limits),
            detail::RequestSource(requests, Find(sections, native.section), blocks, read_limits),
            detail::RequestSource(requests, Find(materials, native.material), blocks, read_limits)};
        part_rows.emplace(id, d.properties.size());
        d.properties.push_back(p);
    }
    d.sources = detail::ReadRequestedSources(requests.requests, member, read_limits);
    std::map<std::size_t, std::size_t> source_rows;
    for (std::size_t i = 0; i < d.sources.size(); ++i) source_rows.emplace(d.sources[i].block.first_line, i);
    for (auto& p : d.properties) {
        for (auto& row : p.sources) row = source_rows.at(row);
        ReadProperty(p, d.sources);
    }
    for (auto& match : d.matches) match.property = part_rows.at(topology.shells[6*match.canonical_shell_row+1]);
}
} // namespace crash::modelio::tied_shell::search_detail
