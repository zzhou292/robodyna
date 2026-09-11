#include "Internal.h"
#include <algorithm>
#include <map>
#include <set>
#include <sstream>

namespace crash::modelio::vehicle::resolution {
namespace {
bool Eligible(const PartDisposition& part) {
    if (part.status != Disposition::Unresolved || part.obligations.empty() ||
        std::any_of(part.obligations.begin(), part.obligations.end(), [](const auto& obligation) {
            return obligation.stage != "material";
        })) return false;
    const auto& source = part.unresolved_sources[2];
    if (source.keyword != "*MAT_024" && source.keyword != "*MAT_PIECEWISE_LINEAR_PLASTICITY") return false;
    std::istringstream stream(source.raw_text);
    std::string line;
    while (std::getline(stream, line)) {
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '*' || line[first] == '$') continue;
        const auto failure = detail::SourceScalar(line, 6);
        return failure && *failure > 0;
    }
    return false;
}
void SameBlock(const assembly::SourceBlock& current, const assembly::SourceBlock& original) {
    Require(current.filename == original.filename && current.keyword == original.keyword &&
            current.first_line == original.first_line && current.last_line == original.last_line &&
            current.sha256 == original.sha256 && current.raw_text == original.raw_text,
            "Resolved declaration changed its original source block");
}
template<class T> std::size_t Index(const std::vector<T>& rows, std::uint64_t id) {
    const auto found = std::lower_bound(rows.begin(), rows.end(), id, [](const auto& row, std::uint64_t value) {
        return row.id < value;
    });
    Require(found != rows.end() && found->id == id, "Missing resolved declaration identity");
    return found - rows.begin();
}
void Count(ResolutionCounts& counts, SectionDisposition status, std::size_t shells) {
    ++counts.parts;
    counts.shells += shells;
    if (status == SectionDisposition::Existing) {
        ++counts.existing_parts;
        counts.existing_shells += shells;
    } else if (status == SectionDisposition::ConstantFailure) {
        ++counts.failure_parts;
        counts.failure_shells += shells;
    } else {
        ++counts.unresolved_parts;
        counts.unresolved_shells += shells;
    }
}
void CheckCounts(const Value& value, const ResolutionCounts& counts) {
    Require(Unsigned(value,"parts") == counts.parts && Unsigned(value,"shells") == counts.shells &&
            Unsigned(value,"existing_parts") == counts.existing_parts &&
            Unsigned(value,"existing_shells") == counts.existing_shells &&
            Unsigned(value,"failure_parts") == counts.failure_parts &&
            Unsigned(value,"failure_shells") == counts.failure_shells &&
            Unsigned(value,"unresolved_parts") == counts.unresolved_parts &&
            Unsigned(value,"unresolved_shells") == counts.unresolved_shells,
            "Vehicle section resolution coverage disagrees");
}
} // namespace

Declarations ReadDeclarations(const VehicleSourcePlan& plan, const Value& doc, ResolutionLimits limits) {
    output::Document scope;
    const auto& bytes = plan.canonical().data().scope_bytes;
    scope.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(), bytes.size());
    Require(!scope.HasParseError(), "Invalid retained authenticated source scope");
    std::map<std::uint64_t, const Value*> original_parts;
    for (const auto& row : Member(Member(scope, "declarations"), "parts").GetArray()) {
        Require(original_parts.emplace(Unsigned(row, "source_part_id"), &row).second,
                "Duplicate original part identity");
    }
    Declarations next;
    next.failure.schema = assembly::SectionInventorySchema;
    const auto& typed = Member(doc, "constant_failure_declarations");
    TextIs(typed, "schema", assembly::SectionInventorySchema);
    ReadMaterialPolicy(typed, next.failure);
    assembly::ReadLimits read;
    read.parts = limits.parts;
    read.tables = limits.tables;
    read.curve_points = limits.curve_points;
    ReadConstantFailureDeclarations(typed, read, next.failure);
    const auto& rows = Array(doc, "parts", limits.parts, 1);
    Require(rows.Size() == plan.parts().size(), "Resolution must retain every original selected part");
    std::set<std::uint64_t> used_materials, used_sections, used_curves;
    for (const auto& row : rows.GetArray()) {
        const auto& original = plan.parts()[next.parts.size()];
        Require(Unsigned(row,"part_id") == original.part_id &&
                Unsigned(row,"material_id") == original.material_id &&
                Unsigned(row,"section_id") == original.section_id &&
                Unsigned(row,"shells") == original.shell_count,
                "Resolution part/source/count order changed");
        SectionPartResolution part;
        const char* expected = "unresolved";
        if (original.status == Disposition::SupportedDeclaration) {
            part.status = SectionDisposition::Existing;
            expected = "existing";
        } else if (Eligible(original)) {
            part.status = SectionDisposition::ConstantFailure;
            expected = "constant_failure";
        }
        TextIs(row, "status", expected);
        if (part.status == SectionDisposition::ConstantFailure) {
            const auto& source = next.failure.parts[Index(next.failure.parts, original.part_id)];
            Require(source.material_id == original.material_id && source.section_id == original.section_id,
                    "Resolved source part changed its material/section association");
            const auto origin = original_parts.find(original.part_id);
            Require(origin != original_parts.end() && source.title == Text(*origin->second, "title"),
                    "Resolved source part title changed");
            part.material_index = Index(next.failure.materials, original.material_id);
            part.section_index = Index(next.failure.sections, original.section_id);
            const auto& material = next.failure.materials[part.material_index];
            const auto& section = next.failure.sections[part.section_index];
            SameBlock(source.source, original.unresolved_sources[0]);
            SameBlock(section.source, original.unresolved_sources[1]);
            SameBlock(material.source, original.unresolved_sources[2]);
            detail::CheckTypedCards(source.source, source.cards);
            detail::CheckTypedCards(section.source, section.cards);
            detail::CheckTypedCards(material.source, material.cards);
            part.failure_strain = *material.cards[0].values[6];
            used_materials.insert(material.id);
            used_sections.insert(section.id);
            if (material.curve_id) used_curves.insert(material.curve_id);
        }
        Count(next.counts, part.status, original.shell_count);
        next.parts.push_back(part);
    }
    Require(next.counts.failure_parts == next.failure.parts.size() &&
            used_materials.size() == next.failure.materials.size() &&
            used_sections.size() == next.failure.sections.size() &&
            used_curves.size() == next.failure.curves.size(), "Unreferenced resolved declaration");
    const auto& originals = Member(Member(Member(scope,"declarations"),"tables"),"curve");
    std::map<std::uint64_t, const Value*> curves;
    for (const auto& row : originals.GetArray()) {
        Require(curves.emplace(Unsigned(row,"identity"), &row).second, "Duplicate original curve identity");
    }
    for (const auto& curve : next.failure.curves) {
        const auto found = curves.find(curve.id);
        Require(found != curves.end() && used_curves.count(curve.id), "Missing original failure hardening curve");
        detail::CheckSource(curve.source, *found->second);
        detail::CheckTypedCards(curve.source, curve.cards, 20);
    }
    CheckCounts(Member(doc,"counts"), next.counts);
    return next;
}
} // namespace crash::modelio::vehicle::resolution
