#include "Internal.h"
#include "modelio/source_assembly/MaterialDeclarationFields.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"
#include "modelio/source_assembly/JsonReader.h"
#include <algorithm>
#include <map>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
namespace {
bool Same(double a, double b) { return tl::math::SameScalarBits(a, b); }
bool CardsEqual(const std::vector<a::DeclarationCard>& x, const std::vector<a::DeclarationCard>& y,
    bool material) noexcept {
    if (x.size() != y.size())return false;
    for (std::size_t r = 0; r<x.size(); ++r) {
        const auto& first = x[r]; const auto& second = y[r];
        if (first.names != second.names || first.values.size() != second.values.size())return false;
        for (std::size_t k = 0; k<first.values.size(); ++k) {
            if (r == 0 && k == 0)continue; // Actual MID/SECID are compared separately.
            if (!material && r == 1 && k == 4 && k<first.names.size() && first.names[k] == "nloc")continue;
            if (first.values[k].has_value() != second.values[k].has_value())return false;
            if (first.values[k] && !Same(*first.values[k], *second.values[k]))return false;
        }
    }
    return true;
}
bool GroupMaterial(const a::Material& material) noexcept {
    const auto& keyword = material.source.keyword;
    return keyword == "*MAT_ELASTIC" || keyword == "*MAT_001" ||
        keyword == "*MAT_PIECEWISE_LINEAR_PLASTICITY" || keyword == "*MAT_024" ||
        keyword == "*MAT_MODIFIED_PIECEWISE_LINEAR_PLASTICITY" || keyword == "*MAT_123";
}
}
bool GroupPrefixEqual(const PartValue& x, const PartValue& y) noexcept {
    if (!x.material || !y.material || !x.section || !y.section ||
        !x.ordinary_part_controls || !y.ordinary_part_controls ||
        !GroupMaterial(*x.material) || !GroupMaterial(*y.material))return false;
    const auto& a = *x.material; const auto& b = *y.material;
    const auto& p = *x.section; const auto& q = *y.section;
    // Ordinary isotropic property1 source profile. NLOC only changes
    // IGEO99/GEO199; none is consumed by get_sort_key_shell key1..4.
    if (a.source.keyword != b.source.keyword || a.law != b.law || a.hardening != b.hardening ||
        p.source.keyword != "*SECTION_SHELL" || q.source.keyword != p.source.keyword ||
        p.source_elform != q.source_elform || p.through_thickness_points != q.through_thickness_points)
        return false;
    return CardsEqual(a.cards, b.cards, true) && CardsEqual(p.cards, q.cards, false);
}
Packed PackShells(const c::CorrectedNodalSource& corrected, const coated::Inputs& input, Limits limits) {
    const auto& model = corrected.pre_correction().physical();
    const auto& references = model.shell_source().references();
    const auto& rows = references.rows();
    Require(references.resolution() && rows.size() == input.shells.size(),"Main coefficient shell reference domain differs");
    const auto units = corrected.pre_correction().provenance().units;
    const a::SourceUnits declared_units{units.mass_kg, units.length_m, units.time_s};
    const double pressure = a::reader::MaterialStressScale(declared_units);
    std::map<std::uint64_t, std::size_t> material_uses, part_indexes;
    for (const auto& part:corrected.part_controls())++material_uses[part.material_id];
    const auto& canonical  =  references.source().canonical().data();
    output::Document document;
    document.Parse(canonical.canonical_bytes.data(), canonical.canonical_bytes.size());
    Require(!document.HasParseError() && document.IsObject(), "Shell part source inventory is unavailable");
    std::map<std::uint64_t, bool> plain_parts;
    for (const auto& part : a::reader::Array(document, "parts", 4096).GetArray())
        plain_parts.emplace(a::reader::Unsigned(part, "source_part_id"), OrdinaryPartControls(part));
    Packed out; out.parts.reserve(std::min(limits.parts, corrected.part_controls().size()));
    out.shells.reserve(rows.size()); out.keys.reserve(rows.size());
    for (std::size_t i = 0; i<rows.size(); ++i) {
        const auto& row = rows[i]; const auto& shell = input.shells[i];
        Require(shell.physical_parent == i && shell.primary.source_id == row.element_id && shell.part_id == row.part_id,
            "Selected-shell physical reference and source row differ");
        auto found = part_indexes.find(row.part_id);
        if (found == part_indexes.end()) {
            if (out.parts.size() >= limits.parts)Reject(Status::ResourceLimit,"Main coefficient part count exceeds cap");
            const auto* material = references.resolution()->material(row.part_index);
            const auto* section = references.resolution()->section(row.part_index);
            Require(material && section && material->id == row.material_id && section->id == row.section_id,
                "Main coefficient material/property source identity differs");
            Require(section->source.keyword == "*SECTION_SHELL" &&
                (section->source_elform == 2 || section->source_elform == 16 || section->source_elform == 9) &&
                !material->cards.empty() && section->cards.size() >= 2,
                "Main coefficient source cards are unavailable");
            const double young = a::reader::RequiredMaterialCard(material->cards[0], 2);
            const double thickness = a::reader::RequiredMaterialCard(section->cards[1], 0);
            Require(std::isfinite(young) && young>0 && std::isfinite(thickness) && thickness>0 &&
                Same(young*pressure, material->young_pa) && Same(thickness*units.length_m, section->thickness_m[0]),
                "Main coefficient native source fields do not reproduce typed SI declaration");
            PartValue value;
            value.pid = row.part_id; value.sid = row.section_id; value.mid = row.material_id;
            value.material = material; value.section = section;
            value.coefficient.property_type = 1; value.coefficient.input_thickness_mode = 0;
            value.coefficient.scale = 1.; value.coefficient.young = young;
            value.coefficient.property_thickness = thickness;
            // Complete plain ELEMENT_SHELL source closure proves override0.
            value.coefficient.element_thickness = 0.;
            value.native_material_id_preserved = material_uses[row.material_id] == 1;
            const auto plain  =  plain_parts.find(row.part_id);
            value.ordinary_part_controls  =  plain != plain_parts.end() && plain->second;
            found = part_indexes.emplace(row.part_id, out.parts.size()).first;
            out.parts.push_back(value);
        }
        out.shells.push_back({found->second}); out.keys.push_back(Key(shell, i));
    }
    std::sort(out.keys.begin(), out.keys.end(), KeyLess);
    Require(out.parts.capacity() <= 2*limits.parts && out.shells.capacity() <= 2*rows.size() && out.keys.capacity() <= 2*rows.size(),
        "Main coefficient source packing exceeds reserved capacities");
    return out;
}
} // namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail
