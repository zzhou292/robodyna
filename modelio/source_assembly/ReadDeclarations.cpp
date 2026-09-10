#include "JsonReader.h"
#include <algorithm>

namespace crash::modelio::assembly::reader {
namespace {
template<class T> void Append(std::vector<T>& table, T entry) {
    Require(entry.id && (table.empty() || table.back().id < entry.id), "Duplicate/unordered assembly declaration ID");
    table.push_back(std::move(entry));
}
double CardValue(const DeclarationCard& card, std::size_t field) {
    Require(field < card.values.size() && card.values[field].has_value(), "Required source card field is blank");
    return *card.values[field];
}
template<class T> bool Contains(const std::vector<T>& table, SourceId id) {
    return std::any_of(table.begin(), table.end(), [=](const auto& row) { return row.id == id; });
}
}  // namespace
void ReadDeclarations(const Value& document, const ReadLimits& limits, Data& data) {
    const auto& declarations = Member(document, "declarations");
    const auto& units = Member(declarations, "units");
    TextIs(units, "mass", "t"); TextIs(units, "length", "mm"); TextIs(units, "time", "s");
    data.units = {Real(units, "mass_to_kg"), Real(units, "length_to_m"), Real(units, "time_to_s")};
    Same(data.units.mass_to_kg, 1000); Same(data.units.length_to_m, .001); Same(data.units.time_to_s, 1);
    const double stress_scale = data.units.mass_to_kg / (data.units.length_to_m * data.units.time_to_s * data.units.time_to_s);
    const double density_scale = data.units.mass_to_kg / (data.units.length_to_m * data.units.length_to_m * data.units.length_to_m);
    std::size_t curve_points = 0;
    for (const auto& value : Array(declarations, "curves", limits.tables, 1).GetArray()) {
        Curve curve; curve.id = Unsigned(value, "curve_id", UINT32_MAX);
        curve.source = Block(Member(value, "source")); curve.cards = Cards(value, limits.curve_points + 1);
        Require(curve.source.keyword == "*DEFINE_CURVE", "Unsupported assembly curve keyword");
        const auto& x = Array(value, "plastic_strain", limits.curve_points - curve_points, 2);
        curve_points += x.Size(); // Shape admission above checks the remaining total pool first.
        const auto& y = Array(value, "stress_pa", x.Size(), x.Size());
        Require(curve.cards.size() == x.Size() + 1, "Curve source card count changed");
        Same(CardValue(curve.cards[0], 0), double(curve.id));
        Same(CardValue(curve.cards[0], 1), 0);
        for (unsigned field = 2; field < 6; ++field)
            if (curve.cards[0].values.at(field)) Same(*curve.cards[0].values[field], field < 4 ? 1 : 0);
        for (unsigned i = 0; i < x.Size(); ++i) {
            curve.plastic_strain.push_back(Real(x[i])); curve.stress_pa.push_back(Real(y[i]));
            Require(curve.plastic_strain.back() >= 0 && curve.stress_pa.back() > 0 &&
                (!i || (curve.plastic_strain[i] > curve.plastic_strain[i - 1] && curve.stress_pa[i] >= curve.stress_pa[i - 1])),
                "Unsupported assembly hardening curve ordering");
            Same(CardValue(curve.cards[i + 1], 0), curve.plastic_strain[i]);
            Same(CardValue(curve.cards[i + 1], 1) * stress_scale, curve.stress_pa[i]);
        }
        Same(curve.plastic_strain.front(), 0);
        Append(data.curves, std::move(curve));
    }
    for (const auto& value : Array(declarations, "materials", limits.tables, 1).GetArray()) {
        Material material;
        material.id = Unsigned(value, "material_id", UINT32_MAX);
        material.curve_id = Unsigned(value, "hardening_curve_id", UINT32_MAX);
        material.density_kg_m3 = Real(value, "density_kg_m3"); material.young_pa = Real(value, "young_pa");
        material.poisson_ratio = Real(value, "poisson_ratio");
        material.rate_c_per_s = Real(value, "rate_coefficient_per_s"); material.rate_p = Real(value, "rate_exponent");
        material.source_rate_type = Unsigned(value, "rate_type", 0);
        material.supplied_sigy_pa = OptionalReal(value, "supplied_sigy_pa");
        material.supplied_etan_pa = OptionalReal(value, "supplied_etan_pa");
        material.source = Block(Member(value, "source")); material.cards = Cards(value, 4);
        Require((material.source.keyword == "*MAT_PIECEWISE_LINEAR_PLASTICITY" || material.source.keyword == "*MAT_024") &&
            material.cards.size() == 4 && material.cards[0].blank_mask == 224 && material.cards[1].blank_mask == 232 &&
            material.cards[2].blank_mask == 255 && material.cards[3].blank_mask == 255,
            "Unsupported assembly MAT024 card options");
        Require(material.density_kg_m3 > 0 && material.young_pa > 0 && material.poisson_ratio >= 0 &&
            material.poisson_ratio < .5 && material.rate_c_per_s > 0 && material.rate_p > 0 &&
            material.supplied_sigy_pa && *material.supplied_sigy_pa >= 0 && !material.supplied_etan_pa &&
            Contains(data.curves, material.curve_id), "Invalid assembly material tuple/curve association");
        const auto& first = material.cards[0]; const auto& second = material.cards[1];
        Same(CardValue(first, 0), double(material.id)); Same(CardValue(first, 1) * density_scale, material.density_kg_m3);
        Same(CardValue(first, 2) * stress_scale, material.young_pa); Same(CardValue(first, 3), material.poisson_ratio);
        Same(CardValue(first, 4) * stress_scale, *material.supplied_sigy_pa);
        Same(CardValue(second, 0) / data.units.time_to_s, material.rate_c_per_s);
        Same(CardValue(second, 1), material.rate_p); Same(CardValue(second, 2), double(material.curve_id));
        Same(CardValue(second, 4), double(material.source_rate_type));
        Append(data.materials, std::move(material));
    }
    for (const auto& value : Array(declarations, "sections", limits.tables, 1).GetArray()) {
        Section section;
        section.id = Unsigned(value, "section_id", UINT32_MAX);
        section.source_elform = Unsigned(value, "source_elform", 16);
        section.through_thickness_points = Unsigned(value, "through_thickness_points", 3);
        section.source = Block(Member(value, "source")); section.cards = Cards(value, 2);
        Require(section.source.keyword == "*SECTION_SHELL" && section.cards.size() == 2 &&
            section.cards[0].blank_mask == 244 && section.cards[1].blank_mask == 240 &&
            (section.source_elform == 2 || section.source_elform == 16) && section.through_thickness_points == 3,
            "Unsupported assembly source section options");
        Same(CardValue(section.cards[0], 0), double(section.id));
        Same(CardValue(section.cards[0], 1), double(section.source_elform)); Same(CardValue(section.cards[0], 3), 3);
        const auto& thickness = Array(value, "thickness_m", 4, 4);
        for (unsigned i = 0; i < 4; ++i) {
            section.thickness_m[i] = Real(thickness[i]);
            Require(section.thickness_m[i] > 0, "Assembly thickness must be positive");
            Same(section.thickness_m[i], section.thickness_m[0]);
            Same(CardValue(section.cards[1], i) * data.units.length_to_m, section.thickness_m[i]);
        }
        Append(data.sections, std::move(section));
    }
    for (const auto& value : Array(declarations, "parts", limits.parts, 1).GetArray()) {
        Part part;
        part.id = Unsigned(value, "part_id", UINT32_MAX); part.material_id = Unsigned(value, "material_id", UINT32_MAX);
        part.section_id = Unsigned(value, "section_id", UINT32_MAX); part.title = Text(value, "title");
        part.source = Block(Member(value, "source")); part.cards = Cards(value, 1);
        Require(part.source.keyword == "*PART" && part.cards[0].blank_mask == 248 &&
            Contains(data.materials, part.material_id) && Contains(data.sections, part.section_id),
            "Unsupported assembly part declaration/association");
        Same(CardValue(part.cards[0], 0), double(part.id)); Same(CardValue(part.cards[0], 1), double(part.section_id));
        Same(CardValue(part.cards[0], 2), double(part.material_id));
        Append(data.parts, std::move(part));
    }
    const auto selected = Ids(Member(document, "selected_part_ids"), limits.parts, true);
    Require(selected.size() == data.parts.size(), "Selected assembly parts do not match declarations");
    for (std::size_t i = 0; i < selected.size(); ++i) Require(selected[i] == data.parts[i].id, "Selected assembly PID mismatch");
}
}  // namespace crash::modelio::assembly::reader
