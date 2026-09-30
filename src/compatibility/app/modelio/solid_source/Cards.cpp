#include "modelio/source_assembly/SourceFields.h"
#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"

namespace crash::modelio::solid_source::detail {
double Required(const std::string& row,unsigned column,unsigned width) {
    return assembly::reader::RequiredScalar(row,column,width);
}
void Blank(const std::string& row,unsigned first,unsigned last) {
    assembly::reader::RequireBlankFields(row,first,last);
}
void ReadCurve(const tied_shell::SourceEvidence& source, Data& data) {
    Require(data.plastic_strain.empty() && data.yield_stress_pa.empty(), "Repeated adhesive curve");
    ReadCurveData(source, 2100010, 8, 1e6, data.plastic_strain, data.yield_stress_pa);
}
void ReadPart(Part& part, const std::vector<tied_shell::SourceEvidence>& sources, Data& data) {
    const auto& original = sources.at(part.sources[0]);
    const auto& section = sources.at(part.sources[1]);
    const auto& material = sources.at(part.sources[2]);
    Require(original.block.keyword == "*PART" && original.cards.size() == 2 &&
        section.block.keyword == "*SECTION_SOLID" && section.cards.size() == 1,
        "Selected solid requires original plain PART and SECTION_SOLID");
    const auto& p = original.cards[1].second;
    const auto& s = section.cards[0].second;
    Require(tied_shell::detail::CardId(p, 0) == part.id &&
        tied_shell::detail::CardId(p, 1) == part.section_id &&
        tied_shell::detail::CardId(p, 2) == part.material_id &&
        part.id == part.section_id && part.id == part.material_id &&
        tied_shell::detail::CardId(s, 0) == part.section_id,
        "Selected solid source PART/SECTION/MATERIAL association changed");
    Require(!material.cards.empty() && tied_shell::detail::CardId(material.cards[0].second, 0) == part.material_id,
            "Selected solid material identity changed");
    if (SelectedAirbag(part.id, data.policy)) {
        Blank(p, 3, 8);
        Blank(s, 1, 8); // Native default uses the global IHQ4 block; retain blank ELFORM.
        ReadAirbagMaterial(part, material, data);
        return;
    }
    if (SelectedRadiator(part.id, data.policy)) {
        Blank(p, 3, 8);
        Require(Required(s, 1) == 2, "Original radiator ELFORM is not two");
        Blank(s, 2, 8);
        ReadRadiatorMaterial(part, material, data);
        return;
    }
    if (SelectedRear(part.id, data.policy)) {
        Blank(p, 3, 8);
        Require(Required(s, 1) == 2, "Original rear-metal ELFORM is not two");
        Blank(s, 2, 8);
        ReadRearMaterial(part, material, data);
        return;
    }
    if (part.id == AdhesivePart) {
        part.material_law = MaterialLaw::Law36;
        Blank(p, 3, 8);
        Require(Required(s, 1) == 2, "Original adhesive ELFORM is not two");
        Blank(s, 2, 8);
        Require(material.block.keyword == "*MAT_PIECEWISE_LINEAR_PLASTICITY" && material.cards.size() == 4,
                "Original adhesive requires its tabulated MAT024 source");
        const auto& first = material.cards[0].second;
        part.density_kg_m3 = Required(first, 1) * 1e12;
        part.law36.young_pa = Required(first, 2) * 1e6;
        part.law36.poisson_ratio = Required(first, 3);
        // SIGY remains a literal unused table-branch field. It is not substituted
        // for the more precise first curve ordinate.
        Require(Required(first, 4) > 0, "Original adhesive SIGY must remain positive");
        Blank(first, 5, 8);
        const auto& controls = material.cards[1].second;
        Require(!vehicle::detail::SourceScalar(controls, 0) && !vehicle::detail::SourceScalar(controls, 1) &&
            tied_shell::detail::CardId(controls, 2) == 2100010 &&
            !vehicle::detail::SourceScalar(controls, 3) && Required(controls, 4) == 0,
            "Adhesive requires the original no-rate LAW36 converter branch");
        Blank(controls, 5, 8);
        Blank(material.cards[2].second, 0, 8);
        Blank(material.cards[3].second, 0, 8);
        part.converter_isolid = 18;
        Require(part.curve_source < sources.size(), "Missing adhesive curve association");
        ReadCurve(sources[part.curve_source], data);
    } else {
        part.material_law = MaterialLaw::Law42;
        Require(SelectedRubber(part.id, data.policy),
                "Rubber source PID is outside named demo selection");
        Require(!vehicle::detail::SourceScalar(p, 3) && tied_shell::detail::CardId(p, 4) == 2000017,
                "Original rubber hourglass association changed");
        Blank(p, 5, 8);
        Blank(s, 1, 8); // Preserve blank ELFORM; do not rewrite it as24.
        Require(material.block.keyword == "*MAT_BLATZ-KO_RUBBER" && material.cards.size() == 1,
                "Original rubber requires MAT007 source");
        const auto& first = material.cards[0].second;
        part.density_kg_m3 = Required(first, 1) * 1e12;
        part.law42.mu_pa = Required(first, 2) * 1e6;
        Blank(first, 3, 8);
        part.hourglass_id = 2000017;
        for (std::size_t i = 0; i < sources.size(); ++i) {
            const auto& hg = sources[i];
            if (hg.block.keyword != "*HOURGLASS") continue;
            Require(hg.cards.size() == 1, "Unsupported original hourglass source layout");
            if (tied_shell::detail::CardId(hg.cards[0].second, 0) != part.hourglass_id) continue;
            Require(part.hourglass_source == SIZE_MAX, "Repeated rubber hourglass identity");
            const auto& row = hg.cards[0].second;
            Require(Required(row, 1) == 2 && Required(row, 2) == .1 && Required(row, 3) == 0 &&
                Required(row, 4) == 1.5e-4 && Required(row, 5) == 6e-5,
                "Original rubber hourglass controls changed");
            Blank(row, 6, 8);
            part.hourglass_source = i;
        }
        Require(part.hourglass_source != SIZE_MAX, "Missing original rubber hourglass card");
        part.converter_isolid = 1; // Explicit original converter outcome, not effective demo element.
    }
}
} // namespace crash::modelio::solid_source::detail
