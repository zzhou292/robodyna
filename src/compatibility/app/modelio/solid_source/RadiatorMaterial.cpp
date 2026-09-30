#include "Internal.h"
#include "modelio/source_assembly/MaterialDeclarationFields.h"

namespace crash::modelio::solid_source::detail {
void ReadRadiatorMaterial(Part& part, const tied_shell::SourceEvidence& source, Data& data) {
    Require(part.id == RadiatorPart && source.block.keyword == "*MAT_LOW_DENSITY_FOAM" &&
            source.cards.size() == 2, "Original radiator requires MAT057");
    const auto& first = source.cards[0].second;
    const auto& second = source.cards[1].second;
    Require(Required(first, 1) == 7.72e-10 && Required(first, 2) == 21 &&
        tied_shell::detail::CardId(first, 3) == 2100015 && Required(first, 4) == 15,
        "Original radiator density, modulus, curve or tension cutoff changed");
    Blank(first, 5, 8); // Original HU/BETA/DAMP are blank; HU resolves to IFLAG1.
    for (unsigned i = 0; i < 5; ++i)
        Require(!vehicle::detail::SourceScalar(second, i), "Unsupported radiator material option");
    Require(Required(second, 5) == 20000, "Original radiator KCON changed");
    Blank(second, 6, 8);
    part.material_law = MaterialLaw::Law90;
    part.converter_isolid = 18;
    // CheckOriginal authenticates t/mm/s. Reuse the source-unit utilities;
    // this represented density matches the native HM_GET conversion, including
    // its final rounding, rather than a regrouped literal1e12 multiplier.
    const assembly::SourceUnits units{1000, .001, 1};
    const double stress_scale = assembly::reader::MaterialStressScale(units);
    part.density_kg_m3 = Required(first, 1) * assembly::reader::MaterialDensityScale(units);
    auto& input = part.law90_input;
    input.density_kg_m3 = part.density_kg_m3;
    input.card_young_pa = Required(first, 2) * stress_scale;
    input.tension_cutoff_pa = Required(first, 4) * stress_scale;
    // Native SDI converts the literal KCON to the LAW90 pressure dimension.
    input.contact_modulus_pa = Required(second, 5) * stress_scale;
    input.hysteresis = 0;
    input.curve_scale_dimension = stress_scale;
    Require(part.curve_source < data.sources.size(), "Missing original radiator curve");
    ReadCurveData(data.sources[part.curve_source], 2100015, 28, 1,
                  data.foam_compression_strain, data.foam_curve_ordinate);
}
} // namespace crash::modelio::solid_source::detail
