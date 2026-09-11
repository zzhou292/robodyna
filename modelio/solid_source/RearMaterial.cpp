#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"

namespace crash::modelio::solid_source::detail {
namespace {
void ReadRearCurve(const tied_shell::SourceEvidence& source, Data& data) {
    ReadCurveData(source, 2100270, 46, 1e6, data.rear_plastic_strain, data.rear_yield_stress_pa);
}
}
void ReadRearMaterial(Part& part, const tied_shell::SourceEvidence& source, Data& data) {
    Require(source.block.keyword == "*MAT_PIECEWISE_LINEAR_PLASTICITY" && source.cards.size() == 4,
            "Original rear solid requires tabulated MAT024");
    const auto& first = source.cards[0].second;
    const auto& controls = source.cards[1].second;
    Require(Required(first, 2) == (part.id == 2000016 ? 50000.0 : 200000.0) &&
        Required(first, 3) == .3 && Required(first, 4) == 270,
        "Original rear metal elastic/SIGY declaration changed");
    Blank(first, 5, 8);
    Require(Required(controls, 0) == 8000 && Required(controls, 1) == 8 &&
        tied_shell::detail::CardId(controls, 2) == 2100270 &&
        !vehicle::detail::SourceScalar(controls, 3) && Required(controls, 4) == 0,
        "Rear metal requires original filtered-rate LAW44 conversion");
    Blank(controls, 5, 8);
    Blank(source.cards[2].second, 0, 8);
    Blank(source.cards[3].second, 0, 8);
    part.material_law = MaterialLaw::Law44;
    part.density_kg_m3 = Required(first, 1) * 1e12;
    auto& material = part.law44.material;
    material.young_pa = Required(first, 2) * 1e6;
    material.poisson_ratio = Required(first, 3);
    material.density_kg_m3 = part.density_kg_m3;
    material.rate_c_per_s = Required(controls, 0);
    material.rate_p = Required(controls, 1);
    // Qualified MAT024 VP0 filtered-rate mapping and native default cutoff.
    material.cutoff_hz = 10000;
    material.native_units = tl::material::law44::solid::WorkingUnits::TonneMillimetreSecond;
    part.converter_isolid = 18;
    Require(part.curve_source < data.sources.size(), "Missing original rear LAW44 curve");
    ReadRearCurve(data.sources[part.curve_source], data);
}
} // namespace crash::modelio::solid_source::detail
