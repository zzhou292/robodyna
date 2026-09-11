#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"

namespace crash::modelio::solid_source::detail {
namespace {
void ReadRearCurve(const tied_shell::SourceEvidence& source, Data& data) {
    Require(source.block.keyword == "*DEFINE_CURVE" && source.cards.size() == 47 &&
        tied_shell::detail::CardId(source.cards[0].second, 0) == 2100270,
        "Original rear LAW44 curve identity or point count changed");
    const auto& header = source.cards[0].second;
    Require(Required(header, 1) == 0 && Required(header, 2) == 1 && Required(header, 3) == 1,
            "Original rear LAW44 curve scales or options changed");
    Blank(header, 4, 8);
    if (!data.rear_plastic_strain.empty()) {
        Require(data.rear_plastic_strain.size() == 46 && data.rear_yield_stress_pa.size() == 46,
                "Incomplete shared rear LAW44 curve");
        return; // Both parts request the same authenticated source block.
    }
    data.rear_plastic_strain.reserve(46);
    data.rear_yield_stress_pa.reserve(46);
    for (std::size_t i = 1; i < source.cards.size(); ++i) {
        const auto& row = source.cards[i].second;
        Require(assembly::reader::auxiliary::BlankTail(row, 40), "Extra rear LAW44 curve values");
        data.rear_plastic_strain.push_back(Required(row, 0, 20));
        data.rear_yield_stress_pa.push_back(Required(row, 1, 20) * 1e6);
    }
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
