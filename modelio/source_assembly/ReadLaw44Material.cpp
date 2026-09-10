#include "JsonReader.h"
#include <algorithm>
#include <cmath>

namespace crash::modelio::assembly::reader {
namespace {
double Required(const DeclarationCard& card, std::size_t field) {
    Require(field < card.values.size() && card.values[field], "Required material card field is blank");
    return *card.values[field];
}
} // namespace

// Both schemas retain literal cards and original SI conversion. V1 remains
// table-only; V2 requires an explicit tag on every material, including tables.
Material ReadLaw44Material(const Value& value, const Data& data) {
    Material m;
    if (data.schema == Law44InventorySchema) {
        const auto mode = Text(value, "hardening_model");
        Require(mode == "law44_tabulated" || mode == "law44_linear", "Unsupported explicit LAW44 hardening tag");
        if (mode == "law44_linear") m.hardening = MaterialHardening::LinearLaw44;
    } else {
        Require(!value.HasMember("hardening_model"), "Tagged materials require the explicit V2 schema");
    }
    const bool linear = m.hardening == MaterialHardening::LinearLaw44;
    m.id = Unsigned(value, "material_id", UINT32_MAX);
    m.curve_id = Unsigned(value, "hardening_curve_id", UINT32_MAX);
    m.density_kg_m3 = Real(value, "density_kg_m3"); m.young_pa = Real(value, "young_pa");
    m.poisson_ratio = Real(value, "poisson_ratio");
    m.rate_c_per_s = Real(value, "rate_coefficient_per_s"); m.rate_p = Real(value, "rate_exponent");
    m.source_rate_type = Unsigned(value, "rate_type", 0);
    m.supplied_sigy_pa = OptionalReal(value, "supplied_sigy_pa");
    m.supplied_etan_pa = OptionalReal(value, "supplied_etan_pa");
    m.source = Block(Member(value, "source")); m.cards = Cards(value, 4);
    Require((m.source.keyword == "*MAT_PIECEWISE_LINEAR_PLASTICITY" || m.source.keyword == "*MAT_024") &&
        m.cards.size() == 4 && m.cards[0].blank_mask == (linear ? 192u : 224u) &&
        m.cards[1].blank_mask == 232 && m.cards[2].blank_mask == 255 && m.cards[3].blank_mask == 255,
        "Unsupported assembly MAT024 card options");
    Require(m.density_kg_m3 > 0 && m.young_pa > 0 && m.poisson_ratio >= 0 && m.poisson_ratio < .5 &&
        m.rate_c_per_s > 0 && m.rate_p > 0, "Invalid assembly material tuple");
    if (linear) {
        Require(!m.curve_id && m.supplied_sigy_pa && *m.supplied_sigy_pa > 0 && m.supplied_etan_pa &&
            *m.supplied_etan_pa >= 0 && *m.supplied_etan_pa < m.young_pa, "Invalid analytic SIGY/ETAN or curve association");
        const double tangent = *m.supplied_etan_pa;
        const double b = tangent * m.young_pa / (m.young_pa - tangent);
        Require(std::isfinite(b) && b >= 0 && (tangent == 0 || b > 0), "Analytic plastic modulus overflow or underflow");
    } else {
        Require(m.supplied_sigy_pa && *m.supplied_sigy_pa >= 0 && !m.supplied_etan_pa &&
            std::any_of(data.curves.begin(), data.curves.end(), [&](const auto& c) { return c.id == m.curve_id; }),
            "Invalid tabulated material tuple/curve association");
    }
    const auto& u = data.units;
    const double stress = u.mass_to_kg / (u.length_to_m * u.time_to_s * u.time_to_s);
    const double density = u.mass_to_kg / (u.length_to_m * u.length_to_m * u.length_to_m);
    const auto& first = m.cards[0]; const auto& second = m.cards[1];
    Same(Required(first, 0), double(m.id)); Same(Required(first, 1) * density, m.density_kg_m3);
    Same(Required(first, 2) * stress, m.young_pa); Same(Required(first, 3), m.poisson_ratio);
    Same(Required(first, 4) * stress, *m.supplied_sigy_pa);
    if (linear) Same(Required(first, 5) * stress, *m.supplied_etan_pa);
    Same(Required(second, 0) / u.time_to_s, m.rate_c_per_s); Same(Required(second, 1), m.rate_p);
    Same(Required(second, 2), double(m.curve_id)); Same(Required(second, 4), double(m.source_rate_type));
    return m;
}
} // namespace crash::modelio::assembly::reader
