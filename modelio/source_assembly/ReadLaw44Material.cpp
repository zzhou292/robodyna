#include "MaterialDeclarationFields.h"
#include <algorithm>
#include <cmath>

namespace crash::modelio::assembly::reader {
// Every schema retains literal cards and original SI conversion. V1 remains
// table-only; V2/V3 require an explicit hardening tag on each LAW44 material.
namespace {
Material ReadLaw44Fields(const Value& value, const Data& data, bool constant_failure) {
    Material m;
    if (data.schema == Law44InventorySchema || data.schema == SectionInventorySchema) {
        const auto mode = Text(value, "hardening_model");
        Require(mode == "law44_tabulated" || mode == "law44_linear", "Unsupported explicit LAW44 hardening tag");
        if (mode == "law44_linear") m.hardening = MaterialHardening::LinearLaw44;
    } else {
        Require(!value.HasMember("hardening_model"), "Tagged materials require the explicit V2 schema");
    }
    const bool linear = m.hardening == MaterialHardening::LinearLaw44;
    ReadMaterialTuple(value,m);
    m.curve_id = Unsigned(value, "hardening_curve_id", UINT32_MAX);
    m.rate_c_per_s = Real(value, "rate_coefficient_per_s"); m.rate_p = Real(value, "rate_exponent");
    m.source_rate_type = Unsigned(value, "rate_type", 0);
    m.supplied_sigy_pa = OptionalReal(value, "supplied_sigy_pa");
    m.supplied_etan_pa = OptionalReal(value, "supplied_etan_pa");
    m.source = Block(Member(value, "source")); m.cards = Cards(value, 4);
    const auto expected_first_mask = linear ? (constant_failure ? 128u : 192u) :
        (constant_failure ? (m.supplied_etan_pa ? 128u : 160u) : 224u);
    Require((m.source.keyword == "*MAT_PIECEWISE_LINEAR_PLASTICITY" || m.source.keyword == "*MAT_024") &&
        m.cards.size() == 4 && m.cards[0].blank_mask == expected_first_mask &&
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
        // The pinned positive-C/P table branch overwrites native B with zero.
        // Preserve a supplied zero ETAN only in the explicit failure scope;
        // legacy table cards still require the original blank field.
        const bool table_etan = !m.supplied_etan_pa || (constant_failure && *m.supplied_etan_pa == 0);
        Require(m.supplied_sigy_pa && *m.supplied_sigy_pa >= 0 && table_etan &&
            std::any_of(data.curves.begin(), data.curves.end(), [&](const auto& c) { return c.id == m.curve_id; }),
            "Invalid tabulated material tuple/curve association");
    }
    const auto& u = data.units;
    const double stress = MaterialStressScale(u);
    const double density = MaterialDensityScale(u);
    const auto& first = m.cards[0]; const auto& second = m.cards[1];
    Same(RequiredMaterialCard(first, 0), double(m.id)); Same(RequiredMaterialCard(first, 1) * density, m.density_kg_m3);
    Same(RequiredMaterialCard(first, 2) * stress, m.young_pa); Same(RequiredMaterialCard(first, 3), m.poisson_ratio);
    Same(RequiredMaterialCard(first, 4) * stress, *m.supplied_sigy_pa);
    if (m.supplied_etan_pa) Same(RequiredMaterialCard(first, 5) * stress, *m.supplied_etan_pa);
    Same(RequiredMaterialCard(second, 0) / u.time_to_s, m.rate_c_per_s); Same(RequiredMaterialCard(second, 1), m.rate_p);
    Same(RequiredMaterialCard(second, 2), double(m.curve_id)); Same(RequiredMaterialCard(second, 4), double(m.source_rate_type));
    return m;
}
} // namespace

Material ReadLaw44Material(const Value& value, const Data& data) {
    Require(!value.HasMember("failure_strain"), "Failure declarations require an explicit runtime resolution catalog");
    return ReadLaw44Fields(value, data, false);
}

Material ReadConstantFailureMaterial(const Value& value, const Data& data) {
    Require(data.schema == SectionInventorySchema, "Constant failure requires typed section declarations");
    TextIs(value, "material_law", "layered_law44");
    const auto failure_strain = Real(value, "failure_strain");
    Require(failure_strain > 0, "Constant failure requires literal positive FAIL");
    auto material = ReadLaw44Fields(value, data, true);
    Same(RequiredMaterialCard(material.cards[0], 6), failure_strain);
    return material;
}
} // namespace crash::modelio::assembly::reader
