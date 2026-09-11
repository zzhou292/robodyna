#include "TestSupport.h"

namespace crash::modelio::solid_source::test {
TEST(VehicleSolidFields, OriginalBlankAndConvertedRubberPolicyStayDistinct) {
    auto data = Rubber();
    Resolve(data);
    const auto& part = data.parts[0];
    EXPECT_EQ(part.converter_isolid, 1u);
    EXPECT_EQ(part.material_law, MaterialLaw::Law42);
    EXPECT_EQ(part.hourglass_id, 2000017u);
    EXPECT_EQ(part.hourglass_source, 3u);
    EXPECT_FALSE(vehicle::detail::SourceScalar(data.sources[1].cards[0].second, 1));
    EXPECT_EQ(output::Bits(part.density_kg_m3), output::Bits(1.98e-9 * 1e12));
    EXPECT_EQ(output::Bits(part.law42.mu_pa), output::Bits(24. * 1e6));
    EXPECT_EQ(part.law42.poisson_ratio, .463);
    EXPECT_EQ(part.law42.tension_cutoff_pa, 1e20 * 1e6);
    EXPECT_EQ(part.law36.curve.count, 0u);
}
TEST(VehicleSolidFields, AdhesiveCurveOwnsValuesAndUnusedSigyIsNotYieldSubstitution) {
    auto data = Adhesive();
    Resolve(data);
    auto& part = data.parts[0];
    EXPECT_EQ(part.converter_isolid, 18u);
    EXPECT_EQ(part.material_law, MaterialLaw::Law36);
    EXPECT_EQ(part.law36.curve.count, 8u);
    EXPECT_EQ(part.law36.curve.plastic_strain, data.plastic_strain.data());
    EXPECT_EQ(part.law36.curve.yield_stress_pa, data.yield_stress_pa.data());
    EXPECT_EQ(part.law36.curve.yield_stress_pa[0], 10.1e6);
    EXPECT_NE(part.law36.curve.yield_stress_pa[0], 10.041e6);
    EXPECT_EQ(output::Bits(part.density_kg_m3), output::Bits(1.07e-9 * 1e12));
    EXPECT_EQ(part.law42.mu_pa, 0);
}
TEST(VehicleSolidFields, WrongFormulationRateFailureAndLateHourglassFieldsReject) {
    for (unsigned fault = 0; fault < 6; ++fault) {
        SCOPED_TRACE(fault);
        auto data = fault < 3 ? Rubber() : Adhesive();
        if (fault == 0) data.sources[1].cards[0].second = Card({"2000477", "24"});
        if (fault == 1) data.sources[0].cards[1].second = Card({"2000477", "2000477", "2000477", "", "2000018"});
        if (fault == 2) data.sources[3].cards[0].second = Card({"2000017", "2", ".1", "0", "1.5E-4", "6E-5", "1"});
        if (fault == 3) data.sources[2].cards[1].second = Card({"8000", "8", "2100010", "", "0"});
        if (fault == 4) data.sources[2].cards[0].second = Card({"2000977", "1.07E-9", "1887", ".417", "10.041", "", "3.5"});
        if (fault == 5) data.sources[3].cards.back().second = Card({".007", "-1"}, 20);
        EXPECT_THROW(Resolve(data), std::runtime_error);
    }
    auto retry = Adhesive();
    ASSERT_NO_THROW(Resolve(retry));
}
TEST(VehicleSolidFields, NumericalSourceBitsAndBlankFlagsArePreserved) {
    EXPECT_EQ(output::Bits(detail::Required(Card({"-0.0"}), 0)), output::Bits(-0.0));
    EXPECT_THROW(detail::Required(Card({""}), 0), std::runtime_error);
    EXPECT_THROW(detail::Required(Card({"nan"}), 0), std::runtime_error);
    auto data = Rubber();
    data.sources[1].cards[0].second = Card({"2000477", "0"});
    EXPECT_THROW(Resolve(data), std::runtime_error); // Original blank is not rewritten as zero.
}
} // namespace crash::modelio::solid_source::test
