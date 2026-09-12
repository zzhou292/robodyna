#include "TestSupport.h"

namespace crash::modelio::solid_source::test {
TEST(VehicleAirbagFields, OriginalControlsAndNativeAnalyticConversionRemainDistinctFromSelectedElement) {
    auto data = Airbag();
    // A resident rear curve must never become an airbag material authority.
    data.rear_plastic_strain = {0, 1};
    data.rear_yield_stress_pa = {1, 2};
    ASSERT_NO_THROW(Resolve(data));
    const auto& p = data.parts[0];
    EXPECT_EQ(p.converter_isolid, 5u);
    EXPECT_EQ(p.selected_isolid, 18u);
    EXPECT_EQ(p.original_ihq, 4u);
    EXPECT_EQ(p.original_qh, .02);
    EXPECT_EQ(p.hourglass_source, 3u);
    EXPECT_EQ(p.curve_source, SIZE_MAX);
    EXPECT_EQ(output::Sha256(data.sources[3].block.raw_text), detail::AirbagHourglassHash);
    EXPECT_FALSE(vehicle::detail::SourceScalar(data.sources[1].cards[0].second, 1));
    const auto& m = p.law44.material;
    EXPECT_EQ(m.hardening, tl::material::law44::solid::HardeningKind::Analytic);
    EXPECT_EQ(output::Bits(m.density_kg_m3), output::Bits(1.95e-9 * 1e12));
    EXPECT_EQ(m.young_pa, 1e9);
    EXPECT_EQ(m.poisson_ratio, .3);
    EXPECT_EQ(m.analytic.a_pa, 20e6);
    EXPECT_EQ(output::Bits(m.analytic.b_pa), output::Bits((10. * 1000. / (1000. - 10.)) * 1e6));
    EXPECT_EQ(m.analytic.exponent, 1);
    EXPECT_EQ(m.rate_c_per_s, 8000);
    EXPECT_EQ(m.rate_p, 8);
    EXPECT_EQ(m.cutoff_hz, 10000);
    EXPECT_EQ(p.law44.curve.count, 0u);
    EXPECT_EQ(p.law44.curve.plastic_strain, nullptr);
    EXPECT_EQ(p.law44.curve.yield_stress_pa, nullptr);
    EXPECT_NE(p.law44.plastic_cap_strain, p.law44.failure_plastic_strain);
    auto moved = std::move(data);
    data = {};
    EXPECT_EQ(moved.parts[0].law44.curve.plastic_strain, nullptr);
    EXPECT_EQ(moved.parts[0].law44.material.analytic.a_pa, 20e6);
}
TEST(VehicleAirbagFields, NonOriginalBranchesAndLateControlsRejectThenRetry) {
    for (unsigned fault = 0; fault < 11; ++fault) {
        SCOPED_TRACE(fault);
        auto data = Airbag();
        if (fault == 0) data.policy = Policy::OriginalExtendedSolidsV4;
        if (fault == 1) data.sources[1].cards[0].second = Card({"2000945", "0"});
        if (fault == 2) data.sources[2].cards[1].second = Card({"8000", "8", "2100270", "", "0"});
        if (fault == 3) data.sources[2].cards[1].second = Card({"8000", "8", "0", "", "1"});
        if (fault == 4) data.sources[2].cards.back().second = Card({"1"});
        if (fault == 5) data.sources[2].cards[0].second = Card({"2000945", "1.95E-9", "1000", ".3", "20", "10", "1"});
        if (fault == 6) data.sources[3].cards[0].second = Card({"1", ".1"});
        if (fault == 7) data.sources[3].block.raw_text.back() = ' ';
        if (fault == 8) data.sources[3].cards[0].first = 146;
        if (fault == 9) data.sources.push_back(data.sources.back());
        if (fault == 10) data.parts[0].curve_source = 3;
        EXPECT_THROW(Resolve(data), std::runtime_error);
    }
    auto retry = Airbag();
    ASSERT_NO_THROW(Resolve(retry));
    EXPECT_FALSE(detail::SelectedAirbag(2000945, Policy::OriginalExtendedSolidsV4));
    const auto census = detail::ExpectedCensus(Policy::OriginalVehicleSupportsV5);
    EXPECT_EQ(census.parts, 17u);
    EXPECT_EQ(census.parents, 4980u);
    EXPECT_EQ(census.solid18_law44, 386u);
    EXPECT_EQ(Data{}.policy, Policy::OriginalAdhesive18RubberHephS6zV1);
}
TEST(VehicleAirbagFields, GlobalControlReceiptRequiresCompleteUniqueInventoryAndExactCapacity) {
    const std::string bytes = std::string(R"({"combine.key":{"blocks":[{"keyword":"*CONTROL_HOURGLASS",
      "file":"combine.key","first_line":142,"last_line":148,"data_records":1,
      "source_block_sha256":")") + detail::AirbagHourglassHash +
      R"("}],"keyword_counts":{"*CONTROL_HOURGLASS":1}}})";
    output::Document files;
    files.Parse(bytes.c_str());
    ASSERT_FALSE(files.HasParseError());
    Data data;
    data.policy = Policy::OriginalVehicleSupportsV5;
    Limits limits;
    limits.blocks = 1;
    limits.metadata_bytes = sizeof(detail::AirbagHourglassRaw) - 1;
    auto short_cap = limits;
    --short_cap.metadata_bytes;
    EXPECT_THROW(detail::ReadAirbagHourglass(files, data, short_cap), std::runtime_error);
    EXPECT_TRUE(data.sources.empty());
    ASSERT_NO_THROW(detail::ReadAirbagHourglass(files, data, limits));
    ASSERT_EQ(data.sources.size(), 1u);
    EXPECT_EQ(data.sources[0].cards.size(), 1u);
    EXPECT_EQ(data.sources[0].cards[0].first, 145u);
    EXPECT_THROW(detail::ReadAirbagHourglass(files, data, limits), std::runtime_error);
    files["combine.key"]["keyword_counts"]["*CONTROL_HOURGLASS"].SetUint(2);
    Data changed;
    changed.policy = data.policy;
    EXPECT_THROW(detail::ReadAirbagHourglass(files, changed, limits), std::runtime_error);
    EXPECT_TRUE(changed.sources.empty());
}
} // namespace crash::modelio::solid_source::test
