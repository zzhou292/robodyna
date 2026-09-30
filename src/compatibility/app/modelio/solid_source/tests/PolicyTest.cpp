#include "TestSupport.h"

namespace crash::modelio::solid_source::test {
namespace {
constexpr auto Extended = Policy::OriginalAdhesive18ExtendedRubberHephS6zV2;
}
TEST(VehicleSolidPolicy, ExtendedRubberRequiresOptInAndPreservesBothLiteralDensities) {
    EXPECT_EQ(Data{}.policy, Policy::OriginalAdhesive18RubberHephS6zV1);
    for (auto id : {2000017u, 2000393u, 2000509u, 2000521u}) {
        SCOPED_TRACE(id);
        auto legacy = Rubber(id);
        EXPECT_THROW(Resolve(legacy), std::runtime_error);
        const bool antiroll = id == 2000509 || id == 2000521;
        auto selected = Rubber(id, Extended, antiroll ? "1.9990E-9" : "1.9800E-9");
        ASSERT_NO_THROW(Resolve(selected));
        const auto& part = selected.parts[0];
        const double rho = antiroll ? 1.999e-9 : 1.98e-9;
        EXPECT_EQ(output::Bits(part.density_kg_m3), output::Bits(rho * 1e12));
        EXPECT_EQ(output::Bits(part.law42.density_kg_m3), output::Bits(rho * 1e12));
        EXPECT_EQ(part.law42.mu_pa, 24. * 1e6);
        EXPECT_EQ(part.law42.poisson_ratio, .463);
        EXPECT_EQ(part.hourglass_id, 2000017u);
        EXPECT_EQ(part.converter_isolid, 1u);
        EXPECT_EQ(part.material_law, MaterialLaw::Law42);
        EXPECT_FALSE(vehicle::detail::SourceScalar(selected.sources[1].cards[0].second, 1));
    }
    for (std::uint64_t id = 2000477; id <= 2000484; ++id) {
        auto legacy = Rubber(id);
        auto selected = Rubber(id, Extended);
        ASSERT_NO_THROW(Resolve(legacy));
        ASSERT_NO_THROW(Resolve(selected));
        EXPECT_EQ(output::Bits(legacy.parts[0].law42.bulk_pa),
                  output::Bits(selected.parts[0].law42.bulk_pa));
    }
}
TEST(VehicleSolidPolicy, NeighbourMetalFoamAndUnknownPoliciesRemainClosed) {
    for (auto id : {2000016u, 2000392u, 2000945u, 2000063u, 2000018u, 2000508u, 2000522u}) {
        SCOPED_TRACE(id);
        auto selected = Rubber(id, Extended);
        EXPECT_THROW(Resolve(selected), std::runtime_error);
        EXPECT_FALSE(detail::Selected(id, Extended));
    }
    const auto unknown = static_cast<Policy>(99);
    EXPECT_THROW(detail::ExpectedCensus(unknown), std::runtime_error);
    EXPECT_FALSE(detail::Selected(2000977, unknown));
    auto changed = Rubber(2000521, Extended, "1.9990E-9");
    changed.sources[3].cards[0].second = Card({"2000017", "4", ".02", "0", "1.5E-4", "6E-5"});
    EXPECT_THROW(Resolve(changed), std::runtime_error);
    auto retry = Rubber(2000521, Extended, "1.9990E-9");
    ASSERT_NO_THROW(Resolve(retry));
}
} // namespace crash::modelio::solid_source::test
