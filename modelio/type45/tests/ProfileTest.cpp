#include "../Internal.h"
#include "../SourcePolicy.h"
#include <gtest/gtest.h>

namespace crash::modelio::type45 {
namespace {
Data Rows(Policy policy) {
    Data value;
    for (std::size_t i = 0; i < 44; ++i) {
        Row row;
        row.source_id = 2200512 + i;
        row.property_index = i < 17 ? 0 : i < 39 ? 1 : 2;
        for (unsigned k = 0; k < 4; ++k) {
            row.nodes[k].domain_index = 10 * i + k;
            row.nodes[k].body.kind = BodyKind::PlainGroup;
        }
        if (detail::Boundary(policy, row.source_id)) row.nodes[1].domain_index = SIZE_MAX;
        value.rows.push_back(row);
    }
    return value;
}
Data Resolve(Data input, Policy policy) { detail::CheckOriginal(input, policy); return input; }
}
TEST(VehicleType45Profile, ExplicitV4RestoresOnlyTwoSphericalJointsAndPreservesFourRodBoundaries) {
    for (const auto policy : {Policy::OriginalDirectSdiType45V1, Policy::OriginalDirectSdiType45ExtendedSolidsV4}) {
        const auto value = Resolve(Rows(policy), policy);
        EXPECT_EQ(value.required, detail::Required(policy));
        EXPECT_EQ(value.boundaries, 44u - detail::Required(policy));
        for (const auto& row : value.rows)
            EXPECT_EQ(row.disposition == Disposition::OmittedAssemblyBoundary, detail::Boundary(policy, row.source_id));
    }
    EXPECT_EQ(detail::DomainPolicy(Policy::OriginalDirectSdiType45ExtendedSolidsV4),
              physical_domain::Policy::RetainedShellAssembliesExtendedSolidsV4);
    EXPECT_THROW(detail::Required(static_cast<Policy>(99)), std::runtime_error);
}
TEST(VehicleType45Profile, ProfileMismatchAndLateAxisFailureLeavePriorSourceDispositionAndRetry) {
    constexpr auto extended = Policy::OriginalDirectSdiType45ExtendedSolidsV4;
    constexpr auto legacy = Policy::OriginalDirectSdiType45V1;
    auto value = Resolve(Rows(extended), extended);
    EXPECT_THROW(value = Resolve(Rows(extended), legacy), std::runtime_error);
    EXPECT_THROW(value = Resolve(Rows(legacy), extended), std::runtime_error);
    auto bad = Rows(extended);
    bad.rows.back().nodes[2].domain_index = SIZE_MAX;
    EXPECT_THROW(value = Resolve(bad, extended), std::runtime_error);
    EXPECT_EQ(value.required, 40u);
    EXPECT_EQ(value.rows[2].disposition, Disposition::Required);
    EXPECT_NO_THROW(value = Resolve(Rows(extended), extended));
}
TEST(VehicleType45Profile, SupportsAdmitsAll44OnlyWithTheirActualEndpointAndAxisNodes) {
    constexpr auto supports = Policy::OriginalDirectSdiType45VehicleSupportsV5;
    EXPECT_EQ(detail::DomainPolicy(supports), physical_domain::Policy::RetainedShellAssembliesVehicleSupportsV5);
    auto accepted = Resolve(Rows(supports),supports);
    EXPECT_EQ(accepted.required,44u); EXPECT_EQ(accepted.boundaries,0u);
    auto bad = Rows(supports); bad.rows[17].nodes[1].domain_index = SIZE_MAX;
    EXPECT_THROW(accepted=Resolve(bad,supports),std::runtime_error);
    EXPECT_EQ(accepted.required,44u);
    bad = Rows(supports); bad.rows.back().nodes[2].domain_index = SIZE_MAX;
    EXPECT_THROW(accepted=Resolve(bad,supports),std::runtime_error);
    EXPECT_NO_THROW(accepted=Resolve(Rows(supports),supports));
    EXPECT_THROW(Resolve(Rows(Policy::OriginalDirectSdiType45ExtendedSolidsV4),supports),std::runtime_error);
}
TEST(VehicleType45Profile, NativeV6PreservesAll44WithoutAdmittingTheLegacyDomainPolicy) {
    const auto native=Policy::OriginalDirectSdiType45NativeSupportsV6;
    const auto legacy=Policy::OriginalDirectSdiType45VehicleSupportsV5;
    EXPECT_EQ(detail::DomainPolicy(native),physical_domain::Policy::RetainedShellAssembliesNativeSupportsV6);
    EXPECT_NE(detail::DomainPolicy(native),detail::DomainPolicy(legacy));
    EXPECT_TRUE(detail::HasVehicleSupports(native));EXPECT_TRUE(detail::HasVehicleSupports(legacy));
    const auto value=Resolve(Rows(native),native);EXPECT_EQ(value.required,44u);EXPECT_EQ(value.boundaries,0u);
    for(const auto& row:value.rows)EXPECT_FALSE(detail::Boundary(native,row.source_id));
    auto bad=Rows(native);bad.rows[0].nodes[1].domain_index=SIZE_MAX;
    EXPECT_THROW(Resolve(bad,native),std::runtime_error);
}
} // namespace crash::modelio::type45
