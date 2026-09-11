#include "../Internal.h"
#include "lib_src/elements/type25/Type25Property.h"
#include <gtest/gtest.h>

namespace crash::modelio::type25 {
namespace {
const std::vector<std::uint64_t> Ids{10, 20, 30, 40};
const std::vector<double> Positions{-0., 0, 0, .002, .001, 0, .02, -.003, 0, .04, 0, .001};
const std::vector<std::uint16_t> Roles{physical_scope::Shell, physical_scope::ProvisionalType25,
                                     physical_scope::Solid, physical_scope::BeamOrientation};
tl::fea::NodalNodeDomain Domain(bool extra = false, bool altered_zero = false) {
    std::vector<tl::fea::NodalDomainNode> nodes;
    for (int n = extra ? 3 : 2; n >= 0; --n)
        nodes.push_back({Ids[n], {Positions[3 * n], Positions[3 * n + 1], Positions[3 * n + 2]}});
    if (altered_zero) nodes.back().position.x = 0.;
    tl::fea::NodalNodeDomain domain;
    const auto report = domain.Initialize({71, nodes.data(), nodes.size()});
    output::Require(bool(report), "Tiny TYPE25 domain construction failed");
    return domain;
}
std::vector<physical_scope::Spotweld> Welds() {
    return {{901, {10, 20}, 2, 0, true}, {902, {20, 30}, 2, 2, true}};
}
}
TEST(VehicleType25Values,SourcePropertyKeepsNativeDefaultAndEndpointValues) {
    const auto p = assembly::ResolvedSpotweldProperty();
    EXPECT_EQ(output::Bits(p.mass_kg), output::Bits(.001));
    EXPECT_EQ(output::Bits(p.isotropic_inertia_kg_m2), output::Bits(1e-8));
    for (unsigned c = 0; c < 4; ++c) {
        EXPECT_EQ(output::Bits(p.stiffness[c]), output::Bits(c < 2 ? 1e8 : 1000.));
        EXPECT_EQ(p.damping[c], 0.);
        EXPECT_EQ(p.failure_weight[c], 1.);
        EXPECT_EQ(p.failure_exponent[c], 2.);
        EXPECT_EQ(output::Bits(p.failure_positive[c]), output::Bits(c < 2 ? 1e30 : 1e27));
        EXPECT_EQ(p.failure_negative[c], -p.failure_positive[c]);
    }
    native::MassCoefficients mass;
    ASSERT_EQ(native::EndpointCoefficients(p, mass), native::Status::Success);
    EXPECT_EQ(output::Bits(mass.mass_kg), output::Bits(.0005));
    EXPECT_EQ(output::Bits(mass.isotropic_inertia_kg_m2), output::Bits(5e-9));
}
TEST(VehicleType25Values,ReorderedAndSupersetDomainsMatchCanonicalBitsWithoutClosingOtherRoles) {
    const auto domain = Domain();
    ASSERT_NO_THROW(detail::CheckDomain(Ids, Positions, Roles, domain, 71, 3));
    ASSERT_NO_THROW(detail::CheckDomain(Ids, Positions, Roles, Domain(true), 71, 3));
    const auto packed = detail::Pack(Welds(), domain);
    ASSERT_EQ(packed.size(), 2);
    EXPECT_EQ(packed[0].global_node[0], 2);
    EXPECT_EQ(packed[1].global_node[1], 0);
    EXPECT_EQ(output::Bits(packed[0].position[0].x), output::Bits(-0.));
    EXPECT_EQ(packed[1].source_element_id, 902);
    native::PropertyInput property{801, assembly::ResolvedSpotweldProperty()};
    native::ModelInput input{71, domain.node_count(), &property, 1, packed.data(), packed.size(), assembly::SpotweldSourceUnits};
    native::Model model;
    ASSERT_TRUE(model.Initialize(input));
    EXPECT_EQ(model.connection_count(), 2);
    EXPECT_EQ(model.endpoint_mass()[3].global_node, 0);
}
TEST(VehicleType25Values,LateOptionalMissingEndpointAndSignedZeroRejectAtomically) {
    const auto domain = Domain();
    auto packed = detail::Pack(Welds(), domain);
    auto changed = Welds(); changed.back().default_only = false;
    EXPECT_THROW(packed = detail::Pack(changed, domain), std::runtime_error);
    EXPECT_EQ(packed.back().source_element_id, 902);
    changed = Welds(); changed.back().nodes[1] = 40;
    EXPECT_THROW(packed = detail::Pack(changed, domain), std::runtime_error);
    EXPECT_EQ(packed.back().source_node_id[1], 30);
    EXPECT_THROW(detail::CheckDomain(Ids, Positions, Roles, Domain(false, true), 71, 3), std::runtime_error);
    EXPECT_THROW(detail::CheckDomain(Ids, Positions, Roles, domain, 72, 3), std::runtime_error);
    EXPECT_NO_THROW(packed = detail::Pack(Welds(), domain));
}
} // namespace crash::modelio::type25
