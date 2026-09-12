#include "Support.h"

namespace crash::cases::vehicle_startup::physical_model::extended_test {
TEST(VehicleExtendedPhysicalOriginal, ExplicitPolicyAndCompleteCapsPreserveImmutableModelsAndRetry) {
    EXPECT_THROW(domain_source::VehiclePhysicalDomain::Preflight(Scope(),
        domain_source::Policy::RetainedShellAssembliesV1), std::runtime_error);
    EXPECT_THROW(joint_source::VehicleType45Source::Preflight(Domain(),
        joint_source::Policy::OriginalDirectSdiType45V1), std::runtime_error);
    auto domain = Domain();
    domain_source::Limits domain_cap;
    domain_cap.host_bytes = domain.forecast().total_bytes - 1;
    EXPECT_THROW(domain = domain_source::VehiclePhysicalDomain::Prepare(Scope(), DomainPolicy, domain_cap), std::runtime_error);
    EXPECT_TRUE(domain.domain().SharesStorage(Domain().domain()));
    ++domain_cap.host_bytes;
    EXPECT_EQ(domain_source::VehiclePhysicalDomain::Preflight(Scope(), DomainPolicy, domain_cap).total_bytes, domain_cap.host_bytes);
    auto model = Model();
    const auto* before = model.coefficients().nodes().data();
    EXPECT_EQ(Limits{}.solid_bytes, 32u << 20);
    EXPECT_EQ(Limits::ExtendedSolids().solid_bytes, 64u << 20);
    EXPECT_THROW(model = VehiclePhysicalModel::Prepare(Domain(), test::Shells()), std::runtime_error);
    EXPECT_EQ(model.coefficients().nodes().data(), before);
    auto cap = Limits::ExtendedSolids(); cap.host_bytes = model.forecast().total_bytes - 1;
    EXPECT_THROW(model = VehiclePhysicalModel::Prepare(Domain(), test::Shells(), cap), std::runtime_error);
    EXPECT_EQ(model.coefficients().nodes().data(), before);
    ++cap.host_bytes;
    EXPECT_EQ(VehiclePhysicalModel::Preflight(Domain(), test::Shells(), cap).total_bytes, cap.host_bytes);
    cap = Limits::ExtendedSolids(); cap.solid_bytes = model.solids().startup_payload_bytes() - 1;
    EXPECT_THROW(model = VehiclePhysicalModel::Prepare(Domain(), test::Shells(), cap), std::runtime_error);
    EXPECT_EQ(model.coefficients().nodes().data(), before);
    ++cap.solid_bytes;
    cap.plain_bytes = model.plain_groups().startup_payload_bytes() - 1;
    EXPECT_THROW(model = VehiclePhysicalModel::Prepare(Domain(), test::Shells(), cap), std::runtime_error);
    EXPECT_EQ(model.coefficients().nodes().data(), before);
    ++cap.plain_bytes;
    model = VehiclePhysicalModel::Prepare(Domain(), test::Shells(), cap);
    EXPECT_EQ(model.solids().startup_payload_bytes(), cap.solid_bytes);
    EXPECT_TRUE(model.coefficients().Matches(Model().coefficients()));
    EXPECT_EQ(model.source_domain().policy(), DomainPolicy);
    const auto copy = model;
    EXPECT_TRUE(copy.solids().SharesStorage(model.solids()));
    EXPECT_EQ(copy.solids().materials90()[0].value.curve().stress_pa, model.solids().materials90()[0].value.curve().stress_pa);
    const fe::NodalCoefficientSourcesWithSolids sources{{model.coefficients().shells(), model.coefficients().type25(),
        model.coefficients().type13()}, model.coefficients().element_mass(), model.solids().contributions()};
    fe::NodalCoefficientLedger rejected;
    EXPECT_FALSE(rejected.InitializeWithSolids(sources, fe::CoefficientLimits::Vehicle()));
    EXPECT_FALSE(rejected.prepared());
    EXPECT_TRUE(model.coefficients().MatchesWithExtendedSolids(sources));
}
} // namespace crash::cases::vehicle_startup::physical_model::extended_test
