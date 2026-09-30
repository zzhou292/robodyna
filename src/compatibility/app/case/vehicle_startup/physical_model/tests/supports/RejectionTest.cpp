#include "Support.h"
namespace crash::cases::vehicle_startup::physical_model::supports_test {
TEST(VehicleSupportsPhysicalOriginal, CompleteAuthorityAndLateBeamBudgetFailurePreservePriorModelAndRetry) {
    using S=modelio::physical_scope::PhysicalScope;
    const auto& in=Inputs();
    EXPECT_THROW(S::Preflight(in.masses,in.tied,in.beams,in.solids),std::runtime_error);
    EXPECT_THROW(domain_source::VehiclePhysicalDomain::Preflight(Scope(),
        domain_source::Policy::RetainedShellAssembliesExtendedSolidsV4),std::runtime_error);
    EXPECT_THROW(joint_source::VehicleType45Source::Preflight(Domain(),
        joint_source::Policy::OriginalDirectSdiType45ExtendedSolidsV4),std::runtime_error);
    modelio::physical_scope::Limits scope_cap;
    scope_cap.host_bytes=Scope().forecast().total_bytes-1;
    auto scope=Scope();
    EXPECT_THROW(scope=S::PrepareVehicleSupports(in.masses,in.tied,in.beams,in.solids,Beams(),scope_cap),std::runtime_error);
    EXPECT_EQ(&scope.data(),&Scope().data());
    ++scope_cap.host_bytes;
    EXPECT_EQ(S::PreflightVehicleSupports(in.masses,in.tied,in.beams,in.solids,Beams(),scope_cap).total_bytes,scope_cap.host_bytes);
    auto model=Model(); const auto* before=model.coefficients().nodes().data();
    auto cap=Limits::VehicleSupports(); cap.host_bytes=model.forecast().total_bytes-1;
    EXPECT_THROW(model=VehiclePhysicalModel::Prepare(Domain(),test::Shells(),cap),std::runtime_error);
    EXPECT_EQ(model.coefficients().nodes().data(),before);
    ++cap.host_bytes;
    EXPECT_EQ(VehiclePhysicalModel::Preflight(Domain(),test::Shells(),cap).total_bytes,cap.host_bytes);
    cap=Limits::VehicleSupports();
    cap.structural_beam_bytes=model.structural_beams()->startup_payload_bytes();
    cap.structural_contribution_bytes=model.coefficients().beam18()->startup_payload_bytes()-1;
    EXPECT_THROW(model=VehiclePhysicalModel::Prepare(Domain(),test::Shells(),cap),std::runtime_error);
    EXPECT_EQ(model.coefficients().nodes().data(),before);
    ++cap.structural_contribution_bytes;
    model=VehiclePhysicalModel::Prepare(Domain(),test::Shells(),cap);
    EXPECT_TRUE(model.coefficients().Matches(Model().coefficients()));
    EXPECT_EQ(model.structural_beams()->startup_payload_bytes(),cap.structural_beam_bytes);
    const fe::NodalCoefficientSourcesWithSolids physical{{model.coefficients().shells(),model.coefficients().type25(),
        model.coefficients().type13()},model.coefficients().element_mass(),model.solids().contributions()};
    EXPECT_TRUE(model.coefficients().MatchesWithBeams({physical,model.coefficients().beam18()}));
    EXPECT_FALSE(model.coefficients().MatchesWithExtendedSolids(physical));
}
} // namespace crash::cases::vehicle_startup::physical_model::supports_test
