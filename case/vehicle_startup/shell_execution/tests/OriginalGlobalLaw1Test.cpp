#include "OriginalSupport.h"
namespace crash::cases::vehicle_startup::shell_execution::test {
TEST(VehicleShellExecutionOriginal, NativeSourceProfileKeepsAllParentsAndFailureIdentityWithZeroLaw1Points) {
    const auto refs=VehicleShellReferences::Prepare(modelio::vehicle::test::RigidResolution(),
        QephMetricProfile::AuthenticatedSourceLength,ReferenceLimits::ResolvedSections());
    const auto shells=VehicleShellBinding::Prepare(refs);
    const auto model=physical_model::VehiclePhysicalModel::Prepare(physical_model::test::Source(),shells);
    const auto forecast=VehicleShellExecution::Preflight(model,Law1ExecutionProfile::NativeA62OrdinaryNpt0,{});
    const auto native=VehicleShellExecution::Prepare(model,Law1ExecutionProfile::NativeA62OrdinaryNpt0,{});
    EXPECT_EQ(native.forecast().total_bytes,forecast.total_bytes);
    EXPECT_TRUE(native.law1_policy().requires_ordinary_explicit_defaults());
    Same(native.law1_policy().coefficient_working_length_m(),.001);
    std::size_t q=0,t=0,other=0;
    for(std::size_t i=0;i<native.catalog().parent_count();++i) {
        const auto* actual=native.catalog().parent(i);const auto* failure=native.failure().parent(i);
        const auto& raw=native.resolution().native_parent(i)->source;
        ASSERT_NE(actual,nullptr);ASSERT_NE(failure,nullptr);
        EXPECT_EQ(actual->source_parent_id,raw.source_parent_id);EXPECT_EQ(actual->source_part_id,raw.source_part_id);
        EXPECT_EQ(actual->material_id,raw.material_id);EXPECT_EQ(actual->section_id,raw.section_id);
        ASSERT_TRUE(detail::Same(*actual,failure->source));
        const auto p=native.resolution().parents()[i].part_index;
        if(native.resolution().material(p)->source.keyword=="*MAT_ELASTIC") {
            EXPECT_EQ(native.resolution().section(p)->through_thickness_points,3u);
            EXPECT_EQ(actual->execution.policy,fe::ShellParentExecutionPolicy::GlobalLaw1Npt0);
            EXPECT_EQ(native.execution().parent(i)->material_points,0u);
            actual->family==fe::ShellBindingFamily::Qeph?++q:++t;
        } else {++other;EXPECT_TRUE(detail::Same(*actual,raw));}
    }
    EXPECT_EQ(q,26225u);EXPECT_EQ(t,952u);EXPECT_EQ(other,349645u-27177u);
    EXPECT_EQ(native.execution().counts().material_points,1037877u-3*27177u);
    EXPECT_EQ(native.execution().counts().rigid_skin,5102u);
    EXPECT_TRUE(native.physical().coefficients()->Matches(model.coefficients()));
    EXPECT_THROW(VehicleShellExecution::Preflight(physical_model::test::Actual(),
        Law1ExecutionProfile::NativeA62OrdinaryNpt0,{}),std::exception);
}
TEST(VehicleShellExecutionOriginal, CompleteRigidOverlayChargesAllParentCopiesAtExactCap) {
    using Resolution = modelio::vehicle::VehicleSectionResolution;
    using Resource = modelio::vehicle::ResolutionLimits;
    const auto profile = modelio::vehicle::ResolutionProfile::OriginalRigidPartsV1;
    const auto& base = modelio::vehicle::test::MidlayerResolution();
    const auto declared = Resource::CompleteRigidOverlay();
    const auto bytes = Resolution::ForecastOriginalRigidParts(base, profile, declared);
    EXPECT_EQ(Resource{}.host_bytes, std::size_t{512} << 20);
    EXPECT_EQ(declared.host_bytes, std::size_t{640} << 20);
    EXPECT_EQ(modelio::vehicle::rigid_part::Limits{}.host_bytes, std::size_t{512} << 20);
    EXPECT_GT(bytes, Resource{}.host_bytes);
    EXPECT_LE(bytes, declared.host_bytes);
    EXPECT_THROW(Resolution::ForecastOriginalRigidParts(base, profile), std::exception);
    auto exact = declared;
    exact.host_bytes = bytes;
    const auto resolved = Resolution::ResolveOriginalRigidParts(base, modelio::vehicle::test::OriginalMember(), profile, exact);
    EXPECT_EQ(resolved.startup_budget_bytes(), bytes);
    --exact.host_bytes;
    EXPECT_THROW(Resolution::ResolveOriginalRigidParts(base, modelio::vehicle::test::OriginalMember(), profile, exact), std::exception);
    RecordProperty("complete_rigid_overlay_bytes", std::to_string(bytes));
    RecordProperty("failure_parent_bytes", std::to_string(sizeof(fe::ShellFailureParentInput)));
    RecordProperty("retained_parent_copies", "3");
    RecordProperty("original_parent_count", std::to_string(base.parents().size()));
}

}
