#include "../Internal.h"
#include "lib_utest/qualification/radioss_type25_nodal_seed/NativeOracle.h"
#include "lib_utest/qualification/radioss_type25_coefficients/NativeOracle.h"
#include "lib_utest/qualification/radioss_type25_gap_source/Fixture.h"
#include "lib_utest/qualification/radioss_type25_friction/NativeOracle.h"
#include "lib_utest/qualification/radioss_type25_friction/Assertions.h"
namespace crash::cases::vehicle_wall::native::wall_interface::test {
namespace d=detail;
namespace {
void Exact(double a,double b){EXPECT_EQ(output::Bits(a),output::Bits(b));}
n::UnitScale Units(double length,double mass) {n::UnitScale u;u.length_m=length;u.mass_kg=mass;u.time_s=1;return u;}
}
TEST(FiniteWallValues, RealDeclaredComponentMatchesWholeNativeShellGatherAndAsstifi) {
    for(const auto units:{Units(.001,1000),Units(.01,1),Units(1,1)}) {
        const auto actual=d::WallComponent(MaterialDeclaration{},units,71,{12,13,14,15});
        type25_seed_test::Case packet;packet.nodes=4;packet.shells.push_back(actual.shell);
        for(unsigned k=0;k<4;++k)packet.shells[0].nodes[k]=k;
        const auto native=type25_seed_test::Oracle(packet);
        ASSERT_EQ(native.finalized.size(),4u);
        for(unsigned k=0;k<4;++k)Exact(actual.global_coefficients[k],native.finalized[k].stiffness);
        n::NativeShellMainCoefficientInput main;
        main.face=n::MainFaceKind::OrdinaryExterior;main.layout=n::ShellLayout::Quad4;
        main.property_type=1;main.input_thickness_mode=0;main.scale=1;
        main.young=actual.shell.young;main.property_thickness=actual.shell.property_thickness;
        Exact(actual.primary_coefficient,type25_coefficient_test::Oracle(main).value);
        const std::uint32_t roster[]{3,0,2,1};double actual_secondary[4];
        d::ScaleSecondary(actual.global_coefficients.data(),4,roster,4,1.,actual_secondary);
        for(unsigned k=0;k<4;++k)Exact(actual_secondary[k],type25_coefficient_test::Oracle(
            n::NativeSecondaryCoefficientInput{1.,native.finalized[roster[k]].stiffness,1.}).value);
    }
}
TEST(FiniteWallValues, WallMaskDiffersFromSelfWhileBeamAndSpringGapsSurviveInNativeOrder) {
    gap_source_test::Fixture f;f.nodes=16;
    const auto wall=d::WallComponent(MaterialDeclaration{},Units(.001,1000),71,{12,13,14,15});
    f.shells=d::GapShells({f.shells.data(),f.shells.size()},2,wall.shell,1u<<20);
    f.springs[0].part_contact_thickness=8;
    const unsigned reverse[]{1,0,3,2};
    for(unsigned k=0;k<4;++k){f.mains[0].nodes[k]=12+k;f.mains[1].nodes[k]=12+reverse[k];}
    f.main_nodes={12,13,14,15};f.secondary.clear();for(unsigned i=0;i<16;++i)f.secondary.push_back(i);
    const auto in=f.Input();gap_source_test::Attempt actual(in);
    ASSERT_EQ(actual.Run(in).status,n::source_gaps::Status::Ok);
    gap_source_test::Same(actual.result,gap_source_test::Oracle(in));
    Exact(actual.result.secondary[0],0.); // Vehicle shell thickness was masked.
    Exact(actual.result.secondary[4],3.); // Native beam half-sqrt(area) survives.
    Exact(actual.result.secondary[5],4.); // Explicit spring thickness comes later.
    for(unsigned i=12;i<16;++i)Exact(actual.result.secondary[i],wall.half_gap);
    for(const auto& row:actual.result.mains)for(double x:row.corner)Exact(x,wall.half_gap);
    for(unsigned k=0;k<4;++k){f.mains[0].nodes[k]=k;f.mains[1].nodes[k]=reverse[k];}
    f.main_nodes={0,1,2,3};const auto self=gap_source_test::Oracle(f.Input());
    EXPECT_NE(output::Bits(actual.result.secondary[0]),output::Bits(self.secondary[0]));
}
TEST(FiniteWallValues, CompleteComponentLimitKeepsLeafOutputCapAndNativeGapResult) {
    gap_source_test::Fixture fixture;
    const auto input=fixture.Input();
    auto limits=d::WallGapLimits(Limits{});
    EXPECT_EQ(limits.output_bytes,n::source_gaps::Limits{}.output_bytes);
    n::source_gaps::Forecast forecast;
    ASSERT_EQ(n::source_gaps::Preflight(input,limits,forecast).status,n::source_gaps::Status::Ok);
    tl::util::HostArena scratch;
    ASSERT_TRUE(scratch.Initialize(forecast.scratch_bytes));
    std::vector<double> secondary(input.secondary_count),main_nodes(input.main_node_count);
    std::vector<n::source_gaps::MainGapFields> mains(input.main_count);
    const auto report=n::source_gaps::Build(input,limits,scratch.data(),scratch.bytes(),
        {secondary.data(),secondary.size(),main_nodes.data(),main_nodes.size(),mains.data(),mains.size()});
    ASSERT_EQ(report.status,n::source_gaps::Status::Ok);
    const auto native=gap_source_test::Oracle(input);
    for(std::size_t i=0;i<secondary.size();++i)Exact(secondary[i],native.secondary[i]);
    for(std::size_t i=0;i<main_nodes.size();++i)Exact(main_nodes[i],native.main_nodes[i]);
    for(std::size_t i=0;i<mains.size();++i)
        for(unsigned k=0;k<4;++k)Exact(mains[i].corner[k],native.mains[i].corner[k]);
    auto outer=Limits{};
    outer.own_bytes=forecast.output_bytes;
    EXPECT_EQ(n::source_gaps::Preflight(input,d::WallGapLimits(outer),forecast).status,n::source_gaps::Status::Ok);
    --outer.own_bytes;
    EXPECT_EQ(n::source_gaps::Preflight(input,d::WallGapLimits(outer),forecast).status,n::source_gaps::Status::ResourceLimit);
}
TEST(FiniteWallValues, ConstantWallFrictionUsesExistingNativeModelWithoutCopiedForceMath) {
    const auto controls=d::DeclaredControls(Units(.001,1000),.6);
    for(double speed:{0.,1.,100.,1000.})for(double pressure_scale:{.25,1.,4.}) {
        auto input=type25_friction_test::Basic();
        input.normal_config=controls.runtime.normal;input.controls=controls.runtime.friction;
        input.coefficients=controls.runtime.friction_coefficients;
        input.input.relative_velocity.x=speed;input.input.normal.stiffness*=pressure_scale;
        type25_friction_test::RefreshNormalVelocity(input);
        const auto expected=type25_friction_test::Oracle(input);n::NativeFrictionResult actual;
        ASSERT_EQ(n::EvaluateNativeFriction(input.normal_config,input.controls,input.coefficients,
            input.input,input.history,&actual),n::NormalStatus::Ok);
        type25_friction_test::Same(actual,expected);Exact(actual.coefficient,.6);
    }
    EXPECT_EQ(controls.runtime.response_mass,n::ResponseMassPolicy::AcceptedOwnerCoefficients);
    EXPECT_EQ(controls.runtime.physical_source,n::PhysicalSourceProfile::CompleteBoundLedger);
    EXPECT_EQ(controls.runtime.activity,n::ContactActivityPolicy::AllActivePrefix);
    for(double c:controls.runtime.friction_coefficients.c)Exact(c,0.);
}
TEST(FiniteWallValues, SourceRosterAndComponentRejectBadInputsWithoutMutatingTheirInputs) {
    gap_source_test::Fixture f;const auto before=f.shells;
    auto wall=d::WallComponent(MaterialDeclaration{},Units(.001,1000),71,{12,13,14,15});
    EXPECT_ANY_THROW(d::GapShells({f.shells.data(),f.shells.size()},99,wall.shell,1u<<20));
    EXPECT_ANY_THROW(d::GapShells({f.shells.data(),f.shells.size()},2,wall.shell,sizeof(wall.shell)));
    auto bad=MaterialDeclaration{};bad.thickness_m=-1;
    EXPECT_ANY_THROW(d::WallComponent(bad,Units(.001,1000),71,{12,13,14,15}));
    EXPECT_ANY_THROW(d::WallComponent(MaterialDeclaration{},Units(0,1000),71,{12,13,14,15}));
    ASSERT_EQ(f.shells.size(),before.size());
    for(std::size_t i=0;i<before.size();++i) {
        EXPECT_EQ(f.shells[i].source_element_id,before[i].source_element_id);
        Exact(f.shells[i].property_thickness,before[i].property_thickness);
    }
    const auto merged=d::GapShells({f.shells.data(),f.shells.size()},2,wall.shell,1u<<20);
    ASSERT_EQ(merged.size(),4u);EXPECT_EQ(merged[2].source_element_id,71u);
    EXPECT_EQ(merged[3].layout,n::ShellLayout::Triangle3);
}
}
