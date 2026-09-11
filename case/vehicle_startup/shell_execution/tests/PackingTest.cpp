#include "PackingFixture.h"

namespace crash::cases::vehicle_startup::shell_execution::test {
TEST(VehicleShellExecutionPacking, NativeModesAndOriginalOneThicknessSectionRemainExact) {
    detail::Packing out;
    out.Reserve(3,4,3);
    Part ordinary, elastic(101), midlayer(2000524);
    ordinary.native.continuation = tl::material::ShellPlasticityCurveContinuation::StrictDomain;
    elastic.native.law = fe::ShellSectionLaw::LayeredLaw1Nip3;
    elastic.native.hardening = tl::material::ShellPlasticityHardeningKind::Tabulated;
    elastic.native.linear = {};
    elastic.native.rate = {};
    midlayer.native.rate = {true,0,1,10000,tl::material::ShellPlasticityRatePolicy::FilteredZeroC};
    midlayer.formulation = fe::ShellSectionFormulation::OneThicknessPoint;
    midlayer.section.through_thickness_points = 1;
    ordinary.Add(out);
    elastic.Add(out);
    midlayer.Add(out);
    midlayer.Add(out); // Shared original T3/QBAT MID/SID, not another declaration.
    ASSERT_EQ(out.materials.size(),3);
    ASSERT_EQ(out.sections.size(),3);
    EXPECT_TRUE(detail::Same(out.materials[0],ordinary.native));
    EXPECT_TRUE(detail::Same(out.materials[1],elastic.native));
    EXPECT_TRUE(detail::Same(out.materials[2],midlayer.native));
    EXPECT_EQ(out.sections[2].section_id,2000524);
    EXPECT_EQ(out.sections[2].through_thickness_points,1);
    EXPECT_EQ(out.sections[2].formulation,fe::ShellSectionFormulation::OneThicknessPoint);
}
TEST(VehicleShellExecutionPacking, RigidRoleKeepsReferenceValuesWithoutMutatingOriginalCards) {
    Part rigid;
    rigid.Rigid();
    detail::Packing out;
    rigid.Add(out);
    ASSERT_EQ(out.materials.size(),1);
    auto expected = rigid.native;
    expected.law = fe::ShellSectionLaw::RigidSkin;
    EXPECT_TRUE(detail::Same(out.materials[0],expected));
    EXPECT_EQ(out.sections[0].through_thickness_points,0);
    EXPECT_EQ(out.sections[0].formulation,fe::ShellSectionFormulation::Nonconstitutive);
    EXPECT_EQ(rigid.native.law,fe::ShellSectionLaw::LayeredLaw1Nip3);
    EXPECT_EQ(rigid.section.through_thickness_points,3);
    rigid.role = source::SourceShellRole::ConstitutiveShell;
    EXPECT_THROW(rigid.Add(out),std::runtime_error);
    EXPECT_EQ(out.materials.size(),1);
}
TEST(VehicleShellExecutionPacking, LateSourceAndDuplicateConflictsPreserveExistingRowsThenRetry) {
    Part first;
    detail::Packing out;
    first.Add(out);
    for (unsigned fault = 0; fault < 6; ++fault) {
        SCOPED_TRACE(fault);
        auto bad = first;
        switch (fault) {
        case 0: ++bad.id.material_id; break;
        case 1: bad.section.thickness_m[3] *= 2; break;
        case 2: bad.native.rate.cowper_symonds_p = 7; break;
        case 3: bad.native.linear.tangent_modulus_pa = -0.; break;
        case 4: bad.section.thickness_m.fill(.002); break;
        case 5: bad.native.density_kg_m3 = 7850; break;
        }
        EXPECT_THROW(bad.Add(out),std::runtime_error);
        ASSERT_EQ(out.materials.size(),1);
        ASSERT_EQ(out.sections.size(),1);
        EXPECT_TRUE(detail::Same(out.materials[0],first.native));
    }
    first.Add(out);
    EXPECT_EQ(out.materials.size(),1);
    Part second(101);
    second.native.linear.tangent_modulus_pa = 0.;
    second.Add(out);
    second.native.linear.tangent_modulus_pa = -0.;
    EXPECT_THROW(second.Add(out),std::runtime_error);
    EXPECT_EQ(out.materials.size(),2);
}
TEST(VehicleShellExecutionPacking, CurvesUseOnlyReferencedOriginalValuesAndRejectSuppliedValueConflicts) {
    Part part;
    part.native.curve_id = 10;
    part.native.hardening = tl::material::ShellPlasticityHardeningKind::Tabulated;
    part.native.linear = {};
    std::vector<modelio::assembly::Curve> original{Curve(),Curve(11)}, failure{Curve()};
    detail::Packing out;
    part.Add(out);
    detail::PackCurves(out,original,failure);
    ASSERT_EQ(out.curves.size(),1);
    EXPECT_EQ(out.curves[0].curve.plastic_strain,original[0].plastic_strain.data());
    EXPECT_EQ(out.curves[0].curve.count,3);
    for (unsigned fault = 0; fault < 3; ++fault) {
        detail::Packing rejected;
        part.Add(rejected);
        auto changed = failure;
        if (fault == 0) changed[0].stress_pa.back() += 1;
        if (fault == 1) changed[0].plastic_strain[0] = -0.;
        if (fault == 2) changed[0].stress_pa.pop_back();
        EXPECT_THROW(detail::PackCurves(rejected,original,changed),std::runtime_error);
    }
    detail::Packing missing;
    part.native.curve_id = 99;
    part.Add(missing);
    EXPECT_THROW(detail::PackCurves(missing,original,failure),std::runtime_error);
}
} // namespace crash::cases::vehicle_startup::shell_execution::test
