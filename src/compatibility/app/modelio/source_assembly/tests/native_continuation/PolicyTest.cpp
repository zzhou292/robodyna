#include "Fixture.h"
namespace crash::modelio::assembly::continuation_test {
TEST(NativeMaterialContinuation, SourceFactorySelectsNativeTableAdmissionWithoutChangingRateOrCurve) {
    plasticity_binding_test::Fixture fixture;
    for (const auto& old : fixture.materials) {
        const auto source = Source(old);
        const auto native = detail::NativeMaterial(source);
        EXPECT_EQ(native.continuation, Policy::NativeLastSegment);
        EXPECT_EQ(native.material_id, source.id); EXPECT_EQ(native.curve_id, source.curve_id);
        EXPECT_EQ(output::Bits(native.young_pa), output::Bits(source.young_pa));
        EXPECT_EQ(output::Bits(native.poisson_ratio), output::Bits(source.poisson_ratio));
        EXPECT_EQ(output::Bits(native.density_kg_m3), output::Bits(source.density_kg_m3));
        EXPECT_TRUE(native.rate.enabled);
        EXPECT_EQ(native.rate.policy, mat::ShellPlasticityRatePolicy::Legacy);
        EXPECT_EQ(output::Bits(native.rate.cowper_symonds_c_per_s), output::Bits(source.rate_c_per_s));
        EXPECT_EQ(output::Bits(native.rate.cowper_symonds_p), output::Bits(source.rate_p));
        EXPECT_EQ(native.rate.cutoff_hz, 10000.);
    }
}
TEST(NativeMaterialContinuation, AnalyticElasticAndStandaloneDefaultsRemainStrict) {
    plasticity_binding_test::Fixture fixture;
    auto source = Source(fixture.materials[0]);
    source.curve_id = 0;
    source.hardening = MaterialHardening::LinearLaw44;
    source.supplied_sigy_pa = 2700.; source.supplied_etan_pa = 1000.;
    for (auto rate : {detail::NativeLaw44Rate::SuppliedPositive, detail::NativeLaw44Rate::FilteredZeroC}) {
        const auto native = detail::NativeMaterial(source, rate);
        EXPECT_EQ(native.continuation, Policy::StrictDomain);
        EXPECT_EQ(native.hardening, mat::ShellPlasticityHardeningKind::LinearLaw44);
        EXPECT_EQ(native.linear.initial_yield_pa, 2700.); EXPECT_EQ(native.linear.tangent_modulus_pa, 1000.);
        EXPECT_EQ(native.rate.policy, rate == detail::NativeLaw44Rate::FilteredZeroC
            ? mat::ShellPlasticityRatePolicy::FilteredZeroC : mat::ShellPlasticityRatePolicy::Legacy);
    }
    source.law = MaterialLaw::LayeredLaw1;
    const auto elastic = detail::NativeMaterial(source);
    EXPECT_EQ(elastic.continuation, Policy::StrictDomain);
    EXPECT_EQ(elastic.law, fe::ShellSectionLaw::LayeredLaw1Nip3);
    EXPECT_FALSE(elastic.rate.enabled);
    EXPECT_EQ(fe::ShellPlasticityMaterialInput{}.continuation, Policy::StrictDomain);
    mat::TabulatedShellPlasticityParameters standalone;
    ASSERT_EQ(mat::PrepareTabulatedShellPlasticity(2e6,.3,1024,{fixture.x,fixture.yq,3},standalone),
        mat::TabulatedShellPlasticityStatus::Ok);
    EXPECT_EQ(standalone.continuation, Policy::StrictDomain);
}
TEST(NativeMaterialContinuation, UnsupportedHardeningAndMissingNativeTableAreStillRejected) {
    plasticity_binding_test::Fixture fixture;
    auto source = Source(fixture.materials[0]);
    source.hardening = static_cast<MaterialHardening>(97);
    EXPECT_THROW(detail::NativeMaterial(source), std::runtime_error);
    source = Source(fixture.materials[0]); source.curve_id = 0;
    EXPECT_THROW(detail::NativeMaterial(source), std::runtime_error);
    source = Source(fixture.materials[0]);
    EXPECT_THROW(detail::NativeMaterial(source, detail::NativeLaw44Rate::FilteredZeroC), std::runtime_error);
}
TEST(NativeMaterialContinuation, CatalogPreservesOrdinaryAndFailurePoliciesIndependently) {
    Fixture fixture;
    for (auto family : {fe::ShellBindingFamily::Qeph, fe::ShellBindingFamily::T3}) {
        const auto p = fixture.Parameters(family);
        EXPECT_EQ(p.continuation, Policy::NativeLastSegment);
        EXPECT_EQ(p.rate.policy, mat::ShellPlasticityRatePolicy::Legacy);
        ASSERT_EQ(p.curve.count, 3u);
        for (unsigned k = 0; k < 3; ++k) {
            EXPECT_EQ(output::Bits(p.curve.plastic_strain[k]), output::Bits(fixture.x[k]));
            const auto stress = family == fe::ShellBindingFamily::Qeph ? fixture.yq[k] : fixture.yt[k];
            EXPECT_EQ(output::Bits(p.curve.yield_stress_pa[k]), output::Bits(stress));
        }
        const auto* failure = fixture.failure.parent(family, 0);
        ASSERT_NE(failure, nullptr);
        EXPECT_EQ(failure->policy, family == fe::ShellBindingFamily::Qeph
            ? fe::ShellFailurePolicy::None : fe::ShellFailurePolicy::ConstantAllPoints);
    }
}
TEST(NativeMaterialContinuation, InDomainSectionHistoryWorkAndRateRemainBitIdentical) {
    Fixture fixture;
    for (auto family : {fe::ShellBindingFamily::Qeph, fe::ShellBindingFamily::T3}) {
        const auto native = fixture.Parameters(family);
        auto strict = native; strict.continuation = Policy::StrictDomain;
        fe::sections::ShellLayeredJ2History old_native, old_strict;
        for (unsigned step = 0; step < 24; ++step) {
            fe::sections::ShellLayeredJ2Input input;
            input.reference_thickness = input.reported_thickness = .01;
            input.transverse_shear_modulus = native.shear_modulus;
            input.dt = 1e-4;
            input.strain_curvature_increment[0] = step < 18 ? .002 : -.0001;
            input.strain_curvature_increment[5] = .001;
            fe::sections::ShellLayeredJ2Result a, b;
            ASSERT_EQ(fe::sections::UpdateShellLayeredJ2(native, old_native, input, a), fe::sections::PointStatus::Ok);
            ASSERT_EQ(fe::sections::UpdateShellLayeredJ2(strict, old_strict, input, b), fe::sections::PointStatus::Ok);
            Exact(a, b);
            EXPECT_LT(a.diagnostics.maximum_plastic_strain, native.curve.plastic_strain[native.curve.count - 1]);
            old_native = a.history; old_strict = b.history;
        }
    }
}
}
