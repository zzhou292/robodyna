#include "ThinShellScreenInternal.h"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace crash::qualification {
namespace {
using namespace reference;
namespace detail=thin_shell_detail;
using Status=ElasticCouponStatus;

ThinShellSpectrumDiagnostic Modes(bool cluster=false) {
    ThinShellSpectrumDiagnostic result;
    result.available=true;
    // Four orthonormal world-z translation shapes. The first two have coherent
    // tip motion but orthogonal complete shapes; the other two twist the tip.
    constexpr std::size_t z[]{2,8,14,20};
    constexpr double signs[4][4]{{1,-1,1,1},{1,-1,-1,-1},{1,1,1,-1},{1,1,-1,1}};
    for (std::size_t row=0;row<4;++row) for (std::size_t column=0;column<4;++column)
        result.mass_modes(z[row],column)=.5*signs[row][column];
    std::size_t column=4;
    for (std::size_t row=0;row<kCouponFreeDofs;++row)
        if (row!=2&&row!=8&&row!=14&&row!=20) result.mass_modes(row,column++)=1;
    for (std::size_t mode=0;mode<kCouponFreeDofs;++mode) {
        const double frequency=cluster&&mode==1 ? 10.05 : 10*(mode+1);
        result.squared_frequency(mode)=frequency*frequency;
    }
    std::string diagnostic;
    EXPECT_TRUE(detail::ClassifyModes(result,ThinShellVector::Ones(),diagnostic))<<diagnostic;
    return result;
}

patch_audit::PatchNodalMass UnitMass() {
    patch_audit::PatchNodalMass result;
    result.mass.fill(1); result.total_isotropic_inertia.fill(1); return result;
}

ThinShellFixtureDiagnostic SyntheticDecision() {
    ThinShellFixtureDiagnostic result;
    result.setup_available=true;
    for (auto& raw:result.measurement) { raw.available=true; raw.status=Status::kSuccess; }
    for (std::size_t p=0;p<2;++p) {
        auto& policy=result.policies[p];
        policy.policy=p==0 ? ThinShellInertiaPolicy::PhysicalThickness : ThinShellInertiaPolicy::AreaCounterfactual;
        policy.mass=UnitMass(); policy.inverse_root_mass.setOnes();
        policy.reference_attempted=true; policy.reference_passed=true; policy.reference_status=Status::kSuccess;
        policy.directional_attempted=true; policy.directional_status=Status::kSuccess;
        for (auto& spectrum:policy.spectrum) {
            spectrum=Modes();
            for (auto& mode:spectrum.modes) { mode.kinetic_available=true; mode.kinetic_status=Status::kSuccess; }
        }
    }
    return result;
}

TEST(ThinShellScreen, FixedSixExplicitFixturesPreserveOriginalModelDefaults) {
    constexpr double edge[]{.01,.02},multiple[]{.5,1,2};
    for (std::size_t i=0;i<kThinShellFixtureCount;++i) {
        const auto p=ThinShellFixtureParameters(i);
        EXPECT_EQ(p.length,2*edge[i/3]); EXPECT_EQ(p.width,edge[i/3]);
        EXPECT_EQ(p.thickness,multiple[i%3]*.001648);
        EXPECT_EQ(p.young_modulus,200e9); EXPECT_EQ(p.density,7890); EXPECT_EQ(p.poisson_ratio,.3);
        EXPECT_EQ(p.shear_factor,5.0/6.0); EXPECT_EQ(p.torque_factor,.01);
        EXPECT_FALSE(IsDefaultElasticCouponParameters(p));
    }
    EXPECT_TRUE(IsDefaultElasticCouponParameters(ElasticCouponParameters{}));
    EXPECT_THROW(ThinShellFixtureParameters(kThinShellFixtureCount),std::out_of_range);
    EXPECT_THROW(ScreenThinShellFixture(kThinShellFixtureCount),std::out_of_range);
}

TEST(ThinShellScreen, RawNegativeSpectrumIsRetainedWithoutAdmittedIdentity) {
    ThinShellMatrix matrix=ThinShellMatrix::Identity(); matrix(0,0)=-2;
    ThinShellSpectrumDiagnostic output;
    std::string diagnostic;
    ASSERT_TRUE(detail::InspectSpectrum(matrix,ThinShellVector::Ones(),output,diagnostic))<<diagnostic;
    EXPECT_TRUE(output.available); EXPECT_FALSE(output.positive_spectrum);
    EXPECT_EQ(output.squared_frequency(0),-2); EXPECT_EQ(output.cluster_count,0);
    EXPECT_EQ(output.eigen_residual,0); EXPECT_FALSE(output.diagnostic.empty());
    const auto before=output;
    auto invalid=ThinShellVector::Ones().eval(); invalid(23)=0;
    EXPECT_FALSE(detail::InspectSpectrum(matrix,invalid,output,diagnostic));
    EXPECT_TRUE(output.squared_frequency==before.squared_frequency);
    EXPECT_TRUE(output.mass_modes==before.mass_modes);
    EXPECT_EQ(output.diagnostic,before.diagnostic);
}

TEST(ThinShellScreen, ClusterClassificationUsesBasisInvariantParticipationAndTipRows) {
    const auto original=Modes(true);
    ASSERT_EQ(original.clusters[0].count,2);
    ASSERT_TRUE(original.clusters[0].bending_eligible);
    auto rotated=original;
    const ThinShellVector a=rotated.mass_modes.col(0),b=rotated.mass_modes.col(1);
    const double c=std::sqrt(.5);
    rotated.mass_modes.col(0)=c*(a+b); rotated.mass_modes.col(1)=c*(b-a);
    std::string diagnostic;
    ASSERT_TRUE(detail::ClassifyModes(rotated,ThinShellVector::Ones(),diagnostic))<<diagnostic;
    EXPECT_TRUE(rotated.clusters[0].bending_eligible);
    EXPECT_NEAR(rotated.clusters[0].mean_out_of_plane_fraction,original.clusters[0].mean_out_of_plane_fraction,1e-14);
    EXPECT_NEAR(rotated.clusters[0].mean_normal_translation_fraction,original.clusters[0].mean_normal_translation_fraction,1e-14);
    EXPECT_NEAR(rotated.clusters[0].mean_tip_row_norm,original.clusters[0].mean_tip_row_norm,1e-14);
    ThinShellModeMatchDiagnostic match;
    detail::MatchCluster(original,ThinShellVector::Ones(),0,rotated,ThinShellVector::Ones(),UnitMass(),true,match);
    EXPECT_TRUE(match.matched)<<match.diagnostic; EXPECT_TRUE(match.frequency_refined);
    EXPECT_NEAR(match.comparison.minimum_squared_cosine,1,1e-14);
    EXPECT_EQ(match.comparison.clusters[0].mode_count,2);
}

TEST(ThinShellScreen, ExtraEligibleCandidateBranchesDoNotReplaceLowestBaselineIdentity) {
    auto valid=SyntheticDecision();
    detail::CompleteScreen(valid);
    ASSERT_TRUE(valid.screen_passed)<<valid.diagnostic;
    EXPECT_FALSE(valid.simulation_ready);
    EXPECT_EQ(valid.hybrid_match.reference_pool.mode_count,1);
    EXPECT_EQ(valid.hybrid_match.candidate_pool.mode_count,2);
    EXPECT_EQ(valid.hybrid_match.comparison.clusters[0].candidate_cluster,0);
    auto invalid=SyntheticDecision();
    for (auto& spectrum:invalid.policies[0].spectrum) {
        const ThinShellVector lowest=spectrum.mass_modes.col(0);
        spectrum.mass_modes.col(0)=spectrum.mass_modes.col(2); spectrum.mass_modes.col(2)=lowest;
        std::string diagnostic;
        ASSERT_TRUE(detail::ClassifyModes(spectrum,ThinShellVector::Ones(),diagnostic));
        EXPECT_FALSE(spectrum.clusters[0].bending_eligible);
        EXPECT_TRUE(spectrum.clusters[1].bending_eligible);
    }
    detail::CompleteScreen(invalid);
    EXPECT_FALSE(invalid.screen_passed); EXPECT_FALSE(invalid.hybrid_match.attempted);
    EXPECT_NE(invalid.diagnostic.find("Lowest physical"),std::string::npos);
}

TEST(ThinShellScreen, TrackedRefinementUsesCoarseDenominatorAndStrictHalfPercentBoundary) {
    const auto fine=Modes();
    for (const double coarse_frequency:{9.95,9.96}) {
        auto coarse=Modes(); coarse.squared_frequency(0)=coarse_frequency*coarse_frequency;
        std::string diagnostic;
        ASSERT_TRUE(detail::ClassifyModes(coarse,ThinShellVector::Ones(),diagnostic));
        ThinShellModeMatchDiagnostic match;
        detail::MatchCluster(fine,ThinShellVector::Ones(),0,coarse,ThinShellVector::Ones(),UnitMass(),true,match);
        ASSERT_TRUE(match.matched)<<match.diagnostic; // Geometric + hybrid 5% gate.
        EXPECT_EQ(match.frequency_refined,coarse_frequency==9.96);
        EXPECT_NEAR(match.maximum_fd_relative_frequency_change,
                    std::abs(10-coarse_frequency)/coarse_frequency,1e-15);
        EXPECT_EQ(match.status,coarse_frequency==9.96 ? ShellModeComparisonStatus::kSuccess :
                                                       ShellModeComparisonStatus::kFrequencyMismatch);
    }
    for (double frequency:{std::nextafter(10.05,0.),std::nextafter(10.05,20.)}) {
        auto changed=Modes(); changed.squared_frequency(0)=frequency*frequency;
        std::string diagnostic;
        ASSERT_TRUE(detail::ClassifyModes(changed,ThinShellVector::Ones(),diagnostic));
        ThinShellModeMatchDiagnostic match;
        detail::MatchCluster(changed,ThinShellVector::Ones(),0,fine,ThinShellVector::Ones(),UnitMass(),true,match);
        const double actual=changed.modes[0].angular_frequency;
        const bool exact_gate=200*std::fabs(static_cast<long double>(actual)-10)<=10;
        EXPECT_EQ(match.frequency_refined,exact_gate);
    }
}

TEST(ThinShellScreen, OwningAuditRejectionCannotBePromotedByGoodRawModeComparisons) {
    auto result=SyntheticDecision();
    result.measurement[0].stiffness(7,8)=123;
    result.policies[0].reference_passed=false;
    result.policies[0].reference_status=Status::kAuditRejected;
    result.policies[0].reference_diagnostic="unresolved derivative uncertainty";
    detail::CompleteScreen(result);
    EXPECT_FALSE(result.screen_passed); EXPECT_FALSE(result.simulation_ready);
    EXPECT_TRUE(result.hybrid_match.matched);
    EXPECT_EQ(result.measurement[0].stiffness(7,8),123);
    EXPECT_EQ(result.policies[0].reference_diagnostic,"unresolved derivative uncertainty");
    EXPECT_NE(result.diagnostic.find("Existing reference"),std::string::npos);
}

TEST(ThinShellScreen, NeutralOnlyCostEstimateSeparatesCounterfactualFromCurrentRotaryPolicy) {
    const auto p=ThinShellFixtureParameters(0);
    const ThinShellMatrix stiffness=100*ThinShellMatrix::Identity();
    ThinShellStepDiagnostic original,changed;
    detail::EstimateSteps(p,p.width,ThinShellInertiaPolicy::PhysicalThickness,stiffness,ThinShellVector::Ones(),original);
    detail::EstimateSteps(p,p.width,ThinShellInertiaPolicy::AreaCounterfactual,stiffness,ThinShellVector::Ones(),changed);
    ASSERT_TRUE(original.available); ASSERT_TRUE(changed.available);
    EXPECT_EQ(original.neutral_operator_norm,100);
    EXPECT_EQ(original.neutral_spectral_limit,.1/std::sqrt(200));
    EXPECT_EQ(original.neutral_spectral_steps_200ms,std::ceil(.2/original.neutral_spectral_limit));
    EXPECT_EQ(original.physical_rotary_limit,.1*p.thickness/std::sqrt(12*(p.young_modulus/(2*(1+p.poisson_ratio)))/p.density));
    EXPECT_TRUE(original.original_policy_estimate_available);
    EXPECT_FALSE(changed.original_policy_estimate_available);
    EXPECT_EQ(changed.original_policy_steps_200ms,0);
    EXPECT_EQ(changed.physical_rotary_limit,original.physical_rotary_limit);
}

TEST(ThinShellScreen, ActualThinFixtureRetainsMatricesKineticsAndBothFiniteAmplitudeLevels) {
    const auto result=ScreenThinShellFixture(0);
    ASSERT_TRUE(result.setup_available)<<result.diagnostic;
    EXPECT_FALSE(result.simulation_ready);
    for (const auto& measured:result.measurement) {
        ASSERT_TRUE(measured.available)<<measured.diagnostic;
        EXPECT_TRUE(measured.stiffness.allFinite()); EXPECT_GT(measured.stiffness.norm(),0);
    }
    EXPECT_EQ(result.difference_steps.translation_m,1e-6*.01);
    EXPECT_EQ(result.difference_steps.rotation_rad,1e-6);
    EXPECT_TRUE(result.screen_passed||!result.diagnostic.empty());
    for (const auto& policy:result.policies) {
        EXPECT_TRUE(policy.reference_attempted);
        EXPECT_EQ(policy.reference_passed,policy.reference_status==Status::kSuccess);
        EXPECT_TRUE(policy.reference_passed||!policy.reference_diagnostic.empty());
        for (const auto& spectrum:policy.spectrum) {
            ASSERT_TRUE(spectrum.available)<<spectrum.diagnostic;
            for (const auto& mode:spectrum.modes) {
                ASSERT_TRUE(mode.kinetic_available)<<mode.kinetic_diagnostic;
                const auto& energy=mode.kinetic;
                EXPECT_NEAR(energy.translation+energy.physical_rotation+energy.original_artificial_drilling+
                            energy.added_tangential+energy.added_drilling,.5,1e-13);
            }
        }
        for (const auto& mode:policy.spectrum[1].modes) {
            ASSERT_TRUE(mode.finite_amplitude_available)<<mode.finite_amplitude_diagnostic;
            EXPECT_EQ(mode.amplitude_m[0],1e-4*result.parameters.thickness);
            EXPECT_EQ(mode.amplitude_m[1],.5*mode.amplitude_m[0]);
            EXPECT_GT(mode.total_energy[0],0); EXPECT_GT(mode.total_energy[1],0);
            for (std::size_t level=0;level<2;++level)
                EXPECT_EQ(mode.shear_fraction[level],mode.shear_energy[level]/mode.total_energy[level]);
        }
        EXPECT_TRUE(policy.step.available)<<policy.step.diagnostic;
    }
}
} // namespace
} // namespace crash::qualification
