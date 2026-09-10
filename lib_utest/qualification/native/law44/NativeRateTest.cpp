#include "NativeRateTestSupport.h"

namespace {
namespace native = tl::qualification::law44;
using namespace native::rate_test;

TEST(Law44RateNative, SourceFilterRiseAndDecayMatchClosedRecurrence) {
    const auto p = Prepare();
    auto in = NativeInput(p);
    History accepted;
    PointResult actual;
    native::Result expected;
    const double alpha = native::NativeFilterCoefficient(Cutoff, Dt);
    const double pi = std::atan2(0., -1.);
    EXPECT_EQ(alpha, std::min(1., (2. * pi * Cutoff) * Dt));
    ASSERT_GT(alpha, 0.);
    ASSERT_LT(alpha, 1.);
    const long double decay = 1.L - alpha;
    for (unsigned step = 1; step <= 500; ++step) {
        SCOPED_TRACE(step);
        in.rate.total_shell_rate_per_s = 1000.;
        ASSERT_TRUE(Advance(p, in, accepted, actual, expected));
        const long double exact = 1000.L * (1.L - std::pow(decay, step));
        EXPECT_NEAR(accepted.filtered_rate_per_s, exact, 2.e-10);
        const long double yield = 270.e6L * (1.L + std::pow(exact / SourceC, 1.L / SourceP));
        EXPECT_NEAR(actual.yield_before_pa, yield, 1.e-5);
        EXPECT_EQ(accepted.plastic_strain, 0.);
        EXPECT_EQ(actual.plastic_work_density, 0.);
    }
    EXPECT_GT(actual.yield_before_pa / 270.e6, 1.7);
    const long double initial = 1000.L * (1.L - std::pow(decay, 500));
    for (unsigned step = 1; step <= 500; ++step) {
        SCOPED_TRACE(step);
        in.rate.total_shell_rate_per_s = 0.;
        ASSERT_TRUE(Advance(p, in, accepted, actual, expected));
        EXPECT_NEAR(accepted.filtered_rate_per_s, initial * std::pow(decay, step), 2.e-10);
    }
    EXPECT_GT(accepted.filtered_rate_per_s, 0.);
    EXPECT_LT(accepted.filtered_rate_per_s, initial);
}

TEST(Law44RateNative, OriginalCurvePersistentLoadHoldAndReverseKeepsNativeRateHistory) {
    const auto p = Prepare();
    auto in = NativeInput(p);
    History accepted;
    PointResult actual;
    native::Result expected;
    long double plastic_work = 0.;
    double peak_shear = 0., previous_plastic = 0.;
    for (unsigned step = 0; step < 1664; ++step) {
        SCOPED_TRACE(step);
        const double shear_rate = step < 768 ? 400. : step < 896 ? 0. : -400.;
        in.strain_increment[2] = shear_rate * Dt;
        in.rate.total_shell_rate_per_s = std::abs(shear_rate) / std::sqrt(3.);
        ASSERT_TRUE(Advance(p, in, accepted, actual, expected));
        EXPECT_GE(accepted.plastic_strain, previous_plastic);
        EXPECT_GE(actual.plastic_work_density, 0.);
        EXPECT_GE(accepted.filtered_rate_per_s, 0.);
        previous_plastic = accepted.plastic_strain;
        plastic_work += actual.plastic_work_density;
        peak_shear = std::max(peak_shear, accepted.stress[2]);
    }
    EXPECT_GT(accepted.plastic_strain, .005);
    EXPECT_LT(accepted.plastic_strain, .05);
    EXPECT_GT(plastic_work, 0.);
    EXPECT_GT(peak_shear, 270.e6 / std::sqrt(3.));
    EXPECT_LT(accepted.stress[2], -270.e6 / std::sqrt(3.));
}

TEST(Law44RateNative, SaturatedFilterAndZeroRateRecoverTheirExactLimits) {
    const auto p = Prepare();
    constexpr double coarse_dt = .001;
    auto in = NativeInput(p, coarse_dt);
    EXPECT_EQ(in.rate.filter_coefficient, 1.);
    in.rate.total_shell_rate_per_s = 1000.;
    History accepted;
    PointResult actual;
    native::Result expected;
    ASSERT_TRUE(Advance(p, in, accepted, actual, expected, coarse_dt));
    EXPECT_EQ(accepted.filtered_rate_per_s, 1000.);
    const long double expected_yield = 270.e6L * (1.L + std::pow(1000.L / SourceC, 1.L / SourceP));
    EXPECT_NEAR(actual.yield_before_pa, expected_yield, 1.e-6);

    const auto off = Prepare(false);
    auto rate_zero = NativeInput(p);
    auto rate_off = NativeInput(off);
    rate_zero.strain_increment = rate_off.strain_increment = {1.e-5, -2.e-6, .004, 3.e-6, -4.e-6};
    History zero_history, off_history;
    PointResult zero_actual, off_actual;
    native::Result zero_expected, off_expected;
    ASSERT_TRUE(Advance(p, rate_zero, zero_history, zero_actual, zero_expected));
    ASSERT_TRUE(Advance(off, rate_off, off_history, off_actual, off_expected));
    for (unsigned c = 0; c < 5; ++c) EXPECT_EQ(zero_history.stress[c], off_history.stress[c]);
    EXPECT_EQ(zero_history.plastic_strain, off_history.plastic_strain);
    EXPECT_EQ(zero_actual.plastic_work_density, off_actual.plastic_work_density);
    EXPECT_EQ(zero_actual.tangent_ratio, off_actual.tangent_ratio);
    EXPECT_EQ(zero_history.filtered_rate_per_s, 0.);
}
}  // namespace
