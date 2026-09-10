#include "NativeRateTestSupport.h"
#include "lib_src/elements/sections/ShellLayeredJ2.h"
#include <array>

namespace {
namespace native = tl::qualification::law44;
namespace section = tl::fea::sections;
using namespace native::rate_test;

TEST(Law44RateNative, ThreeLayerLoadingCarriesNativeFilterAndAcceptedThickness) {
    const auto p = Prepare();
    const auto rule = native::NativeSectionRule();
    std::array<native::Input, 3> independent;
    for (auto& point : independent) point = NativeInput(p);
    section::ShellLayeredJ2History accepted;
    double actual_thickness = Thickness, native_thickness = Thickness;
    long double actual_work = 0., native_work = 0.;
    double maximum_thickness_change = 0., maximum_bending = 0.;
    for (unsigned step = 0; step < 1664; ++step) {
        SCOPED_TRACE(step);
        const double direction = step < 768 ? 1. : step < 896 ? 0. : -.75;
        const std::array<double, 8> dx{
            direction * 100. * Dt, direction * -25. * Dt, direction * 200. * Dt,
            direction * 3. * Dt, direction * -4. * Dt,
            direction * 120000. * Dt, direction * -20000. * Dt, direction * 40000. * Dt};
        section::ShellLayeredJ2Input in;
        std::copy(dx.begin(), dx.end(), in.strain_curvature_increment);
        // Source ITHICK=1 freezes the accepted thickness for this interval's
        // force coefficients, point z positions, rate and layer-volume weights.
        in.reference_thickness = actual_thickness;
        in.reported_thickness = actual_thickness;
        in.transverse_shear_modulus = independent[0].transverse_shear_modulus;
        in.dt = Dt;
        const double expected_rate = native::NativeShellRate(dx, native_thickness, Dt);
        ASSERT_TRUE(std::isfinite(expected_rate));
        Close(section::LayeredJ2TotalStrainRate(in), expected_rate, 2.e-11);
        section::ShellLayeredJ2Result actual;
        ASSERT_EQ(section::UpdateShellLayeredJ2(p, accepted, in, actual), Status::Ok);

        std::array<double, 5> force{};
        std::array<double, 3> moment{};
        double expected_thickness = native_thickness;
        double mean_tangent = 0., minimum_tangent = 1., mean_yield = 0., last_yield = 0.;
        double mean_plastic = 0., maximum_plastic = 0., work_density = 0.;
        for (unsigned layer = 0; layer < 3; ++layer) {
            SCOPED_TRACE(layer);
            auto& point = independent[layer];
            const double z = rule.position[layer] * native_thickness;
            for (unsigned c = 0; c < 3; ++c) point.strain_increment[c] = dx[c] + z * dx[c + 5];
            for (unsigned c = 3; c < 5; ++c) point.strain_increment[c] = dx[c];
            point.rate.total_shell_rate_per_s = expected_rate;
            native::Result expected;
            ASSERT_TRUE(native::Evaluate(point, expected));
            for (unsigned c = 0; c < 5; ++c) {
                Close(actual.history.point[layer].stress[c], expected.stress[c], 1.e-8);
                force[c] += rule.membrane_weight[layer] * expected.stress[c];
            }
            for (unsigned c = 0; c < 3; ++c) moment[c] += rule.moment_weight[layer] * expected.stress[c];
            Close(actual.history.point[layer].plastic_strain, expected.plastic_strain, 2.e-14);
            Close(actual.history.point[layer].filtered_rate_per_s, expected.filtered_rate_per_s, 2.e-11);
            const double weight = rule.membrane_weight[layer];
            // The unchanged native point exposes combined DEZZ. Production
            // preserves MULAWC's two additions; this check allows only the
            // existing absolute thickness roundoff budget, not a new model.
            expected_thickness += expected.total_thickness_strain * (weight * native_thickness);
            mean_tangent += weight * expected.tangent_ratio;
            minimum_tangent = std::min(minimum_tangent, expected.tangent_ratio);
            mean_yield += weight * expected.yield_before;
            last_yield = expected.yield_before;
            mean_plastic += weight * expected.plastic_strain;
            maximum_plastic = std::max(maximum_plastic, expected.plastic_strain);
            work_density += weight * expected.plastic_work_density;
            Accept(point, expected);
        }
        for (unsigned c = 0; c < 5; ++c) {
            Close(actual.material_stress[c], force[c], 1.e-8);
            Close(actual.material_stress[c] * actual_thickness,
                  force[c] * native_thickness, 1.e-8);
        }
        for (unsigned c = 0; c < 3; ++c) {
            Close(actual.bending_stress[c], moment[c], 1.e-8);
            Close(actual.bending_stress[c] * actual_thickness * actual_thickness,
                  moment[c] * native_thickness * native_thickness, 1.e-8);
            maximum_bending = std::max(maximum_bending, std::abs(actual.bending_stress[c]));
        }
        Close(actual.reported_thickness, expected_thickness, 2.e-14);
        const auto& d = actual.diagnostics;
        Close(d.mean_tangent_ratio, mean_tangent, 2.e-14);
        Close(d.minimum_tangent_ratio, minimum_tangent, 2.e-14);
        Close(d.mean_yield_before_pa, mean_yield, 1.e-8);
        Close(d.last_point_yield_before_pa, last_yield, 1.e-8);
        Close(d.mean_plastic_strain, mean_plastic, 2.e-14);
        Close(d.maximum_plastic_strain, maximum_plastic, 2.e-14);
        Close(d.plastic_work_density_increment, work_density, 1.e-8);
        actual_work += d.plastic_work_density_increment * actual_thickness;
        native_work += work_density * native_thickness;
        accepted = actual.history;
        actual_thickness = actual.reported_thickness;
        native_thickness = expected_thickness;
        maximum_thickness_change = std::max(maximum_thickness_change, std::abs(actual_thickness - Thickness));
    }
    Close(static_cast<double>(actual_work), static_cast<double>(native_work), 1.e-8);
    EXPECT_GT(actual_work, 0.);
    EXPECT_GT(maximum_bending, 1.e6);
    EXPECT_GT(maximum_thickness_change, 1.e-7);
    EXPECT_GT(accepted.point[0].plastic_strain + accepted.point[2].plastic_strain, .001);
    EXPECT_GT(std::abs(accepted.point[0].plastic_strain - accepted.point[2].plastic_strain), 1.e-5);
    EXPECT_GT(actual_thickness, .9 * Thickness);
    EXPECT_LT(actual_thickness, 1.1 * Thickness);
}
}  // namespace
