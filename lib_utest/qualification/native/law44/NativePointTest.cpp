#include "NativePoint.h"
#include "YarisCurveFixture.h"
#include "lib_src/materials/TabulatedShellPlasticity.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {
namespace native = tl::qualification::law44;
namespace material = tl::material;
using Curve = material::TabulatedShellPlasticityCurve;
using Parameters = material::TabulatedShellPlasticityParameters;
using History = material::TabulatedShellPlasticityHistory;
using Result = material::TabulatedShellPlasticityResult;
constexpr double Young = 200.e9;
constexpr double Poisson = .3;
constexpr double Density = 7889.999999999999;

Curve YarisCurve() {
    return {native::YarisPlasticStrain.data(), native::YarisYieldStress.data(),
            static_cast<std::uint32_t>(native::YarisPlasticStrain.size())};
}
Parameters Prepare(Curve curve = YarisCurve()) {
    Parameters parameters;
    EXPECT_EQ(material::PrepareTabulatedShellPlasticity(
        Young, Poisson, Density, curve, parameters),
        material::TabulatedShellPlasticityStatus::Ok);
    return parameters;
}
native::Input NativeInput(const Parameters& p) {
    native::Input input;
    input.young = Young;
    input.poisson = Poisson;
    input.density = Density;
    input.transverse_shear_modulus = (5. / 6.) * (Young / 2. / (1. + Poisson));
    input.plastic_strain = p.curve.plastic_strain;
    input.yield_stress = p.curve.yield_stress_pa;
    input.point_count = p.curve.count;
    return input;
}
// Fixed before execution. Stress scales give 0.006 Pa near 300 MPa, so a
// 1 Pa constitutive error is not hidden by the material's physical scale.
void Close(double actual, double expected, double absolute) {
    EXPECT_TRUE(std::isfinite(actual));
    EXPECT_TRUE(std::isfinite(expected));
    EXPECT_NEAR(actual, expected,
                absolute + 2.e-11 * std::max(std::abs(actual), std::abs(expected)));
}
void Compare(const Parameters& p, const Result& actual, const native::Result& expected) {
    for (int i = 0; i < 5; ++i) {
        SCOPED_TRACE(i);
        Close(actual.history.stress[i], expected.stress[i], 1.e-8);
    }
    Close(actual.history.plastic_strain, expected.plastic_strain, 2.e-14);
    Close(actual.plastic_increment, expected.plastic_increment, 2.e-14);
    Close(actual.tangent_ratio, expected.tangent_ratio, 2.e-14);
    Close(actual.elastic_thickness_strain + actual.plastic_thickness_strain,
          expected.total_thickness_strain, 2.e-14);
    Close(actual.yield_before_pa, expected.yield_before, 1.e-8);
    Close(p.sound_speed, expected.sound_speed, 1.e-10);
    Close(actual.equivalent_stress_pa, expected.equivalent_stress, 1.e-8);
    Close(actual.plastic_work_density, expected.plastic_work_density, 1.e-8);
}
bool Advance(const Parameters& p, native::Input& input, History& history,
             Result& actual, native::Result& expected) {
    material::TabulatedShellPlasticityInput update;
    std::copy(input.strain_increment.begin(), input.strain_increment.end(),
              update.strain_increment);
    update.transverse_shear_modulus = input.transverse_shear_modulus;
    if (!native::Evaluate(input, expected)) {
        ADD_FAILURE() << "Native point update rejected the admitted test input";
        return false;
    }
    if (material::UpdateTabulatedShellPlasticity(p, history, update, actual) !=
        material::TabulatedShellPlasticityStatus::Ok) {
        ADD_FAILURE() << "Production point update rejected the admitted test input";
        return false;
    }
    Compare(p, actual, expected);
    // Each implementation owns its persistent trajectory; neither accepted
    // history is replaced by the result of the other implementation.
    input.accepted_stress = expected.stress;
    input.accepted_plastic_strain = expected.plastic_strain;
    history = actual.history;
    return true;
}

TEST(Law44PointNative, ElasticLimitIncludesTransverseShearAndThickness) {
    const auto p = Prepare();
    auto input = NativeInput(p);
    constexpr double axial = 1.e-5;
    input.strain_increment = {axial, -Poisson * axial, 2.e-6, -3.e-6, 4.e-6};
    History history;
    Result actual;
    native::Result expected;
    ASSERT_TRUE(Advance(p, input, history, actual, expected));
    const double g = Young / 2. / (1. + Poisson);
    const double gs = (5. / 6.) * g;
    const double roundoff = 128 * std::numeric_limits<double>::epsilon() * Young * axial;
    EXPECT_NEAR(history.stress[0], Young * axial, roundoff);
    EXPECT_NEAR(history.stress[1], 0., roundoff);
    EXPECT_NEAR(history.stress[2], g * 2.e-6, roundoff);
    EXPECT_NEAR(history.stress[3], gs * -3.e-6, roundoff);
    EXPECT_NEAR(history.stress[4], gs * 4.e-6, roundoff);
    EXPECT_NEAR(actual.elastic_thickness_strain, -Poisson * axial, 1.e-19);
    EXPECT_EQ(actual.plastic_thickness_strain, 0.);
    EXPECT_EQ(history.plastic_strain, 0.);
    EXPECT_EQ(actual.plastic_work_density, 0.);
    EXPECT_EQ(actual.tangent_ratio, 1.);
}

TEST(Law44PointNative, OriginalCurvePersistentLoadingAndUnloadingLeavesPermanentStrain) {
    const auto p = Prepare();
    auto input = NativeInput(p);
    input.strain_increment[2] = 2.e-5;
    History history;
    Result actual;
    native::Result expected;
    double total_shear = 0.;
    long double plastic_work = 0.;
    for (int step = 0; step < 1200; ++step) {
        SCOPED_TRACE(step);
        const double previous = history.plastic_strain;
        ASSERT_TRUE(Advance(p, input, history, actual, expected));
        EXPECT_GE(history.plastic_strain, previous);
        EXPECT_GE(actual.plastic_work_density, 0.);
        total_shear += input.strain_increment[2];
        plastic_work += actual.plastic_work_density;
    }
    EXPECT_GT(history.plastic_strain, native::YarisPlasticStrain[1]);
    EXPECT_LT(history.plastic_strain, native::YarisPlasticStrain[3]);
    EXPECT_GT(plastic_work, 0.);
    const double accumulated = history.plastic_strain;
    input.strain_increment[2] = -history.stress[2] / p.shear_modulus;
    total_shear += input.strain_increment[2];
    ASSERT_TRUE(Advance(p, input, history, actual, expected));
    EXPECT_NEAR(history.stress[2], 0., 1.e-7);
    EXPECT_EQ(history.plastic_strain, accumulated);
    EXPECT_EQ(actual.plastic_increment, 0.);
    EXPECT_EQ(actual.plastic_work_density, 0.);
    EXPECT_GT(total_shear, .001);
}

TEST(Law44PointNative, PerfectPlasticShearHasIndependentYieldAndUnloadSolution) {
    constexpr std::array<double, 2> strain{0., .3};
    constexpr std::array<double, 2> stress{270.e6, 270.e6};
    const auto p = Prepare({strain.data(), stress.data(), 2});
    auto input = NativeInput(p);
    // An unloaded prestrained point uses the native tabulated-slope branch;
    // its separate virgin HS=E branch is exercised by the actual-curve path.
    input.accepted_plastic_strain = .01;
    input.strain_increment[2] = .006;
    History history;
    history.plastic_strain = .01;
    Result actual;
    native::Result expected;
    ASSERT_TRUE(Advance(p, input, history, actual, expected));
    const long double root_three = std::sqrt(3.L);
    const long double shear_yield = 270.e6L / root_three;
    const long double plastic_shear = .006L - shear_yield / p.shear_modulus;
    const long double plastic_increment = plastic_shear / root_three;
    EXPECT_NEAR(history.stress[2], shear_yield, 1.e-6);
    EXPECT_NEAR(actual.equivalent_stress_pa, 270.e6, 1.e-6);
    EXPECT_NEAR(actual.plastic_increment, plastic_increment, 2.e-17);
    EXPECT_NEAR(history.plastic_strain, .01L + plastic_increment, 2.e-17);
    EXPECT_EQ(actual.tangent_ratio, 0.);
    EXPECT_EQ(actual.elastic_thickness_strain, 0.);
    EXPECT_EQ(actual.plastic_thickness_strain, 0.);
    EXPECT_NEAR(actual.plastic_work_density, .5L * 270.e6L * plastic_increment, 1.e-8);
    const double accumulated = history.plastic_strain;
    input.strain_increment[2] = -history.stress[2] / p.shear_modulus;
    const double permanent_shear = .006 + input.strain_increment[2];
    ASSERT_TRUE(Advance(p, input, history, actual, expected));
    EXPECT_NEAR(history.stress[2], 0., 1.e-7);
    EXPECT_EQ(history.plastic_strain, accumulated);
    EXPECT_NEAR(permanent_shear, plastic_shear, 2.e-17);
    EXPECT_GT(permanent_shear, 0.);
}

TEST(Law44PointNative, OriginalCurveKnotsUseNativeLeftSegment) {
    const auto p = Prepare();
    for (std::size_t knot = 0; knot < native::YarisPlasticStrain.size(); ++knot) {
        SCOPED_TRACE(knot);
        auto input = NativeInput(p);
        input.accepted_plastic_strain = native::YarisPlasticStrain[knot];
        // The final endpoint gets a subyield increment so its accepted state
        // remains inside the deliberately non-extrapolating public domain.
        input.strain_increment = {1.e-6, -2.e-6,
            knot + 1 == native::YarisPlasticStrain.size() ? 1.e-6 : .005,
            3.e-6, -4.e-6};
        History history;
        history.plastic_strain = input.accepted_plastic_strain;
        Result actual;
        native::Result expected;
        ASSERT_TRUE(Advance(p, input, history, actual, expected));
        EXPECT_NEAR(actual.yield_before_pa, native::YarisYieldStress[knot], 1.e-6);
        if (knot && knot + 1 < native::YarisPlasticStrain.size()) {
            const double slope = (native::YarisYieldStress[knot] -
                native::YarisYieldStress[knot - 1]) / (native::YarisPlasticStrain[knot] -
                native::YarisPlasticStrain[knot - 1]);
            EXPECT_NEAR(actual.tangent_ratio, slope / (slope + Young), 1.e-16);
        }
    }
}

TEST(Law44PointNative, NativeThreePointTablesPreserveLiteralPromotionAndMoments) {
    const auto rule = native::NativeSectionRule();
    const std::array<double, 3> position{-.5, 0., .5};
    const std::array<double, 3> force{.25, .5, .25};
    const std::array<double, 3> moment{
        static_cast<double>(-.0833333f), 0., static_cast<double>(.0833333f)};
    double constant_force = 0., constant_moment = 0., linear_force = 0., linear_moment = 0.;
    for (int layer = 0; layer < 3; ++layer) {
        SCOPED_TRACE(layer);
        EXPECT_EQ(rule.position[layer], position[layer]);
        EXPECT_EQ(rule.membrane_weight[layer], force[layer]);
        EXPECT_EQ(rule.moment_weight[layer], moment[layer]);
        constant_force += rule.membrane_weight[layer];
        constant_moment += rule.moment_weight[layer];
        linear_force += rule.membrane_weight[layer] * rule.position[layer];
        linear_moment += rule.moment_weight[layer] * rule.position[layer];
    }
    EXPECT_EQ(constant_force, 1.);
    EXPECT_EQ(constant_moment, 0.);
    EXPECT_EQ(linear_force, 0.);
    EXPECT_EQ(linear_moment, static_cast<double>(.0833333f));
    EXPECT_NE(linear_moment, 1. / 12.);
    EXPECT_LT(std::abs(linear_moment / (1. / 12.) - 1.), 6.e-7);
    EXPECT_NE(rule.moment_weight[0], rule.membrane_weight[0] * rule.position[0]);
}
}  // namespace
