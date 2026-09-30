#pragma once
#include "NativePoint.h"
#include "YarisCurveFixture.h"
#include "lib_src/materials/TabulatedShellPlasticity.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>

namespace tl::qualification::law44::rate_test {
namespace material = tl::material;
using Parameters = material::TabulatedShellPlasticityParameters;
using History = material::TabulatedShellPlasticityHistory;
using PointResult = material::TabulatedShellPlasticityResult;
using Status = material::TabulatedShellPlasticityStatus;
constexpr double Young = 200.e9, Poisson = .3, Density = 7889.999999999999;
constexpr double SourceC = 8000., SourceP = 8., Cutoff = 10000.;
constexpr double Dt = 0x1p-24, Thickness = .001648;

inline Parameters Prepare(bool rate_enabled = true) {
    Parameters p;
    const material::TabulatedShellPlasticityCurve curve{
        YarisPlasticStrain.data(), YarisYieldStress.data(),
        static_cast<std::uint32_t>(YarisPlasticStrain.size())};
    material::TabulatedShellPlasticityRate rate;
    if (rate_enabled) rate = {true, SourceC, SourceP, Cutoff};
    EXPECT_EQ(material::PrepareTabulatedShellPlasticity(
        Young, Poisson, Density, curve, rate, p), Status::Ok);
    return p;
}
inline Input NativeInput(const Parameters& p, double dt = Dt) {
    Input in;
    in.young = Young;
    in.poisson = Poisson;
    in.density = Density;
    in.transverse_shear_modulus = (5. / 6.) * (Young / 2. / (1. + Poisson));
    in.plastic_strain = p.curve.plastic_strain;
    in.yield_stress = p.curve.yield_stress_pa;
    in.point_count = p.curve.count;
    if (p.rate.enabled) {
        in.rate.active = true;
        in.rate.coefficient_per_s = SourceC;
        in.rate.exponent = SourceP;
        in.rate.filter_coefficient = NativeFilterCoefficient(Cutoff, dt);
    }
    return in;
}
// The same precommitted point-law comparison budget as NativePointTest.
inline void Close(double actual, double expected, double absolute) {
    EXPECT_TRUE(std::isfinite(actual));
    EXPECT_TRUE(std::isfinite(expected));
    EXPECT_NEAR(actual, expected,
        absolute + 2.e-11 * std::max(std::abs(actual), std::abs(expected)));
}
inline void ComparePoint(const PointResult& actual, const Result& expected) {
    for (unsigned c = 0; c < 5; ++c) {
        SCOPED_TRACE(c);
        Close(actual.history.stress[c], expected.stress[c], 1.e-8);
    }
    Close(actual.history.plastic_strain, expected.plastic_strain, 2.e-14);
    Close(actual.history.filtered_rate_per_s, expected.filtered_rate_per_s, 2.e-11);
    Close(actual.plastic_increment, expected.plastic_increment, 2.e-14);
    Close(actual.tangent_ratio, expected.tangent_ratio, 2.e-14);
    Close(actual.elastic_thickness_strain + actual.plastic_thickness_strain,
          expected.total_thickness_strain, 2.e-14);
    Close(actual.yield_before_pa, expected.yield_before, 1.e-8);
    Close(actual.equivalent_stress_pa, expected.equivalent_stress, 1.e-8);
    Close(actual.plastic_work_density, expected.plastic_work_density, 1.e-8);
}
inline void Accept(Input& in, const Result& next) {
    in.accepted_stress = next.stress;
    in.accepted_plastic_strain = next.plastic_strain;
    in.rate.accepted_filtered_rate_per_s = next.filtered_rate_per_s;
}
inline bool Advance(const Parameters& p, Input& in, History& accepted,
                    PointResult& actual, Result& expected, double dt = Dt) {
    material::TabulatedShellPlasticityInput increment;
    std::copy(in.strain_increment.begin(), in.strain_increment.end(),
              increment.strain_increment);
    increment.transverse_shear_modulus = in.transverse_shear_modulus;
    increment.dt = dt;
    increment.total_strain_rate_per_s = in.rate.total_shell_rate_per_s;
    if (!Evaluate(in, expected)) {
        ADD_FAILURE() << "Native rate point rejected an admitted input";
        return false;
    }
    if (material::UpdateTabulatedShellPlasticity(p, accepted, increment, actual) != Status::Ok) {
        ADD_FAILURE() << "Production rate point rejected an admitted input";
        return false;
    }
    ComparePoint(actual, expected);
    Accept(in, expected);
    accepted = actual.history; // Independent persistent trajectories.
    return true;
}
}  // namespace tl::qualification::law44::rate_test
