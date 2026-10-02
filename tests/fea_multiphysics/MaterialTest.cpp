#include <gtest/gtest.h>

#include "chrono/fea/multiphysics/ChMaterial3DStressStVenant.h"
#include "chrono/fea/multiphysics/ChMaterial3DThermalLinear.h"

namespace {
using namespace chrono;
using namespace chrono::fea;

TEST(MultiphysicsMaterial, FourierFluxAndHeatCapacity) {
    ChMaterial3DThermalLinear material;
    material.SetDensity(2700);
    material.SetSpecificHeatCapacity(900);
    material.SetThermalConductivity(200);
    const ChVector3d gradient(1, -2, 0.5);
    ChVector3d flux;
    material.ComputeHeatFlux(flux, gradient, 300, nullptr, nullptr);
    EXPECT_NEAR((flux - ChVector3d(-200, 400, -100)).Length(), 0, 1e-12);

    // User-supplied anisotropy must reach the constitutive law as well.
    material.GetConductivityMatrix()(1, 1) = 50;
    material.ComputeHeatFlux(flux, gradient, 300, nullptr, nullptr);
    EXPECT_NEAR((flux - ChVector3d(-200, 100, -100)).Length(), 0, 1e-12);
    ChMatrix33d tangent;
    material.ComputeTangentModulus(tangent, gradient, 300, nullptr, nullptr);
    EXPECT_DOUBLE_EQ(tangent(0, 0), 200);
    EXPECT_DOUBLE_EQ(tangent(1, 1), 50);
    EXPECT_DOUBLE_EQ(tangent(2, 2), 200);
    double heat_capacity = 0;
    material.ComputeDtMultiplier(heat_capacity, 300, nullptr, nullptr);
    EXPECT_DOUBLE_EQ(heat_capacity, 2430000);
    EXPECT_DOUBLE_EQ(material.Get_DtMultiplier(), heat_capacity);
}

TEST(MultiphysicsMaterial, StVenantUniaxialStressAndTangent) {
    // A uniaxial Green strain with free transverse contraction has Sxx=E*e.
    constexpr double young = 2e6;
    constexpr double poisson = 0.25;
    constexpr double strain = 0.001;
    ChMaterial3DStressStVenant material(young, poisson);
    ChMatrix33d right_cauchy_green(1);
    right_cauchy_green(0, 0) += 2 * strain;
    right_cauchy_green(1, 1) -= 2 * poisson * strain;
    right_cauchy_green(2, 2) -= 2 * poisson * strain;
    ChStressTensor<> stress;
    material.ComputeElasticStress(stress, right_cauchy_green);
    EXPECT_NEAR(stress.XX(), young * strain, 1e-8);
    EXPECT_NEAR(stress.YY(), 0, 1e-8);
    EXPECT_NEAR(stress.ZZ(), 0, 1e-8);
    EXPECT_NEAR(stress.GetXY(), 0, 1e-12);
    ChMatrixNM<double, 6, 6> tangent;
    material.ComputeElasticTangentModulus(tangent, right_cauchy_green);
    // Check the public tangent against a constitutive finite difference.
    constexpr double delta = 1e-7;
    ChMatrix33d plus = right_cauchy_green;
    ChMatrix33d minus = right_cauchy_green;
    plus(0, 0) += 2 * delta;
    minus(0, 0) -= 2 * delta;
    ChStressTensor<> stress_plus, stress_minus;
    material.ComputeElasticStress(stress_plus, plus);
    material.ComputeElasticStress(stress_minus, minus);
    EXPECT_NEAR((stress_plus.XX() - stress_minus.XX()) / (2 * delta), tangent(0, 0), 0.002);
    EXPECT_NEAR((stress_plus.YY() - stress_minus.YY()) / (2 * delta), tangent(1, 0), 0.002);
}
}  // namespace
