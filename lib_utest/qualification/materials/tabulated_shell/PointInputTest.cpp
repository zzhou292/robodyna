#include "lib_src/materials/TabulatedShellPlasticity.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace {
using namespace tl::material;
using Status = TabulatedShellPlasticityStatus;
constexpr double strain[]{0, .1, .3}, stress[]{270e6, 340e6, 362e6};
TabulatedShellPlasticityParameters Parameters() {
  TabulatedShellPlasticityParameters p;
  EXPECT_EQ(PrepareTabulatedShellPlasticity(200e9, .3, 7890, {strain, stress, 3}, p), Status::Ok);
  return p;
}
TabulatedShellPlasticityResult Sentinel() {
  TabulatedShellPlasticityResult result;
  result.history.stress[0] = 731.; result.history.plastic_strain = .123;
  result.plastic_increment = .017; result.plastic_work_density = 83.;
  return result;
}

TEST(TabulatedShellPlasticityInputs, PreparationRejectsInvalidDataWithoutPublication) {
  const auto valid = Parameters();
  auto output = valid;
  const double bad_strain[]{0, .1, .1}, bad_stress[]{270e6, 269e6, 362e6};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (auto curve : {TabulatedShellPlasticityCurve{},
                    TabulatedShellPlasticityCurve{bad_strain, stress, 3},
                    TabulatedShellPlasticityCurve{strain, bad_stress, 3},
                    TabulatedShellPlasticityCurve{strain, stress, 1025}}) {
    EXPECT_EQ(PrepareTabulatedShellPlasticity(200e9, .3, 7890, curve, output), Status::InvalidCurve);
    EXPECT_EQ(std::memcmp(&output, &valid, sizeof valid), 0);
  }
  for (double rho : {0., -1., nan, std::numeric_limits<double>::denorm_min()}) {
    EXPECT_EQ(PrepareTabulatedShellPlasticity(200e9, .3, rho, {strain, stress, 3}, output),
              Status::InvalidParameters);
    EXPECT_EQ(std::memcmp(&output, &valid, sizeof valid), 0);
  }
}

TEST(TabulatedShellPlasticityInputs, HistoryAndIncrementFailuresPreserveAllOutput) {
  const auto p = Parameters();
  const auto sentinel = Sentinel();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  TabulatedShellPlasticityHistory base;
  TabulatedShellPlasticityInput input;
  input.transverse_shear_modulus = p.shear_modulus*5./6.;
  for (int component = 0; component < 5; ++component) {
    auto bad_base = base;
    bad_base.stress[component] = nan;
    auto output = sentinel;
    EXPECT_EQ(UpdateTabulatedShellPlasticity(p, bad_base, input, output), Status::InvalidHistory);
    EXPECT_EQ(std::memcmp(&output, &sentinel, sizeof output), 0);
    auto bad_input = input;
    bad_input.strain_increment[component] = nan;
    EXPECT_EQ(UpdateTabulatedShellPlasticity(p, base, bad_input, output), Status::InvalidIncrement);
    EXPECT_EQ(std::memcmp(&output, &sentinel, sizeof output), 0);
  }
  auto output = sentinel;
  base.plastic_strain = -.001;
  EXPECT_EQ(UpdateTabulatedShellPlasticity(p, base, input, output), Status::InvalidHistory);
  EXPECT_EQ(std::memcmp(&output, &sentinel, sizeof output), 0);
}

TEST(TabulatedShellPlasticityInputs, LateCurveDomainFailureHasAnExactCleanRetry) {
  const double short_strain[]{0, .0005}, short_stress[]{270e6, 271e6};
  TabulatedShellPlasticityParameters p;
  ASSERT_EQ(PrepareTabulatedShellPlasticity(200e9, .3, 7890,
      {short_strain, short_stress, 2}, p), Status::Ok);
  TabulatedShellPlasticityHistory accepted;
  const auto saved = accepted;
  TabulatedShellPlasticityInput input;
  input.transverse_shear_modulus = p.shear_modulus*5./6.;
  input.strain_increment[0] = .2;
  const auto sentinel = Sentinel();
  auto rejected = sentinel;
  ASSERT_EQ(UpdateTabulatedShellPlasticity(p, accepted, input, rejected), Status::CurveDomainExceeded);
  EXPECT_EQ(std::memcmp(&rejected, &sentinel, sizeof rejected), 0);
  EXPECT_EQ(std::memcmp(&accepted, &saved, sizeof accepted), 0);
  input.strain_increment[0] = 1e-5;
  TabulatedShellPlasticityResult clean;
  ASSERT_EQ(UpdateTabulatedShellPlasticity(p, saved, input, clean), Status::Ok);
  ASSERT_EQ(UpdateTabulatedShellPlasticity(p, accepted, input, rejected), Status::Ok);
  EXPECT_EQ(std::memcmp(&rejected, &clean, sizeof clean), 0);
  EXPECT_EQ(std::memcmp(&accepted, &saved, sizeof accepted), 0);
}
} // namespace
