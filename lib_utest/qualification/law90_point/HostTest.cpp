#include "TestSupport.h"
#include <gtest/gtest.h>

namespace law90_point_test {
TEST(Law90PointHost, VirginTimeAndReferenceDensityAreExplicit) {
  auto input = OriginalInput();
  input.reference_density_kg_m3 = 1234;
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(input, OriginalCurve(), material), law::Status::Ok);
  law::PointResult point;
  ASSERT_EQ(law::InitializePointSI(material, {}, point), law::PointStatus::Ok);
  EXPECT_EQ(point.history.effective_modulus_pa, material.updated().young_pa);
  EXPECT_EQ(point.history.unloading_factor, 1);
  EXPECT_EQ(point.history.reserved5, 0);
  EXPECT_DOUBLE_EQ(point.sound_speed_m_s, std::sqrt(material.updated().young_pa/1234));
  EXPECT_NE(point.sound_speed_m_s, std::sqrt(material.updated().young_pa/input.density_kg_m3));
  for (double stress : point.cauchy_stress_pa) EXPECT_EQ(stress, 0);
  const auto before = Bytes(point);
  EXPECT_EQ(law::UpdatePointSI(material, point.history, {}, 0, point), law::PointStatus::InvalidInput);
  EXPECT_EQ(Bytes(point), before);
  EXPECT_EQ(law::UpdatePointSI(material, {}, {}, .1, point), law::PointStatus::InvalidHistory);
  EXPECT_EQ(Bytes(point), before);
}

TEST(Law90PointHost, TrueSlopeRatioUnloadingRatePathAndReservedHistory) {
  ToyCurve curve;
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(ToyInput(), curve.view(), material), law::Status::Ok);
  law::PointResult point;
  ASSERT_EQ(law::InitializePointSI(material, {}, point), law::PointStatus::Ok);
  point.history.reserved5 = 17;
  auto compressed = Stretch(.6, 1, 1);
  std::fill_n(compressed.engineering_rate_s_inverse, 6, 0.);
  compressed.engineering_rate_s_inverse[0] = 1;
  ASSERT_EQ(law::UpdatePointSI(material, point.history, compressed, .1, point), law::PointStatus::Ok);
  EXPECT_NEAR(point.tangent_factor, .25, 1e-14);
  EXPECT_NEAR(point.history.stress_norm_pa, 2.5e6, 1e-7);
  EXPECT_NEAR(point.history.path_energy_pa, 5e5, 1e-7);
  EXPECT_NEAR(point.history.effective_modulus_pa, 10.375e6, 1e-6);
  const auto maximum = point.history.maximum_path_energy_pa;
  const auto loading_rate = point.history.scalar_rate_s_inverse;
  auto unload = Stretch(.8, 1, 1);
  std::fill_n(unload.engineering_rate_s_inverse, 6, 0.);
  unload.engineering_rate_s_inverse[0] = 10;
  ASSERT_EQ(law::UpdatePointSI(material, point.history, unload, .2, point), law::PointStatus::Ok);
  EXPECT_EQ(point.history.scalar_rate_s_inverse, loading_rate);
  EXPECT_EQ(point.history.maximum_path_energy_pa, maximum);
  EXPECT_NEAR(point.history.path_energy_pa, 5e4, 1e-6);
  EXPECT_EQ(point.history.unloading_factor, 1);
  EXPECT_EQ(point.history.reserved5, 17);
  compressed.engineering_rate_s_inverse[0] = 2;
  ASSERT_EQ(law::UpdatePointSI(material, point.history, compressed, .3, point), law::PointStatus::Ok);
  EXPECT_GT(point.history.scalar_rate_s_inverse, loading_rate);
  EXPECT_EQ(point.history.reserved5, 17);
}

TEST(Law90PointHost, TensionCapFollowsPrecapHistoryAndPrecedesStretchDivision) {
  ToyCurve curve;
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(ToyInput(), curve.view(), material), law::Status::Ok);
  const auto input = Stretch(1.7, .8, .9);
  law::PointResult point;
  ASSERT_EQ(law::InitializePointSI(material, input, point), law::PointStatus::Ok);
  EXPECT_NEAR(point.cauchy_stress_pa[0], 1e6/.8/.9, 1e-6);
  EXPECT_GT(point.history.stress_norm_pa, 7e6);
  EXPECT_EQ(point.active, 1);
  EXPECT_EQ(point.maximum_viscosity_pa_s, 0);
}

TEST(Law90PointHost, OriginalRotatedLoadUnloadRetainsAllChannels) {
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(OriginalInput(), OriginalCurve(), material), law::Status::Ok);
  law::PointResult point;
  ASSERT_EQ(law::InitializePointSI(material, Path(0, true), point), law::PointStatus::Ok);
  double maximum_modulus = point.history.effective_modulus_pa;
  double maximum_shear = 0;
  bool backtracked = false;
  for (unsigned step = 1; step <= 320; ++step) {
    const auto before = point.history;
    ASSERT_EQ(law::UpdatePointSI(material, before, Path(step, true), step*1e-4, point), law::PointStatus::Ok) << step;
    maximum_modulus = std::max(maximum_modulus, point.history.effective_modulus_pa);
    for (unsigned k = 3; k < 6; ++k) maximum_shear = std::max(maximum_shear, std::abs(point.cauchy_stress_pa[k]));
    for (unsigned k = 0; k < 3; ++k) backtracked = backtracked || point.history.cursor[k] < before.cursor[k];
    EXPECT_EQ(point.history.unloading_factor, 1);
    EXPECT_EQ(point.history.reserved5, 0);
  }
  EXPECT_GT(maximum_modulus, material.updated().young_pa);
  EXPECT_GT(maximum_shear, 1e6);
  EXPECT_TRUE(backtracked);
}

TEST(Law90PointHost, LateCursorCurveStretchAndArithmeticRejectionRetry) {
  ToyCurve curve;
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(ToyInput(), curve.view(), material), law::Status::Ok);
  law::PointResult point;
  ASSERT_EQ(law::InitializePointSI(material, Stretch(.8, .9, 1), point), law::PointStatus::Ok);
  const auto saved = point;
  const auto bytes = Bytes(point);
  auto history = point.history;
  history.cursor[2] = 99;
  EXPECT_EQ(law::UpdatePointSI(material, history, {}, .1, point), law::PointStatus::InvalidCursor);
  EXPECT_EQ(Bytes(point), bytes);
  const auto value = curve.y[3];
  curve.y[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(law::UpdatePointSI(material, saved.history, {}, .1, point), law::PointStatus::InvalidCurve);
  EXPECT_EQ(Bytes(point), bytes);
  curve.y[3] = value;
  auto invalid = Stretch(.7, .9, 1);
  invalid.total_b_minus_i_engineering[0] = -3;
  EXPECT_EQ(law::UpdatePointSI(material, saved.history, invalid, .1, point), law::PointStatus::InvalidStretch);
  EXPECT_EQ(Bytes(point), bytes);
  // Native near-triple diagonal fallback alone would hide this indefinite B.
  invalid = {};
  const double tiny = std::nextafter(-1., 0.);
  invalid.total_b_minus_i_engineering[0] = tiny;
  invalid.total_b_minus_i_engineering[1] = tiny;
  invalid.total_b_minus_i_engineering[2] = tiny;
  invalid.total_b_minus_i_engineering[3] = 1e-15;
  EXPECT_EQ(law::UpdatePointSI(material, saved.history, invalid, .1, point), law::PointStatus::InvalidStretch);
  EXPECT_EQ(Bytes(point), bytes);
  invalid = Stretch(.7, .9, 1, .3, .4);
  invalid.engineering_rate_s_inverse[5] = 1e300;
  EXPECT_EQ(law::UpdatePointSI(material, saved.history, invalid, .1, point), law::PointStatus::NonfiniteResult);
  EXPECT_EQ(Bytes(point), bytes);
  law::PointResult control;
  const auto next = Stretch(.7, .9, 1);
  ASSERT_EQ(law::UpdatePointSI(material, saved.history, next, .1, control), law::PointStatus::Ok);
  ASSERT_EQ(law::UpdatePointSI(material, saved.history, next, .1, point), law::PointStatus::Ok);
  double a[21], b[21];
  Pack(point, a);
  Pack(control, b);
  for (unsigned k = 0; k < 21; ++k) EXPECT_TRUE(SameBits(a[k], b[k]));
}
} // namespace law90_point_test
