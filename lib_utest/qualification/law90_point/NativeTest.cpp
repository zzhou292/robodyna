#include "NativeSupport.h"

namespace law90_point_test {
TEST(Law90PointNative, VirginActualPreparationDensityAndNonzeroConstruction) {
  auto input = OriginalInput();
  input.reference_density_kg_m3 = 1234;
  const auto prepared = NativePrepared(input, OriginalCurve());
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(input, OriginalCurve(), material), law::Status::Ok);
  for (const auto& kinematics : {law::PointKinematics{}, Stretch(.8, .95, 1.1, .37, -.8),
                                Stretch(1.7, .8, .9, .6, .4)}) {
    NativeState native;
    AdvanceNative(prepared.data(), OriginalCurve(), kinematics, 0, native);
    law::PointResult point;
    ASSERT_EQ(law::InitializePointSI(material, kinematics, point), law::PointStatus::Ok);
    ASSERT_TRUE(Compare(point, native));
    auto incorrect_density = prepared;
    incorrect_density[1] = incorrect_density[0];
    NativeState wrong;
    AdvanceNative(incorrect_density.data(), OriginalCurve(), kinematics, 0, wrong);
    EXPECT_FALSE(Compare(point, wrong));
  }
}

void OriginalTrajectory(bool rotated) {
  const auto prepared = NativePrepared(OriginalInput(), OriginalCurve());
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(OriginalInput(), OriginalCurve(), material), law::Status::Ok);
  NativeState native;
  law::PointResult point;
  auto kinematics = Path(0, rotated);
  AdvanceNative(prepared.data(), OriginalCurve(), kinematics, 0, native);
  ASSERT_EQ(law::InitializePointSI(material, kinematics, point), law::PointStatus::Ok);
  ASSERT_TRUE(Compare(point, native));
  native.history[4] = 17.25;
  point.history.reserved5 = 17.25;
  double largest_modulus = point.history.effective_modulus_pa;
  bool nonunit_et = false, backtracked = false;
  for (unsigned step = 1; step <= 320; ++step) {
    const auto old = point.history;
    kinematics = Path(step, rotated);
    AdvanceNative(prepared.data(), OriginalCurve(), kinematics, step*1e-4, native);
    ASSERT_EQ(law::UpdatePointSI(material, old, kinematics, step*1e-4, point), law::PointStatus::Ok) << step;
    ASSERT_TRUE(Compare(point, native)) << "step " << step;
    EXPECT_EQ(point.history.reserved5, 17.25);
    largest_modulus = std::max(largest_modulus, point.history.effective_modulus_pa);
    nonunit_et = nonunit_et || std::abs(point.tangent_factor-1) > .1;
    for (unsigned k = 0; k < 3; ++k) backtracked = backtracked || point.history.cursor[k] < old.cursor[k];
  }
  EXPECT_GT(largest_modulus, material.updated().young_pa);
  EXPECT_TRUE(nonunit_et);
  EXPECT_TRUE(backtracked);
  auto changed = native;
  changed.history[4] = 0;
  EXPECT_FALSE(Compare(point, changed));
  changed = native;
  changed.values[8] += .1; // ET is distinct from SSP and cannot be filled with1.
  EXPECT_FALSE(Compare(point, changed));
}
TEST(Law90PointNative, OriginalAxisLoadUnloadTensionAndAllHistory) { OriginalTrajectory(false); }
TEST(Law90PointNative, OriginalRotatedLoadUnloadTensionAndAllHistory) { OriginalTrajectory(true); }

TEST(Law90PointNative, KnotsPlateauTensionAndNativeCursorDirections) {
  ToyCurve curve;
  curve.y[2] = curve.y[1];
  const auto input = ToyInput();
  const auto prepared = NativePrepared(input, curve.view());
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(input, curve.view(), material), law::Status::Ok);
  NativeState native;
  law::PointResult point;
  AdvanceNative(prepared.data(), curve.view(), {}, 0, native);
  ASSERT_EQ(law::InitializePointSI(material, {}, point), law::PointStatus::Ok);
  unsigned step = 0;
  bool zero_et = false;
  for (double compression : {0., .1, .2, .4, .6, .8, .6, .4, .2, .1, 0., -.7}) {
    const auto kinematics = Stretch(1-compression, 1, 1);
    const double time = ++step*.01;
    AdvanceNative(prepared.data(), curve.view(), kinematics, time, native);
    ASSERT_EQ(law::UpdatePointSI(material, point.history, kinematics, time, point), law::PointStatus::Ok);
    ASSERT_TRUE(Compare(point, native)) << compression;
    zero_et = zero_et || point.tangent_factor == 0;
    EXPECT_GT(point.sound_speed_m_s, 0);
  }
  EXPECT_TRUE(zero_et);
  EXPECT_NEAR(point.cauchy_stress_pa[0], 1e6, 1e-6);
  EXPECT_GT(point.history.stress_norm_pa, point.cauchy_stress_pa[0]);
}

TEST(Law90PointNative, FailedAttemptRetriesItsIndependentNativeHistory) {
  const auto prepared = NativePrepared(OriginalInput(), OriginalCurve());
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(OriginalInput(), OriginalCurve(), material), law::Status::Ok);
  NativeState native;
  law::PointResult point;
  AdvanceNative(prepared.data(), OriginalCurve(), Path(0, true), 0, native);
  ASSERT_EQ(law::InitializePointSI(material, Path(0, true), point), law::PointStatus::Ok);
  for (unsigned step = 1; step <= 42; ++step) {
    ASSERT_EQ(law::UpdatePointSI(material, point.history, Path(step, true), step*.01, point), law::PointStatus::Ok);
    AdvanceNative(prepared.data(), OriginalCurve(), Path(step, true), step*.01, native);
  }
  const auto saved = point;
  auto invalid = Path(43, true);
  invalid.engineering_rate_s_inverse[5] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(law::UpdatePointSI(material, point.history, invalid, .43, point), law::PointStatus::InvalidInput);
  EXPECT_EQ(Bytes(point), Bytes(saved));
  ASSERT_EQ(law::UpdatePointSI(material, point.history, Path(43, true), .43, point), law::PointStatus::Ok);
  AdvanceNative(prepared.data(), OriginalCurve(), Path(43, true), .43, native);
  ASSERT_TRUE(Compare(point, native));
}
} // namespace law90_point_test
