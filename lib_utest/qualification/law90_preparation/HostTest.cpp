#include "TestSupport.h"
#include <gtest/gtest.h>
#include <limits>

namespace law90_test {
TEST(Law90Preparation, OriginalPreAndPostFieldsStayDistinct) {
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(OriginalInput(), OriginalCurve(), material), law::Status::Ok);
  EXPECT_EQ(material.curve().stress_pa, original_radiator::stress_pa);
  EXPECT_EQ(material.reader().card_young_pa, 21e6);
  EXPECT_EQ(material.reader().initial_shear_pa, 10.5e6);
  EXPECT_EQ(material.reader().contact_modulus_pa, 20e9);
  EXPECT_EQ(material.reader().tension_cutoff_pa, 15e6);
  EXPECT_NEAR(material.updated().young_pa, 34381236.52694611, 1e-7);
  EXPECT_NEAR(material.updated().maximum_modulus_pa, 3438123652.694611, 1e-5);
  EXPECT_GT(material.updated().shear_pa, material.reader().initial_shear_pa);
  EXPECT_GT(material.updated().maximum_curve_slope_pa, material.updated().maximum_modulus_pa);
  EXPECT_EQ(material.reader().smooth, 0);
  EXPECT_EQ(material.reader().rate_flag, 0);
  EXPECT_EQ(material.reader().loading_flag, 2);
  EXPECT_EQ(material.reader().history_count, 10);
  EXPECT_EQ(material.reader().cursor_count, 3);
}

TEST(Law90Preparation, NativeDefaultsPrecedeCurveUpdate) {
  auto input = OriginalInput();
  input.contact_modulus_pa = 0;
  input.tension_cutoff_pa = -1;
  input.hysteresis = -1;
  input.curve_scale_dimension = 2;
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(input, OriginalCurve(), material), law::Status::Ok);
  EXPECT_EQ(material.reader().contact_modulus_pa, input.card_young_pa);
  EXPECT_NE(material.reader().contact_modulus_pa, material.updated().young_pa);
  EXPECT_EQ(material.reader().tension_cutoff_pa, 1e20);
  EXPECT_EQ(material.reader().reference_density_kg_m3, input.density_kg_m3);
  EXPECT_EQ(material.reader().curve_scale, 2);
  EXPECT_EQ(material.reader().shape, 1);
  EXPECT_EQ(material.reader().alpha, 1);
  EXPECT_EQ(material.reader().hysteresis, 1);
  EXPECT_NEAR(material.updated().young_pa, 68762473.05389222, 2e-7);
  law::CurveResult result;
  ASSERT_EQ(law::LookupCurve(material, original_radiator::strain[1], 0, result), law::Status::Ok);
  law::PreparedMaterial unscaled;
  ASSERT_EQ(law::PrepareSI(OriginalInput(), OriginalCurve(), unscaled), law::Status::Ok);
  law::CurveResult reference;
  ASSERT_EQ(law::LookupCurve(unscaled, original_radiator::strain[1], 0, reference), law::Status::Ok);
  EXPECT_TRUE(SameBits(result.stress_pa, reference.stress_pa));
  EXPECT_NE(result.stress_pa, 2 * reference.stress_pa); // lookup remains unscaled
}

TEST(Law90Preparation, DirectionalKnotAndEndpointValues) {
  const double x[] = {0, 1, 2, 3};
  const double y[] = {0, 2, 8, 9};
  tl::material::detail::Vinter2Result result;
  ASSERT_TRUE(tl::material::detail::Vinter2Value(x, y, 4, 1, 0, result));
  EXPECT_EQ(result.cursor, 0u);
  EXPECT_EQ(result.slope, 2);
  ASSERT_TRUE(tl::material::detail::Vinter2Value(x, y, 4, 1, 2, result));
  EXPECT_EQ(result.cursor, 1u);
  EXPECT_EQ(result.slope, 6);
  ASSERT_TRUE(tl::material::detail::Vinter2Value(x, y, 4, -.5, 2, result));
  EXPECT_EQ(result.value, -1);
  ASSERT_TRUE(tl::material::detail::Vinter2Value(x, y, 4, 3.5, 0, result));
  EXPECT_EQ(result.value, 9.5);
  const double descending[] = {1, -1};
  ASSERT_TRUE(tl::material::detail::Vinter2Value(x, descending, 2, .5, 0, result));
  EXPECT_EQ(result.value, 0);
  EXPECT_EQ(result.slope, -2);
}

TEST(Law90Preparation, LateCurveAndCursorFailurePreserveOutputsThenRetry) {
  std::array<double, 28> x;
  std::copy_n(original_radiator::strain, 28, x.begin());
  law::CurveView curve{x.data(), original_radiator::stress_pa, 28};
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(OriginalInput(), curve, material), law::Status::Ok);
  const auto saved = Bytes(material);
  const double last = x.back();
  x.back() = x[26];
  EXPECT_EQ(law::PrepareSI(OriginalInput(), curve, material), law::Status::InvalidCurve);
  EXPECT_EQ(Bytes(material), saved);
  law::CurveResult result{13, 17, 19};
  const auto saved_result = Bytes(result);
  EXPECT_EQ(law::LookupCurve(material, .01, 0, result), law::Status::InvalidCurve);
  EXPECT_EQ(Bytes(result), saved_result);
  x.back() = last;
  EXPECT_EQ(law::LookupCurve(material, .01, 27, result), law::Status::InvalidCursor);
  EXPECT_EQ(Bytes(result), saved_result);
  ASSERT_EQ(law::PrepareSI(OriginalInput(), curve, material), law::Status::Ok);
  ASSERT_EQ(law::LookupCurve(material, .01, 0, result), law::Status::Ok);
  EXPECT_GT(result.stress_pa, 0);
}

TEST(Law90Preparation, DifferentNativeBranchesRemainClosed) {
  law::PreparedMaterial output;
  ASSERT_EQ(law::PrepareSI(OriginalInput(), OriginalCurve(), output), law::Status::Ok);
  const auto saved = Bytes(output);
  for (int branch = 0; branch < 6; ++branch) {
    auto input = OriginalInput();
    if (branch == 0) input.hysteresis = 0; // Hys becomes1 but IFLAG is1
    if (branch == 1) input.tension_flag = 0; // native TFLAG1
    if (branch == 2) input.failure_mode = 1;
    if (branch == 3) input.poisson_ratio = .3;
    if (branch == 4) input.curve_rate_s_inverse = 1;
    if (branch == 5) input.filter_cutoff_hz = 1000;
    EXPECT_EQ(law::PrepareSI(input, OriginalCurve(), output), law::Status::UnsupportedProfile);
    EXPECT_EQ(Bytes(output), saved);
  }
  auto invalid = OriginalInput();
  invalid.density_kg_m3 = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(law::PrepareSI(invalid, OriginalCurve(), output), law::Status::InvalidInput);
  EXPECT_EQ(Bytes(output), saved);
}
}  // namespace law90_test
