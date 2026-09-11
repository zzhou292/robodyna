#include "TestSupport.h"
#include "native/NativeOracle.h"
#include <gtest/gtest.h>

namespace law90_test {
TEST(Law90Native, CompletePreparationAndDefaultBranches) {
  for (int mode = 0; mode < 4; ++mode) {
    auto input = OriginalInput();
    if (mode == 1) {
      input.contact_modulus_pa = 0;
      input.tension_cutoff_pa = 0;
      input.hysteresis = -1;
      input.curve_scale_dimension = 2;
    }
    if (mode == 2) {
      input.reference_density_kg_m3 = 700;
      input.card_young_pa = 200e9; // EMAX<=E0 branch
      input.curve_scale = .125;
      input.shape = 1;
      input.alpha = 1;
      input.smooth = 0;
    }
    if (mode == 3) {
      input.tension_cutoff_pa = -3;
      input.curve_scale = 3;
    }
    law::PreparedMaterial material;
    ASSERT_EQ(law::PrepareSI(input, OriginalCurve(), material), law::Status::Ok);
    double actual[33], expected[33];
    Pack(material, actual);
    const auto values = InputValues(input);
    const auto flags = InputFlags(input);
    const int count = 28;
    law90_native_prepare(values.data(), flags.data(), original_radiator::strain,
                         original_radiator::stress_pa, &count, expected);
    for (int i = 0; i < 33; ++i) {
      ASSERT_TRUE(SameBits(actual[i], expected[i])) << mode << ':' << i
          << " actual=" << actual[i] << " native=" << expected[i];
    }
  }
  // The native defaults can produce deceptively similar printed values while
  // selecting a different material branch. Bind the rejection to that receipt.
  auto input = OriginalInput();
  input.hysteresis = 0;
  input.tension_flag = 0;
  const auto values = InputValues(input);
  const auto flags = InputFlags(input);
  const int count = 28;
  double expected[33];
  law90_native_prepare(values.data(), flags.data(), original_radiator::strain,
                       original_radiator::stress_pa, &count, expected);
  EXPECT_EQ(expected[8], 1);  // resolved Hys1
  EXPECT_EQ(expected[16], 1); // native IFLAG1, distinct from selected IFLAG2
  EXPECT_EQ(expected[18], 1); // default TFLAG1
  law::PreparedMaterial rejected;
  EXPECT_EQ(law::PrepareSI(input, OriginalCurve(), rejected), law::Status::UnsupportedProfile);
}

TEST(Law90Native, OriginalWorkingUnitsAreComparedSeparately) {
  auto input = OriginalInput();
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(input, OriginalCurve(), material), law::Status::Ok);
  auto values = InputValues(input);
  values[0] = 7.7200e-10;
  values[2] = 21;
  values[4] = 20000;
  values[5] = 15;
  const auto flags = InputFlags(input);
  const int count = 28;
  double native[33], actual[33];
  law90_native_prepare(values.data(), flags.data(), original_radiator::strain,
                       original_radiator::stress_mpa, &count, native);
  Pack(material, actual);
  for (int i = 0; i < 33; ++i) {
    double factor = 1;
    if (i <= 1) factor = 1e12;
    if ((i >= 2 && i <= 3) || (i >= 5 && i <= 7) || (i >= 23 && i <= 31)) factor = 1e6;
    ASSERT_TRUE(Close(actual[i], native[i] * factor)) << i;
  }
  EXPECT_FALSE(Close(material.reader().card_young_pa, native[27] * 1e6));
  EXPECT_FALSE(Close(material.reader().contact_modulus_pa, native[27] * 1e6));
}

TEST(Law90Native, EveryKnotFromEveryCursorAndExtrapolation) {
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(OriginalInput(), OriginalCurve(), material), law::Status::Ok);
  const int count = 28;
  for (int cursor = 0; cursor < 27; ++cursor) {
    for (int knot = -1; knot <= 28; ++knot) {
      const double query = knot < 0 ? -.2 : knot == 28 ? .99 : original_radiator::strain[knot];
      double expected[2];
      int next = -1;
      law90_native_curve(original_radiator::strain, original_radiator::stress_pa,
                         &count, &query, &cursor, expected, &next);
      law::CurveResult actual;
      ASSERT_EQ(law::LookupCurve(material, query, cursor, actual), law::Status::Ok);
      ASSERT_EQ(actual.cursor, unsigned(next));
      ASSERT_TRUE(SameBits(actual.stress_pa, expected[0])) << cursor << ':' << knot;
      ASSERT_TRUE(SameBits(actual.slope_pa, expected[1])) << cursor << ':' << knot;
    }
  }
}

TEST(Law90Native, ReversalNegativeSlopeAndSignedZero) {
  const double x[] = {0, 1, 2, 3};
  const double cases[][4] = {{0, 2, -1, 4}, {-0.0, -0.0, 0.0, 0.0}};
  const double path[] = {-1, 0, 2.5, 2, 1, .5, 3, 4, 2, 0, -1};
  const int count = 4;
  for (const auto& y : cases) {
    int cursor = 0;
    for (double query : path) {
      double expected[2];
      int next = -1;
      law90_native_curve(x, y, &count, &query, &cursor, expected, &next);
      tl::material::detail::Vinter2Result actual;
      ASSERT_TRUE(tl::material::detail::Vinter2Value(x, y, count, query, cursor, actual));
      ASSERT_TRUE(SameBits(actual.value, expected[0]));
      ASSERT_TRUE(SameBits(actual.slope, expected[1]));
      ASSERT_EQ(actual.cursor, unsigned(next));
      cursor = next;
    }
  }
}
}  // namespace law90_test
