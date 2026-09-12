// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"

namespace law44_analytic_test {
TEST(SolidLaw44Analytic, OriginalConverterOrderAndDistinctFiniteReaderLimits) {
  const auto p = Airbag();
  Empty(p);
  EXPECT_TRUE(Bits(p.material.analytic.a_pa, 20. * 1e6));
  EXPECT_TRUE(Bits(p.material.analytic.b_pa, (10. * 1000. / (1000. - 10.)) * 1e6));
  EXPECT_EQ(p.material.analytic.exponent, 1);
  EXPECT_EQ(p.failure_plastic_strain, law::detail::NativeInfinity());
  EXPECT_LT(p.plastic_cap_strain, p.failure_plastic_strain);
  EXPECT_TRUE(Bits(p.plastic_cap_strain,
      std::pow((law::detail::NativeInfinity() - p.material.analytic.a_pa / 1e6) /
          (p.material.analytic.b_pa / 1e6), 1.)));
  const auto table = law44_solid_test::Parameters();
  EXPECT_EQ(table.material.hardening, law::HardeningKind::Tabulated);
  EXPECT_EQ(table.plastic_cap_strain, law::detail::NativeInfinity());
  EXPECT_EQ(table.failure_plastic_strain, law::detail::NativeInfinity());
}
TEST(SolidLaw44Analytic, CanonicalCurveCursorAndDerivedValuesRejectBeforePublication) {
  const auto p = Airbag();
  auto output = p;
  const auto before = output;
  const double x[2]{0, 1}, y[2]{20e6, 30e6};
  EXPECT_EQ(law::Prepare(p.material, {x, y, 2}, output), law::Status::InvalidCurve);
  EXPECT_EQ(std::memcmp(&before, &output, sizeof(output)), 0);
  EXPECT_EQ(law::Prepare(p.material, {x, nullptr, 0}, output), law::Status::InvalidCurve);
  auto invalid = p;
  invalid.plastic_cap_strain = std::nextafter(p.plastic_cap_strain, INFINITY);
  law::Result result; result.yield_stress_pa = 123;
  const auto seed = result;
  EXPECT_EQ(law::Update(invalid, {}, Motion(0), result), law::Status::InvalidParameters);
  Same(result, seed);
  law::History h; h.curve_cursor = 1;
  EXPECT_EQ(law::Update(p, h, Motion(0), result), law::Status::InvalidHistory);
  Same(result, seed);
  auto table = law44_solid_test::Parameters();
  table.material.analytic.a_pa = 20e6;
  EXPECT_FALSE(law::detail::ParametersValid(table));
  EXPECT_EQ(law::Update(p, {}, Motion(0), result), law::Status::Ok);
  EXPECT_EQ(result.history.curve_cursor, 0u);
}
TEST(SolidLaw44Analytic, ElasticPlasticUnloadReloadConstructorAndLateRetry) {
  const auto p = Airbag();
  auto in = Motion(0); in.dt_s = 0;
  law::Result result;
  ASSERT_EQ(law::Initialize(p, in, result), law::Status::Ok);
  EXPECT_EQ(result.history.plastic_strain, 0);
  EXPECT_EQ(result.plastic_increment, 0);
  EXPECT_EQ(law::Update(p, {}, in, result), law::Status::InvalidInput);
  law::History history;
  bool plastic = false, unloading = false;
  for (unsigned step = 0; step < 320; ++step) {
    ASSERT_EQ(law::Update(p, history, Motion(step), result), law::Status::Ok);
    plastic |= result.plastic_increment > 0;
    unloading |= step > 45 && result.plastic_increment == 0 && history.plastic_strain > 0;
    history = result.history;
  }
  EXPECT_TRUE(plastic); EXPECT_TRUE(unloading);
  const auto before = result;
  in = Motion(320); in.engineering_rate_per_s[5] = INFINITY;
  EXPECT_EQ(law::Update(p, history, in, result), law::Status::InvalidInput);
  Same(result, before);
  ASSERT_EQ(law::Update(p, history, Motion(320), result), law::Status::Ok);
  EXPECT_EQ(result.history.curve_cursor, 0u);
}
TEST(SolidLaw44Analytic, StressCapPrecedesSeparatePlasticFailureAndZeroStrainUsesE) {
  const auto p = Capped();
  EXPECT_NEAR(p.plastic_cap_strain, .4, 1e-15);
  EXPECT_EQ(p.failure_plastic_strain, .8);
  law::Input in; in.dt_s = 1e-6;
  law::History h; h.stress_pa[0] = 100e6;
  law::Result virgin, capped, failed;
  ASSERT_EQ(law::Update(p, h, in, virgin), law::Status::Ok);
  EXPECT_NEAR(virgin.tangent_factor, .5, 1e-15);
  h.plastic_strain = p.plastic_cap_strain;
  ASSERT_EQ(law::Update(p, h, in, capped), law::Status::Ok);
  EXPECT_EQ(capped.tangent_factor, 0);
  EXPECT_GT(std::abs(capped.history.stress_pa[0]), 0);
  h.plastic_strain = p.failure_plastic_strain;
  ASSERT_EQ(law::Update(p, h, in, failed), law::Status::Ok);
  for (double stress : failed.history.stress_pa) EXPECT_EQ(stress, 0);
}
TEST(SolidLaw44Analytic, PreparedLimitReceiptPreservesBitsAndRejectsChangedDeclarations) {
  const auto p = Capped(.6);
  ASSERT_TRUE(p.analytic_preparation.Matches(p.material, p.plastic_cap_strain));
  law::Parameters portable;
  ASSERT_TRUE(law::detail::Coefficients(p.material, portable, &p));
  EXPECT_TRUE(Bits(portable.plastic_cap_strain, p.plastic_cap_strain));
  EXPECT_TRUE(law::detail::ParametersValid(portable));
  const auto table = law44_solid_test::Parameters();
  EXPECT_FALSE(table.analytic_preparation.initialized());
  for (unsigned fault = 0; fault < 8; ++fault) {
    SCOPED_TRACE(fault);
    auto changed = p;
    if (fault == 0) changed.plastic_cap_strain = std::nextafter(p.plastic_cap_strain, INFINITY);
    if (fault == 1) changed.material.analytic.a_pa = std::nextafter(p.material.analytic.a_pa, INFINITY);
    if (fault == 2) changed.material.analytic.b_pa = std::nextafter(p.material.analytic.b_pa, INFINITY);
    if (fault == 3) changed.material.analytic.exponent = std::nextafter(.6, INFINITY);
    if (fault == 4) changed.material.analytic.maximum_stress_pa = 25e6;
    if (fault == 5) changed.material.analytic.maximum_plastic_strain = .7;
    if (fault == 6) changed.material.native_units = law::WorkingUnits::SI;
    if (fault == 7) changed.analytic_preparation = {};
    EXPECT_FALSE(law::detail::ParametersValid(changed));
    law::Parameters scratch;
    EXPECT_FALSE(law::detail::Coefficients(changed.material, scratch, &changed));
    law::Result result; result.yield_stress_pa = 731;
    const auto before = result;
    EXPECT_EQ(law::Update(changed, {}, Motion(0), result), law::Status::InvalidParameters);
    Same(result, before);
  }
  EXPECT_TRUE(law::detail::ParametersValid(p));
  static_assert(std::is_trivially_copyable<law::Parameters>::value, "Prepared values must remain upload-safe");
  RecordProperty("parameters_bytes", sizeof(law::Parameters));
  RecordProperty("analytic_preparation_bytes", sizeof(law::AnalyticPreparation));
}
} // namespace law44_analytic_test
