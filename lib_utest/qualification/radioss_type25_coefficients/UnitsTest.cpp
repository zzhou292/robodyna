// SPDX-License-Identifier: AGPL-3.0-or-later
#include "UnitFixture.h"
#include "Assertions.h"
namespace type25_coefficient_test {
TEST(Type25CoefficientUnits, AllPacketDimensionsMatchNativeAndIndependentSiScaling) {
  for (auto units : {n::UnitScale{1, 1, 1}, n::UnitScale{.5, 8, .25}, n::UnitScale{.001, 1000, 1}})
    for (const auto& packet : Cases()) {
      const auto actual = Evaluate(ToSi(packet, units));
      ASSERT_EQ(actual.status, n::CoefficientStatus::Ok);
      Same(actual, ScaledReference(packet, units));
    }
}
TEST(Type25CoefficientUnits, NativeVolumeFloorDoesNotBecomeAnSiVolumeFloor) {
  Packet packet; packet.kind = Kind::Nodal;
  packet.nodal = {5e-31, 2e-30, 0, 0, 11};
  const n::UnitScale units{.001, 1000, 1};
  const auto input = ToSi(packet, units);
  ASSERT_LT(input.nodal.volume, 1e-30);
  const auto actual = Evaluate(input); ASSERT_EQ(actual.status, n::CoefficientStatus::Ok);
  Same(actual, ScaledReference(packet, units));
  // Normalized pressure is around 2e6 Pa, not a spurious 2e-3 Pa floor result.
  EXPECT_GT(actual.first, 1e6);
}
TEST(Type25CoefficientUnits, CapturedPairConvertsNPerMillimetreToNPerMetre) {
  n::SiScalarCoefficient out;
  ASSERT_EQ(n::EvaluateSiPairCoefficient({.001, 1000, 1}, {4, 0},
      {2800000, -2410000, 0, 1e33}, &out), n::CoefficientStatus::Ok);
  EXPECT_EQ(out.value, 2410000.);
}
TEST(Type25CoefficientUnits, SelectorUnderflowAndInvalidOrOverflowingConversionsPreserveOutput) {
  n::SiScalarCoefficient out{73};
  Packet packet; packet.shell = Shell(); const auto ordinary = ToSi(packet, {1, 1, 1}).shell;
  EXPECT_EQ(n::EvaluateSiShellMainCoefficient({0, 1, 1}, ordinary, &out), n::CoefficientStatus::InvalidInput);
  EXPECT_EQ(out.value, 73.);
  EXPECT_EQ(n::EvaluateSiShellMainCoefficient({1e-200, 1e200, 1}, ordinary, &out), n::CoefficientStatus::InvalidInput);
  EXPECT_EQ(out.value, 73.); // Area/volume scale underflow is rejected before division.
  auto tiny = ordinary; tiny.element_thickness = std::numeric_limits<double>::denorm_min();
  EXPECT_EQ(n::EvaluateSiShellMainCoefficient({1e100, 1e-100, 1}, tiny, &out), n::CoefficientStatus::NonfiniteResult);
  EXPECT_EQ(out.value, 73.);
  EXPECT_EQ(n::EvaluateSiSecondaryCoefficient({1e-100, 1e200, 1},
      {std::numeric_limits<double>::denorm_min(), 1., 1.}, &out), n::CoefficientStatus::NonfiniteResult);
  EXPECT_EQ(out.value, 73.);
  auto large = ordinary; large.young = std::numeric_limits<double>::max();
  EXPECT_EQ(n::EvaluateSiShellMainCoefficient({1e100, 1e-100, 1}, large, &out), n::CoefficientStatus::NonfiniteResult);
  EXPECT_EQ(out.value, 73.);
}
TEST(Type25CoefficientUnits, SignedZeroSecondarySkipsTheUnusedGlobalFieldInBothUnitDomains) {
  n::SiScalarCoefficient out{73};
  ASSERT_EQ(n::EvaluateSiSecondaryCoefficient({.001, 1000, 1},
      {-0., std::numeric_limits<double>::quiet_NaN(), 1.}, &out), n::CoefficientStatus::Ok);
  Number(out.value, -0., true);
}
} // namespace type25_coefficient_test
