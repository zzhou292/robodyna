// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25Coefficients.h"
#include <gtest/gtest.h>
#include <cmath>
TEST(Type25CoefficientConsumer, PublicProductionOnlyInterfaceNeedsNoNativeOracleOrOwner) {
  namespace n = tlfea::contact::radioss_type25;
  n::NativeScalarCoefficient native;
  EXPECT_EQ(n::EvaluateNativePairCoefficient({4, 0}, {2800, -2410, 0, 1e30}, &native), n::CoefficientStatus::Ok);
  EXPECT_EQ(native.value, 2410.);
  n::SiScalarCoefficient si;
  EXPECT_EQ(n::EvaluateSiSecondaryCoefficient({.001, 1000, 1}, {-0., 7., 0.}, &si), n::CoefficientStatus::Ok);
  EXPECT_TRUE(std::signbit(si.value));
}
