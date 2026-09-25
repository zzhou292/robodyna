// SPDX-License-Identifier: AGPL-3.0-or-later
#include "PrecisionProbe.h"
#include <gtest/gtest.h>
#include <cmath>
TEST(Type25FrictionConsumer, PublicUsageProvidesProductionOnlyLinkAndUncontractedArithmetic) {
  volatile double values[]{1. + std::ldexp(1., -27), 1. - std::ldexp(1., -27), -1.};
  EXPECT_EQ(type25_precision_test::ProductSum(values[0], values[1], values[2]), 0.);
  EXPECT_NE(std::fma(values[0], values[1], values[2]), 0.);
  namespace n = tlfea::contact::radioss_type25;
  n::ResolvedNormalConfig normal; normal.engine = {0, 0, 0};
  n::FrictionControls controls{2, 10, 0, 1, 0, 0, 1};
  n::NativeFrictionResult result;
  EXPECT_EQ(n::EvaluateNativeFriction(normal, controls, {}, {}, {}, &result), n::NormalStatus::Ok);
  EXPECT_FALSE(result.contact_active);
}
