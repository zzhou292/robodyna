// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25Normal.h"
#include <gtest/gtest.h>
namespace {
namespace n = tlfea::contact::radioss_type25;
TEST(Type25NormalHeader, UnknownEngineControlCannotSelectAProductionBranch) {
  n::ResolvedNormalConfig config;
  n::NativeNormalResult result; result.normal_force = 321.;
  EXPECT_EQ(n::EvaluateNativeNormal(config, {}, {}, &result), n::NormalStatus::UnsupportedProfile);
  EXPECT_EQ(result.normal_force, 321.);
  config.engine = {0, 0, 0};
  EXPECT_EQ(n::EvaluateNativeNormal(config, {}, {}, &result), n::NormalStatus::Ok);
  EXPECT_EQ(result.normal_force, 0.);
}
TEST(Type25NormalHeader, SiBoundaryRequiresExplicitValidUnits) {
  n::ResolvedNormalConfig config; config.engine = {0, 0, 0};
  n::SiNormalResult result; result.normal_force = 321.;
  EXPECT_EQ(n::EvaluateSiNormal(config, {}, {}, {}, &result), n::NormalStatus::InvalidInput);
  EXPECT_EQ(result.normal_force, 321.);
  EXPECT_EQ(n::EvaluateSiNormal(config, {0.001, 1000., 1.}, {}, {}, &result), n::NormalStatus::Ok);
}
}
