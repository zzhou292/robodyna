// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include "Assertions.h"
#include <limits>
namespace type25_normal_test {
TEST(Type25Normal, CompleteSelectedNativeBlockCorpusAndBothHistoryLayouts) {
  for (const auto& c : Cases()) {
    normal::NativeNormalResult result;
    ASSERT_EQ(normal::EvaluateNativeNormal(c.config, c.input, c.history, &result), normal::NormalStatus::Ok);
    Same(result, Oracle(c.config, c.input, c.history));
    Same(result, Oracle(c.config, c.input, c.history, true));
  }
}
TEST(Type25Normal, InitialMaximaStoredBeforeHalfAndDamping) {
  auto c = Basic(); c.config.damping_factor = 0;
  c.history.staged_penetration = 0.003; c.history.staged_stiffness = 600;
  normal::NativeNormalResult result;
  ASSERT_EQ(normal::EvaluateNativeNormal(c.config, c.input, c.history, &result), normal::NormalStatus::Ok);
  Same(result, Oracle(c.config, c.input, c.history));
  EXPECT_EQ(result.history.staged_penetration, 0.003);
  EXPECT_EQ(result.history.staged_stiffness, 600);
  EXPECT_EQ(result.force_stiffness, 200);
  EXPECT_DOUBLE_EQ(result.normal_force, -0.4);
  EXPECT_DOUBLE_EQ(result.elastic_energy, 0.0004);
}
TEST(Type25Normal, ZeroPenetrationSkipsStateUpdatesAndClearsWeights) {
  auto c = Basic(); c.input.penetration = -0.; c.history = {1, 2, 3, 4, -5};
  for (const auto engine : {normal::EngineControls{0, 0, 0}, normal::EngineControls{1, 2, 1}}) {
    c.config.engine = engine;
    normal::NativeNormalResult result;
    ASSERT_EQ(normal::EvaluateNativeNormal(c.config, c.input, c.history, &result), normal::NormalStatus::Ok);
    Same(result, Oracle(c.config, c.input, c.history));
    EXPECT_EQ(result.history.damping_half_force, -5);
    EXPECT_EQ(result.history.staged_penetration, 3);
    EXPECT_EQ(result.stability_stiffness, c.input.stiffness);
    for (double w : result.weights) EXPECT_EQ(w, 0);
    EXPECT_FALSE(result.terms_valid);
  }
}
TEST(Type25Normal, EppThresholdAndUnclampedSignedReboundFollowNative) {
  auto c = Basic(); c.input.time = 1.; c.input.dt = 0.;
  c.history = {0.001, 300., 0, 0, 0};
  const double threshold = c.history.previous_penetration + normal::native_constant::epp;
  for (double p : {std::nextafter(threshold, 0.), threshold,
                   std::nextafter(threshold, std::numeric_limits<double>::infinity())}) {
    c.input.penetration = p;
    normal::NativeNormalResult result;
    ASSERT_EQ(normal::EvaluateNativeNormal(c.config, c.input, c.history, &result), normal::NormalStatus::Ok);
    Same(result, Oracle(c.config, c.input, c.history));
  }
  c = Basic(); c.input.normal_velocity = 1e6;
  normal::NativeNormalResult rebound;
  ASSERT_EQ(normal::EvaluateNativeNormal(c.config, c.input, c.history, &rebound), normal::NormalStatus::Ok);
  EXPECT_GT(rebound.normal_force, 0); // Native signed damping is not our unilateral clamp.
  Same(rebound, Oracle(c.config, c.input, c.history));
}
TEST(Type25Normal, HistorySequenceUnloadRecontactAndAliasedOutputHistory) {
  auto c = Basic(); normal::NativeNormalHistory native_history;
  normal::NativeNormalResult result;
  for (double p : {0.002, 0.0021, 0.001, 0., 0.003, 0.0031}) {
    c.input.penetration = p;
    const auto expected = Oracle(c.config, c.input, native_history);
    ASSERT_EQ(normal::EvaluateNativeNormal(c.config, c.input, c.history, &result), normal::NormalStatus::Ok);
    Same(result, expected);
    auto alias = result;
    ASSERT_EQ(normal::EvaluateNativeNormal(c.config, c.input, alias.history, &alias), normal::NormalStatus::Ok);
    Same(alias, Oracle(c.config, c.input, result.history));
    c.history = result.history; native_history = expected.history;
    // Test harness models the caller's explicit next-step accepted-slot update.
    c.history.previous_penetration = c.history.staged_penetration;
    c.history.previous_stiffness = c.history.staged_stiffness;
    native_history.previous_penetration = native_history.staged_penetration;
    native_history.previous_stiffness = native_history.staged_stiffness;
    c.input.time += c.input.dt;
  }
}
TEST(Type25Normal, ZeroAdmissibleCoefficientsAndWeightsKeepSourceFloor) {
  auto c = Basic(); c.input.stiffness = 0; c.input.secondary_mass = 0;
  for (auto& mass : c.input.main_mass) mass = 0;
  normal::NativeNormalResult result;
  ASSERT_EQ(normal::EvaluateNativeNormal(c.config, c.input, c.history, &result), normal::NormalStatus::Ok);
  Same(result, Oracle(c.config, c.input, c.history));
  c.input.stiffness = 1e-31; c.input.secondary_mass = 1e-32;
  c.input.main_mass[0] = 1e-32; c.input.weights[0] = 1.;
  ASSERT_EQ(normal::EvaluateNativeNormal(c.config, c.input, c.history, &result), normal::NormalStatus::Ok);
  Same(result, Oracle(c.config, c.input, c.history));
}
TEST(Type25Normal, TypedEngineModesChangeStiffnessTermsWithoutInventingMassScaling) {
  auto c = Basic(); normal::NativeNormalResult combined, separate;
  ASSERT_EQ(normal::EvaluateNativeNormal(c.config, c.input, c.history, &combined), normal::NormalStatus::Ok);
  c.config.engine = {1, 0, 0};
  ASSERT_EQ(normal::EvaluateNativeNormal(c.config, c.input, c.history, &separate), normal::NormalStatus::Ok);
  EXPECT_FALSE(combined.terms_valid); EXPECT_TRUE(separate.terms_valid);
  EXPECT_EQ(separate.separate_elastic_stiffness, c.input.stiffness);
  EXPECT_EQ(combined.normal_force, separate.normal_force);
  EXPECT_GT(combined.stability_stiffness, separate.stability_stiffness);
  Same(separate, Oracle(c.config, c.input, c.history));
}
TEST(Type25Normal, InvalidAndOverflowPreserveCompleteOutputAndRetry) {
  const auto good = Basic(); const auto sentinel = Sentinel();
  for (unsigned mutation = 0; mutation < 8; ++mutation) {
    auto c = good; auto result = sentinel;
    switch (mutation) {
      case 0: c.config.engine.kdtint = -1; break;
      case 1: c.config.adhesion = true; break;
      case 2: c.config.arithmetic_precision = 4; break;
      case 3: c.input.penetration = -1; break;
      case 4: c.input.main_mass[3] = std::numeric_limits<double>::quiet_NaN(); break;
      case 5: c.history.damping_half_force = std::numeric_limits<double>::infinity(); break;
      case 6: c.input.stiffness = std::numeric_limits<double>::max(); c.input.penetration = 10.; break;
      case 7: c.input.normal_velocity = -std::numeric_limits<double>::max(); c.input.dt = 2.; break;
    }
    EXPECT_NE(normal::EvaluateNativeNormal(c.config, c.input, c.history, &result), normal::NormalStatus::Ok);
    Same(result, sentinel, true);
    ASSERT_EQ(normal::EvaluateNativeNormal(good.config, good.input, good.history, &result), normal::NormalStatus::Ok);
    Same(result, Oracle(good.config, good.input, good.history));
  }
  EXPECT_EQ(normal::EvaluateNativeNormal(good.config, good.input, good.history, nullptr), normal::NormalStatus::InvalidInput);
}
} // namespace type25_normal_test
