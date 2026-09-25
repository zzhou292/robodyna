// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "Assertions.h"
namespace type25_friction_test {
namespace {
n::NormalStatus Evaluate(const Case& c, n::NativeFrictionResult* out) {
  return n::EvaluateNativeFriction(c.normal_config, c.controls, c.coefficients, c.input, c.history, out);
}
n::Vector Rotate(n::Vector v) { return {-v.y, v.x, v.z}; }
}
TEST(Type25Friction, CompleteNormalTangentialOutputsMatchBothNativeHistoryLayouts) {
  for (const auto& c : Cases()) {
    n::NativeFrictionResult actual;
    ASSERT_EQ(Evaluate(c, &actual), n::NormalStatus::Ok);
    Same(actual, Oracle(c)); Same(actual, Oracle(c, true));
  }
}
TEST(Type25Friction, StickingAndSlidingUseRawStiffnessAndNativeResultantSign) {
  for (double speed : {5., 2000.}) {
    auto c = Basic(); c.input.relative_velocity.x = speed; RefreshNormalVelocity(c);
    n::NativeFrictionResult actual;
    ASSERT_EQ(Evaluate(c, &actual), n::NormalStatus::Ok);
    Same(actual, Oracle(c));
    EXPECT_DOUBLE_EQ(actual.tangent_predictor.x, c.input.normal.stiffness * speed * c.input.dt12);
    if (speed == 5) EXPECT_EQ(actual.limiter, 1.); else EXPECT_LT(actual.limiter, 1.);
    EXPECT_EQ(actual.tangent_force.z, 0.);
    EXPECT_EQ(actual.native_resultant.z, actual.normal.normal_force);
    EXPECT_EQ(actual.native_resultant.x, actual.tangent_force.x);
  }
}
TEST(Type25Friction, ReversalCanReturnStoredTangentialWorkWithoutClamping) {
  auto c = Basic(); c.input.relative_velocity = {1, 0, -20};
  c.history.previous_force = {-0.1, 0, 0}; RefreshNormalVelocity(c);
  n::NativeFrictionResult actual;
  ASSERT_EQ(Evaluate(c, &actual), n::NormalStatus::Ok);
  Same(actual, Oracle(c)); EXPECT_LT(actual.friction_work, 0.);
  EXPECT_LT(actual.tangent_force.x, 0.);
}
TEST(Type25Friction, CurrentNormalProjectionAndCoordinateRotationMatchNative) {
  auto c = Basic(); c.input.relative_velocity = {0, 0, 0};
  c.history.previous_force = {0.1, 0, 0}; c.input.normal_axis = {0.6, 0, 0.8};
  for (auto& v : c.input.main_vertices) v = {0.8*v.x + 0.6*v.z, v.y, -0.6*v.x + 0.8*v.z};
  RefreshNormalVelocity(c);
  n::NativeFrictionResult first, rotated;
  ASSERT_EQ(Evaluate(c, &first), n::NormalStatus::Ok); Same(first, Oracle(c));
  EXPECT_NEAR(first.tangent_predictor.x, 0.064, 1e-16);
  EXPECT_NEAR(first.tangent_predictor.z, -0.048, 1e-16);
  // A coordinate change rotates all vectors; it is not a new history corotation law.
  c.input.normal_axis = Rotate(c.input.normal_axis); c.input.relative_velocity = Rotate(c.input.relative_velocity);
  c.history.previous_force = Rotate(c.history.previous_force); c.history.staged_force = Rotate(c.history.staged_force);
  for (auto& v : c.input.main_vertices) v = Rotate(v);
  RefreshNormalVelocity(c);
  ASSERT_EQ(Evaluate(c, &rotated), n::NormalStatus::Ok); Same(rotated, Oracle(c));
  const auto expected = Rotate(first.tangent_force);
  EXPECT_DOUBLE_EQ(rotated.tangent_force.x, expected.x);
  EXPECT_DOUBLE_EQ(rotated.tangent_force.y, expected.y);
  EXPECT_DOUBLE_EQ(rotated.tangent_force.z, expected.z);
}
TEST(Type25Friction, CurrentQuadAndTriangleAreaPressureRemainNativeInputs) {
  for (bool triangle : {false, true}) {
    auto c = Basic();
    if (triangle) c.input.main_vertices[3] = c.input.main_vertices[2];
    c.coefficients = {0.1, {0.02, -0.001, 0.03, -0.002, 0.1, -0.001}};
    n::NativeFrictionResult actual;
    ASSERT_EQ(Evaluate(c, &actual), n::NormalStatus::Ok); Same(actual, Oracle(c));
    EXPECT_EQ(actual.contact_area, triangle ? 50. : 100.);
    EXPECT_DOUBLE_EQ(actual.pressure, -actual.normal.normal_force / actual.contact_area);
  }
}
TEST(Type25Friction, ZeroContactSkipsGeometryAndPreservesCallerHistory) {
  auto c = Basic(); c.input.normal.penetration = 0;
  c.input.normal_axis.x = std::numeric_limits<double>::quiet_NaN();
  c.input.relative_velocity.y = std::numeric_limits<double>::quiet_NaN();
  c.input.main_vertices[0].x = std::numeric_limits<double>::quiet_NaN();
  c.history.previous_force = {1, 2, 3}; c.history.staged_force = {4, 5, 6};
  n::NativeFrictionResult actual;
  ASSERT_EQ(Evaluate(c, &actual), n::NormalStatus::Ok); Same(actual, Oracle(c));
  EXPECT_FALSE(actual.contact_active);
  EXPECT_EQ(actual.history.staged_force.x, 4.);
  EXPECT_EQ(actual.native_resultant.x, 0.); // API-defined inactive channel, not native scratch evidence.
}
TEST(Type25Friction, AliasedPriorHistoryAndFailureRetryPublishAtomically) {
  const auto c = Basic(); n::NativeFrictionResult result;
  ASSERT_EQ(Evaluate(c, &result), n::NormalStatus::Ok);
  auto next = c; next.history = result.history;
  const auto expected = Oracle(next);
  ASSERT_EQ(n::EvaluateNativeFriction(c.normal_config, c.controls, c.coefficients, c.input,
      result.history, &result), n::NormalStatus::Ok);
  Same(result, expected);
  for (unsigned mutation = 0; mutation < 8; ++mutation) {
    auto bad = c; auto actual = Sentinel<n::NativeUnitsTag>(); const auto before = actual;
    switch (mutation) {
      case 0: bad.controls.formulation = 0; break;
      case 1: bad.controls.converged = 0; break;
      case 2: bad.input.normal.normal_velocity += 1; break;
      case 3: bad.input.normal_axis = {}; break;
      case 4: for (auto& v : bad.input.main_vertices) v = {}; break;
      case 5: bad.coefficients.c[5] = std::numeric_limits<double>::max(); break;
      case 6: bad.input.normal.friction_viscosity = 0.1; break;
      case 7: bad.input.dt12 = -1; break;
    }
    EXPECT_NE(Evaluate(bad, &actual), n::NormalStatus::Ok); Same(actual, before, true);
    ASSERT_EQ(Evaluate(c, &actual), n::NormalStatus::Ok); Same(actual, Oracle(c));
  }
  EXPECT_EQ(Evaluate(c, nullptr), n::NormalStatus::InvalidInput);
}
TEST(Type25Friction, ActualSourceInputRetainsSequentialVelocityInterpolation) {
  const auto c = ActualForceInput();
  EXPECT_GT(c.input.normal.penetration, 0.);
  EXPECT_LT(c.input.normal.penetration, n::native_constant::epp);
  EXPECT_EQ(c.input.normal.stiffness, 164000.);
  EXPECT_NE(c.input.normal.dt, c.input.dt12);
  n::NativeFrictionResult actual;
  ASSERT_EQ(Evaluate(c, &actual), n::NormalStatus::Ok);
  Same(actual, Oracle(c)); Same(actual, Oracle(c, true));
  EXPECT_TRUE(actual.contact_active);
}
TEST(Type25Friction, ZeroNormalPreservesNativeArithmeticWithoutClaimingGeometryAdmission) {
  auto c = Basic(); c.input.normal_axis = {}; RefreshNormalVelocity(c);
  n::NativeFrictionResult result;
  ASSERT_EQ(Evaluate(c, &result), n::NormalStatus::Ok); Same(result, Oracle(c));
  EXPECT_EQ(result.native_resultant.x, 0.); EXPECT_EQ(result.native_resultant.y, 0.);
  EXPECT_EQ(result.native_resultant.z, 0.); EXPECT_EQ(result.limiter, 0.);
  EXPECT_GT(result.normal.elastic_energy, 0.);
  // A future geometry producer must decide if this numerical packet is physical.
}
} // namespace type25_friction_test
