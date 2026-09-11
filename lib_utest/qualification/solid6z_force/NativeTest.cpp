// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "Compare.h"
#include <cstring>
#include <limits>
namespace solid6z_force_test {
TEST(Solid6zForceNative, CompleteCallerOwnsIndependent400StepRotatedHistoryAndBothWorkPhases) {
  const auto reference = Reference();
  const auto material = Material();
  auto accepted = Initial(reference,material);
  NativeHistory native;
  ASSERT_TRUE(native.Initialize(reference.input(),material));
  for (unsigned step = 0; step < 400; ++step) {
    auto interval = Path(reference,step);
    interval.base_time_s = accepted.stamp().time_s;
    const auto expected = native.Evaluate(interval);
    s::ForceTrial result;
    ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,{},result),s::Status::Success) << step;
    ASSERT_TRUE(Agree(Pack(result),expected)) << step;
    if (step == 199 || step == 399) {
      // The prescribed path returns to an unloaded rotated configuration.
      // The absolute constitutive roundoff bound must not hide a finite stress
      // or resultant error at precisely the cancellation points it admits.
      const auto values = Pack(result);
      double maximum_stress = 0, maximum_difference = 0;
      for (unsigned i = 0; i < 6; ++i) {
        maximum_stress = std::max(maximum_stress,std::abs(expected.material[i]));
        maximum_difference = std::max(maximum_difference,
            std::abs(values.material[i]-expected.material[i]));
      }
      EXPECT_LT(maximum_stress,1e-5);
      RecordProperty(step == 199 ? "half_cycle_stress_error_pa" : "full_cycle_stress_error_pa",
          ::testing::PrintToString(maximum_difference));
      for (unsigned channel : {0u,9u,15u}) {
        auto corrupt = expected;
        corrupt.material[channel] += .01;
        EXPECT_FALSE(Agree(values,corrupt)) << channel;
      }
      auto corrupt = expected;
      corrupt.history[0] += .01;
      EXPECT_FALSE(Agree(values,corrupt));
      for (unsigned channel : {0u,18u,36u}) {
        corrupt = expected;
        corrupt.forces[channel] += .001;
        EXPECT_FALSE(Agree(values,corrupt)) << channel;
      }
    }
    accepted = result.proposed_history;
    native.Accept(expected);
  }
}
TEST(Solid6zForceNative, AllTwelveSeededHourglassHistoriesCarryAcrossZeroVelocityHold) {
  const auto reference = Reference();
  const auto material = Material();
  auto values = Initial(reference,material).data();
  NativeHistory native;
  ASSERT_TRUE(native.Initialize(reference.input(),material));
  auto seed = native.accepted();
  for (unsigned k = 0; k < 3; ++k) {
    for (unsigned m = 0; m < 4; ++m) {
      const double value = (k+1)*1234.5-(m+1)*321.25;
      values.hourglass_stress_pa[k][m] = value;
      seed[9+4*k+m] = value;
    }
  }
  native.Prescribe(seed);
  s::History accepted;
  ASSERT_EQ(s::PreparePrescribedHistory(reference,material,{},values,{},accepted),s::Status::Success);
  s::PrescribedInterval interval;
  interval.dt_s = 1e-6;
  for (unsigned n = 0; n < 6; ++n) {
    const auto x = reference.input().position_m[n];
    interval.position_endpoint_m[n] = {1.05*x.x,x.y,.97*x.z};
  }
  const auto expected = native.Evaluate(interval);
  s::ForceTrial result;
  ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,{},result),s::Status::Success);
  ASSERT_TRUE(Agree(Pack(result),expected));
  for (unsigned k = 0; k < 3; ++k) {
    for (unsigned m = 0; m < 4; ++m)
      EXPECT_EQ(result.proposed_history.data().hourglass_stress_pa[k][m],values.hourglass_stress_pa[k][m]);
  }
}
TEST(Solid6zForceNative, RepairedDampingConsumesMaterialSoundSpeedAndExplicitZeroDn) {
  const auto reference = Reference();
  const auto material = Material();
  const auto interval = Path(reference,0);
  NativeHistory native;
  ASSERT_TRUE(native.Initialize(reference.input(),material));
  const auto once = native.Evaluate(interval,.1,1);
  const auto twice = native.Evaluate(interval,.1,2);
  const auto zero = native.Evaluate(interval,0,1);
  ASSERT_EQ(once.status,0);
  ASSERT_EQ(twice.status,0);
  ASSERT_EQ(zero.status,0);
  EXPECT_DOUBLE_EQ(twice.stabilization[25],2*once.stabilization[25]);
  EXPECT_EQ(zero.stabilization[25],0);
  double effect = 0;
  for (unsigned i = 18; i < 36; ++i) effect += std::abs(twice.forces[i]-once.forces[i]);
  EXPECT_GT(effect,1e-5);
  for (double dn : {.1,0.0}) {
    s::ForceProfile profile;
    profile.damping_coefficient = dn;
    const auto accepted = Initial(reference,material,profile);
    s::ForceTrial result;
    ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,profile,result),s::Status::Success);
    ASSERT_TRUE(Agree(Pack(result),native.Evaluate(interval,dn,1)));
  }
  // Direct shared-force control alters SSP alone, without another material update.
  auto accepted = Initial(reference,material);
  s::ForceTrial normal;
  ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,{},normal),s::Status::Success);
  auto point = normal.material;
  point.point.sound_speed_m_s *= 2;
  auto state = accepted.data();
  state.material = normal.material.history;
  s::Vec3 forces[6];
  std::copy(std::begin(normal.material_local_force_n),std::end(normal.material_local_force_n),forces);
  s::HourglassObservation observation;
  ASSERT_EQ(s::force_detail::Stabilize(reference,material,{},normal.geometry,interval.dt_s,
      point,state,forces,observation),s::Status::Success);
  EXPECT_DOUBLE_EQ(observation.damping_kg_m_s,2*normal.stabilization.damping_kg_m_s);
  double packed[18];
  for (unsigned n = 0; n < 6; ++n) {
    packed[3*n] = forces[n].x; packed[3*n+1] = forces[n].y; packed[3*n+2] = forces[n].z;
  }
  EXPECT_TRUE(Group("twice SSP force",packed,twice.forces.data()+18,0,18));
}
TEST(Solid6zForceNative, RejectedLateHistoryPreservesOutputsAndRetryMatchesOwnNativeState) {
  const auto reference = Reference();
  const auto material = Material();
  const auto accepted = Initial(reference,material);
  const auto interval = Path(reference,0);
  NativeHistory native;
  ASSERT_TRUE(native.Initialize(reference.input(),material));
  const auto expected = native.Evaluate(interval);
  s::ForceTrial output;
  ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,{},output),s::Status::Success);
  const auto clean = Pack(output);
  unsigned char bytes[sizeof(output)];
  std::memcpy(bytes,&output,sizeof(output));
  auto values = accepted.data();
  values.material.internal_energy_density_j_m3 = std::numeric_limits<double>::max();
  values.hourglass_stress_pa[2][3] = std::copysign(std::numeric_limits<double>::max(),
      output.stabilization.modal_velocity_m_s[2][3]);
  s::History bad;
  ASSERT_EQ(s::PreparePrescribedHistory(reference,material,{},values,{},bad),s::Status::Success);
  EXPECT_NE(s::EvaluateForce(reference,bad,interval,material,{},output),s::Status::Success);
  EXPECT_EQ(std::memcmp(bytes,&output,sizeof(output)),0);
  ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,{},output),s::Status::Success);
  EXPECT_TRUE(Agree(Pack(output),expected));
  auto corrupt = expected;
  corrupt.history[20] += 1;
  EXPECT_FALSE(Agree(clean,corrupt));
  corrupt = expected;
  corrupt.stabilization[26] *= 1.001;
  EXPECT_FALSE(Agree(clean,corrupt));
  corrupt = expected;
  corrupt.forces[53] += .01;
  EXPECT_FALSE(Agree(clean,corrupt));
}
} // namespace solid6z_force_test
