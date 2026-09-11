// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"

namespace rear_startup_test {
TEST(Rear18Startup, PointConstructorHasVirginStateAndKeepsOrdinaryZeroStepClosed) {
  namespace p = law::point;
  const auto material = Material();
  p::Input input;
  input.engineering_rate_per_s[0] = 13;
  input.engineering_rate_per_s[3] = -8;
  input.relative_density = 1e-4;
  p::Result result;
  ASSERT_EQ(p::Initialize(material,input,result),p::Status::Ok);
  for (unsigned k = 0; k < 3; ++k)
    EXPECT_DOUBLE_EQ(result.history.stress_pa[k],-material.bulk_pa*input.relative_density);
  for (unsigned k = 3; k < 6; ++k) EXPECT_EQ(result.history.stress_pa[k],0);
  for (double strain : result.history.engineering_strain) EXPECT_EQ(strain,0);
  EXPECT_EQ(result.history.plastic_strain,0);
  EXPECT_EQ(result.history.filtered_rate_per_s,0);
  EXPECT_EQ(result.history.curve_cursor,0u);
  EXPECT_EQ(result.tangent_factor,1);
  EXPECT_EQ(result.sound_speed_m_s,material.sound_speed_m_s);
  const auto bytes = Bytes(result);
  EXPECT_EQ(p::Update(material,{},input,result),p::Status::InvalidInput);
  EXPECT_EQ(Bytes(result),bytes);
  input.dt_s = 1e-5;
  EXPECT_EQ(p::Initialize(material,input,result),p::Status::InvalidInput);
  EXPECT_EQ(Bytes(result),bytes);
  input.dt_s = 0;
  input.engineering_rate_per_s[5] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(p::Initialize(material,input,result),p::Status::InvalidInput);
  EXPECT_EQ(Bytes(result),bytes);
}
TEST(Rear18Startup, BothTopologiesConstructSampleZeroAndRejectBadInputsAtomically) {
  const auto material = Material();
  for (bool collapsed : {false,true}) {
    const auto reference = Reference(collapsed);
    law::ForceTrial result;
    ASSERT_EQ(law::InitializeForce(reference,material,{11.123,-.37,.129},result),s::Status::Success);
    EXPECT_EQ(result.proposed_history.stamp().sample_index,0u);
    EXPECT_EQ(result.proposed_history.stamp().time_s,0);
    EXPECT_EQ(result.diagnostics.caller_degeneracy,collapsed ? 12u : 0u);
    EXPECT_GT(result.diagnostics.minimum_unscaled_dt_s,0);
    EXPECT_GT(result.diagnostics.raw_stiffness_n_m,0);
    for (unsigned ip = 0; ip < 8; ++ip) {
      const auto& h = result.proposed_history.data().point[ip];
      EXPECT_EQ(h.material.filtered_rate_per_s,0);
      EXPECT_EQ(h.material.plastic_strain,0);
      EXPECT_EQ(h.material.curve_cursor,0u);
      EXPECT_EQ(h.initial_volume_m3,reference.geometry().point[ip].initial_volume_m3);
      for (double x : h.material.engineering_strain) EXPECT_EQ(x,0);
    }
    const auto bytes = Bytes(result);
    EXPECT_EQ(law::InitializeForce(reference,material,{0,0,INFINITY},result),s::Status::InvalidInput);
    EXPECT_EQ(Bytes(result),bytes);
    auto bad_material = material;
    bad_material.material.density_kg_m3 *= 2;
    EXPECT_EQ(law::InitializeForce(reference,bad_material,{},result),s::Status::InvalidInput);
    EXPECT_EQ(Bytes(result),bytes);
    auto interval = Step(reference,result.proposed_history);
    interval.dt_s = 0;
    EXPECT_EQ(law::EvaluateForce(reference,result.proposed_history,interval,material,result),s::Status::InvalidInput);
    EXPECT_EQ(Bytes(result),bytes);
    interval = Step(reference,result.proposed_history);
    ASSERT_EQ(law::EvaluateForce(reference,result.proposed_history,interval,material,result),s::Status::Success);
    EXPECT_EQ(result.proposed_history.stamp().sample_index,1u);
    ASSERT_EQ(law::InitializeForce(reference,material,{},result),s::Status::Success);
    EXPECT_EQ(result.proposed_history.stamp().sample_index,0u);
  }
}
TEST(Rear18Startup, ScratchReusesEveryFieldAcrossTopologiesAndFourHundredIntervals) {
  const auto material = Material();
  auto scratch = std::make_unique<law::detail::ForceScratch>();
  for (bool collapsed : {false,true,false}) {
    const auto reference = Reference(collapsed);
    DirtyExcluded(*scratch);
    law::ForceTrial expected;
    ASSERT_EQ(law::InitializeForce(reference,material,{11.123,-.37,.129},expected),s::Status::Success);
    ASSERT_EQ(law::detail::InitializeForceScratch(reference,material,{11.123,-.37,.129},*scratch),s::Status::Success);
    Exact(scratch->trial,expected);
    auto accepted = scratch->trial.proposed_history;
    for (unsigned step = 0; step < 400; ++step) {
      SCOPED_TRACE(step);
      auto interval = Path(reference,step,true);
      interval.base_time_s = accepted.stamp().time_s;
      DirtyExcluded(*scratch);
      ASSERT_EQ(law::EvaluateForce(reference,accepted,interval,material,expected),s::Status::Success);
      ASSERT_EQ(law::detail::EvaluateForceScratch(reference,accepted,interval,material,*scratch),s::Status::Success);
      Exact(scratch->trial,expected);
      accepted = scratch->trial.proposed_history;
    }
  }
}
TEST(Rear18Startup, ScratchLateFailureLeavesAcceptedAndOtherWorkerUntouchedThenRetries) {
  const auto reference = Reference(true);
  const auto material = Material();
  auto scratch = std::make_unique<law::detail::ForceScratch>();
  auto other = std::make_unique<law::detail::ForceScratch>();
  ASSERT_EQ(law::detail::InitializeForceScratch(reference,material,{},*scratch),s::Status::Success);
  ASSERT_EQ(law::detail::InitializeForceScratch(reference,material,{1,2,3},*other),s::Status::Success);
  const auto other_bytes = Bytes(*other);
  const auto accepted = scratch->trial.proposed_history;
  const auto accepted_bytes = Bytes(accepted);
  const auto interval = Step(reference,accepted);
  auto values = accepted.data();
  values.point[7].material.stress_pa[5] = 1e200;
  law::History bad;
  ASSERT_EQ(law::PreparePrescribedHistory(reference,material,values,accepted.stamp(),bad),s::Status::Success);
  EXPECT_EQ(law::detail::EvaluateForceScratch(reference,bad,interval,material,*scratch),s::Status::NonfiniteResult);
  EXPECT_EQ(Bytes(accepted),accepted_bytes);
  EXPECT_EQ(Bytes(*other),other_bytes);
  law::ForceTrial expected;
  ASSERT_EQ(law::EvaluateForce(reference,accepted,interval,material,expected),s::Status::Success);
  ASSERT_EQ(law::detail::EvaluateForceScratch(reference,accepted,interval,material,*scratch),s::Status::Success);
  Exact(scratch->trial,expected);
}
}  // namespace rear_startup_test
