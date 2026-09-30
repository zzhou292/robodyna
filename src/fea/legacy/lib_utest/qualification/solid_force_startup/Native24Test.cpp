// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../solid24_force/NativeOracle.h"
#include "../solid24_force/PrescribedPath.h"
#include "../solid24_reference/SourceFixture.h"
#include "lib_src/elements/solids/ForceStiffness.h"

namespace heph_test {
void CheckStartup(const s::Reference& reference,s::Vec3 velocity,bool recurrence) {
  const auto material=Material(reference.input().density_kg_m3);
  auto native=InitializeNative(reference.input());
  s::PrescribedInterval initial;
  for (unsigned n=0;n<8;++n) {
    initial.position_m[n]=reference.input().position_m[n];
    initial.velocity_m_s[n]=velocity;
  }
  s::ForceTrial actual;
  ASSERT_EQ(s::InitializeForce(reference,material,velocity,actual),s::ForceStatus::Success);
  const auto expected=NativeStep(native,initial,material);
  Compare(actual,expected);
  ASSERT_EQ(actual.proposed_history.stamp().sample_index,0u);
  ASSERT_EQ(actual.proposed_history.stamp().time_s,0);
  tl::fea::solids::NodalStiffness stiffness;
  ASSERT_TRUE(tl::fea::solids::PrepareNodalStiffness(actual,stiffness));
  EXPECT_NEAR(stiffness.translation_n_m,.25*expected.values[183],
              3e-10*stiffness.translation_n_m);
  AcceptNative(expected,native);
  auto accepted=actual.proposed_history;
  for (unsigned step=1;recurrence && step<=4;++step) {
    auto interval=Path(reference,step,1e-8);
    interval.base_time_s=accepted.stamp().time_s;
    const auto next=NativeStep(native,interval,material);
    ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,actual),s::ForceStatus::Success);
    Compare(actual,next);
    accepted=actual.proposed_history;
    AcceptNative(next,native);
  }
}
TEST(SolidStartupNative, HephIndependentTT0ThenPositiveIntervals) {
  for (const s::Vec3 velocity : {s::Vec3{},s::Vec3{11.123,-.37,.129}})
    CheckStartup(Reference(),velocity,true);
}
TEST(SolidStartupSource, All1309OriginalMappedBricksInitialFieldsAndStiffness) {
  unsigned count=0;
  for (unsigned row=0;row<solid24_test::SourceCount;++row) {
    const auto input=solid24_test::Source(row);
    if (!solid24_test::IsBrick(input)) continue;
    SCOPED_TRACE(input.source_element_id);
    CheckStartup(Reference(solid24_test::TotalReference(input,s::WorkingLengthUnit::Millimetre)),
                  {11.123,0,0},false);
    ASSERT_FALSE(HasFatalFailure());
    ++count;
  }
  EXPECT_EQ(count,1309u);
}
} // namespace heph_test
