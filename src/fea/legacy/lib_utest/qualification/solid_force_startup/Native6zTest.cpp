// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../solid6z_force/TestSupport.h"
#include "../solid6z_force/Compare.h"
#include "../solid6z_reference/SourceFixture.h"
#include "lib_src/elements/solids/ForceStiffness.h"

namespace solid6z_force_test {
void CheckStartup(const s::Reference& reference,s::Vec3 velocity,bool recurrence) {
  const auto material=Material(reference.input().density_kg_m3);
  NativeHistory native;
  ASSERT_TRUE(native.Initialize(reference.input(),material));
  s::ForceTrial actual;
  ASSERT_EQ(s::InitializeForce(reference,material,{},velocity,actual),s::Status::Success);
  const auto expected=native.InitializeForce(velocity);
  ASSERT_TRUE(Agree(Pack(actual),expected));
  ASSERT_EQ(actual.proposed_history.stamp().sample_index,0u);
  ASSERT_EQ(actual.proposed_history.stamp().time_s,0);
  tl::fea::solids::NodalStiffness stiffness;
  ASSERT_TRUE(tl::fea::solids::PrepareNodalStiffness(actual,stiffness));
  EXPECT_NEAR(stiffness.translation_n_m,(1.0/3.0)*expected.material[32],
              3e-10*stiffness.translation_n_m);
  native.Accept(expected);
  auto accepted=actual.proposed_history;
  for (unsigned step=0;recurrence && step<4;++step) {
    auto interval=Path(reference,step);
    interval.base_time_s=accepted.stamp().time_s;
    const auto next=native.Evaluate(interval);
    ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,{},actual),s::Status::Success);
    ASSERT_TRUE(Agree(Pack(actual),next));
    accepted=actual.proposed_history;
    native.Accept(next);
  }
}
TEST(SolidStartupNative, WedgeIndependentTT0ThenPositiveIntervals) {
  for (const s::Vec3 velocity : {s::Vec3{},s::Vec3{11.123,-.37,.129}})
    CheckStartup(Reference(),velocity,true);
}
TEST(SolidStartupNative, WedgeConstructorKeepsOrdinaryZeroStepClosed) {
  const auto reference=Reference();
  NativeHistory native;
  ASSERT_TRUE(native.Initialize(reference.input(),Material()));
  s::PrescribedInterval zero;
  for (unsigned n=0;n<6;++n) zero.position_endpoint_m[n]=reference.input().position_m[n];
  EXPECT_NE(native.Evaluate(zero).status,0);
  const auto initial=native.InitializeForce({});
  ASSERT_EQ(initial.status,0);
  EXPECT_NE(native.InitializeForce({0,0,std::numeric_limits<double>::quiet_NaN()}).status,0);
  EXPECT_EQ(native.accepted()[7],0); // Constructing a packet does not advance this holder.
}
TEST(SolidStartupSource, All195OriginalMappedWedgesInitialFieldsAndStiffness) {
  unsigned count=0;
  for (unsigned row=0;row<solid6z_test::SourceCount;++row) {
    s::ReferenceInput input;
    if (!solid6z_test::Source(row,input)) continue;
    SCOPED_TRACE(input.source_element_id);
    CheckStartup(Reference(input),{11.123,0,0},false);
    ASSERT_FALSE(HasFatalFailure());
    ++count;
  }
  EXPECT_EQ(count,195u);
}
} // namespace solid6z_force_test
